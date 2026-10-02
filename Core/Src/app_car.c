/**
  ******************************************************************************
  * @file    app_car.c
  * @brief   循迹控制实现
  *
  * 【一拍的完整数据流】
  *
  *   ① 采样       BSP_Track_GetMask()            八路压线掩码
  *                 BSP_Encoder_GetDelta/RPM()    两轮转速与里程
  *                        │
  *   ② 算误差     BSP_Track_GetError() / 7       归一化到 [-1, +1]
  *                 （丢线时该函数自动沿用上一次的有效误差）
  *                        │
  *   ③ 路况修正   十字（八路全黑）→ 误差强制 0，按时长保持直行
  *                 丢线（八路全白）→ 计时，超 60ms 判定丢线
  *                        │
  *   ④ 转向环     BSP_PID_UpdateErr()            得到 turn
  *                 丢线时不用 PID，直接按最后方向给固定转向量
  *                        │
  *   ⑤ 速度规划   基础速度 × 起步斜坡 × 丢线减速
  *                        │
  *   ⑥ 差速合成   左 = 基础 + turn     右 = 基础 - turn
  *                 内侧轮抬到 MINSPD（防止弯道内轮停转造成顿挫）
  *                        │
  *   ⑦ 输出       BSP_Motor_SetSpeed(左, 右)
  *                        │
  *   ⑧ 保护       丢线超时 → 刹车；给电不转 → 刹车。两者都进报警状态
  *
  * 【★ 为什么把 dt 交给 PID，而不是用固定的 0.01 秒】
  *   正常时主循环确实是 10ms 一拍，但 OLED 整屏刷一次要几十毫秒，
  *   那几拍就会被拉长。如果 PID 里写死 dt=0.01，一次 30ms 的卡顿会让
  *   微分项算出 3 倍大的值 —— 车会毫无理由地猛抖一下。
  *   所以这里把"这一拍实际过了多久"传进去（CTRL_DT_S 只是名义值，
  *   正常情况下就等于它），由 PID 自己按实际时间缩放。
  *
  *   注：本工程的主循环是"到点就跑、错过不补"，所以 dt 也可以用常数；
  *   真正需要实测 dt 的是编码器测速（见 bsp_encoder.c），那里必须实测，
  *   因为它算的是"增量 ÷ 时间"。
  ******************************************************************************
  */
#include "app_car.h"
#include "app_param.h"

#include "bsp_track.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_buzzer.h"
#include "bsp_pid.h"
#include "bsp_uart.h"

/* ==========================================================================
 * 内部常量
 * ========================================================================== */

/* 八路全白持续多久才判定为"真的丢线"。
 * 为什么不立刻判：探头经过黑线的缺口、赛道上的反光点、十字路口的白缝，
 * 都会让八路瞬间全白。给 60ms 缓冲，这些瞬时情况就被自然忽略了。
 * 真丢线的场景下 60ms 也才走几厘米，来得及救。 */
#define CAR_LOST_DEBOUNCE_MS        60u

/* 丢线时基础速度打几折。留时间给转向，也降低冲出赛道的概率 */
#define CAR_LOST_SPEED_SCALE        0.6f

/* 大偏差减速的下限系数：不管偏多少，至少保留这么多速度。
   理由见 APP_Car_Update() "速度规划"一节 —— 停住的车是没法转向的 */
#define CAR_SLOW_MIN_SCALE          0.25f

/* 报警蜂鸣：间隔、单次时长、最多响几声。
 *
 * ★ 为什么必须限制次数（"最多几声"这一条是后加的，之前是个 bug）：
 *   原来只要停在 LOST / STALL 状态就每 700ms 响一声、永远不停，
 *   车一停就"嘀嘀嘀"没完没了，在教室里尤其难受。
 *   报警的目的只是"告诉你它停了"，OLED 上已经写着 LOST/STALL，
 *   响个三四声足够引起注意，之后必须自己闭嘴。 */
#define CAR_ALERT_PERIOD_MS         700u
#define CAR_ALERT_BEEP_MS           90u
#define CAR_ALERT_MAX_BEEPS         4u

/* 启动提示音 */
#define CAR_START_BEEP_MS           80u

/* 积分项限幅取输出限幅的 60%：留出余量给 P 项，
 * 否则积分的"存量"会一直霸占着输出，P 项变得毫无作用 */
#define CAR_I_LIMIT_RATIO           0.6f

/* 控制周期的合法上限（毫秒）。
 * 主循环是"到点就跑"，名义上 10ms 一拍，但 OLED 刷屏、串口打印都会
 * 把某一拍拉长到 15~50ms。用实测 dt 喂 PID，控制量就不会因为
 * "这一拍特别长"而突变。超过 50ms 的间隔已经不算是控制周期了，
 * 按 50ms 处理，避免一次 3 秒的阻塞（串口自检）把积分项一次灌满 */
#define CAR_DT_MAX_MS               50u

/* ==========================================================================
 * 内部状态
 * ========================================================================== */

static BSP_PID_t       s_pid_turn;          /* 转向环 */
static APP_CarStatus_t s_st;                /* 对外的状态快照 */

static int8_t   s_last_dir    = 1;          /* 最后一次看到线时误差的正负（1 = 线在右侧） */
static uint16_t s_lost_acc    = 0u;         /* 全白累计毫秒 */
static uint16_t s_lost_cnt    = 0u;         /* ★ "全丢黑线"的次数（丢线事件计数） */
static uint8_t  s_lost_armed  = 1u;         /* 1 = 允许计数；看到线够久之后重新置 1 */
static uint16_t s_seen_acc    = 0u;         /* 重新武装用：连续"看到线"的毫秒数 */
static uint16_t s_cross_acc   = 0u;         /* 全黑累计毫秒 */
static uint16_t s_stall_acc   = 0u;         /* 堵转累计毫秒 */
static uint16_t s_alert_acc   = 0u;         /* 报警蜂鸣节流 */
static uint32_t s_run_ms      = 0u;         /* 本次启动已运行毫秒 */
static uint8_t  s_pid_stale   = 0u;         /* 1 = 丢线期间没跑 PID，重新看到线时要复位一次 */
static uint32_t s_last_tick   = 0u;         /* 上一拍的时刻，用来量出真实 dt */
static uint8_t  s_ramping     = 0u;         /* 1 = 还在起步斜坡里，见"速度规划"一节 */
static uint8_t  s_alert_cnt   = 0u;         /* 报警已经响过几声（响够就不再响） */


/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/** 把 g_param 里的转向环参数同步给 PID 实例。
 *  ★ 每个控制周期都调一次，所以菜单里一改参数下一拍就生效。
 *    BSP_PID_SetGains 不会清积分，所以在行驶中调参车不会顿 */
static void car_pid_sync(void)
{
    BSP_PID_SetGains(&s_pid_turn, g_param.turn_kp, g_param.turn_ki, g_param.turn_kd);
    BSP_PID_SetLimits(&s_pid_turn,
                      (float)g_param.turn_limit * CAR_I_LIMIT_RATIO,
                      (float)g_param.turn_limit);
    BSP_PID_SetDeadZone(&s_pid_turn, g_param.dead_zone);
    BSP_PID_SetDFilter(&s_pid_turn, PID_D_FILTER_ALPHA);
}

/** 带符号速度限幅到 ±MOTOR_PWM_MAX。
 *  必须先夹再算，理由和 bsp_motor 里一样：防止取负时整数溢出 */
static int16_t car_clamp_speed(float v)
{
    if (v > (float)MOTOR_PWM_MAX) {
        return (int16_t)MOTOR_PWM_MAX;
    }
    if (v < -(float)MOTOR_PWM_MAX) {
        return -(int16_t)MOTOR_PWM_MAX;
    }
    return (int16_t)v;
}

/** 停车并把状态切过去，同时立刻刹一脚。
 *  丢线超时、堵转都走这里 */
static void car_halt(uint8_t new_state)
{
    BSP_Motor_Brake();
    s_st.state  = new_state;
    s_st.spd_l  = 0;
    s_st.spd_r  = 0;
    s_st.turn   = 0.0f;
    s_st.base_now = 0;
    s_alert_acc = CAR_ALERT_PERIOD_MS;  /* 让下一次 Update 立刻响第一声 */
    s_alert_cnt = 0u;                   /* 报警计数重新开始（这次是新的故障） */
}

/** 报警状态下的间歇蜂鸣。非阻塞 —— 绝不能用 Beep()，
 *  那会在控制循环里阻塞上百毫秒，把节拍彻底打乱 */
static void car_alert_poll(uint32_t dt_ms)
{
    /* 响够 CAR_ALERT_MAX_BEEPS 声就彻底闭嘴，不再打扰。
       状态本身在 OLED 上一直显示着，不怕人看不见 */
    if (s_alert_cnt >= CAR_ALERT_MAX_BEEPS) {
        return;
    }

    s_alert_acc = (uint16_t)(s_alert_acc + dt_ms);
    if (s_alert_acc >= CAR_ALERT_PERIOD_MS) {
        s_alert_acc = 0u;
        s_alert_cnt++;
        BSP_Buzzer_BeepAsync(CAR_ALERT_BEEP_MS);
    }
}

/** 刷新转速/里程这类"看一眼"的遥测量。
 *  待机时也要刷新，不然菜单里停着车看不到数值 */
static void car_refresh_telemetry(void)
{
    s_st.rpm_l  = BSP_Encoder_GetRPM(ENC_LEFT);
    s_st.rpm_r  = BSP_Encoder_GetRPM(ENC_RIGHT);
    s_st.dist_l = BSP_Encoder_GetTotal(ENC_LEFT);
    s_st.dist_r = BSP_Encoder_GetTotal(ENC_RIGHT);
}

/* ==========================================================================
 * 初始化与启停
 * ========================================================================== */

void APP_Car_Init(void)
{
    /* 初始增益取参数里的值；限幅按执行器量程给。
       即使一直不调 car_pid_sync()，PID 也是可用的合法状态 */
    BSP_PID_Init(&s_pid_turn, g_param.turn_kp, g_param.turn_ki, g_param.turn_kd);
    car_pid_sync();

    /* 状态快照清零 */
    s_st.state     = APP_CAR_IDLE;
    s_st.mask      = 0u;
    s_st.lost      = 0u;
    s_st.crossing  = 0u;
    s_st.err       = 0.0f;
    s_st.turn      = 0.0f;
    s_st.base_now  = 0;
    s_st.spd_l     = 0;
    s_st.spd_r     = 0;
    s_st.lost_ms   = 0u;
    s_st.run_ms    = 0u;
    s_st.dist_l    = 0;
    s_st.dist_r    = 0;
    s_st.rpm_l     = 0.0f;
    s_st.rpm_r     = 0.0f;

    s_run_ms = 0u;
    BSP_Motor_Stop();
}


void APP_Car_Start(void)
{
    /* 清零所有计时与积分：上一次跑车留下的积分存量如果带到这一次，
       起步瞬间就会有一股莫名其妙的转向 */
    s_run_ms     = 0u;
    s_lost_acc   = 0u;
    s_cross_acc  = 0u;
    s_stall_acc  = 0u;
    s_alert_acc  = CAR_ALERT_PERIOD_MS;
    s_alert_cnt  = 0u;
    s_ramping    = 1u;
    s_last_dir   = 1;
    s_pid_stale  = 0u;
    s_last_tick  = HAL_GetTick();   /* dt 基准重新起算，避免把启动前的空档算进来 */

    car_pid_sync();
    BSP_PID_Reset(&s_pid_turn);

    BSP_Encoder_Reset();        /* 里程从 0 起算 */

    s_st.state    = APP_CAR_RUN;
    s_st.run_ms   = 0u;
    s_st.lost     = 0u;
    s_st.crossing = 0u;
    s_st.lost_ms  = 0u;
    s_st.dist_l   = 0;
    s_st.dist_r   = 0;

    /* 先确保电机是停的，再交给控制律。这样即使从 LOST 状态重启，
       也不会带着上一次的残余占空比启动 */
    BSP_Motor_Stop();

    BSP_Buzzer_BeepAsync(CAR_START_BEEP_MS);
}

void APP_Car_Stop(void)
{
    BSP_PID_Reset(&s_pid_turn);
    BSP_Motor_Stop();           /* 滑行停：方向脚清零，电机自由减速 */

    s_st.state    = APP_CAR_IDLE;
    s_st.spd_l    = 0;
    s_st.spd_r    = 0;
    s_st.turn     = 0.0f;
    s_st.base_now = 0;
    s_st.lost     = 0u;
    s_st.crossing = 0u;
}

void APP_Car_Brake(void)
{
    BSP_PID_Reset(&s_pid_turn);
    BSP_Motor_Brake();          /* 刹车停：绕组短接，最快停住 */

    s_st.state    = APP_CAR_IDLE;
    s_st.spd_l    = 0;
    s_st.spd_r    = 0;
    s_st.turn     = 0.0f;
    s_st.base_now = 0;
    s_st.lost     = 0u;
    s_st.crossing = 0u;
}

void APP_Car_ToggleRun(void)
{
    if (APP_Car_IsRunning() != 0u) {
        APP_Car_Stop();
    } else {
        APP_Car_Start();
    }
}

uint8_t APP_Car_IsRunning(void)
{
    return (s_st.state == APP_CAR_RUN) ? 1u : 0u;
}

uint8_t APP_Car_GetState(void)
{
    return s_st.state;
}

const APP_CarStatus_t *APP_Car_GetStatus(void)
{
    return &s_st;
}

uint16_t APP_Car_GetLostCount(void)
{
    return s_lost_cnt;
}

void APP_Car_ResetLostCount(void)
{
    s_lost_cnt   = 0u;
    s_lost_armed = 1u;      /* 重新武装，下一段地形可以正常计数 */
    s_seen_acc   = 0u;
}

void APP_Car_ResetPid(void)
{
    /* 清掉积分与微分历史。给路线模块用：盲走穿过地形之后重新压上线的那一刻，
       PID 里还留着"进地形之前"的历史，不清理的话恢复循线的第一拍会猛打一下方向 */
    BSP_PID_Reset(&s_pid_turn);
    s_pid_stale = 0u;
}

const char *APP_Car_StateName(uint8_t state)
{
    switch (state) {
    case APP_CAR_IDLE:  return "IDLE ";
    case APP_CAR_RUN:   return "RUN  ";
    case APP_CAR_LOST:  return "LOST ";
    case APP_CAR_STALL: return "STALL";
    default:            return "?????";
    }
}

/* ==========================================================================
 * 控制律 —— 全车最核心的 50 行
 * ========================================================================== */

void APP_Car_Update(void)
{
    uint32_t now;
    uint32_t dt_ms;
    float    dt_s;
    float   e;
    float   turn;
    float   base;
    int16_t l;
    int16_t r;
    uint8_t all_white;
    uint8_t all_black;

    /* ---- ⓪ 量出这一拍实际过了多久 ----
       ★ 名义上是 CTRL_PERIOD_MS（10ms），但 main 的节拍器是"到点就跑、
         错过不补"，所以 OLED 刷屏、串口打印之后的那一拍实测量会更大。
         把真实时间喂给 PID 和所有计时器，控制量才不会因为卡一下就突变。
         （bsp_encoder 也在做同样的事，两者用的都是 HAL_GetTick()） */
    now   = HAL_GetTick();
    dt_ms = (uint32_t)(now - s_last_tick);      /* 无符号相减，回绕安全 */
    s_last_tick = now;

    if (dt_ms < 1u) {
        dt_ms = 1u;
    }
    if (dt_ms > CAR_DT_MAX_MS) {
        dt_ms = CAR_DT_MAX_MS;
    }
    dt_s = (float)dt_ms / 1000.0f;

    /* ★ 把"电机转向取反"从参数刷进驱动层，每拍都刷。
       放在状态判断之前，所以待机时也在同步 —— 这样测试 4（电机正反转）
       用的也是菜单里拨的值：菜单里改完立刻去跑测试就能看到效果，
       不需要重新编译。改的只是方向极性，运行中改也安全 */
    BSP_Motor_SetInvert(g_param.motor_l_inv, g_param.motor_r_inv);

    /* ★ 探头极性同样每拍同步（理由和上面一样：菜单里一拨立刻生效，
       在 SENSOR CHECK 页能马上看到方框对没对） */
    BSP_Track_SetActiveLow(g_param.act_low);

    /* 遥测每拍都刷新（待机时屏幕上也有转速看） */
    car_refresh_telemetry();

    /* ---- 非行驶状态：只维持报警提示音，不碰电机 ---- */
    if (s_st.state == APP_CAR_IDLE) {
        return;
    }
    if (s_st.state != APP_CAR_RUN) {
        car_alert_poll(dt_ms);
        return;
    }

    s_run_ms += dt_ms;
    s_st.run_ms = s_run_ms;

    /* ---- ① 采样 ---- */
    s_st.mask     = BSP_Track_GetMask();
    all_white     = BSP_Track_IsAllWhite();
    all_black     = BSP_Track_IsAllBlack();
    s_st.crossing = all_black;

    /* ---- ② 误差归一化 ----
       BSP_Track_GetError() 是八路探头的加权平均位置误差：
         误差 < 0 → 线在左边（车偏右了）→ 要往左拐
         误差 > 0 → 线在右边（车偏左了）→ 要往右拐
       丢线（八路全白）时它返回上一次的有效误差而不是 0，
       所以不会被 PID 误当成"正好居中"。 */
    e = BSP_Track_GetError() / TRACK_ERROR_MAX;

    /* 转向取反开关：接线/装配导致左右反了，在菜单里改 STEERINV=1 即可，
       不用把车拆开重焊 */
    if (g_param.steer_inv != 0u) {
        e = -e;
    }
    s_st.err = e;

    /* 记住"最后一次看到线在哪一边"，丢线后要靠它决定往哪找。
       只在误差明显不为 0 时更新，避免在中心线附近被噪声来回改方向 */
    if (e > 0.05f) {
        s_last_dir = 1;
    } else if (e < -0.05f) {
        s_last_dir = -1;
    }

    /* ---- ② 转向思路：连续 PID 循迹（转角那套方案已舍弃）----
     *
     * 【当前采用的做法 —— 就是最朴素、最可靠的那一种】
     *   八路探头 → 加权位置误差 → PID → 左右轮差速，
     *   每个 10ms 周期都算一次，车边跑边连续纠偏。
     *   编码器只做测速与里程显示，不参与转向。
     *
     * 【曾经试过、现已舍弃的做法】
     *   "最外侧探头一亮 = 前方有转角 → 先保持直行 → 等车头全部探头
     *     看不见线了再按记住的方向急转"。思路本身没错，但：
     *     · 本车黑线较宽，车一大偏外侧探头也会亮，容易误判成转角
     *     · 直行那一段等于放弃了纠偏，车会冲出去
     *     · 地图固定，路线用脚本（app_route.c）分段控制更直接
     *   代码已经删干净，只留这段说明，免得后来的人又走一遍。
     *
     *   对应参数 EDGEN / PIVOTms 也一起删了；
     *   PIVOT 保留下来改名成路线脚本用的"转弯速度"。 */

    /* ---- ③ 路况修正：十字 ---- */
    if (all_black != 0u) {
        if (s_cross_acc < g_param.cross_hold_ms) {
            /* 头 CROSSms 毫秒强制按"居中"处理 → 直行通过路口。
               不这么做的话，车轮压上横向黑线时会跟着它甩头 */
            e = 0.0f;
            s_st.err = 0.0f;
        }
        s_cross_acc = (uint16_t)(s_cross_acc + dt_ms);
    } else {
        s_cross_acc = 0u;
    }

    /* ---- ③ 路况修正：丢线计时 ---- */
    if (all_white != 0u) {
        if (s_lost_acc < 0xFFFFu) {
            s_lost_acc = (uint16_t)(s_lost_acc + dt_ms);
        }
    } else {
        s_lost_acc = 0u;
    }
    s_st.lost    = (s_lost_acc >= CAR_LOST_DEBOUNCE_MS) ? 1u : 0u;
    s_st.lost_ms = s_lost_acc;

    /* ---- ③.5 ★ 丢线事件计数："全丢黑线"发生了几次 ----
     *
     * 这就是"地形编号"的来源：每块地形（石块/草地/减速带/台阶）都会盖住黑线，
     * 各产生一次全丢线事件，它们在固定赛道上的先后顺序是已知的，
     * 所以"第几次丢线"就等于"到了第几个地形"。
     *
     * 用【沿触发】而不是电平计数：
     *   只在"由有线变成全丢"的那一瞬间 +1，否则一次地形会被数成成百上千次。
     *
     * 用【重新武装】避免相邻地形被数成两次：
     *   计数之后要先"连续看到线达到 CAR_LOST_REARM_MS"才允许下一次计数。
     *   草地和减速带挨在一起、中间只有一小段线时，那一段线不够长，
     *   于是两块地形只算一次 —— 对得上"第几个地形"这个语义。 */
    if ((s_st.lost != 0u) && (all_white != 0u)) {
        /* 丢线中：看到线的累计时间清零 */
        s_seen_acc = 0u;

        if (s_lost_armed != 0u) {
            s_lost_armed = 0u;              /* 先缴械，避免同一段地形重复计数 */
            if (s_lost_cnt < 0xFFFFu) {
                s_lost_cnt++;
            }
        }
    } else if (s_lost_armed == 0u) {
        /* 已经不丢线了：攒够"连续看到线"的时间就把计数器重新武装 */
        if (s_seen_acc < 0xFFFFu) {
            s_seen_acc = (uint16_t)(s_seen_acc + dt_ms);
        }
        if (s_seen_acc >= CAR_LOST_REARM_MS) {
            s_lost_armed = 1u;
            s_seen_acc   = 0u;
        }
    }

    s_st.lost_cnt = s_lost_cnt;

    /* ---- ⑤ 转向环：连续 PID ----
       丢线时不用 PID，改成"按最后看到线的方向固定猛打"（见下面），
       其余时候都是 PID 连续纠偏 —— 这就是最终采用的转向思路 */
    car_pid_sync();

    if (s_st.lost != 0u) {
        /* 丢线期间不跑 PID：
           误差已经没有意义了，继续积分只会攒下一堆将在重新找到线时
           突然释放的"存量"，车会猛地一拐。改用固定转向量猛打方向找线。 */
        s_pid_stale = 1u;
        turn = (float)g_param.lost_turn * (float)s_last_dir;
    } else {
        if (s_pid_stale != 0u) {
            /* 刚从丢线里恢复：复位一次，让积分和微分历史从头开始。
               这样重新压上线的那一刻不会有"历史包袱"造成的抖动 */
            BSP_PID_Reset(&s_pid_turn);
            s_pid_stale = 0u;
        }
        turn = BSP_PID_UpdateErr(&s_pid_turn, e, dt_s);
    }

    /* ---- ⑥ 基础速度规划 ---- */
    base = (float)g_param.base_speed;

    if ((g_param.ramp_ms > 0u) && (s_run_ms < (uint32_t)g_param.ramp_ms)) {
        /* 起步斜坡：从 0 线性爬到 BASE SPD。
           直接给满速起步，轮子会打滑、车头会甩一下，循迹环还没反应过来
           就偏出去了。给 0.4 秒爬升，起步就稳得多。

           ★★ 转向量必须跟着一起缩放 —— 这是之前的一个真 bug ★★
           起步头几十毫秒里 base 只有几十，如果 turn 保持满值（可能有上千），
           那么 左 = base + turn 会是个大正数、右 = base - turn 会是个大负数：
           ★ 车一启动就是"原地打转"，而不是往前开。
           现象就是"按下 START 之后车不前进、直接往一边转"，而且看起来
           和循迹有没有压到线无关 —— 因为此刻转向环的输出根本没被限住。
           两边同比例缩放之后，起步过程中"差速/速度"的比值保持不变，
           车会沿着应有的方向平稳加速，绝不会打转。 */
        float k = (float)s_run_ms / (float)g_param.ramp_ms;

        base = base * k;
        turn = turn * k;
        s_ramping = 1u;
    } else {
        s_ramping = 0u;
    }

    /* ★★ 大偏差自动减速 —— 大车/履带车能不能循好线的关键一条 ★★
     *
     *   循迹是"看见了才开始纠"。车偏得越远，说明纠得越晚；
     *   这时候如果还保持全速，等转向环把车头掰回来，探头早就越过线了。
     *   8 路探头阵列就那么宽（前瞻距离很短），大车一冲就是几十厘米 ——
     *   现象就是"还没转过来就已经过线、驶出路线"。
     *
     *   减速把"看见 → 越过线"这段时间直接拉长：
     *       基速 = BASESPD × (1 - SLOWK × |误差|)
     *     误差 0   → 全速（直道完全不受影响）
     *     误差 0.5 → 约 70% 速度（SLOWK=0.6）
     *     误差 1.0 → 约 40% 速度（误差满格 = 线已经压到阵列最边上）
     *
     *   ★ 用"归一化误差的绝对值"，不用 PID 的输出：
     *     PID 输出已经限过幅，拿它减速会变成"弯道里恒为最低速"；
     *     用 |误差| 则和"车偏了多少"严格成正比，直观、可预测。
     *   ★ 下限 25%：再怎么偏也不能把车停下 —— 停住的车是没法转向的
     *     （履带车原地转向靠两边速度差，速度全归零就只剩摩擦了）。 */
    if (g_param.slow_k > 0.0f) {
        float mag = (e > 0.0f) ? e : -e;
        float k   = 1.0f - (g_param.slow_k * mag);

        if (k < CAR_SLOW_MIN_SCALE) {
            k = CAR_SLOW_MIN_SCALE;
        }
        base = base * k;
    }

    if (s_st.lost != 0u) {
        base = base * CAR_LOST_SPEED_SCALE;
    }

    s_st.base_now = (int16_t)(base + 0.5f);

    /* ---- ⑥ 差速合成 ---- */
    l = car_clamp_speed(base + turn);
    r = car_clamp_speed(base - turn);

    /* 内侧轮最低速：
       turn > 0 时车向右转，内侧是右轮。把它抬到 MINSPD 之上，
       弯道里右轮就不会停转 —— 停转会让车身"顿"一下，还会让编码器
       读数归零，看起来像丢速。设成 0 就允许内轮反转（能更急地转弯）

       ★ 但起步阶段绝不能生效：那时 base 还不到 MINSPD，
         把"慢的那一侧"硬抬到 MINSPD 反而会变成"它比另一侧还快"，
         车会朝反方向偏 —— 本来想让它跑顺，结果一启动就歪。
         所以要等基础速度爬过 MINSPD 之后，这条限制才有意义。 */
    if ((g_param.inner_min_spd > 0) && (s_ramping == 0u)
        && (base >= (float)g_param.inner_min_spd)) {
        if (turn > 0.0f) {
            if (r < g_param.inner_min_spd) {
                r = g_param.inner_min_spd;
            }
        } else if (turn < 0.0f) {
            if (l < g_param.inner_min_spd) {
                l = g_param.inner_min_spd;
            }
        }
    }

    s_st.turn  = turn;
    s_st.spd_l = l;
    s_st.spd_r = r;

    /* ---- ⑦ 输出 ---- */
    BSP_Motor_SetSpeed(l, r);

    /* ---- ⑧ 保护一：丢线超时（LOSTms = 0 时关闭这条保护）----
       ★ 默认已经关掉了。原因很实际：调试阶段车还跟不上线，
         "丢线 0.8 秒就刹车"只会让人不停地按 START，根本没法观察循迹效果。
         现在默认行为是"一直按最后看到线的方向找下去，直到人工停车"，
         完全符合"我要它一直走"的预期。
         等循迹整定好了，想让它在冲出赛道时自动停下来，
         再把菜单里的 LOSTms 调成 800~1500 即可。 */
    if (g_param.lost_stop_ms > 0u) {
        if ((s_st.lost != 0u) && (s_lost_acc >= g_param.lost_stop_ms)) {
            car_halt(APP_CAR_LOST);
            return;
        }
    }

    /* ---- ⑧ 保护二：堵转 ----
       判定条件：至少一个轮子被要求"明显转"，但两个轮子的编码器都不动。
       · 车真的卡住了（撞墙、轮子被按住、卷进东西）
       · 或者编码器线掉了 —— 这种情况也会在这里暴露出来，很有价值 */
    if (g_param.stall_ms > 0u) {
        int16_t d_l = BSP_Encoder_GetDelta(ENC_LEFT);
        int16_t d_r = BSP_Encoder_GetDelta(ENC_RIGHT);
        int32_t demand = (l > 0) ? l : -l;
        int32_t demand_r = (r > 0) ? r : -r;

        if (demand_r > demand) {
            demand = demand_r;
        }

        if ((demand > CAR_STALL_DUTY_MIN) && ((d_l == 0) && (d_r == 0))) {
            s_stall_acc = (uint16_t)(s_stall_acc + dt_ms);
            if (s_stall_acc >= g_param.stall_ms) {
                car_halt(APP_CAR_STALL);
                return;
            }
        } else {
            s_stall_acc = 0u;
        }
    }
}

/* ==========================================================================
 * 调试输出
 * ========================================================================== */

void APP_Car_PrintStatus(void)
{
    char mask_str[TRACK_SENSOR_NUM + 1u];

    BSP_Track_MaskToString(s_st.mask, mask_str);

    BSP_UART_Printf("st=%s t=%5u mask=%s err=%+5.2f turn=%+6.0f base=%4d L=%5d R=%5d rpmL=%7.1f rpmR=%7.1f\r\n",
                    APP_Car_StateName(s_st.state),
                    (unsigned int)s_run_ms,
                    mask_str,
                    (double)s_st.err,
                    (double)s_st.turn,
                    (int)s_st.base_now,
                    (int)s_st.spd_l,
                    (int)s_st.spd_r,
                    (double)s_st.rpm_l,
                    (double)s_st.rpm_r);
}
