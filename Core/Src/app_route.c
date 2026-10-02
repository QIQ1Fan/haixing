/**
  ******************************************************************************
  * @file    app_route.c
  * @brief   ★ 路线脚本与动作原语实现
  *
  * 【数据流】
  *   main.c / 串口命令
  *        └─► APP_Route_Run()
  *              └─► 一行一行调用动作原语（Route_xxx）
  *                    └─► 每个原语内部：下发电机指令 → 循环等时间到
  *                          └─► route_tick()  ← 每 10ms 一次，把系统维持住
  *                                ├─ BSP_Encoder_Update()   采样
  *                                ├─ BSP_Key_Scan()         按键
  *                                ├─ BSP_Buzzer_Poll()      关蜂鸣
  *                                ├─ APP_Car_Update()       只在"循线段"里调
  *                                ├─ APP_Leg_Update()       支撑脚斜坡
  *                                ├─ 按键处理：长按 K2 = 急停 + 中止路线
  *                                └─ OLED 状态刷新（每 200ms）
  *
  * 【为什么不干脆让脚本阻塞着空等】
  *   因为那样车会"发完指令就没人管了"：编码器不再采样、按键不再响应、
  *   蜂鸣器一直响、支撑脚不动。动作原语内部维持 tick 之后，
  *   脚本写起来是顺序的，运行起来却是"活的"。
  *
  * 【★ 循线段为什么要单独开关 APP_Car_Update】
  *   循线时是 APP_Car_Update() 在根据探头误差驱动电机；
  *   直行/转弯时是本模块在直接下指令。
  *   两者同时跑就会互相抢方向盘（PID 会立刻把直行指令改掉），
  *   所以 tick 里用一个参数决定"这一拍要不要让循迹环参与"。
  * ============================================================================
  */
#include "app_route.h"

#include "app_param.h"
#include "app_car.h"
#include "app_leg.h"

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_buzzer.h"
#include "bsp_oled.h"
#include "bsp_track.h"
#include "bsp_uart.h"

/* ==========================================================================
 * 内部状态
 * ========================================================================== */
static uint8_t  s_active     = 0u;      /* 1 = 正在跑路线 */
static uint8_t  s_abort      = 0u;      /* 1 = 收到中止请求（长按 K2） */
static uint8_t  s_step_no    = 0u;      /* 当前是第几个动作（给屏幕看） */
static const char *s_step_name = "----";/* 当前动作名（给屏幕看） */
static uint32_t s_pump_tick  = 0u;      /* tick 节拍基准 */
static uint32_t s_draw_tick  = 0u;      /* 屏幕刷新节流 */
static uint32_t s_step_start = 0u;      /* 本动作开始时刻 */
static uint16_t s_step_ms    = 0u;      /* 本动作总时长（给屏幕显示剩余） */

/* "鸣两声"阶段标记的后台状态机（0 = 空闲，1 = 第一声已响，2 = 第二声已响） */
static uint8_t  s_beep2_stage = 0u;
static uint32_t s_beep2_tick  = 0u;

#define ROUTE_BEEP2_ON_MS       70u     /* 每一声多久 */
#define ROUTE_BEEP2_GAP_MS      150u    /* 两声之间的间隔（从第一声开始算） */

/* ---- 预约式舵机动作队列 ----
   用来表达"一边循线一边摆支撑脚"这类【时间上重叠】的动作。
   队列只有几条，满了就立刻执行（宁可当场动一下，也不要静默丢掉） */
#define ROUTE_SRVQ_MAX          8u

typedef struct {
    uint32_t at_ms;         /* 该执行的绝对时刻（HAL_GetTick 的刻度） */
    uint8_t  deg;           /* 目标角度 */
    uint8_t  used;          /* 1 = 这个槽里有预约 */
} route_servo_evt_t;

static route_servo_evt_t s_srvq[ROUTE_SRVQ_MAX];

/* ---- 循线段里的"地形自动穿越"状态 ----
   见 bsp_config.h 里 CAR_TERRAIN_* 的说明。简单说：
   循线时如果探头全白超过 40ms，就认为压上了地形（石块/草地/减速带盖住了线），
   这一段改成"两轮同速直行"，等重新压到线再恢复循线。
   ★ 实测本赛道的地形线中断很短，这条兜底正常不会触发；
     但万一遇到更长的无线段，它能把车从"丢线打转"里救回来。 */
static uint8_t  s_terrain  = 0u;    /* 1 = 正在盲走穿地形 */
static uint16_t s_terr_in  = 0u;    /* 进入判据：连续全白的毫秒数 */
static uint16_t s_terr_run = 0u;    /* 盲走已经持续多少毫秒 */
static uint16_t s_terr_out = 0u;    /* 退出判据：盲走中连续看到线的毫秒数 */

/* route_tick() 的实现在本文件后半部分（10ms 小主循环那一段），
   而 route_lf_tick() 要用它，所以先声明一下。
   ★ 少了这一行 armcc 会按 C89 的隐式声明规则把它当成"返回 int、参数不定"，
     然后在真正的定义处报 "declaration is incompatible" —— 很难一眼看出原因 */
static void route_tick(uint8_t use_car);

/** 每段循线开始时清一次地形状态 */
static void route_lf_reset(void)
{
    s_terrain  = 0u;
    s_terr_in  = 0u;
    s_terr_run = 0u;
    s_terr_out = 0u;
}

/**
  * @brief  循线一拍（自带地形穿越）
  * @note   ★ 所有循线原语都走这个函数，而不是直接调 route_tick()。
  *         它比 route_tick() 多做两件事：
  *           1) 判断"是不是压上地形了"（全白超时）→ 是就切到盲走
  *           2) 盲走中判断"是不是穿出来了"（重新看到线）→ 是就切回循线
  */
static void route_lf_tick(void)
{
    const APP_CarStatus_t *st;

    /* 盲走时 use_car = 0：绝不让循迹环参与 ——
       它看到全白会走丢线逻辑（朝一边猛转），那正是要避免的 */
    route_tick((s_terrain == 0u) ? 1u : 0u);
    st = APP_Car_GetStatus();

    if (s_terrain == 0u) {
        /* ---- 正常循线：盯着"线是不是被盖住了" ---- */
        if (st->mask == 0u) {
            if (s_terr_in < 0xFFFFu) {
                s_terr_in = (uint16_t)(s_terr_in + CTRL_PERIOD_MS);
            }
            if (s_terr_in >= CAR_TERRAIN_ENTER_MS) {
                s_terrain  = 1u;        /* 切盲走 */
                s_terr_run = 0u;
                s_terr_out = 0u;
            }
        } else {
            s_terr_in = 0u;
        }
    } else {
        /* ---- 盲走中：盯着"是不是重新压到线了" ---- */
        if (s_terr_run < 0xFFFFu) {
            s_terr_run = (uint16_t)(s_terr_run + CTRL_PERIOD_MS);
        }

        if (st->mask != 0u) {
            if (s_terr_out < 0xFFFFu) {
                s_terr_out = (uint16_t)(s_terr_out + CTRL_PERIOD_MS);
            }
            if (s_terr_out >= CAR_MARK_HOLD_MS) {
                route_lf_reset();
                APP_Car_ResetPid();     /* 清历史，避免恢复第一拍猛打方向 */
            }
        } else {
            s_terr_out = 0u;
        }

        /* 盲走太久还看不到线 → 不是地形，是真丢了。
           交回正常的丢线逻辑（转向搜索），总比一直直着冲出去好 */
        if ((s_terrain != 0u) && (s_terr_run >= CAR_TERRAIN_MAX_MS)) {
            route_lf_reset();
        }
    }

    /* 盲走期间每拍都重新下发"两轮同速直行"：
       电机方向脚可能被别的动作动过，重发一次最省心 */
    if (s_terrain != 0u) {
        BSP_Motor_SetSpeed(g_param.base_speed, g_param.base_speed);
    }
}

#define ROUTE_DRAW_PERIOD_MS    200u    /* 屏幕上刷新路线的周期 */

/* ==========================================================================
 * 内部：一个 10ms 的"小主循环"
 * ========================================================================== */

/**
  * @brief  把主循环该做的事做一遍（路线运行期间由各原语的等待循环反复调用）
  * @param  use_car 1 = 这一拍让循迹环驱动电机（循线段用），0 = 不动电机
  */
static void route_tick(uint8_t use_car)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_pump_tick) < CTRL_PERIOD_MS) {
        return;                     /* 还没到下一拍 */
    }
    s_pump_tick = now;

    /* ---- ① 和主循环完全一样的周期任务 ---- */
    BSP_Encoder_Update();
    BSP_Key_Scan();
    BSP_Buzzer_Poll();

    if (use_car != 0u) {
        APP_Car_Update();           /* 只有循线段才让 PID 参与 */
    }

    APP_Leg_Update();               /* 支撑脚斜坡（Route_Servo 靠它走完） */

    /* ---- ② 按键：只认一件事 —— 长按 K2 = 急停并中止整条路线 ----
       ★ 这里刻意不把按键交给 app_menu：
         跑路线时不允许从菜单里插一脚（比如误触把参数改了、
         或者菜单的"长按 K2 = 保存"在行驶中擦 Flash）。
         路线期间屏幕和按键都由本模块独占。 */
    {
        uint8_t k;

        for (k = 0u; k < (uint8_t)KEY_ID_NUM; k++) {
            BSP_KeyEvent_t e = BSP_Key_GetEvent((BSP_KeyId_t)k);

            if ((k == (uint8_t)KEY_BACK) && (e == KEY_EVENT_LONG)) {
                APP_Route_Abort();
            }
        }
    }

    /* ---- ③ 阶段标记的两声蜂鸣：在这里被"喂"下去 ----
       非阻塞的关键就在这：Route_Beep2() 只登记一下，真正的两声由这里按时间发。
       所以调用它的地方立刻就返回了，下一段动作可以马上开始，不会因为
       "要等蜂鸣器响完"而在阶段之间停顿。 */
    if (s_beep2_stage == 1u) {
        if ((uint32_t)(now - s_beep2_tick) >= ROUTE_BEEP2_GAP_MS) {
            s_beep2_stage = 2u;
            s_beep2_tick  = now;
            BSP_Buzzer_BeepAsync(ROUTE_BEEP2_ON_MS);
        }
    } else if (s_beep2_stage == 2u) {
        if ((uint32_t)(now - s_beep2_tick) >= ROUTE_BEEP2_GAP_MS) {
            s_beep2_stage = 0u;
        }
    }

    /* ---- ③.5 到点的预约舵机动作 ----
       这就是"一边循线一边摆支撑脚"能实现的地方：循线段的等待循环
       反复调 route_tick()，每 10ms 顺手检查一次有没有到点的舵机动作。
       用的是"绝对时刻比较"，所以调用时机早晚几毫秒都不影响精度。 */
    {
        uint8_t q;

        for (q = 0u; q < ROUTE_SRVQ_MAX; q++) {
            if ((s_srvq[q].used != 0u)
                && ((int32_t)(now - s_srvq[q].at_ms) >= 0)) {
                s_srvq[q].used = 0u;
                APP_Leg_SetTarget(s_srvq[q].deg);   /* 走斜坡过去，不硬顶 */
            }
        }
    }

    /* ---- ④ 屏幕：让操作的人随时知道跑到第几步了 ---- */
    if ((uint32_t)(now - s_draw_tick) >= ROUTE_DRAW_PERIOD_MS) {
        uint32_t left = 0u;

        s_draw_tick = now;

        if (s_step_ms > 0u) {
            uint32_t used = (uint32_t)(now - s_step_start);
            left = (used < (uint32_t)s_step_ms) ? ((uint32_t)s_step_ms - used) : 0u;
        }

        BSP_OLED_Clear();
        BSP_OLED_DrawString(0u, 0u, "ROUTE RUNNING");
        BSP_OLED_Printf(0u, 2u, "STEP %u", (unsigned int)s_step_no);
        BSP_OLED_DrawString(0u, 3u, s_step_name);
        if (s_abort != 0u) {
            BSP_OLED_DrawString(0u, 5u, "ABORTED");
        } else {
            BSP_OLED_Printf(0u, 5u, "left %4u ms", (unsigned int)left);
        }
        BSP_OLED_DrawString(0u, 7u, "K2 HOLD:STOP");
        BSP_OLED_RefreshDirty();
    }
}

/**
  * @brief  等待 ms 毫秒，期间照常维持系统
  * @param  use_car 见 route_tick()
  * @retval 1 = 正常等完；0 = 被中止（调用者应尽快返回）
  * @note   所有动作原语都靠它来"等时间"，所以中止能在一两拍内传导到整条脚本
  */
static uint8_t route_wait(uint16_t ms, uint8_t use_car)
{
    uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < (uint32_t)ms) {
        if (s_abort != 0u) {
            return 0u;
        }
        route_tick(use_car);
    }
    return 1u;
}

/** 每个动作开头统一做的事：记下序号/名字/时长，给屏幕用 */
static void route_begin_step(const char *name, uint16_t ms)
{
    s_step_no++;
    s_step_name = name;
    s_step_ms   = ms;
    s_step_start = HAL_GetTick();
}

/* ==========================================================================
 * 动作原语
 * ========================================================================== */

void Route_LineFollow(uint16_t ms)
{
    route_begin_step("LINE FOLLOW", ms);

    if (s_abort != 0u) {
        return;
    }

    APP_Car_Start();                    /* 启动循迹（内部自带起步斜坡） */
    route_lf_reset();

    /* ★ 不用 route_wait()，改用 route_lf_tick()：
       它在等待期间除了维持系统，还会自动处理"线被地形盖住"的情况
       （见 bsp_config.h 里 CAR_TERRAIN_* 的说明） */
    {
        uint32_t t0 = HAL_GetTick();

        while ((uint32_t)(HAL_GetTick() - t0) < (uint32_t)ms) {
            if (s_abort != 0u) {
                break;
            }
            route_lf_tick();
        }
    }

    APP_Car_Stop();
}

void Route_LineFollowUntilCross(uint8_t n)
{
    uint8_t  seen = 0u;
    uint8_t  prev = 0u;
    uint32_t mark = HAL_GetTick();

    route_begin_step("LINE->CROSS", 0u);

    if ((s_abort != 0u) || (n == 0u)) {
        return;
    }

    APP_Car_Start();
    route_lf_reset();

    /* 一直跑到"经过 n 根横线"为止。没有时间上限 —— 固定地图上
       横线就是路标，用路标终止比用时间可靠得多。
       长按 K2 随时可以中止。 */
    while ((seen < n) && (s_abort == 0u)) {
        const APP_CarStatus_t *st;

        route_lf_tick();                /* 自带地形穿越 */

        st = APP_Car_GetStatus();

        /* 只在"由不黑变成全黑"的那一刻计一次数，并且两根横线之间
           至少隔 CAR_CROSS_MIN_GAP_MS —— 否则同一根横线会被数成好几根 */
        if ((st->crossing != 0u) && (prev == 0u)
            && ((uint32_t)(HAL_GetTick() - mark) >= CAR_CROSS_MIN_GAP_MS)) {
            seen++;
            mark = HAL_GetTick();
            BSP_Buzzer_BeepAsync(60u);      /* 每过一根横线"嘀"一声 */
        }
        prev = st->crossing;
    }

    APP_Car_Stop();
}

/**
  * @brief  ★★ 循线，直到"全丢黑线"发生第 n 次 ★★
  * @note   实现很短，因为机制都在 app_car 里：它按"沿触发 + 重新武装"维护
  *         s_lost_cnt（见 APP_Car_GetLostCount 的注释）。
  *         这里只是"一边循线一边盯着计数够没够"。
  */
void Route_LineFollowUntilLostCount(uint8_t n, uint16_t timeout_ms)
{
    uint32_t t0;

    route_begin_step("LINE->LOST", timeout_ms);

    if ((s_abort != 0u) || (n == 0u)) {
        return;
    }

    APP_Car_Start();
    route_lf_reset();                   /* 清地形兜底状态 */
    t0 = HAL_GetTick();

    /* 一边循线一边等计数。★ 用 route_lf_tick() 而不是 route_tick()：
       万一天形把线盖住得比较久，它会自动切成直行盲走，
       不会让丢线逻辑把车带着原地打转 */
    while ((APP_Car_GetLostCount() < (uint16_t)n) && (s_abort == 0u)) {
        route_lf_tick();

        if ((timeout_ms > 0u)
            && ((uint32_t)(HAL_GetTick() - t0) >= (uint32_t)timeout_ms)) {
            BSP_Buzzer_BeepAsync(400u);     /* 长音 = 一直没等到这个地形 */
            break;
        }
    }

    BSP_Buzzer_BeepAsync(120u);             /* 短音 = 到这个地形了 */

    /* ★ 这里刻意【不停车】也不 Stop()：
       等到的那一刻车正压在地形上，紧接着调用者就会用特殊方式开过去。
       停一下会让车在地形边缘顿住，反而不好。
       保留循迹运行态，下一个原语（比如 Route_DriveUntilLine）无缝接手。 */
}

void Route_LineFollowUntilMark(uint8_t on_mask, uint16_t timeout_ms)
{
    uint32_t t0;
    uint16_t hold = 0u;

    /* 把超时时间交给屏幕显示 —— 这一段的"剩余时间"就是"离放弃还差多久"，
       跑的时候心里有数（正常情况它不会走到 0） */
    route_begin_step("LINE->MARK", timeout_ms);

    if ((s_abort != 0u) || (on_mask == 0u)) {
        return;
    }

    APP_Car_Start();
    route_lf_reset();
    t0 = HAL_GetTick();

    for (;;) {
        const APP_CarStatus_t *st;

        route_lf_tick();                /* 循迹由 APP_Car_Update 驱动，自带地形穿越 */

        if (s_abort != 0u) {
            break;
        }

        st = APP_Car_GetStatus();

        /* ---- 路标匹配：要求的那几路必须【同时】压线，并保持一小会儿 ---- */
        if ((st->mask & on_mask) == on_mask) {
            hold = (uint16_t)(hold + CTRL_PERIOD_MS);

            if (hold >= CAR_MARK_HOLD_MS) {
                BSP_Buzzer_BeepAsync(120u);     /* 一声短音 = 看到路标了 */
                break;
            }
        } else {
            hold = 0u;                          /* 中途断开就重新计时 */
        }

        /* ---- 兜底超时：宁可带着偏差往下走，也不要卡死在这里 ---- */
        if ((timeout_ms > 0u)
            && ((uint32_t)(HAL_GetTick() - t0) >= (uint32_t)timeout_ms)) {
            BSP_Buzzer_BeepAsync(400u);         /* 一声长音 = 没看到路标，超时了 */
            break;
        }
    }

    APP_Car_Stop();
}

void Route_DriveUntilLine(int16_t speed, uint8_t on_mask, uint16_t timeout_ms)
{
    int16_t  spd;
    uint32_t t0;
    uint16_t hold = 0u;

    route_begin_step("CROSS TERRAIN", timeout_ms);

    if (s_abort != 0u) {
        return;
    }

    /* speed = 0 表示"用默认速度"，这样绝大多数情况调用者只要写 0 就行 */
    spd = (speed != 0) ? speed : g_param.base_speed;

    /* ★ 这里【不能】用 APP_Car_Update：
       它一看到探头全白就会走丢线逻辑（朝一边猛转），
       而那正是穿地形时必然出现的状态。
       所以本段完全绕开循迹环，直接给两轮同速 —— 真正的"盲走"。 */
    BSP_Motor_SetSpeed(spd, spd);
    t0 = HAL_GetTick();

    for (;;) {
        const APP_CarStatus_t *st;

        route_tick(0u);                 /* use_car = 0：不让 PID 抢方向盘 */

        if (s_abort != 0u) {
            break;
        }

        st = APP_Car_GetStatus();

        /* 看到线了（或看到指定图案）。同样要求保持一小会儿，
           防止车头刚探到线的那一瞬间误触发 */
        /* ★ 判据用的是 CAR_LOST_REARM_MS（500ms）而不是 CAR_MARK_HOLD_MS（60ms）：
           必须和"丢线计数器的重新武装时间"一致。
           草地和减速带挨在一起、中间只有一小段线时，那一小段线虽然能让探头
           看到 60ms 以上，却不够 500ms —— 于是判据不会在这里误认为"穿出来了"，
           车会继续直行压过减速带，两块地形只算一次。
           否则：车停在两块地形中间那一小段线上，脚本却以为已经过完了。 */
        if ((on_mask == 0u) ? (st->mask != 0u) : ((st->mask & on_mask) == on_mask)) {
            hold = (uint16_t)(hold + CTRL_PERIOD_MS);

            if (hold >= CAR_LOST_REARM_MS) {
                BSP_Buzzer_BeepAsync(120u);     /* 短音 = 穿出来了 */
                break;
            }
        } else {
            hold = 0u;
        }

        if ((timeout_ms > 0u)
            && ((uint32_t)(HAL_GetTick() - t0) >= (uint32_t)timeout_ms)) {
            BSP_Buzzer_BeepAsync(400u);         /* 长音 = 超时没找到线 */
            break;
        }
    }

    BSP_Motor_Stop();
}

void Route_Straight(uint16_t ms)
{
    int16_t spd = g_param.base_speed;

    route_begin_step("STRAIGHT", ms);

    if (s_abort != 0u) {
        return;
    }

    BSP_Motor_SetSpeed(spd, spd);       /* 两轮同速 = 直行 */
    (void)route_wait(ms, 0u);           /* ★ use_car = 0：不让 PID 抢方向盘 */
    BSP_Motor_Stop();
}

/** 原地转向的公共实现。dir = -1 左转，+1 右转 */
static void route_pivot(int8_t dir, uint16_t ms, const char *name)
{
    int16_t t = g_param.pivot_spd;
    int16_t l;
    int16_t r;

    route_begin_step(name, ms);

    if (s_abort != 0u) {
        return;
    }

    /* 逻辑上的左右：左转 = 左轮倒转、右轮正转（履带车搓地转向） */
    if (dir < 0) {
        l = (int16_t)(-t);
        r = t;
    } else {
        l = t;
        r = (int16_t)(-t);
    }

    /* ★ 和循迹、转角模式共用同一套左右约定：
       STEERINV=1（装配上左右反了）时整体对调，脚本里的
       Route_TurnLeft() 就永远是"真的往左转"，不用改脚本 */
    if (g_param.steer_inv != 0u) {
        int16_t tmp = l;
        l = r;
        r = tmp;
    }

    BSP_Motor_SetSpeed(l, r);
    (void)route_wait(ms, 0u);
    BSP_Motor_Stop();
}

void Route_TurnLeft(uint16_t ms)
{
    route_pivot(-1, ms, "TURN LEFT");
}

void Route_TurnRight(uint16_t ms)
{
    route_pivot(1, ms, "TURN RIGHT");
}

void Route_Servo(uint8_t deg)
{
    route_begin_step("SERVO", 0u);

    if (s_abort != 0u) {
        return;
    }

    APP_Leg_SetTarget(deg);

    /* 等它走到位（支撑脚斜坡由 APP_Leg_Update() 推进，route_tick 里会调），
       最多等 3 秒，避免机械卡住时脚本永远停在这里 */
    {
        uint32_t t0 = HAL_GetTick();

        while ((APP_Leg_IsMoving() != 0u) && (s_abort == 0u)
               && ((uint32_t)(HAL_GetTick() - t0) < 3000u)) {
            route_tick(0u);
        }
    }
}

void Route_ServoAt(uint8_t deg, uint16_t after_ms)
{
    uint8_t i;

    /* 找一个空槽登记。★ 用"绝对时刻"而不是"相对剩余时间"，
       这样每拍只要比一次大小，不用担心节拍抖动累积误差 */
    for (i = 0u; i < ROUTE_SRVQ_MAX; i++) {
        if (s_srvq[i].used == 0u) {
            s_srvq[i].used  = 1u;
            s_srvq[i].deg   = deg;
            s_srvq[i].at_ms = HAL_GetTick() + (uint32_t)after_ms;
            return;
        }
    }

    /* 队列满：不静默丢弃，立刻动一下 —— 少摆一次比"什么都没发生"好查 */
    APP_Leg_SetTarget(deg);
}

void Route_ServoNow(uint8_t deg)
{
    route_begin_step("SERVO NOW", 0u);

    if (s_abort != 0u) {
        return;
    }

    APP_Leg_JumpTo(deg);
}

void Route_Stop(uint16_t ms)
{
    route_begin_step("STOP", ms);

    BSP_Motor_Stop();
    (void)route_wait(ms, 0u);
}

void Route_Brake(void)
{
    route_begin_step("BRAKE", 0u);
    BSP_Motor_Brake();
}

void Route_Beep(uint16_t ms)
{
    route_begin_step("BEEP", 0u);

    BSP_Buzzer_Beep(ms);        /* 这里可以用阻塞版：动作本身就是"响一下" */
}

void Route_Beep2(void)
{
    /* 只登记，不等待。真正的两声在 route_tick() 里按时间发出去，
       所以这个函数是"瞬时"的 —— 阶段之间不会因为蜂鸣而停顿 */
    s_beep2_stage = 1u;
    s_beep2_tick  = HAL_GetTick();
    BSP_Buzzer_BeepAsync(ROUTE_BEEP2_ON_MS);
}

/**
  * @brief  ★ 左右方向自检：原地左转 1 秒 → 停 1 秒 → 原地右转 1 秒
  * @note   用途是"一次性把左右看清楚"，不用再靠推理。
  *         跑法：把车架空（或放在空旷地面上），串口敲 T。
  *         看两个动作分别把车头转向哪边，就能确定：
  *           · 标着"左转"的那个动作真的往左  → 方向全对
  *           · 标着"左转"的却往右            → STEERINV 反了，
  *             去菜单 TUNE PARAMS 把 STEERINV 拨一下
  *         ★ 注意：它走的是和路线脚本完全相同的 Route_TurnLeft/Right，
  *           所以这里看到的方向，就是路线里那三步会走的方向。
  */
void APP_Route_TestTurn(void)
{
    s_active    = 1u;
    s_abort     = 0u;                   /* 清掉上一次跑路线留下的中止标志 */
    s_step_no   = 0u;
    s_pump_tick = HAL_GetTick();
    s_draw_tick = HAL_GetTick();
    s_beep2_stage = 0u;

    APP_Car_Stop();
    BSP_Motor_Stop();

    BSP_UART_SendString("\r\n[turn] LEFT 1s -> stop -> RIGHT 1s. watch the car.\r\n");
    Route_Beep(150u);

    Route_TurnLeft(1000u);
    Route_Stop(800u);
    Route_TurnRight(1000u);
    Route_Stop(400u);

    s_active = 0u;
    BSP_UART_SendString("[turn] done. if LEFT went right, flip STEERINV\r\n");
    BSP_OLED_ClearAndRefresh();
}

/* ==========================================================================
 * 入口
 * ========================================================================== */

uint8_t APP_Route_IsActive(void)
{
    return s_active;
}

void APP_Route_Abort(void)
{
    s_abort = 1u;
}

/* ==========================================================================
 * ★★★  路 线 脚 本 —— 改路线只改这一个函数  ★★★
 * --------------------------------------------------------------------------
 *   每一行是一个动作，按从上到下的顺序执行。
 *   参数单位都是【毫秒】，舵机那一个是【角度 0~180】。
 *
 *   写脚本的思路（固定地图）：
 *     1) 先按地图把路线切成几段：每段要么"循线走多远"，要么"直行/转向多久"
 *     2) 有横线（路口、停止线）的地方优先用 Route_LineFollowUntilCross(n)，
 *        它比"循线 x 毫秒"可靠 —— 不随电池电压、地面摩擦变化
 *     3) 原地转向的角度靠时间标定：跑一次，量出实际转了多少度，
 *        再按比例改毫秒数（比如 600ms 转了 70°，那 90° 就写 770ms）
 *     4) 每段之间可以插 Route_Beep()，跑的时候听声音就知道走到哪一步了
 * --------------------------------------------------------------------------
 *   当前路线（时长值都在菜单 ROUTE TIME 里调）：
 * ========================================================================== */
/* --------------------------------------------------------------------------
 * 路线各段的时长已经改成【参数】了，不再写死在这里。
 * 改时长有两个入口，都不用重新编译：
 *   · 菜单 → ROUTE TIME      （一屏看全六段，专为调试路线用）
 *   · 菜单 → TUNE PARAMS     （LF1_ms / TN1_ms / ... 同名参数）
 * 参数名与步骤的对应：
 *   LF1_ms = step1 循线    TN1_ms = step2 转弯
 *   LF2_ms = step3 循线    TN2_ms = step4 转弯
 *   LF3_ms = step5 循线    TN3_ms = step6 转弯
 * 转弯时长仍需实车标定：跑一次量出实际转了多少度，按比例改。
 * -------------------------------------------------------------------------- */

void APP_Route_Run(void)
{
    /* ---- 进入路线模式：先把状态清干净 ---- */
    s_active  = 1u;
    s_abort   = 0u;
    s_step_no = 0u;
    s_pump_tick = HAL_GetTick();
    s_draw_tick = HAL_GetTick();

    /* 清掉上一次跑剩下的预约舵机动作，免得这次一启动就乱摆 */
    {
        uint8_t q;

        for (q = 0u; q < ROUTE_SRVQ_MAX; q++) {
            s_srvq[q].used = 0u;
        }
    }

    /* ★ 丢线计数从 0 开始：路线上"第几次全丢线"就是"第几个地形"，
       每一趟都要重新数，不然第二趟的判断就全错了 */
    APP_Car_ResetLostCount();

    APP_Car_Stop();
    BSP_Motor_Stop();

    BSP_UART_SendString("\r\n[route] start\r\n");

    /* ======================= ↓↓↓ 路线从这里开始 ↓↓↓ ======================= */

    /* ======================================================================
     * ★★★ 本路线用"全丢黑线的次数"当地形编号 ★★★
     * ----------------------------------------------------------------------
     *   赛道上每一块地形（石块 / 草地+减速带 / 台阶）都是平铺上去的板子，
     *   会把下面的黑线盖住 —— 每经过一块地形，就必然产生一次【全丢线】。
     *   地形在固定赛道上的先后顺序是已知的，于是：
     *
     *       Route_LineFollowUntilLostCount(1u, ...) = 循线直到第 1 个地形
     *       Route_LineFollowUntilLostCount(2u, ...) = 循线直到第 2 个地形
     *       ...
     *
     *   等到了地形，就用"这个地形专用的方式"开过去，然后接着循线。
     *   ★ 全程都在循迹状态，只是到了特定编号换一种跑法。
     *   ★ 它和时间、电池电压无关：慢一点就晚一点到，编号永远对得上。
     *
     *   【跑第一趟时请确认计数】OLED 状态页第 6 行： "M00111100 L  0 N 1"
     *     N 就是当前计数。要看两件事：
     *       1) 每压上一块地形，N 是不是正好 +1
     *       2) 草地和减速带是【两块挨着的地形】，中间那一小段线够不够
     *          让计数器重新武装（CAR_LOST_REARM_MS = 500ms）——
     *          够 → 会被数成 2 次，后面台阶的编号要跟着 +1
     *          不够 → 只数 1 次，就是下面写的这样
     *     对不上就改下面那几个数字，别改别的。
     * ====================================================================== */

    /* ------------------------- 地形 1：石块 ------------------------------ */
    Route_LineFollowUntilLostCount(1u, g_param.rt_lf1_ms);
    Route_DriveUntilLine(0, 0u, g_param.rt_cross_ms);   /* 盲走穿过：不看线、
                                                           两轮同速直行，
                                                           重新连续压到线才算出来 */
    Route_Beep2();

    Route_TurnLeft(g_param.rt_tn1_ms);                  /* 左转 */
    Route_Beep2();

    /* --------------------- 地形 2：草地 + 减速带 ------------------------- */
    Route_LineFollowUntilLostCount(2u, g_param.rt_lf2_ms);
    Route_DriveUntilLine(0, 0u, g_param.rt_cross_ms);
    Route_Beep2();

    Route_TurnLeft(g_param.rt_tn2_ms);                  /* 左转 */
    Route_Beep2();

    /* ------------------- 一段纯循线（到下一个路口）----------------------- */
    /* 这一段没有地形，用"经过 1 根横线"来收尾 ——
       比时间可靠，而且不需要预先量图案（八路全压就是横线）。
       ★ 如果这里等不到横线（每次都听长音），说明线不够宽，
         请用菜单 SENSOR VIEW 推过路口看一下掩码，再改判据。 */
    Route_LineFollowUntilCross(1u);
    Route_Beep2();

    Route_TurnLeft(g_param.rt_tn3_ms);                  /* 左转 */
    Route_Beep2();

    /* ---------------- 地形 3：上台阶（换第三种跑法）--------------------
     *
     * 先循线到第 3 个地形（★ 如果草地和减速带被数成两次，这里就写 4u）
     * 这一幕"一边走一边摆脚"是时间重叠的，所以用 Route_ServoAt() 预约三次
     * 舵机动作，然后立刻开始盲走 —— 走的过程中后台节拍按时间点把舵机动到位。
     *
     *   时间轴（相对本段开始）：
     *       t = 0.0s   支撑脚放下到 LEGDN（约 0.9 秒走完）
     *       t = 2.0s   支撑脚升起到 LEGUP（约 0.9 秒走完）
     *       t = 3.0s   支撑脚放下到 LEGDN
     *
     *   ★ 角度用参数 LEGDN / LEGUP（菜单里标定过的"放下位/收起位"），
     *     不写死 180/0 —— 以后标定改了角度，这里自动跟着改。
     *   ★ 想让摆动更"甩"（不等走完就折返），把时间点往前提（2000 → 1200）。 */
    Route_LineFollowUntilLostCount(3u, g_param.rt_lf4_ms);

    Route_ServoAt((uint8_t)g_param.leg_down_deg,    0u);    /* t=0    放下 */
    Route_ServoAt((uint8_t)g_param.leg_up_deg,   2000u);    /* t=2s   升起 */
    Route_ServoAt((uint8_t)g_param.leg_down_deg, 3000u);    /* t=3s   再放下 */

    Route_DriveUntilLine(0, 0u, g_param.rt_cross_ms);       /* 盲走爬上台阶 */
    Route_Beep2();

    /* 路线结束 */
    Route_Beep(400u);                   /* 结尾用一声长音，和中间的"两声"区分开 */

    /* ======================= ↑↑↑ 路线到这里结束 ↑↑↑ ======================= */

    /* ---- 收尾：不管怎么结束的，都必须把车停下来 ---- */
    BSP_Motor_Brake();
    APP_Car_Stop();
    APP_Route_Abort();          /* 让后续的原语也立刻返回（中止时用） */
    s_active = 0u;

    BSP_UART_SendString("[route] done\r\n");

    /* 屏幕交回菜单 */
    BSP_OLED_ClearAndRefresh();
}

/**
  * @brief  问一句"要不要跑路线"，然后按选择执行
  * @note   main.c 初始化完成后调用一次。选 K2 就直接返回进主循环（调参模式），
  *         选 K1 就跑整条路线，跑完返回主循环。
  */
void APP_Route_Enter(void)
{
    uint32_t t0;

    s_abort = 0u;

    /* ---- 显示询问画面 ---- */
    BSP_OLED_Clear();
    BSP_OLED_DrawString(0u, 0u, "HAIXING READY");
    BSP_OLED_DrawString(0u, 2u, "K1 : RUN ROUTE");
    BSP_OLED_DrawString(0u, 3u, "K2 : MENU/TUNE");
    BSP_OLED_DrawString(0u, 5u, "put car on the line");
    BSP_OLED_DrawString(0u, 7u, "");
    BSP_OLED_Refresh();

    BSP_UART_SendString("\r\n[route] K1 = RUN ROUTE, K2 = MENU\r\n");

    /* ---- 等按键。这里也走 route_tick，方便看状态 ---- */
    s_pump_tick = HAL_GetTick();
    s_draw_tick = HAL_GetTick();
    t0 = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();
        uint8_t  k;

        /* 超时 30 秒没按键就自动进菜单，防止有人开了电就走开、
           车一直停在询问画面上 */
        if ((uint32_t)(now - t0) > 30000u) {
            BSP_UART_SendString("[route] timeout -> menu\r\n");
            return;
        }

        /* ==================================================================
         * 按键必须严格按 10ms 一拍来扫 —— 这里之前写错了
         *
         *   原来这段是"死循环里不停调 BSP_Key_Scan()"。
         *   死循环几微秒就能转一圈，于是：
         *     · 消抖计数（要连续 3 次）瞬间加满 → 消抖形同不存在
         *     · 长按计数（要 60 次）也在几百微秒内加满 → 手刚按下就判成"长按"
         *     · 一旦判过长按，松手时就不再补发"短按"事件
         *   结果：按键一直在报长按/连发，而这段代码只认短按，
         *   所以按 K1、K2 完全没有反应。
         *
         *   按键状态机的时间阈值全是"扫描次数 × 10ms"，
         *   节奏必须和主循环里保持一致，否则整套判定全错。
         * ================================================================== */
        if ((uint32_t)(now - s_pump_tick) < CTRL_PERIOD_MS) {
            continue;                   /* 还没到下一拍 */
        }
        s_pump_tick = now;

        BSP_Key_Scan();
        BSP_Buzzer_Poll();

        for (k = 0u; k < (uint8_t)KEY_ID_NUM; k++) {
            BSP_KeyEvent_t e = BSP_Key_GetEvent((BSP_KeyId_t)k);

            /* 短按、长按都认 —— 按久一点也算，别让人以为没反应 */
            if ((e != KEY_EVENT_PRESS) && (e != KEY_EVENT_LONG)) {
                continue;
            }
            if (k == (uint8_t)KEY_OK) {
                BSP_Buzzer_KeyFeedback();
                BSP_UART_SendString("[route] RUN\r\n");

                /* 给 1 秒缓冲：按下 K1 之后手离开按键、人退开一点，再让车动起来。
                   不然车会在手还搭在车上时冲出去 */
                BSP_OLED_Clear();
                BSP_OLED_DrawString(0u, 3u, "GO ...");
                BSP_OLED_Refresh();
                {
                    uint32_t g0 = HAL_GetTick();

                    while ((uint32_t)(HAL_GetTick() - g0) < 1000u) {
                        BSP_Buzzer_Poll();
                    }
                }

                APP_Route_Run();        /* 跑整条路线，跑完返回 */
                return;
            }
            if (k == (uint8_t)KEY_BACK) {
                BSP_Buzzer_KeyFeedback();
                BSP_UART_SendString("[route] canceled -> menu\r\n");
                return;
            }
        }
    }
}
