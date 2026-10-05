/**
  ******************************************************************************
  * @file    app_route.c
  * @brief   ★ 路线脚本与动作原语实现
  *
  * 【数据流】
  *   开机 K1 / 菜单 RUN ROUTE / 菜单 START STEP / 串口 G
  *        └─► APP_Route_Run()  或  APP_Route_RunFrom(第几步)
  *              └─► 一行一行调用动作原语（Route_xxx）
  *                    └─► 每个原语内部：下发电机指令 → 循环等"时间到/事件到"
  *                          └─► route_tick()  ← 每 10ms 一次，把系统维持住
  *                                ├─ BSP_Encoder_Update()   采样
  *                                ├─ BSP_Key_Scan()         按键
  *                                ├─ BSP_Buzzer_Poll()      关蜂鸣
  *                                ├─ APP_Car_Update()       只在"循线段"里调
  *                                ├─ APP_Leg_Update()       支撑脚斜坡
  *                                ├─ 到点的预约舵机动作（Route_ServoAt）
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
  *
  * 【当前路线一览】—— 详细脚本在文件后半部分的 APP_Route_Run() 里
  *
  *     step1   最左三路全压线        → 盲走 BLD1_ms（默认 4.5 s）
  *     step2   第 1 根横线            → 循迹 POST_ms(1 s) → 左转 TN1_ms
  *     step3   第 3 根横线            → 盲走 BLD2_ms（默认 5.5 s）
  *     step4①  第 4 根横线            → 循迹 POST_ms → 左转 TN2_ms
  *     step4②  下一次左三压线         → 盲走 STR_ms  → 左转 TN4_ms
  *     step4③  第 5 根横线            → 循迹 POST_ms → 左转 TN3_ms
  *     step5   第 6 根横线（台阶前）  → 盲走 UPST_ms（默认 9 s），期间舵机四拍
  *     step6   第 7 根横线（不转向）  → 循迹 1.4 s → 盲走 3.5 s（t=2s 放脚 / t=3s 收回）
  *                                      → 循到第 8 根线 → 循迹 POST3_ms(0.7 s) → 左转 TN5_ms(1 s)
  *     step7   左转转完后的循迹段      → 循迹 LF7_ms(0.7 s) → 舵机放下→升回 → 到"左路有线"
  *                                      → 掉头 TN6_ms(2 s) → 循迹 0.7 s → 盲走 3.8 s（舵机放下→升回）
  *                                      → 切回循迹 → 循到第 9 根线（不停，接 step8）
  *     step8   第 9 根线之后          → 循迹 LF8_ms(3 s) → 右转 100° TN8_ms(1.1 s)
  *                                      → 盲走 3.5 s（第 1.4s 舵机放下→第 2.8s 升回）
  *                                      → 倒车 S8_REV_MS(3 s) → 左转 120° TN7_ms(1.3 s)
  *                                      → 盲走 2 s
  *                                      → 切回循迹 → 循到第 10 根线急停，路线结束
  *
  *   一共 5 次左转（含 2 次掉头 180°）、2 次右转、6 处舵机收放。
  *   所有时长都在菜单 ROUTE TIME 页里可调，不用重编。
  *
  * 【三个必须知道的机制】
  *   ① 循线段用"循迹结果事件"推进，不用时间 ——
  *      电池掉电压、车速变慢，只是晚一点到，"第几根横线"永远对得上。
  *
  *   ② ★★ 盲走 / 直走 / 转弯期间【不计数】★★
  *      那几段是闭着眼睛（或原地转）走的，压到横线也不算。
  *      所以脚本里的"第几根横线"= "循迹时压到的第几根"，不是物理第几根。
  *      ⇒ 改了任何一段盲走的起点，后面所有编号都要跟着重算
  *        （包括 START STEP 用的那张预置表，见 APP_Route_Run 上方）。
  *
  *   ③ 舵机动作是【预约】出去的（Route_ServoAt）：调用只花几微秒就返回，
  *      真正的动作由后台节拍按时间点执行 —— 所以车可以一边不停顿地走、
  *      支撑脚一边按节奏收放（爬台阶就靠这个）。
  *
  * 【跑完 / 中止之后不会直接回菜单】
  *   会停在"结果页"（ROUTE DONE / ROUTE ABORTED）上，
  *   把停在第几步、三个计数器各是多少、探头现在看到什么留在屏幕上，
  *   按任意键才回菜单。见 route_hold_result()。
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

/* ★ 倒车速度 = 基础速度 × 这个系数（1.0 = 和前进一样快）。
   倒车不敢太快：后轮容易打滑，而且盲倒时车尾没有传感器，冲过头救不回来。 */
#define ROUTE_REV_RATIO     0.7f

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

/* ---- ★ 从第几个阶段开始跑 ----
 * 1 = 从头跑（正常情况）。菜单里选 "START STEP" 可以指定从某一步开始，
 * 某一步失误之后就不用整条重来了。
 * 见 APP_Route_RunFrom() 和脚本里每段开头的 if (s_from <= N) 判断。 */
static uint8_t  s_from       = 1u;

/* "鸣两声"阶段标记的后台状态机（0 = 空闲，1 = 第一声已响，2 = 第二声已响） */
static uint8_t  s_beep2_stage = 0u;
static uint32_t s_beep2_tick  = 0u;

#define ROUTE_BEEP2_ON_MS       70u     /* 每一声多久 */
#define ROUTE_BEEP2_GAP_MS      150u    /* 两声之间的间隔（从第一声开始算） */

#define ROUTE_DRAW_PERIOD_MS    200u    /* 屏幕上刷新路线的周期 */

/* ---- 预约式舵机动作队列 ----
 * 用来表达"车一边盲走、舵机一边按时间点收放"这类【时间上重叠】的动作。
 * 脚本是顺序执行的，一行不跑完不会跑下一行 —— 所以把舵机动作
 * "预约"出去（只登记，立刻返回），真正的动作交给后台节拍按时间点执行。 */
#define ROUTE_SRVQ_MAX          8u

typedef struct {
    uint32_t at_ms;         /* 该执行的绝对时刻（HAL_GetTick 刻度） */
    uint8_t  deg;           /* 目标角度 */
    uint8_t  used;          /* 1 = 这个槽里有预约 */
} route_servo_evt_t;

static route_servo_evt_t s_srvq[ROUTE_SRVQ_MAX];

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
       盲走段（Route_Straight）的等待循环反复调 route_tick()，
       每 10ms 顺手检查一次有没有到点的舵机动作 —— 这就是
       "车不停顿地走、舵机按点收放"能实现的地方。
       用绝对时刻比较，所以节拍抖动几毫秒不影响动作时刻。 */
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

        const APP_CarStatus_t *st = APP_Car_GetStatus();

        BSP_OLED_Clear();
        BSP_OLED_DrawString(0u, 0u, "ROUTE RUNNING");
        BSP_OLED_Printf(0u, 2u, "STEP %u", (unsigned int)s_step_no);
        BSP_OLED_DrawString(0u, 3u, s_step_name);
        if (s_abort != 0u) {
            BSP_OLED_DrawString(0u, 5u, "ABORTED");
        } else if (s_step_ms > 0u) {
            BSP_OLED_Printf(0u, 5u, "left %4u ms", (unsigned int)left);
        } else {
            BSP_OLED_DrawString(0u, 5u, "wait event...");
        }
        /* ★ 三个事件计数实时显示：左三 / 右三 / 全黑 + 当前掩码。
           跑的时候盯着它，就能看出"脚本现在等在第几根线"、
           以及是不是多算了一根 —— 排查计数问题全靠这一行。 */
        {
            char mstr[TRACK_SENSOR_NUM + 1u];

            BSP_Track_MaskToString(BSP_Track_GetMask(), mstr);
            BSP_OLED_Printf(0u, 6u, "L%u R%u B%u %s",
                            (unsigned int)st->cnt_left3,
                            (unsigned int)st->cnt_right3,
                            (unsigned int)st->cnt_black,
                            mstr);
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
    (void)route_wait(ms, 1u);           /* 等待期间由 APP_Car_Update 驱动电机 */
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

    /* 一直跑到"经过 n 根横线"为止。没有时间上限 —— 固定地图上
       横线就是路标，用路标终止比用时间可靠得多。
       长按 K2 随时可以中止。 */
    while ((seen < n) && (s_abort == 0u)) {
        const APP_CarStatus_t *st;

        route_tick(1u);

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

/* 等事件的安全网：这么久还没等到就放弃、继续往下走（毫秒）。
   60 秒足够跑完最长的直道；真卡住时也不会让车一直往前冲 */
#define ROUTE_EVENT_TIMEOUT_MS  60000u

/**
  * @brief  循线的公共等待：一直循线，直到某个事件的计数【超过 base】
  * @param  base 进入时的计数（等"绝对第 n 次"就传 n-1；等"下一次"就传当前值）
  * @retval 1 = 等到了；0 = 超时或被中止
  * @note   两种用法都由它实现，见下面两个对外函数。
  */
static uint8_t route_wait_event_over(uint8_t evt, uint16_t base)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t shown = base;

    /* ★★ 先"作废当前正压着的那一根" ★★
     *   车停下或转完弯时，探头可能正压在线上（或压在地形黑边上）。
     *   不作废的话，一进来第一拍就白送一次计数 ——
     *   脚本就会提前一根线触发，现象正是"莫名检测到一根线"。
     *   作废之后：必须先松开（保持 400ms），再重新压到才算新的一根。 */
    APP_Car_ArmOnNext(evt);

    /* 一边循线一边等计数。★ 期间完全交给循迹环驱动电机，
       所以车一直在线上纠偏，不是"闭着眼睛等"。 */
    while ((APP_Car_GetEventCount(evt) <= base) && (s_abort == 0u)) {
        route_tick(1u);

        /* 每跨过一次计数就"嘀"一声：跑的时候凭声音就知道过了几根横线，
           不用盯着屏幕 */
        if (APP_Car_GetEventCount(evt) != shown) {
            shown = APP_Car_GetEventCount(evt);
            BSP_Buzzer_BeepAsync(60u);
        }

        if ((uint32_t)(HAL_GetTick() - t0) >= ROUTE_EVENT_TIMEOUT_MS) {
            BSP_Buzzer_BeepAsync(400u);     /* 长音 = 一直没等到，放弃了 */
            return 0u;
        }
    }

    if (s_abort != 0u) {
        return 0u;
    }

    BSP_Buzzer_BeepAsync(120u);             /* 短音 = 等到了 */
    return 1u;
}

void Route_LineFollowUntilEvent(uint8_t evt, uint16_t n)
{
    route_begin_step("LINE->EVENT", 0u);

    if ((s_abort != 0u) || (n == 0u)) {
        return;
    }

    APP_Car_Start();

    /* "等到第 n 次" = "等到计数超过 n-1" */
    (void)route_wait_event_over(evt, (uint16_t)(n - 1u));

    /* ★ 这里【不停车】：等到的那一刻车还在线上正常跑，
       下一个动作（左转 / 盲走）紧接着开始，停一下反而会顿一下 */
}

void Route_LineFollowUntilNextEvent(uint8_t evt)
{
    uint16_t base;

    route_begin_step("LINE->NEXT", 0u);

    if (s_abort != 0u) {
        return;
    }

    /* ★ 记下"现在数到几了"，然后等它变大 —— 也就是等【下一次】该图案出现。
       为什么不写死"第几次"：同一个图案在一趟路线里可能已经出现过好几次
       （比如线偏到左边也会触发"左三压线"），写死编号就得先去数清楚，
       很容易对不上；"下一次"永远是对的。 */
    base = APP_Car_GetEventCount(evt);

    APP_Car_Start();
    (void)route_wait_event_over(evt, base);
}

void Route_LineFollowUntilNextLeft3(void)
{
    Route_LineFollowUntilNextEvent((uint8_t)APP_CAR_EVT_LEFT3);
}

void Route_LineFollowUntilNextBlack(void)
{
    Route_LineFollowUntilNextEvent((uint8_t)APP_CAR_EVT_BLACK);
}

void Route_LineFollowUntilLeft3(uint16_t n)
{
    Route_LineFollowUntilEvent((uint8_t)APP_CAR_EVT_LEFT3, n);
}

void Route_LineFollowUntilRight3(uint16_t n)
{
    Route_LineFollowUntilEvent((uint8_t)APP_CAR_EVT_RIGHT3, n);
}

void Route_LineFollowUntilBlack(uint16_t n)
{
    Route_LineFollowUntilEvent((uint8_t)APP_CAR_EVT_BLACK, n);
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

/**
 * @brief  倒车 ms 毫秒（两轮同速反转），走完自动停
 * @note   ★ 和 Route_Straight 是一对：只差速度的符号。
 *         倒车速度取基础速度的 ROUTE_REV_RATIO 倍 —— 倒车不能太快，
 *         一是后轮容易打滑，二是盲倒太久后面没有传感器，冲过头救不回来。
 *         想改倒车快慢就调那个宏（1.0 = 和前进一样快）。
 */
void Route_Reverse(uint16_t ms)
{
    int16_t spd = (int16_t)(-(float)g_param.base_speed * ROUTE_REV_RATIO);

    route_begin_step("REVERSE", ms);

    if (s_abort != 0u) {
        return;
    }

    if (spd == 0) {
        spd = -1;                       /* 防止参数被调成 0 时倒不动 */
    }

    BSP_Motor_SetSpeed(spd, spd);       /* 两轮同速反转 = 倒车 */
    (void)route_wait(ms, 0u);           /* ★ use_car = 0：不让 PID 抢方向盘 */
    BSP_Motor_Stop();
}

/**
 * @brief  原地转向：dir = -1 左转、+1 右转；spd = 搓地的速度；ms = 转多久
 * @note   ★ 速度和时长都显式传进来 —— 路线里的 Route_TurnLeft/Right 用
 *         g_param.pivot_spd，而菜单里的"转向标定工具"要用它自己设的速度，
 *         所以这里不能写死取参数。
 */
void Route_Pivot(int8_t dir, int16_t spd, uint16_t ms, const char *name)
{
    int16_t t = spd;

    if (t < 0) {
        t = (int16_t)(-t);              /* 速度只取大小，方向由 dir 决定 */
    }
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

/**
 * @brief  原地左转，直到八路探头【全部压线（全黑）】为止
 * @note   ★ 和 Route_TurnLeft(ms) 的区别：这个是【事件驱动】的 ——
 *         转到位置就停，电池快慢、地面打滑都不影响落点。
 *         用的还是 BLACK 事件（全黑持续 80ms 才算一次，之后要
 *         离开 400ms 才能再算），所以不会因为抖动提前停。
 *         转向期间【电机由本函数直接控制】，但每拍都调
 *         APP_Car_PollEvents() 让计数器继续走。
 *         ROUTE_TURN_EVT_TIMEOUT_MS(3500) 是安全网：转 3.5 秒还没全黑就放弃，
 *         免得车原地打转烧电机。
 */
#define ROUTE_TURN_EVT_TIMEOUT_MS   3500u   /* 转 3.5 秒还没全黑就放弃 */

void Route_TurnLeftUntilBlack(void)
{
    int16_t  t = g_param.pivot_spd;
    int16_t  l;
    int16_t  r;
    uint16_t base;
    uint32_t t0;

    route_begin_step("TURN-L >BLK", 0u);

    if (s_abort != 0u) {
        return;
    }

    /* ---- 先【武装】：把当前就压着的那根线丢掉，等的是【下一次】全黑 ---- */
    APP_Car_ArmOnNext((uint8_t)APP_CAR_EVT_BLACK);
    base = APP_Car_GetEventCount((uint8_t)APP_CAR_EVT_BLACK);

    /* 左转 = 左轮倒转、右轮正转（和 route_pivot 一致）*/
    l = (int16_t)(-t);
    r = t;
    if (g_param.steer_inv != 0u) {
        int16_t tmp = l;
        l = r;
        r = tmp;
    }

    BSP_Motor_SetSpeed(l, r);

    t0 = HAL_GetTick();
    while (s_abort == 0u) {
        route_tick(0u);                     /* 维持系统：按键/蜂鸣/屏幕/支撑脚 */
        APP_Car_PollEvents();               /* ★ 只采样+计数，不动电机 */

        if (APP_Car_GetEventCount((uint8_t)APP_CAR_EVT_BLACK) != base) {
            break;                          /* 全黑出现了 → 到位 */
        }
        if ((uint32_t)(HAL_GetTick() - t0) >= ROUTE_TURN_EVT_TIMEOUT_MS) {
            BSP_Buzzer_BeepAsync(400u);     /* 长音：没等到，放弃 */
            break;
        }
    }

    BSP_Motor_Brake();                      /* 到位立刻刹住 */
}

void Route_TurnLeft(uint16_t ms)
{
    Route_Pivot(-1, g_param.pivot_spd, ms, "TURN LEFT");
}

void Route_TurnRight(uint16_t ms)
{
    Route_Pivot(1, g_param.pivot_spd, ms, "TURN RIGHT");
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

    /* 找一个空槽登记。★ 记的是"绝对时刻"而不是"还要等多久"，
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

    /* ★ 清掉上一次跑剩下的预约舵机动作。
       如果上一次路线是在 step5（舵机四拍）被长按 K2 中止的，
       队列里可能还压着没执行的收放动作 —— 不清的话，
       这个"左右方向自检"跑到一半舵机会自己动一下，非常难查。 */
    {
        uint8_t q;

        for (q = 0u; q < ROUTE_SRVQ_MAX; q++) {
            s_srvq[q].used = 0u;
        }
    }

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

/**
  * @brief  ★ 路线结束后停在"结果页"，把当时的数据留在屏幕上
  * @param  aborted 1 = 是被长按 K2 中止的（标题显示 ABORTED）
  *
  * 【为什么要停一下】
  *   整条路线几十秒，跑完（或者某一步失误被中止）之后最想知道的是
  *   "它停在哪一步、三个计数器各数到几、当时探头看到什么"。
  *   以前是一结束就清屏回主菜单，这些信息全丢了，只能重跑一遍。
  *   现在把结果页留在屏幕上，按【任意键】才回菜单。
  *
  * ★ 这一页里电机是停着的，也不会调用 APP_Car_Update（不会突然动起来），
  *   但 MASK 是【实时】读的 —— 所以你可以用手推着车看掩码怎么变。
  */
static void route_hold_result(uint8_t aborted)
{
    const APP_CarStatus_t *st = APP_Car_GetStatus();
    uint32_t next = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();
        uint8_t  k;
        char     mstr[TRACK_SENSOR_NUM + 1u];
        char     rstr[TRACK_SENSOR_NUM + 1u];

        if ((uint32_t)(now - next) < CTRL_PERIOD_MS) {
            continue;                   /* 还没到下一拍（最多等 10ms）*/
        }
        next = now;

        BSP_Key_Scan();
        BSP_Buzzer_Poll();

        /* 任意按键 → 离开（读事件即清事件） */
        for (k = 0u; k < (uint8_t)KEY_ID_NUM; k++) {
            BSP_KeyEvent_t e = BSP_Key_GetEvent((BSP_KeyId_t)k);

            if ((e == KEY_EVENT_PRESS) || (e == KEY_EVENT_LONG)
                || (e == KEY_EVENT_REPEAT)) {
                return;
            }
        }

        /* 每拍重画一次（掩码/原始电平是实时的，方便用手推车观察）*/
        BSP_Track_MaskToString(BSP_Track_GetMask(), mstr);
        BSP_Track_MaskToString(BSP_Track_GetRawMask(), rstr);

        BSP_OLED_Clear();
        BSP_OLED_DrawString(0u, 0u, (aborted != 0u) ? "ROUTE ABORTED"
                                                   : "ROUTE DONE");
        BSP_OLED_Printf(0u, 1u, "STEP %u", (unsigned int)s_step_no);
        BSP_OLED_DrawString(0u, 2u, s_step_name);
        BSP_OLED_Printf(0u, 3u, "L%u R%u B%u",
                        (unsigned int)st->cnt_left3,
                        (unsigned int)st->cnt_right3,
                        (unsigned int)st->cnt_black);
        BSP_OLED_Printf(0u, 4u, "M%s", mstr);
        BSP_OLED_Printf(0u, 5u, "R%s", rstr);
        BSP_OLED_DrawString(0u, 7u, "any key: MENU");
        BSP_OLED_RefreshDirty();
    }
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

void APP_Route_ClearAbort(void)
{
    /* ★ 给菜单里的"手动动作"（比如 TURN TEST）用：
       如果上一次是在菜单里长按 K2 急停的，s_abort 会一直是 1，
       后面所有的原语都会立刻返回 —— 这里清一下才好继续用。 */
    s_abort = 0u;
}

/* ==========================================================================
 * ★★★  路 线 脚 本 —— 改路线只改这一个函数  ★★★
 * --------------------------------------------------------------------------
 *   每一行是一个动作，按从上到下的顺序执行。
 *
 *   ★★ 思路：用"循迹结果出现了几次"来推进，不用死时间 ★★
 *
 *   三种"循迹结果"各有一个计数器（在 app_car 里，掩码见 bsp_config.h）：
 *       LEFT3   最左三路全压线 —— 线跑到车头左边去了
 *       RIGHT3  最右三路全压线
 *       BLACK   全黑（八路全压）—— 横线 / 十字 / 台阶
 *
 *   计数【从本函数开头清零之后累计】，所以脚本里写的是"绝对第几次"：
 *   第 1 根横线、第 3 根横线…… 要改就改这些数字。
 *
 *   ★★★ 盲走 / 直走 / 转弯期间不计数 ★★★
 *     那几段车是闭着眼睛（或原地转）走的，压到横线也不算数。
 *     所以"第几根横线"= "循迹时压到的第几根"。
 *     ⇒ 改了任何一段盲走的起点，后面所有编号都要跟着重算
 *       （以及下面那张预置表）。当前这条路线就是按这个规则排的：
 *          盲走在第 3 根开始 → 后面依次是第 4、5 根，台阶在第 7 根。
 *
 *   只有三种动作用到时间（都在菜单 ROUTE TIME 页里可调）：
 *       BLD*_ms   盲走：闭着眼睛直冲，用来过地形
 *       STR_ms    直走：同上，用在"左三压线"之后
 *       POST_ms   过线后再循迹一小段，把车身摆正了再转
 *       TN*_ms    转弯：原地左转，角度靠时间标定（现在都是 1000）
 *       UPST_ms   上台阶时盲走的总时长（默认 14 秒）
 *
 *   ★ 好处：循线段不再随电池电压漂移 —— 慢一点就晚一点到，
 *     "第几次看到横线"永远对得上；标定一次就不用再管电量。
 *
 *   ★★★ 关于"走路时间"和车速的关系（2024 提速后统一乘了 0.7）★★★
 *     车速 BASESPD：1950 → 2800（×1.44）。车速变快之后，
 *     "同样的时间"会跑出"1.44 倍的距离"，所以路线里所有【走路】的
 *     时间都按 1950/2800 ≈ 0.7 缩了一遍，保证走的路程和以前一样。
 *     涉及：BLD1/BLD2/UPST/POST/POST2/POST3/STR/LF7/LF8
 *        + step5~8 段内写死的那些秒数（S5_xxx / S6_xxx / S7_xxx / S8_xxx）。
 *     ★ 转弯时间（TN1~TN7）【没动】：转弯用的是独立的 PIVOT 速度，
 *       和 BASESPD 无关，所以转同样的角度还是要同样的时间。
 *     ★ 这些都是按比例算的理论值，实车跑起来还要在菜单里微调 ——
 *       宁可先偏短（跑不够可以调大），不要偏长（冲过头不好救）。
 * --------------------------------------------------------------------------
 *   跑的时候凭声音判断走到哪了：
 *       每过一段        鸣两声（阶段标记）
 *       每数到一个事件  嘀一声
 *       等到目标事件    嘀一声（短）
 *       一直等不到      嘀——（长，60 秒安全网）
 *       整条结束        一声长音，然后停在"结果页"上
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * ★ 各阶段的"计数器预置值"
 * --------------------------------------------------------------------------
 * 脚本里等的是【绝对第几根横线】（第 3 根、第 4 根……），
 * 所以从中间某一步开始跑时，必须把计数器先垫成"前面那些步骤跑完之后
 * 应该有的数"，否则它会一直等一个永远不会来的编号。
 *
 * s_step_preset_black[n] = 从第 n 步开始跑时，BLACK 计数器预置成多少
 *   第 1 步：0  （什么都没数过）
 *   第 2 步：0  （它自己等第 1 根）
 *   第 3 步：1  （第 1 根已经数过了 —— step3 自己等第 3 根）
 *   第 4 步：3  （第 3 根已经数过了 —— step4① 自己等第 4 根）
 *   第 5 步：5  （第 5 根已经数过了 —— step5 自己等第 6 根）
 *   第 6 步：6  （第 6 根已经数过了 —— step6 自己等第 7 根）
 *   第 7 步：7  （第 7 根已经数过了 —— step7 不等横线，等【左路有线】）
 *   第 8 步：9  （step7 结尾已经等了第 9 根 —— step8 不再等横线）
 *   ★ 第 6 步特殊：它要等【第 7 根】和【第 8 根】两根，
 *     所以预置 6 之后，先数到 7（不转向），再数到 8 才左转。
 *
 * ★ 改了路线里等的编号（比如把 UntilBlack(5u) 改成 6u），
 *   这张表要跟着改，否则"从中间开始跑"会对不上。
 * ★ LEFT3 不用预置：step1 等它的第 1 次（只有从头跑才用），
 *   step4 里用的是"下一次"（相对），不依赖前面的次数。
 * -------------------------------------------------------------------------- */
#define ROUTE_STEP_NUM      8u

static const uint16_t s_step_preset_black[ROUTE_STEP_NUM + 1u] = {
    0u,     /* [0] 占位，不用 */
    0u,     /* 从第 1 步开始 */
    0u,     /* 从第 2 步开始 */
    1u,     /* 从第 3 步开始：第 1 根已过，本步等第 3 根 */
    3u,     /* 从第 4 步开始：第 3 根已过，本步等第 4 根 */
    5u,     /* 从第 5 步开始：第 5 根已过，本步等第 6 根 */
    6u,     /* 从第 6 步开始：第 6 根已过，本步等第 7 根 */
    8u,     /* 从第 7 步开始：step6 已经数到【第 8 根】了
               （所以预置 8 —— step7 最后要等的是第 9 根）*/
    9u      /* 从第 8 步开始：第 9 根已过（step7 结尾等的就是它），
               本步不再等横线，直接开始循迹 5 秒 */
};

/** 第 n 步的短名字（给"从哪一步开始"的菜单页显示用） */
const char *APP_Route_StepName(uint8_t step)
{
    switch (step) {
    case 1u: return "start -> blind";
    case 2u: return "line1 -> left";
    case 3u: return "line3 -> blind";
    case 4u: return "line4/5 -> left";
    case 5u: return "stairs + servo";
    case 6u: return "line7/8 -> left";
    case 7u: return "uturn + servo";
    case 8u: return "right + blind";
    default: return "----";
    }
}

uint8_t APP_Route_StepCount(void)
{
    return (uint8_t)ROUTE_STEP_NUM;
}

void APP_Route_RunFrom(uint8_t step)
{
    /* 夹到合法范围：菜单传进来的值可能越界（比如参数被改坏） */
    if (step < 1u) {
        step = 1u;
    }
    if (step > (uint8_t)ROUTE_STEP_NUM) {
        step = (uint8_t)ROUTE_STEP_NUM;
    }
    s_from = step;

    APP_Route_Run();
}

void APP_Route_Run(void)
{
    /* ---- 进入路线模式：先把状态清干净 ---- */
    s_active  = 1u;
    s_abort   = 0u;
    s_step_no = 0u;
    s_pump_tick = HAL_GetTick();
    s_draw_tick = HAL_GetTick();

    /* ★ 三个循迹结果计数器从 0 开始：
       脚本里写的是"第 1 根横线、第 2 根横线"，每一趟都必须重新数，
       否则第二趟会把上一趟的数带进来，判断全乱。
       计数器在待机时也在更新（方便推着车标定），所以这一步不能省。 */
    APP_Car_ResetEventCounts();

    /* ★ 从中间某一步开始时，把 BLACK 计数器垫到"前面步骤跑完后的值"，
       否则它会一直等一个永远不会来的编号（见上面的预置表）。
       从头跑（s_from = 1）时预置值就是 0，等于没做。 */
    {
        uint16_t preset = s_step_preset_black[s_from];

        APP_Car_SetEventCount((uint8_t)APP_CAR_EVT_BLACK, preset);
    }

    /* 清掉上一次跑剩下的预约舵机动作，免得这次一启动就乱摆 */
    {
        uint8_t q;

        for (q = 0u; q < ROUTE_SRVQ_MAX; q++) {
            s_srvq[q].used = 0u;
        }
    }

    APP_Car_Stop();
    BSP_Motor_Stop();

    BSP_UART_SendString("\r\n[route] start\r\n");

    /* ======================= ↓↓↓ 路线从这里开始 ↓↓↓ ======================= */

    /* ===================== step1：起步 → 盲走 ============================
     * 发车后正常循迹，等"最左三路全压线"出现第 1 次 ——
     * 线跑到车头左边去了，说明到了该盲走的位置。
     * 然后闭着眼睛直冲 BLD1_ms 过掉这一段。
     *
     * ★ 每一段外面那层 if (s_from <= N) 就是"从第几步开始跑"的实现：
     *   s_from = 1 时全部执行（正常）；菜单里选从第 3 步开始，
     *   前两段整段跳过，直接从 step3 往下跑。 */
    if (s_from <= 1u) {
    Route_LineFollowUntilLeft3(1u);
    Route_Straight(g_param.rt_bld1_ms);     /* ★ 盲走（菜单 ROUTEUTE TIME 里调）*/
    Route_Beep2();
    }

    /* ============ step2：第 1 根横线 → 再循迹一段 → 左转 ================
     * 回到循迹模式，一直循到第 1 根横线（八路全压）。
     *
     * ★ 压到横线的那一刻【不能马上转】：车架大，这时候车身压在横线上、
     *   还没摆正，原地转会转歪。
     *   所以再正常循迹 POST_ms（默认 1.5 秒）——
     *   这段时间循迹环会把车纠到"线在探头正中间"，也就是转弯的最佳起点，
     *   再原地左转，落点才准。
     *
     * ★ 想让车走得更靠前一点再转，就把 POST_ms 调大（菜单 ROUTE TIME 页）。
     *   ★ 三条"横线后接左转"现在都加了这一小段（step2 / step4① / step4③），
     *     用的是同一个 POST_ms —— 调一个数三条一起变，省得分别标定。
     *     （step4② 那次是"左三压线"触发，用的是盲走 STR_ms，见那一节）*/
    if (s_from <= 2u) {
    Route_LineFollowUntilBlack(1u);
    Route_LineFollow(g_param.rt_post_ms);   /* ★ 过线后再循迹 1.5 秒，走到线中心 */
    Route_TurnLeft(g_param.rt_tn1_ms);
    Route_Beep2();
    }

    /* ===================== step3：第 3 根横线 → 盲走 =====================
     * ★ 注意这里等的是【第 3 根】：
     *   中间第 2 根横线会正常循迹开过去、不做任何动作，
     *   计数器照样在数，所以"等到第 3 根"就是"开过第 2 根之后"。
     *   ★ 跑的时候听声音：每过一根横线会"嘀"一声，
     *     听到第 3 声之后就该盲走了。
     *   ★★ 这一改，后面所有"第几根"都要跟着 -1 ★★
     *     因为盲走段（Route_Straight）是闭着眼睛走的，
     *     期间压到的横线【不计数】—— 盲走提前一根开始，
     *     后面数到的编号整体就往前挪一根（见下面的预置表）。 */
    if (s_from <= 3u) {
    Route_LineFollowUntilBlack(3u);
    Route_Straight(g_param.rt_bld2_ms);     /* ★ 盲走（默认 5.5 秒）*/
    Route_Beep2();
    }

    /* ============== step4：第 4 根横线 → 左转 → 左三 → 直走 → 左转
     *                          → 第 5 根横线 → 左转 ==========================
     * 盲走结束回到循迹模式：
     *     ① 循到第 4 根横线                → 再循迹 POST_ms → 左转
     *     ② 再次循迹，检测到最左三路全压线  → 盲走 STR_ms   → 左转
     *     ③ 回到循迹模式，循到第 5 根横线   → 再循迹 POST2_ms → 左转
     *     → 路线结束
     *
     * ★ ①③ 这两段"过线后再走一小段"是实测出来的：车架大，压到横线的那一刻
     *   车身还没摆正，直接原地转会转歪。多走这一小段，循迹环把车纠到
     *   "线在探头正中间"，再转落点才准。
     *   ★ ① 和 step2 共用 POST_ms（默认 1.5 秒）；
     *     ③ 用 POST2_ms（默认 1 秒）—— 这一处实测只要 1 秒。
     *
     * ★ 第 ② 步用的是 Route_LineFollowUntilNextLeft3()（等"下一次"左三压线），
     *   而不是写死"第 2 次"：左三压线在一趟里可能已经出现过好几次
     *   （线偏到左边就会触发），写死编号很容易对不上。
     *   "下一次"永远是对的。
     * ★ 第 ② 步的"直走"用 Route_Straight()：这时候不看线、两轮同速直冲，
     *   和 step1 的盲走是同一个原语，时长在菜单 ROUTE TIME 的 STR_ms 里。 */
    if (s_from <= 4u) {
    Route_LineFollowUntilBlack(4u);
    Route_LineFollow(g_param.rt_post_ms);   /* ★ 过线后再循迹一小段，把车身摆正 */
    Route_TurnLeft(g_param.rt_tn2_ms);
    Route_Beep2();

    Route_LineFollowUntilNextLeft3();       /* 再次循迹，等下一次"左三压线" */
    Route_Straight(g_param.rt_str_ms);      /* ★ 直走 1 秒（STR_ms）*/
    Route_TurnLeft(g_param.rt_tn4_ms);
    Route_Beep2();

    Route_LineFollowUntilBlack(5u);
    Route_LineFollow(g_param.rt_post2_ms);  /* ★ 这一处只要 1 秒（POST2_ms）*/
    Route_TurnLeft(g_param.rt_tn3_ms);
    Route_Beep2();
    }

    /* ================== step5：上三阶台阶（全程不停顿地盲走）===============
     *
     * ★ 台阶是【全黑】的 —— 在探头看来和"横线"是同一种图案，
     *   所以它同样会被 BLACK 计数器数到（连续全黑只算一次）。
     *   实测：数到【第 6 根】的时候，车刚好停在台阶前。
     *
     * 时间轴（t = 0 就是"检测到第 6 根线"的那一刻）：
     *
     *      t = 0s       车开始【盲走】—— 从这里到 t=9s，车不停、不循线、直着走
     *      t = 1.4s     支撑脚放下   （撑住台阶）
     *      t = 2.8s     支撑脚升起   （放下了 1.4 秒）
     *      t = 4.2s     支撑脚再放下 （升起了 1.4 秒；★ 这一次只放到 130°）
     *      t = 6.3s     支撑脚完全升起（放下了 2.1 秒）
     *      t = 9s       盲走结束 → 阶段标记两声 → 接 step6
     *
     *   ★ 舵机的四个时间点就是上面这 1.4 / 2.8 / 4.2 / 6.3 秒，
     *     想让节奏变快变慢，改下面那四个 #define 即可（它们只影响台阶这一段）。
     *
     * ★★ 关键：舵机的四个动作是【预约】出去的（Route_ServoAt），
     *    调用它们只花几微秒就返回，真正的动作由后台节拍按时间点执行 ——
     *    所以车从 t=0 到 t=9s 一直在盲走，中间【一次都不停】。
     *    如果用 Route_Servo()（走过去再等它到位），车就会在每一拍停一下，
     *    那是爬不上台阶的。
     *
     * ★ 总盲走时长 = 菜单 ROUTE TIME 页的 UPST_ms（默认 9000 = 9 秒）。
     *   舵机的四个时间点是写死的（上面那四个 #define）——
     *   所以调 UPST_ms 只改"走多久"，不会动舵机节奏。 */
#define S5_DOWN1_MS     1400u       /* t=1.4s 放下（角度 = 菜单 LEGDN）*/
#define S5_UP1_MS       2800u       /* t=2.8s 升起（放下了 1.4 秒）*/
#define S5_DOWN2_MS     4200u       /* t=4.2s 再放下（升起了 1.4 秒）*/
#define S5_UP2_MS       6300u       /* t=6.3s 完全升起（放下了 2.1 秒）*/
/* ★★ 第 2 次【放下】的角度：不走菜单里的 LEGDN，单独用这个值 ——
   台阶第二下只需要撑到 130°，撑满 180° 反而会把车头顶起来。
   想改就改这一个数（0~180）。 */
#define S5_DOWN2_DEG     130u       /* t=4.2s 那次放下的角度 */

    if (s_from <= 5u) {
    Route_LineFollowUntilBlack(6u);         /* 循到第 6 根（台阶前）*/

    /* ---- 先把四个舵机动作预约好（立刻返回，车这就开始走）----
       角度：第 1 次放下 / 两次升起都用菜单标定过的 LEGDN / LEGUP，
             只有第 2 次放下用写死的 S5_DOWN2_DEG（130°）*/
    Route_ServoAt((uint8_t)g_param.leg_down_deg, S5_DOWN1_MS);
    Route_ServoAt((uint8_t)g_param.leg_up_deg,   S5_UP1_MS);
    Route_ServoAt((uint8_t)S5_DOWN2_DEG,         S5_DOWN2_MS);  /* ★ 130° */
    Route_ServoAt((uint8_t)g_param.leg_up_deg,   S5_UP2_MS);

    /* ---- 然后一直盲走到时间用完 ----
       这期间车不停：舵机的四次收放由后台节拍按上面的时间点执行 */
    Route_Straight(g_param.rt_upst_ms);

    Route_Beep2();
    }

    /* ==== step6：第 7 根线【不转向】→ 循迹 1.4s → 盲走 3.5s（舵机收放）
     *              → 循到第 8 根线 → 循迹 0.7s → 左转 1s ===============
     *
     * ★★ 注意：第 7 根线【只是个信号，不转向】★★
     *   扫到第 7 根线之后车先循迹走一小段，然后切成【盲走】3.5 秒，
     *   盲走期间把支撑脚放下再升回（过障碍）；盲走结束后切回循迹，
     *   一直循到【第 8 根线】才左转。
     *
     * 时间轴（t = 0 就是"扫到第 7 根线"的那一刻）：
     *
     *      t = 0s            第 7 根线（只是触发点，不转向）
     *      t = 0 ~ 1.4s      循迹 1.4 秒（走到障碍跟前）
     *      t = 1.4s          ★ 切【盲走】3.5 秒
     *      t = 2.0s          ★ 支撑脚放下
     *      t = 3.0s          ★ 支撑脚升回（放下 1 秒）
     *      t = 4.9s          盲走结束 → 切回【循迹】，直到扫到【第 8 根线】
     *      t = 第8根线       再循迹 POST3_ms(0.7 秒) 把车身摆正
     *      然后              左转 TN5_ms(1 秒)
     *
     *   ★ 时间点全部写死在下面（想改节奏就改这三个数）：
     *       S6_DOWN_DELAY_MS = 测到第 7 根线之后【先循迹走多久】才切盲走，
     *                          也就是"障碍离触发线有多远"。
     *       S6_BLIND_MS      = 盲走走多久。
     *       S6_DOWN_AT_MS / S6_UP_AT_MS = 从第 7 根线算起，脚什么时候放下/收回。
     *   ★ 盲走期间【不计数】—— 这一段压到的横线不算，
     *     所以后面等的"第 8 根线"是盲走结束后重新开始数的那一根。 */
#define S6_DOWN_DELAY_MS  1400u     /* ★ 测到第 7 根线后，先循迹走 1.4 秒 */
#define S6_BLIND_MS       3500u     /* ★ 然后盲走 3.5 秒（舵机在这一段里收放）*/
/* ★★ 舵机的两个时间点：从【第 7 根线】算起的绝对时刻（不是从盲走开始算）
   —— 上面已经先循迹走了 S6_DOWN_DELAY_MS(1.4s)，所以预约时要减掉它。
   实车要挪动放脚位置就改这两个数：改小 = 更早放脚。 */
#define S6_DOWN_AT_MS     2000u     /* t=2.0s 放下 */
#define S6_UP_AT_MS       3000u     /* t=3.0s 升回（脚放下 1 秒）*/
#define S6_SRV_DOWN_MS   (S6_DOWN_AT_MS - S6_DOWN_DELAY_MS)   /* = 600，相对盲走开始 */
#define S6_SRV_UP_MS     (S6_UP_AT_MS   - S6_DOWN_DELAY_MS)   /* = 1600 */

    if (s_from <= 6u) {
    Route_LineFollowUntilBlack(7u);         /* 扫到第 7 根线（只是触发，不转向）*/

    /* ---- 先循迹 1.4 秒（走到障碍跟前）---- */
    Route_LineFollow(S6_DOWN_DELAY_MS);

    /* ---- 预约两个舵机动作（立刻返回，车接着走）----
       盲走开始后 0.6 秒放下、1.6 秒升回 —— 也就是从第 7 根线算起 t=2s / t=3s */
    Route_ServoAt((uint8_t)g_param.leg_down_deg, S6_SRV_DOWN_MS);
    Route_ServoAt((uint8_t)g_param.leg_up_deg,   S6_SRV_UP_MS);

    /* ---- ★ 盲走 3.5 秒（舵机在这一段里收放）---- */
    Route_Straight(S6_BLIND_MS);

    /* ---- 切回循迹，循到【第 8 根线】，再循迹一小段把车身摆正，然后左转 ---- */
    Route_LineFollowUntilBlack(8u);
    Route_LineFollow(g_param.rt_post3_ms);  /* 过线后再循迹（默认 0.7 秒）*/
    Route_TurnLeft(g_param.rt_tn5_ms);      /* 左转 1 秒 */
    Route_Beep2();
    }

    /* ==== step7：循迹1s → 盲走2s(舵机收放) → 左路有线 → 掉头
     *              → 循迹1s → 盲走5s(舵机收放) → 循到第 9 根线 ==============
     *
     * ★★ 这一段的规律：每到"舵机收放"就切成【盲走】，其余时间都循迹 ★★
     *   舵机动作全部用"预约"（Route_ServoAt），登记完就返回 ——
     *   所以盲走/循迹都在继续走，不会停下来等舵机。
     *
     * 时间轴（两段盲走都是 S7_BLIND_MS = 3.8 秒 ——
     *         第 1 段和【掉头回来】那段用的是同一个数；
     *         舵机【放下的时间点不变】—— 还是各自盲走一开始就放，
     *         放 1.4 秒后升回，所以升回之后车还会继续盲走 2.4 秒）：
     *
     *    t = 0s               step6 左转转完，重新开始循迹
     *    t = 0 ~ 0.7s         循迹 0.7 秒（时长 = LF7_ms）
     *    t = 0.7s             ★ 切【盲走】3.8 秒，同时支撑脚放下（第 1 次）
     *    t = 2.1s             支撑脚升回
     *    t = 4.5s             盲走结束 → ★ 切回【循迹】，直到【左路探头压到线】
     *    ──────────────────────────────────────────────
     *    接着                 ★ 掉头（原地转 180°，TN6_ms）
     *    t = 0 ~ 0.7s         掉完头先【循迹】0.7 秒（S7C_LF_MS）
     *    t = 0.7s             ★ 切【盲走】3.8 秒，同时支撑脚放下（第 2 次）
     *    t = 2.1s             支撑脚升回
     *    t = 4.5s             盲走结束 → ★ 切回【循迹】，
     *                         循到【第 9 根线】→ 急停，路线结束
     *
     *   ★ 参数：
     *       LF7_ms（菜单 ROUTE TIME）  第 1 段先循迹多久
     *                                  （放脚的时刻 = 这段循迹结束、盲走开始的那一刻）
     *       TN6_ms（菜单 ROUTE TIME）  掉头要转多久。掉头 = 180°，
     *                                  按"90° ≈ 1000ms"估的默认 2000 ——
     *                                  ★ 实测转不够/转过头就改它；
     *                                    方向反了就把 Route_TurnLeft 换成 TurnRight
     *   ★ 其余时间点写死在下面（S7_BLIND_MS / S7_DOWN_KEEP_MS / S7C_LF_MS）。
     *   ★ "左路有线"用 Route_LineFollowUntilNextLeft3() —— 等【下一次】左三压线，
     *     不写死"第几次"（左三在一趟里可能已经出现过好几次）。
     *   ★ 盲走期间【不计数】—— 所以最后等的"第 9 根线"是
     *     第 2 段盲走结束后重新开始数的那一根。 */
#define S7_BLIND_MS      3800u      /* ★ 两段盲走都是 3.8 秒（第 1 段 + 掉头回来那段）*/
#define S7_DOWN_KEEP_MS  1400u      /* 脚放下之后 1.4 秒升回（两段时间点一样，
                                       放下的时刻 = 各自盲走开始那一刻）*/
#define S7C_LF_MS         700u      /* 掉完头先循迹 0.7 秒 */

    if (s_from <= 7u) {
    /* ---- 第 1 段：循迹 1 秒 → 盲走 2 秒（第 1 次舵机收放在盲走里完成）---- */
    Route_LineFollow(g_param.rt_lf7_ms);        /* 先循迹 1 秒 */

    Route_ServoAt((uint8_t)g_param.leg_down_deg, 0u);             /* 盲走一开始就放下 */
    Route_ServoAt((uint8_t)g_param.leg_up_deg, S7_DOWN_KEEP_MS);  /* 2 秒后升回 */

    Route_Straight(S7_BLIND_MS);                /* ★ 盲走 5.5 秒（后 3.5 秒脚已收回）*/

    /* ---- 切回循迹，直到左路探头压到线 ---- */
    Route_LineFollowUntilNextLeft3();

    /* ---- ★ 掉头：原地转 180°（用 TN6_ms 控制转多少）---- */
    Route_TurnLeft(g_param.rt_tn6_ms);
    Route_Beep2();                          /* 掉头是一个阶段，报一下 */

    /* ---- 掉完头：先【循迹】1 秒 ---- */
    Route_LineFollow(S7C_LF_MS);            /* ★ 新增：掉完头先循迹 1 秒 */

    /* ---- 再【盲走】5 秒，期间放第 2 次脚 ----
       ★ 盲走期间【不计数】—— 这一段压到的横线不算，
         所以后面等的"第 9 根线"是盲走结束后重新开始数的那一根。 */
    Route_ServoAt((uint8_t)g_param.leg_down_deg, 0u);             /* 盲走一开始就放下 */
    Route_ServoAt((uint8_t)g_param.leg_up_deg, S7_DOWN_KEEP_MS);  /* 2 秒后升回 */
    Route_Straight(S7_BLIND_MS);            /* ★ 盲走 5.5 秒（后 3.5 秒脚已收回）*/

    /* ---- 盲走结束 → 回到循迹模式，循到【第 9 根线】 ----
       （Route_LineFollowUntilBlack 内部会调 APP_Car_Start()，
         所以这一行本身就是切换回循迹模式）
       ★ 这里【不再急停】—— 第 9 根线之后直接接 step8 继续跑。 */
    Route_LineFollowUntilBlack(9u);

    Route_Beep2();
    }

    /* ============ step8：第 9 根线之后 → 循迹 3s → 右转 100°
     *              → 盲走 3.5s（放脚）→ 倒车 3s → 左转 120° → 盲走 2s
     *              → 切回循迹 → 第 10 根线 → 急停，路线结束 ===========
     *
     * 时间轴（t = 0 就是 step7 循到第 9 根线的那一刻）：
     *
     *    t = 0 ~ 3s           循迹（时长 LF8_ms，默认 3000）
     *    接着                 ★ 右转 100°（TN8_ms，默认 1100 ≈ 1.1 秒）
     *    ──────────── 下面 t 都从"右转转完"重新算 ────────────
     *    t = 0s               ★ 开始【盲走】3.5 秒（期间收放支撑脚）
     *    t = 1.4s             支撑脚放下
     *    t = 2.8s             支撑脚升回（放下 1.4 秒）
     *    t = 3.5s             盲走结束
     *    接着                 ★ 【倒车】3 秒（S8_REV_MS）
     *    接着                 ★ 【左转】120°（TN7_ms ≈ 1.3 秒）
     *    接着                 ★ 再【盲走】2 秒（S8_BLIND3_MS）
     *    接着                 ★ 切回【循迹模式】，直到【第 10 根线】
     *    第10根线             → 急停，整条路线结束
     *
     *   ★ 两段盲走：右转后 S8_BLIND1_MS = 3.5 秒（这一段的 1.4s/2.8s 收放脚），
     *              左转 120° 之后再盲走 S8_BLIND3_MS = 2 秒。
     *   ★ 倒车用 Route_Reverse()：两轮同速反转，走完自动停；
     *     倒车速度 = 基础速度 × ROUTE_REV_RATIO(0.7) —— 倒车不敢太快。
     *   ★ 转弯参数：右转 100° 用 TN8_ms，倒车后的左转 120° 用 TN7_ms。
     *   ★ 去掉了"转到全黑"那版（Route_TurnLeftUntilBlack 保留在文件里备用）。
     *   ★ 盲走/倒车期间【不计数】—— 所以最后等的"第 10 根线"是
     *     切回循迹之后重新开始数的那一根。 */
#define S8_DOWN_MS       1400u      /* 盲走开始后第 1.4 秒：放下 */
#define S8_UP_MS         2800u      /* 第 2.8 秒：升回（放下 1.4 秒）*/
#define S8_BLIND1_MS     3500u      /* 右转后的盲走 3.5 秒（期间收放脚）*/
#define S8_BLIND3_MS     2000u      /* ★ 左转 105° 之后的盲走 2 秒 */
#define S8_REV_MS        3000u      /* ★ 倒车 3 秒（速度 = 基础速度 × 0.7）*/

    if (s_from <= 8u) {
    Route_LineFollow(g_param.rt_lf8_ms);    /* 循迹（默认 3 秒）*/

    Route_TurnRight(g_param.rt_tn8_ms);     /* ★ 第 1 次右转 100°（默认 1.1 秒）*/
    Route_Beep2();

    /* ---- 盲走第 1 段（5 秒），期间舵机放下再升回 ---- */
    Route_ServoAt((uint8_t)g_param.leg_down_deg, S8_DOWN_MS);   /* t=2s 放下 */
    Route_ServoAt((uint8_t)g_param.leg_up_deg,   S8_UP_MS);     /* t=4s 升回 */
    Route_Straight(S8_BLIND1_MS);           /* 盲走 5 秒 */

    /* ---- ★ 倒车 3 秒（车尾往后退一段）---- */
    Route_Reverse(S8_REV_MS);
    Route_Beep2();

    /* ---- ★ 左转 120°（定时）----
       ★ 2024 决定：这一步还是用定时转 ——
         Route_TurnLeftUntilBlack()（转到全黑为止）那版留在本文件里没删，
         以后想改成事件驱动，把这一行换回去即可。 */
    Route_TurnLeft(g_param.rt_tn7_ms);      /* ★ 左转 120°（默认 1.3 秒）*/
    Route_Beep2();

    /* ---- ★ 再盲走 2 秒（转完之后再直冲一段）---- */
    Route_Straight(S8_BLIND3_MS);

    /* ---- ★ 切回【循迹模式】，直到【第 10 根线】→ 急停，路线结束 ----
       （Route_LineFollowUntilBlack 内部会调 APP_Car_Start()，
         所以这一行本身就是"切回循迹模式"）*/
    Route_LineFollowUntilBlack(10u);
    Route_Brake();                          /* ★ 第 10 根线到了，立刻刹住 */

    Route_Beep2();
    }

    /* 路线结束 */
    Route_Beep(400u);                   /* 结尾用一声长音，和中间的"两声"区分开 */

    /* ======================= ↑↑↑ 路线到这里结束 ↑↑↑ ======================= */

    /* ---- 收尾：不管怎么结束的，都必须把车停下来 ---- */
    uint8_t was_aborted = s_abort;      /* ★ 先记下来：下面 APP_Route_Abort() 会把它置 1 */

    BSP_Motor_Brake();
    APP_Car_Stop();
    APP_Route_Abort();          /* 让后续的原语也立刻返回（中止时用） */
    s_active = 0u;

    /* ★★ 把"从第几步开始"复位成 1 ★★
     *   s_from 是静态变量，APP_Route_RunFrom() 会把它设成 3、4、5。
     *   如果这里不复位，那"从第 4 步跑过一次"之后，
     *   再从菜单按 RUN ROUTE、或下次开机按 K1 跑，
     *   都会【悄悄从第 4 步开始】—— 前半条路线被跳过。
     *   放在收尾处复位：这一趟跑完之后，一切回到"从头跑"。 */
    s_from = 1u;

    BSP_UART_SendString("[route] done\r\n");

    /* ★★ 停在结果页，不直接回主菜单 ★★
     *   把"停在第几步、三个计数器各是多少、探头现在看到什么"留在屏幕上，
     *   按任意键才回菜单（详见 route_hold_result 的说明）。
     *   中止（长按 K2）也走这里，只是标题变成 ABORTED。 */
    route_hold_result(was_aborted);

    /* 按键之后：清屏，把屏幕交回菜单 */
    BSP_OLED_Clear();
    BSP_OLED_MarkAllDirty();
    BSP_OLED_Refresh();
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

    /* ★ 把四个"方向类"开关直接摆在这里。
       它们错了的后果最严重（车倒着冲、或者朝远离线的方向拐），
       而其它参数的错只是"跑得不好"。开机就能看到，就不用每次
       烧完固件再去菜单里翻一遍了。
       顺序： MLINV / MRINV / STEERINV / ACTLOW
       本车正确的值： 1 / 0 / 0 / 0 */
    BSP_OLED_Printf(0u, 6u, "DIR %u/%u/%u/%u",
                    (unsigned int)g_param.motor_l_inv,
                    (unsigned int)g_param.motor_r_inv,
                    (unsigned int)g_param.steer_inv,
                    (unsigned int)g_param.act_low);
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
