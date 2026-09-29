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

    /* ---- ③ 屏幕：让操作的人随时知道跑到第几步了 ---- */
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
 *   标定前的示例路线（照着改就行）：
 * ========================================================================== */
void APP_Route_Run(void)
{
    /* ---- 进入路线模式：先把状态清干净 ---- */
    s_active  = 1u;
    s_abort   = 0u;
    s_step_no = 0u;
    s_pump_tick = HAL_GetTick();
    s_draw_tick = HAL_GetTick();

    APP_Car_Stop();
    BSP_Motor_Stop();

    BSP_UART_SendString("\r\n[route] start\r\n");

    /* ======================= ↓↓↓ 路线从这里开始 ↓↓↓ ======================= */

    /* ① 沿主线循迹，直到经过第 1 根横线（路口/停止线） */
    Route_LineFollowUntilCross(1);

    /* ② 原地右转（900ms 大约 90°，必须实车标定） */
    Route_TurnRight(900);

    /* ③ 直行一小段（速度 = BASESPD） */
    Route_Straight(600);

    /* ④ 放下支撑脚到 180 度 */
    Route_Servo(180);

    /* ⑤ 停 1 秒（等机构稳定下来） */
    Route_Stop(1000);

    /* ⑥ 收起支撑脚 */
    Route_Servo(0);

    /* ⑦ 继续循迹 3 秒 */
    Route_LineFollow(3000);

    /* ⑧ 提示音：整条路线跑完了 */
    Route_Beep(300);

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
