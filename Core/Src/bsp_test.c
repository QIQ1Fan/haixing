/**
  ******************************************************************************
  * @file    bsp_test.c
  * @brief   串口自检命令行（★ 已精简：只留最常用的几条）
  *
  * 【数据流】
  *   电脑串口助手 ──敲一个字符──► USART1
  *     └─► main.c 主循环轮询 BSP_UART_GetChar() 收到
  *           └─► BSP_Test_RunCommand(字符)
  *                 └─► switch 分发到对应模块的测试函数
  *                       └─► 测试函数用串口打印结果 / 操作硬件
  *                             └─► 返回 → 重新打印菜单
  *
  * 【为什么只剩这几条】
  *   调参、保存、恢复默认、支撑脚、探头查看、启停、状态显示……
  *   这些 OLED 菜单里全都有，而且不用开电脑。串口这边只保留
  *   "OLED 里没有、又必须能测"的几条（见 bsp_test.h 的清单）。
  *
  *   精简掉的还有"编码器读数"——本车编码器没接线，用不到。
  *
  * 【正式跑车时怎么处理】
  *   整份文件可以删掉，不影响其它模块：没有任何 bsp_* 模块反过来依赖它。
  *   只要把 main.c 里对 BSP_Test_* 的调用去掉即可。
  ******************************************************************************
  */
#include "bsp_test.h"

#include "bsp_motor.h"
#include "bsp_track.h"
#include "bsp_encoder.h"      /* ★ 编码器自检要用 ENC_LEFT / ENC_RIGHT */
#include "bsp_buzzer.h"
#include "bsp_uart.h"

/* 保留命令直接操作整车 / 参数，所以要包含应用层的头 */
#include "app_param.h"
#include "app_menu.h"
#include "app_route.h"

/* 测试用电机占空比，别用满速，桌面测试时车会飞出去 */
#define TEST_MOTOR_DUTY     1200        /* 满量程 3599，约 33% */

/* ==========================================================================
 * 串口菜单
 * ========================================================================== */

void BSP_Test_ShowMenu(void)
{
    BSP_UART_SendString("\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("   循迹小车 串口自检 (USART1 115200)\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("  4. 电机正反转      6. 编码器自检\r\n");
    BSP_UART_SendString("  7. 循迹传感器\r\n");
    BSP_UART_SendString("--------------------------------------------\r\n");
    BSP_UART_SendString("  V. 打印全部参数    T. 左右方向自检\r\n");
    BSP_UART_SendString("  G. 跑路线脚本      M. 回到 OLED 主界面\r\n");
    BSP_UART_SendString("  ?. 重新打印本菜单\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("请输入 4 / 6 / 7 / V / T / G / M / ? :\r\n");
    BSP_UART_SendString("(其余功能都在 OLED 菜单里: START CAR / TUNE PARAMS /\r\n");
    BSP_UART_SendString(" START STEP / ROUTE TIME / SUPPORT LEG / TURN TEST /)\r\n");
    BSP_UART_SendString(" SENSOR VIEW / SAVE PARAMS / LOAD DEFAULT)\r\n");
}

/**
  * @brief  判断字符是否为合法菜单命令
  * @note   ★ 这道校验是必须的，不是可有可无的保险。
  *         如果 RX 悬空（只接 TX 打印是很常见的接法），浮空的引脚会
  *         持续捡拾环境噪声、产生随机字节。若不加校验，每个噪声字节
  *         都会触发一次蜂鸣 + 一次测试分发，现象就是"一上电蜂鸣器乱响、
  *         舵机乱动"，而且完全看不出原因。
  *         RX 已在 BSP_UART_Init() 里改成上拉，这里是第二道防线。
  */
static uint8_t test_is_valid_cmd(char cmd)
{
    switch (cmd) {
    case '4': case '6': case '7':
    case 'V': case 'v':     /* 打印全部参数 */
    case 'T': case 't':     /* 左右方向自检 */
    case 'G': case 'g':     /* 跑路线脚本 */
    case 'M': case 'm':     /* 回到 OLED 主界面 */
    case '?':
        return 1u;
    default:
        return 0u;          /* 换行、回车、噪声一律静默丢弃 */
    }
}

/**
  * @brief  收到"不是命令"的字节时，把原因说清楚，而不是默默丢掉
  *
  * 【为什么要专门写这个函数】
  *   调试时最要命的就是"发了没反应"：
  *     · 输入法在全角状态打出「？」→ UTF-8 是三个字节 EF BC 9F，
  *       一个都不是 '?'，全被丢掉 → 现象和"串口没接好"一模一样
  *     · 命令敲错一个字母（比如把 V 敲成 B）→ 也是静默丢弃
  *   两种情况的排查方向完全不同，却给出同样的沉默。
  *   现在把收到的字节原样报出来，一眼就能分辨。
  *
  *   ★ 另外：精简之后，很多老命令（1/3/6/8/W/S/D/L/E…）已经不认了，
  *     敲它们会走到这里 —— 提示里会说明"这些功能在 OLED 菜单里"。
  */
static void test_report_bad_cmd(uint8_t c)
{
    /* 最高位为 1：一定是多字节字符（中文/全角标点）或乱码。
       全角「？」「，」这类字符一次会来 3 个字节，所以这里不刷菜单，
       否则一条命令能刷出三遍菜单，反而看不清 */
    if (c >= 0x80u) {
        BSP_UART_Printf("\r\n[uart] 收到非 ASCII 字节 0x%02X —— 输入法多半是全角。\r\n",
                        (unsigned int)c);
        BSP_UART_SendString("[uart] 请切到【半角】再敲 ?, 或用串口助手的 HEX 发送 3F\r\n");
        return;
    }

    /* 可打印 ASCII 但不在命令表里：多半是敲错了，直接给提示 + 菜单 */
    if ((c >= 0x20u) && (c <= 0x7Eu)) {
        BSP_UART_Printf("\r\n[uart] '%c' (0x%02X) 不是有效命令。\r\n",
                        (char)c, (unsigned int)c);
        BSP_UART_SendString("[uart] 提示: 调参/保存/支撑脚/探头查看等已在 OLED 菜单里，\r\n");
        BSP_UART_SendString("        串口这边只剩 4 / 7 / V / T / G / M / ?\r\n");
        return;
    }

    /* 其余（回车、换行、退格、串口助手自动附加的控制字符）静默忽略。
       ★ 必须忽略：很多助手每发一条命令都会自动补 "\r\n"，
         如果连它也要响应，每条命令后面都会多刷一遍菜单 */
}

void BSP_Test_RunCommand(char cmd)
{
    /* 先判定合法性，再决定要不要响、要不要执行 */
    if (test_is_valid_cmd(cmd) == 0u) {
        test_report_bad_cmd((uint8_t)cmd);
        return;
    }

    BSP_UART_Printf("\r\n>>> 执行测试 [%c]\r\n", cmd);

    /* ★ 这里用阻塞的 Beep() 而不是 BeepAsync()。
       因为下面很多测试本身是阻塞的（进去几秒到几十秒），主循环出不去，
       BSP_Buzzer_Poll() 根本不会被调用 —— 用非阻塞鸣叫的话，PB3 会
       一直保持"响"的状态直到整段测试结束。 */
    BSP_Buzzer_Beep(30u);

    switch (cmd) {
    case '4': BSP_Test_Motor();                    break;
    case '6': BSP_Test_Encoder();                  break;   /* 编码器自检 */
    case '7': BSP_Test_Track();                    break;
    case 'V': case 'v': BSP_Test_ParamShow();      break;
    case 'M': case 'm': BSP_Test_MenuShow();       break;
    case 'G': case 'g': APP_Route_Run();           break;   /* 跑路线脚本 */
    case 'T': case 't': APP_Route_TestTurn();      break;   /* 左右方向自检 */

    default:
        BSP_Test_ShowMenu();
        return;
    }

    BSP_UART_SendString("<<< 测试结束\r\n");
    BSP_Test_ShowMenu();
}

/* ==========================================================================
 * 4. 电机正反转
 * ========================================================================== */

/**
 * @brief  同一个占空比跑 2 秒，边跑边比两轮转速 —— 量"左右差异"用
 * @param  duty 占空比（带符号：正 = 前进、负 = 后退）
 * @param  tag  打印用的标签
 * @retval 两轮平均转速之差占较大的那个的百分比（正 = 左轮快）
 * @note   ★ 本函数是阻塞的，必须【自己】调 BSP_Encoder_Update()，
 *         否则 rpm 永远是 0（主循环的 10ms 节拍被占住了）。
 */
static float test_motor_run(int16_t duty, const char *tag)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t tp = 0u;
    float    sum_l = 0.0f;
    float    sum_r = 0.0f;
    uint16_t n     = 0u;
    float    avg_l;
    float    avg_r;
    float    big;

    BSP_UART_Printf("[电机] %s 2s (占空比 %d)\r\n", tag, (int)duty);
    BSP_UART_SendString("   t(ms)   rpm_L   rpm_R    差%\r\n");

    /* 起步那 300ms 先丢掉：有加速过程，算进去会把结论带偏 */
    BSP_Motor_SetSpeed(duty, duty);
    HAL_Delay(300u);
    BSP_Encoder_Reset();

    while ((uint32_t)(HAL_GetTick() - t0) < 2000u) {
        float rl;
        float rr;

        BSP_Encoder_Update();               /* ★ 自己采样 */
        rl = BSP_Encoder_GetRPM(ENC_LEFT);
        rr = BSP_Encoder_GetRPM(ENC_RIGHT);

        sum_l += rl;
        sum_r += rr;
        n++;

        if ((uint32_t)(HAL_GetTick() - tp) >= 200u) {
            float d = (rl > rr) ? (rl - rr) : (rr - rl);
            float b = (rl > rr) ? rl : rr;
            tp = HAL_GetTick();
            BSP_UART_Printf("   %5u   %6.1f   %6.1f   %5.1f\r\n",
                            (unsigned int)(tp - t0), (double)rl, (double)rr,
                            (double)((b > 1.0f) ? (d * 100.0f / b) : 0.0f));
        }
        HAL_Delay(10u);
    }

    BSP_Motor_Stop();
    BSP_UART_SendString("[电机] 停止 0.5s\r\n");
    HAL_Delay(500u);

    if (n == 0u) {
        return 0.0f;
    }
    avg_l = sum_l / (float)n;
    avg_r = sum_r / (float)n;
    big   = (avg_l > avg_r) ? avg_l : avg_r;

    BSP_UART_Printf("[电机] 平均: L=%d.%d  R=%d.%d rpm\r\n",
                    (int)avg_l, (int)((avg_l - (float)(int)avg_l) * 10.0f),
                    (int)avg_r, (int)((avg_r - (float)(int)avg_r) * 10.0f));

    if (big < 1.0f) {
        BSP_UART_SendString("[电机] 两轮都没转（rpm=0）—— 编码器没读数或电机没动\r\n");
        return 0.0f;
    }
    return (avg_l - avg_r) * 100.0f / big;   /* 正 = 左快 */
}

void BSP_Test_Motor(void)
{
    float d_fwd;
    float d_rev;
    float dabs;

    BSP_UART_SendString("[电机] 把车【架空】或放地上跑直线，测两轮转速差...\r\n");
    d_fwd = test_motor_run(TEST_MOTOR_DUTY,  "正转");
    d_rev = test_motor_run((int16_t)(-TEST_MOTOR_DUTY), "反转");

    dabs = (d_fwd >= 0.0f) ? d_fwd : -d_fwd;
    BSP_UART_Printf("[电机] 结论: 正转差 %+d.%d%%（正=左快），反转差 %+d.%d%%\r\n",
                    (int)d_fwd, (int)((d_fwd - (float)(int)d_fwd) * 10.0f),
                    (int)d_rev, (int)((d_rev - (float)(int)d_rev) * 10.0f));

    if (dabs < 5.0f) {
        BSP_UART_SendString("  OK < 5%% -> 两轮基本一致，不用补偿\r\n");
    } else if (dabs < 15.0f) {
        BSP_UART_SendString("  ~ 5~15%% -> 可以用软件补偿（LGAIN/RGAIN 或编码器闭环）\r\n");
    } else {
        BSP_UART_SendString("  !! > 15%% -> 差别偏大，先查机械：轮胎直径 / 转动阻力 / 配重\r\n");
    }
    BSP_UART_SendString("  * 若某个轮子方向相反，改菜单里的 MLINV / MRINV\r\n");
    BSP_UART_SendString("[电机] 结束。若某个轮子方向相反，改菜单里的 MLINV / MRINV\r\n");
    BSP_UART_SendString("       注意: 两个都改会同时翻前进和转向方向，改完要用 T 重验左右\r\n");
}

/* ==========================================================================
 * 7. 循迹
 * ========================================================================== */

/* ==========================================================================
 * 6. 编码器自检（★ 判断"编码器是死是活、接线对不对"最快的方法）
 * ========================================================================== */

/**
 * @brief  编码器自检：5 秒内用手转动两个轮子，看计数有没有变化
 * @note   ★ 本函数是【阻塞】的，主循环的 10ms 节拍出不来 ——
 *         所以这里必须【自己】调 BSP_Encoder_Update()，
 *         否则 GetDelta() 永远返回 0（没人采样），会误判成编码器坏了。
 *         其它自检也一样：谁把主循环占住了，谁就得自己喂节拍。
 *       ★ 判定标准在末尾打印：两个轮子都出现过计数 = 编码器正常。
 */
void BSP_Test_Encoder(void)
{
    uint32_t t0      = HAL_GetTick();
    uint32_t t_print = 0u;
    int32_t  sum_l   = 0;
    int32_t  sum_r   = 0;

    BSP_Encoder_Reset();

    BSP_UART_SendString("[编码器] 5 秒内请用手【慢慢转动两个轮子】...\r\n");
    BSP_UART_SendString("   L_delta   L_total | R_delta   R_total\r\n");

    while ((uint32_t)(HAL_GetTick() - t0) < 5000u) {
        int16_t dl;
        int16_t dr;

        BSP_Encoder_Update();               /* ★ 自己采样（主循环被占住了）*/

        dl = BSP_Encoder_GetDelta(ENC_LEFT);
        dr = BSP_Encoder_GetDelta(ENC_RIGHT);

        /* 累计"转过多少格"（正反转都算，取绝对值累加）*/
        sum_l += (dl >= 0) ? (int32_t)dl : -(int32_t)dl;
        sum_r += (dr >= 0) ? (int32_t)dr : -(int32_t)dr;

        if ((uint32_t)(HAL_GetTick() - t_print) >= 200u) {
            t_print = HAL_GetTick();
            BSP_UART_Printf("  %+8d  %8ld | %+8d  %8ld\r\n",
                            (int)dl, (long)BSP_Encoder_GetTotal(ENC_LEFT),
                            (int)dr, (long)BSP_Encoder_GetTotal(ENC_RIGHT));
        }
        HAL_Delay(10u);
    }

    BSP_UART_Printf("\r\n[编码器] 5 秒内转过：左 %ld 格，右 %ld 格\r\n",
                    (long)sum_l, (long)sum_r);

    if ((sum_l == 0) && (sum_r == 0)) {
        BSP_UART_SendString("  X 两个都没变化 -> 编码器没接/坏了（或者刚才没转轮子）\r\n");
    } else if (sum_l == 0) {
        BSP_UART_SendString("  X 左轮没变化 -> 查左编码器（TIM2 / PA0 / PA1）\r\n");
    } else if (sum_r == 0) {
        BSP_UART_SendString("  X 右轮没变化 -> 查右编码器（TIM4 / PB6 / PB7）\r\n");
    } else {
        BSP_UART_SendString("  OK 两个都有计数 -> 编码器是活的，接线也对\r\n");
    }

    BSP_UART_SendString("  * 转得慢时 rpm 显示 0 是正常的（10ms 内数不到格子）\r\n");
    BSP_UART_SendString("  * 如果编码器确实坏了：菜单里把 STALLms 改成 0，\r\n");
    BSP_UART_SendString("    否则堵转保护会拿编码器当依据，把车误判成堵转而停车\r\n");
}

void BSP_Test_Track(void)
{
    uint32_t t0 = HAL_GetTick();

    BSP_UART_SendString("[循迹] 3 秒内请把车在黑线上左右移动...\r\n");
    BSP_UART_SendString("  掩码  压线数  位置误差  原始电平\r\n");

    while ((uint32_t)(HAL_GetTick() - t0) < 3000u) {
        /* ★ 缓冲区用宏算，不要写死数字。
           MaskToString 会写入 TRACK_SENSOR_NUM + 1 个字节（含结尾 '\0'），
           探头数量从 6 路改成 8 路后，写死的 [8] 会溢出。 */
        char str[TRACK_SENSOR_NUM + 1u];
        char rawstr[TRACK_SENSOR_NUM + 1u];
        uint8_t mask = BSP_Track_GetMask();
        uint8_t raw  = BSP_Track_GetRawMask();

        BSP_Track_MaskToString(mask, str);
        BSP_Track_MaskToString(raw, rawstr);
        BSP_UART_Printf("  %s    %u     %+6.2f     %s\r\n",
                        str,
                        (unsigned int)BSP_Track_CountActive(),
                        (double)BSP_Track_GetError(),
                        rawstr);
        HAL_Delay(150u);
    }

    BSP_UART_SendString("[循迹] 结束。判断标准：\r\n");
    BSP_UART_SendString("  a) 探头压在黑线上时掩码对应位应当为 1\r\n");
    BSP_UART_SendString("  b) 若全为 1 或全为 0 且不随位置变化，多半是极性反了，\r\n");
    BSP_UART_SendString("     改菜单里的 ACTLOW\r\n");
    BSP_UART_SendString("  c) 走横线时掩码是什么？它决定 CAR_EVT_MASK_BLACK 该写多少\r\n");
}

/* ==========================================================================
 * V / M
 * ========================================================================== */

void BSP_Test_ParamShow(void)
{
    APP_Param_Print();
}

void BSP_Test_MenuShow(void)
{
    /* 强制菜单整屏重画：本文件里别的测试直接动过显存，菜单自己察觉不到 */
    APP_Menu_Invalidate();
    APP_Menu_ShowStatus();
    BSP_UART_SendString("[menu] back to the OLED status page\r\n");
}
