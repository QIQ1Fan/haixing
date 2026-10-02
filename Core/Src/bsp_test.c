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
    BSP_UART_SendString("  4. 电机正反转      7. 循迹传感器\r\n");
    BSP_UART_SendString("--------------------------------------------\r\n");
    BSP_UART_SendString("  V. 打印全部参数    T. 左右方向自检\r\n");
    BSP_UART_SendString("  G. 跑路线脚本      M. 回到 OLED 主界面\r\n");
    BSP_UART_SendString("  ?. 重新打印本菜单\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("请输入 4 / 7 / V / T / G / M / ? :\r\n");
    BSP_UART_SendString("(其余功能都在 OLED 菜单里: START CAR / TUNE PARAMS /\r\n");
    BSP_UART_SendString(" ROUTE TIME / SUPPORT LEG / SENSOR VIEW / SAVE / RUN ROUTE)\r\n");
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
    case '4': case '7':
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

void BSP_Test_Motor(void)
{
    BSP_UART_Printf("[电机] 正转 2s (占空比 %d)\r\n", (int)TEST_MOTOR_DUTY);
    BSP_Motor_SetSpeed(TEST_MOTOR_DUTY, TEST_MOTOR_DUTY);
    HAL_Delay(2000u);

    BSP_UART_SendString("[电机] 停止 0.5s\r\n");
    BSP_Motor_Stop();
    HAL_Delay(500u);

    BSP_UART_Printf("[电机] 反转 2s (占空比 %d)\r\n", (int)TEST_MOTOR_DUTY);
    BSP_Motor_SetSpeed(-TEST_MOTOR_DUTY, -TEST_MOTOR_DUTY);
    HAL_Delay(2000u);

    BSP_Motor_Stop();
    BSP_UART_SendString("[电机] 结束。若某个轮子方向相反，改菜单里的 MLINV / MRINV\r\n");
    BSP_UART_SendString("       注意: 两个都改会同时翻前进和转向方向，改完要用 T 重验左右\r\n");
}

/* ==========================================================================
 * 7. 循迹
 * ========================================================================== */

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
