/**
  ******************************************************************************
  * @file    bsp_test.c
  * @brief   底层驱动自检实现
  *
  * 【数据流】
  *   电脑串口助手 ──敲一个字符──► USART1
  *     └─► main.c 主循环轮询 BSP_UART_GetChar() 收到
  *           └─► BSP_Test_RunCommand(字符)
  *                 └─► switch 分发到对应模块的测试函数
  *                       └─► 测试函数用串口打印结果 / 操作硬件
  *                             └─► 返回 → 重新打印菜单
  *
  * 【设计原则：每个测试只验一个模块】
  *   所有测试都是"阻塞式"的：进去、跑完、返回。这样排查问题时
  *   可以把范围缩到一个模块，而不是一堆现象混在一起。
  *
  *   每个测试函数末尾都会打印一段"失败时该查哪里"的提示 ——
  *   这是刻意写的，因为硬件调不通时最有价值的信息不是"错了"，
  *   而是"接下来该动哪个螺丝"。
  *
  * 【正式跑车时怎么处理】
  *   整份文件可以删掉，不影响其它模块：没有任何 bsp_* 模块反过来依赖它。
  *   只要把 main.c 里对 BSP_Test_* 的调用去掉即可。
  ******************************************************************************
  */
#include "bsp_test.h"

#include "bsp_gpio.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_encoder.h"
#include "bsp_track.h"
#include "bsp_key.h"
#include "bsp_buzzer.h"
#include "bsp_oled.h"
#include "bsp_uart.h"

/* ★ 自检文件也要包含应用层的头：下面的 "R/启动/调参" 命令直接操作小车,
   属于"把整车也当成一个可测试对象"的用法 */
#include "app_param.h"
#include "app_car.h"
#include "app_leg.h"
#include "app_menu.h"
#include "app_route.h"

/* 测试用电机占空比，别用满速，桌面测试时车会飞出去 */
#define TEST_MOTOR_DUTY     1200        /* 满量程 3599，约 33% */
#define TEST_MOTOR_RAMP_MAX 1800        /* 约 50% */

/* ==========================================================================
 * 串口菜单
 * ========================================================================== */

void BSP_Test_ShowMenu(void)
{
    BSP_UART_SendString("\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("   循迹小车 BSP 底层自检  (USART1 115200)\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("  1. 蜂鸣器        2. 串口回显\r\n");
    BSP_UART_SendString("  3. 舵机扫描      4. 电机正反转\r\n");
    BSP_UART_SendString("  5. 电机渐变      6. 编码器读数\r\n");
    BSP_UART_SendString("  7. 循迹传感器    8. 按键事件\r\n");
    BSP_UART_SendString("  9. OLED 显示     0. 全部跑一遍\r\n");
    BSP_UART_SendString("--------------------------------------------\r\n");
    BSP_UART_SendString("  P. 蜂鸣器极性自检   (响/静音分不清时先跑这个)\r\n");
    BSP_UART_SendString("  H. 舵机隔离保持     (抖动时用来定位原因)\r\n");
    BSP_UART_SendString("--------------- 整车 / 调参 ----------------\r\n");
    BSP_UART_SendString("  C. 启动/停止循迹    R. 实时状态打印 5s\r\n");
    BSP_UART_SendString("  V. 打印全部参数     W. 串口调参(改一个值)\r\n");
    BSP_UART_SendString("  S. 保存参数到Flash  D. 恢复出厂参数\r\n");
    BSP_UART_SendString("  L. 支撑脚测试       M. 回到 OLED 主界面\r\n");
    BSP_UART_SendString("  E. 串口接收监视     G. 跑路线脚本\r\n");
    BSP_UART_SendString("  ?. 重新打印本菜单\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("请输入 1-9 / 0 / P / H / C / R / V / W / S / D / L / M / E / ? :\r\n");
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
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
    case '?':
    case 'P': case 'p':     /* 蜂鸣器极性自检 */
    case 'H': case 'h':     /* 舵机隔离保持测试 */
    /* ---- 整车 / 调参 ---- */
    case 'C': case 'c':     /* 启动 / 停止循迹 */
    case 'R': case 'r':     /* 实时状态打印 */
    case 'V': case 'v':     /* 打印全部参数 */
    case 'W': case 'w':     /* 串口调参 */
    case 'S': case 's':     /* 保存参数 */
    case 'D': case 'd':     /* 恢复默认参数 */
    case 'L': case 'l':     /* 支撑脚测试 */
    case 'M': case 'm':     /* 回到 OLED 主界面 */
    case 'E': case 'e':     /* 串口接收监视 */
    case 'G': case 'g':     /* 跑路线脚本 */
        return 1u;
    default:
        return 0u;          /* 换行、回车、噪声一律静默丢弃 */
    }
}

/**
  * @brief  收到"不是命令"的字节时，把原因说清楚，而不是默默丢掉
  *
  * 【为什么要专门写这个函数】
  *   原来的实现是"不是命令就直接 return"。这在正常情况下没问题，
  *   但调试初期最要命的就是"发了没反应"：
  *     · 输入法在全角状态打出「？」→ UTF-8 是三个字节 EF BC 9F，
  *       一个都不是 '?'，全被丢掉 → 现象和"串口没接好"一模一样
  *     · 命令敲错一个字母（比如把 V 敲成 B）→ 也是静默丢弃
  *   两种情况的排查方向完全不同，却给出同样的沉默，这就是把人卡住的原因。
  *   现在把收到的字节原样报出来，一眼就能分辨。
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
        BSP_Test_ShowMenu();
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
    case '1': BSP_Test_Buzzer();        break;
    case '2': BSP_Test_Uart();          break;
    case '3': BSP_Test_Servo();         break;
    case '4': BSP_Test_Motor();         break;
    case '5': BSP_Test_MotorRamp();     break;
    case '6': BSP_Test_Encoder();       break;
    case '7': BSP_Test_Track();         break;
    case '8': BSP_Test_Key();           break;
    case '9': BSP_Test_Oled();          break;
    case '0': BSP_Test_All();           break;
    case 'P': case 'p': BSP_Test_BuzzerPolarity(); break;
    case 'H': case 'h': BSP_Test_ServoHold();      break;

    /* ---- 整车 / 调参 ----
       ★ 注意 'C' 和 'R' 会启动电机，测试前请确认车在赛道上、
         或者干脆把车架空。这两个命令不做任何"确认"提示，
         因为赛道调试时最烦的就是每一步都要回一次车。 */
    case 'C': case 'c': BSP_Test_CarRun();         break;
    case 'R': case 'r': BSP_Test_CarStatus();      break;
    case 'V': case 'v': BSP_Test_ParamShow();      break;
    case 'W': case 'w': BSP_Test_ParamSet();       break;
    case 'S': case 's': BSP_Test_ParamSave();      break;
    case 'D': case 'd': BSP_Test_ParamDefault();   break;
    case 'L': case 'l': BSP_Test_Leg();            break;
    case 'M': case 'm': BSP_Test_MenuShow();       break;
    case 'E': case 'e': BSP_Test_UartMonitor();    break;
    case 'G': case 'g': APP_Route_Run();           break;   /* 跑路线脚本 */

    default:
        BSP_Test_ShowMenu();
        return;
    }

    BSP_UART_SendString("<<< 测试结束\r\n");
    BSP_Test_ShowMenu();
}

/* ==========================================================================
 * 1. 蜂鸣器
 * ========================================================================== */

void BSP_Test_Buzzer(void)
{
    BSP_UART_SendString("[蜂鸣器] 应当听到 3 声短鸣\r\n");
    BSP_Buzzer_Alarm(3u);
    BSP_UART_SendString("[蜂鸣器] 结束。没声音请查：\r\n");
    BSP_UART_SendString("  a) PB3 是否被 JTAG 占用（HAL_MspInit 里的 SWJ_NOJTAG）\r\n");
    BSP_UART_SendString("  b) 你的蜂鸣器是不是高电平触发的（改 BUZZER_ON_LEVEL）\r\n");
}

/* ==========================================================================
 * 2. 串口
 * ========================================================================== */

void BSP_Test_Uart(void)
{
    int i;

    BSP_UART_SendString("[串口] 打印测试\r\n");
    BSP_UART_Printf("  整数 : %d\r\n",   (int)-12345);
    BSP_UART_Printf("  无符号: %u\r\n",  (unsigned int)65535u);
    BSP_UART_Printf("  十六进制: 0x%08X\r\n", (unsigned int)0xDEADBEEFu);
    BSP_UART_Printf("  浮点 : %.3f\r\n", (double)3.14159);

    BSP_UART_SendString("[串口] 逐字节输出 ABCDEFG...\r\n  ");
    for (i = 0; i < 26; i++) {
        BSP_UART_SendByte((uint8_t)('A' + i));
    }
    BSP_UART_SendString("\r\n[串口] 如果上面显示正常，说明串口通了\r\n");
}

/* ==========================================================================
 * 3. 舵机
 * ========================================================================== */

void BSP_Test_Servo(void)
{
    /* 本工程角度约定：0°~180°，中位 90°。
       序列故意走 中位→两端→中位，方便一眼看出行程是否对得上 */
    static const int16_t seq[] = { 90, 180, 90, 0, 90 };
    static const char *const label[] = { "中位", "一端(180°)", "中位",
                                         "另一端(0°)", "中位" };
    uint32_t i;

    BSP_UART_SendString("[舵机] 中位 -> 180° -> 中位 -> 0° -> 中位\r\n");
    BSP_UART_SendString("  角度   脉宽    说明\r\n");

    for (i = 0u; i < (sizeof(seq) / sizeof(seq[0])); i++) {
        BSP_Servo_SetAngle((float)seq[i]);
        BSP_UART_Printf("  %3d°  %4u us  %s\r\n",
                        (int)seq[i],
                        (unsigned int)BSP_Servo_GetPulseUs(),
                        label[i]);
        HAL_Delay(700u);
    }

    BSP_UART_SendString("[舵机] 结束。判断标准：\r\n");
    BSP_UART_SendString("  a) 两端应当明显对称，脉宽 500us / 2500us\r\n");
    BSP_UART_SendString("  b) 若两端不对称或打不满，改 bsp_config.h 的 SERVO_PULSE_x_US\r\n");
    BSP_UART_SendString("  c) 若到位后嗡嗡响个不停，说明顶到机械限位了，把行程往里收 50us\r\n");
}

/* ==========================================================================
 * P. 蜂鸣器极性自检
 * --------------------------------------------------------------------------
 * 用途：不靠猜，直接试出"哪个电平是静音"。
 * 这个测试故意绕开 BUZZER_ON_LEVEL / BUZZER_OFF_LEVEL，直接写裸电平，
 * 所以即使当前配置是反的也能正确测出来。
 * ========================================================================== */

void BSP_Test_BuzzerPolarity(void)
{
    uint8_t i;

    BSP_UART_SendString("[蜂鸣器极性] 交替输出低/高电平各 1 秒，共 3 轮\r\n");
    BSP_UART_SendString("  请记住哪一段是【安静】的，然后对应设置 bsp_config.h：\r\n");
    BSP_UART_SendString("\r\n");
    BSP_UART_SendString("   低电平段安静 -> 低电平触发（当前默认），不用改：\r\n");
    BSP_UART_SendString("       #define BUZZER_ON_LEVEL   GPIO_PIN_RESET\r\n");
    BSP_UART_SendString("       #define BUZZER_OFF_LEVEL  GPIO_PIN_SET\r\n");
    BSP_UART_SendString("\r\n");
    BSP_UART_SendString("   高电平段安静 -> 高电平触发，改成：\r\n");
    BSP_UART_SendString("       #define BUZZER_ON_LEVEL   GPIO_PIN_SET\r\n");
    BSP_UART_SendString("       #define BUZZER_OFF_LEVEL  GPIO_PIN_RESET\r\n");
    BSP_UART_SendString("\r\n");

    for (i = 0u; i < 3u; i++) {
        BSP_UART_SendString("  >>> 现在输出【低电平】...\r\n");
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        HAL_Delay(1000u);

        BSP_UART_SendString("  >>> 现在输出【高电平】...\r\n");
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
        HAL_Delay(1000u);
    }

    /* 恢复成配置里的静音电平 */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_OFF_LEVEL);
    BSP_UART_SendString("[蜂鸣器极性] 结束\r\n");
}

/* ==========================================================================
 * H. 舵机隔离保持测试
 * --------------------------------------------------------------------------
 * 用途：把其它活动全停掉，只让舵机保持一个角度，用来区分抖动的来源。
 *   保持期间稳定  -> 抖动是"别的东西干扰"（串口/OLED 刷屏/噪声触发测试）
 *   仍然抖动      -> 是软件 PWM 本身，或供电/接线问题
 * ========================================================================== */

void BSP_Test_ServoHold(void)
{
    BSP_UART_SendString("[舵机隔离] 保持 90° 中位 5 秒，期间不做任何串口输出\r\n");
    BSP_UART_SendString("  稳定不动   -> 抖动来自其它任务或噪声干扰\r\n");
    BSP_UART_SendString("  仍然抖动   -> 软件 PWM 本身，或舵机供电/接线问题\r\n");
    BSP_UART_SendString("  现在开始...\r\n");

    BSP_Servo_SetAngle(90.0f);

    /* ★ 这 5 秒内故意什么都不做：不打印、不刷 OLED、不扫按键。
       目的就是把变量减到一个，让抖动的有无变得可判断。 */
    HAL_Delay(5000u);

    BSP_UART_Printf("[舵机隔离] 结束，脉宽 %u us\r\n",
                    (unsigned int)BSP_Servo_GetPulseUs());
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
    BSP_UART_SendString("[电机] 结束。若某个轮子方向相反，改 bsp_config.h 的 MOTOR_x_INVERT\r\n");
}

/* ==========================================================================
 * 5. 电机占空比渐变
 * ========================================================================== */

void BSP_Test_MotorRamp(void)
{
    int16_t d;

    BSP_UART_SendString("[渐变] 0 -> 50% -> 0\r\n");

    for (d = 0; d <= TEST_MOTOR_RAMP_MAX; d += 60) {
        BSP_Motor_SetSpeed(d, d);
        HAL_Delay(30u);
    }
    BSP_UART_Printf("  峰值占空比 %d\r\n", (int)TEST_MOTOR_RAMP_MAX);

    for (d = TEST_MOTOR_RAMP_MAX; d >= 0; d -= 60) {
        BSP_Motor_SetSpeed(d, d);
        HAL_Delay(30u);
    }

    BSP_Motor_Stop();
    BSP_UART_SendString("[渐变] 结束。若低速有啸叫或抖动，属正常，可调 PWM 频率\r\n");
}

/* ==========================================================================
 * 6. 编码器
 * ========================================================================== */

void BSP_Test_Encoder(void)
{
    uint32_t t0;
    uint32_t next;
    uint32_t printed = 0u;

    BSP_UART_SendString("[编码器] 3 秒内请用手转动两个轮子...\r\n");
    BSP_UART_SendString("  格式: L增量 L累计 L转速 | R增量 R累计 R转速\r\n");

    BSP_Encoder_Reset();
    t0   = HAL_GetTick();
    next = t0;

    while ((uint32_t)(HAL_GetTick() - t0) < 3000u) {
        if ((uint32_t)(HAL_GetTick() - next) >= ENC_SAMPLE_PERIOD_MS) {
            next += ENC_SAMPLE_PERIOD_MS;
            BSP_Encoder_Update();

            {
                int16_t dl = BSP_Encoder_GetDelta(ENC_LEFT);
                int16_t dr = BSP_Encoder_GetDelta(ENC_RIGHT);

                /* 只在有变化时打印，否则刷屏太快看不清 */
                if ((dl != 0) || (dr != 0)) {
                    BSP_UART_Printf(
                        "  L %6d %8d %8.2f | R %6d %8d %8.2f\r\n",
                        (int)dl, (int)BSP_Encoder_GetTotal(ENC_LEFT),
                        (double)BSP_Encoder_GetRPM(ENC_LEFT),
                        (int)dr, (int)BSP_Encoder_GetTotal(ENC_RIGHT),
                        (double)BSP_Encoder_GetRPM(ENC_RIGHT));
                    printed++;
                }
            }
        }
    }

    if (printed == 0u) {
        BSP_UART_SendString("[编码器] 全程无计数！请检查：\r\n");
        BSP_UART_SendString("  a) 编码器 A/B 相是否接到了 PA0/PA1 与 PB6/PB7\r\n");
        BSP_UART_SendString("  b) 编码器供电与共地\r\n");
        BSP_UART_SendString("  c) 若是开漏输出，确认内部上拉已开（BSP_Encoder_Init 已开）\r\n");
    } else {
        BSP_UART_Printf("[编码器] 共收到 %u 次有效增量\r\n",
                        (unsigned int)printed);
        BSP_UART_Printf("  标定值 ENC_CPR = %.1f，若转速显示不符请改 bsp_config.h\r\n",
                        (double)ENC_CPR);
    }
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
    BSP_UART_SendString("  b) 若全为 1 或全为 0 且不随位置变化，多半是极性反了\r\n");
    BSP_UART_SendString("     把 bsp_config.h 的 TRACK_ACTIVE_LOW 取反试试\r\n");
}

/* ==========================================================================
 * 8. 按键
 * ========================================================================== */

void BSP_Test_Key(void)
{
    static const char *const name[KEY_ID_NUM] = {
        "确认(PB13)", "退出(PB14)", "加  (PB15)", "减  (PA15)"
    };
    uint32_t t0   = HAL_GetTick();
    uint32_t next = t0;
    uint32_t i;
    uint8_t  prev_raw = 0xFFu;      /* 上一次的原始电平位图。0xFF = 还没读过，
                                       这样第一拍必定打印一次初始电平 */

    /* ★ 这个测试同时看两层数据，缺一不可：
     *     第一层 = 引脚原始电平（绕开消抖、绕开事件，直接读 GPIO）
     *     第二层 = 消抖后的事件（短按/长按/连发）
     *   为什么必须分开看："按下没反应"有两种完全不同的原因 ——
     *     · 电平根本没变  → 线没接对/按键没接这个脚，是硬件问题
     *     · 电平变了但没事件 → 才是程序的问题
     *   只看事件的话，这两种现象一模一样，只能靠猜。 */
    BSP_UART_SendString("\r\n[按键] 10 秒内请按任意键（短按、长按都试试）...\r\n");
    BSP_UART_SendString("[按键] 同时监视四个引脚的【原始电平】：1 = 松开，0 = 按下\r\n");
    BSP_UART_SendString("[按键] 判断方法：\r\n");
    BSP_UART_SendString("[按键]   按住时电平 1 -> 0   = 引脚和按键都是好的\r\n");
    BSP_UART_SendString("[按键]   按住时电平不动     = 线没接到这个脚（换脚或查线，不是程序问题）\r\n");
    BSP_UART_SendString("[按键]   一直是 0           = 这路被短接到地了，或者按键常闭\r\n");

    while ((uint32_t)(HAL_GetTick() - t0) < 10000u) {
        uint8_t raw = 0u;

        /* 保持 10ms 一拍，和 main 里 BSP_Key_Scan() 的节奏一致 ——
           消抖(30ms)和长按(0.6s)的判定都建立在这个节奏上 */
        if ((uint32_t)(HAL_GetTick() - next) < KEY_SCAN_PERIOD_MS) {
            continue;
        }
        next += KEY_SCAN_PERIOD_MS;

        BSP_Key_Scan();

        /* ---- ① 原始电平：只在这一层能看出"线到底接对没有" ---- */
        for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
            if (BSP_GPIO_ReadKeyRaw((uint8_t)i) != 0u) {
                raw |= (uint8_t)(1u << i);
            }
        }

        if (raw != prev_raw) {
            prev_raw = raw;
            BSP_UART_Printf("[按键] 电平变化  OK=%u BACK=%u PLUS=%u MINUS=%u  (1=松开 0=按下)\r\n",
                            (unsigned int)((raw >> 0) & 1u),
                            (unsigned int)((raw >> 1) & 1u),
                            (unsigned int)((raw >> 2) & 1u),
                            (unsigned int)((raw >> 3) & 1u));
        }

        /* ---- ② 事件：这一层才是菜单真正用的东西 ---- */
        for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
            BSP_KeyEvent_t e = BSP_Key_GetEvent((BSP_KeyId_t)i);

            if (e == KEY_EVENT_NONE) {
                continue;
            }
            switch (e) {
            case KEY_EVENT_PRESS:
                BSP_UART_Printf("  %s  短按\r\n", name[i]);
                break;
            case KEY_EVENT_LONG:
                BSP_UART_Printf("  %s  长按\r\n", name[i]);
                break;
            case KEY_EVENT_REPEAT:
                BSP_UART_Printf("  %s  连发\r\n", name[i]);
                break;
            default:
                break;
            }
        }
    }

    BSP_UART_SendString("[按键] 结束。结论怎么下：\r\n");
    BSP_UART_SendString("  电平有 1->0、也打出了事件  = 这一路完全正常\r\n");
    BSP_UART_SendString("  电平有 1->0、但没有事件    = 程序和接线都没问题，是按键抖动太大，加大 KEY_DEBOUNCE_CNT\r\n");
    BSP_UART_SendString("  电平一直不动              = 这一路没接到单片机，查线/查脚位\r\n");
    BSP_UART_SendString("  只有【减】(PA15)不动       = 先查 PA15 的 JTAG 释放（SWJ_NOJTAG），再查按键\r\n");
}

/* ==========================================================================
 * 9. OLED
 * ========================================================================== */

void BSP_Test_Oled(void)
{
    uint32_t t0;
    uint8_t  percent = 0u;

    BSP_UART_SendString("[OLED] 显示测试图案 5 秒\r\n");

    BSP_OLED_TestPattern();
    HAL_Delay(5000u);

    BSP_UART_SendString("[OLED] 切换到实时数值显示 5 秒\r\n");

    /* 重新取基准，否则下面这个循环会因为上面已经等掉 5 秒而提前结束 */
    t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < 5000u) {
        BSP_OLED_Clear();
        BSP_OLED_DrawString(0u, 0u, "OLED BSP SELFTEST");
        BSP_OLED_DrawHLine(0u, 1u, 128u);
        BSP_OLED_Printf(0u, 2u, "Uptime: %5u ms", (unsigned int)HAL_GetTick());
        BSP_OLED_Printf(0u, 3u, "Servo : %4u us", (unsigned int)BSP_Servo_GetPulseUs());
        BSP_OLED_Printf(0u, 4u, "Track : %+6.2f", (double)BSP_Track_GetError());
        BSP_OLED_DrawBar(0u, 6u, 120u, 1u, percent);
        BSP_OLED_Refresh();

        percent = (uint8_t)((percent + 5u) % 101u);
        HAL_Delay(100u);
    }

    BSP_UART_SendString("[OLED] 结束。若全黑，检查：\r\n");
    BSP_UART_SendString("  a) OLED_I2C_ADDR 是否为 0x78（有的模块是 0x7A）\r\n");
    BSP_UART_SendString("  b) 调小 OLED_I2C_DELAY_LOOPS 试试（高速下时序不足）\r\n");
    BSP_UART_SendString("  c) SCL/SDA 是否接反\r\n");

    /* ★ 把屏幕交还给菜单。
       本测试直接往显存里画了图案，而菜单是靠"脏页"机制省刷屏时间的 ——
       它并不知道屏幕被别人动过，如果只是切回状态页而不强制重画，
       屏幕上会留着这个自检图案（菜单认为"内容没变，不用刷"）。
       Invalidate() 就是告诉菜单"显存已经不可信了，整屏重来一次" */
    APP_Menu_Invalidate();
}

/* ==========================================================================
 * 0. 综合
 * ========================================================================== */

void BSP_Test_All(void)
{
    BSP_UART_SendString("[综合] 依次执行全部测试\r\n");

    BSP_Test_Buzzer();
    BSP_Test_Uart();
    BSP_Test_Oled();
    BSP_Test_Servo();
    BSP_Test_Encoder();
    BSP_Test_Track();
    BSP_Test_Key();
    BSP_Test_MotorRamp();
    BSP_Test_Motor();

    BSP_UART_SendString("[综合] 全部完成\r\n");
}

/* ==========================================================================
 * 整车 / 调参命令
 * --------------------------------------------------------------------------
 * 这一节和上面 1~9 的自检性质不同：
 *   自检是"验硬件"，一次只验一个模块，跑完就停；
 *   这一节是"用整车"，会真的让车跑起来，也会改参数。
 *
 * ★ 一个必须注意的坑：整个 bsp_test 都是"阻塞"的 —— 一旦进了测试函数，
 *   main 的主循环就停了，APP_Car_Update() 不会被调用。
 *   如果这时候车还在跑，它会保持上一次的 PWM 一直冲下去。
 *   所以：
 *     · 凡是可能长时间停留的命令，要么先停车（ParamSet），
 *       要么在循环里自己把控制任务也跑起来（CarStatus）
 * ========================================================================== */

/** 把一行参数按"序号 名字 = 值"打印出来，整型和浮点分别处理 */
static void test_print_param_line(uint8_t idx)
{
    const APP_ParamItem_t *item = &g_param_table[idx];
    float v = APP_Param_GetByIndex(idx);

    if (item->type == PTYPE_F32) {
        BSP_UART_Printf("  %2u  %-9s = %.*f\r\n",
                        (unsigned int)idx, item->name,
                        (int)item->decimals, (double)v);
    } else {
        BSP_UART_Printf("  %2u  %-9s = %d\r\n",
                        (unsigned int)idx, item->name,
                        (int)(v + ((v >= 0.0f) ? 0.5f : -0.5f)));
    }
}

/** 跳过分隔符（空格/制表符） */
static const char *test_skip_spaces(const char *s)
{
    while ((*s == ' ') || (*s == '\t')) {
        s++;
    }
    return s;
}

/** 解析一个十进制整数，成功返回 1 并前移指针 */
static uint8_t test_parse_int(const char **ps, int32_t *out)
{
    const char *s = test_skip_spaces(*ps);
    int32_t v = 0;
    uint8_t neg = 0u;
    uint8_t any = 0u;

    if (*s == '-') {
        neg = 1u;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while ((*s >= '0') && (*s <= '9')) {
        v = (v * 10) + (int32_t)(*s - '0');
        s++;
        any = 1u;
    }

    if (any == 0u) {
        return 0u;
    }

    *out = (neg != 0u) ? (-v) : v;
    *ps  = s;
    return 1u;
}

/** 解析一个十进制浮点（支持负号和小数点），成功返回 1 并前移指针 */
static uint8_t test_parse_float(const char **ps, float *out)
{
    const char *s = test_skip_spaces(*ps);
    float   v = 0.0f;
    float   scale = 0.1f;
    uint8_t neg = 0u;
    uint8_t any = 0u;

    if (*s == '-') {
        neg = 1u;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while ((*s >= '0') && (*s <= '9')) {
        v = (v * 10.0f) + (float)(*s - '0');
        s++;
        any = 1u;
    }

    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9')) {
            v += (float)(*s - '0') * scale;
            scale *= 0.1f;
            s++;
            any = 1u;
        }
    }

    if (any == 0u) {
        return 0u;
    }

    *out = (neg != 0u) ? (-v) : v;
    *ps  = s;
    return 1u;
}

/**
  * @brief  读一行（以回车/换行结束），带超时
  * @param  buf        输出缓冲
  * @param  max_len    缓冲长度（含结尾 '\0'）
  * @param  timeout_ms 超时毫秒
  * @retval 实际读到的字符数，0 = 超时或空行
  * @note   故意不做"回显"：串口助手本身就带本地回显，
  *         再回一遍会变成每个字符显示两遍
  */
static uint8_t test_read_line(char *buf, uint8_t max_len, uint32_t timeout_ms)
{
    uint8_t  n = 0u;
    uint8_t  c;
    uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms) {
        if (BSP_UART_WaitChar(&c, 20u) == 0u) {
            continue;
        }

        if ((c == '\r') || (c == '\n')) {
            if (n == 0u) {
                continue;   /* 吃掉行首多余的回车换行，不当作"空输入" */
            }
            break;
        }

        if ((c == 8u) || (c == 127u)) {     /* 退格 / DEL */
            if (n > 0u) {
                n--;
            }
            continue;
        }

        if ((uint16_t)n + 1u < (uint16_t)max_len) {
            buf[n] = (char)c;
            n++;
        }
    }

    buf[n] = '\0';
    return n;
}

/* -------------------------------------------------------------------------- */

void BSP_Test_CarRun(void)
{
    if (APP_Car_IsRunning() != 0u) {
        APP_Car_Stop();
        BSP_UART_SendString("[car] STOPPED (coast)\r\n");
        BSP_UART_Printf("[car] run time was %u ms\r\n",
                        (unsigned int)APP_Car_GetStatus()->run_ms);
        return;
    }

    APP_Car_Start();
    BSP_UART_SendString("[car] STARTED - now following the line\r\n");
    BSP_UART_SendString("  a) OLED shows live error / turn / speeds\r\n");
    BSP_UART_SendString("  b) BACK key on the car = emergency stop\r\n");
    BSP_UART_SendString("  c) press 'C' here again to stop\r\n");
}

void BSP_Test_CarStatus(void)
{
    uint32_t t0;
    uint32_t next_tick;
    uint32_t next_print;
    uint8_t  running = APP_Car_IsRunning();

    BSP_UART_SendString("[car] live status for 5 s, one line per 100 ms\r\n");

    if (running != 0u) {
        /* ★ 车在跑的时候，这个循环必须自己把"控制 + 采样"这两件事做掉，
           否则车会在 5 秒里无人驾驶地冲出去。
           做法就是把 main 主循环里那两个调用原样搬进来 */
        BSP_UART_SendString("[car] car is RUNNING - control loop is kept alive here\r\n");
    } else {
        BSP_UART_SendString("[car] car is IDLE - only telemetry is printed\r\n");
    }

    t0         = HAL_GetTick();
    next_tick  = t0;
    next_print = t0;

    while ((uint32_t)(HAL_GetTick() - t0) < 5000u) {
        /* 10ms 控制节拍，和 main 里一模一样 */
        if ((uint32_t)(HAL_GetTick() - next_tick) >= CTRL_PERIOD_MS) {
            next_tick = HAL_GetTick();
            BSP_Encoder_Update();
            APP_Car_Update();
        }

        /* 100ms 打印一次。★ 别再快了：一行约 90 字节，
           115200 下要 8ms，10ms 打一行等于把节拍全占了 */
        if ((uint32_t)(HAL_GetTick() - next_print) >= 100u) {
            next_print = HAL_GetTick();
            APP_Car_PrintStatus();
        }
    }

    BSP_UART_Printf("[car] encoder dt = %u ms (should be ~%u)\r\n",
                    (unsigned int)BSP_Encoder_GetDtMs(),
                    (unsigned int)CTRL_PERIOD_MS);
    BSP_UART_SendString("[car] check: err sign matches the car's offset,\r\n");
    BSP_UART_SendString("             turn = err * KP, L/R speeds swing together\r\n");
}

void BSP_Test_ParamShow(void)
{
    APP_Param_Print();
}

void BSP_Test_ParamSave(void)
{
    BSP_UART_SendString("[param] writing to flash...\r\n");
    BSP_UART_SendString("  (the whole system is blocked for ~30 ms, that is normal)\r\n");

    if (APP_Param_Save() != 0u) {
        BSP_UART_SendString("[param] SAVED. survives power-off now\r\n");
    } else {
        BSP_UART_SendString("[param] SAVE FAILED!\r\n");
        BSP_UART_SendString("  a) PARAM_FLASH_ENABLE is 0 in bsp_config.h\r\n");
        BSP_UART_SendString("  b) page 0x0800FC00 collides with your program\r\n");
    }
}

void BSP_Test_ParamDefault(void)
{
    APP_Param_ResetDefault();
    BSP_UART_SendString("[param] factory defaults loaded into RAM\r\n");
    BSP_UART_SendString("  press 'S' to save them, or 'V' to look at them\r\n");
}

void BSP_Test_ParamSet(void)
{
    char        line[32];
    const char *p;
    int32_t     idx = 0;
    float       val = 0.0f;
    uint8_t     i;

    /* 调参必须在车停下来的时候做：本函数是阻塞的，
       期间没人跑控制律，车会保持上一次的 PWM 一直冲 */
    if (APP_Car_IsRunning() != 0u) {
        APP_Car_Stop();
        BSP_UART_SendString("[param] car was RUNNING -> stopped for safety\r\n");
    }

    BSP_UART_SendString("\r\n[param] index table:\r\n");
    for (i = 0u; i < g_param_count; i++) {
        test_print_param_line(i);
    }

    BSP_UART_SendString("\r\n[param] type:  <index> <value>   then Enter\r\n");
    BSP_UART_SendString("[param] e.g.:  0 1500      (set KP to 1500)\r\n");
    BSP_UART_SendString("[param] just Enter = cancel.  waiting 20 s...\r\n");

    if (test_read_line(line, (uint8_t)sizeof(line), 20000u) == 0u) {
        BSP_UART_SendString("[param] timeout / empty -> cancelled\r\n");
        return;
    }

    p = line;
    if ((test_parse_int(&p, &idx) == 0u) || (test_parse_float(&p, &val) == 0u)) {
        BSP_UART_SendString("[param] cannot parse. expected: <index> <value>\r\n");
        return;
    }

    if ((idx < 0) || (idx >= (int32_t)g_param_count)) {
        BSP_UART_Printf("[param] index %d out of range (0..%u)\r\n",
                        (int)idx, (unsigned int)(g_param_count - 1u));
        return;
    }

    APP_Param_SetByIndex((uint8_t)idx, val);

    BSP_UART_SendString("[param] new value -> ");
    test_print_param_line((uint8_t)idx);
    BSP_UART_SendString("  (clamped to the range in the table)\r\n");

    BSP_UART_SendString("[param] save to flash? y / n  (5 s)\r\n");
    if (test_read_line(line, (uint8_t)sizeof(line), 5000u) != 0u) {
        if ((line[0] == 'y') || (line[0] == 'Y')) {
            BSP_Test_ParamSave();
            return;
        }
    }
    BSP_UART_SendString("[param] kept in RAM only. press 'S' when you are happy\r\n");
}

void BSP_Test_Leg(void)
{
    uint32_t t0;

    BSP_UART_SendString("[leg] up -> down -> up\r\n");
    BSP_UART_SendString("  watch the support leg: it should move smoothly,\r\n");
    BSP_UART_SendString("  no buzzing at either end (buzzing = hitting the limit)\r\n");

    /* ---- 收起 ---- */
    APP_Leg_Up();
    t0 = HAL_GetTick();
    while ((APP_Leg_IsMoving() != 0u) && ((uint32_t)(HAL_GetTick() - t0) < 3000u)) {
        APP_Leg_Update();       /* 主循环停着，得自己推斜坡 */
        HAL_Delay(10u);
    }
    BSP_UART_Printf("  UP   angle = %u deg (target %u)\r\n",
                    (unsigned int)APP_Leg_GetAngle(),
                    (unsigned int)APP_Leg_GetTarget());
    HAL_Delay(600u);

    /* ---- 放下 ---- */
    APP_Leg_Down();
    t0 = HAL_GetTick();
    while ((APP_Leg_IsMoving() != 0u) && ((uint32_t)(HAL_GetTick() - t0) < 3000u)) {
        APP_Leg_Update();
        HAL_Delay(10u);
    }
    BSP_UART_Printf("  DOWN angle = %u deg (target %u)\r\n",
                    (unsigned int)APP_Leg_GetAngle(),
                    (unsigned int)APP_Leg_GetTarget());
    HAL_Delay(800u);

    /* ---- 回收起位 ---- */
    APP_Leg_Up();
    t0 = HAL_GetTick();
    while ((APP_Leg_IsMoving() != 0u) && ((uint32_t)(HAL_GetTick() - t0) < 3000u)) {
        APP_Leg_Update();
        HAL_Delay(10u);
    }
    BSP_UART_Printf("  UP   angle = %u deg\r\n", (unsigned int)APP_Leg_GetAngle());

    BSP_UART_SendString("[leg] done. to calibrate:\r\n");
    BSP_UART_SendString("  a) press W and tune LEGDN until the wheels just leave the ground\r\n");
    BSP_UART_SendString("  b) then back off 5~10 deg so the servo is not stalled\r\n");
    BSP_UART_SendString("  c) tune LEGUP so the leg is fully tucked in\r\n");
    BSP_UART_SendString("  d) press S to save\r\n");
}


/* -------------------------------------------------------------------------- */

/** 控制字符的可读名字，只为了让监视输出好认 */
static const char *test_byte_name(uint8_t c)
{
    switch (c) {
    case 0x0Du: return "CR";
    case 0x0Au: return "LF";
    case 0x09u: return "TAB";
    case 0x20u: return "SP";
    default:    return (c < 0x20u) ? "CTRL" : ".";
    }
}

void BSP_Test_UartMonitor(void)
{
    uint32_t t0;
    uint32_t next_tick;
    uint32_t n = 0u;
    uint8_t  c;

    BSP_UART_SendString("\r\n[uart] 接收监视 10 秒：每收到一个字节就打印一行\r\n");
    BSP_UART_SendString("       现在请在串口助手里敲几个字符（比如 ? 或 abc）\r\n");
    BSP_UART_SendString("       格式: 序号  十六进制  字符\r\n\r\n");

    t0        = HAL_GetTick();
    next_tick = t0;

    while ((uint32_t)(HAL_GetTick() - t0) < 10000u) {
        /* ★ 和 CarStatus 一样：车在跑的时候，这个阻塞循环必须自己把
           "采样 + 控制"做掉，否则 10 秒里车是无人驾驶的 */
        if ((uint32_t)(HAL_GetTick() - next_tick) >= CTRL_PERIOD_MS) {
            next_tick = HAL_GetTick();
            BSP_Encoder_Update();
            APP_Car_Update();
        }

        if (BSP_UART_GetChar(&c) == 0u) {
            continue;
        }

        n++;

        if ((c >= 0x20u) && (c <= 0x7Eu)) {
            BSP_UART_Printf("  #%u  0x%02X  '%c'\r\n",
                            (unsigned int)n, (unsigned int)c, (char)c);
        } else {
            BSP_UART_Printf("  #%u  0x%02X  <%s>\r\n",
                            (unsigned int)n, (unsigned int)c, test_byte_name(c));
        }
    }

    BSP_UART_Printf("\r\n[uart] 10 秒共收到 %u 个字节\r\n", (unsigned int)n);

    if (n == 0u) {
        /* 一个字节都没收到，这是接线问题的典型特征，把要查的点列清楚 */
        BSP_UART_SendString("[uart] 一个字节都没收到 —— 是【接线】问题，按顺序查：\r\n");
        BSP_UART_SendString("  a) 板子 PA10(USART1_RX) 接【适配器的 TX】，不是接适配器的 RX\r\n");
        BSP_UART_SendString("     （TX 对 RX 要交叉；只接一根 TX 是最常见的错）\r\n");
        BSP_UART_SendString("  b) 板子和适配器【共地】了吗（GND 必须连）\r\n");
        BSP_UART_SendString("  c) 适配器是 3.3V 电平吗？5V 的 TTL 也能收，但别接 RS232 的 ±12V\r\n");
        BSP_UART_SendString("  d) 串口助手是不是选错了 COM 口 / 被别的软件占着\r\n");
        BSP_UART_SendString("[uart] 提示：RX 不通不影响用 OLED 菜单，四按键照样能调参和跑车\r\n");
        return;
    }

    BSP_UART_SendString("[uart] 收到数据了，RX 通路正常。对照上面每一行的十六进制：\r\n");
    BSP_UART_SendString("  0x3F           = 半角 '?'，正确\r\n");
    BSP_UART_SendString("  EF BC 9F 连着来 = 全角「？」，请把输入法切成半角\r\n");
    BSP_UART_SendString("  每行只有一个字节且值很随机 = 适配器/波特率有问题\r\n");
}

void BSP_Test_MenuShow(void)
{
    /* 强制菜单整屏重画：本文件里别的测试（尤其是 OLED 自检）
       直接动过显存，菜单自己察觉不到 */
    APP_Menu_Invalidate();
    APP_Menu_ShowStatus();
    BSP_UART_SendString("[menu] back to the OLED status page\r\n");
}

