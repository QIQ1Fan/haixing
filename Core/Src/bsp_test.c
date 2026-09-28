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
    BSP_UART_SendString("  ?. 重新打印本菜单\r\n");
    BSP_UART_SendString("============================================\r\n");
    BSP_UART_SendString("请输入 1-9 / 0 / ? :\r\n");
}

void BSP_Test_RunCommand(char cmd)
{
    BSP_UART_Printf("\r\n>>> 执行测试 [%c]\r\n", cmd);
    BSP_Buzzer_KeyFeedback();

    switch (cmd) {
    case '1': BSP_Test_Buzzer();     break;
    case '2': BSP_Test_Uart();       break;
    case '3': BSP_Test_Servo();      break;
    case '4': BSP_Test_Motor();      break;
    case '5': BSP_Test_MotorRamp();  break;
    case '6': BSP_Test_Encoder();    break;
    case '7': BSP_Test_Track();      break;
    case '8': BSP_Test_Key();        break;
    case '9': BSP_Test_Oled();       break;
    case '0': BSP_Test_All();        break;
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
    static const int16_t seq[] = { 0, 90, 0, -90, 0 };
    uint32_t i;

    BSP_UART_SendString("[舵机] 中位 -> 右 -> 中位 -> 左 -> 中位\r\n");

    for (i = 0u; i < (sizeof(seq) / sizeof(seq[0])); i++) {
        BSP_Servo_SetAngle((float)seq[i]);
        BSP_UART_Printf("  角度 %+4d deg  脉宽 %4u us\r\n",
                        (int)seq[i], (unsigned int)BSP_Servo_GetPulseUs());
        HAL_Delay(700u);
    }

    BSP_UART_SendString("[舵机] 结束。若只在中间小幅抖动，检查舵机供电是否独立且共地\r\n");
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
        char str[8];
        char rawstr[8];
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

    BSP_UART_SendString("[按键] 5 秒内请按任意键（试试短按和长按）...\r\n");

    while ((uint32_t)(HAL_GetTick() - t0) < 5000u) {
        if ((uint32_t)(HAL_GetTick() - next) >= KEY_SCAN_PERIOD_MS) {
            next += KEY_SCAN_PERIOD_MS;
            BSP_Key_Scan();

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
    }

    BSP_UART_SendString("[按键] 结束。若某个键一直触发，检查是否上拉缺失或按键常闭\r\n");
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
