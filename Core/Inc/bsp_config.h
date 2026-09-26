/**
  ******************************************************************************
  * @file    bsp_config.h
  * @brief   循迹小车 BSP 层集中配置：引脚映射 + 参数常量
  * @note    这是唯一的硬件抽象入口。换接线 / 换车体 / 调 PWM 频率，只改本文件。
  *
  * 时钟前提（CubeMX 已配置）：
  *   SYSCLK = 72MHz, AHB = 72MHz, APB1 = 36MHz(分频2), APB2 = 72MHz
  *   ★ 关键：APB1 分频系数 != 1 时，APB1 上的定时器时钟会 x2 = 72MHz
  *     所以 TIM2/TIM3/TIM4 的计数时钟都是 72MHz，而 TIM1(APB2) 也是 72MHz。
  ******************************************************************************
  */
#ifndef __BSP_CONFIG_H
#define __BSP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ==========================================================================
 * 1. 舵机 —— TIM1_CH1 / PA8 / 50Hz
 * ========================================================================== */
#define SERVO_TIM                   TIM1
#define SERVO_TIM_CH                TIM_CHANNEL_1
#define SERVO_GPIO_PORT             GPIOA
#define SERVO_GPIO_PIN              GPIO_PIN_8

/* 72MHz / (71+1) = 1MHz 计数频率，即 1 tick = 1us */
#define SERVO_TIM_PSC               (72u - 1u)
/* 1MHz / (19999+1) = 50Hz */
#define SERVO_TIM_ARR               (20000u - 1u)

#define SERVO_PULSE_MIN_US          500u    /* 0.5ms -> 机械一端 */
#define SERVO_PULSE_MAX_US          2500u   /* 2.5ms -> 机械另一端 */
#define SERVO_PULSE_MID_US          1500u   /* 1.5ms -> 中位     */
#define SERVO_ANGLE_MIN             (-90)
#define SERVO_ANGLE_MAX             (90)
#define SERVO_ANGLE_MID             (0)

/* ==========================================================================
 * 2. 电机 —— TIM3_CH1/CH2 / PA6 / PA7 / 20kHz
 * ========================================================================== */
#define MOTOR_TIM                   TIM3
#define MOTOR_CH_L                  TIM_CHANNEL_1   /* PA6 */
#define MOTOR_CH_R                  TIM_CHANNEL_2   /* PA7 */
#define MOTOR_GPIO_PORT             GPIOA
#define MOTOR_GPIO_PIN_L            GPIO_PIN_6
#define MOTOR_GPIO_PIN_R            GPIO_PIN_7

/* 72MHz / (0+1) / (3599+1) = 20000Hz，占空比分辨率 1/3600 */
#define MOTOR_TIM_PSC               (0u)
#define MOTOR_TIM_ARR               (3600u - 1u)
#define MOTOR_PWM_MAX               (MOTOR_TIM_ARR)     /* 满占空比对应值 */

/* 方向控制引脚（TB6612 AIN1/AIN2/BIN1/BIN2）+ 使能 STBY */
#define MOTOR_DIR_PORT              GPIOB
#define MOTOR_AIN1_PIN              GPIO_PIN_0
#define MOTOR_AIN2_PIN              GPIO_PIN_1
#define MOTOR_BIN1_PIN              GPIO_PIN_10
#define MOTOR_BIN2_PIN              GPIO_PIN_11
#define MOTOR_STBY_PORT             GPIOA
#define MOTOR_STBY_PIN              GPIO_PIN_4

/* 若某个轮子转向装反了，把对应宏置 1 即可，不用改接线 */
#define MOTOR_L_INVERT              0
#define MOTOR_R_INVERT              0

/* ==========================================================================
 * 3. 编码器 —— TIM2(PA0/PA1) / TIM4(PB6/PB7)  硬件编码器模式
 * ========================================================================== */
#define ENC_L_TIM                   TIM2
#define ENC_R_TIM                   TIM4
#define ENC_GPIO_PORT_L             GPIOA
#define ENC_GPIO_PIN_LA             GPIO_PIN_0      /* TIM2_CH1 */
#define ENC_GPIO_PIN_LB             GPIO_PIN_1      /* TIM2_CH2 */
#define ENC_GPIO_PORT_R             GPIOB
#define ENC_GPIO_PIN_RA             GPIO_PIN_6      /* TIM4_CH1 */
#define ENC_GPIO_PIN_RB             GPIO_PIN_7      /* TIM4_CH2 */

/* ★ STM32F103 的 TIM2/TIM3/TIM4 全部是 16 位定时器，计数会回绕，取增量时按 int16_t 处理 */
#define ENC_TIM_IS_16BIT            1

/* 编码器测速标定：按你实际电机参数修改 */
#define ENC_PPR                     13.0f   /* 编码器每转脉冲数（单相） */
#define ENC_MULT                    4.0f    /* 四倍频（TI12 模式） */
#define ENC_GEAR_RATIO              30.0f   /* 减速箱减速比 */
/* 输出轴转一圈的总计数 */
#define ENC_CPR                     (ENC_PPR * ENC_MULT * ENC_GEAR_RATIO)

/* 测速采样周期（ms），必须与调用 BSP_Encoder_Update() 的周期一致 */
#define ENC_SAMPLE_PERIOD_MS        10u

/* 编码器输入滤波系数 0..15（0=不滤波，越大越抗噪但会漏掉极窄脉冲）
 * 电机碳刷噪声大，建议 8~12。出现"高速丢计数"就往下调。 */
#define ENC_IC_FILTER               10u

/* 编码器方向反了就置 1 */
#define ENC_L_INVERT                0
#define ENC_R_INVERT                0

/* ==========================================================================
 * 4. 六路循迹 —— GPIO 输入，内部上拉
 * ========================================================================== */
/* 索引 0..5 对应物理位置（默认按 从左到右 排列，装反了把下面数组倒过来即可） */
#define TRACK_SENSOR_NUM            6u

/* ★ 这里必须用纯宏，不能用 GPIO_PIN_RESET：
 *   GPIO_PinState 是 enum 而不是宏，在 #if 里两个标识符都会被预处理成 0，
 *   导致条件恒真。所以用 1/0 表达极性。 */
#define TRACK_ACTIVE_LOW            1   /* 1 = 压到黑线读到低电平（常见），0 = 高电平有效 */

#define TRACK0_PORT                 GPIOA
#define TRACK0_PIN                  GPIO_PIN_2      /* PA2  */
#define TRACK1_PORT                 GPIOA
#define TRACK1_PIN                  GPIO_PIN_3      /* PA3  */
#define TRACK2_PORT                 GPIOA
#define TRACK2_PIN                  GPIO_PIN_5      /* PA5  */
#define TRACK3_PORT                 GPIOA
#define TRACK3_PIN                  GPIO_PIN_11     /* PA11 */
#define TRACK4_PORT                 GPIOA
#define TRACK4_PIN                  GPIO_PIN_12     /* PA12 */
#define TRACK5_PORT                 GPIOB
#define TRACK5_PIN                  GPIO_PIN_5      /* PB5  */

/* 加权位置误差的权重表：索引 0..5 对应传感器 0..5。
 * 误差 = Σ(权重 × 该路是否压线) / 压线个数，范围 [-5.0, +5.0]。
 *   < 0 车偏左   > 0 车偏右   0 居中
 * 若探头装反（左右颠倒），把这张表倒过来写即可。
 * 注意偶数个探头没有物理中点，居中时误差在 ±1 之间跳，这是正常的。 */
#define TRACK_WEIGHT_0              (-5.0f)
#define TRACK_WEIGHT_1              (-3.0f)
#define TRACK_WEIGHT_2              (-1.0f)
#define TRACK_WEIGHT_3              ( 1.0f)
#define TRACK_WEIGHT_4              ( 3.0f)
#define TRACK_WEIGHT_5              ( 5.0f)
/* 误差绝对值上限，用于归一化到 [-1,1] */
#define TRACK_ERROR_MAX             (5.0f)

/* ==========================================================================
 * 5. 独立按键 —— GPIO 输入，内部上拉，按下读低
 * ========================================================================== */
#define KEY_NUM                     4u
/* 按键按下时的电平（运行期比较，枚举可用） */
#define KEY_ACTIVE_LEVEL            GPIO_PIN_RESET

#define KEY_OK_PORT                 GPIOB
#define KEY_OK_PIN                  GPIO_PIN_13     /* 确认 */
#define KEY_BACK_PORT               GPIOB
#define KEY_BACK_PIN                GPIO_PIN_14     /* 退出 */
#define KEY_PLUS_PORT               GPIOB
#define KEY_PLUS_PIN                GPIO_PIN_15     /* 加   */
#define KEY_MINUS_PORT              GPIOA
#define KEY_MINUS_PIN               GPIO_PIN_15     /* 减   */

/* 消抖：连续 N 次扫描电平一致才认可（扫描周期 10ms -> 30ms 消抖） */
#define KEY_SCAN_PERIOD_MS          10u
#define KEY_DEBOUNCE_CNT            3u
/* 长按判定阈值（单位：扫描次数），100 次 x 10ms = 1000ms */
#define KEY_LONG_PRESS_CNT          100u
/* 长按后连发间隔（单位：扫描次数） */
#define KEY_REPEAT_CNT              10u

/* ==========================================================================
 * 6. 蜂鸣器 —— PB3 / 推挽输出 / 低电平触发
 * ========================================================================== */
/* ★ 注意：PB3 是 JTAG 的 JTDO，CubeMX 已在 HAL_MspInit() 里调用
 *   __HAL_AFIO_REMAP_SWJ_NOJTAG() 释放了它，不要删掉那行。 */
#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  GPIO_PIN_3
/* 低电平触发：响 = RESET，静音 = SET */
#define BUZZER_ON_LEVEL             GPIO_PIN_RESET
#define BUZZER_OFF_LEVEL            GPIO_PIN_SET

/* ==========================================================================
 * 7. OLED —— 软件 I2C / PB8=SCL / PB9=SDA
 * ========================================================================== */
#define OLED_SCL_PORT               GPIOB
#define OLED_SCL_PIN                GPIO_PIN_8
#define OLED_SDA_PORT               GPIOB
#define OLED_SDA_PIN                GPIO_PIN_9
#define OLED_I2C_ADDR               0x78u   /* SSD1306 8 位写地址（7 位 0x3C << 1） */
#define OLED_WIDTH                  128u
#define OLED_HEIGHT                 64u
/* 软件 I2C 半周期延时（空循环次数）。屏幕花屏/无显示时调这个值 */
#define OLED_I2C_DELAY_LOOPS        20u

/* ==========================================================================
 * 8. 串口调试 —— USART1 / PA9(TX) / PA10(RX) / 115200
 * ========================================================================== */
#define DBG_UART                    USART1
#define DBG_UART_BAUD               115200u
#define DBG_UART_TX_PORT            GPIOA
#define DBG_UART_TX_PIN             GPIO_PIN_9
#define DBG_UART_RX_PORT            GPIOA
#define DBG_UART_RX_PIN             GPIO_PIN_10
/* printf 缓冲区大小（栈上分配，注意别超过主栈 0x400 = 1KB） */
#define DBG_PRINTF_BUF_SIZE         128u

/* 是否把 fputc 重定向到串口，让标准 printf() 直接可用。
 *   0 = 默认。只用 BSP_UART_Printf()。功能与 printf 等价，不依赖工具链特性，
 *       不屏蔽半主机（Event Recorder / ITM 仍可用），保证一定能编过。
 *   1 = 额外把 fputc 重定向到 USART1，之后可以直接写 printf()。
 *       代价：屏蔽半主机调试。若链接报 _sys_exit / __use_no_semihosting 相关错误，
 *       把它改回 0 即可，BSP_UART_Printf() 不受影响。 */
#define BSP_UART_USE_PRINTF_RETARGET    0

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CONFIG_H */
