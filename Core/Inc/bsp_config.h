/**
  ******************************************************************************
  * @file    bsp_config.h
  * @brief   ★ 工程总入口：硬件资源分配 + 所有可调参数
  *
  * ============================================================================
  *  一、这个工程是什么
  * ============================================================================
  *  基于 STM32F103C8T6 的六路循迹小车。两种转向方案二选一：
  *    · 差速转向 —— 左右轮转速不同，靠 BSP_Motor_SetSpeed() 的左右差值
  *    · 舵机转向 —— 前轮舵机打角，靠 BSP_Servo_SetAngle()
  *  双路编码器提供速度反馈，OLED 显示，四按键调参，串口打印调试。
  *
  * ============================================================================
  *  二、外设怎么分（一个外设只干一件事，互不打架）
  * ============================================================================
  *  定时器类：
  *    TIM1  CH1  PA8         舵机 PWM                     50Hz
  *    TIM3  CH1  PA6         左电机 PWM                   20kHz
  *    TIM3  CH2  PA7         右电机 PWM                   20kHz
  *    TIM2  CH1/2 PA0/PA1    左编码器 A/B 相（硬件四倍频） 不计频
  *    TIM4  CH1/2 PB6/PB7    右编码器 A/B 相（硬件四倍频） 不计频
  *    SysTick                10ms 主循环节拍 + HAL_Delay
  *
  *  串口类：
  *    USART1     PA9/PA10    调试打印 / 蓝牙模块           115200-8-N-1
  *
  *  纯 GPIO 类：
  *    PB0  PB1     TB6612 AIN1/AIN2   左电机方向
  *    PB10 PB11    TB6612 BIN1/BIN2   右电机方向
  *    PA4          TB6612 STBY        高=工作 低=待机
  *    PA2 PA3 PA5 PA11 PA12 PB5        六路循迹（内部上拉输入）
  *    PB13 PB14 PB15 PA15              四按键：确认/退出/加/减
  *    PB3          蜂鸣器（低电平触发）
  *    PB8 PB9      OLED 软件 I2C（SCL/SDA）
  *    PA13 PA14    SWD 调试口（不能动）
  *
  *  ★ PA15 和 PB3 默认是 JTAG 的 JTDI / JTDO，不是普通 GPIO。
  *    CubeMX 已在 HAL_MspInit() 里调用 __HAL_AFIO_REMAP_SWJ_NOJTAG()
  *    把它们释放成 GPIO（同时保留 PA13/PA14 的 SWD 下载）。删掉那行，
  *    这两个脚就彻底写不进电平。
  *
  * ============================================================================
  *  三、模块依赖（箭头 = 谁调用谁）
  * ============================================================================
  *                        ┌──────────────────────────────┐
  *                        │  main.c  10ms 节拍器          │
  *                        │  Encoder_Update / Key_Scan /  │
  *                        │  Buzzer_Poll                  │
  *                        └───────────────┬──────────────┘
  *                                        │
  *        ┌───────────────┬───────────────┼───────────────┬──────────────┐
  *        ▼               ▼               ▼               ▼              ▼
  *   bsp_encoder     bsp_track       bsp_key         bsp_servo     bsp_buzzer
  *   （速度反馈）    （位置误差）    （按键事件）    （转向角）     （提示音）
  *        │               │               │
  *        │               ▼               ▼
  *        │         ┌───────────┐   bsp_oled（菜单显示）
  *        │         │   PID     │
  *        │         └─────┬─────┘
  *        │               ▼
  *        └────────► bsp_motor ──► TB6612 ──► 左右电机
  *                   （PWM+方向）
  *
  *   公共底座：bsp_gpio —— 所有"纯 GPIO"引脚的初始化与裸读写，
  *             被 bsp_motor / bsp_track / bsp_key 共用。
  *   调试入口：bsp_test —— 串口菜单驱动的九项独立自检。
  *
  * ============================================================================
  *  四、初始化顺序（main.c 里按此顺序调用，原因见各模块注释）
  * ============================================================================
  *   UART → GPIO → Buzzer → Key → Track → Encoder → Servo → Motor → OLED
  *   · UART 最先：后面任何模块出问题，报错才打得出来
  *   · Motor 靠后且 STBY 最后使能：避免上电瞬间电机乱冲
  *   · OLED 最后：它自带 100ms 上电等待，不拖累前面的模块
  *
  * ============================================================================
  *  五、怎么用
  * ============================================================================
  *   1. 硬件对不上时，只改本文件里的宏，别的地方不用动
  *   2. Keil 编译下载，串口 115200 打开，按 1~9 逐项验硬件
  *   3. 底层全绿后，把 main.c 主循环里的测试代码换成 PID 就能跑车
  *
  * ============================================================================
  *  六、时钟前提（CubeMX 已配好，这里只是说明为什么下面的公式成立）
  * ============================================================================
  *   HSE 8MHz → PLL×9 → SYSCLK = 72MHz
  *   AHB  = 72MHz
  *   APB1 = 36MHz（HCLK 分频 2）→ 挂在 APB1 上的 TIM2/TIM3/TIM4 时钟 = 72MHz
  *   APB2 = 72MHz                → 挂在 APB2 上的 TIM1 时钟      = 72MHz
  *
  *   ★ 关键点：APB1 分频系数只要不等于 1，STM32 会自动把该总线上
  *     定时器的时钟再乘 2。所以 TIM2/TIM3/TIM4 虽然挂在 36MHz 的 APB1 上，
  *     实际计数时钟仍是 72MHz。算 PWM 频率时要用 72MHz，不是 36MHz。
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
 * --------------------------------------------------------------------------
 * 数据流： BSP_Servo_SetAngle(角度) → 换算成脉宽 → CCR 寄存器 → PA8 输出方波
 *          → 舵机内部比较脉宽 → 转到对应角度
 *
 * 为什么是 50Hz：舵机（航模舵机）的标准控制协议就是每 20ms 收到一个脉冲，
 * 脉冲高电平 0.5~2.5ms 对应左右两个极限位置。频率不对舵机会抖或不响应。
 * ========================================================================== */
#define SERVO_TIM                   TIM1
#define SERVO_TIM_CH                TIM_CHANNEL_1
#define SERVO_GPIO_PORT             GPIOA
#define SERVO_GPIO_PIN              GPIO_PIN_8

/* 分频后计数频率 72MHz/(71+1) = 1MHz，即 1 个计数 = 1 微秒。
 * 这样 CCR 里填的值直接就是脉宽微秒数，调试时看寄存器就能读脉宽。 */
#define SERVO_TIM_PSC               (72u - 1u)
/* 1MHz/(19999+1) = 50Hz，正好 20ms 一个周期 */
#define SERVO_TIM_ARR               (20000u - 1u)

/* 舵机的机械行程标定。换舵机型号若出现"打不满角度"或"顶到限位嗡嗡响"，
 * 改这两个值。标准舵机是 500/2500，有些小舵机是 1000/2000。 */
#define SERVO_PULSE_MIN_US          500u    /* 对应 SERVO_ANGLE_MIN */
#define SERVO_PULSE_MAX_US          2500u   /* 对应 SERVO_ANGLE_MAX */
#define SERVO_PULSE_MID_US          1500u   /* 中位 */
#define SERVO_ANGLE_MIN             (-90)
#define SERVO_ANGLE_MAX             (90)
#define SERVO_ANGLE_MID             (0)

/* ==========================================================================
 * 2. 电机 —— TIM3_CH1/CH2 / PA6 / PA7 / 20kHz
 * --------------------------------------------------------------------------
 * 数据流： BSP_Motor_SetSpeed(带符号速度)
 *            ├─ 符号位 → 决定 AIN1/AIN2 电平组合 → TB6612 输出极性
 *            └─ 绝对值 → 写进 CCR → 决定 PWM 占空比 → TB6612 输出电压
 *
 * 为什么是 20kHz：超过人耳听觉上限电机就不发出刺耳啸叫；低于 16kHz
 * 能听到尖锐噪音。20kHz 是"噪音"与"MOS 开关损耗"之间的常用折中。
 * ========================================================================== */
#define MOTOR_TIM                   TIM3
#define MOTOR_CH_L                  TIM_CHANNEL_1   /* PA6 */
#define MOTOR_CH_R                  TIM_CHANNEL_2   /* PA7 */
#define MOTOR_GPIO_PORT             GPIOA
#define MOTOR_GPIO_PIN_L            GPIO_PIN_6
#define MOTOR_GPIO_PIN_R            GPIO_PIN_7

/* PSC=0 → 计数时钟就是 72MHz；ARR=3599 → 72MHz/3600 = 20kHz。
 * 占空比分辨率 = 1/3600 ≈ 0.028%。若图省事取 PSC=71（1MHz 计数），
 * ARR 只剩 999，分辨率掉到 0.1%，低速时能看出明显台阶感。 */
#define MOTOR_TIM_PSC               (0u)
#define MOTOR_TIM_ARR               (3600u - 1u)
#define MOTOR_PWM_MAX               (MOTOR_TIM_ARR)     /* 满占空比对应值 */

/* TB6612 方向引脚 + 使能 */
#define MOTOR_DIR_PORT              GPIOB
#define MOTOR_AIN1_PIN              GPIO_PIN_0
#define MOTOR_AIN2_PIN              GPIO_PIN_1
#define MOTOR_BIN1_PIN              GPIO_PIN_10
#define MOTOR_BIN2_PIN              GPIO_PIN_11
#define MOTOR_STBY_PORT             GPIOA
#define MOTOR_STBY_PIN              GPIO_PIN_4

/* 轮子转向装反了不用重新接线，把对应宏改成 1 即可（先跑测试 4 看方向） */
#define MOTOR_L_INVERT              0
#define MOTOR_R_INVERT              0

/* ==========================================================================
 * 3. 编码器 —— TIM2(PA0/PA1) / TIM4(PB6/PB7)   硬件编码器模式
 * --------------------------------------------------------------------------
 * 数据流： 电机转 → 编码器出 A/B 两相方波 → 定时器硬件四倍频自动计数（不耗 CPU）
 *          → bsp_encoder 每 10ms 读一次计数差 → s_delta[] → PID 速度环
 *
 * 为什么用定时器的编码器模式而不是外部中断：四倍频后脉冲频率很高
 * （减速比 30 时每转上千个计数），用中断会吃掉大量 CPU；硬件模式下
 * 计数完全由定时器外设完成，CPU 只负责周期性读一次数值。
 * ========================================================================== */
#define ENC_L_TIM                   TIM2
#define ENC_R_TIM                   TIM4
#define ENC_GPIO_PORT_L             GPIOA
#define ENC_GPIO_PIN_LA             GPIO_PIN_0      /* TIM2_CH1 */
#define ENC_GPIO_PIN_LB             GPIO_PIN_1      /* TIM2_CH2 */
#define ENC_GPIO_PORT_R             GPIOB
#define ENC_GPIO_PIN_RA             GPIO_PIN_6      /* TIM4_CH1 */
#define ENC_GPIO_PIN_RB             GPIO_PIN_7      /* TIM4_CH2 */

/* ★ STM32F103 的 TIM2~TIM5 全是 16 位计数器（只有 F4/F7 的 TIM2/TIM5 是 32 位），
 *   计数到 65535 会回绕。bsp_encoder 用 (int16_t)(now - last) 处理回绕，
 *   前提是单个采样周期内位移不超过 ±32767 个计数。 */
#define ENC_TIM_IS_16BIT            1

/* --- 测速标定：换电机/换编码器/换轮子都要改这里 --- */
#define ENC_PPR                     13.0f   /* 编码器每转脉冲数（单相） */
#define ENC_MULT                    4.0f    /* 四倍频：A/B 两相各上下沿都计数 */
#define ENC_GEAR_RATIO              30.0f   /* 减速箱减速比 */
/* 输出轴（轮子）转一圈的总计数。转速公式的分母就是它 */
#define ENC_CPR                     (ENC_PPR * ENC_MULT * ENC_GEAR_RATIO)

/* 测速采样周期，必须与 main.c 主循环节拍以及 BSP_Encoder_Update() 的
 * 调用周期三者一致，否则算出来的转速会成比例偏大或偏小 */
#define ENC_SAMPLE_PERIOD_MS        10u

/* 编码器输入滤波系数 0..15（0=不滤波）。电机碳刷噪声大，建议 8~12。
 * 出现"高速丢计数"就往下调；出现"轮子没动计数却在跳"就往上调。 */
#define ENC_IC_FILTER               10u

/* 编码器方向反了（比如差速时左右轮读数符号相反）置 1 */
#define ENC_L_INVERT                0
#define ENC_R_INVERT                0

/* ==========================================================================
 * 4. 六路循迹 —— GPIO 上拉输入
 * --------------------------------------------------------------------------
 * 数据流： 探头 → 电平 → BSP_Track_GetMask() 位掩码
 *          → 加权求和 → BSP_Track_GetError() 位置误差 → PID 转向环
 *
 * 索引约定：索引 0..5 默认对应对着车头从左到右。装反了不用改接线，
 * 把下面 TRACK_WEIGHT_0..5 这六个值倒过来写即可。
 * ========================================================================== */
#define TRACK_SENSOR_NUM            6u

/* ★ 这里必须用纯宏 1/0，不能写 GPIO_PIN_RESET：
 *   GPIO_PinState 是 enum 而不是 #define，在 #if 预处理里两个标识符
 *   都会被当成未定义、一律替换成 0，导致条件恒真、分支永远进同一边。
 *   bsp_track.c 里就是用 #if TRACK_ACTIVE_LOW 选分支的。 */
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

/* 加权位置误差的权重表，索引 0..5 对应传感器 0..5。
 * 计算方式：误差 = Σ(权重 × 该路是否压线) / 压线个数，量程 [-5.0, +5.0]
 *    误差 < 0 → 车偏左（需要右转）     误差 > 0 → 车偏右（需要左转）
 *
 * 注意：6 是偶数，探头阵列没有物理中点，所以车正对线时误差会在 ±1
 * 之间跳动而不是稳定的 0，这是正常的，不用去"修"。 */
#define TRACK_WEIGHT_0              (-5.0f)
#define TRACK_WEIGHT_1              (-3.0f)
#define TRACK_WEIGHT_2              (-1.0f)
#define TRACK_WEIGHT_3              ( 1.0f)
#define TRACK_WEIGHT_4              ( 3.0f)
#define TRACK_WEIGHT_5              ( 5.0f)
/* 误差绝对值上限，用于把误差归一化到 [-1,1] 喂给 PID */
#define TRACK_ERROR_MAX             (5.0f)

/* ==========================================================================
 * 5. 四路按键 —— GPIO 上拉输入，按下读低
 * --------------------------------------------------------------------------
 * 数据流： 引脚电平 → BSP_Key_Scan() 每 10ms 消抖一次
 *          → 短按/长按/连发事件 → BSP_Key_GetEvent() 读走
 *          → 菜单处理 / 参数加减
 * ========================================================================== */
#define KEY_NUM                     4u
/* 按下时的电平。上拉输入 → 按下是低电平。这里只在运行期做比较，用枚举没问题 */
#define KEY_ACTIVE_LEVEL            GPIO_PIN_RESET

#define KEY_OK_PORT                 GPIOB
#define KEY_OK_PIN                  GPIO_PIN_13     /* 确认 */
#define KEY_BACK_PORT               GPIOB
#define KEY_BACK_PIN                GPIO_PIN_14     /* 退出 */
#define KEY_PLUS_PORT               GPIOB
#define KEY_PLUS_PIN                GPIO_PIN_15     /* 加   */
#define KEY_MINUS_PORT              GPIOA
#define KEY_MINUS_PIN               GPIO_PIN_15     /* 减   */

/* 消抖与长按的时间参数，单位都是"扫描次数"，一次扫描 = KEY_SCAN_PERIOD_MS */
#define KEY_SCAN_PERIOD_MS          10u
#define KEY_DEBOUNCE_CNT            3u      /* 连续 3 次读数一致才认可 = 30ms 消抖 */
#define KEY_LONG_PRESS_CNT          100u    /* 100 × 10ms = 按住 1 秒算长按 */
#define KEY_REPEAT_CNT              10u     /* 长按后每 10 × 10ms = 100ms 连发一次 */

/* ==========================================================================
 * 6. 蜂鸣器 —— PB3 / 推挽输出 / 低电平触发
 * --------------------------------------------------------------------------
 * 数据流： BSP_Buzzer_On/Off → PB3 电平 → 蜂鸣器响或不响
 *
 * ★ PB3 是 JTAG 的 JTDO。CubeMX 已在 HAL_MspInit() 里调用
 *   __HAL_AFIO_REMAP_SWJ_NOJTAG() 释放了它。如果哪天把那次调用删了，
 *   PB3 会写不进电平，蜂鸣器就完全没反应，而且是静默失败、很难查。
 * ========================================================================== */
#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  GPIO_PIN_3
/* 低电平触发：响 = RESET，静音 = SET。
 * 如果你的蜂鸣器模块是高电平触发，把这两个值对调即可。 */
#define BUZZER_ON_LEVEL             GPIO_PIN_RESET
#define BUZZER_OFF_LEVEL            GPIO_PIN_SET

/* ==========================================================================
 * 7. OLED —— 软件 I2C / PB8=SCL / PB9=SDA
 * --------------------------------------------------------------------------
 * 数据流： 绘制函数 → 改 1KB 显存 s_buf[] → BSP_OLED_Refresh()
 *          → 软件 I2C 整屏 1024 字节 → SSD1306 显示
 *
 * 为什么要显存：如果每画一个点就直接发 I2C，屏幕会逐点刷新出现撕裂感，
 * 而且 I2C 通信次数暴增。先在内存里拼好整幅图再一次性送出，屏幕上是
 * 完整的一帧，观感好得多。
 * ========================================================================== */
#define OLED_SCL_PORT               GPIOB
#define OLED_SCL_PIN                GPIO_PIN_8
#define OLED_SDA_PORT               GPIOB
#define OLED_SDA_PIN                GPIO_PIN_9
/* SSD1306 的 8 位写地址（7 位地址 0x3C 左移一位）。
 * 如果屏幕全黑但接线没问题，试试改成 0x7A（7 位 0x3D）。 */
#define OLED_I2C_ADDR               0x78u
#define OLED_WIDTH                  128u
#define OLED_HEIGHT                 64u
/* 软件 I2C 的半周期延时（空循环次数）。
 * 屏幕花屏 / 无显示 / 偶发乱码时调这个值：太小则时序过快，太大则刷屏卡顿。 */
#define OLED_I2C_DELAY_LOOPS        20u

/* ==========================================================================
 * 8. 串口调试 —— USART1 / PA9(TX) / PA10(RX) / 115200
 * --------------------------------------------------------------------------
 * 数据流： BSP_UART_Printf() → vsnprintf 格式化 → HAL_UART_Transmit 阻塞发送
 *          接收方向：主循环轮询 BSP_UART_GetChar()，不占用中断
 * ========================================================================== */
#define DBG_UART                    USART1
#define DBG_UART_BAUD               115200u
#define DBG_UART_TX_PORT            GPIOA
#define DBG_UART_TX_PIN             GPIO_PIN_9
#define DBG_UART_RX_PORT            GPIOA
#define DBG_UART_RX_PIN             GPIO_PIN_10
/* printf 缓冲区大小。注意它分配在栈上，别超过主栈（CubeMX 配的 0x400 = 1KB） */
#define DBG_PRINTF_BUF_SIZE         128u

/* 是否把 fputc 重定向到串口，让标准 printf() 直接可用。
 *   0 = 默认。只用 BSP_UART_Printf()。功能与 printf 等价，不依赖工具链特性，
 *       不屏蔽半主机（Event Recorder / ITM 仍可用），保证一定能编过。
 *   1 = 额外把 fputc 重定向到 USART1，之后可以直接写 printf()。
 *       代价：屏蔽半主机调试。若链接报 _sys_exit / __use_no_semihosting
 *       相关错误，把它改回 0 即可，BSP_UART_Printf() 不受影响。 */
#define BSP_UART_USE_PRINTF_RETARGET    0

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CONFIG_H */
