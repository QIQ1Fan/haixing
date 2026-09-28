/**
  ******************************************************************************
  * @file    bsp_servo.c
  * @brief   舵机驱动实现 —— TIM1_CH1 / PA8 / 50Hz
  *
  * 【数据流】
  *   BSP_Servo_SetAngle(角度)
  *        │  线性插值： -90°→500us，0°→1500us，+90°→2500us
  *        ▼
  *   BSP_Servo_SetPulseUs(脉宽)  ← 也可以从这一层直接进来
  *        │  饱和到 [500, 2500]
  *        ▼
  *   TIM1->CCR1 = 脉宽微秒数
  *        │  硬件比较：计数器 < CCR1 时输出高
  *        ▼
  *   PA8 输出：每 20ms 一个高电平脉冲，宽度 = 设定的微秒数
  *        │
  *        ▼
  *   舵机内部：把脉宽和 1.5ms 比较，差值决定转角
  *
  * 【时基为什么这样配】
  *   72MHz / (71+1) = 1MHz   → 1 个计数 = 1 微秒
  *   1MHz / (19999+1) = 50Hz → 周期 20ms
  *   于是 CCR1 里存的值直接就是"高电平多少微秒"。
  *   调试时开寄存器窗口，看到 CCR1 = 1500 就知道舵机在中位。
  ******************************************************************************
  */
#include "bsp_servo.h"
#include "main.h"       /* Error_Handler() */

static TIM_HandleTypeDef s_htim_servo;
static uint16_t s_pulse_us = SERVO_PULSE_MID_US;

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_Servo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* PA8 → TIM1_CH1，复用推挽。配好之后引脚归 TIM1 管，
       HAL_GPIO_WritePin 对它不再起作用。 */
    gpio.Pin   = SERVO_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio);

    s_htim_servo.Instance               = SERVO_TIM;
    s_htim_servo.Init.Prescaler         = SERVO_TIM_PSC;
    s_htim_servo.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_servo.Init.Period            = SERVO_TIM_ARR;
    s_htim_servo.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_servo.Init.RepetitionCounter = 0u;   /* ★ 见下方说明 */
    s_htim_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim_servo) != HAL_OK) {
        Error_Handler();
    }

    /* ★ RepetitionCounter 只对 TIM1/TIM8 这类高级定时器存在。
       它是"重复计数次数"：每经过 (RCR+1) 个周期才产生一次更新事件。
       舵机需要每 20ms 都有输出，所以必须是 0（不重复）。
       忘了赋这个值、又恰好拿到一块非零的 RAM，舵机就会变成
       每 40ms 才动一下，表现为严重抖动 —— 这是高级定时器的经典坑。
       这里结构体是静态变量（上电清零），但显式写 0 更保险。 */

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = SERVO_PULSE_MID_US;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;     /* 互补通道不用，但结构体要填全 */
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim_servo, &oc, SERVO_TIM_CH) != HAL_OK) {
        Error_Handler();
    }

    /* ★ TIM1 是高级定时器，输出前必须置 MOE（主输出使能）位，
       否则引脚上一点波形都没有。好消息是 HAL_TIM_PWM_Start() 内部
       会自己判断是不是带刹车功能的高级定时器并代劳，不用手动
       __HAL_TIM_MOE_ENABLE()。但要知道这回事 —— 以后若直接操作寄存器
       写 PWM，忘了 MOE 就是"配置全对但没输出"的头号原因。 */
    HAL_TIM_PWM_Start(&s_htim_servo, SERVO_TIM_CH);

    s_pulse_us = SERVO_PULSE_MID_US;
}

/* ==========================================================================
 * 控制接口
 * ========================================================================== */

void BSP_Servo_SetPulseUs(uint16_t pulse_us)
{
    /* 饱和保护：脉宽超范围时舵机会顶到机械限位，堵转发热甚至烧齿轮。
       宁可让它停在极限位置，也不能把无效值写进寄存器。 */
    if (pulse_us < SERVO_PULSE_MIN_US) {
        pulse_us = SERVO_PULSE_MIN_US;
    } else if (pulse_us > SERVO_PULSE_MAX_US) {
        pulse_us = SERVO_PULSE_MAX_US;
    }

    __HAL_TIM_SET_COMPARE(&s_htim_servo, SERVO_TIM_CH, (uint32_t)pulse_us);
    s_pulse_us = pulse_us;
}

void BSP_Servo_SetAngle(float deg)
{
    const float span_pulse = (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    const float span_angle = (float)(SERVO_ANGLE_MAX  -  SERVO_ANGLE_MIN);
    int32_t us;

    /* 先饱和再换算，避免把过大的角度代进公式算出一个超范围的脉宽 */
    if (deg > (float)SERVO_ANGLE_MAX) {
        deg = (float)SERVO_ANGLE_MAX;
    } else if (deg < (float)SERVO_ANGLE_MIN) {
        deg = (float)SERVO_ANGLE_MIN;
    }

    /* 线性插值：中位 + 偏差×(脉宽跨度 / 角度跨度)
       即 1500us + deg × (2000/180)，约每度 11.1 微秒 */
    us = (int32_t)((float)SERVO_PULSE_MID_US + deg * span_pulse / span_angle);

    BSP_Servo_SetPulseUs((uint16_t)us);
}

uint16_t BSP_Servo_GetPulseUs(void)
{
    return s_pulse_us;
}

float BSP_Servo_GetAngle(void)
{
    const float span_pulse = (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    const float span_angle = (float)(SERVO_ANGLE_MAX  -  SERVO_ANGLE_MIN);
    float d = (float)s_pulse_us - (float)SERVO_PULSE_MID_US;

    /* 由脉宽反算角度，而不是另外存一个 s_angle 变量。
       这样即使有人绕过 SetAngle 直接调 SetPulseUs，读回来的角度
       也永远是最新的，不会读到过期的缓存值。 */
    return d * span_angle / span_pulse;
}
