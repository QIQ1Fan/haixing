/**
  ******************************************************************************
  * @file    bsp_servo.c
  * @brief   舵机驱动实现 —— TIM1_CH1 / PA8 / 50Hz
  *
  * 时基：72MHz / (71+1) = 1MHz  ->  1 个计数 = 1us
  *       1MHz / (19999+1) = 50Hz
  * 因此 CCR 的值直接就是"高电平微秒数"，调试时看 CCR 就是看脉宽，非常直观。
  ******************************************************************************
  */
#include "bsp_servo.h"
#include "main.h"       /* Error_Handler() */

static TIM_HandleTypeDef s_htim_servo;
static uint16_t s_pulse_us = SERVO_PULSE_MID_US;

/* ------------------------------- 初始化 -------------------------------- */

void BSP_Servo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* PA8 -> TIM1_CH1 复用推挽 */
    gpio.Pin   = SERVO_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio);

    /* TIM1 时基：1MHz 计数 -> 20ms 周期 */
    s_htim_servo.Instance               = SERVO_TIM;
    s_htim_servo.Init.Prescaler         = SERVO_TIM_PSC;
    s_htim_servo.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_servo.Init.Period            = SERVO_TIM_ARR;
    s_htim_servo.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_servo.Init.RepetitionCounter = 0u;   /* ★ 高级定时器必须显式赋值 */
    s_htim_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim_servo) != HAL_OK) {
        Error_Handler();
    }

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = SERVO_PULSE_MID_US;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim_servo, &oc, SERVO_TIM_CH) != HAL_OK) {
        Error_Handler();
    }

    /* 高级定时器的 MOE 位由 HAL_TIM_PWM_Start 内部处理 */
    HAL_TIM_PWM_Start(&s_htim_servo, SERVO_TIM_CH);

    s_pulse_us = SERVO_PULSE_MID_US;
}

/* ------------------------------- 控制接口 ------------------------------ */

void BSP_Servo_SetPulseUs(uint16_t pulse_us)
{
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

    if (deg > (float)SERVO_ANGLE_MAX) {
        deg = (float)SERVO_ANGLE_MAX;
    } else if (deg < (float)SERVO_ANGLE_MIN) {
        deg = (float)SERVO_ANGLE_MIN;
    }

    /* 线性插值：-90deg -> 500us, +90deg -> 2500us */
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

    return d * span_angle / span_pulse;
}
