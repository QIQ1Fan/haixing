/**
  ******************************************************************************
  * @file    bsp_motor.c
  * @brief   直流电机驱动实现
  *
  * TB6612 真值表：
  *   AIN1  AIN2   输出
  *    0     1     正转
  *    1     0     反转
  *    0     0     滑行（输出高阻）
  *    1     1     刹车（绕组短接）
  ******************************************************************************
  */
#include "bsp_motor.h"
#include "bsp_gpio.h"
#include "main.h"       /* Error_Handler() */

static TIM_HandleTypeDef s_htim_motor;
static int16_t s_speed_l = 0;
static int16_t s_speed_r = 0;

/* ------------------------------ 内部工具 -------------------------------- */

/**
  * @brief  设定单个电机的方向与占空比
  * @param  dir_pin_fwd  该电机"正转"时需拉高的方向引脚
  * @param  dir_pin_rev  该电机"反转"时需拉高的方向引脚
  * @param  ch           PWM 通道
  * @param  speed        带符号速度（已饱和处理）
  */
static void motor_apply(uint16_t dir_pin_fwd, uint16_t dir_pin_rev,
                        uint32_t ch, int16_t speed)
{
    uint32_t duty;

    if (speed > 0) {
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_fwd, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_rev, GPIO_PIN_RESET);
        duty = (uint32_t)speed;
    } else if (speed < 0) {
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_fwd, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, dir_pin_rev, GPIO_PIN_SET);
        duty = (uint32_t)(-speed);
    } else {
        /* 滑行：两引脚同时拉低 */
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, (uint32_t)(dir_pin_fwd | dir_pin_rev),
                          GPIO_PIN_RESET);
        duty = 0u;
    }

    __HAL_TIM_SET_COMPARE(&s_htim_motor, ch, duty);
}

/** @brief 把速度饱和到 ±MOTOR_PWM_MAX，避免取负时溢出 */
static int16_t motor_saturate(int16_t v)
{
    if (v > (int16_t)MOTOR_PWM_MAX) {
        return (int16_t)MOTOR_PWM_MAX;
    }
    if (v < -(int16_t)MOTOR_PWM_MAX) {
        return -(int16_t)MOTOR_PWM_MAX;
    }
    return v;
}

/* ------------------------------- 初始化 -------------------------------- */

void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};

    /* --- 1. PWM 引脚 PA6/PA7 复用推挽输出 --- */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Pin   = MOTOR_GPIO_PIN_L | MOTOR_GPIO_PIN_R;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_GPIO_PORT, &gpio);

    /* --- 2. TIM3 时基：72MHz / 3600 = 20kHz --- */
    s_htim_motor.Instance               = MOTOR_TIM;
    s_htim_motor.Init.Prescaler         = MOTOR_TIM_PSC;
    s_htim_motor.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_motor.Init.Period            = MOTOR_TIM_ARR;
    s_htim_motor.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_motor.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim_motor) != HAL_OK) {
        Error_Handler();
    }

    /* --- 3. 两路 PWM 通道，初值 0 占空比 --- */
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0u;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim_motor, &oc, MOTOR_CH_L) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&s_htim_motor, &oc, MOTOR_CH_R) != HAL_OK) {
        Error_Handler();
    }

    /* --- 4. 启动输出并进入停止状态 --- */
    HAL_TIM_PWM_Start(&s_htim_motor, MOTOR_CH_L);
    HAL_TIM_PWM_Start(&s_htim_motor, MOTOR_CH_R);

    s_speed_l = 0;
    s_speed_r = 0;
    BSP_Motor_Stop();

    /* --- 5. 使能 TB6612 --- */
    BSP_Motor_Enable(1u);
}

/* ------------------------------- 控制接口 ------------------------------ */

void BSP_Motor_Enable(uint8_t enable)
{
    BSP_GPIO_WriteStby(enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (!enable) {
        BSP_Motor_Stop();
    }
}

void BSP_Motor_SetSpeed(int16_t left, int16_t right)
{
    int16_t l = motor_saturate(left);
    int16_t r = motor_saturate(right);

#if MOTOR_L_INVERT
    l = (int16_t)(-l);
#endif
#if MOTOR_R_INVERT
    r = (int16_t)(-r);
#endif

    /* 左轮 = A 通道，正转时 AIN2 拉高 */
    motor_apply(MOTOR_AIN2_PIN, MOTOR_AIN1_PIN, MOTOR_CH_L, l);
    /* 右轮 = B 通道，正转时 BIN2 拉高 */
    motor_apply(MOTOR_BIN2_PIN, MOTOR_BIN1_PIN, MOTOR_CH_R, r);

    s_speed_l = l;
    s_speed_r = r;
}

void BSP_Motor_SetDuty(uint16_t duty_l, uint16_t duty_r)
{
    if (duty_l > (uint16_t)MOTOR_PWM_MAX) {
        duty_l = (uint16_t)MOTOR_PWM_MAX;
    }
    if (duty_r > (uint16_t)MOTOR_PWM_MAX) {
        duty_r = (uint16_t)MOTOR_PWM_MAX;
    }
    __HAL_TIM_SET_COMPARE(&s_htim_motor, MOTOR_CH_L, (uint32_t)duty_l);
    __HAL_TIM_SET_COMPARE(&s_htim_motor, MOTOR_CH_R, (uint32_t)duty_r);
}

void BSP_Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_DIR_PORT,
                      MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
                      MOTOR_BIN1_PIN | MOTOR_BIN2_PIN,
                      GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&s_htim_motor, MOTOR_CH_L, 0u);
    __HAL_TIM_SET_COMPARE(&s_htim_motor, MOTOR_CH_R, 0u);
    s_speed_l = 0;
    s_speed_r = 0;
}

void BSP_Motor_Brake(void)
{
    HAL_GPIO_WritePin(MOTOR_DIR_PORT,
                      MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
                      MOTOR_BIN1_PIN | MOTOR_BIN2_PIN,
                      GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&s_htim_motor, MOTOR_CH_L, 0u);
    __HAL_TIM_SET_COMPARE(&s_htim_motor, MOTOR_CH_R, 0u);
    s_speed_l = 0;
    s_speed_r = 0;
}

void BSP_Motor_GetSpeed(int16_t *left, int16_t *right)
{
    if (left  != NULL) { *left  = s_speed_l; }
    if (right != NULL) { *right = s_speed_r; }
}
