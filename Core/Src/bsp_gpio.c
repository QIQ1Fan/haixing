/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @brief   纯 GPIO 汇总层实现
  ******************************************************************************
  */
#include "bsp_gpio.h"

/* ------------------------------ 内部引脚表 ------------------------------ */

/* 六路循迹 */
static GPIO_TypeDef *const s_track_port[TRACK_SENSOR_NUM] = {
    TRACK0_PORT, TRACK1_PORT, TRACK2_PORT,
    TRACK3_PORT, TRACK4_PORT, TRACK5_PORT,
};
static const uint16_t s_track_pin[TRACK_SENSOR_NUM] = {
    TRACK0_PIN, TRACK1_PIN, TRACK2_PIN,
    TRACK3_PIN, TRACK4_PIN, TRACK5_PIN,
};

/* 四路按键：索引顺序需与 bsp_key.h 的 BSP_KeyId_t 保持一致 */
static GPIO_TypeDef *const s_key_port[KEY_NUM] = {
    KEY_OK_PORT, KEY_BACK_PORT, KEY_PLUS_PORT, KEY_MINUS_PORT,
};
static const uint16_t s_key_pin[KEY_NUM] = {
    KEY_OK_PIN, KEY_BACK_PIN, KEY_PLUS_PIN, KEY_MINUS_PIN,
};

/* ------------------------------- 初始化 -------------------------------- */

void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t i;

    /* 时钟：GPIOA/GPIOB 已由 MX_GPIO_Init 打开，这里再开一次是幂等操作。
       AFIO 必须开，否则 SWJ 重映射等 AFIO 寄存器写不进去。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* ---- 1. 电机方向 AIN1/AIN2/BIN1/BIN2 + STBY：推挽输出 ---- */
    /* 初始全低：方向引脚为"停转"，STBY 为"待机"，
       上电时电机不会因为引脚浮空而乱转。 */
    HAL_GPIO_WritePin(MOTOR_DIR_PORT,
                      MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
                      MOTOR_BIN1_PIN | MOTOR_BIN2_PIN,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN;
    HAL_GPIO_Init(MOTOR_DIR_PORT, &gpio);

    gpio.Pin = MOTOR_STBY_PIN;
    HAL_GPIO_Init(MOTOR_STBY_PORT, &gpio);

    /* ---- 2. 六路循迹：上拉输入 ---- */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    for (i = 0; i < TRACK_SENSOR_NUM; i++) {
        gpio.Pin = s_track_pin[i];
        HAL_GPIO_Init(s_track_port[i], &gpio);
    }

    /* ---- 3. 四路按键：上拉输入 ---- */
    for (i = 0; i < KEY_NUM; i++) {
        gpio.Pin = s_key_pin[i];
        HAL_GPIO_Init(s_key_port[i], &gpio);
    }
}

/* ---------------------------- 电机方向与使能 ---------------------------- */

void BSP_GPIO_WriteMotorDir(GPIO_PinState ain1, GPIO_PinState ain2,
                            GPIO_PinState bin1, GPIO_PinState bin2)
{
    HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_AIN1_PIN, ain1);
    HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_AIN2_PIN, ain2);
    HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_BIN1_PIN, bin1);
    HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_BIN2_PIN, bin2);
}

void BSP_GPIO_WriteStby(GPIO_PinState st)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, st);
}

/* ------------------------------- 循迹输入 ------------------------------- */

uint8_t BSP_GPIO_ReadTrackRaw(void)
{
    uint8_t raw = 0u;
    uint32_t i;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        if (HAL_GPIO_ReadPin(s_track_port[i], s_track_pin[i]) == GPIO_PIN_SET) {
            raw |= (uint8_t)(1u << i);
        }
    }
    return raw;
}

/* ------------------------------- 按键输入 ------------------------------- */

uint8_t BSP_GPIO_ReadKeyRaw(uint8_t idx)
{
    if (idx >= KEY_NUM) {
        return 0u;
    }
    /* 上拉输入：按下为低电平 */
    return (HAL_GPIO_ReadPin(s_key_port[idx], s_key_pin[idx]) == KEY_ACTIVE_LEVEL)
           ? 1u : 0u;
}
