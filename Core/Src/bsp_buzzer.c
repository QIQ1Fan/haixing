/**
  ******************************************************************************
  * @file    bsp_buzzer.c
  * @brief   蜂鸣器实现
  ******************************************************************************
  */
#include "bsp_buzzer.h"

#define BUZZER_KEY_BEEP_MS      30u     /* 按键反馈时长 */
#define BUZZER_ALARM_ON_MS      150u    /* 报警单次鸣叫时长 */
#define BUZZER_ALARM_OFF_MS     100u    /* 报警间隔 */

static uint8_t  s_async_on = 0u;        /* 非阻塞鸣叫是否在进行中 */
static uint32_t s_off_tick = 0u;        /* 该关声的时刻 */

/* ------------------------------- 初始化 -------------------------------- */

void BSP_Buzzer_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ★ 先写电平再配方向，避免初始化瞬间产生一声异响 */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_OFF_LEVEL);

    gpio.Pin   = BUZZER_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);

    s_async_on = 0u;
}

/* ------------------------------- 基础控制 ------------------------------ */

void BSP_Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_ON_LEVEL);
}

void BSP_Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_OFF_LEVEL);
    s_async_on = 0u;
}

void BSP_Buzzer_Toggle(void)
{
    HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

/* ------------------------------- 鸣叫接口 ------------------------------ */

void BSP_Buzzer_Beep(uint16_t ms)
{
    BSP_Buzzer_On();
    HAL_Delay(ms);
    BSP_Buzzer_Off();
}

void BSP_Buzzer_BeepAsync(uint16_t ms)
{
    /* 防止 ms=0 时立刻被 Poll 关掉 */
    if (ms == 0u) {
        ms = 1u;
    }
    BSP_Buzzer_On();
    s_off_tick = HAL_GetTick() + (uint32_t)ms;
    s_async_on = 1u;
}

void BSP_Buzzer_Poll(void)
{
    if (s_async_on == 0u) {
        return;
    }
    /* 用有符号差值比较，HAL_GetTick() 回绕时依然正确 */
    if ((int32_t)(HAL_GetTick() - s_off_tick) >= 0) {
        BSP_Buzzer_Off();
    }
}

void BSP_Buzzer_KeyFeedback(void)
{
    BSP_Buzzer_BeepAsync(BUZZER_KEY_BEEP_MS);
}

void BSP_Buzzer_Alarm(uint16_t times)
{
    uint16_t i;

    for (i = 0u; i < times; i++) {
        BSP_Buzzer_On();
        HAL_Delay(BUZZER_ALARM_ON_MS);
        BSP_Buzzer_Off();
        if (i + 1u < times) {
            HAL_Delay(BUZZER_ALARM_OFF_MS);
        }
    }
}
