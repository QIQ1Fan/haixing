/**
  ******************************************************************************
  * @file    bsp_buzzer.c
  * @brief   蜂鸣器实现 —— PB3 / 低电平触发
  *
  * 【数据流（阻塞路径）】
  *   BSP_Buzzer_Beep(ms) → On() → HAL_Delay(ms) → Off()
  *     简单直接，但 Delay 期间 CPU 完全空转，不能用在控制循环里
  *
  * 【数据流（非阻塞路径）】
  *   BSP_Buzzer_BeepAsync(ms)
  *        ├─ 立即 On()
  *        └─ 记下 s_off_tick = 当前时刻 + ms
  *              │
  *   主循环每轮 ──► BSP_Buzzer_Poll()
  *                    └─ 若已到 s_off_tick → Off()
  *
  *   这样"什么时候关声"变成了主循环的一个顺便检查，不占用任何等待时间。
  *   主循环里还会调用 Encoder_Update 和 Key_Scan，三者共用同一个循环。
  *
  * 【为什么 tick 比较要用有符号减法】
  *   HAL_GetTick() 是 32 位，跑到 49.7 天会回绕归零。
  *   若写成 if (HAL_GetTick() >= s_off_tick)，回绕瞬间会判断失败，
  *   蜂鸣器就一直响下去。
  *   写成 if ((int32_t)(HAL_GetTick() - s_off_tick) >= 0) 则天然正确：
  *   无符号相减处理回绕，再按有符号解释就得到"过了多少毫秒"。
  ******************************************************************************
  */
#include "bsp_buzzer.h"

/* 各种提示音的时长参数，想调手感改这里 */
#define BUZZER_KEY_BEEP_MS      30u     /* 按键反馈：短促一声 */
#define BUZZER_ALARM_ON_MS      150u    /* 报警：单次鸣叫时长 */
#define BUZZER_ALARM_OFF_MS     100u    /* 报警：两声之间的间隔 */

static uint8_t  s_async_on = 0u;        /* 非阻塞鸣叫是否正在进行中 */
static uint32_t s_off_tick = 0u;        /* 该关声的时刻 */

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_Buzzer_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ★ 先写电平、再配方向。
       反过来的话，引脚从输入切成输出输出的那一刻，会先输出引脚上
       残留的旧值（可能是低 = 触发），产生一声"咔"的异响。 */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_OFF_LEVEL);

    gpio.Pin   = BUZZER_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;   /* 开关量，不需要很快的边沿 */
    HAL_GPIO_Init(BUZZER_PORT, &gpio);

    s_async_on = 0u;
}

/* ==========================================================================
 * 基础控制
 * ========================================================================== */

void BSP_Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_ON_LEVEL);
}

void BSP_Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_OFF_LEVEL);

    /* 同时取消可能正在进行的非阻塞鸣叫。
       否则手动关声之后，Poll() 还会在原本的到点时刻再关一次（无害），
       但更重要的是状态标志要一致，避免下次 BeepAsync 的计时被干扰 */
    s_async_on = 0u;
}

void BSP_Buzzer_Toggle(void)
{
    HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

/* ==========================================================================
 * 鸣叫接口
 * ========================================================================== */

void BSP_Buzzer_Beep(uint16_t ms)
{
    BSP_Buzzer_On();
    HAL_Delay(ms);
    BSP_Buzzer_Off();
}

void BSP_Buzzer_BeepAsync(uint16_t ms)
{
    /* 防止 ms = 0 时"刚记下到点时刻就已经过期"，
       导致 Poll() 立刻关声、听不到任何声音 */
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
        return;     /* 没有正在进行的非阻塞鸣叫，直接返回，几乎零开销 */
    }

    /* 有符号差值比较，回绕安全。详见文件头的说明 */
    if ((int32_t)(HAL_GetTick() - s_off_tick) >= 0) {
        BSP_Buzzer_Off();   /* 内部会把 s_async_on 清掉 */
    }
}

void BSP_Buzzer_KeyFeedback(void)
{
    BSP_Buzzer_BeepAsync(BUZZER_KEY_BEEP_MS);
}

void BSP_Buzzer_Alarm(uint16_t times)
{
    uint16_t i;

    /* 阻塞式连响。循环里 i + 1 < times 的判断是为了让最后一声之后
       不再多等一个间隔，否则报警会拖出一条没有意义的静音尾巴 */
    for (i = 0u; i < times; i++) {
        BSP_Buzzer_On();
        HAL_Delay(BUZZER_ALARM_ON_MS);
        BSP_Buzzer_Off();

        if (i + 1u < times) {
            HAL_Delay(BUZZER_ALARM_OFF_MS);
        }
    }
}
