/**
  ******************************************************************************
  * @file    bsp_encoder.c
  * @brief   双路正交编码器测速实现
  *
  * 原理：TIM2 / TIM4 配成 TI12 编码器模式，硬件自动对 A/B 两相四倍频计数，
  *       CPU 只需周期性读计数差值即可反推转速与方向，零 CPU 开销。
  *
  * 回绕处理：F103 的 TIM2/TIM4 都是 16 位。计数器在 0 <-> 65535 之间回绕，
  *          用 (int16_t)(now - last) 可以直接得到正确的带符号增量，
  *          前提是单个采样周期内的位移不超过 ±32767 个计数。
  ******************************************************************************
  */
#include "bsp_encoder.h"
#include "main.h"       /* Error_Handler() */

static TIM_HandleTypeDef s_htim[ENC_NUM];
static uint16_t s_last[ENC_NUM];    /* 上次采样时的原始计数值 */
static int16_t  s_delta[ENC_NUM];   /* 本次增量 */
static int32_t  s_total[ENC_NUM];   /* 累计计数 */

/* 方向反相标记，索引与 BSP_EncId_t 对应 */
static const uint8_t s_invert[ENC_NUM] = {
#if ENC_L_INVERT
    1u,
#else
    0u,
#endif
#if ENC_R_INVERT
    1u
#else
    0u
#endif
};

/* ------------------------------ 内部工具 -------------------------------- */

static void encoder_hw_init(TIM_HandleTypeDef *h, TIM_TypeDef *inst)
{
    TIM_Encoder_InitTypeDef enc = {0};

    h->Instance               = inst;
    h->Init.Prescaler         = 0u;         /* 不分频，计数 = 编码器脉冲 */
    h->Init.CounterMode       = TIM_COUNTERMODE_UP;
    h->Init.Period            = 0xFFFFu;    /* ★ 满量程，设小了会丢计数 */
    h->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;    /* A/B 双相四倍频 */
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = ENC_IC_FILTER;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = ENC_IC_FILTER;

    if (HAL_TIM_Encoder_Init(h, &enc) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(h, 0u);
}

/* ------------------------------- 初始化 -------------------------------- */

void BSP_Encoder_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t i;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* 编码器 A/B 相：上拉输入。编码器多为开漏输出，必须上拉；
       若你的编码器是推挽输出，上拉也不会冲突。 */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;

    gpio.Pin = ENC_GPIO_PIN_LA | ENC_GPIO_PIN_LB;
    HAL_GPIO_Init(ENC_GPIO_PORT_L, &gpio);

    gpio.Pin = ENC_GPIO_PIN_RA | ENC_GPIO_PIN_RB;
    HAL_GPIO_Init(ENC_GPIO_PORT_R, &gpio);

    encoder_hw_init(&s_htim[ENC_LEFT],  ENC_L_TIM);
    encoder_hw_init(&s_htim[ENC_RIGHT], ENC_R_TIM);

    for (i = 0u; i < ENC_NUM; i++) {
        s_last[i]  = 0u;
        s_delta[i] = 0;
        s_total[i] = 0;
    }
}

/* ------------------------------- 测速核心 ------------------------------ */

void BSP_Encoder_Update(void)
{
    uint32_t i;

    for (i = 0u; i < ENC_NUM; i++) {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&s_htim[i]);
        /* 16 位回绕在这里自动被处理掉 */
        int16_t  d   = (int16_t)(now - s_last[i]);

        s_last[i] = now;

        if (s_invert[i]) {
            d = (int16_t)(-d);
        }

        s_delta[i] = d;
        s_total[i] += (int32_t)d;
    }
}

int16_t BSP_Encoder_GetDelta(BSP_EncId_t id)
{
    return (id < ENC_NUM) ? s_delta[id] : 0;
}

int32_t BSP_Encoder_GetTotal(BSP_EncId_t id)
{
    return (id < ENC_NUM) ? s_total[id] : 0;
}

float BSP_Encoder_GetRPM(BSP_EncId_t id)
{
    float counts_per_sec;
    float rev_per_sec;

    if (id >= ENC_NUM) {
        return 0.0f;
    }

    /* 增量 / 采样周期 -> 计数/秒 */
    counts_per_sec = (float)s_delta[id] * 1000.0f / (float)ENC_SAMPLE_PERIOD_MS;
    /* 计数/秒 -> 输出轴转/秒 */
    rev_per_sec = counts_per_sec / ENC_CPR;

    return rev_per_sec * 60.0f;
}

float BSP_Encoder_GetSpeedMMps(BSP_EncId_t id, float wheel_diameter_mm)
{
    float rpm = BSP_Encoder_GetRPM(id);

    /* RPM -> 转/分 -> mm/s */
    return rpm * 3.14159265f * wheel_diameter_mm / 60.0f;
}

void BSP_Encoder_Reset(void)
{
    uint32_t i;

    for (i = 0u; i < ENC_NUM; i++) {
        __HAL_TIM_SET_COUNTER(&s_htim[i], 0u);
        s_last[i]  = 0u;
        s_delta[i] = 0;
        s_total[i] = 0;
    }
}
