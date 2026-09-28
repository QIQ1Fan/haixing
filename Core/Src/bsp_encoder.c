/**
  ******************************************************************************
  * @file    bsp_encoder.c
  * @brief   双路正交编码器测速实现 —— TIM2(PA0/PA1) + TIM4(PB6/PB7)
  *
  * 【数据流】
  *   电机转动
  *     └─► 编码器输出 A/B 两相方波（相位差 90°，转向决定谁超前）
  *           └─► TIM2/TIM4 硬件编码器模式，自动四倍频计数（全程不占 CPU）
  *                 └─► 本模块每 10ms 读一次计数值
  *                       ├─ 与上次相减 → s_delta[]  ← 这一段的转速
  *                       └─ 累加        → s_total[]  ← 总里程
  *                             └─► BSP_Encoder_GetRPM() → PID 速度环
  *
  * 【什么是"四倍频"】
  *   编码器每转输出 N 个脉冲（ENC_PPR）。A/B 两相各有一个上升沿和下降沿，
  *   一共 4 个边沿都可以用来计数，所以有效分辨率是 4N —— 这叫四倍频。
  *   分辨率越高，低速时测速越细腻，速度环越不容易抖。
  *
  * 【怎么知道转的方向】
  *   A 相超前 B 相 90° 是一个方向，B 超前 A 是反方向。硬件在 TI12 模式下
  *   根据两相的相位关系自动决定是加计数还是减计数，软件只要读数值的
  *   增减就能知道方向，不需要额外判断。
  *
  * 【16 位回绕怎么处理 —— 本文件最容易看错的地方】
  *   F103 的 TIM2/TIM4 都是 16 位，计到 65535 会跳回 0（反向则从 0 跳到 65535）。
  *   直接相减会得到巨大的错误值。用 (int16_t)(now - last) 这一招即可：
  *   无符号相减的溢出位被丢掉后再按有符号解释，正好得到正确的带符号增量。
  *      例：now=5, last=65530 → (uint16_t)(5-65530)=11 → (int16_t)11 = +11  ✓
  *   前提是单个采样周期内位移不超过 ±32767 个计数。
  ******************************************************************************
  */
#include "bsp_encoder.h"
#include "main.h"       /* Error_Handler() */

static TIM_HandleTypeDef s_htim[ENC_NUM];
static uint16_t s_last[ENC_NUM];    /* 上次采样时的原始计数值 */
static int16_t  s_delta[ENC_NUM];   /* 本次采样周期内的增量 */
static int32_t  s_total[ENC_NUM];   /* 累计计数（累加增量，不受回绕影响） */

/* 方向反相标记，索引与 BSP_EncId_t 一一对应。
   编译期常量，用 #if 在编译时就定下来，不占运行期判断。 */
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

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/**
  * @brief  把一路定时器配成编码器接口模式
  * @param  h    句柄
  * @param  inst TIM2 或 TIM4
  */
static void encoder_hw_init(TIM_HandleTypeDef *h, TIM_TypeDef *inst)
{
    TIM_Encoder_InitTypeDef enc = {0};

    h->Instance               = inst;
    h->Init.Prescaler         = 0u;         /* 不分频：每个边沿都要计数，一个不放过 */
    h->Init.CounterMode       = TIM_COUNTERMODE_UP;
    h->Init.Period            = 0xFFFFu;    /* ★ 满量程，见下方说明 */
    h->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* ★ Period 必须是 0xFFFF。
       编码器模式下计数器是双向的：正转加、反转减。如果把 ARR 设成比如
       3599（想着"我只需要测一小段"），计数器减到 0 以下时不会变成 65535，
       而是被 ARR 截断回 3599 —— 计数直接丢了，而且丢得很隐蔽：
       表现为高速时读数偏小。设成满量程就没有这个截断点。 */

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;    /* A/B 双相四倍频 */
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;/* 输入直接来自 TI1 引脚 */
    enc.IC1Prescaler = TIM_ICPSC_DIV1;          /* 不预分频，每个边沿都算 */
    enc.IC1Filter    = ENC_IC_FILTER;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = ENC_IC_FILTER;
    /* 滤波器是数字采样滤波器：连续 N 次采样电平一致才认可。
       电机碳刷产生的尖峰干扰很脏，不滤波会产生大量虚假计数，
       表现为"轮子没动，速度读数却乱跳"。 */

    if (HAL_TIM_Encoder_Init(h, &enc) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(h, 0u);
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_Encoder_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t i;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* 编码器 A/B 相配成输入（不是 AF！）。
       编码器模式下定时器的输入捕获通路会自己去读引脚电平，
       引脚本身保持普通输入即可 —— 这点和 PWM 输出脚要配 AF_PP 不同。

       上拉：多数编码器是开漏输出，没上拉就永远读不到高电平。
       如果你的编码器是推挽输出，内部上拉也不会造成冲突，可以保留。 */
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

/* ==========================================================================
 * 测速核心
 * ========================================================================== */

void BSP_Encoder_Update(void)
{
    uint32_t i;

    /* ★ 这个函数必须每 ENC_SAMPLE_PERIOD_MS 被调用一次。
       它自己不知道过了多久 —— 转速换算是拿"增量 / ENC_SAMPLE_PERIOD_MS"
       算的。调用周期不对，转速就成比例地错。 */
    for (i = 0u; i < ENC_NUM; i++) {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&s_htim[i]);
        /* 16 位回绕在这一行被自动处理掉，不需要任何 if 判断 */
        int16_t  d   = (int16_t)(now - s_last[i]);

        s_last[i] = now;

        if (s_invert[i]) {
            d = (int16_t)(-d);
        }

        s_delta[i] = d;
        s_total[i] += (int32_t)d;   /* 累加增量而不是做差，天然免疫回绕 */
    }
}

/* ==========================================================================
 * 读取接口
 * ========================================================================== */

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

    /* 第一步：增量 → 每秒计数。
       乘 1000 再除以周期(ms)，把"每 10ms 多少个计数"换成"每秒多少个计数" */
    counts_per_sec = (float)s_delta[id] * 1000.0f / (float)ENC_SAMPLE_PERIOD_MS;

    /* 第二步：每秒计数 → 每秒转数。
       ENC_CPR 是输出轴转一圈的总计数（= 线数 × 4 × 减速比） */
    rev_per_sec = counts_per_sec / ENC_CPR;

    /* 第三步：每秒转数 → 每分钟转数 */
    return rev_per_sec * 60.0f;
}

float BSP_Encoder_GetSpeedMMps(BSP_EncId_t id, float wheel_diameter_mm)
{
    float rpm = BSP_Encoder_GetRPM(id);

    /* RPM → 转/分 → mm/s：
       转/分 ÷ 60 = 转/秒；× 周长(π·d) = mm/s */
    return rpm * 3.14159265f * wheel_diameter_mm / 60.0f;
}

void BSP_Encoder_Reset(void)
{
    uint32_t i;

    /* 硬件计数器和软件缓存都要清，否则下一次 Update 会算出巨大的增量 */
    for (i = 0u; i < ENC_NUM; i++) {
        __HAL_TIM_SET_COUNTER(&s_htim[i], 0u);
        s_last[i]  = 0u;
        s_delta[i] = 0;
        s_total[i] = 0;
    }
}
