/**
  ******************************************************************************
  * @file    bsp_servo.c
  * @brief   舵机驱动实现 —— PB4 / 软件 PWM / 50Hz
  *
  * 【数据流】
  *   BSP_Servo_SetAngle(角度)
  *        │  线性插值（角度约定 0~180°）：
  *        │      0° → 500us，90° → 1500us，180° → 2500us
  *        ▼
  *   BSP_Servo_SetPulseUs(脉宽)   ← 也可以从这一层直接进来
  *        │  饱和到 [500, 2500]，存进 s_pulse_us
  *        ▼
  *   TIM1 更新中断（每 20ms 触发两次）
  *        ├─ 低电平阶段结束 → PB4 拉高，ARR = 脉宽
  *        └─ 高电平阶段结束 → PB4 拉低，ARR = 20000 - 脉宽
  *        ▼
  *   PB4 输出：每 20ms 一个高电平脉冲，宽度 = 设定的微秒数
  *        │
  *        ▼
  *   舵机内部：把脉宽和 1.5ms 比较，差值决定转角
  *
  * 【为什么用"交替修改 ARR"而不是输出比较中断】
  *   本模块只需要一个中断向量（TIM1_UP_IRQn），不需要配置任何输出通道，
  *   TIM1 的四个通道全部空闲。少配一个外设单元就少一处出错的可能。
  *
  * 【★ AutoReloadPreload 必须是 DISABLE】
  *   中断里写 ARR 要立即生效。若使能了预装载，新值会被压到影子寄存器，
  *   等到下一次更新事件才加载 —— 于是 ARR 的切换整整滞后一个周期，
  *   舵机收到的将是两个长度都错的脉冲，表现为剧烈抖动。
  ******************************************************************************
  */
#include "bsp_servo.h"
#include "main.h"       /* Error_Handler() */

static TIM_HandleTypeDef s_htim_servo;

/* 目标脉宽。主循环通过 SetPulseUs 写，中断里读 */
static uint16_t s_pulse_us = SERVO_PULSE_MID_US;

/* 本周期正在使用的脉宽。中断里锁定下来的副本 —— 这样即使主循环在
   一个周期中途改了 s_pulse_us，本周期的"高电平 + 低电平"依然严格
   等于 20000us，周期不会因为参数变化而抖动 */
static uint16_t s_pulse_active = SERVO_PULSE_MID_US;

/* 当前处于哪个阶段：0 = 低电平阶段，1 = 高电平阶段 */
static uint8_t s_phase = 0u;

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_Servo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* PB4 → 普通推挽输出，初始低电平。
       ★ 用 GPIO 而不是复用模式：软件 PWM 就是靠软件翻转引脚，
       所以这里必须让 CPU 能直接控制它，绝不能配成 AF_PP。
       ★ PB4 是 JTAG 的 NJTRST，依赖 HAL_MspInit() 里的 SWJ_NOJTAG
       把它释放成普通 GPIO。删掉那行，这里写电平就完全没反应。 */
    HAL_GPIO_WritePin(SERVO_GPIO_PORT, SERVO_GPIO_PIN, GPIO_PIN_RESET);

    gpio.Pin   = SERVO_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;  /* 脉冲边沿要陡，用高速档 */
    HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio);

    /* ---- TIM1 只当时基用，不配置任何输出通道 ----
       1MHz 计数 → 1 个计数就是 1 微秒，中断里算时间不需要任何换算。 */
    s_htim_servo.Instance               = SERVO_TIM;
    s_htim_servo.Init.Prescaler         = SERVO_TIM_PSC;
    s_htim_servo.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_servo.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_servo.Init.RepetitionCounter = 0u;   /* 每周期都产生更新事件 */
    /* ★ 必须 DISABLE，理由见文件头 */
    s_htim_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    /* 初始 ARR = 低电平阶段的长度。
       这样上电后先在低电平等待，第一次中断才开始发第一个脉冲，
       引脚不会在启动瞬间吐出一个残缺的脉冲给舵机。 */
    s_htim_servo.Init.Period = (uint32_t)(SERVO_PWM_PERIOD_US - SERVO_PULSE_MID_US);

    if (HAL_TIM_Base_Init(&s_htim_servo) != HAL_OK) {
        Error_Handler();
    }

    /* ★ HAL_TIM_Base_Init() 内部会写 EGR 的 UG 位强制产生一次更新事件
       （为了让 PSC 立即生效），这一次事件会把 UIF 标志置起来。
       等下 HAL_TIM_Base_Start_IT() 一开中断，这个陈旧的标志会立刻引发
       一次本不该有的中断。虽然 ISR 里会把它清掉、不至于出错，但会打乱
       第一个周期的相位。这里先手动清干净。 */
    __HAL_TIM_CLEAR_FLAG(&s_htim_servo, TIM_FLAG_UPDATE);

    /* 状态初始化成"正处在低电平阶段"，与上面设的 ARR 对应 */
    s_pulse_us     = SERVO_PULSE_MID_US;
    s_pulse_active = SERVO_PULSE_MID_US;
    s_phase        = 0u;

    /* 中断优先级比 SysTick(15) 高，保证脉冲边沿不被主循环拖慢。
       这个中断很短（就是两次 GPIO 写 + 一次 ARR 写），不会影响别的中断。 */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, SERVO_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

    /* Start_IT：既启动计数，也打开更新中断 */
    if (HAL_TIM_Base_Start_IT(&s_htim_servo) != HAL_OK) {
        Error_Handler();
    }
}

/* ==========================================================================
 * 中断服务函数
 * --------------------------------------------------------------------------
 * ★ 放在这里而不是 stm32f1xx_it.c，是为了让舵机的全部代码（引脚、时基、
 *   中断）集中在一个文件里。要移植或删掉舵机功能时不用去别处翻。
 *   启动文件里的中断向量表按符号名查找，定义在哪个 .c 文件都一样。
 * ========================================================================== */

void TIM1_UP_IRQHandler(void)
{
    /* 防御性检查：本函数理论上只由更新事件触发，但共享向量的情况
       （比如以后又开了 TIM1 的刹车中断）下先判断一下更稳妥 */
    if (__HAL_TIM_GET_FLAG(&s_htim_servo, TIM_FLAG_UPDATE) == RESET) {
        return;
    }
    __HAL_TIM_CLEAR_FLAG(&s_htim_servo, TIM_FLAG_UPDATE);

    if (s_phase == 0u) {
        /* ---- 低电平阶段结束：开始一个脉冲 ---- */

        /* 锁存本周期脉宽。之后即使主循环改了 s_pulse_us，
           也只影响下一个周期，本周期的高+低仍然严格等于 20ms */
        s_pulse_active = s_pulse_us;

        HAL_GPIO_WritePin(SERVO_GPIO_PORT, SERVO_GPIO_PIN, GPIO_PIN_SET);
        /* 高电平持续"脉宽"微秒后再次中断 */
        __HAL_TIM_SET_AUTORELOAD(&s_htim_servo, (uint32_t)s_pulse_active);
        s_phase = 1u;
    } else {
        /* ---- 高电平阶段结束：脉冲发完，拉低 ---- */
        HAL_GPIO_WritePin(SERVO_GPIO_PORT, SERVO_GPIO_PIN, GPIO_PIN_RESET);
        /* 补足剩下的时间，凑满 20ms。
           脉宽被饱和在 [500, 2500]，所以这里最小也有 17500，不会算出 0 */
        __HAL_TIM_SET_AUTORELOAD(&s_htim_servo,
                                 (uint32_t)(SERVO_PWM_PERIOD_US - s_pulse_active));
        s_phase = 0u;
    }

    /* 说明：s_pulse_us 是 16 位变量且在 32 位总线上按 2 字节对齐存放，
       读写本身就是原子的，主循环那边不需要关中断保护 */
}

/* ==========================================================================
 * 控制接口
 * ========================================================================== */

void BSP_Servo_SetPulseUs(uint16_t pulse_us)
{
    /* 饱和保护：脉宽超范围时舵机会顶到机械限位，堵转发热甚至烧齿轮。
       宁可让它停在极限位置，也不能把无效值写进寄存器。
       顺带保证了中断里算 ARR 时不会得到 0（0 会导致周期错乱）。 */
    if (pulse_us < SERVO_PULSE_MIN_US) {
        pulse_us = SERVO_PULSE_MIN_US;
    } else if (pulse_us > SERVO_PULSE_MAX_US) {
        pulse_us = SERVO_PULSE_MAX_US;
    }

    /* 只更新目标值，实际生效由中断在下一个周期开头完成，最多滞后 20ms。
       这样调用者（包括中断上下文以外的主循环）不会打断正在输出的脉冲。 */
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

    /* 线性插值：以中位为基准，按角度偏离中位的量换算成脉宽偏移。
       即 1500us + (deg - 90) × (2000/180)，约每度 11.1 微秒。
       ★ 注意是 (deg - SERVO_ANGLE_MID) 而不是 deg —— 本工程的角度是
       0..180 而不是以中位为 0，直接乘 deg 会把中位算成 0° 那一端。 */
    us = (int32_t)((float)SERVO_PULSE_MID_US
                   + (deg - (float)SERVO_ANGLE_MID) * span_pulse / span_angle);

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
    return (float)SERVO_ANGLE_MID + d * span_angle / span_pulse;
}
