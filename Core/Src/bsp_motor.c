/**
  ******************************************************************************
  * @file    bsp_motor.c
  * @brief   直流电机驱动实现：TIM3 双路 20kHz PWM + TB6612 方向控制
  *
  * 【数据流】
  *   BSP_Motor_SetSpeed(左, 右)
  *        │
  *        ├─ 符号位 ──► 查 TB6612 真值表 ──► 写 AIN1/AIN2、BIN1/BIN2
  *        │                                    （决定电流方向 = 正转/反转/停）
  *        └─ 绝对值 ──► 饱和到 ±MOTOR_PWM_MAX ──► 写 TIM3->CCR1/CCR2
  *                                              （决定 PWM 占空比 = 电压大小）
  *
  *   TB6612 把这两个信息合起来，输出对应极性和大小的电压给电机。
  *
  * 【TB6612 真值表】（本文件所有方向逻辑的依据）
  *     AIN1  AIN2   │  输出状态
  *     ─────────────┼──────────────
  *       0     1    │  正转
  *       1     0    │  反转
  *       0     0    │  滑行（输出高阻，电机自己转完惯性停下）
  *       1     1    │  刹车（绕组短接，快速制动）
  *
  *   注意"滑行"和"刹车"的区别：停车时想要哪种由 BSP_Motor_Stop() 还是
  *   BSP_Motor_Brake() 决定，二者都把占空比清零，区别只在方向脚。
  ******************************************************************************
  */
#include "bsp_motor.h"
#include "bsp_gpio.h"
#include "main.h"       /* Error_Handler() */

static TIM_HandleTypeDef s_htim_motor;
static int16_t s_speed_l = 0;   /* 缓存最近一次设定值，供回显查询 */
static int16_t s_speed_r = 0;

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/**
  * @brief  设定单个电机的方向与占空比
  * @param  dir_pin_fwd  该电机"正转"时需要拉高的那个方向引脚
  * @param  dir_pin_rev  该电机"反转"时需要拉高的那个方向引脚
  * @param  ch           该电机对应的 PWM 通道
  * @param  speed        带符号速度（调用者保证已饱和处理）
  *
  * 两个电机共用这一份逻辑，靠参数告诉它"哪个引脚代表前进"。
  * 这样左右电机的代码路径完全相同，不会出现"改了一边忘了另一边"。
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
        duty = (uint32_t)(-speed);      /* 取绝对值给占空比，符号已经用掉了 */
    } else {
        /* 速度为 0：两脚同时拉低 = TB6612 的滑行态 */
        HAL_GPIO_WritePin(MOTOR_DIR_PORT, (uint32_t)(dir_pin_fwd | dir_pin_rev),
                          GPIO_PIN_RESET);
        duty = 0u;
    }

    __HAL_TIM_SET_COMPARE(&s_htim_motor, ch, duty);
}

/**
  * @brief  把速度饱和到 ±MOTOR_PWM_MAX
  *
  * 必须做这一步，有两个原因：
  *   1. 占空比寄存器只有 0..ARR 的范围，超了会写到无效值
  *   2. 更重要：下面要取 -speed，如果 speed 是 -32768，取负会整数溢出。
  *      先饱和再取负，就永远不会碰到这个边界。
  */
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

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};

    /* ---- 1. PWM 引脚 PA6/PA7 复用推挽输出 ----
       复用(AF)模式：引脚不再由 GPIO 的 ODR 寄存器控制，而是交给 TIM3 的
       输出比较单元。所以一旦配成 AF_PP，写 HAL_GPIO_WritePin 就没用了。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Pin   = MOTOR_GPIO_PIN_L | MOTOR_GPIO_PIN_R;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;  /* PWM 边沿要陡，用高速档 */
    HAL_GPIO_Init(MOTOR_GPIO_PORT, &gpio);

    /* ---- 2. TIM3 时基：72MHz / (0+1) / (3599+1) = 20kHz ----
       HAL_TIM_PWM_Init() 内部会调一次 HAL_TIM_PWM_MspInit()，那是空函数
       （HAL 里是 __weak），所以上面的时钟和引脚都得我们自己配。 */
    s_htim_motor.Instance               = MOTOR_TIM;
    s_htim_motor.Init.Prescaler         = MOTOR_TIM_PSC;
    s_htim_motor.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim_motor.Init.Period            = MOTOR_TIM_ARR;
    s_htim_motor.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim_motor.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim_motor) != HAL_OK) {
        Error_Handler();
    }

    /* ---- 3. 两路 PWM 通道，初值 0 占空比（电机先别动）----
       PWM1 模式：计数器小于 CCR 时输出高电平。
       所以 CCR 越大，高电平时间越长，平均电压越高，电机越快。 */
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0u;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;     /* 普通 PWM 不需要快速使能 */
    if (HAL_TIM_PWM_ConfigChannel(&s_htim_motor, &oc, MOTOR_CH_L) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&s_htim_motor, &oc, MOTOR_CH_R) != HAL_OK) {
        Error_Handler();
    }

    /* ---- 4. 启动输出 ----
       这一步才真正让 PA6/PA7 开始吐方波。在此之前通道是静默的。 */
    HAL_TIM_PWM_Start(&s_htim_motor, MOTOR_CH_L);
    HAL_TIM_PWM_Start(&s_htim_motor, MOTOR_CH_R);

    s_speed_l = 0;
    s_speed_r = 0;
    BSP_Motor_Stop();

    /* ---- 5. 最后才使能 TB6612 ----
       顺序很重要：先让 PWM 稳定输出 0%，再打开 TB6612，
       这样上电瞬间电机不会有任何抖动。反过来做，TB6612 会在 PWM
       还没配好时就输出，电机可能"咯噔"一下。 */
    BSP_Motor_Enable(1u);
}

/* ==========================================================================
 * 控制接口
 * ========================================================================== */

void BSP_Motor_Enable(uint8_t enable)
{
    BSP_GPIO_WriteStby(enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 进入待机时顺手把速度清零。否则下次使能时，TB6612 会立刻按
       上次遗留的占空比输出，电机猛地窜出去。 */
    if (!enable) {
        BSP_Motor_Stop();
    }
}

void BSP_Motor_SetSpeed(int16_t left, int16_t right)
{
    int16_t l = motor_saturate(left);
    int16_t r = motor_saturate(right);

    /* 编译期开关，用来纠正轮子接线接反的情况。
       #if 而不是 if：这样参数为 0 时这段代码根本不编译进来 */
#if MOTOR_L_INVERT
    l = (int16_t)(-l);
#endif
#if MOTOR_R_INVERT
    r = (int16_t)(-r);
#endif

    /* 左轮走 A 通道：TB6612 真值表里 AIN2=1 是正转 */
    motor_apply(MOTOR_AIN2_PIN, MOTOR_AIN1_PIN, MOTOR_CH_L, l);
    /* 右轮走 B 通道：BIN2=1 是正转 */
    motor_apply(MOTOR_BIN2_PIN, MOTOR_BIN1_PIN, MOTOR_CH_R, r);

    /* 存下来供回显。存的是"纠偏之后"的实际值，也就是电机真正执行的速度 */
    s_speed_l = l;
    s_speed_r = r;
}

void BSP_Motor_SetDuty(uint16_t duty_l, uint16_t duty_r)
{
    /* 注意这里只改占空比，不动方向引脚 —— 方向保持上一次 SetSpeed 的结果。
       适合速度环高频微调：省掉每次重写方向引脚的开销，
       也避免在换向边界上抖动时方向脚频繁翻转。 */
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
    /* 四脚全低 = 两个电机都进入滑行态，靠惯性停 */
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
    /* 四脚全高 = 两个电机都进入刹车态，绕组短接快速制动。
       循迹车冲出赛道需要紧急停时用这个，比 Stop() 的滑行短得多。 */
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
    /* 允许传 NULL 只取其中一个，调用方不用为了拿一个值而准备两个变量 */
    if (left  != NULL) { *left  = s_speed_l; }
    if (right != NULL) { *right = s_speed_r; }
}
