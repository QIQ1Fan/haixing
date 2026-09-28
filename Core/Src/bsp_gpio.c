/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @brief   纯 GPIO 汇总层实现
  *
  * 【数据流】
  *   初始化： main.c → BSP_GPIO_Init() → 一次配好全部纯 GPIO 引脚
  *
  *   运行期： bsp_motor ──写──► BSP_GPIO_WriteMotorDir / WriteStby ──► 方向/使能脚
  *            bsp_track ──读──► BSP_GPIO_ReadTrackRaw ──► 六路探头电平
  *            bsp_key   ──读──► BSP_GPIO_ReadKeyRaw   ──► 四个按键电平
  *
  * 【为什么要单独有这一层】
  *   把"引脚表"集中在一个地方，好处是换引脚时只改这里和 bsp_config.h。
  *   而且上层的 bsp_track / bsp_key 不需要知道引脚在哪 —— 它们只按索引
  *   问"第 3 路压线了吗"，具体第 3 路接在哪个口由本文件的数组决定。
  ******************************************************************************
  */
#include "bsp_gpio.h"

/* ==========================================================================
 * 引脚表
 * --------------------------------------------------------------------------
 * 用数组而不是散落的宏，是为了让上层的循环写法成立：
 *   for (i = 0; i < TRACK_SENSOR_NUM; i++) { 读 s_track_port[i] 的 s_track_pin[i] }
 * 想加第 7 路探头，只要在这里加一项、改 bsp_config.h 的 TRACK_SENSOR_NUM。
 * ========================================================================== */

/* 六路循迹：索引 0..5 对应车头从左到右 */
static GPIO_TypeDef *const s_track_port[TRACK_SENSOR_NUM] = {
    TRACK0_PORT, TRACK1_PORT, TRACK2_PORT,
    TRACK3_PORT, TRACK4_PORT, TRACK5_PORT,
};
static const uint16_t s_track_pin[TRACK_SENSOR_NUM] = {
    TRACK0_PIN, TRACK1_PIN, TRACK2_PIN,
    TRACK3_PIN, TRACK4_PIN, TRACK5_PIN,
};

/* 四路按键。★ 顺序必须与 bsp_key.h 的 BSP_KeyId_t 枚举严格一致，
   否则会出现"按确认键，菜单里退出"这种诡异现象。 */
static GPIO_TypeDef *const s_key_port[KEY_NUM] = {
    KEY_OK_PORT, KEY_BACK_PORT, KEY_PLUS_PORT, KEY_MINUS_PORT,
};
static const uint16_t s_key_pin[KEY_NUM] = {
    KEY_OK_PIN, KEY_BACK_PIN, KEY_PLUS_PIN, KEY_MINUS_PIN,
};

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t i;

    /* GPIOA/GPIOB 的时钟 MX_GPIO_Init() 已经开过了，这里再开一次是幂等的，
       主要是让本函数能独立工作，不依赖调用者是否先跑过 CubeMX 的初始化。
       AFIO 时钟必须开：SWJ 重映射等寄存器都挂在 AFIO 上。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* ---- 1. 电机方向 + 使能：推挽输出 ---- */

    /* 先写电平再配方向，这样引脚从输入切到输出的那一刻就已经是目标电平，
       中间不会出现一瞬间的浮空或错误电平。

       初始值全部给低：
         方向脚两两为低 = TB6612 的"滑行"态，电机不转
         STBY 为低     = 整片 TB6612 待机，彻底不输出
       上电时电机不会因为引脚状态未定义而乱冲一下。 */
    HAL_GPIO_WritePin(MOTOR_DIR_PORT,
                      MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
                      MOTOR_BIN1_PIN | MOTOR_BIN2_PIN,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;   /* 方向脚只在换向时翻转，低速档足够 */

    gpio.Pin = MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN;
    HAL_GPIO_Init(MOTOR_DIR_PORT, &gpio);

    gpio.Pin = MOTOR_STBY_PIN;
    HAL_GPIO_Init(MOTOR_STBY_PORT, &gpio);

    /* ---- 2. 六路循迹：上拉输入 ----
       上拉的作用是给引脚一个确定的默认电平。探头没接或线断了时读到高电平
       （不压线），而不是随机跳变 —— 这样故障表现为"完全循不到线"，
       比"随机报线"好查得多。 */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    for (i = 0; i < TRACK_SENSOR_NUM; i++) {
        gpio.Pin = s_track_pin[i];
        HAL_GPIO_Init(s_track_port[i], &gpio);
    }

    /* ---- 3. 四路按键：上拉输入 ----
       按键另一端接 GND，所以平时读高（上拉），按下读低。 */
    for (i = 0; i < KEY_NUM; i++) {
        gpio.Pin = s_key_pin[i];
        HAL_GPIO_Init(s_key_port[i], &gpio);
    }
}

/* ==========================================================================
 * 电机方向与使能
 * ========================================================================== */

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

/* ==========================================================================
 * 循迹输入
 * ========================================================================== */

uint8_t BSP_GPIO_ReadTrackRaw(void)
{
    uint8_t raw = 0u;
    uint32_t i;

    /* 拼成位掩码返回，比返回 6 个 bool 好用的多：
       bit0 对应传感器 0，上层可以整体比较、整体取反 */
    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        if (HAL_GPIO_ReadPin(s_track_port[i], s_track_pin[i]) == GPIO_PIN_SET) {
            raw |= (uint8_t)(1u << i);
        }
    }
    return raw;
}

/* ==========================================================================
 * 按键输入
 * ========================================================================== */

uint8_t BSP_GPIO_ReadKeyRaw(uint8_t idx)
{
    /* 越界保护：上层传错索引时返回"未按下"，而不是去读数组外的内存 */
    if (idx >= KEY_NUM) {
        return 0u;
    }

    /* 上拉输入 → 按下是低电平，和 KEY_ACTIVE_LEVEL 比较得出"是否按下"。
       极性写在这里，以后换成按下读高的电路只改那个宏。 */
    return (HAL_GPIO_ReadPin(s_key_port[idx], s_key_pin[idx]) == KEY_ACTIVE_LEVEL)
           ? 1u : 0u;
}
