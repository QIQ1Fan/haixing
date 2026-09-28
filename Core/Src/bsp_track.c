/**
  ******************************************************************************
  * @file    bsp_track.c
  * @brief   六路循迹传感器实现
  *
  * 【数据流】
  *   地面（黑线吸收红外，白底反射红外）
  *     └─► 六路探头各自输出高低电平
  *           └─► BSP_GPIO_ReadTrackRaw()  → 原始电平掩码（位为 1 = 高电平）
  *                 └─► 按 TRACK_ACTIVE_LOW 做极性转换
  *                       └─► BSP_Track_GetMask() → 压线掩码（位为 1 = 压到黑线）
  *                             ├─► CountActive / IsAllBlack / IsAllWhite  状态判断
  *                             └─► GetError() 加权求位置误差 → PID 转向环
  *
  * 【为什么要有"原始电平"和"压线掩码"两套】
  *   不同厂家的循迹模块输出极性可能相反（有的压线输出低，有的输出高）。
  *   调试时如果发现车往反方向跑，需要先分清是"极性配错了"还是"左右接反了"。
  *   GetRawMask() 给你未经处理的真实电平，GetMask() 给你转换后的结果，
  *   两者一比就能定位问题。
  ******************************************************************************
  */
#include "bsp_track.h"
#include "bsp_gpio.h"

/* 权重表，顺序与传感器索引 0..5 一致。数值本身在 bsp_config.h 里 */
static const float s_weight[TRACK_SENSOR_NUM] = {
    TRACK_WEIGHT_0, TRACK_WEIGHT_1, TRACK_WEIGHT_2,
    TRACK_WEIGHT_3, TRACK_WEIGHT_4, TRACK_WEIGHT_5,
};

/* 六路全压线的掩码。
   1u << 6 = 0x40，减 1 得 0x3F，正好低 6 位全 1。 */
#define TRACK_MASK_ALL      ((uint8_t)((1u << TRACK_SENSOR_NUM) - 1u))

/* 最近一次的有效误差。
   丢线时沿用它，避免"丢线返回 0"被 PID 误判成"正好居中"。详见 GetError() */
static float s_last_error = 0.0f;

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_Track_Init(void)
{
    /* 引脚方向与上拉由 BSP_GPIO_Init() 统一配置。
       这里故意不做重复初始化，否则两处都配引脚，以后改引脚时容易漏改一处。
       保留这个空函数只是为了让 main.c 的初始化序列整齐、可读。
       若以后加了"探头自校准"这类功能，就实现在这里。 */
}

/* ==========================================================================
 * 读取接口
 * ========================================================================== */

uint8_t BSP_Track_GetRawMask(void)
{
    return BSP_GPIO_ReadTrackRaw();
}

uint8_t BSP_Track_GetMask(void)
{
    uint8_t raw = BSP_GPIO_ReadTrackRaw();
    uint8_t mask = 0u;
    uint32_t i;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        uint8_t level = (uint8_t)((raw >> i) & 0x01u);

        /* 把"引脚电平"翻译成"是否压线"。极性由 TRACK_ACTIVE_LOW 决定。
           用 #if 而不是 if：编译期就把不需要的分支去掉，运行期零开销 */
#if TRACK_ACTIVE_LOW
        if (level == 0u) {          /* 低电平 = 压线（常见） */
            mask |= (uint8_t)(1u << i);
        }
#else
        if (level != 0u) {          /* 高电平 = 压线 */
            mask |= (uint8_t)(1u << i);
        }
#endif
    }
    return mask;
}

uint8_t BSP_Track_IsActive(uint8_t idx)
{
    if (idx >= TRACK_SENSOR_NUM) {
        return 0u;
    }
    return (uint8_t)((BSP_Track_GetMask() >> idx) & 0x01u);
}

uint8_t BSP_Track_CountActive(void)
{
    uint8_t mask = BSP_Track_GetMask();
    uint8_t n = 0u;
    uint32_t i;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        if ((mask >> i) & 0x01u) {
            n++;
        }
    }
    return n;
}

/* ==========================================================================
 * 误差解算
 * ========================================================================== */

float BSP_Track_GetError(void)
{
    uint8_t mask = BSP_Track_GetMask();
    float sum = 0.0f;
    uint8_t n = 0u;
    uint32_t i;

    /* 把压线那几路的权重加起来，同时数一共压了几路 */
    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        if ((mask >> i) & 0x01u) {
            sum += s_weight[i];
            n++;
        }
    }

    /* ★ 丢线（一路都没压到）时的处理，是这段代码最关键的决策。
       不能返回 0 —— 那等于告诉 PID"车正好居中"，车会直着冲出赛道。
       正确做法是返回上一次的有效误差，让转向趋势延续下去，
       车就会继续朝原来找线的方向转，通常几百毫秒内就能把线找回来。
       首次上电就丢线时（还没建立过有效值）才返回 0。 */
    if (n == 0u) {
        return s_last_error;
    }

    /* 除以压线个数求平均，是为了让"压 1 路"和"压 3 路"算出来的
       误差量级一致，PID 参数不用随压线路数变化。 */
    s_last_error = sum / (float)n;
    return s_last_error;
}

float BSP_Track_GetErrorNorm(void)
{
    /* 归一化到 ±1.0。建议 PID 用这个值：
       以后换了探头数量或调整了间距，TRACK_ERROR_MAX 跟着改，
       PID 参数不用重新整定。 */
    return BSP_Track_GetError() / TRACK_ERROR_MAX;
}

uint8_t BSP_Track_IsAllBlack(void)
{
    /* 六路全压线。特征场景：十字路口、T 字口、起跑线/停止线。
       业务上看到这个通常意味着"这里不是普通循迹段，要特殊处理" */
    return (BSP_Track_GetMask() == TRACK_MASK_ALL) ? 1u : 0u;
}

uint8_t BSP_Track_IsAllWhite(void)
{
    /* 六路全白 = 丢线。可能是冲出赛道、遇到断线、或者被手拿起来了。
       配合 GetError() 的"沿用上次误差"策略，车会自己找回来 */
    return (BSP_Track_GetMask() == 0u) ? 1u : 0u;
}

/* ==========================================================================
 * 调试辅助
 * ========================================================================== */

void BSP_Track_MaskToString(uint8_t mask, char *buf)
{
    uint32_t i;

    if (buf == NULL) {
        return;
    }

    /* 高位（传感器 5，最右）先写，这样字符串看起来和车头朝向一致：
       显示 "001100" 就是左边数第 3、4 路压线。
       若倒序输出，读的人要在脑子里做一次翻转，很容易看错左右。 */
    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        buf[i] = ((mask >> (TRACK_SENSOR_NUM - 1u - i)) & 0x01u) ? '1' : '0';
    }
    buf[TRACK_SENSOR_NUM] = '\0';
}
