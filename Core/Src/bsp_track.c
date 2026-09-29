/**
  ******************************************************************************
  * @file    bsp_track.c
  * @brief   八路循迹传感器实现
  *
  * 【数据流】
  *   地面（黑线吸收红外，白底反射红外）
  *     └─► 八路探头各自输出高低电平
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

/* 权重表，顺序与传感器索引 0..7 一致。数值本身在 bsp_config.h 里 */
static const float s_weight[TRACK_SENSOR_NUM] = {
    TRACK_WEIGHT_0, TRACK_WEIGHT_1, TRACK_WEIGHT_2, TRACK_WEIGHT_3,
    TRACK_WEIGHT_4, TRACK_WEIGHT_5, TRACK_WEIGHT_6, TRACK_WEIGHT_7,
};

/* 八路全压线的掩码。
   1u << 8 = 0x100，减 1 得 0xFF，正好低 8 位全 1。 */
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

/* 当前生效的极性。出厂值取自 bsp_config.h 的宏，
   运行期可以被 BSP_Track_SetActiveLow() 改写（app_car 每拍从参数同步） */
static uint8_t s_active_low = (uint8_t)TRACK_ACTIVE_LOW;

void BSP_Track_SetActiveLow(uint8_t active_low)
{
    s_active_low = (active_low != 0u) ? 1u : 0u;
}

uint8_t BSP_Track_GetActiveLow(void)
{
    return s_active_low;
}

uint8_t BSP_Track_GetMask(void)
{
    uint8_t raw = BSP_GPIO_ReadTrackRaw();
    uint8_t mask = 0u;
    uint32_t i;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        uint8_t level = (uint8_t)((raw >> i) & 0x01u);

        /* 把"引脚电平"翻译成"是否压线"。
           ★ 以前这里是 #if 编译期分支，现在改成运行期判断：
             极性是"调试期间最常改错、又最需要边看边改"的一项，
             做成参数之后在 SENSOR CHECK 页里拨一下就能看到方框对不对，
             不用重编重烧。多出来的那一次判断开销可以忽略。 */
        if (s_active_low != 0u) {
            if (level == 0u) {          /* 压线读到低电平 */
                mask |= (uint8_t)(1u << i);
            }
        } else {
            if (level != 0u) {          /* 压线读到高电平 */
                mask |= (uint8_t)(1u << i);
            }
        }
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

float BSP_Track_GetErrorExclEdge(uint8_t n)
{
    uint8_t  mask;
    float    sum = 0.0f;
    uint8_t  cnt = 0u;
    uint32_t i;

    /* 保护：最多排除到只剩中间一路，再排就没探头可用了 */
    if (n >= (uint8_t)(TRACK_SENSOR_NUM / 2u)) {
        n = (uint8_t)((TRACK_SENSOR_NUM / 2u) - 1u);
    }

    mask = BSP_Track_GetMask();

    /* 只累加 [n, TRACK_SENSOR_NUM-n) 这一段，也就是中间那几路。
       权重仍用它们自己的（±5、±3 …），所以误差量程跟着变小 ——
       这正是我们要的"直道更柔"的效果 */
    for (i = (uint32_t)n; i < ((uint32_t)TRACK_SENSOR_NUM - (uint32_t)n); i++) {
        if ((mask >> i) & 0x01u) {
            sum += s_weight[i];
            cnt++;
        }
    }

    /* 中间几路一个都没压到：和 GetError() 一样沿用上次的有效误差，
       绝不返回 0（返回 0 会被 PID 当成"正好居中"，车会直着冲出去） */
    if (cnt == 0u) {
        return s_last_error;
    }

    s_last_error = sum / (float)cnt;
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
    /* 八路全压线。特征场景：十字路口、T 字口、起跑线/停止线。
       业务上看到这个通常意味着"这里不是普通循迹段，要特殊处理" */
    return (BSP_Track_GetMask() == TRACK_MASK_ALL) ? 1u : 0u;
}

uint8_t BSP_Track_IsAllWhite(void)
{
    /* 八路全白 = 丢线。可能是冲出赛道、遇到断线、或者被手拿起来了。
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

    /* ★ 从传感器 0 开始写：字符串最左边的字符 = 传感器 0。
     *
     * 【这里原来写反了，是个真 bug】
     *   之前的实现是先写 bit7，也就是把"传感器 7"放在最左边 ——
     *   结果同一个掩码，串口里的字符串和 OLED 上那 8 个方框
     *   左右是镜像的：
     *       只有最左边一路压线时
     *         屏幕方框：■□□□□□□□   （正确）
     *         串口字符串：00000001    ← 看着像"最右边压线"
     *   两处显示互相矛盾，读的人自然会怀疑"是不是极性配反了"，
     *   于是跑去改 TRACK_ACTIVE_LOW —— 越改越乱。
     *
     *   现在统一成"索引 0 在最左边"，和下面三处完全一致：
     *     · menu_draw_mask_boxes()  画方框的顺序
     *     · bsp_config.h 里的权重表 TRACK_WEIGHT_0..7
     *     · 探头索引本身的定义（0..7 = 车头从左到右）
     *
     * ★ 调用者给的缓冲区至少要 TRACK_SENSOR_NUM + 1 = 9 字节。 */
    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        buf[i] = ((mask >> i) & 0x01u) ? '1' : '0';
    }
    buf[TRACK_SENSOR_NUM] = '\0';
}
