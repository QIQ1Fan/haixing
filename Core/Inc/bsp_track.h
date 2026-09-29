/**
  ******************************************************************************
  * @file    bsp_track.h
  * @brief   八路循迹传感器：原始读取 + 加权位置误差解算
  *
  * 【在工程里的位置】 整个闭环的"眼睛"之一，位置反馈的唯一来源
  *
  *   地面黑线 → 八路探头电平 → BSP_Track_GetMask() 位掩码
  *     → 加权求和 → BSP_Track_GetError() 位置误差
  *       → PID 转向环 → BSP_Motor_SetSpeed(左, 右) 或 BSP_Servo_SetAngle()
  *
  * 【上游】 main.c 主循环（不需要 10ms 节拍，随时读都是当前值）
  * 【下游】 PID 转向环
  *
  * 【探头排布与权重】
  *   车头朝上，索引 0 在最左：
  *
  *       索引:   0     1     2     3     4     5     6     7
  *       位置: 最左   左   中左  中偏左 中偏右 中右   右   最右
  *       权重:  -7    -5    -3    -1    +1    +3    +5    +7
  *       引脚: PA2   PA3   PA5   PA8   PA11  PA12  PB5   PB12
  *              └──────┴──────┴── 黑线 ──┴──────┴──────┘
  *                    （线在中间时 3/4 两路压线）
  *
  *   误差 = Σ(权重 × 该路是否压线) ÷ 压线个数
  *     误差 < 0 → 车偏左，需要右转
  *     误差 > 0 → 车偏右，需要左转
  *     误差 = 0 → 居中
  *
  *   注意 PA8 原来是舵机脚，现在改作第 3 路循迹输入；舵机已挪到 PB4。
  * ============================================================================
  */
#ifndef __BSP_TRACK_H
#define __BSP_TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化循迹传感器
  * @note   实际动作（引脚方向 + 上拉）由 BSP_GPIO_Init() 完成，
  *         这里只做状态清零。保留这个函数是为了让 main.c 的初始化序列整齐。
  */
void BSP_Track_Init(void);

/**
  * @brief  取"是否压线"的位掩码 —— 最常用
  * @retval bit0..bit7 依次对应传感器 0..7，位为 1 表示压到黑线
  * @note   极性由"运行期开关"决定（出厂值 = bsp_config.h 的 TRACK_ACTIVE_LOW），
  *         已经转换好了。极性错了的时候现象是"整条掩码反了过来"：
  *         车压在黑线上，掩码却在离线的那几路为 1。
  */

/**
  * @brief  ★ 设置探头极性（运行期）
  * @param  active_low 1 = 压到黑线读到低电平；0 = 压到黑线读到高电平
  * @note   做成运行期接口的理由和电机取反一样：极性错了要重编重烧太慢，
  *         而在 OLED 的 SENSOR CHECK 页里拨一下就能立刻看到方框对不对。
  *         出厂默认值来自 bsp_config.h 的 TRACK_ACTIVE_LOW，
  *         实际使用值来自 app_param 的 ACTLOW 参数（app_car 每拍同步）。
  */
void BSP_Track_SetActiveLow(uint8_t active_low);

/** @brief 取回当前极性设置（1 = 压线读低） */
uint8_t BSP_Track_GetActiveLow(void);

uint8_t BSP_Track_GetMask(void);

/**
  * @brief  取原始引脚电平掩码（未做极性转换）
  * @note   专门用来排查接线。如果 GetMask() 结果反了，用这个看一下
  *         原始值就能确认是极性配置问题还是接线问题
  */
uint8_t BSP_Track_GetRawMask(void);

/** @brief 指定通道是否压线 */
uint8_t BSP_Track_IsActive(uint8_t idx);

/** @brief 当前压线的探头个数 0..8 */
uint8_t BSP_Track_CountActive(void);

/**
  * @brief  加权位置误差 —— PID 转向环的输入
  * @retval <0 车偏左, >0 车偏右, 0 居中；量程 ±TRACK_ERROR_MAX（八路时是 ±7）
  * @note   ★ 丢线（八路全白）时不返回 0，而是返回上一次的有效误差。
  *         因为返回 0 会被 PID 理解成"正好居中"，车会直着冲出去；
  *         返回上次的值则能保持原来的转向趋势，继续把线找回来。
  */
float BSP_Track_GetError(void);

/**
  * @brief  归一化位置误差
  * @retval 量程 ±1.0
  * @note   建议 PID 用这个而不是原始值，这样换了探头数量或间距后
  *         PID 参数不用重新整定
  */
float BSP_Track_GetErrorNorm(void);

/**
  * @brief  八路全压线
  * @note   特征值：十字路口、T 字口、或者起跑线/停止线。业务上看到它
  *         通常要"保持直行一小段"或者"停车"。
  *         探头数量多了以后这条路况的误判率明显低于六路，是加探头的主要收益。
  */
uint8_t BSP_Track_IsAllBlack(void);

/** @brief 八路全未压线（丢线），通常是冲出赛道或遇到断线 */
uint8_t BSP_Track_IsAllWhite(void);

/**
  * @brief  把掩码渲染成可读字符串
  * @param  mask 位掩码
  * @param  buf  输出缓冲，★ 至少 TRACK_SENSOR_NUM + 1 = 9 字节
  * @note   传感器 0 显示在最左边，压线显示 '1'。
  *         例：左数第 4、5 路压线 → "00011000"
  */
void BSP_Track_MaskToString(uint8_t mask, char *buf);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TRACK_H */
