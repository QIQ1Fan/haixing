/**
  ******************************************************************************
  * @file    bsp_encoder.h
  * @brief   双路正交编码器测速：TIM2(PA0/PA1) + TIM4(PB6/PB7) 硬件编码器模式
  *
  * 【在工程里的位置】 整个闭环的"眼睛"，速度反馈的唯一来源
  *
  *   电机转 → 编码器 A/B 两相方波 → 定时器硬件四倍频自动计数（不耗 CPU）
  *     → 本模块每 10ms 读一次计数差 → s_delta[]
  *       → BSP_Encoder_GetRPM() → PID 速度环 → BSP_Motor_SetSpeed()
  *
  * 【上游】 main.c 的 10ms 节拍器调用 BSP_Encoder_Update()
  * 【下游】 以后的 PID 速度环读 GetDelta / GetRPM / GetSpeedMMps
  *
  * 【占用的资源】
  *   TIM2_CH1/CH2 → PA0/PA1   左编码器 A/B 相
  *   TIM4_CH1/CH2 → PB6/PB7   右编码器 A/B 相
  *
  * 【★ 使用约定：按 CTRL_PERIOD_MS 调用，但允许抖动】
  *   BSP_Encoder_Update() 算的是"两次调用之间的计数增量"。
  *   ★ 转速换算用的是 HAL_GetTick() 实测出来的真实间隔（见
  *     BSP_Encoder_GetDtMs()），不是写死的 10ms —— 这样主循环偶尔被
  *     OLED 刷屏拖慢十几毫秒时，转速依然是准的，不会虚高。
  *   前提是"长期平均"的调用周期仍应是 10ms：如果整体变成 20ms 一拍，
  *   控制环的相位滞后会翻倍，车会变得迟钝、画龙。
  *
  * 【为什么用硬件编码器模式】
  *   减速比 30 的情况下，轮子转一圈 A 相就有 390 个脉冲，四倍频后 1560 个。
  *   若用外部中断逐个计数，电机全速时中断频率可达几十 kHz，CPU 会被吃光。
  *   硬件模式下计数完全由定时器外设自己完成，CPU 只需要每 10ms 读一个数。
  * ============================================================================
  */
#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

typedef enum {
    ENC_LEFT  = 0,      /* TIM2 / PA0 / PA1 */
    ENC_RIGHT = 1,      /* TIM4 / PB6 / PB7 */
    ENC_NUM
} BSP_EncId_t;

/**
  * @brief  实际采样间隔（毫秒）—— 上一次 BSP_Encoder_Update() 距上上次过了多久
  * @note   ★ 这个值不是常数：主循环被 OLED 刷屏或串口打印拖慢时它会变大。
  *         转速换算用的就是它，所以"偶尔拍子被拉长"不会算错速度。
  *         显示出来也能当健康指标看 —— 长期远大于 CTRL_PERIOD_MS
  *         说明主循环里有东西在阻塞，该去查了。
  */
uint16_t BSP_Encoder_GetDtMs(void);

/**
  * @brief  初始化两路编码器接口
  * @note   把 TIM2/TIM4 配成 TI12 编码器模式（A/B 双相四倍频），
  *         并把 ARR 设为 0xFFFF 满量程 —— 设小了会在中途归零，直接丢计数。
  *         引脚配成上拉输入，因为多数编码器是开漏输出。
  */
void BSP_Encoder_Init(void);

/**
  * @brief  采样一次计数增量
  * @note   ★ 必须按 ENC_SAMPLE_PERIOD_MS（默认 10ms）的周期调用，
  *         否则测速结果不准。main.c 的主循环负责这个节拍。
  */
void BSP_Encoder_Update(void);

/**
  * @brief  最近一个采样周期内的计数增量
  * @retval 带符号，正负代表旋转方向
  */
int16_t BSP_Encoder_GetDelta(BSP_EncId_t id);

/**
  * @brief  自初始化/复位以来的累计计数（带符号）
  * @note   可用来算"走了多少距离"：总计数 / ENC_CPR × 轮周长
  */
int32_t BSP_Encoder_GetTotal(BSP_EncId_t id);

/**
  * @brief  输出轴（轮子）转速
  * @retval 转/分（RPM），带符号
  * @note   依赖 ENC_CPR 标定值。若读数明显不对，改 bsp_config.h 里的
  *         ENC_PPR / ENC_GEAR_RATIO，别改这里。
  */
float BSP_Encoder_GetRPM(BSP_EncId_t id);

/**
  * @brief  换算成线速度
  * @param  wheel_diameter_mm 轮子直径，单位 mm（用卡尺量，含橡胶胎面）
  * @retval mm/s，带符号
  */
float BSP_Encoder_GetSpeedMMps(BSP_EncId_t id, float wheel_diameter_mm);

/**
  * @brief  清零累计计数与增量
  * @note   开始一段新测试或新赛程前调用，让里程从 0 起算
  */
void BSP_Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ENCODER_H */
