/**
  ******************************************************************************
  * @file    bsp_encoder.h
  * @brief   双路正交编码器测速：TIM2(PA0/PA1) + TIM4(PB6/PB7) 硬件编码器模式
  * @note    ★ 使用约定：必须以固定周期调用 BSP_Encoder_Update()，
  *          周期等于 ENC_SAMPLE_PERIOD_MS（默认 10ms），
  *          否则测速结果会成比例偏大/偏小。
  ******************************************************************************
  */
#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

typedef enum {
    ENC_LEFT  = 0,
    ENC_RIGHT = 1,
    ENC_NUM
} BSP_EncId_t;

/**
  * @brief  初始化两路编码器接口
  * @note   TIM2/TIM4 均为 16 位，ARR 设为 0xFFFF 满量程以避免丢计数。
  */
void BSP_Encoder_Init(void);

/**
  * @brief  采样一次计数增量（必须按 ENC_SAMPLE_PERIOD_MS 周期调用）
  */
void BSP_Encoder_Update(void);

/**
  * @brief  最近一次采样周期内的计数增量（带符号，正负代表旋转方向）
  */
int16_t BSP_Encoder_GetDelta(BSP_EncId_t id);

/**
  * @brief  自初始化/复位以来的累计计数（带符号）
  */
int32_t BSP_Encoder_GetTotal(BSP_EncId_t id);

/**
  * @brief  输出轴转速
  * @retval 转/分（RPM），带符号
  */
float BSP_Encoder_GetRPM(BSP_EncId_t id);

/**
  * @brief  换算成线速度
  * @param  wheel_diameter_mm 轮子直径，单位 mm
  * @retval mm/s，带符号
  */
float BSP_Encoder_GetSpeedMMps(BSP_EncId_t id, float wheel_diameter_mm);

/**
  * @brief  清零累计计数与增量
  */
void BSP_Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ENCODER_H */
