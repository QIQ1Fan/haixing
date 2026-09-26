/**
  ******************************************************************************
  * @file    bsp_track.h
  * @brief   六路循迹传感器：原始读取 + 加权位置误差解算
  ******************************************************************************
  */
#ifndef __BSP_TRACK_H
#define __BSP_TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化循迹传感器（实际动作由 BSP_GPIO_Init 完成，此处做状态清零）
  */
void BSP_Track_Init(void);

/**
  * @brief  取"是否压线"的位掩码
  * @retval bit0..bit5 依次对应传感器 0..5，位为 1 表示压到黑线
  */
uint8_t BSP_Track_GetMask(void);

/**
  * @brief  取原始电平掩码（未做极性转换，用于排查接线）
  */
uint8_t BSP_Track_GetRawMask(void);

/** @brief 指定通道是否压线 */
uint8_t BSP_Track_IsActive(uint8_t idx);

/** @brief 当前压线的探头个数 0..6 */
uint8_t BSP_Track_CountActive(void);

/**
  * @brief  加权位置误差
  * @retval <0 车偏左, >0 车偏右, 0 居中；量程 ±TRACK_ERROR_MAX
  */
float BSP_Track_GetError(void);

/**
  * @brief  归一化位置误差
  * @retval 量程 ±1.0
  */
float BSP_Track_GetErrorNorm(void);

/** @brief 六路全压线（十字路口 / 停止线的特征） */
uint8_t BSP_Track_IsAllBlack(void);

/** @brief 六路全未压线（丢线） */
uint8_t BSP_Track_IsAllWhite(void);

/**
  * @brief  把掩码渲染成可读字符串，例如 "001100"
  * @param  buf 输出缓冲，至少 7 字节
  * @note   传感器 0 在最左边，压线显示 '1'
  */
void BSP_Track_MaskToString(uint8_t mask, char *buf);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TRACK_H */
