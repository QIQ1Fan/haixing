/**
  ******************************************************************************
  * @file    bsp_gpio.h
  * @brief   纯 GPIO 汇总层：电机方向/使能、六路循迹输入、四路按键输入
  * @note    本层只做"引脚电平读写"，不含任何业务语义。
  *          语义层见 bsp_motor.c / bsp_track.c / bsp_key.c。
  ******************************************************************************
  */
#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化所有"纯 GPIO"引脚（推挽输出 + 上拉输入）
  * @note   必须在 MX_GPIO_Init() 之后调用。
  *         会覆盖 CubeMX 对 PB3 的初始电平设置（那边写成了低电平=蜂鸣器常鸣）。
  */
void BSP_GPIO_Init(void);

/* ---------------------------- 电机方向与使能 ---------------------------- */
/**
  * @brief  直接写四路方向引脚电平（裸操作，语义封装见 BSP_Motor_SetSpeed）
  */
void BSP_GPIO_WriteMotorDir(GPIO_PinState ain1, GPIO_PinState ain2,
                            GPIO_PinState bin1, GPIO_PinState bin2);

/**
  * @brief  TB6612 使能控制：SET=工作, RESET=待机(电机自由停转)
  */
void BSP_GPIO_WriteStby(GPIO_PinState st);

/* ------------------------------- 循迹输入 ------------------------------- */
/**
  * @brief  读取六路循迹模块的原始电平
  * @retval bit0..bit5 依次对应传感器 0..5，位为 1 表示引脚为高电平
  * @note   需要"是否压线"的判断请用 BSP_Track_GetMask()
  */
uint8_t BSP_GPIO_ReadTrackRaw(void);

/* ------------------------------- 按键输入 ------------------------------- */
/**
  * @brief  读取单个按键的原始状态
  * @param  idx 按键索引，见 bsp_key.h 的 BSP_KeyId_t
  * @retval 1 = 按下, 0 = 松开
  */
uint8_t BSP_GPIO_ReadKeyRaw(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
