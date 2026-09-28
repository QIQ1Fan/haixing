/**
  ******************************************************************************
  * @file    bsp_gpio.h
  * @brief   纯 GPIO 汇总层：电机方向/使能、六路循迹输入、四路按键输入
  *
  * 【在工程里的位置】 所有"不需要定时器/串口"的引脚都归这里管，是 bsp_motor、
  *                    bsp_track、bsp_key 三个模块的公共底座。
  *
  *   初始化： main.c 调 BSP_GPIO_Init() 一次，把上述引脚全部配好
  *
  *   运行期： ┌── bsp_motor  用 BSP_GPIO_WriteMotorDir / WriteStby
  *            ├── bsp_track  用 BSP_GPIO_ReadTrackRaw
  *            └── bsp_key    用 BSP_GPIO_ReadKeyRaw
  *
  * 【上游】 main.c 启动时调用一次
  * 【下游】 无，直接读写寄存器电平
  *
  * 【本层只做一件事】 引脚电平读写，不含任何业务语义。
  *   比如这里只有"把 AIN1 拉高"，至于拉高代表前进还是后退，
  *   那是 bsp_motor 的事。这样分工的好处是以后换驱动芯片（TB6612 → L298N）
  *   只需要改 bsp_motor，本文件可以原样不动。
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
  *         会覆盖 CubeMX 对 PB3 的初始电平设置（那边写成了低电平 = 蜂鸣器常鸣）。
  */
void BSP_GPIO_Init(void);

/* ---------------------------- 电机方向与使能 ---------------------------- */
/**
  * @brief  直接写四路方向引脚电平（裸操作，语义封装见 BSP_Motor_SetSpeed）
  * @param  ain1/ain2 左电机方向，bin1/bin2 右电机方向
  */
void BSP_GPIO_WriteMotorDir(GPIO_PinState ain1, GPIO_PinState ain2,
                            GPIO_PinState bin1, GPIO_PinState bin2);

/**
  * @brief  TB6612 使能控制
  * @param  st SET = 工作, RESET = 待机（电机自由停转，且不耗电）
  */
void BSP_GPIO_WriteStby(GPIO_PinState st);

/* ------------------------------- 循迹输入 ------------------------------- */
/**
  * @brief  读取六路循迹模块的原始电平
  * @retval bit0..bit5 依次对应传感器 0..5，位为 1 表示引脚当前是高电平
  * @note   这是"引脚电平"，不是"是否压线"。要判断压线请用 BSP_Track_GetMask()
  */
uint8_t BSP_GPIO_ReadTrackRaw(void);

/* ------------------------------- 按键输入 ------------------------------- */
/**
  * @brief  读取单个按键的原始状态
  * @param  idx 按键索引，见 bsp_key.h 的 BSP_KeyId_t
  * @retval 1 = 按下, 0 = 松开
  * @note   未做消抖，直接用会有抖动误触发。业务上请用 bsp_key 的事件接口
  */
uint8_t BSP_GPIO_ReadKeyRaw(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
