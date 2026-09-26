/**
  ******************************************************************************
  * @file    bsp_motor.h
  * @brief   直流电机驱动：TIM3_CH1/CH2 硬件 PWM (20kHz) + TB6612 方向/使能
  ******************************************************************************
  */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化电机驱动
  * @note   完成：TIM3 时钟 + PA6/PA7 复用推挽 + 20kHz PWM + 启动两路输出，
  *         并把 STBY 拉高使能 TB6612。调用前需先执行 BSP_GPIO_Init()。
  */
void BSP_Motor_Init(void);

/**
  * @brief  使能 / 待机（控制 TB6612 的 STBY）
  * @param  enable 1 = 使能工作, 0 = 待机（电机自由停转）
  */
void BSP_Motor_Enable(uint8_t enable);

/**
  * @brief  设定左右轮带符号速度
  * @param  left  左轮：正=前进，负=后退，0=滑行；范围 ±MOTOR_PWM_MAX
  * @param  right 右轮：同上
  * @note   超出范围会自动饱和，不会溢出。方向引脚由本函数维护。
  */
void BSP_Motor_SetSpeed(int16_t left, int16_t right);

/**
  * @brief  只设定占空比、不改变方向（方向保持上一次 SetSpeed 的结果）
  * @param  duty_l 左轮占空比 0..MOTOR_PWM_MAX
  * @param  duty_r 右轮占空比 0..MOTOR_PWM_MAX
  */
void BSP_Motor_SetDuty(uint16_t duty_l, uint16_t duty_r);

/**
  * @brief  滑行停止（方向引脚清零，电机输出高阻，靠惯性停下）
  */
void BSP_Motor_Stop(void);

/**
  * @brief  刹车（两方向引脚同时拉高，电机绕组短接，快速制动）
  */
void BSP_Motor_Brake(void);

/**
  * @brief  取回最近一次设定的带符号速度（供 OLED / 串口回显）
  */
void BSP_Motor_GetSpeed(int16_t *left, int16_t *right);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MOTOR_H */
