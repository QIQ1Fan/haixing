/**
  ******************************************************************************
  * @file    bsp_servo.h
  * @brief   舵机驱动：TIM1_CH1 硬件 PWM (50Hz)，PA8 输出
  ******************************************************************************
  */
#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化舵机 PWM
  * @note   TIM1 属高级定时器，HAL_TIM_PWM_Start() 内部会自动置 MOE 位，
  *         不需要手动 __HAL_TIM_MOE_ENABLE()。
  */
void BSP_Servo_Init(void);

/**
  * @brief  按脉宽驱动舵机
  * @param  pulse_us 高电平时间，单位微秒。
  *                  会自动饱和到 [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]
  */
void BSP_Servo_SetPulseUs(uint16_t pulse_us);

/**
  * @brief  按角度驱动舵机
  * @param  deg 目标角度，负=一侧，正=另一侧。自动饱和到 ±90
  */
void BSP_Servo_SetAngle(float deg);

/**
  * @brief  取回当前脉宽 / 角度（供回显用）
  */
uint16_t BSP_Servo_GetPulseUs(void);
float    BSP_Servo_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_H */
