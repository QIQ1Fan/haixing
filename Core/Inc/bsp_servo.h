/**
  ******************************************************************************
  * @file    bsp_servo.h
  * @brief   舵机驱动：TIM1_CH1 / PA8 / 50Hz
  *
  * 【在工程里的位置】 转向执行器。和 bsp_motor 的差速转向是两种并列方案：
  *                     差速转向 → 两个轮子转速不同
  *                     舵机转向 → 前轮打角（本模块）
  *                   用哪种取决于你的车体结构，两者可以同时存在。
  *
  *   控制算法 ──SetAngle(角度)──► bsp_servo ──脉宽──► 舵机
  *
  * 【上游】 main.c / 以后的转向环 PID
  * 【下游】 舵机（三线：电源、地、信号）
  *
  * 【占用的资源】 TIM1_CH1 → PA8
  *
  * 【为什么 CCR 就是微秒数】
  *   PSC = 71 → 计数频率 72MHz/72 = 1MHz → 1 个计数 = 1 微秒
  *   ARR = 19999 → 周期 20000 微秒 = 20ms = 50Hz
  *   于是写进 CCR 的值直接就是高电平的微秒数。调试时打开寄存器窗口，
  *   看到 CCR = 1500 就知道是中位，非常直观。
  *
  * 【供电提醒】 舵机堵转电流可达 1A 以上，千万别从 STM32 的 3.3V 取电。
  *             要独立 5V 供电，并且和单片机共地，否则会出现"一打角就复位"。
  * ============================================================================
  */
#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化舵机 PWM 并回到中位
  * @note   TIM1 属高级定时器，输出需要置 MOE 位。这步由 HAL_TIM_PWM_Start()
  *         内部自动完成，不用手动 __HAL_TIM_MOE_ENABLE()。
  *         若舵机没反应，先确认这一句被调用了 —— 高级定时器忘了置 MOE
  *         是"配置全对但就是没波形"的最常见原因。
  */
void BSP_Servo_Init(void);

/**
  * @brief  按脉宽驱动舵机
  * @param  pulse_us 高电平持续时间，单位微秒
  *                  会自动饱和到 [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]
  * @note   需要精确标定舵机行程时用这个，比角度更接近硬件真实值
  */
void BSP_Servo_SetPulseUs(uint16_t pulse_us);

/**
  * @brief  按角度驱动舵机 —— 最常用的接口
  * @param  deg 目标角度，负 = 一侧，正 = 另一侧，自动饱和到 ±90
  */
void BSP_Servo_SetAngle(float deg);

/** @brief 取回当前脉宽（微秒） */
uint16_t BSP_Servo_GetPulseUs(void);

/** @brief 取回当前角度（由脉宽反算，不会读到过期的缓存值） */
float    BSP_Servo_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_H */
