/**
  ******************************************************************************
  * @file    bsp_servo.h
  * @brief   舵机驱动：PB4 / 软件 PWM / 50Hz
  *
  * 【在工程里的位置】 转向执行器。和 bsp_motor 的差速转向是两种并列方案：
  *                     差速转向 → 两个轮子转速不同
  *                     舵机转向 → 前轮打角（本模块）
  *                   用哪种取决于你的车体结构，两者可以同时存在。
  *
  *   控制算法 ──SetAngle(角度)──► bsp_servo ──20ms 方波──► 舵机
  *
  * 【上游】 main.c / 以后的转向环 PID。只在需要改变角度时调一次即可
  * 【下游】 舵机（三线：电源、地、信号）
  *
  * 【占用的资源】
  *   PB4          舵机信号（普通 GPIO，靠中断翻转）
  *   TIM1         纯时基，不占任何输出引脚
  *   TIM1_UP_IRQn 中断，每 20ms 触发 2 次 → 合计 100 次/秒
  *
  * 【★ 为什么是软件 PWM 而不是硬件 PWM】
  *   PB4 上唯一可用的定时器输出是 TIM3_CH1，但它需要"部分重映射"TIM3，
  *   而重映射是整组生效的 —— CH1/CH2 会一起从 PA6/PA7 搬到 PB4/PB5，
  *   电机 PWM 就没波形了，PB5 还被循迹占用。
  *   TIM1/TIM2/TIM4 的通道都不在 PB4 上；TIM3 完全重映射的目标 PC6~PC9
  *   在 LQFP48 封装上没有引出。所以只能用软件 PWM。
  *
  * 【工作原理：用一个定时器产生两种长度的周期】
  *   TIM1 配成 1MHz 计数（1 格 = 1 微秒），但 ARR 的值在中断里动态切换：
  *
  *     低电平阶段  ARR = 20000 - 脉宽   →  到点后进中断：拉高、改 ARR
  *     高电平阶段  ARR = 脉宽           →  到点后进中断：拉低、改 ARR
  *
  *   两个阶段加起来恒等于 20000 微秒（20ms），高电平那一段正好是脉宽。
  *   一周期只中断两次，CPU 占用可以忽略。
  *
  * 【代价】 脉冲边沿有几十微秒的软件抖动（受中断响应延迟影响）。
  *   普通舵机会有轻微嗡嗡声，但不影响定位精度。
  *   如果在意，把舵机挪到 PB0/PB1（TIM3_CH3/CH4，不影响 PA6/PA7 的电机
  *   PWM），就能改回完全无抖动的硬件 PWM。
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
  * @brief  初始化舵机（PB4 引脚 + TIM1 时基 + 中断）并回到中位
  * @note   调用后舵机就开始收到 50Hz 方波，不需要额外的"启动"步骤。
  *         TIM1_UP_IRQHandler 定义在 bsp_servo.c 里，不在 stm32f1xx_it.c，
  *         这样舵机相关的代码全部集中在一个文件，便于单独移植或删除。
  */
void BSP_Servo_Init(void);

/**
  * @brief  按脉宽驱动舵机
  * @param  pulse_us 高电平持续时间，单位微秒
  *                  会自动饱和到 [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]
  * @note   需要精确标定舵机行程时用这个，比角度更接近硬件真实值。
  *         新脉宽在下一个 20ms 周期的低电平阶段生效，最多滞后 20ms。
  */
void BSP_Servo_SetPulseUs(uint16_t pulse_us);

/**
  * @brief  按角度驱动舵机 —— 最常用的接口
  * @param  deg 目标角度，★ 范围 0.0 ~ 180.0（本工程的约定，不是 ±90）
  *             0 = 一端(500us)，90 = 中位(1500us)，180 = 另一端(2500us)
  *             超出范围会自动饱和
  */
void BSP_Servo_SetAngle(float deg);

/** @brief 取回当前脉宽（微秒） */
uint16_t BSP_Servo_GetPulseUs(void);

/** @brief 取回当前角度，0~180°，由脉宽反算（不会读到过期的缓存值） */
float    BSP_Servo_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_H */
