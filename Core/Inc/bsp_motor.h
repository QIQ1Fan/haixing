/**
  ******************************************************************************
  * @file    bsp_motor.h
  * @brief   直流电机驱动：TIM3 双路 20kHz PWM + TB6612 方向/使能
  *
  * 【在工程里的位置】 执行器链的最末端，下游直接是硬件
  *
  *   控制算法        bsp_motor              TB6612            电机
  *   ┌────────┐    ┌──────────┐         ┌──────────┐      ┌──────┐
  *   │  PID   │───►│ 符号→方向 │────────►│  H 桥    │─────►│ 左轮 │
  *   │ 或直接 │    │ 数值→占空比│        │          │      ├──────┤
  *   │ 给速度 │    └──────────┘         │          │─────►│ 右轮 │
  *   └────────┘                         └──────────┘      └──────┘
  *
  * 【上游】 main.c 主循环 / 以后的 PID 速度环，调用 BSP_Motor_SetSpeed()
  * 【下游】 TB6612 驱动模块 → 左右直流电机
  *
  * 【占用的资源】
  *   TIM3_CH1 → PA6        左电机 PWM，20kHz
  *   TIM3_CH2 → PA7        右电机 PWM，20kHz
  *   PB0/PB1  → AIN1/AIN2  左电机方向
  *   PB10/PB11→ BIN1/BIN2  右电机方向
  *   PA4      → STBY       使能，高=工作 低=待机
  *
  * 【一个约定】 所有"速度"都是带符号的 int16_t：
  *   正数 = 前进，负数 = 后退，0 = 滑行，绝对值 = 占空比（0..MOTOR_PWM_MAX）
  * ============================================================================
  */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化电机驱动
  * @note   完成四件事：TIM3 时钟 + PA6/PA7 复用推挽 + 20kHz PWM 参数 + 启动输出，
  *         最后把 STBY 拉高使能 TB6612。
  *         调用前需先执行 BSP_GPIO_Init()（方向引脚在那里配）。
  */
void BSP_Motor_Init(void);

/**
  * @brief  使能 / 待机（控制 TB6612 的 STBY 脚）
  * @param  enable 1 = 使能工作, 0 = 待机
  * @note   传 0 时会顺带调 BSP_Motor_Stop()，避免重新使能时电机突然窜出去
  */
void BSP_Motor_Enable(uint8_t enable);

/**
  * @brief  设定左右轮速度 —— 最常用的接口
  * @param  left  左轮：正=前进，负=后退，0=滑行；范围 ±MOTOR_PWM_MAX
  * @param  right 右轮：同上
  * @note   超出范围会自动饱和到 ±MOTOR_PWM_MAX，不会溢出。
  *         方向引脚由本函数统一维护，调用者不用管。
  *
  *   差速转向示例：
  *     直线      BSP_Motor_SetSpeed(1500, 1500);
  *     缓右转    BSP_Motor_SetSpeed(1800, 1200);   // 左快右慢
  *     原地右转  BSP_Motor_SetSpeed(1200, -1200);  // 左正右反
  */
void BSP_Motor_SetSpeed(int16_t left, int16_t right);

/**
  * @brief  只改占空比、不动方向引脚
  * @param  duty_l/duty_r 0..MOTOR_PWM_MAX
  * @note   方向保持上一次 SetSpeed 的结果。适合速度环高频微调的场景，
  *         省掉每次重写方向引脚的开销。正常使用直接用 SetSpeed 就行。
  */
void BSP_Motor_SetDuty(uint16_t duty_l, uint16_t duty_r);

/**
  * @brief  滑行停止 —— 方向引脚清零，电机输出高阻，靠惯性慢慢停下
  */
void BSP_Motor_Stop(void);

/**
  * @brief  刹车 —— 两方向引脚同时拉高，电机绕组短接，快速制动
  * @note   急停或冲出赛道时用这个，比 Stop() 停得快得多
  */
void BSP_Motor_Brake(void);

/**
  * @brief  取回最近一次设定的速度（供 OLED / 串口回显，不用自己存一份）
  */
void BSP_Motor_GetSpeed(int16_t *left, int16_t *right);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MOTOR_H */
