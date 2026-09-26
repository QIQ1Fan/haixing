/**
  ******************************************************************************
  * @file    bsp_buzzer.h
  * @brief   蜂鸣器：PB3 推挽输出，低电平触发
  ******************************************************************************
  */
#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化蜂鸣器引脚，并置为"静音"
  * @note   PB3 默认是 JTAG 的 JTDO。CubeMX 已在 HAL_MspInit() 中调用
  *         __HAL_AFIO_REMAP_SWJ_NOJTAG() 释放该引脚（保留 SWD 调试）。
  *         如果哪天把那次调用删了，PB3 会写不进电平，蜂鸣器完全没反应。
  */
void BSP_Buzzer_Init(void);

/** @brief 鸣响（输出低电平） */
void BSP_Buzzer_On(void);

/** @brief 静音（输出高电平） */
void BSP_Buzzer_Off(void);

/** @brief 翻转当前状态 */
void BSP_Buzzer_Toggle(void);

/**
  * @brief  阻塞式鸣叫
  * @param  ms 持续毫秒数
  * @note   会占用 CPU，不要在控制周期里调用
  */
void BSP_Buzzer_Beep(uint16_t ms);

/**
  * @brief  非阻塞鸣叫：立即开声，到点后由 BSP_Buzzer_Poll() 关掉
  */
void BSP_Buzzer_BeepAsync(uint16_t ms);

/**
  * @brief  非阻塞鸣叫的状态机，需要放在主循环里反复调用
  */
void BSP_Buzzer_Poll(void);

/** @brief 按键反馈：短促"嘀"一声（非阻塞） */
void BSP_Buzzer_KeyFeedback(void);

/** @brief 报警：连响 times 次（阻塞） */
void BSP_Buzzer_Alarm(uint16_t times);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BUZZER_H */
