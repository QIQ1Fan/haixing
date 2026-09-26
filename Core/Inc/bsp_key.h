/**
  ******************************************************************************
  * @file    bsp_key.h
  * @brief   四路独立按键：软件消抖 + 短按/长按/连发事件
  * @note    ★ 使用约定：必须以 KEY_SCAN_PERIOD_MS（默认 10ms）周期调用
  *          BSP_Key_Scan()，事件的时间阈值才有意义。
  ******************************************************************************
  */
#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/** 按键索引。顺序必须与 bsp_gpio.c 中的 s_key_port / s_key_pin 一致 */
typedef enum {
    KEY_OK = 0,     /* PB13  确认 */
    KEY_BACK,       /* PB14  退出 */
    KEY_PLUS,       /* PB15  加   */
    KEY_MINUS,      /* PA15  减   */
    KEY_ID_NUM
} BSP_KeyId_t;

/** 按键事件。所有事件都在 BSP_Key_Scan() 里产生，读走即清零 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS,    /* 短按：松开瞬间产生 */
    KEY_EVENT_LONG,     /* 长按：按住满 KEY_LONG_PRESS_CNT 次扫描时产生一次 */
    KEY_EVENT_REPEAT    /* 长按后的连发：每 KEY_REPEAT_CNT 次扫描产生一次 */
} BSP_KeyEvent_t;

/**
  * @brief  初始化按键模块（引脚方向由 BSP_GPIO_Init 配置，此处只清状态）
  */
void BSP_Key_Init(void);

/**
  * @brief  扫描一次按键状态机（每 10ms 调用一次）
  */
void BSP_Key_Scan(void);

/**
  * @brief  取出并清除某按键的事件
  * @retval 本次待处理的事件，无事件返回 KEY_EVENT_NONE
  */
BSP_KeyEvent_t BSP_Key_GetEvent(BSP_KeyId_t id);

/**
  * @brief  查询某按键当前是否处于按下状态（已消抖）
  */
uint8_t BSP_Key_IsPressed(BSP_KeyId_t id);

/**
  * @brief  丢弃所有未读事件（例如刚从菜单切走时用）
  */
void BSP_Key_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_KEY_H */
