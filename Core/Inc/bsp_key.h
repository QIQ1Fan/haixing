/**
  ******************************************************************************
  * @file    bsp_key.h
  * @brief   四路独立按键：软件消抖 + 短按/长按/连发事件
  *
  * 【在工程里的位置】 人机交互的输入端，配合 bsp_oled 构成调参菜单
  *
  *   按键引脚 → BSP_Key_Scan() 每 10ms 消抖一次 → 事件队列（每键一个）
  *     → BSP_Key_GetEvent() 读走事件
  *       → 菜单状态机：确认进入 / 退出返回 / 加 / 减
  *
  * 【上游】 main.c 的 10ms 节拍器调用 BSP_Key_Scan()
  * 【下游】 以后的菜单处理逻辑
  *
  * 【占用的资源】
  *   PB13  确认      上拉输入，按下读低
  *   PB14  退出
  *   PB15  加
  *   PA15  减        （原 JTAG 的 JTDI，已被 SWJ_NOJTAG 释放）
  *
  * 【★ 使用约定：必须定周期调用】
  *   长按 1 秒、连发 100ms 这些时间阈值都是靠"扫描次数 × 周期"算出来的，
  *   周期不对阈值就不对。main.c 里用 10ms 软节拍保证这一点。
  *
  * 【为什么要软件消抖】
  *   机械按键在按下和松开的瞬间，触点会弹跳几毫秒，产生一串高低跳变。
  *   不消抖的话按一次会被识别成好几次。这里用"连续 3 次读数一致才认可"
  *   的办法（3 × 10ms = 30ms），简单且够用。
  * ============================================================================
  */
#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/** 按键索引。顺序必须与 bsp_gpio.c 中的 s_key_port / s_key_pin 数组一致 */
typedef enum {
    KEY_OK = 0,     /* PB13  确认 */
    KEY_BACK,       /* PB14  退出 */
    KEY_PLUS,       /* PB15  加   */
    KEY_MINUS,      /* PA15  减   */
    KEY_ID_NUM
} BSP_KeyId_t;

/** 按键事件。都在 BSP_Key_Scan() 里产生，被读取后自动清零 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS,    /* 短按：松开瞬间产生，且这次按住期间没触发过长按 */
    KEY_EVENT_LONG,     /* 长按：按住满 KEY_LONG_PRESS_CNT 次扫描时产生一次 */
    KEY_EVENT_REPEAT    /* 连发：长按之后每 KEY_REPEAT_CNT 次扫描产生一次 */
} BSP_KeyEvent_t;

/**
  * @brief  初始化按键模块
  * @note   引脚方向与上拉由 BSP_GPIO_Init() 配置，这里只清状态
  */
void BSP_Key_Init(void);

/**
  * @brief  扫描一次按键状态机
  * @note   ★ 必须每 KEY_SCAN_PERIOD_MS（默认 10ms）调用一次。
  *         内部会把每个键的当前电平、消抖计数、按住时长全部推进一格。
  */
void BSP_Key_Scan(void);

/**
  * @brief  取出并清除某按键的事件
  * @retval 本次待处理的事件，无事件返回 KEY_EVENT_NONE
  * @note   读走即清零。典型的菜单循环写法：
  *
  *           BSP_KeyEvent_t e = BSP_Key_GetEvent(KEY_PLUS);
  *           if (e == KEY_EVENT_PRESS) { 参数 += 1; }
  *           if (e == KEY_EVENT_LONG || e == KEY_EVENT_REPEAT) { 参数 += 10; }
  */
BSP_KeyEvent_t BSP_Key_GetEvent(BSP_KeyId_t id);

/** @brief 查询某按键当前是否按下（已消抖，适合做"按住才动"的操作） */
uint8_t BSP_Key_IsPressed(BSP_KeyId_t id);

/** @brief 丢弃所有未读事件（从菜单切走时用，避免残留事件误触发） */
void BSP_Key_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_KEY_H */
