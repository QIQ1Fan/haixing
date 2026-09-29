/**
  ******************************************************************************
  * @file    app_menu.h
  * @brief   OLED 调参界面：状态页 / 主菜单 / 参数编辑 / 支撑脚 / 传感器检查
  *
  * 【在工程里的位置】
  *   人机交互层。上面接 main.c 的 10ms 节拍和串口命令，下面用 bsp_oled 显示、
  *   bsp_key 取按键，中间读写 app_param 的参数总线、指挥 app_car / app_leg。
  *
  *     BSP_Key_GetEvent() ──►┌──────────────┐──► BSP_OLED_Clear/DrawString
  *                           │   app_menu   │        └─► BSP_OLED_RefreshDirty()
  *                           └──┬────┬───┬──┘
  *                              │    │   └──► APP_Leg_*    支撑脚升降
  *                              │    └──────► APP_Car_*    启停 / 急停 / 状态快照
  *                              └───────────► APP_Param_*  改参数 / 存 Flash
  *
  * 【★ 调用约定（main.c 必须照这个来）】
  *   APP_Menu_Init()      —— 初始化序列最后调用（在 BSP_OLED_Init 之后）。
  *                           内部会整屏重画一次，把开机的自检图案擦掉。
  *   APP_Menu_Update()    —— ★ 主循环每 10ms 调用一次。内部绝对不阻塞：
  *                           没有 HAL_Delay、没有 while 等待、不做 Flash 擦写
  *                           （"保存参数"这个阻塞 30ms 的动作只在停车时允许，
  *                             且必须由用户按键触发）。
  *   APP_Menu_ShowStatus()—— 串口命令用：强制切回状态页。只置标志，
  *                           真正的重画留给下一次 Update()，不打断控制节拍。
  *   APP_Menu_Invalidate()—— 别的模块（bsp_test 自检）往屏幕上画过东西之后
  *                           调用：清空显存 + 强制下一次整屏重画。
  *
  * 【为什么界面要单独一个模块】
  *   界面的本质是"按键事件 → 改状态 → 重画"，这堆逻辑和硬件、和控制律
  *   都没有关系。把它独立出来之后，app_car / app_leg 完全不知道屏幕的存在，
  *   换一块屏（比如换成 LCD）只需要重写本模块，控制部分一行都不用动。
  * ============================================================================
  */
#ifndef __APP_MENU_H
#define __APP_MENU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化调参界面
  * @note   切到状态页并整屏重画一次（唯一一次用整屏刷新，因为不在 10ms 循环里）。
  *         ★ 必须在 BSP_OLED_Init() / APP_Param_Init() / APP_Car_Init() 之后调用。
  */
void APP_Menu_Init(void);

/**
  * @brief  ★ 界面主循环：处理按键 + 节流刷新，每 10ms 调用一次
  * @note   本函数不会阻塞。刷屏节流规则：
  *           车在跑 → 每 MENU_RUN_REFRESH_MS（200ms）刷一次
  *           车停着 → 每 MENU_IDLE_REFRESH_MS（120ms）刷一次
  *           这一拍按了键、画面内容变了 → 立刻重画 + 刷新（保证手感）
  *         重画只用 BSP_OLED_RefreshDirty()，不用整屏的 BSP_OLED_Refresh()。
  */
void APP_Menu_Update(void);

/**
  * @brief  强制切回状态页（串口命令用）
  * @note   只改状态标志，重画由下一次 APP_Menu_Update() 完成，
  *         所以它可以在任何地方安全调用，不会拖慢控制节拍。
  */
void APP_Menu_ShowStatus(void);

/**
  * @brief  屏幕被别的模块画过之后调用：清空显存 + 强制下一次整屏重画
  * @note   bsp_test 的 OLED 自检（TestPattern / 自检图案）画完之后必须调一次，
  *         否则界面会在一堆残留图案上叠着画，看起来是花屏。
  */
void APP_Menu_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MENU_H */
