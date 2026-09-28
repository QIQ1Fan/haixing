/**
  ******************************************************************************
  * @file    bsp_buzzer.h
  * @brief   蜂鸣器：PB3 推挽输出，低电平触发
  *
  * 【在工程里的位置】 最简单的输出设备，两个用途：
  *                     1. 按键反馈 —— 按下"嘀"一声，确认操作生效
  *                     2. 报警 —— 丢线、故障、比赛开始时提示
  *
  *   按键事件 ──► BSP_Buzzer_KeyFeedback()
  *   异常状态 ──► BSP_Buzzer_Alarm()
  *
  * 【上游】 main.c 主循环调用 BSP_Buzzer_Poll() 推进非阻塞鸣叫
  * 【下游】 无，直接写 PB3 电平
  *
  * 【占用的资源】 PB3
  *
  * 【★ PB3 的特殊性】
  *   PB3 默认是 JTAG 的 JTDO，不是普通 GPIO。CubeMX 已在 HAL_MspInit() 里
  *   调用 __HAL_AFIO_REMAP_SWJ_NOJTAG() 把它释放成 GPIO（同时保留 PA13/PA14
  *   的 SWD 下载能力）。删掉那行，PB3 就会写不进电平 —— 而且是静默失败，
  *   编译下载都正常，只是蜂鸣器永远不响，很难查。
  *
  * 【阻塞 vs 非阻塞】
  *   Beep() 内部是 HAL_Delay，会把 CPU 占住，只能用在启动阶段或自检里。
  *   控制循环里要用 BeepAsync() + Poll()，否则会打乱 10ms 节拍。
  * ============================================================================
  */
#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化蜂鸣器引脚并置为静音
  * @note   内部先写电平再配方向，避免初始化瞬间产生一声异响。
  *         引脚方向与上拉也在这里配，不依赖 BSP_GPIO_Init()。
  */
void BSP_Buzzer_Init(void);

/** @brief 鸣响（输出低电平） */
void BSP_Buzzer_On(void);

/** @brief 静音（输出高电平），会顺带取消正在进行的非阻塞鸣叫 */
void BSP_Buzzer_Off(void);

/** @brief 翻转当前状态 */
void BSP_Buzzer_Toggle(void);

/**
  * @brief  阻塞式鸣叫
  * @param  ms 持续毫秒数
  * @note   会占用 CPU，不要在控制循环或自检之外的地方调用
  */
void BSP_Buzzer_Beep(uint16_t ms);

/**
  * @brief  非阻塞鸣叫：立即开始响，到点后由 BSP_Buzzer_Poll() 负责关掉
  * @param  ms 持续毫秒数
  */
void BSP_Buzzer_BeepAsync(uint16_t ms);

/**
  * @brief  非阻塞鸣叫的状态机
  * @note   必须放在主循环里反复调用，否则 BeepAsync() 会一直响下去
  */
void BSP_Buzzer_Poll(void);

/**
  * @brief  按键反馈：短促"嘀"一声（非阻塞）
  * @note   菜单里每处理一次按键事件就调一次，操作手感会好很多
  */
void BSP_Buzzer_KeyFeedback(void);

/**
  * @brief  报警：连响 times 次（阻塞）
  * @param  times 连响次数，比如 3 表示"嘀-嘀-嘀"
  */
void BSP_Buzzer_Alarm(uint16_t times);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BUZZER_H */
