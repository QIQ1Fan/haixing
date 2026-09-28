/**
  ******************************************************************************
  * @file    bsp_test.h
  * @brief   底层驱动自检：每个模块一个可独立运行的测试函数
  *
  * 【在工程里的位置】 开发阶段的调试工具，不参与正式跑车
  *
  *   电脑串口助手 ──敲 1~9──► USART1 ──► BSP_Test_RunCommand()
  *                                          └─► 对应模块的测试函数
  *
  * 【上游】 main.c 主循环把收到的字符交给 BSP_Test_RunCommand()
  * 【下游】 调用所有 bsp_* 模块
  *
  * 【怎么用】
  *   1. 编译下载，串口助手开 115200-8-N-1
  *   2. 上电后会自动打印菜单
  *   3. 按 1~9 逐项验硬件，每项测完都会回到菜单
  *   4. 建议顺序：2(串口) → 1(蜂鸣器) → 9(OLED) → 3(舵机)
  *                → 6(编码器) → 7(循迹) → 8(按键) → 4/5(电机)
  *
  * 【正式跑车时怎么处理】
  *   整个文件（连同 bsp_test.c）可以直接删掉，不影响其它模块 ——
  *   没有任何 bsp_* 模块反过来依赖它。只要把 main.c 里的调用去掉即可。
  * ============================================================================
  */
#ifndef __BSP_TEST_H
#define __BSP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/** @brief 打印菜单（上电时自动调用一次，之后输入 '?' 也能重新打印） */
void BSP_Test_ShowMenu(void);

/**
  * @brief  执行一条菜单命令
  * @param  cmd 用户输入的字符，见 BSP_Test_ShowMenu() 的清单
  * @note   测试都是阻塞的，跑完才会返回主循环
  */
void BSP_Test_RunCommand(char cmd);

/* --------------------------- 单项测试（阻塞） --------------------------- */

/** 1. 蜂鸣器：响 3 声。没声音时打印排查清单 */
void BSP_Test_Buzzer(void);

/** 2. 串口：打印整数/无符号/十六进制/浮点 + 逐字节输出，验证链路和格式化 */
void BSP_Test_Uart(void);

/** 3. 舵机：中位 → 右 → 中位 → 左 → 中位 慢速扫描，同时打印脉宽 */
void BSP_Test_Servo(void);

/** 4. 电机：正转 2s → 停 0.5s → 反转 2s → 停。用来确认转向和接线 */
void BSP_Test_Motor(void);

/** 5. 电机渐变：占空比 0 → 50% → 0，观察低速是否顺畅、有无啸叫 */
void BSP_Test_MotorRamp(void);

/** 6. 编码器：提示用手转轮，打印 3 秒内的增量 / 累计 / 转速 */
void BSP_Test_Encoder(void);

/** 7. 循迹：打印 3 秒内的掩码 / 压线数 / 位置误差 / 原始电平 */
void BSP_Test_Track(void);

/** 8. 按键：打印 5 秒内的所有事件（短按 / 长按 / 连发） */
void BSP_Test_Key(void);

/** 9. OLED：先显示自检图案 5s，再切到实时数值显示 5s */
void BSP_Test_Oled(void);

/** 0. 综合：按合理顺序把所有测试跑一遍 */
void BSP_Test_All(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TEST_H */
