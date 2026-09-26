/**
  ******************************************************************************
  * @file    bsp_test.h
  * @brief   底层驱动自检：每个模块一个可独立运行的测试函数
  * @note    通过串口菜单驱动（115200-8-N-1），也可以直接调单个函数。
  ******************************************************************************
  */
#ifndef __BSP_TEST_H
#define __BSP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/* 串口菜单 */
void BSP_Test_ShowMenu(void);

/**
  * @brief  执行一条菜单命令
  * @param  cmd 用户输入的字符，见 BSP_Test_ShowMenu() 打印的清单
  */
void BSP_Test_RunCommand(char cmd);

/* --------------------------- 单项测试（阻塞） --------------------------- */

/** 蜂鸣器：响 3 声 */
void BSP_Test_Buzzer(void);

/** 串口：打印一段文字和浮点数，验证链路与格式化输出 */
void BSP_Test_Uart(void);

/** 舵机：中位 -> 右 -> 中位 -> 左 -> 中位 慢速扫描 */
void BSP_Test_Servo(void);

/** 电机开环：正转 2s -> 停 0.5s -> 反转 2s -> 停 */
void BSP_Test_Motor(void);

/** 电机占空比渐变：0 -> 50% -> 0，观察低速是否顺畅 */
void BSP_Test_MotorRamp(void);

/** 编码器：提示用手转轮，打印 3 秒内的增量/累计/RPM */
void BSP_Test_Encoder(void);

/** 循迹：打印 3 秒内六路掩码、压线个数与位置误差 */
void BSP_Test_Track(void);

/** 按键：打印 5 秒内产生的事件（短按/长按/连发） */
void BSP_Test_Key(void);

/** OLED：显示测试图案与实时信息 */
void BSP_Test_Oled(void);

/** 综合：一次性跑完所有测试 */
void BSP_Test_All(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TEST_H */
