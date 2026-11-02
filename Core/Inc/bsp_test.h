/**
  ******************************************************************************
  * @file    bsp_test.h
  * @brief   串口自检命令行（★ 已精简：只留最常用的几条）
  *
  * 【保留了什么 —— 只留"OLED 菜单里没有、又必须能测"的】
  *     4  电机正反转    换电机/改接线后必须验（OLED 里没有）
 *     6  编码器自检    转轮子看计数 + 自动给出"死活"结论（OLED 里没有）
  *     7  循迹传感器    打印掩码/压差/原始电平（和 SENSOR VIEW 互补）
  *     V  打印全部参数  一次把整套参数打出来，存底/对比用
  *     T  左右方向自检  原地左转→右转，确认 STEERINV 对不对（OLED 里没有）
  *     G  跑路线脚本    等价于菜单里的 RUN ROUTE
  *     M  回到 OLED 界面
  *     ?  重新打印菜单
  *
  * 【删掉了什么】
  *     1 蜂鸣器 / 2 串口回显 / 3 舵机扫描 / 5 电机渐变 /
  *     8 按键事件 / 9 OLED / 0 全部跑一遍 / P 蜂鸣器极性 / H 舵机保持 /
  *     C 启停循迹 / R 状态打印 / W 串口调参 / S 存参数 / D 恢复默认 /
  *     L 支撑脚测试 / E 串口接收监视
  *
  *   删除的理由：这些功能 OLED 菜单里全都有（调参、保存、支撑脚、探头、
  *   启停、状态显示…），而它们的字符串和代码占了固件里很大一块
  *   （精简前 bsp_test.c 一家就 12 kB，接近整片 Flash 的 1/5）。
  *   编码器读数更是不用留 —— 本车编码器没接。
  *
  *   ★ 以后真要用到某一条，告诉我，我再加回来（代码都还在我的记录里）。
  *
  * 【上游】 main.c 把收到的字符交给 BSP_Test_RunCommand()
  ******************************************************************************
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

/* --------------------------- 保留的单项测试 --------------------------- */

/** 4. 电机：正转 2s → 停 0.5s → 反转 2s → 停。用来确认转向和接线 */
void BSP_Test_Motor(void);

/** 7. 循迹：打印 3 秒内的掩码 / 压线数 / 位置误差 / 原始电平 */
void BSP_Test_Track(void);

/**
 * @brief  编码器自检：5 秒内用手转两个轮子，打印计数并给出"死活"结论
 * @note   阻塞 5 秒；内部自己调 BSP_Encoder_Update()。
 */
void BSP_Test_Encoder(void);

/** V. 打印当前全部参数（和刚上电时那份一样） */
void BSP_Test_ParamShow(void);

/**
  * @brief  M. 回到 OLED 状态页
  * @note   调参调晕了、或者别的测试把屏幕画花了，用它回到主界面
  */
void BSP_Test_MenuShow(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TEST_H */
