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

/** 3. 舵机：中位 → 180° → 中位 → 0° → 中位 慢速扫描，同时打印脉宽 */
void BSP_Test_Servo(void);

/** 4. 电机：正转 2s → 停 0.5s → 反转 2s → 停。用来确认转向和接线 */
void BSP_Test_Motor(void);

/** 5. 电机渐变：占空比 0 → 50% → 0，观察低速是否顺畅、有无啸叫 */
void BSP_Test_MotorRamp(void);

/** 6. 编码器：提示用手转轮，打印 3 秒内的增量 / 累计 / 转速 */
void BSP_Test_Encoder(void);

/** 7. 循迹：打印 3 秒内的掩码 / 压线数 / 位置误差 / 原始电平 */
void BSP_Test_Track(void);

/**
  * @brief  8. 按键：10 秒内打印所有事件（短按 / 长按 / 连发）
  * @note   ★ 同时显示四个引脚的【原始电平】（1=松开 0=按下）。
  *         这是分辨"没接对线"和"程序没认出来"的唯一手段：
  *           按住时电平 1->0 有变化 = 接线和引脚都对
  *           按住时电平一动不动     = 这一路没接到单片机，是硬件问题
  *         只看事件的话，这两种现象长得一模一样，没法区分。
  */
void BSP_Test_Key(void);

/** 9. OLED：先显示自检图案 5s，再切到实时数值显示 5s */
void BSP_Test_Oled(void);

/** 0. 综合：按合理顺序把所有测试跑一遍 */
void BSP_Test_All(void);

/* ----------------------------- 排障专用 ------------------------------- */

/**
  * @brief  P. 蜂鸣器极性自检
  * @note   交替输出低/高电平各 1 秒，让耳朵直接判断哪个电平是静音。
  *         故意绕开 BUZZER_ON_LEVEL / BUZZER_OFF_LEVEL 写裸电平，
  *         所以即使当前配置是反的也能正确测出来。
  *         蜂鸣器"响/静音"和预期相反时，先跑这个，别靠猜。
  */
void BSP_Test_BuzzerPolarity(void);

/**
  * @brief  H. 舵机隔离保持测试
  * @note   5 秒内停掉所有其它活动（不打印、不刷屏、不扫键），只让舵机
  *         保持 90°。用来区分抖动的来源：
  *           稳定 → 是别的任务或串口噪声在干扰
  *           仍抖 → 软件 PWM 本身，或供电/接线问题
  */
void BSP_Test_ServoHold(void);

/* ------------------------- 整车 / 调参命令 ------------------------- */

/**
  * @brief  C. 启动 / 停止循迹
  * @note   和 OLED 菜单里的 "START CAR" 是同一个动作。
  *         车必须在赛道上、支撑脚已收起时再按
  */
void BSP_Test_CarRun(void);

/**
  * @brief  R. 实时状态打印 5 秒
  * @note   100ms 打一行（约 90 字节 → 115200 下 8ms），
  *         ★ 刻意留出余量：如果每 10ms 打一行，光串口就把控制节拍吃光了，
  *         打印出来的现象本身就会把车搞得不正常
  */
void BSP_Test_CarStatus(void);

/**
  * @brief  V. 打印当前全部参数（和刚上电时那份一样）
  */
void BSP_Test_ParamShow(void);

/**
  * @brief  W. 串口调参：按序号改一个参数，并可选保存到 Flash
  * @note   给"手上没有 OLED 或者懒得按键"的场合用。
  *         交互式：先打印带序号的参数表，再等输入 "序号 值"
  */
void BSP_Test_ParamSet(void);

/** @brief  S. 把当前参数保存到 Flash（掉电不丢） */
void BSP_Test_ParamSave(void);

/** @brief  D. 恢复出厂默认参数（内存生效，要保存请再按 S） */
void BSP_Test_ParamDefault(void);

/**
  * @brief  L. 支撑脚测试：收起 → 放下 → 收起
  * @note   顺便可以标定 LEGUP / LEGDN 两个角度
  */
void BSP_Test_Leg(void);

/**
  * @brief  M. 回到 OLED 状态页
  * @note   调参调晕了、或者别的测试把屏幕画花了，用它回到主界面
  */
void BSP_Test_MenuShow(void);

/**
  * @brief  E. 串口接收监视：10 秒内把收到的每个字节按 "序号 十六进制 字符" 打出来
  *
  * 【什么时候用它】
  *   现象是"发命令没反应"时，第一件要做的事就是分清下面两种可能：
  *     a) 板子根本没收到 —— 接线问题（PA10/RX 没接、TX-RX 没交叉、没共地）
  *     b) 收到了但字节不对 —— 输入法全角、助手在发送时加了别的东西
  *   跑一次这个命令，两种情况的区别一目了然：
  *     全程 0 字节        → 是 (a)，去查线
  *     收到 0xEF 0xBC 0x9F → 是 (b)，全角「？」，切半角
  *     收到 0x3F           → 那就是 '?' 本身，命令表或代码有问题
  *
  * @note   ★ 车在跑的时候跑这个命令不会让车失控：循环内部自己维持
  *         10ms 的编码器采样和控制律（和 BSP_Test_CarStatus 一样的做法）
  */
void BSP_Test_UartMonitor(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TEST_H */
