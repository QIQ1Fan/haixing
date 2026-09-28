/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   串口调试：USART1 / PA9(TX) / PA10(RX) / 115200-8-N-1
  *
  * 【在工程里的位置】 整个工程的调试通道，也是 bsp_test 自检菜单的输入输出口
  *
  *   发送： BSP_UART_Printf("RPM=%.2f\r\n", rpm)
  *            → vsnprintf 格式化到栈上缓冲 → HAL_UART_Transmit 阻塞发出
  *
  *   接收： 主循环里轮询 BSP_UART_GetChar()
  *            → 收到字符 → BSP_Test_RunCommand() 跑对应自检
  *
  * 【上游】 main.c 主循环轮询接收；任何模块都可以调它打印
  * 【下游】 电脑串口助手 / 蓝牙模块 / WiFi 模块
  *
  * 【占用的资源】 USART1 → PA9(TX) / PA10(RX)
  *
  * 【为什么用轮询而不是中断】
  *   调参场景下串口流量极低（人手敲键盘的速度），轮询完全够用，
  *   而且不用改 stm32f1xx_it.c、不用考虑中断优先级和共享变量竞争。
  *   如果以后要做上位机波形实时回传（每秒几百帧），再改成中断或 DMA。
  *
  * 【打印接口怎么选】
  *   BSP_UART_Printf()  —— 默认用这个。自带你自己的格式化，不依赖工具链的
  *                         stdio 重定向，换编译器也不会出问题。
  *   标准 printf()      —— 需要把 bsp_config.h 的 BSP_UART_USE_PRINTF_RETARGET
  *                         改成 1。好处是可以用 printf 的习惯写法，代价是
  *                         会屏蔽半主机调试（Event Recorder 用不了）。
  * ============================================================================
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/* 全局串口句柄。想在别处直接调 HAL_UART_xxx 时用它 */
extern UART_HandleTypeDef huart1;

/**
  * @brief  初始化 USART1（引脚 + 时钟 + 参数）
  * @note   纯轮询收发，不开中断，所以不需要动 stm32f1xx_it.c
  */
void BSP_UART_Init(void);

/** @brief 发送单字节（阻塞，直到发完） */
void BSP_UART_SendByte(uint8_t byte);

/** @brief 发送指定长度的数据（阻塞） */
void BSP_UART_SendBuf(const uint8_t *buf, uint16_t len);

/** @brief 发送以 '\0' 结尾的字符串（阻塞） */
void BSP_UART_SendString(const char *str);

/**
  * @brief  格式化输出，用法和 printf 一样
  * @retval 实际发出的字符数
  * @note   ★ 单次输出上限 DBG_PRINTF_BUF_SIZE（默认 128）字节，超出的部分
  *         会被截断（不会溢出缓冲区，但内容不全）。浮点 %f 可用。
  *         本函数不依赖 stdio 重定向，任何工具链下都能编过。
  *
  *   例： BSP_UART_Printf("L=%d R=%d err=%.2f\r\n", l, r, err);
  */
int BSP_UART_Printf(const char *fmt, ...);

/**
  * @brief  非阻塞读一个字节 —— 放在主循环里轮询
  * @param  c 输出缓冲
  * @retval 1 = 读到了, 0 = 暂无数据
  * @note   内置溢出（ORE）恢复。STM32F1 的串口一旦溢出，不处理的话
  *         接收会永久锁死，之后再也收不到任何数据 —— 这是轮询方案
  *         最容易踩的坑，所以这里每次都检查并清除。
  */
uint8_t BSP_UART_GetChar(uint8_t *c);

/**
  * @brief  阻塞等待一个字节
  * @param  timeout_ms 超时毫秒数
  * @retval 1 = 读到了, 0 = 超时
  */
uint8_t BSP_UART_WaitChar(uint8_t *c, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
