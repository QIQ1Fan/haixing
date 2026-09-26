/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   串口调试：USART1 / PA9(TX) / PA10(RX) / 115200-8-N-1
  ******************************************************************************
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/* 全局串口句柄，供需要直接调用 HAL 的场合使用 */
extern UART_HandleTypeDef huart1;

/**
  * @brief  初始化 USART1（引脚 + 时钟 + 参数）
  * @note   纯轮询收发，不启用中断，因此不需要改 stm32f1xx_it.c。
  */
void BSP_UART_Init(void);

/** @brief 发送单字节（阻塞） */
void BSP_UART_SendByte(uint8_t byte);

/** @brief 发送指定长度的数据（阻塞） */
void BSP_UART_SendBuf(const uint8_t *buf, uint16_t len);

/** @brief 发送以 '\0' 结尾的字符串（阻塞） */
void BSP_UART_SendString(const char *str);

/**
  * @brief  格式化输出，等价于 printf
  * @retval 实际发出的字符数
  * @note   ★ 只支持 128 字节以内的输出（DBG_PRINTF_BUF_SIZE），超出会被截断。
  *         浮点 %f 可用。本函数不依赖 stdio 重定向，必定能编过。
  */
int BSP_UART_Printf(const char *fmt, ...);

/**
  * @brief  非阻塞读一个字节
  * @param  c 输出缓冲
  * @retval 1 = 读到了, 0 = 暂无数据
  * @note   内置溢出（ORE）恢复：不处理 ORE 的话，串口一旦溢出就会永久收不到数据。
  */
uint8_t BSP_UART_GetChar(uint8_t *c);

/** @brief 阻塞等待一个字节 */
uint8_t BSP_UART_WaitChar(uint8_t *c, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
