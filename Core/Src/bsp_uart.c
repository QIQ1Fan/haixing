/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   串口调试实现
  ******************************************************************************
  */
#include "bsp_uart.h"
#include "main.h"       /* Error_Handler() */

#include <stdarg.h>
#include <stdio.h>      /* vsnprintf */

UART_HandleTypeDef huart1;

/* ------------------------------- 初始化 -------------------------------- */

void BSP_UART_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* PA9  TX -> 复用推挽输出 */
    gpio.Pin   = DBG_UART_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DBG_UART_TX_PORT, &gpio);

    /* PA10 RX -> 浮空输入（外部模块驱动，不要上拉，否则可能读不到低电平） */
    gpio.Pin  = DBG_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DBG_UART_RX_PORT, &gpio);

    huart1.Instance          = DBG_UART;
    huart1.Init.BaudRate     = DBG_UART_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* ------------------------------- 发送接口 ------------------------------ */

void BSP_UART_SendByte(uint8_t byte)
{
    (void)HAL_UART_Transmit(&huart1, &byte, 1u, 0xFFFFu);
}

void BSP_UART_SendBuf(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u)) {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 0xFFFFu);
}

void BSP_UART_SendString(const char *str)
{
    if (str == NULL) {
        return;
    }
    while (*str != '\0') {
        BSP_UART_SendByte((uint8_t)(*str));
        str++;
    }
}

int BSP_UART_Printf(const char *fmt, ...)
{
    char    buf[DBG_PRINTF_BUF_SIZE];
    va_list args;
    int     len;

    if (fmt == NULL) {
        return 0;
    }

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) {
        return 0;
    }
    /* vsnprintf 返回"本该写入的长度"，可能大于缓冲区，这里按实际截断长度发送 */
    if (len >= (int)sizeof(buf)) {
        len = (int)sizeof(buf) - 1;
    }
    BSP_UART_SendBuf((const uint8_t *)buf, (uint16_t)len);

    return len;
}

/* ------------------------------- 接收接口 ------------------------------ */

uint8_t BSP_UART_GetChar(uint8_t *c)
{
    if (c == NULL) {
        return 0u;
    }

    /* 先处理溢出：F1 的 ORE 置位后会一直锁死接收，必须读 SR 再读 DR 才能清掉 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        return 0u;
    }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET) {
        return 0u;
    }

    *c = (uint8_t)(huart1.Instance->DR & 0xFFu);
    return 1u;
}

uint8_t BSP_UART_WaitChar(uint8_t *c, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    if (c == NULL) {
        return 0u;
    }

    while ((uint32_t)(HAL_GetTick() - start) < timeout_ms) {
        if (BSP_UART_GetChar(c) != 0u) {
            return 1u;
        }
    }
    return 0u;
}

/* ==========================================================================
 * 可选：把 fputc 重定向到 USART1，让标准 printf() 直接可用
 * ========================================================================== */
#if BSP_UART_USE_PRINTF_RETARGET

#if defined(__ARMCC_VERSION)
/* ---- ARM Compiler 5 / 6 ---- */
#if (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting\n\t");
#else
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
#endif

FILE __stdout;
FILE __stdin;

void _sys_exit(int x)
{
    (void)x;
    while (1) {
        /* 半主机模式下程序退出会死等，这里空转 */
    }
}

void _ttywrch(int ch)
{
    (void)ch;
}

int fputc(int ch, FILE *f)
{
    (void)f;
    BSP_UART_SendByte((uint8_t)ch);
    return ch;
}

#endif /* __ARMCC_VERSION */

#endif /* BSP_UART_USE_PRINTF_RETARGET */
