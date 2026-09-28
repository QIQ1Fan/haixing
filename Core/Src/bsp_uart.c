/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   串口调试实现 —— USART1 / 115200-8-N-1
  *
  * 【数据流】
  *   发送： BSP_UART_Printf("RPM=%.2f\r\n", rpm)
  *            └─ vsnprintf 格式化到栈上的 buf[128]
  *                 └─ HAL_UART_Transmit 逐字节阻塞发送
  *
  *   接收： 主循环每轮 ──► BSP_UART_GetChar(&ch)
  *            └─ 直接读 SR/DR 寄存器判断，非阻塞，没有数据就立刻返回 0
  *                 └─ main.c 把收到的字符交给 BSP_Test_RunCommand()
  *
  * 【为什么接收不用 HAL_UART_Receive】
  *   HAL 的那个函数即使 Timeout 传 0，内部也会做一连串状态检查、加锁、
  *   超时判断，每次调用有不小的开销。而这里只是"看一眼有没有数据"，
  *   直接查 RXNE 标志 + 读数据寄存器就够了，快得多。
  *
  * 【★ 溢出（ORE）必须处理】
  *   STM32F1 的串口如果来了数据但没被及时读走，ORE 标志会置位。
  *   一旦置位，如果不清除，接收通路就永久卡死 —— 之后再也收不到任何数据，
  *   而且没有任何报错。这是轮询式串口最隐蔽的坑，所以每次读之前都检查。
  ******************************************************************************
  */
#include "bsp_uart.h"
#include "main.h"       /* Error_Handler() */

#include <stdarg.h>
#include <stdio.h>      /* vsnprintf */

UART_HandleTypeDef huart1;

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_UART_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* PA9 TX → 复用推挽输出。
       串口是推挽驱动的（不是 I2C 那种开漏），不需要外部上拉 */
    gpio.Pin   = DBG_UART_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;  /* 115200 下边沿要够陡 */
    HAL_GPIO_Init(DBG_UART_TX_PORT, &gpio);

    /* PA10 RX → 浮空输入。
       ★ 这里特意不用上拉：如果外部模块的 TX 是开漏或驱动能力弱，
       内部上拉会把空闲电平顶在高位，导致收到的高电平被"粘住"、
       低电平塌不下去。浮空让外部信号完全主导 */
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
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;    /* 标准 16 倍过采样 */
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* ==========================================================================
 * 发送接口
 * ========================================================================== */

void BSP_UART_SendByte(uint8_t byte)
{
    /* 超时给 0xFFFF 而不是 HAL_MAX_DELAY：万一串口硬件真的有问题，
       宁愿超时返回也不要死等在这里把整个程序卡住 */
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

    /* 逐字节发送而不是一次性算 strlen 再发：
       字符串通常很短，省掉一次 strlen 遍历，代码也更简洁 */
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

    /* ★ vsnprintf 的返回值有个容易踩的坑：
       它返回的是"本该写入的长度"，而不是"实际写入的长度"。
       如果格式化结果比缓冲区长，返回值会大于 sizeof(buf)，
       但实际只写了 sizeof(buf)-1 个字符（末尾留 '\0'）。
       所以要夹一下，否则会把缓冲区外的内存发出去。 */
    if (len >= (int)sizeof(buf)) {
        len = (int)sizeof(buf) - 1;
    }
    BSP_UART_SendBuf((const uint8_t *)buf, (uint16_t)len);

    return len;
}

/* ==========================================================================
 * 接收接口
 * ========================================================================== */

uint8_t BSP_UART_GetChar(uint8_t *c)
{
    if (c == NULL) {
        return 0u;
    }

    /* 先处理溢出。★ 顺序很重要：必须放在检查 RXNE 之前。
       因为 ORE 置位时 RXNE 通常也会置位，若先看 RXNE 就会先返回数据、
       把清除 ORE 的机会错过，接收通路仍然是死锁状态。
       清除方法是"读 SR 再读 DR"，HAL 提供了宏封装。 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        return 0u;      /* 这一字节已经丢了，本次返回"无数据" */
    }

    /* 正常路径：RXNE = 接收数据寄存器非空，说明有数据可读 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET) {
        return 0u;
    }

    /* 读数据寄存器。这一步同时会清掉 RXNE 标志。
       只取低 8 位，因为 8 位无校验模式下高 8 位是无效的 */
    *c = (uint8_t)(huart1.Instance->DR & 0xFFu);
    return 1u;
}

uint8_t BSP_UART_WaitChar(uint8_t *c, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    if (c == NULL) {
        return 0u;
    }

    /* 有符号差值比较，HAL_GetTick() 回绕时依然正确 */
    while ((uint32_t)(HAL_GetTick() - start) < timeout_ms) {
        if (BSP_UART_GetChar(c) != 0u) {
            return 1u;
        }
    }
    return 0u;
}

/* ==========================================================================
 * 可选：把 fputc 重定向到 USART1，让标准 printf() 直接可用
 * --------------------------------------------------------------------------
 * 由 bsp_config.h 的 BSP_UART_USE_PRINTF_RETARGET 控制，默认关闭。
 * 关闭时这段代码完全不参与编译，不会有任何副作用。
 *
 * 开启的代价：__use_no_semihosting 会禁用半主机调试，
 * 于是 Keil 的 Event Recorder、ITM printf 就不能用了。
 * ========================================================================== */
#if BSP_UART_USE_PRINTF_RETARGET

#if defined(__ARMCC_VERSION)
/* ---- ARM Compiler 5 / 6 ---- */
#if (__ARMCC_VERSION >= 6010050)
/* AC6 用汇编指示符 */
__asm(".global __use_no_semihosting\n\t");
#else
/* AC5 用 pragma */
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
#endif

/* 半主机模式需要这几个符号，必须自己定义，否则链接会报
   "Symbol __stdout not defined" 之类的错误 */
FILE __stdout;
FILE __stdin;

void _sys_exit(int x)
{
    (void)x;
    while (1) {
        /* 半主机模式下程序"退出"会死等调试器响应，
           这里直接空转，避免程序跑飞 */
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
