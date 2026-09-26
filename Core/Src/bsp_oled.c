/**
  ******************************************************************************
  * @file    bsp_oled.c
  * @brief   SSD1306 OLED 驱动实现（软件 I2C）
  *
  * 硬件说明：
  *   PB8 = SCL, PB9 = SDA，配置为推挽输出。
  *   标准 I2C 要求开漏，但单主机、无时钟拉伸、总线上只有一片 SSD1306 时，
  *   推挽驱动完全可用，而且省掉外部上拉电阻。若总线上还挂了别的从机，
  *   请把这两个脚改成 GPIO_MODE_OUTPUT_OD 并外接 4.7k 上拉。
  *
  * 显存布局：
  *   1024 字节，按页组织。s_buf[page * 128 + x]，字节的 bit0 是每页最上面一行。
  *   SSD1306 原生的页寻址格式就是这样，所以刷屏时可以直接整块丢过去。
  ******************************************************************************
  */
#include "bsp_oled.h"

#include <stdarg.h>
#include <stdio.h>      /* vsnprintf */

#define OLED_BUF_SIZE       ((uint32_t)OLED_WIDTH * OLED_PAGE_NUM)   /* 1024 */

#define OLED_CTRL_CMD       0x00u
#define OLED_CTRL_DATA      0x40u

static uint8_t s_buf[OLED_BUF_SIZE];

/* ==========================================================================
 * 6x8 ASCII 字库：可打印字符 0x20..0x7E，共 95 个，每个字符 6 列
 * 每列 1 字节，bit0 = 最上面一行
 * ========================================================================== */
static const uint8_t s_font6x8[95][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* ' ' */
    {0x00, 0x00, 0x00, 0x2F, 0x00, 0x00}, /* '!' */
    {0x00, 0x00, 0x07, 0x00, 0x07, 0x00}, /* '"' */
    {0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14}, /* '#' */
    {0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* '$' */
    {0x00, 0x62, 0x64, 0x08, 0x13, 0x23}, /* '%' */
    {0x00, 0x36, 0x49, 0x55, 0x22, 0x50}, /* '&' */
    {0x00, 0x00, 0x05, 0x03, 0x00, 0x00}, /* '\'' */
    {0x00, 0x00, 0x1C, 0x22, 0x41, 0x00}, /* '(' */
    {0x00, 0x00, 0x41, 0x22, 0x1C, 0x00}, /* ')' */
    {0x00, 0x14, 0x08, 0x3E, 0x08, 0x14}, /* '*' */
    {0x00, 0x08, 0x08, 0x3E, 0x08, 0x08}, /* '+' */
    {0x00, 0x00, 0x00, 0xA0, 0x60, 0x00}, /* ',' */
    {0x00, 0x08, 0x08, 0x08, 0x08, 0x08}, /* '-' */
    {0x00, 0x00, 0x60, 0x60, 0x00, 0x00}, /* '.' */
    {0x00, 0x20, 0x10, 0x08, 0x04, 0x02}, /* '/' */
    {0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E}, /* '0' */
    {0x00, 0x00, 0x42, 0x7F, 0x40, 0x00}, /* '1' */
    {0x00, 0x42, 0x61, 0x51, 0x49, 0x46}, /* '2' */
    {0x00, 0x21, 0x41, 0x45, 0x4B, 0x31}, /* '3' */
    {0x00, 0x18, 0x14, 0x12, 0x7F, 0x10}, /* '4' */
    {0x00, 0x27, 0x45, 0x45, 0x45, 0x39}, /* '5' */
    {0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30}, /* '6' */
    {0x00, 0x01, 0x71, 0x09, 0x05, 0x03}, /* '7' */
    {0x00, 0x36, 0x49, 0x49, 0x49, 0x36}, /* '8' */
    {0x00, 0x06, 0x49, 0x49, 0x29, 0x1E}, /* '9' */
    {0x00, 0x00, 0x36, 0x36, 0x00, 0x00}, /* ':' */
    {0x00, 0x00, 0x56, 0x36, 0x00, 0x00}, /* ';' */
    {0x00, 0x08, 0x14, 0x22, 0x41, 0x00}, /* '<' */
    {0x00, 0x14, 0x14, 0x14, 0x14, 0x14}, /* '=' */
    {0x00, 0x00, 0x41, 0x22, 0x14, 0x08}, /* '>' */
    {0x00, 0x02, 0x01, 0x51, 0x09, 0x06}, /* '?' */
    {0x00, 0x32, 0x49, 0x59, 0x51, 0x3E}, /* '@' */
    {0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C}, /* 'A' */
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x36}, /* 'B' */
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x22}, /* 'C' */
    {0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C}, /* 'D' */
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x41}, /* 'E' */
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x01}, /* 'F' */
    {0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A}, /* 'G' */
    {0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 'H' */
    {0x00, 0x00, 0x41, 0x7F, 0x41, 0x00}, /* 'I' */
    {0x00, 0x20, 0x40, 0x41, 0x3F, 0x01}, /* 'J' */
    {0x00, 0x7F, 0x08, 0x14, 0x22, 0x41}, /* 'K' */
    {0x00, 0x7F, 0x40, 0x40, 0x40, 0x40}, /* 'L' */
    {0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 'M' */
    {0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F}, /* 'N' */
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 'O' */
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x06}, /* 'P' */
    {0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 'Q' */
    {0x00, 0x7F, 0x09, 0x19, 0x29, 0x46}, /* 'R' */
    {0x00, 0x46, 0x49, 0x49, 0x49, 0x31}, /* 'S' */
    {0x00, 0x01, 0x01, 0x7F, 0x01, 0x01}, /* 'T' */
    {0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 'U' */
    {0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 'V' */
    {0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F}, /* 'W' */
    {0x00, 0x63, 0x14, 0x08, 0x14, 0x63}, /* 'X' */
    {0x00, 0x07, 0x08, 0x70, 0x08, 0x07}, /* 'Y' */
    {0x00, 0x61, 0x51, 0x49, 0x45, 0x43}, /* 'Z' */
    {0x00, 0x00, 0x7F, 0x41, 0x41, 0x00}, /* '[' */
    {0x00, 0x02, 0x04, 0x08, 0x10, 0x20}, /* '\\' */
    {0x00, 0x00, 0x41, 0x41, 0x7F, 0x00}, /* ']' */
    {0x00, 0x04, 0x02, 0x01, 0x02, 0x04}, /* '^' */
    {0x00, 0x40, 0x40, 0x40, 0x40, 0x40}, /* '_' */
    {0x00, 0x00, 0x01, 0x02, 0x04, 0x00}, /* '`' */
    {0x00, 0x20, 0x54, 0x54, 0x54, 0x78}, /* 'a' */
    {0x00, 0x7F, 0x48, 0x44, 0x44, 0x38}, /* 'b' */
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x20}, /* 'c' */
    {0x00, 0x38, 0x44, 0x44, 0x48, 0x7F}, /* 'd' */
    {0x00, 0x38, 0x54, 0x54, 0x54, 0x18}, /* 'e' */
    {0x00, 0x08, 0x7E, 0x09, 0x01, 0x02}, /* 'f' */
    {0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C}, /* 'g' */
    {0x00, 0x7F, 0x08, 0x04, 0x04, 0x78}, /* 'h' */
    {0x00, 0x00, 0x44, 0x7D, 0x40, 0x00}, /* 'i' */
    {0x00, 0x40, 0x80, 0x84, 0x7D, 0x00}, /* 'j' */
    {0x00, 0x7F, 0x10, 0x28, 0x44, 0x00}, /* 'k' */
    {0x00, 0x00, 0x41, 0x7F, 0x40, 0x00}, /* 'l' */
    {0x00, 0x7C, 0x04, 0x18, 0x04, 0x78}, /* 'm' */
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x78}, /* 'n' */
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x38}, /* 'o' */
    {0x00, 0xFC, 0x24, 0x24, 0x24, 0x18}, /* 'p' */
    {0x00, 0x18, 0x24, 0x24, 0x18, 0xFC}, /* 'q' */
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x08}, /* 'r' */
    {0x00, 0x48, 0x54, 0x54, 0x54, 0x20}, /* 's' */
    {0x00, 0x04, 0x3F, 0x44, 0x40, 0x20}, /* 't' */
    {0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C}, /* 'u' */
    {0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C}, /* 'v' */
    {0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C}, /* 'w' */
    {0x00, 0x44, 0x28, 0x10, 0x28, 0x44}, /* 'x' */
    {0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C}, /* 'y' */
    {0x00, 0x44, 0x64, 0x54, 0x4C, 0x44}, /* 'z' */
    {0x00, 0x00, 0x08, 0x36, 0x41, 0x00}, /* '{' */
    {0x00, 0x00, 0x00, 0x7F, 0x00, 0x00}, /* '|' */
    {0x00, 0x00, 0x41, 0x36, 0x08, 0x00}, /* '}' */
    {0x00, 0x08, 0x04, 0x08, 0x10, 0x08}, /* '~' */
};

/* ==========================================================================
 * 软件 I2C 底层
 * ========================================================================== */

static void i2c_delay(void)
{
    volatile uint32_t i;
    for (i = 0u; i < OLED_I2C_DELAY_LOOPS; i++) {
        /* 空循环，靠循环次数拉长半周期 */
    }
}

static void scl_high(void) { HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET); }
static void scl_low(void)  { HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET); }
static void sda_high(void) { HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET); }
static void sda_low(void)  { HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET); }

static void i2c_start(void)
{
    sda_high();
    scl_high();
    i2c_delay();
    sda_low();      /* SCL 为高时 SDA 下降沿 = 起始 */
    i2c_delay();
    scl_low();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_high();
    i2c_delay();
    sda_high();     /* SCL 为高时 SDA 上升沿 = 停止 */
    i2c_delay();
}

/** @brief 写一个字节并吞掉 ACK 位（不检查应答） */
static void i2c_write_byte(uint8_t byte)
{
    uint8_t i;

    for (i = 0u; i < 8u; i++) {
        if ((byte & 0x80u) != 0u) {
            sda_high();
        } else {
            sda_low();
        }
        byte = (uint8_t)(byte << 1);
        i2c_delay();
        scl_high();
        i2c_delay();
        scl_low();
        i2c_delay();
    }

    /* 第 9 个时钟：释放 SDA，让从机拉低表示 ACK。这里不读，直接跳过 */
    sda_high();
    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
    i2c_delay();
}

/** @brief 向 SSD1306 写一段数据，ctrl 决定是命令还是显存数据 */
static void oled_write(uint8_t ctrl, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((data == NULL) || (len == 0u)) {
        return;
    }

    i2c_start();
    i2c_write_byte((uint8_t)OLED_I2C_ADDR);
    i2c_write_byte(ctrl);
    for (i = 0u; i < len; i++) {
        i2c_write_byte(data[i]);
    }
    i2c_stop();
}

/* ==========================================================================
 * 显存操作
 * ========================================================================== */

void BSP_OLED_Clear(void)
{
    uint32_t i;

    for (i = 0u; i < OLED_BUF_SIZE; i++) {
        s_buf[i] = 0x00u;
    }
}

void BSP_OLED_Refresh(void)
{
    /* 水平寻址模式下，先框定整个屏幕的窗口，然后一口气流 1024 字节，
       SSD1306 会自动换行换页，不需要逐页设置地址 */
    static const uint8_t col_cmd[3]  = { 0x21u, 0x00u, (uint8_t)(OLED_WIDTH - 1u) };
    static const uint8_t page_cmd[3] = { 0x22u, 0x00u, (uint8_t)(OLED_PAGE_NUM - 1u) };

    oled_write(OLED_CTRL_CMD, col_cmd, 3u);
    oled_write(OLED_CTRL_CMD, page_cmd, 3u);
    oled_write(OLED_CTRL_DATA, s_buf, (uint16_t)OLED_BUF_SIZE);
}

void BSP_OLED_ClearAndRefresh(void)
{
    BSP_OLED_Clear();
    BSP_OLED_Refresh();
}

void BSP_OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
    uint16_t idx;
    uint8_t  bit;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return;
    }

    idx = (uint16_t)(((uint16_t)(y >> 3) * OLED_WIDTH) + x);
    bit = (uint8_t)(1u << (y & 0x07u));

    if (on != 0u) {
        s_buf[idx] |= bit;
    } else {
        s_buf[idx] &= (uint8_t)(~bit);
    }
}

/* ==========================================================================
 * 字符绘制
 * ========================================================================== */

void BSP_OLED_DrawString(uint8_t x, uint8_t page, const char *str)
{
    uint8_t c;
    uint8_t col;

    if ((str == NULL) || (page >= OLED_PAGE_NUM)) {
        return;
    }

    while (*str != '\0') {
        c = (uint8_t)(*str);

        /* 字库里没有的字符统一用空格代替 */
        if ((c < 0x20u) || (c > 0x7Eu)) {
            c = 0x20u;
        }

        /* 右边放不下就截断，不换行 */
        if ((uint16_t)x + 6u > OLED_WIDTH) {
            break;
        }

        for (col = 0u; col < 6u; col++) {
            s_buf[((uint16_t)page * OLED_WIDTH) + x + col] =
                s_font6x8[c - 0x20u][col];
        }

        x = (uint8_t)(x + 6u);
        str++;
    }
}

void BSP_OLED_DrawStringBig(uint8_t x, uint8_t page, const char *str)
{
    uint8_t c;
    uint8_t col;

    if ((str == NULL) || (page + 1u >= OLED_PAGE_NUM)) {
        return;
    }

    while (*str != '\0') {
        c = (uint8_t)(*str);

        if ((c < 0x20u) || (c > 0x7Eu)) {
            c = 0x20u;
        }

        if ((uint16_t)x + 12u > OLED_WIDTH) {
            break;
        }

        for (col = 0u; col < 6u; col++) {
            uint8_t src = s_font6x8[c - 0x20u][col];
            uint8_t lo = 0u;    /* 上半页：源字节的 bit0..bit3 */
            uint8_t hi = 0u;    /* 下半页：源字节的 bit4..bit7 */
            uint8_t k;

            /* 每个源位横向复制成 2 位，实现 2 倍放大 */
            for (k = 0u; k < 4u; k++) {
                if ((src & (uint8_t)(1u << k)) != 0u) {
                    lo |= (uint8_t)(0x03u << (2u * k));
                }
            }
            for (k = 0u; k < 4u; k++) {
                if ((src & (uint8_t)(1u << (k + 4u))) != 0u) {
                    hi |= (uint8_t)(0x03u << (2u * k));
                }
            }

            /* 纵向复制成 2 列 */
            s_buf[((uint16_t)page * OLED_WIDTH) + x + (2u * col)]         = lo;
            s_buf[((uint16_t)page * OLED_WIDTH) + x + (2u * col) + 1u]    = lo;
            s_buf[((uint16_t)(page + 1u) * OLED_WIDTH) + x + (2u * col)]     = hi;
            s_buf[((uint16_t)(page + 1u) * OLED_WIDTH) + x + (2u * col) + 1u] = hi;
        }

        x = (uint8_t)(x + 12u);
        str++;
    }
}

void BSP_OLED_Printf(uint8_t x, uint8_t page, const char *fmt, ...)
{
    char    buf[64];
    va_list args;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    BSP_OLED_DrawString(x, page, buf);
}

void BSP_OLED_PrintfBig(uint8_t x, uint8_t page, const char *fmt, ...)
{
    char    buf[32];
    va_list args;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    BSP_OLED_DrawStringBig(x, page, buf);
}

/* ==========================================================================
 * 图形绘制
 * ========================================================================== */

void BSP_OLED_DrawHLine(uint8_t x, uint8_t page, uint8_t width)
{
    uint16_t base;
    uint8_t  i;

    if (page >= OLED_PAGE_NUM) {
        return;
    }

    base = (uint16_t)page * OLED_WIDTH;

    for (i = 0u; i < width; i++) {
        uint16_t px = (uint16_t)x + i;
        if (px >= OLED_WIDTH) {
            break;
        }
        /* 画在页的中间一行，视觉上就是一条细线 */
        s_buf[base + px] |= 0x08u;
    }
}

void BSP_OLED_DrawBar(uint8_t x, uint8_t page, uint8_t width,
                      uint8_t height_pages, uint8_t percent)
{
    uint8_t  filled;
    uint8_t  p;
    uint16_t base;
    uint8_t  i;

    if ((page >= OLED_PAGE_NUM) || (height_pages == 0u) || (width == 0u)) {
        return;
    }
    if (page + height_pages > OLED_PAGE_NUM) {
        height_pages = (uint8_t)(OLED_PAGE_NUM - page);
    }
    if (percent > 100u) {
        percent = 100u;
    }

    filled = (uint8_t)(((uint16_t)width * percent) / 100u);

    /* 画边框 */
    BSP_OLED_DrawHLine(x, page, width);
    BSP_OLED_DrawHLine(x, (uint8_t)(page + height_pages - 1u), width);

    for (i = 0u; i < width; i++) {
        uint16_t px = (uint16_t)x + i;
        if (px >= OLED_WIDTH) {
            break;
        }
        for (p = 0u; p < height_pages; p++) {
            base = (uint16_t)(page + p) * OLED_WIDTH;
            if ((i == 0u) || (i == width - 1u)) {
                /* 左右竖边 */
                s_buf[base + px] |= 0x18u;
            } else if (i <= filled) {
                /* 填充部分 */
                s_buf[base + px] |= 0x18u;
            }
        }
    }
}

void BSP_OLED_TestPattern(void)
{
    uint8_t i;
    uint8_t p;

    BSP_OLED_Clear();

    /* 四条边框 */
    for (i = 0u; i < OLED_WIDTH; i++) {
        BSP_OLED_SetPixel(i, 0u, 1u);
        BSP_OLED_SetPixel(i, (uint8_t)(OLED_HEIGHT - 1u), 1u);
    }
    for (i = 0u; i < OLED_HEIGHT; i++) {
        BSP_OLED_SetPixel(0u, i, 1u);
        BSP_OLED_SetPixel((uint8_t)(OLED_WIDTH - 1u), i, 1u);
    }

    /* 斜线，用来检查有没有整行/整列错位 */
    for (i = 0u; i < OLED_WIDTH; i++) {
        uint8_t y = (uint8_t)((i + 3u) % (OLED_HEIGHT - 8u));
        BSP_OLED_SetPixel(i, (uint8_t)(y + 4u), 1u);
    }

    /* 文字：6x8 与 12x16 各一行 */
    BSP_OLED_DrawString(4u, 1u, "SSD1306 128x64 OK");
    BSP_OLED_DrawString(4u, 2u, "0123456789 ABCDEF");
    BSP_OLED_DrawStringBig(4u, 3u, "Haixing");

    /* 进度条 */
    BSP_OLED_DrawBar(4u, 6u, 120u, 1u, 66u);

    /* 每页各点一个像素：若某一页不亮，说明对应的 COM 线虚焊 */
    for (p = 0u; p < OLED_PAGE_NUM; p++) {
        BSP_OLED_SetPixel(124u, (uint8_t)((p * 8u) + 4u), 1u);
    }

    BSP_OLED_Refresh();
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_OLED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    static const uint8_t init_seq[] = {
        0xAEu,              /* 关显示 */
        0x20u, 0x00u,       /* 水平寻址模式 */
        0xB0u,              /* 页起始地址 0 */
        0xC8u,              /* COM 扫描方向：上下翻转（屏幕正装） */
        0x00u,              /* 低列地址 */
        0x10u,              /* 高列地址 */
        0x40u,              /* 起始行 0 */
        0x81u, 0x7Fu,       /* 对比度 */
        0xA1u,              /* 段重映射：左右翻转（屏幕正装） */
        0xA6u,              /* 正常显示（非反色） */
        0xA8u, 0x3Fu,       /* 多路复用比 63（64 行） */
        0xA4u,              /* 显示跟随显存 */
        0xD3u, 0x00u,       /* 显示偏移 0 */
        0xD5u, 0x80u,       /* 时钟分频 */
        0xD9u, 0xF1u,       /* 预充电周期 */
        0xDAu, 0x12u,       /* COM 引脚配置 */
        0xDBu, 0x40u,       /* VCOMH 电压 */
        0x8Du, 0x14u,       /* ★ 电荷泵使能：3.3V 供电必须打开，否则不亮 */
        0xAFu               /* 开显示 */
    };

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 两根线都先拉高，避免起始时产生误起始条件 */
    HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET);

    gpio.Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_SCL_PORT, &gpio);

    HAL_Delay(100u);    /* 等 SSD1306 内部上电复位完成 */

    oled_write(OLED_CTRL_CMD, init_seq, (uint16_t)sizeof(init_seq));

    BSP_OLED_ClearAndRefresh();
}
