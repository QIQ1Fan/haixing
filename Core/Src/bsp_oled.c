/**
  ******************************************************************************
  * @file    bsp_oled.c
  * @brief   SSD1306 OLED 驱动实现（软件 I2C）
  *
  * 【数据流】
  *   绘制函数（DrawString / SetPixel / DrawBar ...）
  *     └─► 只改内存里的 1KB 显存 s_buf[]（这一步极快，微秒级）
  *           └─► BSP_OLED_Refresh()
  *                 └─► 软件 I2C 把整块 1024 字节一次推给屏幕（约几毫秒）
  *                       └─► SSD1306 显示
  *
  * 【★ 为什么是"改显存 + 整屏刷新"，而不是画一个点就发一次 I2C】
  *   如果每画一个像素就发一次 I2C：
  *     · 屏幕上会看到逐点出现的撕裂过程，观感很差
  *     · I2C 事务次数暴增，刷一屏要几百次起始/停止，极慢
  *   先在内存里把一整幅图拼好，再一次性推出，屏幕上永远是完整的一帧。
  *   代价是占 1KB RAM —— F103C8 有 20KB，完全够用。
  *
  * 【显存布局（和 SSD1306 原生格式完全一致）】
  *   1024 字节，按页组织：s_buf[page * 128 + x]
  *   字节的 bit0 是该页最上面一行：
  *
  *       页 0 ─┬─ bit0  ← y=0
  *             ├─ bit1  ← y=1
  *             │  ...
  *             └─ bit7  ← y=7
  *       页 1 ─── bit0  ← y=8
  *             ...
  *       页 7 ─── bit7  ← y=63
  *
  *   正因为内存里的排布和屏幕的原生格式一模一样，刷屏时才能把整块数据
  *   原样丢过去，不需要任何逐位转换 —— 这是本驱动效率的关键。
  *
  * 【硬件说明：为什么推挽输出也能当 I2C 用】
  *   标准 I2C 要求开漏（OD）输出 + 外部上拉，这样多个设备才能"线与"。
  *   但这里总线是单主机、只有一片 SSD1306、且屏幕从不做时钟拉伸，
  *   推挽驱动完全可用，还省掉两个上拉电阻。
  *   如果以后要在同一条总线上挂第二个 I2C 器件，必须改成
  *   GPIO_MODE_OUTPUT_OD 并外接 4.7k 上拉，否则会烧引脚。
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
 * 影子帧：精确找出"这一帧到底哪些页变了"
 * --------------------------------------------------------------------------
 *   s_buf   = 当前帧（所有绘制函数往这里画）
 *   s_shown = 屏幕上真正显示着的那一帧（每次成功刷新后同步）
 *
 * 【为什么用"整帧对比"而不是"画的时候就标脏"】
 *   标脏的做法看似更省事，但它会被最常见的界面写法骗过去：
 *   界面代码几乎都是"先清屏、再按当前状态整帧重画"，
 *   清屏会把 8 页全标脏 —— 于是一帧一帧全都是整屏刷新，
 *   优化的意义荡然无存。
 *
 *   整帧对比没有这个问题：清屏 + 重画之后，内容没变的地方字节完全一样，
 *   一比就知道"这几页根本没动"，只把真正变化的页发出去。
 *   代价是 1KB 影子缓冲（F103C8 有 20KB RAM，完全放得下）
 *   和每帧 1024 次字节比较（几十微秒，可以忽略）。
 *
 * 【收益有多大】
 *   整块显存 1024 字节走软件 I2C 要 40~50ms。跑车时主循环每 10ms 一拍，
 *   每拍整屏刷新等于把一半的时间花在刷屏上，编码器采样和控制律全乱。
 *   而实际一帧里通常只有三四页在变（数值在跳的那几行），
 *   只刷这几页只要 10~15ms，对控制节拍的影响小得多。
 * ========================================================================== */
static uint8_t s_shown[OLED_BUF_SIZE];

/* 1 = 影子帧不可信（刚上电、或者别人直接写过屏幕），下一次刷新强制整屏 */
static uint8_t s_force_full = 1u;

/* ==========================================================================
 * 6x8 ASCII 字库
 * --------------------------------------------------------------------------
 * 可打印字符 0x20(' ') .. 0x7E('~')，共 95 个。下标 = 字符码 - 0x20。
 *
 * 每个字符 6 列，每列 1 字节；字节的 bit0 是这一列的**最上面一行**。
 * 以 '1' 为例（0x00,0x42,0x7F,0x40,0x00,0x00）：
 *
 *   列0=0x00 这一列全空
 *   列1=0x42 → bit1 和 bit6 亮      ██
 *   列2=0x7F → 低 7 位全亮        ████
 *   列3=0x40 → bit6 亮
 *   列4=0x00 空
 *   列5=0x00 空
 *
 * 注意这里是"列优先"（先排完一列的所有行，再排下一列），
 * 和显存的页式布局天然吻合，所以画同页文字时可以直接逐列拷贝，
 * 不用做任何位移。
 *
 * 字库只占了 570 字节，换来的是完全不需要外部字库芯片。
 * 想改成 8x16 字体的话，把这个数组换掉并调整 DrawString 的列数和步进即可。
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
 * --------------------------------------------------------------------------
 * I2C 协议只靠两根线的高低电平变化实现，所以"软件 I2C"就是用普通 GPIO
 * 手动模拟出这些变化。核心规则只有三条：
 *
 *   1. SCL 高电平期间，SDA 必须保持稳定 —— 此刻的 SDA 就是数据位。
 *   2. SCL 为高时 SDA 下降沿 = 起始条件（START）
 *   3. SCL 为高时 SDA 上升沿 = 停止条件（STOP）
 *
 * 时序图（一次起始 + 一个字节 0x5A 的数据位，简化画法）：
 *
 *   SCL ‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾
 *   SDA ‾‾‾‾‾\________/‾‾‾‾\____
 *            ↑START   ↑数据位在 SCL 高电平期间保持稳定
 * ========================================================================== */

/**
  * @brief  半个时钟周期的延时
  *
  * ★ volatile 不能省。没有它，编译器看到这个循环"什么也没干"，
  *   在 -O2 之类的优化级别下会把整个循环删掉，于是 I2C 时序全乱、
  *   屏幕不亮。volatile 告诉编译器"这里有副作用，不许优化掉"。
  *
  * 快慢由 OLED_I2C_DELAY_LOOPS 控制：屏幕无显示或偶发乱码时调大它。
  */
static void i2c_delay(void)
{
    volatile uint32_t i;
    for (i = 0u; i < OLED_I2C_DELAY_LOOPS; i++) {
        /* 空循环，纯粹用来拖时间 */
    }
}

/* 四个引脚操作封装成函数，让下面的时序代码读起来像协议本身 */
static void scl_high(void) { HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_SET); }
static void scl_low(void)  { HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN, GPIO_PIN_RESET); }
static void sda_high(void) { HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_SET); }
static void sda_low(void)  { HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN, GPIO_PIN_RESET); }

/**
  * @brief  起始条件：SCL 保持高时，SDA 由高变低
  * @note   两根线都先拉高是为了保证起始前总线处于"空闲"状态
  */
static void i2c_start(void)
{
    sda_high();
    scl_high();
    i2c_delay();
    sda_low();      /* ← 这一下就是 START */
    i2c_delay();
    scl_low();      /* 拉低 SCL，为传输第一个数据位做准备 */
    i2c_delay();
}

/**
  * @brief  停止条件：SCL 保持高时，SDA 由低变高
  * @note   注意顺序和起始相反 —— 先把 SDA 拉低确保是个确定状态，
  *         再抬 SCL，最后抬 SDA 产生上升沿
  */
static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_high();
    i2c_delay();
    sda_high();     /* ← 这一下就是 STOP */
    i2c_delay();
}

/**
  * @brief  写一个字节（MSB 先行）并吞掉应答位
  *
  * 顺序：先准备好 SDA，再抬 SCL（此时 SDA 已被从机采样），再拉低 SCL。
  * 循环 8 次送出 8 位，每次都从最高位开始 —— I2C 协议规定 MSB first，
  * 所以用 & 0x80 取最高位、然后左移把下一位顶上来。
  *
  * @note   这里不检查从机的应答（ACK）。正常调试没问题，但如果以后要
  *         做"屏幕掉线检测"，就在第 9 个时钟把 SDA 切成输入读一下电平：
  *         从机会把它拉低表示 ACK，读到高电平说明设备没响应。
  */
static void i2c_write_byte(uint8_t byte)
{
    uint8_t i;

    for (i = 0u; i < 8u; i++) {
        /* 数据位必须在 SCL 拉高之前就准备好 */
        if ((byte & 0x80u) != 0u) {
            sda_high();
        } else {
            sda_low();
        }
        byte = (uint8_t)(byte << 1);    /* 把下一位顶到最高位 */

        i2c_delay();
        scl_high();     /* ← 从机在这一刻采样 SDA */
        i2c_delay();
        scl_low();
        i2c_delay();
    }

    /* 第 9 个时钟：主机释放 SDA（拉高），让从机有机会把它拉低表示 ACK。
       本驱动不读应答，走个过场让时钟完整即可 */
    sda_high();
    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
    i2c_delay();
}

/**
  * @brief  向 SSD1306 写一段数据
  * @param  ctrl 控制字节：0x00 = 后面是命令，0x40 = 后面是显存数据
  * @param  data 数据内容
  * @param  len  字节数
  *
  * SSD1306 的 I2C 帧格式： [地址] [控制字节] [数据...]
  * 控制字节能区分"这是命令还是像素数据"，是因为 SSD1306 把两种含义
  * 复用在同一条总线上。
  */
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

    /* 注意：这里不动影子帧。
       清屏之后 s_buf 全是 0，而影子帧还留着上一帧的内容，
       对比时自然就能发现"哪几页被清掉了"，只把这几页刷下去即可 —— 
       这正是整帧对比的方便之处 */
}

void BSP_OLED_RefreshPages(uint8_t first, uint8_t last)
{
    /* SSD1306 的两条窗口命令：
         0x21 设置列地址范围，后跟 起始列 和 结束列
         0x22 设置页地址范围，后跟 起始页 和 结束页
       窗口定好之后，连续写入的数据会自动按"列递增 → 列满换页"的顺序
       填满窗口。所以送一段连续显存只需要设置一次窗口，中间不用再干预。

       ★ 页区间必须夹住：窗口命令给的是真实寄存器值，
         越界会让屏幕进入未定义状态（常见表现为花屏或整屏错行） */
    uint8_t  col_cmd[3];
    uint8_t  page_cmd[3];
    uint32_t i;

    if (first >= OLED_PAGE_NUM) {
        return;
    }
    if (last >= OLED_PAGE_NUM) {
        last = (uint8_t)(OLED_PAGE_NUM - 1u);
    }
    if (last < first) {
        return;
    }

    col_cmd[0]  = 0x21u;
    col_cmd[1]  = 0x00u;
    col_cmd[2]  = (uint8_t)(OLED_WIDTH - 1u);

    page_cmd[0] = 0x22u;
    page_cmd[1] = first;
    page_cmd[2] = last;

    oled_write(OLED_CTRL_CMD, col_cmd, 3u);
    oled_write(OLED_CTRL_CMD, page_cmd, 3u);

    /* 只送这几页：起始地址 = first × 128（每页 128 字节） */
    oled_write(OLED_CTRL_DATA,
               &s_buf[(uint32_t)first * OLED_WIDTH],
               (uint16_t)((uint16_t)(last - first + 1u) * OLED_WIDTH));

    /* ★ 发完就把这几页同步进影子帧。
       少同步一次，下次对比就会认为"这几页还没显示"，
       于是把它们又白刷一遍 —— 不影响正确性，但白费时间 */
    for (i = (uint32_t)first * OLED_WIDTH; i < ((uint32_t)last + 1u) * OLED_WIDTH; i++) {
        s_shown[i] = s_buf[i];
    }
}

void BSP_OLED_MarkAllDirty(void)
{
    /* 不真的去改影子帧的内容（改了反而更难判断），
       只竖一个"下一次必须整屏刷"的旗子 */
    s_force_full = 1u;
}

uint8_t BSP_OLED_HasDirty(void)
{
    uint32_t i;

    if (s_force_full != 0u) {
        return 1u;
    }

    for (i = 0u; i < OLED_BUF_SIZE; i++) {
        if (s_buf[i] != s_shown[i]) {
            return 1u;
        }
    }
    return 0u;
}

void BSP_OLED_RefreshDirty(void)
{
    uint8_t  p;
    uint8_t  first = 0xFFu;
    uint8_t  last  = 0x00u;
    uint32_t i;

    if (s_force_full != 0u) {
        /* 影子帧不可信（上电第一帧、或者别人绕过本驱动直接写过屏幕），
           老老实实整屏刷一次，之后就一直走精确对比的快路 */
        BSP_OLED_Refresh();
        return;
    }

    /* 逐页比较当前帧和影子帧，找出真正变化了的页区间。
       ★ 为什么合并成一个区间而不是逐页刷：每页都要重新发
         "起始 + 地址 + 控制字 + 停止"，页数一多，事务开销比数据本身还大。
         中间夹着的没变页跟着刷一遍，代价只有几十微秒，很划算。 */
    for (p = 0u; p < OLED_PAGE_NUM; p++) {
        uint32_t base = (uint32_t)p * OLED_WIDTH;
        uint8_t  changed = 0u;

        for (i = 0u; i < OLED_WIDTH; i++) {
            if (s_buf[base + i] != s_shown[base + i]) {
                changed = 1u;
                break;      /* 一页里只要有一个字节变了就算这一页变了 */
            }
        }

        if (changed != 0u) {
            if (first == 0xFFu) {
                first = p;
            }
            last = p;
        }
    }

    if (first == 0xFFu) {
        return;     /* 什么都没变，一次 I2C 都不发 */
    }

    BSP_OLED_RefreshPages(first, last);
}

void BSP_OLED_Refresh(void)
{
    /* 两条命令框定一个"列 0..127、页 0..7"的矩形窗口：
         0x21 设置列地址范围，后跟 起始列 和 结束列
         0x22 设置页地址范围，后跟 起始页 和 结束页
       窗口定好之后，连续写入的数据会自动按"列递增 → 列满换页"的顺序
       填满整个窗口，不需要软件干预。

       ★ 所以 1024 字节可以一次性推完，中间不用逐页重设地址 ——
       这是"水平寻址模式"(初始化时的 0x20,0x00)带来的便利。
       如果用默认的页寻址模式，就得写一页、重设一次地址、再写一页，
       I2C 事务数会翻 8 倍。

       ★ 具体发送逻辑统一放在 BSP_OLED_RefreshPages() 里，
         整屏刷新就是"刷第 0..7 页"，避免两份几乎一样的代码各自演化。 */
    BSP_OLED_RefreshPages(0u, (uint8_t)(OLED_PAGE_NUM - 1u));

    /* 整屏发完，影子帧已经是权威的，撤掉"强制整屏"的旗子 */
    s_force_full = 0u;
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

void BSP_OLED_DrawBox(uint8_t x, uint8_t page, uint8_t width,
                      uint8_t height_pages, uint8_t filled)
{
    uint8_t  p;
    uint8_t  i;
    uint16_t base;

    if ((page >= OLED_PAGE_NUM) || (height_pages == 0u) || (width < 2u)) {
        return;
    }
    if ((uint16_t)page + height_pages > OLED_PAGE_NUM) {
        height_pages = (uint8_t)(OLED_PAGE_NUM - page);
    }
    if (((uint16_t)x + width) > OLED_WIDTH) {
        width = (uint8_t)(OLED_WIDTH - x);
    }

    for (i = 0u; i < width; i++) {
        uint16_t px = (uint16_t)x + i;
        uint8_t  edge_col = ((i == 0u) || (i == (uint8_t)(width - 1u))) ? 1u : 0u;

        for (p = 0u; p < height_pages; p++) {
            uint8_t edge_row = ((p == 0u) || (p == (uint8_t)(height_pages - 1u))) ? 1u : 0u;
            uint8_t bits;

            if (filled != 0u) {
                /* 实心：整列都点亮 */
                bits = 0xFFu;
            } else if ((edge_col != 0u) || (edge_row != 0u)) {
                /* 镂空：只点亮边框。竖边点 0x18，横边点 0xFF */
                bits = (edge_col != 0u) ? 0x18u : 0xFFu;
            } else {
                continue;   /* 内部不画 */
            }

            base = (uint16_t)(page + p) * OLED_WIDTH;
            s_buf[base + px] |= bits;
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

    /* SSD1306 的上电初始化序列。这些值是模块厂家的标准配方，
       不需要理解每一条，但有三条值得记住（出事时先怀疑它们）：
         0x8D,0x14  电荷泵使能 —— ★ 3.3V 供电时必须打开。
                     OLED 面板需要 7~9V 驱动电压，靠片内电荷泵升压。
                     忘了这条就是"通信正常但屏幕全黑"的头号原因。
         0xC8, 0xA1  扫描方向 —— 决定屏幕的上下和左右。如果显示内容
                     是镜像或倒置的，就是这两条配错了（或者屏幕装反了）。
         0xA8,0x3F  多路复用比 63 —— 表示"一共 64 行"。设错了会出现
                     半屏显示、隔行显示这类诡异现象。 */
    static const uint8_t init_seq[] = {
        0xAEu,              /* 关显示（改配置期间先别看，避免看到花屏） */
        0x20u, 0x00u,       /* 水平寻址模式，让整屏可以一次流式写完 */
        0xB0u,              /* 页起始地址 0 */
        0xC8u,              /* COM 扫描方向：上下翻转（屏幕正装） */
        0x00u,              /* 低列地址 */
        0x10u,              /* 高列地址 */
        0x40u,              /* 起始行 0 */
        0x81u, 0x7Fu,       /* 对比度 0x7F，最大亮度的一半左右 */
        0xA1u,              /* 段重映射：左右翻转（屏幕正装） */
        0xA6u,              /* 正常显示（0xA7 是反色） */
        0xA8u, 0x3Fu,       /* 多路复用比 63 = 64 行 */
        0xA4u,              /* 显示内容跟随显存（0xA5 是全亮测试模式） */
        0xD3u, 0x00u,       /* 显示垂直偏移 0 */
        0xD5u, 0x80u,       /* 时钟分频 / 振荡频率 */
        0xD9u, 0xF1u,       /* 预充电周期 */
        0xDAu, 0x12u,       /* COM 引脚硬件配置（对应 128x64 的接法） */
        0xDBu, 0x40u,       /* VCOMH 反压电平 */
        0x8Du, 0x14u,       /* ★ 电荷泵使能，3.3V 供电必须打开 */
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
