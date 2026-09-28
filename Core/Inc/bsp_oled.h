/**
  ******************************************************************************
  * @file    bsp_oled.h
  * @brief   SSD1306 OLED 显示（128x64，软件 I2C，PB8=SCL / PB9=SDA）
  *
  * 【在工程里的位置】 人机交互的输出端，和 bsp_key 一起构成调参菜单
  *
  *   按键事件 → 菜单状态机 → 绘制函数 → 1KB 显存 s_buf[]
  *     → BSP_OLED_Refresh() → 软件 I2C 送 1024 字节 → 屏幕
  *
  * 【上游】 main.c（开机画面）/ 以后的菜单系统 / 实时状态显示
  * 【下游】 SSD1306 屏幕模块
  *
  * 【占用的资源】 PB8(SCL) / PB9(SDA)，推挽输出模拟 I2C
  *
  * 【★ 为什么用"显存 + 整屏刷新"而不是画一点发一点】
  *   如果每画一个像素就发一次 I2C，屏幕上会看到逐点出现的撕裂过程，
  *   而且 I2C 事务次数暴增、极慢。先在内存里把一整幅图拼好，再一次性
  *   推给屏幕，观感上永远是完整的帧。代价是占 1KB RAM —— F103C8 有 20KB，
  *   完全够用。
  *
  * 【显存布局（和 SSD1306 原生格式一致）】
  *   s_buf[page * 128 + x]，字节的 bit0 是该页最上面一行：
  *
  *      页 0 ─┬─ bit0  ← y=0
  *            ├─ bit1  ← y=1
  *            │  ...
  *            └─ bit7  ← y=7
  *      页 1 ─── bit0  ← y=8
  *            ...
  *      页 7 ─── bit7  ← y=63
  *
  *   正因如此，整块 1024 字节可以原样丢给屏幕，不用做任何格式转换。
  *
  * 【坐标约定】 y 方向按 8 像素一页来定位（page 0..7），不是逐像素。
  *   6x8 字体正好占满一页；12x16 放大字体占两页。
  *   y 用页做单位是为了让绘制快、免移位，代价是不能任意像素行定位文字。
  * ============================================================================
  */
#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/** 页编号 0..7，每页 8 像素高，共 64 像素 */
#define OLED_PAGE_NUM       (OLED_HEIGHT / 8u)

/**
  * @brief  初始化 OLED
  * @note   内部含 100ms 上电等待，等 SSD1306 自己的复位电路完成。
  *         所以放在初始化序列最后调用，不拖累前面的模块。
  */
void BSP_OLED_Init(void);

/** @brief 清空显存（只改内存，不动屏幕） */
void BSP_OLED_Clear(void);

/** @brief 把显存整屏写入 OLED（这一步才真正耗时，约几毫秒） */
void BSP_OLED_Refresh(void);

/** @brief 清空显存并立即写屏 —— 每次重画一整帧的标准起手式 */
void BSP_OLED_ClearAndRefresh(void);

/**
  * @brief  画一个像素（只改显存，需 Refresh 才可见）
  * @param  on 0 = 熄灭, 非 0 = 点亮
  */
void BSP_OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);

/**
  * @brief  在指定位置输出 6x8 字符串
  * @param  x    起始列，0..127
  * @param  page 起始页，0..7
  * @note   一行能放 128/6 = 21 个字符。右边放不下会直接截断，不自动换行
  */
void BSP_OLED_DrawString(uint8_t x, uint8_t page, const char *str);

/**
  * @brief  输出 12x16 放大字符串（横竖各放大 2 倍）
  * @param  page 起始页，0..6（占两页，7 页起会越界）
  * @note   适合显示转速、误差这类需要一眼看清的主数值
  */
void BSP_OLED_DrawStringBig(uint8_t x, uint8_t page, const char *str);

/**
  * @brief  带格式化的字符串输出，相当于在 (x, page) 处 printf
  * @note   单次输出不超过 64 字节
  *   例： BSP_OLED_Printf(0, 3, "Err:%+.2f", err);
  */
void BSP_OLED_Printf(uint8_t x, uint8_t page, const char *fmt, ...);

/** @brief 带格式化的放大字符串输出（缓冲 32 字节） */
void BSP_OLED_PrintfBig(uint8_t x, uint8_t page, const char *fmt, ...);

/** @brief 画一条水平细线（画在页的中间一行，视觉上就是一条横线） */
void BSP_OLED_DrawHLine(uint8_t x, uint8_t page, uint8_t width);

/**
  * @brief  画进度条，用来直观显示速度 / 误差等实时量
  * @param  percent 0..100，超出自动饱和
  * @param  height_pages 高度，单位页（1 页 = 8 像素）
  */
void BSP_OLED_DrawBar(uint8_t x, uint8_t page, uint8_t width,
                      uint8_t height_pages, uint8_t percent);

/**
  * @brief  画一个占满全屏的自检图案
  * @note   含边框、斜线、两种字号、进度条。斜线用来检查有没有整行/整列错位；
  *         每页各点一个像素，用来检查有没有某根 COM 线虚焊（那一页不会亮）。
  *         屏幕点亮但显示异常时，先跑这个排除硬件问题。
  */
void BSP_OLED_TestPattern(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OLED_H */
