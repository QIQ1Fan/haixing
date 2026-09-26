/**
  ******************************************************************************
  * @file    bsp_oled.h
  * @brief   SSD1306 OLED 显示驱动（128x64，软件 I2C，PB8=SCL / PB9=SDA）
  * @note    采用"显存 + 整屏刷新"模式：所有绘制先改内存里的 1KB 缓冲区，
  *          调用 BSP_OLED_Refresh() 才真正写屏。好处是不会看到撕裂的中间态。
  ******************************************************************************
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
  * @brief  初始化 OLED（引脚 + SSD1306 寄存器序列 + 清屏）
  * @note   上电后建议延时 100ms 再调用，等屏幕内部复位完成。
  */
void BSP_OLED_Init(void);

/** @brief 清空显存（不写屏） */
void BSP_OLED_Clear(void);

/** @brief 把显存整屏写入 OLED */
void BSP_OLED_Refresh(void);

/** @brief 清空显存并立即写屏 */
void BSP_OLED_ClearAndRefresh(void);

/**
  * @brief  画一个像素（只改显存）
  * @param  on 0 = 熄灭, 非 0 = 点亮
  */
void BSP_OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);

/**
  * @brief  在指定位置输出 6x8 字符串
  * @param  x    起始列 0..127
  * @param  page 起始页 0..7
  */
void BSP_OLED_DrawString(uint8_t x, uint8_t page, const char *str);

/**
  * @brief  输出 12x16 放大字符串（横向纵向各放大 2 倍，占用 2 个页）
  * @param  page 起始页 0..6
  */
void BSP_OLED_DrawStringBig(uint8_t x, uint8_t page, const char *str);

/**
  * @brief  带格式化的字符串输出，等价于在 (x, page) 处 printf
  * @note   单次输出不超过 64 字节
  */
void BSP_OLED_Printf(uint8_t x, uint8_t page, const char *fmt, ...);

/**
  * @brief  带格式化的放大字符串输出
  */
void BSP_OLED_PrintfBig(uint8_t x, uint8_t page, const char *fmt, ...);

/**
  * @brief  画一条水平细线（放在页的中间，视觉上像一条横线）
  */
void BSP_OLED_DrawHLine(uint8_t x, uint8_t page, uint8_t width);

/**
  * @brief  画进度条（用于显示速度 / 误差等实时量）
  * @param  percent 0..100，超出自动饱和
  */
void BSP_OLED_DrawBar(uint8_t x, uint8_t page, uint8_t width,
                      uint8_t height_pages, uint8_t percent);

/**
  * @brief  画一个占满全屏的测试图案（斜线 + 边框 + 文字），用于验证屏幕
  */
void BSP_OLED_TestPattern(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OLED_H */
