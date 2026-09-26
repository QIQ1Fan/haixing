/**
  ******************************************************************************
  * @file    bsp_track.c
  * @brief   六路循迹传感器实现
  ******************************************************************************
  */
#include "bsp_track.h"
#include "bsp_gpio.h"

/* 权重表，顺序与传感器索引 0..5 一致 */
static const float s_weight[TRACK_SENSOR_NUM] = {
    TRACK_WEIGHT_0, TRACK_WEIGHT_1, TRACK_WEIGHT_2,
    TRACK_WEIGHT_3, TRACK_WEIGHT_4, TRACK_WEIGHT_5,
};

/* 全部压线的掩码 */
#define TRACK_MASK_ALL      ((uint8_t)((1u << TRACK_SENSOR_NUM) - 1u))

/* 最近一次有效误差。丢线时沿用它，避免"丢线返回 0"被误判成"正好居中" */
static float s_last_error = 0.0f;

/* ------------------------------- 初始化 -------------------------------- */

void BSP_Track_Init(void)
{
    /* 引脚方向/上拉由 BSP_GPIO_Init() 统一配置，这里不做重复初始化 */
}

/* ------------------------------- 读取接口 ------------------------------ */

uint8_t BSP_Track_GetRawMask(void)
{
    return BSP_GPIO_ReadTrackRaw();
}

uint8_t BSP_Track_GetMask(void)
{
    uint8_t raw = BSP_GPIO_ReadTrackRaw();
    uint8_t mask = 0u;
    uint32_t i;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        uint8_t level = (uint8_t)((raw >> i) & 0x01u);
        /* 压线电平由 TRACK_ACTIVE_LOW 决定 */
#if TRACK_ACTIVE_LOW
        if (level == 0u) {
            mask |= (uint8_t)(1u << i);
        }
#else
        if (level != 0u) {
            mask |= (uint8_t)(1u << i);
        }
#endif
    }
    return mask;
}

uint8_t BSP_Track_IsActive(uint8_t idx)
{
    if (idx >= TRACK_SENSOR_NUM) {
        return 0u;
    }
    return (uint8_t)((BSP_Track_GetMask() >> idx) & 0x01u);
}

uint8_t BSP_Track_CountActive(void)
{
    uint8_t mask = BSP_Track_GetMask();
    uint8_t n = 0u;
    uint32_t i;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        if ((mask >> i) & 0x01u) {
            n++;
        }
    }
    return n;
}

/* ------------------------------- 误差解算 ------------------------------ */

float BSP_Track_GetError(void)
{
    uint8_t mask = BSP_Track_GetMask();
    float sum = 0.0f;
    uint8_t n = 0u;
    uint32_t i;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        if ((mask >> i) & 0x01u) {
            sum += s_weight[i];
            n++;
        }
    }

    /* 丢线时不返回 0（那会被误判成"正好居中"），
       返回上一次的有效误差，让车按原方向继续找线。
       首次上电若就丢线，则返回 0。 */
    if (n == 0u) {
        return s_last_error;
    }

    s_last_error = sum / (float)n;
    return s_last_error;
}

float BSP_Track_GetErrorNorm(void)
{
    return BSP_Track_GetError() / TRACK_ERROR_MAX;
}

uint8_t BSP_Track_IsAllBlack(void)
{
    return (BSP_Track_GetMask() == TRACK_MASK_ALL) ? 1u : 0u;
}

uint8_t BSP_Track_IsAllWhite(void)
{
    return (BSP_Track_GetMask() == 0u) ? 1u : 0u;
}

/* ------------------------------- 调试辅助 ------------------------------ */

void BSP_Track_MaskToString(uint8_t mask, char *buf)
{
    uint32_t i;

    if (buf == NULL) {
        return;
    }
    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        buf[i] = ((mask >> (TRACK_SENSOR_NUM - 1u - i)) & 0x01u) ? '1' : '0';
    }
    buf[TRACK_SENSOR_NUM] = '\0';
}
