/**
  ******************************************************************************
  * @file    bsp_key.c
  * @brief   四路独立按键实现
  *
  * 每个按键一个独立状态机：
  *   电平抖动 -> 连续 KEY_DEBOUNCE_CNT 次读到同电平方才认可（消抖）
  *   按下后计数 -> 满 KEY_LONG_PRESS_CNT 产生 LONG，之后每 KEY_REPEAT_CNT 产生 REPEAT
  *   松开时若从未触发过 LONG，则产生 PRESS（短按）
  ******************************************************************************
  */
#include "bsp_key.h"
#include "bsp_gpio.h"

typedef struct {
    uint8_t        stable;      /* 消抖后的状态：1 = 按下 */
    uint8_t        debounce;    /* 消抖计数器 */
    uint16_t       hold;        /* 按住持续的扫描次数 */
    uint8_t        long_fired;  /* 本次按住是否已触发长按 */
    uint16_t       repeat;      /* 连发计数器 */
    BSP_KeyEvent_t event;       /* 待读取的事件 */
} key_state_t;

static key_state_t s_key[KEY_ID_NUM];

/* ------------------------------- 初始化 -------------------------------- */

void BSP_Key_Init(void)
{
    uint32_t i;

    for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
        s_key[i].stable     = 0u;
        s_key[i].debounce   = 0u;
        s_key[i].hold       = 0u;
        s_key[i].long_fired = 0u;
        s_key[i].repeat     = 0u;
        s_key[i].event      = KEY_EVENT_NONE;
    }
}

/* ------------------------------- 扫描状态机 ---------------------------- */

void BSP_Key_Scan(void)
{
    uint32_t i;

    for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
        key_state_t *k = &s_key[i];
        uint8_t raw = BSP_GPIO_ReadKeyRaw((uint8_t)i);

        /* ---- 第一级：消抖 ---- */
        if (raw != k->stable) {
            k->debounce++;
            if (k->debounce >= (uint8_t)KEY_DEBOUNCE_CNT) {
                k->debounce = 0u;
                k->stable   = raw;

                if (raw != 0u) {
                    /* 刚按下：开始计时 */
                    k->hold       = 0u;
                    k->long_fired = 0u;
                    k->repeat     = 0u;
                } else {
                    /* 刚松开：若这次按住期间没触发过长按，就是一次短按 */
                    if (k->long_fired == 0u) {
                        k->event = KEY_EVENT_PRESS;
                    }
                    k->hold       = 0u;
                    k->long_fired = 0u;
                }
            }
        } else {
            k->debounce = 0u;
        }

        /* ---- 第二级：长按与连发 ---- */
        if (k->stable != 0u) {
            k->hold++;

            if (k->long_fired == 0u) {
                if (k->hold >= (uint16_t)KEY_LONG_PRESS_CNT) {
                    k->long_fired = 1u;
                    k->repeat     = 0u;
                    k->event      = KEY_EVENT_LONG;
                }
            } else {
                k->repeat++;
                if (k->repeat >= (uint16_t)KEY_REPEAT_CNT) {
                    k->repeat = 0u;
                    k->event  = KEY_EVENT_REPEAT;
                }
            }
        }
    }
}

/* ------------------------------- 读取接口 ------------------------------ */

BSP_KeyEvent_t BSP_Key_GetEvent(BSP_KeyId_t id)
{
    BSP_KeyEvent_t e;

    if ((uint32_t)id >= (uint32_t)KEY_ID_NUM) {
        return KEY_EVENT_NONE;
    }

    e = s_key[id].event;
    s_key[id].event = KEY_EVENT_NONE;   /* 读走即清 */
    return e;
}

uint8_t BSP_Key_IsPressed(BSP_KeyId_t id)
{
    if ((uint32_t)id >= (uint32_t)KEY_ID_NUM) {
        return 0u;
    }
    return s_key[id].stable;
}

void BSP_Key_Flush(void)
{
    uint32_t i;

    for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
        s_key[i].event = KEY_EVENT_NONE;
    }
}
