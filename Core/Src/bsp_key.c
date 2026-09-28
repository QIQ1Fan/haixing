/**
  ******************************************************************************
  * @file    bsp_key.c
  * @brief   四路独立按键实现
  *
  * 【数据流】
  *   引脚电平（抖动）→ 第一级：消抖 → 稳定电平
  *     → 第二级：计时 → 短按 / 长按 / 连发事件
  *       → s_key[i].event → BSP_Key_GetEvent() 读走 → 菜单处理
  *
  * 【每个按键的独立状态机】
  *
  *              消抖：连续 3 次读数一致才认可
  *   ┌──────────┐  raw 变化且稳定   ┌────────────┐
  *   │  松开态  │ ───────────────►  │  按下态    │
  *   │ stable=0 │ ◄───────────────  │  stable=1  │
  *   └──────────┘   raw 变化且稳定   └─────┬──────┘
  *          ▲                              │ hold++
  *          │                              ▼
  *          │                     ┌──────────────────┐
  *          │                     │ hold ≥ 100 ?     │
  *          │                     ├─ 否 ─► 继续累计  │
  *          │                     └─ 是 ─► 发 LONG    │
  *          │                              │ 之后每 10 次
  *          │                              ▼ 发一次 REPEAT
  *          │
  *          └──── 松开时若从未发过 LONG，则发一次 PRESS（短按）
  *
  * 【为什么用状态机而不是阻塞式等待】
  *   阻塞式写法（按下后 while 等待松手）会卡死整个程序，循迹车会直接失控。
  *   状态机每 10ms 推进一格，立即返回，不占用 CPU，可以和其它任务共存。
  ******************************************************************************
  */
#include "bsp_key.h"
#include "bsp_gpio.h"

/** 每个按键一份状态 */
typedef struct {
    uint8_t        stable;      /* 消抖后的状态：1 = 按下 */
    uint8_t        debounce;    /* 消抖计数器，记录"读数与 stable 连续不符"的次数 */
    uint16_t       hold;        /* 本次按住持续的扫描次数 */
    uint8_t        long_fired;  /* 本次按住期间是否已经发过长按事件 */
    uint16_t       repeat;      /* 长按之后的连发计数器 */
    BSP_KeyEvent_t event;       /* 待读取的事件 */
} key_state_t;

static key_state_t s_key[KEY_ID_NUM];

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void BSP_Key_Init(void)
{
    uint32_t i;

    /* 引脚方向与上拉由 BSP_GPIO_Init() 配置，这里只清状态。
       上电时全部按"松开"处理，避免开机瞬间误触发 */
    for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
        s_key[i].stable     = 0u;
        s_key[i].debounce   = 0u;
        s_key[i].hold       = 0u;
        s_key[i].long_fired = 0u;
        s_key[i].repeat     = 0u;
        s_key[i].event      = KEY_EVENT_NONE;
    }
}

/* ==========================================================================
 * 扫描状态机
 * ========================================================================== */

void BSP_Key_Scan(void)
{
    uint32_t i;

    /* ★ 必须每 KEY_SCAN_PERIOD_MS 调用一次。
       长按 1 秒、连发 100ms 这些时间阈值都是"扫描次数 × 周期"，
       周期不对阈值就不对。 */
    for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
        key_state_t *k = &s_key[i];
        uint8_t raw = BSP_GPIO_ReadKeyRaw((uint8_t)i);

        /* ---- 第一级：消抖 ----
           读数与当前稳定值不符时，不立刻改判，而是开始累计。
           连续 KEY_DEBOUNCE_CNT 次都不符，才认为真的变了。
           中途一旦读数又和稳定值一致，计数清零重来 ——
           这就是机械抖动的过滤方式。 */
        if (raw != k->stable) {
            k->debounce++;
            if (k->debounce >= (uint8_t)KEY_DEBOUNCE_CNT) {
                k->debounce = 0u;
                k->stable   = raw;      /* 新状态正式生效 */

                if (raw != 0u) {
                    /* 刚按下：开始计时，清掉上一次的遗留状态 */
                    k->hold       = 0u;
                    k->long_fired = 0u;
                    k->repeat     = 0u;
                } else {
                    /* 刚松开：如果这次按住期间没触发过长按，那它就是个短按。
                       长按已经发过事件了就不再补发短按，避免一次长按
                       被菜单同时当成"长按"和"短按"处理两遍 */
                    if (k->long_fired == 0u) {
                        k->event = KEY_EVENT_PRESS;
                    }
                    k->hold       = 0u;
                    k->long_fired = 0u;
                }
            }
        } else {
            /* 读数与稳定值一致，说明之前的抖动是假的，计数清零 */
            k->debounce = 0u;
        }

        /* ---- 第二级：长按与连发 ----
           只在按键处于按下态时才推进计时 */
        if (k->stable != 0u) {
            k->hold++;

            if (k->long_fired == 0u) {
                /* 还没发过长按：看是否到达阈值 */
                if (k->hold >= (uint16_t)KEY_LONG_PRESS_CNT) {
                    k->long_fired = 1u;
                    k->repeat     = 0u;
                    k->event      = KEY_EVENT_LONG;
                }
            } else {
                /* 已经长按过了：进入连发节奏，用来做"按住加减号快速调参" */
                k->repeat++;
                if (k->repeat >= (uint16_t)KEY_REPEAT_CNT) {
                    k->repeat = 0u;
                    k->event  = KEY_EVENT_REPEAT;
                }
            }
        }
    }
}

/* ==========================================================================
 * 读取接口
 * ========================================================================== */

BSP_KeyEvent_t BSP_Key_GetEvent(BSP_KeyId_t id)
{
    BSP_KeyEvent_t e;

    if ((uint32_t)id >= (uint32_t)KEY_ID_NUM) {
        return KEY_EVENT_NONE;
    }

    e = s_key[id].event;
    s_key[id].event = KEY_EVENT_NONE;   /* ★ 读走即清，保证一个事件只被处理一次 */
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

    /* 只清事件不清状态。用在"从菜单切走"的时候：
       否则切回去时会把切换过程中攒下的旧事件一次性触发出来 */
    for (i = 0u; i < (uint32_t)KEY_ID_NUM; i++) {
        s_key[i].event = KEY_EVENT_NONE;
    }
}
