/**
  ******************************************************************************
  * @file    app_menu.c
  * @brief   OLED 调参界面：状态页 + 主菜单 + 参数编辑 + 支撑脚 + 传感器检查
  *
  * 【在工程里的位置】
  *   人机交互的"总调度"。它对上只是一个 10ms 调一次的 APP_Menu_Update()，
  *   对下同时用到显示、按键、参数、车控、支撑脚五块：
  *
  *     BSP_Key_GetEvent() ─►┌────────────────┐─► BSP_OLED_Clear/DrawString/Printf
  *     （读走即清）          │    app_menu    │     └─► BSP_OLED_RefreshDirty()
  *                          │  按键 → 状态机  │
  *                          │       → 重画    │─► APP_Param_SetByIndex/Save/Reset
  *                          └────────────────┘─► APP_Car_Start/Stop/Brake/GetStatus
  *                                             └─► APP_Leg_Up/Down/SetTarget
  *
  *   ★ 改参数是"下一拍就生效"的：app_car / app_leg 每个控制周期都重新
  *     读一遍 g_param，所以本模块改完不需要通知任何人，也没有"应用"按钮。
  *
  * 【状态机】
  *
  *                    ┌──────────────────────────────┐
  *                    │          UI_STATUS           │◄─────────┐
  *                    │  状态页（默认页，8 页信息）    │          │ BACK
  *                    └───┬──────────────────────┬───┘          │
  *            停车时 OK    │                      │ 行驶时 OK     │
  *                         ▼                      ▼              │
  *                 ┌───────────────┐      ┌───────────────┐      │
  *                 │    UI_MENU    │      │ UI_LIVE_TUNE  │──────┘
  *                 │  主菜单 7 项   │      │ 行驶中调参 4 行│（★ 不停车）
  *                 └──┬──┬──┬──┬───┘      └───────────────┘
  *        TUNE PARAMS │  │  │  └──────────────► UI_INFO ──►(BACK)─► UI_MENU
  *                    │  │  └───► UI_LEG ─────►(BACK)─► UI_MENU
  *                    │  └──────► UI_SENSOR ──►(BACK)─► UI_STATUS
  *                    ▼
  *            ┌────────────────┐   OK   ┌────────────────┐
  *            │ UI_PARAM_LIST  │───────►│ UI_PARAM_EDIT  │
  *            │ 一屏 6 行可滚动 │◄───────│ 大字号 + 步长编辑│
  *            └───────┬────────┘ OK/BACK└────────────────┘
  *                    │ BACK
  *                    ▼  UI_MENU
  *
  * 【刷新策略：为什么不能每 10ms 刷一次】
  *   整块显存 1024 字节走软件 I2C 要 40~50ms。每 10ms 刷一次 = 主循环被吃掉
  *   一半，编码器采样和 PID 节拍全乱，车会画龙。所以：
  *     · 车在跑：MENU_RUN_REFRESH_MS（200ms）刷一次，够看清数值
  *     · 车停着：MENU_IDLE_REFRESH_MS（120ms）刷一次，调参时数字要跟手
  *     · 按了键并且画面内容变了：不节流，立刻重画 + 刷新（手感优先）
  *   重画 = BSP_OLED_Clear() + 按当前页画一遍 + BSP_OLED_RefreshDirty()。
  *   ★ "清屏 + 整帧重画"为什么仍然省时间：见下面的说明。
  *
  * 【★ 为什么"每帧清屏重画"还能省下刷屏时间】
  *   本文件按接口约定用 BSP_OLED_Clear() 起手重画。这一点也不浪费：
  *   bsp_oled 内部保存着一份影子帧 —— 也就是「屏幕上正显示着的那一帧」，
  *   BSP_OLED_RefreshDirty() 会拿当前帧和影子帧逐页比对，只把真正有差异的页
  *   发出去。所以「清屏 + 整帧重画」这种最省心的写法不会多刷一个字节：
  *   状态页里标题、探头条、按键提示这些内容根本没变，比对结果就是「不用发」。
  *
  *   ★ 不要改成「不清屏、只重画变化的字段」：新值比旧值短的时候
  *     （比如 -12.3 变成 0.0）会留下上一帧的残影，为了对齐还得给每个字段
  *     补空格，反而更脆。把「哪几页变了」交给影子帧去算，代码更简单，结论还更准。
  *
  * 【字符串一律 ASCII】
  *   屏幕字库只有 6x8 ASCII（0x20~0x7E），非 ASCII 会被画成空格；而且 armcc
  *   按主机区域设置解释源文件，中文字节对里可能含 0x22 / 0x5C，会把字符串的
  *   引号"吃掉"，报出莫名其妙的 missing closing quote。
  *   所以：中文只出现在注释里，字符串字面量里一个中文都不许有。
  ******************************************************************************
  */
#include "app_menu.h"

#include "app_param.h"
#include "app_car.h"
#include "app_leg.h"

#include "bsp_key.h"
#include "bsp_oled.h"
#include "bsp_track.h"
#include "bsp_buzzer.h"

/* ==========================================================================
 * 1. 常量与版面约定
 * --------------------------------------------------------------------------
 *   屏幕 128x64，6x8 字体一行 21 个字符（128/6），共 8 页（page 0..7）；
 *   12x16 放大字体占两页，起始页只能是 0..6。
 *   本模块所有坐标都写死在这些约定之内，画图函数自己也带边界保护。
 * ========================================================================== */
#define MENU_ROWS           6u      /* 主菜单 / 参数列表一屏能显示几行（page1..6） */
#define MENU_LIVE_ROWS      4u      /* 行驶中调参页一屏几行（page1..4） */
#define MENU_LIVE_MAX       5u      /* 行驶中调参最多放几个参数 */
#define MENU_VAL_X          66u     /* 列表里"值"的列（名字最长 8 字符 = 60 像素）*/

/* 临时提示信息的显示时长（毫秒）。用非阻塞的"到期自动消失"，
   绝不用 HAL_Delay 等它 —— 那会阻塞 10ms 节拍 */
#define MENU_MSG_MS         1300u
#define MENU_MSG_LONG_MS    1800u

/* 主菜单的动作编号。菜单项 = 名字表 + 这个 switch 分发 */
typedef enum {
    MI_TOGGLE_RUN = 0,      /* START CAR / STOP CAR */
    MI_TUNE_PARAM,          /* → UI_PARAM_LIST */
    MI_LEG,                 /* → UI_LEG */
    MI_SENSOR,              /* → UI_SENSOR */
    MI_SAVE,                /* → APP_Param_Save() */
    MI_LOAD_DEFAULT,        /* → APP_Param_ResetDefault() */
    MI_INFO,                /* → UI_INFO */
    MI_NUM                  /* 项数，必须放最后 */
} APP_MenuAction_t;

/* 界面页码 */
typedef enum {
    UI_STATUS = 0,          /* 状态页（默认） */
    UI_MENU,                /* 主菜单 */
    UI_PARAM_LIST,          /* 参数列表 */
    UI_PARAM_EDIT,          /* 参数编辑 */
    UI_LEG,                 /* 支撑脚 */
    UI_SENSOR,              /* 传感器检查 */
    UI_INFO,                /* 关于 */
    UI_LIVE_TUNE            /* 行驶中实时调参 */
} APP_MenuPage_t;

/* 按键的"角色"编号。bsp_key 的 KEY_OK/KEY_BACK/... 是硬件索引，
   这里再包一层是为了让下面的状态机读起来是"确认/返回/加/减" */
typedef enum {
    MKEY_NONE = 0,
    MKEY_OK,
    MKEY_BACK,
    MKEY_PLUS,
    MKEY_MINUS
} APP_MenuKey_t;

/* 主菜单项名字（ASCII，<= 12 字符）。
   ★ 第 0 项在运行时会被替换成 "STOP CAR"，见 menu_draw_menu() */
static const char *const s_menu_name[MI_NUM] = {
    "START CAR",
    "TUNE PARAMS",
    "SUPPORT LEG",
    "SENSOR VIEW",
    "SAVE PARAMS",
    "LOAD DEFAULT",
    "INFO"
};

/* 行驶中调参页关心的参数名。按名字查索引，查不到就跳过 ——
   这样以后参数表改名/删参数，这里不会编译报错，最多少显示一行 */
static const char *const s_live_name[MENU_LIVE_MAX] = {
    "KP", "KD", "BASESPD", "TURNMAX", "DEADZONE"
};

/* ==========================================================================
 * 2. 全部状态（都是本文件私有的 static）
 * ========================================================================== */
static uint8_t  s_page      = (uint8_t)UI_STATUS;   /* 当前页面 */

static uint8_t  s_menu_sel  = 0u;                   /* 主菜单光标 */
static uint8_t  s_menu_top  = 0u;                   /* 主菜单滚动窗口的首项 */

static uint8_t  s_param_sel = 0u;                   /* 参数列表光标（同时也是参数索引） */
static uint8_t  s_param_top = 0u;                   /* 参数列表滚动窗口的首项 */

static uint8_t  s_edit_idx  = 0u;                   /* 正在编辑的参数索引 */
static float    s_edit_orig = 0.0f;                 /* 进编辑页时的原值，BACK 用它还原 */

static uint8_t  s_step_mult = 1u;                   /* 步长倍率：短按 1，长按/连发 5 */

static uint8_t  s_leg_sel   = 0u;                   /* 支撑脚页光标 0=DOWN 1=UP 2=BACK */

static uint8_t  s_live_sel  = 0u;                   /* 行驶中调参页：选中的是第几个参数 */
static uint8_t  s_live_idx[MENU_LIVE_MAX];          /* 参数表里的真实索引 */
static uint8_t  s_live_cnt  = 0u;                   /* 实际找到几个 */

static uint8_t  s_force_draw = 1u;                  /* 1 = 下一次 Update 强制整屏重画 */
static uint32_t s_last_draw_ms = 0u;                /* 上次刷屏的时刻（毫秒） */

static char     s_msg[22];                          /* 临时提示（SAVED OK 之类），空串 = 无 */
static uint32_t s_msg_until = 0u;                   /* 提示的到期时刻 */

/* ==========================================================================
 * 3. 内部小工具
 * ========================================================================== */

/** 设置一条临时提示。到点由 APP_Menu_Update() 自己清掉，不用任何人等它 */
static void menu_set_msg(const char *text, uint16_t hold_ms)
{
    uint8_t i;

    for (i = 0u; (i < (uint8_t)(sizeof(s_msg) - 1u)) && (text[i] != '\0'); i++) {
        s_msg[i] = text[i];
    }
    s_msg[i] = '\0';

    s_msg_until = HAL_GetTick() + (uint32_t)hold_ms;
}

/** 长按和连发都算"快速调整"。连发每 80ms 来一次，天然就是连续调整的节奏 */
static uint8_t menu_is_fast_evt(uint8_t evt)
{
    return ((evt == (uint8_t)KEY_EVENT_LONG) || (evt == (uint8_t)KEY_EVENT_REPEAT)) ? 1u : 0u;
}

/**
  * @brief  取一次按键事件
  * @param  key 输出：哪个键（APP_MenuKey_t）
  * @param  evt 输出：什么事件（BSP_KeyEvent_t）
  * @retval 1 = 拿到了一个事件；0 = 这一拍没有按键
  * @note   ★ BSP_Key_GetEvent() 是"读走即清"的，所以每拍都要来读一次，
  *         读到的就是这一拍要处理的动作。优先级 OK > BACK > + > -，
  *         一次只处理一个：多键同按属于误操作，先把最重要的那个执行掉。
  */
static uint8_t menu_poll_key(uint8_t *key, uint8_t *evt)
{
    BSP_KeyEvent_t e;

    e = BSP_Key_GetEvent(KEY_OK);
    if (e != KEY_EVENT_NONE) {
        *key = (uint8_t)MKEY_OK;
        *evt = (uint8_t)e;
        return 1u;
    }

    e = BSP_Key_GetEvent(KEY_BACK);
    if (e != KEY_EVENT_NONE) {
        *key = (uint8_t)MKEY_BACK;
        *evt = (uint8_t)e;
        return 1u;
    }

    e = BSP_Key_GetEvent(KEY_PLUS);
    if (e != KEY_EVENT_NONE) {
        *key = (uint8_t)MKEY_PLUS;
        *evt = (uint8_t)e;
        return 1u;
    }

    e = BSP_Key_GetEvent(KEY_MINUS);
    if (e != KEY_EVENT_NONE) {
        *key = (uint8_t)MKEY_MINUS;
        *evt = (uint8_t)e;
        return 1u;
    }

    return 0u;
}

/** 参数类型名，编辑页显示用 */
static const char *menu_type_name(APP_ParamType_t type)
{
    switch (type) {
    case PTYPE_F32: return "F32";
    case PTYPE_I16: return "I16";
    case PTYPE_U16: return "U16";
    case PTYPE_U8:  return "U8";
    default:        return "?";
    }
}

/**
  * @brief  把数值按"这个参数该有的精度"打印到 (x, page)
  * @note   ★ 用一张常量格式串表，而不是在运行期拼 "%.2f" 这种格式串：
  *         拼串要引 snprintf，而这里只是按 decimals 选一个常量，简单又零依赖。
  *         整型参数统一四舍五入成整数 —— 直接截断会让 1299.9 显示成 1299，
  *         用户看着像丢了一位。
  */
static void menu_print_value(uint8_t x, uint8_t page, const APP_ParamItem_t *it, float v)
{
    if (it->type == PTYPE_F32) {
        switch ((it->decimals > 4u) ? 4u : it->decimals) {
        case 0u:  BSP_OLED_Printf(x, page, "%.0f", (double)v); break;
        case 1u:  BSP_OLED_Printf(x, page, "%.1f", (double)v); break;
        case 2u:  BSP_OLED_Printf(x, page, "%.2f", (double)v); break;
        case 3u:  BSP_OLED_Printf(x, page, "%.3f", (double)v); break;
        default:  BSP_OLED_Printf(x, page, "%.4f", (double)v); break;
        }
    } else {
        BSP_OLED_Printf(x, page, "%d", (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)));
    }
}

/** menu_print_value() 的放大字体版本。★ page 只能给 0..6（占两页） */
static void menu_print_value_big(uint8_t x, uint8_t page, const APP_ParamItem_t *it, float v)
{
    if (it->type == PTYPE_F32) {
        switch ((it->decimals > 4u) ? 4u : it->decimals) {
        case 0u:  BSP_OLED_PrintfBig(x, page, "%.0f", (double)v); break;
        case 1u:  BSP_OLED_PrintfBig(x, page, "%.1f", (double)v); break;
        case 2u:  BSP_OLED_PrintfBig(x, page, "%.2f", (double)v); break;
        case 3u:  BSP_OLED_PrintfBig(x, page, "%.3f", (double)v); break;
        default:  BSP_OLED_PrintfBig(x, page, "%.4f", (double)v); break;
        }
    } else {
        BSP_OLED_PrintfBig(x, page, "%d", (int)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f)));
    }
}

/**
  * @brief  把 8 路掩码画成一排方框：压线=实心，没压=镂空
  * @param  x0        第一个方框的左边
  * @param  page      起始页
  * @param  box_w     每个方框宽度（>= 2，bsp_oled 要求）
  * @param  gap       方框间距
  * @param  h_pages   方框高度（页）
  * @param  mask      bit0..bit7 对应探头 0..7
  */
static void menu_draw_mask_boxes(uint8_t x0, uint8_t page, uint8_t box_w,
                                 uint8_t gap, uint8_t h_pages, uint8_t mask)
{
    uint8_t i;
    uint8_t x = x0;

    for (i = 0u; i < TRACK_SENSOR_NUM; i++) {
        uint8_t filled = (uint8_t)((mask >> i) & 0x01u);

        /* bsp_oled 自己会夹宽度，但这里先挡一道：
           调用者算错 x0 时宁可少画一个，也不要画出半个框 */
        if (((uint16_t)x + box_w) <= OLED_WIDTH) {
            BSP_OLED_DrawBox(x, page, box_w, h_pages, filled);
        }

        x = (uint8_t)(x + box_w + gap);
    }
}

/**
  * @brief  误差指示条：中间是 0，左端 -1，右端 +1
  * @note   ★ 为什么不用 BSP_OLED_DrawBar：DrawBar 是"从左往右按百分比填充"的
  *         进度条语义，而误差是有符号的居中量。必须一眼看出"偏左还是偏右"，
  *         所以自己画：一条基准线 + 中间刻度 + 跟着误差跑的实心指示块。
  */
static void menu_draw_err_bar(uint8_t page, float err)
{
    uint8_t x;

    if (err < -1.0f) {
        err = -1.0f;
    }
    if (err > 1.0f) {
        err = 1.0f;
    }

    /* 基准线 + 中间(0 点)刻度 */
    BSP_OLED_DrawHLine(0u, page, (uint8_t)OLED_WIDTH);
    BSP_OLED_DrawBox((uint8_t)((OLED_WIDTH / 2u) - 1u), page, 2u, 1u, 1u);

    /* [-1, +1] 线性映射到 [0, 124]，右边留 4 像素给指示块自己 */
    x = (uint8_t)((err + 1.0f) * 0.5f * (float)(OLED_WIDTH - 4u));
    BSP_OLED_DrawBox(x, page, 4u, 1u, 1u);
}

/** 主菜单滚动窗口：让光标始终留在屏幕内 */
static void menu_menu_fix_top(void)
{
    if (s_menu_sel < s_menu_top) {
        s_menu_top = s_menu_sel;
    }
    if ((uint16_t)s_menu_sel >= ((uint16_t)s_menu_top + MENU_ROWS)) {
        s_menu_top = (uint8_t)(s_menu_sel - (MENU_ROWS - 1u));
    }
    /* 窗口尾部别越过最后一项，否则底部会空出一行 */
    if (((uint16_t)s_menu_top + MENU_ROWS) > (uint16_t)MI_NUM) {
        s_menu_top = (uint8_t)(MI_NUM - MENU_ROWS);
    }
}

/** 参数列表滚动窗口，规则同上 */
static void menu_param_fix_top(void)
{
    if (s_param_sel < s_param_top) {
        s_param_top = s_param_sel;
    }
    if ((uint16_t)s_param_sel >= ((uint16_t)s_param_top + MENU_ROWS)) {
        s_param_top = (uint8_t)(s_param_sel - (MENU_ROWS - 1u));
    }
    if (((uint16_t)s_param_top + MENU_ROWS) > (uint16_t)g_param_count) {
        s_param_top = (g_param_count > MENU_ROWS) ? (uint8_t)(g_param_count - MENU_ROWS) : 0u;
    }
}

/** 按名字把"行驶中调参"要用的参数索引收集起来（找不到的跳过） */
static void menu_live_build(void)
{
    uint8_t i;

    s_live_cnt = 0u;

    for (i = 0u; i < MENU_LIVE_MAX; i++) {
        uint8_t idx = APP_Param_FindByName(s_live_name[i]);

        if (idx < g_param_count) {      /* 0xFF = 没找到，直接跳过 */
            s_live_idx[s_live_cnt] = idx;
            s_live_cnt = (uint8_t)(s_live_cnt + 1u);
        }
    }

    if (s_live_sel >= s_live_cnt) {
        s_live_sel = 0u;
    }
}

/**
  * @brief  改完参数后的联动
  * @note   ★ 编辑 LEGUP / LEGDN 时顺手把支撑脚的目标角度也改掉，
  *         这样用户一边调一边就能看到脚动起来，标定角度不用来回翻页。
  *         判断用 APP_Param_FindByName()（按名字）而不是写死索引，
  *         参数表顺序变了也不会失效。
  */
static void menu_sync_leg(uint8_t idx)
{
    float v;

    if ((APP_Param_FindByName("LEGUP") != idx) && (APP_Param_FindByName("LEGDN") != idx)) {
        return;
    }

    v = APP_Param_GetByIndex(idx);
    APP_Leg_SetTarget((uint8_t)((v >= 0.0f) ? (v + 0.5f) : 0.0f));
}

/** 进入行驶中调参页 */
static void menu_enter_live(void)
{
    menu_live_build();
    s_live_sel  = 0u;
    s_step_mult = 1u;
    s_page      = (uint8_t)UI_LIVE_TUNE;
}

/* ==========================================================================
 * 4. 页面：画
 * --------------------------------------------------------------------------
 *   约定：每个 menu_draw_xxx() 只负责"往显存里画"，不调用任何刷新函数。
 *         Clear 和 RefreshDirty 统一由 menu_draw() 负责，这样每一页的
 *         画法都保证是"从干净的一帧开始"，不会互相留残影。
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * UI_STATUS —— 状态页。信息最多的一页，8 页各有分工：
 *   page0 标题 + 车状态      page4 轮速与转速
 *   page1 误差指示条         page5 掩码 + 丢线计时 / 十字标志
 *   page2 误差/转向量/基础速度 page6 运行时间 + 里程
 *   page3 八路探头方框       page7 按键提示
 * -------------------------------------------------------------------------- */
static void menu_draw_status(void)
{
    const APP_CarStatus_t *st = APP_Car_GetStatus();
    char mask_str[TRACK_SENSOR_NUM + 1u];
    uint8_t mask = BSP_Track_GetMask();

    /* ★ 探头掩码直接现读，不用状态快照里的那一份：
       快照只在行驶时由 app_car 更新，停着车的时候它是上一次的旧值。
       现读一次就是几个 GPIO 读，几乎零开销，而且停车摆位时也能实时看到
       "哪几路压线"，标定传感器的时候非常有用 */
    BSP_Track_MaskToString(mask, mask_str);

    /* page0：左边标题、右边车状态。
       98 + 6*5 = 128，正好铺满右侧，不越界 */
    BSP_OLED_DrawString(0u, 0u, "LINE FOLLOWER");
    BSP_OLED_DrawString(98u, 0u, APP_Car_StateName(st->state));

    /* page1：误差指示条（-1 在左，0 在中，+1 在右） */
    menu_draw_err_bar(1u, st->err);

    /* page2：误差 / 转向量 / 当前基础速度。
       19 个字符，留了余量给负号和更大的数值 */
    BSP_OLED_Printf(0u, 2u, "E%+5.2f T%+5.0f B%4d",
                    (double)st->err, (double)st->turn, (int)st->base_now);

    /* page3：八路探头。左边标 L、右边标 R，一眼看出压线位置偏哪边 */
    BSP_OLED_DrawString(0u, 3u, "L");
    BSP_OLED_DrawString(120u, 3u, "R");
    menu_draw_mask_boxes(8u, 3u, 13u, 1u, 1u, mask);

    /* page4：两轮转速（rpm）和实际下发的占空比（L/R 各一组）。
       ★ 21 个字符正好铺满一行：rpm 用 %3.0f 是因为这里只能挤出这么多，
         转速超过三位数时会被截断 —— 显示够用，绝不越界写显存 */
    BSP_OLED_Printf(0u, 4u, "rpm %3.0f/%3.0f %4d/%4d",
                    (double)st->rpm_l, (double)st->rpm_r,
                    (int)st->spd_l, (int)st->spd_r);

    /* page5：压线掩码 + 丢线累计毫秒 + 十字标志 */
    BSP_OLED_Printf(0u, 5u, "M%s L%4u %s",
                    mask_str,
                    (unsigned int)st->lost_ms,
                    (st->crossing != 0u) ? "CROSS" : "-----");

    /* page6：运行时间（秒）和两轮里程计数 */
    BSP_OLED_Printf(0u, 6u, "T%5us %6ld/%6ld",
                    (unsigned int)(st->run_ms / 1000u),
                    (long)st->dist_l, (long)st->dist_r);

    /* page7：按键提示。★ 行驶中 BACK 是急停，必须写在屏幕上提醒 */
    if (APP_Car_IsRunning() != 0u) {
        /* ★ 提示一律用 K1~K4 这套编号 + 一行写完四个键的作用。
           原因：之前这里写的是 "OK/BACK/+/-"，而屏幕左边那 8 个探头方框
           和车上的丝印都用不到这几个词，用户要在脑子里做一次
           "OK 是哪个键"的翻译。直接用 K1~K4（和接线表、和按键上的标注一致）
           就不用翻译了。
           ★ 跑车时必须写 "HOLD"：短按 K2 现在不动作，不写清楚会以为按键坏了 */
        BSP_OLED_DrawString(0u, 7u, "K1:TUNE K2 HOLD:STOP");
    } else {
        BSP_OLED_DrawString(0u, 7u, "K1:MENU");
    }
}

/* --------------------------------------------------------------------------
 * UI_MENU —— 主菜单。7 项、一屏 6 行，所以要滚动
 * -------------------------------------------------------------------------- */
static void menu_draw_menu(void)
{
    uint8_t i;
    uint8_t running = APP_Car_IsRunning();

    /* page0：当前位置 + 未保存标记。
       ★ 星号放在标题行而不是每行，是为了不挤占本来就紧张的行宽 */
    BSP_OLED_Printf(0u, 0u, "MENU %u/%u %s",
                    (unsigned int)(s_menu_sel + 1u),
                    (unsigned int)MI_NUM,
                    (APP_Param_IsDirty() != 0u) ? "*" : " ");

    for (i = 0u; i < MENU_ROWS; i++) {
        uint8_t item = (uint8_t)(s_menu_top + i);
        const char *name;

        if (item >= (uint8_t)MI_NUM) {
            break;                      /* 菜单项不足一屏，剩下的留空 */
        }

        name = s_menu_name[item];

        /* 第一项是启动/停止二合一：车在跑就显示 STOP CAR。
           这样屏幕上永远只有一个"当前该按的那个动作" */
        if (item == (uint8_t)MI_TOGGLE_RUN) {
            name = (running != 0u) ? "STOP CAR" : "START CAR";
        }

        BSP_OLED_Printf(0u, (uint8_t)(1u + i), "%c %s",
                        (item == s_menu_sel) ? '>' : ' ', name);
    }

    BSP_OLED_DrawString(0u, 7u, "K1:OK K2:BK K3+K4-");
}

/* --------------------------------------------------------------------------
 * UI_PARAM_LIST —— 参数列表：遍历 g_param_table，一屏 6 行
 * -------------------------------------------------------------------------- */
static void menu_draw_param_list(void)
{
    uint8_t i;

    BSP_OLED_Printf(0u, 0u, "PARAMS %u/%u %s",
                    (unsigned int)(s_param_sel + 1u),
                    (unsigned int)g_param_count,
                    (APP_Param_IsDirty() != 0u) ? "*" : " ");

    for (i = 0u; i < MENU_ROWS; i++) {
        uint8_t idx = (uint8_t)(s_param_top + i);
        const APP_ParamItem_t *it;
        uint8_t page;

        if (idx >= g_param_count) {
            break;
        }

        it   = &g_param_table[idx];
        page = (uint8_t)(1u + i);

        /* 名字用 DrawString 单独画（左对齐），值统一定在 MENU_VAL_X 列，
           这样整列数值是齐的，扫一眼就能比较大小 */
        BSP_OLED_Printf(0u, page, "%c %s", (idx == s_param_sel) ? '>' : ' ', it->name);
        menu_print_value(MENU_VAL_X, page, it, APP_Param_GetByIndex(idx));
    }

    BSP_OLED_DrawString(0u, 7u, "K1:EDIT K2:BK K3+K4-");
}

/* --------------------------------------------------------------------------
 * UI_PARAM_EDIT —— 编辑单个参数
 *   page0    名字 + 是否已改动
 *   page1    分隔线
 *   page2-3  放大字体的当前值（12x16 占两页，所以只能从 page2 起画）
 *   page4    类型 / BACK 会放弃修改
 *   page5    上下限
 *   page6    步长 + 当前步长倍率
 *   page7    按键提示
 * -------------------------------------------------------------------------- */
static void menu_draw_param_edit(void)
{
    const APP_ParamItem_t *it = &g_param_table[s_edit_idx];
    float v = APP_Param_GetByIndex(s_edit_idx);
    uint8_t edited = (v != s_edit_orig) ? 1u : 0u;

    BSP_OLED_Printf(0u, 0u, "EDIT %s%s", it->name, (edited != 0u) ? " *" : "");
    BSP_OLED_DrawHLine(0u, 1u, (uint8_t)OLED_WIDTH);

    /* 当前值用放大字体：调参时人在一米外看，小字看不清 */
    menu_print_value_big(0u, 2u, it, v);

    BSP_OLED_DrawString(0u, 4u, "type");
    BSP_OLED_DrawString(30u, 4u, menu_type_name(it->type));
    BSP_OLED_DrawString(60u, 4u, "BACK=cancel");

    BSP_OLED_DrawString(0u, 5u, "min");
    menu_print_value(24u, 5u, it, it->vmin);
    BSP_OLED_DrawString(64u, 5u, "max");
    menu_print_value(88u, 5u, it, it->vmax);

    BSP_OLED_DrawString(0u, 6u, "step");
    menu_print_value(30u, 6u, it, it->step);
    /* 倍率指示灯：长按时变成 x5，用户就知道"再按住会走得更快" */
    BSP_OLED_DrawString(96u, 6u, (s_step_mult > 1u) ? "x5" : "x1");

    BSP_OLED_DrawString(0u, 7u, "K1:OK K2:BK K3+K4-");
}

/* --------------------------------------------------------------------------
 * UI_LEG —— 支撑脚手动控制
 * -------------------------------------------------------------------------- */
static void menu_draw_leg(void)
{
    static const char *const items[3] = { "LEG DOWN", "LEG UP", "BACK" };
    const char *state;
    uint8_t i;

    BSP_OLED_DrawString(0u, 0u, "SUPPORT LEG");

    BSP_OLED_Printf(0u, 1u, "angle  %3u", (unsigned int)APP_Leg_GetAngle());
    BSP_OLED_Printf(0u, 2u, "target %3u", (unsigned int)APP_Leg_GetTarget());

    /* 三个状态分开显示比一个"到位/没到位"更有信息量：
       MOVING 说明斜坡还在走，这时候反复按按键是没有意义的 */
    if (APP_Leg_IsMoving() != 0u) {
        state = "MOVING";
    } else if (APP_Leg_IsDown() != 0u) {
        state = "DOWN";
    } else {
        state = "UP";
    }
    BSP_OLED_Printf(0u, 3u, "state  %s", state);

    for (i = 0u; i < 3u; i++) {
        BSP_OLED_Printf(0u, (uint8_t)(4u + i), "%c %s",
                        (i == s_leg_sel) ? '>' : ' ', items[i]);
    }

    BSP_OLED_DrawString(0u, 7u, "K1:OK K2:BK K3+K4-");
}

/* --------------------------------------------------------------------------
 * UI_SENSOR —— 传感器检查：把车放到线上，看哪几路亮
 *   同时显示"压线掩码"和"原始电平掩码"：
 *     两者互补 ⇒ 极性配置对了（TRACK_ACTIVE_LOW）
 *     两者一样 ⇒ 极性配反了，或者探头输出根本不是开关量
 * -------------------------------------------------------------------------- */
static void menu_draw_sensor(void)
{
    uint8_t mask = BSP_Track_GetMask();
    uint8_t raw  = BSP_Track_GetRawMask();
    char mask_str[TRACK_SENSOR_NUM + 1u];
    char raw_str[TRACK_SENSOR_NUM + 1u];

    BSP_Track_MaskToString(mask, mask_str);
    BSP_Track_MaskToString(raw, raw_str);

    BSP_OLED_DrawString(0u, 0u, "SENSOR CHECK");

    /* 大号方框：13 像素宽 + 1 像素间距，8 个占 111 像素，
       从 x=6 起画，左右正好塞得下 L / R 两个字母 */
    BSP_OLED_DrawString(0u, 1u, "L");
    BSP_OLED_DrawString(120u, 1u, "R");
    menu_draw_mask_boxes(6u, 1u, 13u, 1u, 2u, mask);

    BSP_OLED_Printf(0u, 3u, "MASK %s", mask_str);
    BSP_OLED_Printf(0u, 4u, "RAW  %s", raw_str);

    /* 误差用原始值和归一化值各显示一份：
       前者看探头权重表对不对，后者看 PID 实际吃到的量 */
    BSP_OLED_Printf(0u, 5u, "err  %+6.2f", (double)BSP_Track_GetError());
    BSP_OLED_Printf(0u, 6u, "errN %+6.3f", (double)BSP_Track_GetErrorNorm());

    BSP_OLED_DrawString(0u, 7u, "K2:BACK");
}

/* --------------------------------------------------------------------------
 * UI_INFO —— 关于 / 版本
 * -------------------------------------------------------------------------- */
static void menu_draw_info(void)
{
    /* 21 个字符，正好铺满一行 —— 再长一个字就会被截断 */
    BSP_OLED_DrawString(0u, 0u, "HAIXING LINE FOLLOWER");
    BSP_OLED_DrawString(0u, 1u, "STM32F103C8T6");

    /* __DATE__ 由编译器填，永远是"这份固件是什么时候编的"，
       比手写版本号可靠 —— 手写的很容易忘了改 */
    BSP_OLED_DrawString(0u, 2u, "BUILD " __DATE__);

    BSP_OLED_Printf(0u, 3u, "PARAM v%u", (unsigned int)PARAM_VERSION);

#if PARAM_FLASH_ENABLE
    BSP_OLED_DrawString(0u, 4u, "FLASH SAVE  : ON");
#else
    BSP_OLED_DrawString(0u, 4u, "FLASH SAVE  : OFF");
#endif

    BSP_OLED_DrawString(0u, 5u, "SWD  PA13/PA14");
    BSP_OLED_DrawString(0u, 6u, "TICK 10ms OLED I2C");
    BSP_OLED_DrawString(0u, 7u, "K2:BACK");
}

/* --------------------------------------------------------------------------
 * UI_LIVE_TUNE —— 行驶中实时调参（本模块的亮点）
 *
 *   ★ 为什么按键要重新分配：
 *     正常页面里 +/- 是"上下移动光标"，但跑车时一只手要扶着车/随时刹车，
 *     让 +/- 直接调"当前选中的那个参数"最省事；换参数交给 OK 循环切换。
 *   ★ 为什么这一页的刷新最克制（MENU_RUN_REFRESH_MS）：
 *     刷屏时主循环会停住几十毫秒，跑车时刷太勤会让循迹环"看漏"路面。
 * -------------------------------------------------------------------------- */
static void menu_draw_live(void)
{
    const APP_CarStatus_t *st = APP_Car_GetStatus();
    uint8_t i;
    uint8_t top = 0u;

    BSP_OLED_Printf(0u, 0u, "LIVE TUNE %u/%u",
                    (unsigned int)((s_live_cnt == 0u) ? 0u : (s_live_sel + 1u)),
                    (unsigned int)s_live_cnt);

    if (s_live_cnt == 0u) {
        /* 参数表里一个都没找到（比如名字被改了）。
           这时候不能崩，画一行提示，页面其余部分照常显示遥测 */
        BSP_OLED_DrawString(0u, 2u, "no live params");
    } else {
        /* 滚动窗口：5 个参数、一屏 4 行，选到最后两个时窗口跟着往下走 */
        if (s_live_sel >= MENU_LIVE_ROWS) {
            top = (uint8_t)(s_live_sel - (MENU_LIVE_ROWS - 1u));
        }

        for (i = 0u; i < MENU_LIVE_ROWS; i++) {
            uint8_t k = (uint8_t)(top + i);
            uint8_t idx;
            const APP_ParamItem_t *it;
            uint8_t page;

            if (k >= s_live_cnt) {
                break;
            }

            idx  = s_live_idx[k];
            it   = &g_param_table[idx];
            page = (uint8_t)(1u + i);

            BSP_OLED_Printf(0u, page, "%c %s", (k == s_live_sel) ? '>' : ' ', it->name);
            menu_print_value(MENU_VAL_X, page, it, APP_Param_GetByIndex(idx));
        }
    }

    /* 边跑边看效果：误差和转向量必须和参数在同一屏，
       否则"改了 KP 车变得更抖了"这个因果关系要靠记忆去连 */
    BSP_OLED_Printf(0u, 5u, "E%+5.2f T%+5.0f",
                    (double)st->err, (double)st->turn);

    BSP_OLED_Printf(0u, 6u, "%s L%4d R%4d",
                    APP_Car_StateName(st->state), (int)st->spd_l, (int)st->spd_r);

    BSP_OLED_DrawString(0u, 7u, "K1:SEL K2:BK K3+K4-");
}

/* --------------------------------------------------------------------------
 * 统一画帧：清屏 → 按当前页画 → （有临时提示就盖在最后一行）
 * -------------------------------------------------------------------------- */
static void menu_draw(void)
{
    BSP_OLED_Clear();

    switch (s_page) {
    case (uint8_t)UI_MENU:       menu_draw_menu();       break;
    case (uint8_t)UI_PARAM_LIST: menu_draw_param_list(); break;
    case (uint8_t)UI_PARAM_EDIT: menu_draw_param_edit(); break;
    case (uint8_t)UI_LEG:        menu_draw_leg();        break;
    case (uint8_t)UI_SENSOR:     menu_draw_sensor();     break;
    case (uint8_t)UI_INFO:       menu_draw_info();       break;
    case (uint8_t)UI_LIVE_TUNE:  menu_draw_live();       break;
    case (uint8_t)UI_STATUS:
    default:
        /* 非法页码一律回状态页：界面宁可"跳回首页"，也不能停在空白页 */
        s_page = (uint8_t)UI_STATUS;
        menu_draw_status();
        break;
    }

    /* 临时提示画在最后一行，盖住那一行的按键提示。
       ★ DrawString 是"整字节覆盖写"而不是"或"运算，所以盖上去是干净的，
         不会和底下的字糊在一起 */
    if (s_msg[0] != '\0') {
        BSP_OLED_DrawString(0u, 7u, s_msg);
    }
}

/* ==========================================================================
 * 5. 按键处理 —— 每个页面一个处理函数
 * --------------------------------------------------------------------------
 *   约定：返回 1 = "这个键被受理了，而且画面内容可能变了"。
 *         返回 1 时上层会 ① 嘀一声 ② 立刻重画+刷新（不等节流）。
 *         返回 0 = 这个键在本页没有任何含义（不响、不刷，省时间）。
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * 状态页
 * -------------------------------------------------------------------------- */
static uint8_t menu_key_status(uint8_t key, uint8_t evt)
{
    (void)evt;      /* 状态页不区分短按/长按 */

    if (key == (uint8_t)MKEY_OK) {
        if (APP_Car_IsRunning() != 0u) {
            menu_enter_live();          /* 行驶中 → 实时调参页 */
        } else {
            s_page     = (uint8_t)UI_MENU;
            s_menu_sel = 0u;
            s_menu_top = 0u;
        }
        return 1u;
    }

    if (key == (uint8_t)MKEY_BACK) {
        /* ★★ 停车 = 必须【长按】按键2 ★★
         *
         * 为什么改成"只认长按"：跑车时手在车上、眼睛在车上看，
         * 短按太容易误触 —— 碰一下就急停，正跑得好的车莫名其妙刹住。
         * 长按（0.6 秒）是有意为之的动作，不会误触，也够快。
         *
         * ★ 这里判的是"事件"而不是"当前是否按住"：
         *   KEY_EVENT_LONG 在按住满 0.6 秒时产生一次，之后是连发，
         *   所以一次长按只会停一次，不会重复触发。
         *
         * 判断用 state != IDLE 而不是 IsRunning()：
         * 丢线报警(LOST)和堵转报警(STALL)状态下 IsRunning() 也是 0，
         * 但那两个状态下轮子可能还在转，用户长按一定是要它停。 */
        if (evt != (uint8_t)KEY_EVENT_LONG) {
            return 0u;      /* 短按不动作 —— 跑车时防误触 */
        }

        if (APP_Car_GetState() != (uint8_t)APP_CAR_IDLE) {
            APP_Car_Brake();
            menu_set_msg("STOPPED", MENU_MSG_MS);
            return 1u;
        }
        return 0u;      /* 已经停着了，长按也没事可做 */
    }

    return 0u;
}

/* --------------------------------------------------------------------------
 * 主菜单：名字表 + 动作编号，OK 时 switch 分发
 * -------------------------------------------------------------------------- */
static uint8_t menu_key_menu(uint8_t key, uint8_t evt)
{
    (void)evt;

    if ((key == (uint8_t)MKEY_PLUS) || (key == (uint8_t)MKEY_MINUS)) {
        /* 到头循环：菜单项少，循环比停在头尾更省手 */
        if (key == (uint8_t)MKEY_PLUS) {
            s_menu_sel = (uint8_t)((s_menu_sel + (uint8_t)MI_NUM - 1u) % (uint8_t)MI_NUM);
        } else {
            s_menu_sel = (uint8_t)((s_menu_sel + 1u) % (uint8_t)MI_NUM);
        }
        menu_menu_fix_top();
        return 1u;
    }

    if (key == (uint8_t)MKEY_BACK) {
        s_page = (uint8_t)UI_STATUS;
        return 1u;
    }

    if (key != (uint8_t)MKEY_OK) {
        return 0u;
    }

    switch ((APP_MenuAction_t)s_menu_sel) {
    case MI_TOGGLE_RUN:
        if (APP_Car_IsRunning() != 0u) {
            APP_Car_Stop();                     /* 菜单里"停"用滑行，不刹停 */
            menu_set_msg("CAR STOPPED", MENU_MSG_MS);
        } else {
            APP_Car_Start();                    /* 内部按 RAMPms 斜坡起步 */
            menu_set_msg("CAR STARTED", MENU_MSG_MS);
        }
        /* 启停之后立刻切回状态页：用户最想看的就是它跑起来的样子 */
        s_page = (uint8_t)UI_STATUS;
        break;

    case MI_TUNE_PARAM:
        s_page      = (uint8_t)UI_PARAM_LIST;
        s_param_sel = 0u;
        s_param_top = 0u;
        break;

    case MI_LEG:
        s_page    = (uint8_t)UI_LEG;
        s_leg_sel = 0u;
        break;

    case MI_SENSOR:
        s_page = (uint8_t)UI_SENSOR;
        break;

    case MI_SAVE:
        /* ★ 保存要擦写 Flash，全程约 30ms 且 CPU 会被阻塞，只能在停车时做。
           这不是保守，是硬约束：行驶中擦 Flash，取指停顿会把控制节拍打乱 */
        if (APP_Car_IsRunning() != 0u) {
            menu_set_msg("STOP CAR FIRST", MENU_MSG_LONG_MS);
        }
#if PARAM_FLASH_ENABLE
        else if (APP_Param_Save() != 0u) {
            menu_set_msg("SAVED OK", MENU_MSG_MS);
        } else {
            menu_set_msg("SAVE FAIL", MENU_MSG_MS);
        }
#else
        else {
            menu_set_msg("FLASH DISABLED", MENU_MSG_LONG_MS);
        }
#endif
        break;

    case MI_LOAD_DEFAULT:
        if (APP_Car_IsRunning() != 0u) {
            menu_set_msg("STOP CAR FIRST", MENU_MSG_LONG_MS);
        } else {
            APP_Param_ResetDefault();           /* 只改 RAM，不写 Flash */
            menu_set_msg("LOADED need SAVE", MENU_MSG_LONG_MS);
        }
        break;

    case MI_INFO:
        s_page = (uint8_t)UI_INFO;
        break;

    default:
        break;
    }

    return 1u;
}

/* --------------------------------------------------------------------------
 * 参数列表
 * -------------------------------------------------------------------------- */
static uint8_t menu_key_param_list(uint8_t key, uint8_t evt)
{
    (void)evt;

    if ((key == (uint8_t)MKEY_PLUS) || (key == (uint8_t)MKEY_MINUS)) {
        if (g_param_count == 0u) {
            return 0u;
        }
        if (key == (uint8_t)MKEY_PLUS) {
            s_param_sel = (uint8_t)((s_param_sel + g_param_count - 1u) % g_param_count);
        } else {
            s_param_sel = (uint8_t)((s_param_sel + 1u) % g_param_count);
        }
        menu_param_fix_top();
        return 1u;
    }

    if (key == (uint8_t)MKEY_BACK) {
        s_page = (uint8_t)UI_MENU;
        return 1u;
    }

    if (key == (uint8_t)MKEY_OK) {
        /* 进编辑页之前先把原值抄下来：BACK 时要靠它"放弃修改" */
        s_edit_idx  = s_param_sel;
        s_edit_orig = APP_Param_GetByIndex(s_edit_idx);
        s_step_mult = 1u;
        s_page      = (uint8_t)UI_PARAM_EDIT;
        return 1u;
    }

    return 0u;
}

/* --------------------------------------------------------------------------
 * 参数编辑：短按走 1 步，长按/连发走 5 步
 * -------------------------------------------------------------------------- */
static uint8_t menu_key_param_edit(uint8_t key, uint8_t evt)
{
    const APP_ParamItem_t *it = &g_param_table[s_edit_idx];
    float step;

    if ((key == (uint8_t)MKEY_PLUS) || (key == (uint8_t)MKEY_MINUS)) {
        /* 短按 = 精细调整；长按/连发 = 粗调。
           连发事件每 80ms 来一次，所以"按住不放"就是每 80ms 走 5 步 */
        s_step_mult = menu_is_fast_evt(evt) ? 5u : 1u;

        step = it->step * (float)s_step_mult;
        if (key == (uint8_t)MKEY_MINUS) {
            step = -step;
        }

        /* ★ 限幅交给 APP_Param_SetByIndex()（内部按参数表夹），
           本模块不需要知道任何参数的合法范围 —— 参数表是唯一权威 */
        APP_Param_SetByIndex(s_edit_idx, APP_Param_GetByIndex(s_edit_idx) + step);
        menu_sync_leg(s_edit_idx);
        return 1u;
    }

    if (key == (uint8_t)MKEY_OK) {
        s_page = (uint8_t)UI_PARAM_LIST;    /* 完成，保留改动 */
        return 1u;
    }

    if (key == (uint8_t)MKEY_BACK) {
        /* ★ 放弃本次修改：把进编辑页时抄下来的原值写回去。
           这样"按错了键"是有救的，用户敢放心大胆地试 */
        APP_Param_SetByIndex(s_edit_idx, s_edit_orig);
        menu_sync_leg(s_edit_idx);
        s_page = (uint8_t)UI_PARAM_LIST;
        return 1u;
    }

    return 0u;
}

/* --------------------------------------------------------------------------
 * 支撑脚
 * -------------------------------------------------------------------------- */
static uint8_t menu_key_leg(uint8_t key, uint8_t evt)
{
    (void)evt;

    if ((key == (uint8_t)MKEY_PLUS) || (key == (uint8_t)MKEY_MINUS)) {
        if (key == (uint8_t)MKEY_PLUS) {
            s_leg_sel = (uint8_t)((s_leg_sel + 2u) % 3u);
        } else {
            s_leg_sel = (uint8_t)((s_leg_sel + 1u) % 3u);
        }
        return 1u;
    }

    if (key == (uint8_t)MKEY_BACK) {
        s_page = (uint8_t)UI_MENU;
        return 1u;
    }

    if (key == (uint8_t)MKEY_OK) {
        if (s_leg_sel == 0u) {
            APP_Leg_Down();     /* 走斜坡放下，不会"哐"一下顶起来 */
        } else if (s_leg_sel == 1u) {
            APP_Leg_Up();
        } else {
            s_page = (uint8_t)UI_MENU;
        }
        return 1u;
    }

    return 0u;
}

/* --------------------------------------------------------------------------
 * 传感器检查 / 关于页：只有 BACK 有含义
 * -------------------------------------------------------------------------- */
static uint8_t menu_key_sensor(uint8_t key, uint8_t evt)
{
    (void)evt;

    if (key == (uint8_t)MKEY_BACK) {
        s_page = (uint8_t)UI_STATUS;
        return 1u;
    }
    return 0u;
}

static uint8_t menu_key_info(uint8_t key, uint8_t evt)
{
    (void)evt;

    if (key == (uint8_t)MKEY_BACK) {
        s_page = (uint8_t)UI_MENU;
        return 1u;
    }
    return 0u;
}

/* --------------------------------------------------------------------------
 * 行驶中实时调参
 *   + / -  ：调"当前选中的那个参数"（长按 5 倍步长）
 *   OK     ：循环切换选中的参数（长按/连发 = 快速切换）
 *   BACK   ：回状态页，★ 不停车
 * -------------------------------------------------------------------------- */
static uint8_t menu_key_live(uint8_t key, uint8_t evt)
{
    if (s_live_cnt == 0u) {
        /* 没有可调参数时，只留 BACK 这一条出路 */
        if (key == (uint8_t)MKEY_BACK) {
            s_page = (uint8_t)UI_STATUS;
            return 1u;
        }
        return 0u;
    }

    if ((key == (uint8_t)MKEY_PLUS) || (key == (uint8_t)MKEY_MINUS)) {
        uint8_t idx = s_live_idx[s_live_sel];
        const APP_ParamItem_t *it = &g_param_table[idx];
        float step;

        s_step_mult = menu_is_fast_evt(evt) ? 5u : 1u;

        step = it->step * (float)s_step_mult;
        if (key == (uint8_t)MKEY_MINUS) {
            step = -step;
        }

        APP_Param_SetByIndex(idx, APP_Param_GetByIndex(idx) + step);
        menu_sync_leg(idx);
        return 1u;
    }

    if (key == (uint8_t)MKEY_OK) {
        s_live_sel  = (uint8_t)((s_live_sel + 1u) % s_live_cnt);
        s_step_mult = 1u;
        return 1u;
    }

    if (key == (uint8_t)MKEY_BACK) {
        /* 短按 = 回状态页，不停车（这是本页最常用的操作，必须灵敏） */
        if (evt != (uint8_t)KEY_EVENT_LONG) {
            s_page = (uint8_t)UI_STATUS;
            return 1u;
        }

        /* ★ 长按 = 停车。和状态页保持同一套手势：
           不管当前停在哪个页面，"长按按键2"永远等于"让车停下来"，
           跑车的人不需要回忆"我现在这一页该怎么停" */
        if (APP_Car_GetState() != (uint8_t)APP_CAR_IDLE) {
            APP_Car_Brake();
            menu_set_msg("STOPPED", MENU_MSG_MS);
        }
        s_page = (uint8_t)UI_STATUS;
        return 1u;
    }

    return 0u;
}

/** 按键分发。返回 1 = 受理了（上层负责嘀一声 + 立刻重画） */
static uint8_t menu_dispatch(uint8_t key, uint8_t evt)
{
    switch (s_page) {
    case (uint8_t)UI_MENU:       return menu_key_menu(key, evt);
    case (uint8_t)UI_PARAM_LIST: return menu_key_param_list(key, evt);
    case (uint8_t)UI_PARAM_EDIT: return menu_key_param_edit(key, evt);
    case (uint8_t)UI_LEG:        return menu_key_leg(key, evt);
    case (uint8_t)UI_SENSOR:     return menu_key_sensor(key, evt);
    case (uint8_t)UI_INFO:       return menu_key_info(key, evt);
    case (uint8_t)UI_LIVE_TUNE:  return menu_key_live(key, evt);
    case (uint8_t)UI_STATUS:
    default:                     return menu_key_status(key, evt);
    }
}

/* ==========================================================================
 * 6. 对外接口
 * ========================================================================== */

void APP_Menu_Init(void)
{
    /* ★ 上电直接停在【主菜单】而不是状态页。
       理由：开机第一件事几乎总是"我要干点什么"（启动、调参、看探头），
       菜单就是这个入口。状态页退到菜单的下一级（在菜单里按 BACK 就到），
       真正做到"上电即可操作"，不用先摸一遍按键才知道能干嘛。 */
    s_page       = (uint8_t)UI_MENU;
    s_menu_sel   = 0u;
    s_menu_top   = 0u;
    s_param_sel  = 0u;
    s_param_top  = 0u;
    s_edit_idx   = 0u;
    s_edit_orig  = 0.0f;
    s_step_mult  = 1u;
    s_leg_sel    = 0u;
    s_live_sel   = 0u;
    s_live_cnt   = 0u;
    s_msg[0]     = '\0';
    s_msg_until  = 0u;

    menu_live_build();

    /* ★ 这里用整屏的 BSP_OLED_Refresh()：开机画面（bsp_test 的自检图案）
       可能还留在屏幕上，要一次擦干净。整屏刷新约 40ms，但它只在初始化时
       出现一次，不在 10ms 控制循环里，代价可以接受。
       之后就一律走 RefreshDirty()，再也不整屏刷了 */
    menu_draw();
    BSP_OLED_Refresh();

    s_last_draw_ms = HAL_GetTick();
    s_force_draw   = 0u;
}

void APP_Menu_Update(void)
{
    uint8_t  key = 0u;
    uint8_t  evt = (uint8_t)KEY_EVENT_NONE;
    uint8_t  changed = 0u;
    uint32_t now;
    uint32_t period;
    uint32_t age;

    /* ---- ① 处理按键（每拍都读，读走即清） ---- */
    if (menu_poll_key(&key, &evt) != 0u) {
        changed = menu_dispatch(key, evt);
        if (changed != 0u) {
            /* 每次按键被受理都"嘀"一声。★ 用 KeyFeedback()（非阻塞），
               绝不能用 BSP_Buzzer_Beep()：那是 HAL_Delay 阻塞，会吃掉好几拍 */
            BSP_Buzzer_KeyFeedback();

            /* ★ 跑车时"长按连发"不立刻刷屏。
               连发是每 80ms 一次，如果每次都立刻重画 + 刷屏，按住不放
               就等于每 80ms 往控制循环里塞十几毫秒的 I2C，车会一顿一顿的。
               放开手之后数值照样是准的 —— 下一次节流到点（200ms 内）就会重画，
               而且参数本身在按下的一瞬间就已经写进 g_param 了，
               控制律该受的影响一点没少。 */
            if ((evt == (uint8_t)KEY_EVENT_REPEAT) && (APP_Car_IsRunning() != 0u)) {
                changed = 0u;
            }
        }
    }

    /* ---- ② 临时提示到期检查 ----
       用 (int32_t)(now - until) >= 0 判断，HAL_GetTick() 49.7 天回绕也不会出错 */
    now = HAL_GetTick();
    if ((s_msg[0] != '\0') && ((int32_t)(now - s_msg_until) >= 0)) {
        s_msg[0] = '\0';
        changed  = 1u;              /* 提示消失也是画面变化，要重画一次 */
    }

    /* ---- ③ 节流判断 ----
       车在跑就慢一点刷（刷屏时主循环会停住几十毫秒，跑车时刷太勤会
       让循迹环"看漏"路面）；车停着就快一点刷，调参时数字要跟手。
       行驶中调参页统一用"跑车"的节奏，因为它本来就在跑车场景下用 */
    if (s_page == (uint8_t)UI_LIVE_TUNE) {
        period = (uint32_t)MENU_RUN_REFRESH_MS;
    } else {
        period = (APP_Car_IsRunning() != 0u) ? (uint32_t)MENU_RUN_REFRESH_MS
                                            : (uint32_t)MENU_IDLE_REFRESH_MS;
    }

    age = (uint32_t)(now - s_last_draw_ms);

    if ((changed != 0u) || (s_force_draw != 0u) || (age >= period)) {
        menu_draw();

        /* ★ 只用 RefreshDirty()，不用整屏的 Refresh() */
        BSP_OLED_RefreshDirty();

        s_last_draw_ms = now;
        s_force_draw   = 0u;
    }
}

void APP_Menu_ShowStatus(void)
{
    /* ★ 这里刻意"只置标志，不立刻画"：
       本函数是给串口命令调的，而串口命令跑在主循环里。
       立刻整屏重画会阻塞几十毫秒，正好卡在控制节拍上。
       置个标志，10ms 内的下一拍 APP_Menu_Update() 就会把状态页画出来，
       人眼完全看不出延迟 */
    s_page       = (uint8_t)UI_STATUS;
    s_step_mult  = 1u;
    s_force_draw = 1u;
}

void APP_Menu_Invalidate(void)
{
    /* 清空显存，并强制下一次整屏重画 —— 两条缺一不可：
       只清标志不清显存，残留图案还在；只清显存不置标志，
       得等到刷新周期到点才重画，中间会看到一段空白 */
    BSP_OLED_Clear();
    BSP_OLED_MarkAllDirty();

    s_force_draw = 1u;
}
