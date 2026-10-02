/**
  ******************************************************************************
  * @file    app_param.c
  * @brief   可调参数集中管理 + Flash 掉电保存
  *
  * 【数据流】
  *   上电： APP_Param_Init()
  *           ├─ APP_Param_Load()    读 Flash 最后一页
  *           │     ├─ magic 不对 / 版本不对 / 校验和不对 → 返回 0
  *           │     └─ 全对 → memcpy 到 g_param，返回 1
  *           └─ 返回 0 时 → APP_Param_ResetDefault() 用出厂值
  *                 └─ 最后 APP_Param_Clamp() 兜底限幅
  *
  *   调参： 菜单/串口 → APP_Param_SetByIndex() → 夹到合法范围 → 写进 g_param
  *           → 置脏标记 → （用户按"保存"时）APP_Param_Save() 写回 Flash
  *
  *   使用： app_car 每个控制周期直接读 g_param.xxx，所以改完下一拍就生效
  *
  * 【★ 为什么参数表要和结构体分开写两遍】
  *   一份是"数据"（结构体字段），一份是"元数据"（名字/步长/上下限）。
  *   C 语言没有反射，做不到"自动发现字段"，只能手写这张表。
  *   代价是加参数要改两处，收益是菜单、限幅、串口打印全部自动跟着走 ——
  *   这个交易很划算。表里的 offset 用 offsetof() 算，写错字段名会编译报错，
  *   所以不会出现"表里指到了别的字段"这种隐蔽 bug。
  *
  * 【★ 校验和为什么能防住乱码】
  *   Flash 擦除后的值是 0xFF，第一次上电时整页都是 0xFF：
  *     magic 读到 0xFFFFFFFF ≠ PARAM_MAGIC → 直接判无效 → 用默认值。
  *   如果只校验 magic 就写回去、不校验 checksum，那么 Flash 因为电压不稳
  *   写坏几个 bit 时，参数会变成一堆离谱的值（比如 base_speed = 30000），
  *   车一上电就窜出去。checksum 就是为了拦住这种情况。
  ******************************************************************************
  */
#include "app_param.h"
#include "bsp_uart.h"

#include <stddef.h>     /* offsetof */
#include <string.h>     /* memcpy / memcmp / memset */

/* ==========================================================================
 * 全局参数实例
 * ========================================================================== */
APP_Param_t g_param;

/* ==========================================================================
 * 出厂默认值
 * --------------------------------------------------------------------------
 *   这份数值是"能跑起来"的起点，不是最优值。整定顺序见 README：
 *     先只给 KP（KI=0, KD=0）把车调得不画龙，再加 KD 压住摆动，
 *     最后（真的有必要时）才加一点点 KI 消稳态偏移。
 * ========================================================================== */
#define DEF_TURN_KP         1200.0f     /* 误差 1.0（最边上）→ 转向量 1200 */
#define DEF_TURN_KI         0.0f        /* 循迹一般不需要积分 */
/* ★ KD 的单位是"每 (误差/秒)"，不是"每格误差"，所以它天生比 KP 小一两个数量级。
 *   采样周期只有 10ms：误差跳一格(0.14) → 变化率 14/秒 → D = KD × 14
 *     KD=180 → D≈2500（满量程才 3600）：D 长期顶满，P 被淹没，
 *               车会"一看见误差跳变就往反方向甩"
 *     KD=60  → D≈840：是"够用的阻尼"，不是"抢方向盘的乘客"
 *   再加上 PID_D_LIMIT_RATIO（D 项最多占输出一半）双保险。 */
#define DEF_TURN_KD         60.0f
#define DEF_DEAD_ZONE       0.08f       /* 归一化误差死区，约等于半路的宽度 */
/* ★ 大偏差自动减速系数（0~1.5）。
 *      实际基速 = BASESPD × (1 - SLOWK × |误差|)，且不低于 25%
 *   为什么大车必须有它：
 *     循迹是"看见了才纠"。车偏得越远说明纠得越晚；这时候若还保持全速，
 *     等转向环把车头掰回来，探头早就越过线了（8 路阵列就那么宽，
 *     前瞻距离很短）—— 现象就是你说的"还没转过来就过线、驶出路线"。
 *     减速等于把"看见 → 越过线"这段时间拉长，转向环才有机会干活。
 *   0.6 是温和起点；弯特别急可以加到 1.0（误差满格时只剩 40% 速度）。 */
/* ★ 转角模式：最外侧左右各几路只当"转角探测器"，不参与循迹误差。
 *   0 = 关闭，1 = 各 1 路（推荐起点）。
 *   为什么需要它：大车 + 窄阵列时，最外两路在直线上永远不亮，
 *   一亮就说明前方是转角。让它们参与误差计算，车会在逼近转角时
 *   提前得到一个很大的误差然后猛打方向 —— 这就是"边缘一读到就甩"的根。
 *   把它们摘出来当"转角雷达"用，直道更稳、转角交给状态机专门处理。 */
/* ★ 探头极性：默认取自 bsp_config.h 的宏（实车实测 = 压线读高电平） */
#define DEF_ACT_LOW         TRACK_ACTIVE_LOW

/* 转角急转时外侧履带的速度。大车要够劲儿，但又不能大到把车掀了 */
#define DEF_PIVOT_SPD       1600
/* 急转最长持续时间。超时就交回正常逻辑（丢线找线），避免原地转圈停不下来 */

#define DEF_SLOW_K          0.6f

/* ★ 循迹/直行的基础速度：1950（满量程 3599，约 54%）。
 *   注意它同时也是"路线脚本里 Route_Straight / Route_LineFollow 的速度"。
 *   车速越快，同样的误差留给转向的时间就越短 —— 如果发现过弯跟不上，
 *   先降这个值；SLOWK（大偏差自动减速）也会在弯道里自动把它压下来。 */
#define DEF_BASE_SPEED      1950
#define DEF_TURN_LIMIT      1500        /* 转向量上限：最狠时左右差 3000 */
/* ★ 内轮最低速：履带车/大车建议 0，也就是【允许内侧履带反转】。
 *   轮式小车怕内轮停转造成顿挫，所以给 250 的底；
 *   但履带车靠"搓地"转向，内轮反转才能急拐 ——
 *   给 250 等于把最有力的那部分转向能力锁掉，大车会转不过来。 */
#define DEF_INNER_MIN_SPD   0
#define DEF_RAMP_MS         400         /* 0 → 全速用 0.4 秒，起步不打滑 */
/* ★★★ 转向差速取反：默认 0 ★★★
 *
 * 【它管什么】
 *   把"转向差速的方向"整个对调：本该左轮慢右轮快的场合，变成左轮快右轮慢。
 *   用来纠正两类"装配上左右颠倒"的问题：
 *     · 两个电机在 TB6612 上的左右接反了（代码里的"左"其实装在车右侧）
 *     · 探头阵列装反了（索引 0 其实在车的右边）
 *
 * 【它和 MLINV / MRINV 的区别（千万别混）】
 *   MLINV/MRINV 管【单个电机转的方向】：错了的表现是"两个轮子都被命令前进，
 *     车却原地打转"，或者"两个轮子都往后走"。
 *   STEERINV    管【左右两边的分配】：错了的表现是"车能往前开、也能走直线，
 *     但纠偏时总是朝反方向拐"。
 *
 * 【★★ 为什么这次要从 1 改回 0 —— 这两个开关是耦合的 ★★】
 *   把所有轮子的极性同时翻一次，"前后方向"会翻，**"偏航方向"也一起翻**。
 *   本车的实际过程正好走了一遍：
 *
 *     ① 最开始：MLINV=0, MRINV=1 → 两个轮子都被命令前进、却都在往后转
 *        （轮子整体反了一次），于是偏航也跟着反了一次
 *        ⇒ 那时 STEERINV 必须 = 1 才能把转向掰正
 *
 *     ② 换电机之后：改成了 MLINV=1, MRINV=0 → 前进方向对了，
 *        同时偏航也翻回了正确方向
 *        ⇒ 此时 STEERINV 必须 = 0；还留 1 就等于又反了一次，
 *           现象是"车一进循迹模式就朝远离线的方向拐，
 *           冲出线后丢线保护让它原地打转，看起来像走不起来"
 *
 * 【★ 心法：先修前进，再修转向，改一个必须复核另一个】
 *   1) 先用测试 4 把 MLINV / MRINV 调对（两轮都朝车头转）
 *   2) 再上线验证 STEERINV：让线偏在车的左边，车应该朝【左】拐
 *   3) 只要动了第 1 步，第 2 步的结论就作废，必须重新验一遍
 *
 * 【改错了会怎样】
 *   本来是对的、却把它设成 1 → 车会朝远离线的方向冲出去（几秒钟就看出来）。
 *   菜单里拨一下就能试，不用重编重烧。 */
#define DEF_STEER_INV       0
/* ★ 电机转向取反的出厂默认值直接取自 bsp_config.h 的宏。
   这样"改宏重编"和"在菜单里拨"共用同一个初始值，两套用法不会打架 */
#define DEF_MOTOR_L_INV     MOTOR_L_INVERT
#define DEF_MOTOR_R_INV     MOTOR_R_INVERT

#define DEF_LOST_TURN       1200        /* 丢线时按原方向猛打 */
/* ★ 默认 0 = 丢线不停车，一直朝最后看到线的方向找下去，直到人工停车。
 *   调试阶段（车还跟不上线）这是唯一好用的设置：
 *   否则"走 0.8 秒就刹车"会让人不停按 START，根本看不出控制效果。
 *   循迹整定好之后，想让它冲出赛道时自动停，改成 800~1500 即可。 */
#define DEF_LOST_STOP_MS    6000
#define DEF_CROSS_HOLD_MS   260         /* 十字路口保持直行 0.26 秒通过 */

/* ★ 默认 0 = 关闭堵转保护。
 *   它会判"给了占空比、编码器却一个计数都没有" —— 编码器如果还没接线，
 *   每一次跑车都会在 1.5 秒后被误判成堵转而刹车，
 *   现象就是"走一段停一段"，非常容易被误当成控制问题。
 *   等串口敲 6 确认编码器读数正常之后，再打开（1000~2000 比较合适）。 */
#define DEF_STALL_MS        0

/* ★ 支撑脚行程默认给足 180°（0 → 180 走满舵机的整个行程）。
 *   这两个值必须实车标定：
 *     · 如果某一端听到舵机"嗡嗡"顶死不走，把那一端往回收 5~10 度
 *     · 菜单 TUNE PARAMS 里改 LEGUP / LEGDN 能立刻看到脚动，调完按 S 存 Flash */
/* ★★ 路线各阶段的默认时长（毫秒）★★
 *   就是"发车前写在脚本里"的那一份，现在菜单里随时能改，改完立刻生效 ——
 *   下一次按 K1 跑路线就用新值 */
#define DEF_RT_LF1_MS       10500       /* step1 循线 10.5s */
#define DEF_RT_TN1_MS       900         /* step2 左转 ★ 待标定 */
#define DEF_RT_LF2_MS       17000       /* step3 循线 17s */
#define DEF_RT_TN2_MS       900         /* step4 左转 ★ 待标定 */
#define DEF_RT_LF3_MS       15000       /* step5 循线 15s */
#define DEF_RT_TN3_MS       900         /* step6 左转 ★ 待标定 */
/* step7：上台阶那一段的循线时长（毫秒）。
   这 5 秒里车一边循线往前走，支撑脚一边按 Route_ServoAt() 排好的
   时间点上下摆动（见 app_route.c）。实测台阶的线中断很短，
   普通循迹就能开过去，不需要特殊处理 */
#define DEF_RT_LF4_MS       5000        /* 循线到上台阶地形的最长时间 */
/* 穿一块地形时"盲走"的时间上限（毫秒）。
 * ★ 它是【超时兜底】不是【计时走】：正常由"重新连续看到线"收尾。
 *   给正常穿越时间的两三倍 —— 电池掉电压时穿得慢，别卡太紧。 */
#define DEF_RT_CROSS_MS     8000

#define DEF_LEG_UP_DEG      0           /* 全收起 */
#define DEF_LEG_DOWN_DEG    180         /* 全放下（撑到最高） */

/* ==========================================================================
 * 参数描述表
 * --------------------------------------------------------------------------
 *   field        name        type       dec  step    min     max
 *   名字最好 <= 8 个字符，否则 128 像素一行放不下（一行 21 个 6x8 字符）；
 *   显示时前面还要留 2 个字符给光标 ">"。
 * ========================================================================== */
const APP_ParamItem_t g_param_table[] = {
    /* ================= 装车第一步：先把方向弄对 =================
     * ★ 这三项放在表的最前面是有意的。
     *   参数列表一屏只显示 6 行，其余要按"下移"翻 —— 而"电机转向对不对"
     *   是每次装车第一个要解决的问题（而且错了车根本不走直线）。
     *   把方向开关放在第一屏，开机进 TUNE PARAMS 直接就能看到、能改，
     *   不用先翻十下才找到。
     *
     *   判断顺序（一定要用测试 4 验，别靠猜）：
     *     1) 先看 MLINV / MRINV —— 两个轮子是不是都往前转
     *        命令两轮都前进时车原地打转 = 有一个轮子的正负极接反了
     *     2) 两个轮子都对了，再上线跑 —— 如果越纠越偏，才动 STEERINV
     *        （它管的是"看到线在左边该往左拐"这个方向，不是电机本身） */
    { "ACTLOW",   offsetof(APP_Param_t, act_low),      PTYPE_U8,   0,   1.0f,    0.0f,     1.0f },
    { "MLINV",    offsetof(APP_Param_t, motor_l_inv),  PTYPE_U8,   0,   1.0f,    0.0f,     1.0f },
    { "MRINV",    offsetof(APP_Param_t, motor_r_inv),  PTYPE_U8,   0,   1.0f,    0.0f,     1.0f },
    { "SLOWK",    offsetof(APP_Param_t, slow_k),       PTYPE_F32,  2,   0.05f,   0.0f,     1.50f },
    /* ---- 转角模式（大车/履带车专用，详见 app_car.c 里的状态机说明） ---- */
    { "PIVOT",    offsetof(APP_Param_t, pivot_spd),    PTYPE_I16,  0,  50.0f,    0.0f,  3599.0f },
    { "STEERINV", offsetof(APP_Param_t, steer_inv),    PTYPE_U8,   0,   1.0f,    0.0f,     1.0f },
    /* ------------------------------ 转向环 ------------------------------ */
    { "KP",       offsetof(APP_Param_t, turn_kp),      PTYPE_F32,  0,  50.0f,    0.0f,  4000.0f },
    { "KI",       offsetof(APP_Param_t, turn_ki),      PTYPE_F32,  0,   5.0f,    0.0f,   500.0f },
    { "KD",       offsetof(APP_Param_t, turn_kd),      PTYPE_F32,  0,  20.0f,    0.0f,  2000.0f },
    { "DEADZONE", offsetof(APP_Param_t, dead_zone),    PTYPE_F32,  2,   0.02f,   0.0f,     0.50f },
    /* ------------------------------- 行进 ------------------------------- */
    { "BASESPD",  offsetof(APP_Param_t, base_speed),   PTYPE_I16,  0,  50.0f,    0.0f,  3599.0f },
    { "TURNMAX",  offsetof(APP_Param_t, turn_limit),   PTYPE_I16,  0,  50.0f,  100.0f,  3599.0f },
    { "MINSPD",   offsetof(APP_Param_t, inner_min_spd),PTYPE_I16,  0,  50.0f,    0.0f,  2000.0f },
    { "RAMPms",   offsetof(APP_Param_t, ramp_ms),      PTYPE_U16,  0,  50.0f,    0.0f,  3000.0f },
    /* ------------------------------- 丢线 ------------------------------- */
    { "LOSTTRN",  offsetof(APP_Param_t, lost_turn),    PTYPE_I16,  0,  50.0f,    0.0f,  3599.0f },
    { "LOSTms",   offsetof(APP_Param_t, lost_stop_ms), PTYPE_U16,  0, 100.0f,    0.0f, 15000.0f },
    /* ------------------------------- 十字 ------------------------------- */
    { "CROSSms",  offsetof(APP_Param_t, cross_hold_ms),PTYPE_U16,  0,  50.0f,    0.0f,  2000.0f },
    /* ------------------------------- 保护 ------------------------------- */
    { "STALLms",  offsetof(APP_Param_t, stall_ms),     PTYPE_U16,  0, 100.0f,    0.0f, 10000.0f },
    /* ------------------------------ 支撑脚 ------------------------------ */
    /* ★ 支撑脚两个角度的步长是 1 度（其它参数是 5 或更大）：
       角度是"看着机械位置一点点试出来"的，5 度一跳太粗，
       1 度正好；要快速大范围挪动就长按（长按/连发 = 5 倍步长 = 5 度/次） */
    { "LEGUP",    offsetof(APP_Param_t, leg_up_deg),   PTYPE_U8,   0,   1.0f,    0.0f,   180.0f },
    { "LEGDN",    offsetof(APP_Param_t, leg_down_deg), PTYPE_U8,   0,   1.0f,    0.0f,   180.0f },

    /* ===================== 以下是隐藏项（不进 TUNE PARAMS）=====================
     * ★ 必须放在表的最后 —— 参数列表只遍历前 g_param_vis_count 项，
     *   这个数就是"从头开始连续的 hidden = 0 的项数"，见 APP_Param_Init()。
     *
     * 路线各段时长：只在菜单的 ROUTE TIME 页里出现。
     * 放进 TUNE PARAMS 会把列表撑得很长（而且以后还会加步骤），所以隐藏起来。
     * 循线段步长 100ms（一段十几秒，长按 = 500/次）
     * 转弯段步长  50ms（一段不到 1 秒，50ms 大约对应 5 度，标定时要细）
     * ======================================================================= */
    { "LF1_ms",   offsetof(APP_Param_t, rt_lf1_ms),    PTYPE_U16,  0, 100.0f,    0.0f, 60000.0f, 1u },
    { "TN1_ms",   offsetof(APP_Param_t, rt_tn1_ms),    PTYPE_U16,  0,  50.0f,    0.0f, 10000.0f, 1u },
    { "LF2_ms",   offsetof(APP_Param_t, rt_lf2_ms),    PTYPE_U16,  0, 100.0f,    0.0f, 60000.0f, 1u },
    { "TN2_ms",   offsetof(APP_Param_t, rt_tn2_ms),    PTYPE_U16,  0,  50.0f,    0.0f, 10000.0f, 1u },
    { "LF3_ms",   offsetof(APP_Param_t, rt_lf3_ms),    PTYPE_U16,  0, 100.0f,    0.0f, 60000.0f, 1u },
    { "TN3_ms",   offsetof(APP_Param_t, rt_tn3_ms),    PTYPE_U16,  0,  50.0f,    0.0f, 10000.0f, 1u },
    { "LF4_ms",   offsetof(APP_Param_t, rt_lf4_ms),    PTYPE_U16,  0, 100.0f,    0.0f, 60000.0f, 1u },
    { "CROSS_ms", offsetof(APP_Param_t, rt_cross_ms),  PTYPE_U16,  0, 500.0f, 1000.0f, 30000.0f, 1u },
};

const uint8_t g_param_count = (uint8_t)(sizeof(g_param_table) / sizeof(g_param_table[0]));

/* "可见参数"个数（TUNE PARAMS 能翻到几项）。上电时由 APP_Param_Init() 数出来，
   见头文件里对 g_param_vis_count 的说明 */
uint8_t g_param_vis_count = 0u;

/* 脏标记：参数被改过但还没保存回 Flash */
static uint8_t s_dirty = 0u;

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/** 取字段地址（用字节偏移算，表里存的就是偏移） */
static void *param_field_ptr(APP_Param_t *p, const APP_ParamItem_t *item)
{
    return (void *)((uint8_t *)p + item->offset);
}

/** 按类型读一个字段并统一转成 float */
static float param_read_field(const APP_Param_t *p, const APP_ParamItem_t *item)
{
    const void *addr = (const void *)((const uint8_t *)p + item->offset);

    switch (item->type) {
    case PTYPE_F32: return *(const float   *)addr;
    case PTYPE_I16: return (float)(*(const int16_t *)addr);
    case PTYPE_U16: return (float)(*(const uint16_t *)addr);
    case PTYPE_U8:  return (float)(*(const uint8_t  *)addr);
    default:        return 0.0f;
    }
}

/**
  * @brief  按类型写一个字段
  * @note   ★ 先夹到参数表的范围再写。这里是"最后一道闸"：
  *         任何调用者（菜单、串口）都不需要自己记得限幅
  */
static void param_write_field(APP_Param_t *p, const APP_ParamItem_t *item, float v)
{
    void *addr = param_field_ptr(p, item);

    if (v < item->vmin) { v = item->vmin; }
    if (v > item->vmax) { v = item->vmax; }

    switch (item->type) {
    case PTYPE_F32:
        *(float *)addr = v;
        break;
    case PTYPE_I16:
        /* +0.5 / -0.5 是为了四舍五入而不是直接截断：
           串口敲 "1300.7" 时截断会变成 1300，四舍五入也是 1300，
           但敲 "1299.9" 时截断会变成 1299，看着像丢了一位 */
        *(int16_t *)addr = (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
        break;
    case PTYPE_U16:
        *(uint16_t *)addr = (uint16_t)(v + 0.5f);
        break;
    case PTYPE_U8:
        *(uint8_t *)addr = (uint8_t)(v + 0.5f);
        break;
    default:
        break;
    }
}

/** 把出厂默认值逐字段填进去。
 *  ★ 先 memset 再逐字段赋值，而不是直接结构体赋值：
 *    这样结构体内部的填充字节一定是 0，校验和才算得稳。 */
static void param_fill_default(APP_Param_t *p)
{
    memset(p, 0, sizeof(*p));

    p->magic   = PARAM_MAGIC;
    p->version = (uint16_t)PARAM_VERSION;

    p->turn_kp = DEF_TURN_KP;
    p->turn_ki = DEF_TURN_KI;
    p->turn_kd = DEF_TURN_KD;
    p->dead_zone = DEF_DEAD_ZONE;

    p->base_speed    = (int16_t)DEF_BASE_SPEED;
    p->turn_limit    = (int16_t)DEF_TURN_LIMIT;
    p->inner_min_spd = (int16_t)DEF_INNER_MIN_SPD;
    p->ramp_ms       = (uint16_t)DEF_RAMP_MS;
    p->steer_inv     = (uint8_t)DEF_STEER_INV;
    p->motor_l_inv   = (uint8_t)DEF_MOTOR_L_INV;
    p->motor_r_inv   = (uint8_t)DEF_MOTOR_R_INV;
    p->act_low       = (uint8_t)DEF_ACT_LOW;
    p->pivot_spd     = (int16_t)DEF_PIVOT_SPD;
    p->slow_k        = DEF_SLOW_K;

    p->lost_turn    = (int16_t)DEF_LOST_TURN;
    p->lost_stop_ms = (uint16_t)DEF_LOST_STOP_MS;

    p->cross_hold_ms = (uint16_t)DEF_CROSS_HOLD_MS;

    p->stall_ms = (uint16_t)DEF_STALL_MS;

    p->rt_lf1_ms = (uint16_t)DEF_RT_LF1_MS;
    p->rt_tn1_ms = (uint16_t)DEF_RT_TN1_MS;
    p->rt_lf2_ms = (uint16_t)DEF_RT_LF2_MS;
    p->rt_tn2_ms = (uint16_t)DEF_RT_TN2_MS;
    p->rt_lf3_ms = (uint16_t)DEF_RT_LF3_MS;
    p->rt_tn3_ms = (uint16_t)DEF_RT_TN3_MS;
    p->rt_lf4_ms = (uint16_t)DEF_RT_LF4_MS;
    p->rt_cross_ms = (uint16_t)DEF_RT_CROSS_MS;

    p->leg_up_deg   = (uint8_t)DEF_LEG_UP_DEG;
    p->leg_down_deg = (uint8_t)DEF_LEG_DOWN_DEG;

    p->checksum = APP_Param_Checksum(p);
}

/* ==========================================================================
 * 校验和
 * ========================================================================== */

uint16_t APP_Param_Checksum(const APP_Param_t *p)
{
    const uint8_t *b = (const uint8_t *)p;
    uint32_t len = (uint32_t)offsetof(APP_Param_t, checksum);   /* 不含 checksum 本身 */
    uint32_t sum = 0u;
    uint32_t i;

    /* 简单的加法校验（不是 CRC）。
       对"整页被擦成 0xFF"和"个别 bit 翻转"这两种最常见的 Flash 事故
       已经足够 —— 要的只是"拦住乱码"，不是防篡改。 */
    for (i = 0u; i < len; i++) {
        sum = (sum + (uint32_t)b[i]) & 0xFFFFu;
    }

    /* 取反后返回：这样"全 0"或"全 0xFF"的极端数据都不会碰巧通过 */
    return (uint16_t)(~sum & 0xFFFFu);
}

/* ==========================================================================
 * Flash 读写
 * ========================================================================== */

#if PARAM_FLASH_ENABLE

/**
  * @brief  把 g_param 写进 Flash 最后一页
  * @retval 1 = 成功
  *
  * 【为什么全程不能被打断】
  *   Flash 擦写期间 CPU 从 Flash 取指会被硬件暂停（F1 没有取指缓冲），
  *   所以这个函数只能在"车停着"的时候调用。app_menu 里保存参数
  *   只在待机状态开放，就是这个原因。
  */
static uint8_t param_flash_write(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_err = 0xFFFFFFFFu;
    const uint8_t *src = (const uint8_t *)&g_param;
    uint32_t len = (uint32_t)sizeof(APP_Param_t);
    uint32_t i;
    uint8_t ok = 0u;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0u;
    }

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = PARAM_FLASH_PAGE_ADDR;
    erase.NbPages     = 1u;

    /* ★ 整页擦除。擦之前要确认这一页不属于程序代码，
       否则"保存参数"会把程序擦出一个洞。见 bsp_config.h 第 11 节的警告 */
    if (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK) {
        ok = 1u;

        /* F1 的 Flash 只能按"半字（16 位）"编程，所以两字节两字节地写 */
        for (i = 0u; i < len; i += 2u) {
            uint16_t hw;

            if ((i + 1u) < len) {
                hw = (uint16_t)((uint16_t)src[i] | ((uint16_t)src[i + 1u] << 8));
            } else {
                /* 结构体长度是奇数时的兜底（当前是偶数，走不到这里）：
                   高字节补 0xFF，和 Flash 擦除态一致 */
                hw = (uint16_t)((uint16_t)src[i] | 0xFF00u);
            }

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                  PARAM_FLASH_PAGE_ADDR + i,
                                  (uint64_t)hw) != HAL_OK) {
                ok = 0u;
                break;
            }
        }
    }

    HAL_FLASH_Lock();

    /* ★ 回读比对：写完不算完，要确认读回来的和想写的完全一致。
       不过这一关就老老实实返回失败，让上层告诉用户"保存失败"，
       而不是骗用户说存好了、下次上电却回到默认值 */
    if (ok != 0u) {
        ok = (memcmp((const void *)PARAM_FLASH_PAGE_ADDR, src, len) == 0) ? 1u : 0u;
    }

    return ok;
}

static uint8_t param_flash_read(APP_Param_t *out)
{
    /* Flash 是内存映射的，直接当普通内存读就行，不需要任何 HAL 调用 */
    memcpy(out, (const void *)PARAM_FLASH_PAGE_ADDR, sizeof(APP_Param_t));
    return 1u;
}

#endif /* PARAM_FLASH_ENABLE */

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

void APP_Param_ResetDefault(void)
{
    param_fill_default(&g_param);
    s_dirty = 1u;       /* 默认值和 Flash 里的旧值不一样，算"已修改" */
}

void APP_Param_Init(void)
{
    uint8_t loaded = 0u;

    /* ---- 先数出"可见参数"的个数 ----
     * 参数表约定：可见项（hidden = 0）排在前面，隐藏项排在最后。
     * TUNE PARAMS 列表只遍历前 g_param_vis_count 项，所以这里数的是
     * "从第 0 项开始连续有多少个 hidden = 0"。
     * ★ 万一有人把可见项写到隐藏项后面，这里的计数会提前停下 ——
     *   后果是那一项在 TUNE PARAMS 里看不见（但 ROUTE TIME 之类的
     *   专用页按名字查，依然能找到）。这个失败方式是"少显示一项"，
     *   而不是"菜单崩了"，可以接受。 */
    g_param_vis_count = 0u;
    while ((g_param_vis_count < g_param_count)
           && (g_param_table[g_param_vis_count].hidden == 0u)) {
        g_param_vis_count++;
    }

#if PARAM_FLASH_ENABLE
    loaded = APP_Param_Load();
#endif

    if (loaded == 0u) {
        /* 第一次上电、Flash 被擦过、或者参数区损坏 —— 三种情况都走这里 */
        APP_Param_ResetDefault();
        s_dirty = 0u;   /* 默认值本来就没存过，不必显示 MODIFIED 去烦用户 */
    }

    APP_Param_Clamp();
}

void APP_Param_Clamp(void)
{
    uint8_t i;

    for (i = 0u; i < g_param_count; i++) {
        const APP_ParamItem_t *item = &g_param_table[i];
        float v = param_read_field(&g_param, item);

        /* 读出来再写回去，顺带完成"按参数表限幅"。
           用 param_write_field 而不是自己写 switch，保证限幅规则只有一份 */
        param_write_field(&g_param, item, v);
    }

    /* 结构体内的搬运字段也要保护：magic/version 被改坏的话，
       下次上电会判为无效参数，白丢一次标定结果 */
    g_param.magic   = PARAM_MAGIC;
    g_param.version = (uint16_t)PARAM_VERSION;

    /* 检查和不属于参数表，手动补 */
    g_param.checksum = APP_Param_Checksum(&g_param);
}

uint8_t APP_Param_Load(void)
{
#if PARAM_FLASH_ENABLE
    APP_Param_t tmp;

    param_flash_read(&tmp);

    if (tmp.magic != PARAM_MAGIC) {
        return 0u;      /* 空片（全 0xFF）或别的程序留下的数据 */
    }
    if (tmp.version != (uint16_t)PARAM_VERSION) {
        return 0u;      /* 结构体改过版本，老数据不再兼容 */
    }
    if (tmp.checksum != APP_Param_Checksum(&tmp)) {
        return 0u;      /* 数据被写坏 */
    }

    g_param = tmp;
    s_dirty = 0u;
    return 1u;
#else
    return 0u;
#endif
}

uint8_t APP_Param_Save(void)
{
#if PARAM_FLASH_ENABLE
    uint8_t ok;

    APP_Param_Clamp();                  /* 存之前再夹一次，防止存进非法值 */
    g_param.magic   = PARAM_MAGIC;
    g_param.version = (uint16_t)PARAM_VERSION;
    g_param.checksum = APP_Param_Checksum(&g_param);

    ok = param_flash_write();
    if (ok != 0u) {
        s_dirty = 0u;
    }
    return ok;
#else
    return 0u;
#endif
}

uint8_t APP_Param_IsDirty(void)
{
    return s_dirty;
}

void APP_Param_MarkClean(void)
{
    s_dirty = 0u;
}

/* ==========================================================================
 * 按索引读写
 * ========================================================================== */

float APP_Param_GetByIndex(uint8_t idx)
{
    if (idx >= g_param_count) {
        return 0.0f;
    }
    return param_read_field(&g_param, &g_param_table[idx]);
}

void APP_Param_SetByIndex(uint8_t idx, float value)
{
    if (idx >= g_param_count) {
        return;
    }

    param_write_field(&g_param, &g_param_table[idx], value);

    /* 每改一次就重算校验和。这样"还没保存"的内存参数也是自洽的，
       串口随时打印都不会看到一个对不上校验和的中间态 */
    g_param.checksum = APP_Param_Checksum(&g_param);
    s_dirty = 1u;
}

uint8_t APP_Param_FindByName(const char *name)
{
    uint8_t i;

    if (name == NULL) {
        return 0xFFu;
    }

    for (i = 0u; i < g_param_count; i++) {
        const char *a = g_param_table[i].name;
        const char *b = name;
        uint8_t same = 1u;

        /* 自己写字符串比较，不引 strcmp：
           参数名都很短，而且这里要的是"忽略大小写"的宽松匹配
           （串口里敲 kp 和 KP 都应该认） */
        while ((*a != '\0') && (*b != '\0')) {
            char ca = *a;
            char cb = *b;

            if ((ca >= 'a') && (ca <= 'z')) { ca = (char)(ca - 'a' + 'A'); }
            if ((cb >= 'a') && (cb <= 'z')) { cb = (char)(cb - 'a' + 'A'); }

            if (ca != cb) {
                same = 0u;
                break;
            }
            a++;
            b++;
        }

        if ((same != 0u) && (*a == '\0') && (*b == '\0')) {
            return i;
        }
    }

    return 0xFFu;
}

void APP_Param_Print(void)
{
    uint8_t i;

    BSP_UART_SendString("\r\n----- current parameters -----\r\n");

    for (i = 0u; i < g_param_count; i++) {
        const APP_ParamItem_t *item = &g_param_table[i];
        float v = APP_Param_GetByIndex(i);

        /* ★ 串口输出一律用 ASCII：armcc 会按主机区域设置解释源文件里的
           多字节字符，某些中文字节对里恰好含 0x22/0x5C，会把字符串的
           引号"吃掉"，报出莫名其妙的 missing closing quote。
           注释里写中文没问题，字符串字面量里不要写。 */
        if (item->type == PTYPE_F32) {
            BSP_UART_Printf("  %-9s = %.*f   (step %.2f, range %.2f ~ %.2f)\r\n",
                            item->name, (int)item->decimals, (double)v,
                            (double)item->step, (double)item->vmin, (double)item->vmax);
        } else {
            BSP_UART_Printf("  %-9s = %d   (step %.0f, range %.0f ~ %.0f)\r\n",
                            item->name, (int)(v + (v >= 0.0f ? 0.5f : -0.5f)),
                            (double)item->step, (double)item->vmin, (double)item->vmax);
        }
    }

    BSP_UART_Printf("  state: %s\r\n",
                    (APP_Param_IsDirty() != 0u) ? "MODIFIED (not saved)" : "SAVED (=flash)");
    BSP_UART_SendString("--------------------\r\n");
}
