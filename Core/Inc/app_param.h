/**
  ******************************************************************************
  * @file    app_param.h
  * @brief   可调参数集中管理 + 掉电保存到 Flash
  *
  * 【在工程里的位置】 全车的"参数总线"
  *
  *   app_menu ──改──►┐
  *                   ├──► g_param（唯一的参数存放处）──读──► app_car / app_leg
  *   串口命令 ──改──►┘                                      （每个控制周期都读）
  *
  *   ★ 因为 app_car 每个控制周期都重新读一遍 g_param，
  *     所以调参是"下一拍就生效"的，不需要任何通知/刷新机制。
  *
  * 【参数描述表：本文件最核心的设计】
  *   每个参数的 名字/类型/步长/上下限 都写在 g_param_table 里，
  *   菜单只负责"把表渲染出来 + 改值"，完全不需要知道有哪些参数。
  *   好处：
  *     · 加一个参数 = 在结构体里加个字段 + 在表里加一行，菜单代码一行不用改
  *     · 上下限只有一处定义，菜单里夹一次、APP_Param_Clamp() 里夹一次，
  *       用的是同一份数据，不可能对不上
  *
  * 【掉电保存】
  *   参数存在 Flash 的最后一页（地址见 bsp_config.h 第 11 节）。
  *   结构体里有 magic + version + checksum 三道校验，任何一个不对
  *   都会退回默认值 —— 宁可回到出厂手感，也不能拿一堆乱码去跑车。
  * ============================================================================
  */
#ifndef __APP_PARAM_H
#define __APP_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/* ==========================================================================
 * 参数结构体 —— ★ 加字段时记得同步更新 app_param.c 里的两处：
 *   1. g_param_default 默认值表
 *   2. g_param_table   描述表（名字/步长/上下限）
 *   漏了第 2 步的话，新参数在菜单里看不到、也不会被限幅。
 * ========================================================================== */
typedef struct {
    uint32_t magic;         /* PARAM_MAGIC，用来判断这一页 Flash 里是不是有效参数 */
    uint16_t version;       /* 结构体改过就把 PARAM_VERSION 加一，老数据自动作废 */

    /* ------------------------- 转向环 PID ------------------------- */
    float    turn_kp;       /* 比例：误差的放大倍数，决定"反应有多急" */
    float    turn_ki;       /* 积分：消稳态偏差。循迹里通常给 0 或很小 */
    float    turn_kd;       /* 微分：提前量，抑制画龙，别给太大否则抖 */
    float    dead_zone;     /* 归一化误差死区 0~0.5，专治直道上的左右抽动 */

    /* --------------------------- 行进 ----------------------------- */
    int16_t  base_speed;    /* 直线基础速度（占空比，满量程 MOTOR_PWM_MAX=3599） */
    int16_t  turn_limit;    /* 转向量限幅：PID 输出最多能把差速拉到多大 */
    int16_t  inner_min_spd; /* 内侧轮最低速。>0 时弯道内轮不会停转，跑得更顺 */
    uint16_t ramp_ms;       /* 起步斜坡时间：0 → base_speed 用多少毫秒爬上去 */
    uint8_t  steer_inv;     /* 0 = 正常；1 = 转向差速取反（左右轮整体装反时用）*/
    uint8_t  motor_l_inv;   /* 0 = 正常；1 = 左电机转向取反（电机正负极接反了）*/
    uint8_t  motor_r_inv;   /* 0 = 正常；1 = 右电机转向取反（电机正负极接反了）*/
    uint8_t  act_low;       /* ★ 探头极性：1 = 压到黑线读到低电平，0 = 读到高电平。
                               实车实测本车是 0（高电平有效），见 bsp_config.h 的推导。
                               极性错了的现象：掩码整条反过来 —— 车压在线上，
                               掩码却在离线的那几路显示 1 */
    int16_t  pivot_spd;     /* ★ 路线脚本里"原地转弯"的速度（占空比），
                               对应 Route_TurnLeft() / Route_TurnRight() */
    float    slow_k;        /* ★ 大偏差自动减速系数，0 = 关闭。
                               实际基础速度 = BASESPD × (1 - slow_k × |误差|)
                               车越偏跑得越慢，把时间让给转向 ——
                               大车/履带车必开，详见 app_car.c "速度规划"一节 */

    /* --------------------------- 丢线 ----------------------------- */
    int16_t  lost_turn;     /* 丢线时的固定转向量（按最后看到线的方向转） */
    uint16_t lost_stop_ms;  /* 丢线持续超过这么久就停车报警。★ 0 = 不停车，一直找线 */


    /* --------------------------- 十字 ----------------------------- */
    uint16_t cross_hold_ms; /* 八路全黑（十字/停止线）时保持直行的时长 */

    /* -------------------------- 保护 ------------------------------ */
    uint16_t stall_ms;      /* 堵转判定的持续时间：给电这么久还不动就报警停车。0 = 关闭 */

    /* ------------------- 路线脚本里还剩的两类时间 ------------------- */
    /* ★ 循线段已经【不再用时间】了 —— 改成"等某个循迹结果事件发生第 n 次"
       （见 app_route.c 的 Route_LineFollowUntilEvent）。
       所以这里只剩两种还必须用时间的量：
         BLD = Blind（盲走）：闭着眼睛直冲的时长，用来过地形
         TN  = Turn（转弯）：原地左转的时长，靠实车标定角度
       命名里的数字是第几段，和脚本里的注释一一对应。
       做成参数是为了能在菜单 ROUTE TIME 页里一边跑一边调，不用重编重烧。 */
    uint16_t rt_bld1_ms;    /* step1 盲走时长 */
    uint16_t rt_post_ms;    /* ★ 过线后再循迹的时长 ——
                               step2（第 1 根）和 step4①（第 4 根）共用这一个。
                               车架大，压过横线时车身还没摆正；
                               再循迹这一小会儿，车刚好走到"线在正中间"的位置，
                               这时候原地左转，转出来的落点才准 */
    uint16_t rt_post2_ms;   /* ★ step4③（第 5 根横线）专用的"过线后再循迹"时长。
                               和上面那个分开，是因为这一处的距离不一样，
                               实测只要 1 秒（上面两处是 1.5 秒）*/
    uint16_t rt_tn1_ms;     /* step2 左转时长（★ 需实车标定） */
    uint16_t rt_bld2_ms;    /* step3 盲走时长 */
    uint16_t rt_tn2_ms;     /* step4 第 5 根横线后左转时长（★ 需实车标定） */
    uint16_t rt_tn3_ms;     /* step4 第 6 根横线后左转时长（★ 需实车标定） */
    uint16_t rt_tn4_ms;     /* step4 左三压线后左转时长（★ 需实车标定） */
    uint16_t rt_str_ms;     /* step4 左三压线后"直走"的时长（默认 1 秒）*/
    uint16_t rt_post3_ms;   /* step6：扫到【第 8 根】线之后，再循迹走多久才左转
                               （默认 1000 = 1 秒）*/
    uint16_t rt_tn5_ms;     /* step6 左转时长（★ 需实车标定，默认 1 秒）*/
    uint16_t rt_lf8_ms;     /* step8：第 9 根线之后，先循迹走多久（默认 3000 = 3 秒）*/
    uint16_t rt_tn7_ms;     /* step8 倒车之后的【左转】时长（默认 1200 ≈ 110°）*/
    uint16_t rt_tn8_ms;     /* step8 的【第 1 次右转】时长（默认 1100 = 1.1 秒）。
                               ★ 目标是转 100°：按"1000ms ≈ 90°"折算出来的，
                                 实车实测偏了就直接改这个数（它是时间，不是角度）。 */
    uint16_t rt_tn6_ms;     /* ★ step7/step8 的【掉头】时长（原地转 180°），默认 2000。
                               掉头 = 180°，按"90° ≈ 1000ms"估的 ——
                               实测转不够就调大、转过头就调小；
                               方向反了要把 app_route.c 里的
                               Route_TurnLeft 换成 Route_TurnRight。 */
    uint16_t rt_lf7_ms;     /* step7【先循迹走多久再放舵机】（默认 1000 = 1 秒）。
                               ★ 它同时是"支撑脚什么时候放下"的时间点：
                                  放下 = LF7_ms，升回 = LF7_ms + 2000
                                 （2000 那个间隔写死在 app_route.c 的 S7_DOWN_GAP_MS）
                               之后继续循迹，直到左路探头压到线为止。 */
    uint16_t rt_upst_ms;    /* step5【上台阶时盲走的总时长】（默认 13000 = 13 秒）。
                               舵机的四个时间点写死在 app_route.c 里
                               （2s 放下 / 4s 升起 / 6s 再放下 / 9s 完全升起），
                               所以这里只改"走多久" */

    /* -------------------------- 支撑脚 ---------------------------- */
    uint8_t  leg_up_deg;    /* 收起（行驶位）角度，★ 必须实车标定 */
    uint8_t  leg_down_deg;  /* 放下（支撑位）角度，★ 必须实车标定 */

    uint16_t checksum;      /* 前面所有字节的校验和，放在最后 */
} APP_Param_t;

/* ==========================================================================
 * 参数描述表
 * ========================================================================== */

/** 参数类型。用枚举而不是把所有参数都存成 float ——
 *  整型参数存 float 会让 Flash 里的字节数翻倍，而且显示时要处理小数位 */
typedef enum {
    PTYPE_F32 = 0,      /* 浮点，显示 decimals 位小数 */
    PTYPE_I16,          /* 有符号整数 */
    PTYPE_U16,          /* 无符号整数 */
    PTYPE_U8            /* 无符号字节 */
} APP_ParamType_t;

/** 一个可调参数的完整描述 */
typedef struct {
    const char     *name;       /* 屏幕显示名，★ 只能用 ASCII，建议 <= 8 字符 */
    uint16_t        offset;     /* 该字段在 APP_Param_t 里的字节偏移 */
    APP_ParamType_t type;       /* 类型 */
    uint8_t         decimals;   /* 显示小数位（只有 F32 用得到） */
    float           step;       /* 按一下 +/- 变化多少 */
    float           vmin;       /* 下限（含） */
    float           vmax;       /* 上限（含） */
} APP_ParamItem_t;

/** 参数表与参数个数。菜单直接遍历它们 */
extern const APP_ParamItem_t g_param_table[];
extern const uint8_t         g_param_count;

/** 全局参数实例。所有模块都读它 */
extern APP_Param_t g_param;

/* ==========================================================================
 * 接口
 * ========================================================================== */

/**
  * @brief  上电初始化：先从 Flash 读，读不到就用默认值
  * @note   必须在 app_car / app_leg 初始化之前调用
  */
void APP_Param_Init(void);

/** @brief 把 g_param 恢复成出厂默认值（不写 Flash，要保存请另调 APP_Param_Save） */
void APP_Param_ResetDefault(void);

/**
  * @brief  把每个字段夹到参数表规定的范围内
  * @note   菜单里已经夹过一次，这里是第二道防线：防止有人绕过菜单
  *         直接改结构体（比如串口命令算错）后拿非法值去跑车
  */
void APP_Param_Clamp(void);

/** @brief 计算校验和（对 checksum 字段之前的全部字节求和并取反） */
uint16_t APP_Param_Checksum(const APP_Param_t *p);

/**
  * @brief  从 Flash 读参数
  * @retval 1 = 读到有效参数并已填入 g_param；0 = 无效（magic/版本/校验和任一不符）
  */
uint8_t APP_Param_Load(void);

/**
  * @brief  把当前参数写进 Flash
  * @retval 1 = 成功（写完后做过回读比对）；0 = 失败或本工程关闭了保存功能
  * @note   ★ 会擦掉 Flash 最后一页！调用前请确认 bsp_config.h 里的
  *         PARAM_FLASH_PAGE_ADDR 没有和程序代码重叠。
  *         整个操作约 30ms，期间 CPU 会被阻塞（Flash 写不能被打断），
  *         所以只能在车停下来调参时调用，绝不能在行驶中调用。
  */
uint8_t APP_Param_Save(void);

/** @brief 参数自上次保存/加载以来是否被改过（菜单用来显示 SAVED / MODIFIED） */
uint8_t APP_Param_IsDirty(void);

/** @brief 标记为"未修改"（保存成功后内部会自动调） */
void APP_Param_MarkClean(void);

/* --------------------- 按索引读写（供菜单/串口用） --------------------- */

/**
  * @brief  取第 idx 个参数的当前值（统一转成 float）
  * @note   idx 越界返回 0，不崩
  */
float APP_Param_GetByIndex(uint8_t idx);

/**
  * @brief  写第 idx 个参数，并自动按参数表夹到合法范围
  * @note   写成功会置脏标记。菜单里 +/- 就是调它
  */
void APP_Param_SetByIndex(uint8_t idx, float value);

/** @brief 按名字查索引，找不到返回 0xFF（串口命令用，比如 "set KP 1500"） */
uint8_t APP_Param_FindByName(const char *name);

/** @brief 把全部参数打印到串口 */
void APP_Param_Print(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PARAM_H */
