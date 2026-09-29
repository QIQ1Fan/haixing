/**
  ******************************************************************************
  * @file    app_leg.c
  * @brief   支撑脚舵机实现
  *
  * 【数据流】
  *   APP_Leg_Down()
  *     └─► s_target = g_param.leg_down_deg          ← 只记目标，不动硬件
  *           │
  *   APP_Leg_Update()（每 10ms 由 main 调用）
  *           ├─ 已经在目标上 → 直接返回（开销几乎为零）
  *           └─ 没到 → s_angle 朝目标走最多 LEG_MOVE_STEP_DEG 度
  *                 └─► BSP_Servo_SetAngle(s_angle)
  *                       └─► 软件 PWM 出 50Hz 方波 ──► 舵机
  *
  * 【为什么角度用 float 存】
  *   斜坡是"每拍加一小步"，如果角度用整数，2 度/步在 180 度行程上
  *   只有 90 级，看上去一顿一顿的。用 float 累加、只在送给舵机时
  *   转成整数，实际步进就能落到 1 度以下，动作明显更顺。
  ******************************************************************************
  */
#include "app_leg.h"
#include "app_param.h"
#include "bsp_servo.h"

/* 软件认为脚当前所在的角度。上电先假设它在"收起位"，
   随后 APP_Leg_Init() 会真的把舵机摆过去，两者就一致了 */
static float s_angle = 0.0f;

/* 目标角度 */
static uint8_t s_target = 0u;

/* 是否已经就位（供 IsMoving 查询） */
static uint8_t s_moving = 0u;

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/** 角度夹到机械安全范围。参数表里已经夹过一次，这里是第二道防线 ——
 *  支撑脚顶死会把舵机烧掉，宁可多检查一次 */
static uint8_t leg_clamp_deg(float deg)
{
    if (deg < (float)LEG_SAFE_MIN_DEG) {
        deg = (float)LEG_SAFE_MIN_DEG;
    }
    if (deg > (float)LEG_SAFE_MAX_DEG) {
        deg = (float)LEG_SAFE_MAX_DEG;
    }
    return (uint8_t)(deg + 0.5f);
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void APP_Leg_Init(void)
{
    /* 上电直接摆到收起位：
       支撑脚如果在行进姿态下伸出，车一起步就会刮地、跑偏。
       直接到位（不走斜坡）是因为此刻电机还没使能，整车静止，
       舵机猛地一下不会造成什么后果，反而能立刻建立已知姿态。 */
    uint8_t up = g_param.leg_up_deg;

    s_target = leg_clamp_deg((float)up);
    s_angle  = (float)s_target;
    s_moving = 0u;

    BSP_Servo_SetAngle(s_angle);
}

/* ==========================================================================
 * 斜坡
 * ========================================================================== */

void APP_Leg_Update(void)
{
    float diff = (float)s_target - s_angle;

    /* 已经在目标上：一次浮点比较就返回，不产生任何硬件操作 */
    if ((diff < 0.5f) && (diff > -0.5f)) {
        if (s_moving != 0u) {
            /* 刚刚到位：补一次精确值，避免长期停在"差一点点"的位置上 */
            s_angle = (float)s_target;
            BSP_Servo_SetAngle(s_angle);
            s_moving = 0u;
        }
        return;
    }

    s_moving = 1u;

    /* 朝目标走一步。这一步是"最多走多少"，实际差距小于步长时就一次走完 */
    if (diff > 0.0f) {
        s_angle += (diff > (float)LEG_MOVE_STEP_DEG) ? (float)LEG_MOVE_STEP_DEG : diff;
    } else {
        s_angle -= ((-diff) > (float)LEG_MOVE_STEP_DEG) ? (float)LEG_MOVE_STEP_DEG : (-diff);
    }

    BSP_Servo_SetAngle(s_angle);
}

/* ==========================================================================
 * 控制接口
 * ========================================================================== */

void APP_Leg_Up(void)
{
    APP_Leg_SetTarget(g_param.leg_up_deg);
}

void APP_Leg_Down(void)
{
    APP_Leg_SetTarget(g_param.leg_down_deg);
}

void APP_Leg_SetTarget(uint8_t deg)
{
    s_target = leg_clamp_deg((float)deg);
    /* 目标一变，IsMoving 立刻为真，菜单上马上能反映出"正在动" */
    s_moving = ((s_target != (uint8_t)(s_angle + 0.5f)) ? 1u : 0u);
}

void APP_Leg_JumpTo(uint8_t deg)
{
    uint8_t d = leg_clamp_deg((float)deg);

    s_target = d;
    s_angle  = (float)d;
    s_moving = 0u;
    BSP_Servo_SetAngle(s_angle);
}

/* ==========================================================================
 * 查询
 * ========================================================================== */

uint8_t APP_Leg_GetAngle(void)
{
    return (uint8_t)(s_angle + 0.5f);
}

uint8_t APP_Leg_GetTarget(void)
{
    return s_target;
}

uint8_t APP_Leg_IsMoving(void)
{
    return s_moving;
}

uint8_t APP_Leg_IsDown(void)
{
    uint8_t cur;
    uint8_t down;
    uint8_t diff;

    if (s_moving != 0u) {
        return 0u;
    }

    cur  = APP_Leg_GetAngle();
    down = leg_clamp_deg((float)g_param.leg_down_deg);

    /* 给 2 度容差，免得因为四舍五入把"已经放下了"判成"还没放下"，
       菜单上就会一闪一闪的 */
    diff = (cur > down) ? (uint8_t)(cur - down) : (uint8_t)(down - cur);

    return (diff <= 2u) ? 1u : 0u;
}
