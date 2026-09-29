/**
  ******************************************************************************
  * @file    app_leg.h
  * @brief   支撑脚舵机：把车撑起来 / 收起
  *
  * 【在工程里的位置】 纯执行机构，不参与循迹闭环
  *
  *   app_menu（菜单里选"放下/收起"）──┐
  *                                    ├──► APP_Leg_SetTarget() ──► 目标角度
  *   以后的自动逻辑 ───────────────────┘                              │
  *                                                                   ▼
  *                              APP_Leg_Update()（每 10ms 走一小步）
  *                                                                   │
  *                                                                   ▼
  *                                            BSP_Servo_SetAngle() ──► PB4
  *
  * 【本阶段的实现范围】
  *   只做"手动放下 / 收起 / 指定角度 / 查询状态"。
  *   行驶中自动收放的逻辑（什么时候撑、什么时候收）属于后续阶段，
  *   到时候只要在 app_car 的状态变化处调用 APP_Leg_Up() / APP_Leg_Down()
  *   即可，本模块不用改。
  *
  * 【★ 为什么要"走小步"】
  *   支撑脚是刚性机构，角度一步到位等于让舵机瞬间满速冲击机械限位：
  *   齿轮容易打坏、车会被顶得跳一下、电流尖峰还可能把单片机拉复位。
  *   分成每 10ms 走 LEG_MOVE_STEP_DEG 度（默认 2 度），
  *   180 度走完约 0.9 秒 —— 又温柔又看不出慢。
  *
  * 【★ 角度必须实车标定】
  *   LEGUP / LEGDN 两个角度不在代码里写死，而是 app_param 里的可调参数，
  *   菜单里能改、能存 Flash。标定方法：
  *     1. 菜单里把 LEGDN 一点点加大，直到支撑脚刚好把车轮顶离地面
  *     2. 再回调 5~10 度留出余量（舵机不要一直顶在死点上，会发热烧掉）
  *     3. 同样方法定 LEGUP，让脚完全收进车体、不刮地面
  *     4. 按"保存"，掉电不丢
  * ============================================================================
  */
#ifndef __APP_LEG_H
#define __APP_LEG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/**
  * @brief  初始化支撑脚
  * @note   ★ 必须在 BSP_Servo_Init() 之后调用。
  *         内部会把脚立刻摆到"收起位"（不走小步，直接到位）——
  *         上电时车应该是行驶姿态，而不是先慢慢伸出脚来。
  */
void APP_Leg_Init(void);

/**
  * @brief  推进支撑脚的角度斜坡
  * @note   ★ 必须每 CTRL_PERIOD_MS（10ms）调用一次。
  *         没在移动时开销几乎为零（一次比较就返回）。
  */
void APP_Leg_Update(void);

/** @brief 目标设为"收起"位（行驶姿态，角度取 g_param.leg_up_deg） */
void APP_Leg_Up(void);

/** @brief 目标设为"放下"位（支撑姿态，角度取 g_param.leg_down_deg） */
void APP_Leg_Down(void);

/**
  * @brief  手动指定目标角度，会按 LEG_SAFE_MIN/MAX_DEG 夹一下
  * @note   标定用的接口。走斜坡，不会猛地一下
  */
void APP_Leg_SetTarget(uint8_t deg);

/**
  * @brief  跳过斜坡，立刻把脚摆到指定角度
  * @note   ★ 只给标定用。机械上没有任何缓冲，机构较脆时慎用
  */
void APP_Leg_JumpTo(uint8_t deg);

/** @brief 当前软件认为脚所在的角度（就是实际发给舵机的那个角度） */
uint8_t APP_Leg_GetAngle(void);

/** @brief 目标角度 */
uint8_t APP_Leg_GetTarget(void);

/** @brief 是否正在移动（顶到位之后返回 0，菜单可以用它显示 "..." ） */
uint8_t APP_Leg_IsMoving(void);

/** @brief 当前是否停在"放下"位（容差 LEG_SETTLE_MS 内算到位） */
uint8_t APP_Leg_IsDown(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LEG_H */
