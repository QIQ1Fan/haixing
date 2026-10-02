/**
  ******************************************************************************
  * @file    app_car.h
  * @brief   循迹控制：位置误差 → 转向环 PID → 左右轮差速
  *
  * 【在工程里的位置】 全车的"大脑"
  *
  *   八路探头 ──► 位置误差 ──► 转向环 PID ──┬──► 左轮 = 基础速度 + 转向量
  *                                          └──► 右轮 = 基础速度 - 转向量
  *   编码器   ──► 堵转检测 / 转速里程显示
  *
  * 【上游】 main.c 每 CTRL_PERIOD_MS（10ms）调用一次 APP_Car_Update()
  *         app_menu 调用 APP_Car_Start/Stop/Brake 控制启停
  * 【下游】 bsp_motor（执行）、bsp_track / bsp_encoder（反馈）、bsp_pid（算法）
  *
  * ============================================================================
  * 【★ 转向方向是怎么定的 —— 看懂这一段，整车的调试就顺了】
  *
  *   探头权重：最左 -7 … 最右 +7，所以
  *       误差 < 0  ⇔  黑线压在左边几路  ⇔  车偏到线的右边了  ⇔  要往左拐
  *       误差 > 0  ⇔  黑线压在右边几路  ⇔  车偏到线的左边了  ⇔  要往右拐
  *
  *   转向环输出：turn = PID(误差)，误差为正它也为正。
  *
  *   执行： 左轮 = 基础速度 + turn
  *          右轮 = 基础速度 - turn
  *     → turn > 0 时左轮快、右轮慢 → 车向右转 → 正好对应"误差为正要右拐" ✓
  *
  *   ★ 如果实车上方向反了（车偏左反而往右拐），有三种可能，按顺序排查：
  *       1. 两个电机整体接反了 → 把 MOTOR_L_INVERT 和 MOTOR_R_INVERT 同时改成 1
  *       2. 只是转向反了，前进正常 → 菜单里把 STEERINV 改成 1（不用拆车）
  *       3. 探头左右装反了 → 把 bsp_config.h 里 TRACK_WEIGHT_0..7 倒过来写
  *
  * ============================================================================
  * 【★ 每种路况怎么处理】
  *
  *   正常循迹    误差喂 PID，输出差速。
  *   直道微抖    误差死区（DEADZONE）把"车正好骑在线上"的小摆动吃掉。
  *   十字 / 全黑 八路全压线时保持直行 CROSSms 毫秒通过（否则会跟着路口横向的黑
  *               线甩头）。超过这个时间还没出去，说明不是十字，交给 PID 正常处理。
  *   丢线        八路全白持续 60ms 以上算丢线：
  *                 · 转向量强制定为 LOSTTRN，方向取"最后一次看到线的方向"
  *                 · 基础速度降到六成，给找线留时间
  *                 · 丢线超过 LOSTms 还没找回来 → 刹车停车 + 蜂鸣报警，等人工处理
  *   堵转        给了不小占空比、两个轮子却一动不动超过 STALLms →
  *               刹车停车报警（保护减速箱，也顺便发现"编码器线掉了"）
  * ============================================================================
  */
#ifndef __APP_CAR_H
#define __APP_CAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/** 小车状态机 */
typedef enum {
    APP_CAR_IDLE = 0,   /* 待机：电机停、可以随意调参 */
    APP_CAR_RUN,        /* 循迹行驶中 */
    APP_CAR_LOST,       /* 丢线超时，已停车报警 */
    APP_CAR_STALL       /* 疑似堵转，已停车报警 */
} APP_CarState_t;

/**
  * 循迹"结果事件"的编号 —— 路线脚本用它们来推进
  * @note 三种结果各自有一个计数器，见 APP_Car_GetEventCount()。
  *       掩码和去抖时间在 bsp_config.h 里的 CAR_EVT_* 一组宏里。
  */
typedef enum {
    APP_CAR_EVT_LEFT3 = 0,  /* 最左三路全压线（线跑到车头左边去了）*/
    APP_CAR_EVT_RIGHT3,     /* 最右三路全压线 */
    APP_CAR_EVT_BLACK,      /* 全黑（横线 / 十字 / 停止线）*/
    APP_CAR_EVT_NUM
} APP_CarEvent_t;

/** 实时状态快照。菜单、串口都从这里读，不自己去碰底层 */
typedef struct {
    uint8_t  state;         /* APP_CarState_t */
    uint8_t  mask;          /* 当前压线掩码（bit0 = 最左一路） */
    uint8_t  lost;          /* 1 = 当前判定为丢线 */
    uint8_t  crossing;      /* 1 = 当前八路全压（十字/停止线） */
    float    err;           /* 归一化位置误差 [-1, +1]，已按 STEERINV 取反 */
    float    turn;          /* 转向环输出（占空比单位） */
    int16_t  base_now;      /* 当前基础速度（含起步斜坡、丢线减速后的实际值） */
    int16_t  spd_l;         /* 实际下发给左轮的带符号速度 */
    int16_t  spd_r;         /* 实际下发给右轮的带符号速度 */
    uint16_t lost_ms;       /* 已经丢线多少毫秒（0 表示没丢线） */
    uint16_t cnt_left3;     /* ★ 最左三路全压线的累计次数 */
    uint16_t cnt_right3;    /* ★ 最右三路全压线的累计次数 */
    uint16_t cnt_black;     /* ★ 全黑的累计次数（横线/十字）*/
    uint32_t run_ms;        /* 本次启动已经跑了多少毫秒 */
    int32_t  dist_l;        /* 左轮累计计数（本次启动清零） */
    int32_t  dist_r;        /* 右轮累计计数 */
    float    rpm_l;         /* 左轮实时转速 */
    float    rpm_r;         /* 右轮实时转速 */
} APP_CarStatus_t;

/**
  * @brief  初始化循迹控制
  * @note   只在里面建 PID 实例、清状态，不动电机；
  *         真正让车动起来要调 APP_Car_Start()
  */
void APP_Car_Init(void);

/**
  * @brief  启动循迹
  * @note   会上电蜂鸣器"嘀"一声。车从 0 速度按 RAMPms 斜坡加速到 BASESPD，
  *         不会起步就窜出去。编码器里程会清零。
  */
void APP_Car_Start(void);

/** @brief 停车（滑行）：方向脚清零，电机自由减速。调参时用这个 */
void APP_Car_Stop(void);

/** @brief 急停（刹车）：绕组短接快速制动。要立刻停住时用 */
void APP_Car_Brake(void);

/** @brief 运行中调一次 = 停；停着调一次 = 启动。菜单里"START/STOP"用的就是它 */
void APP_Car_ToggleRun(void);

/**
  * @brief  跑一拍控制律
  * @note   ★ 必须每 CTRL_PERIOD_MS（10ms）调用一次，且要在
  *         BSP_Encoder_Update() 之后、BSP_Key_Scan() 之前那一小段里。
  *         待机状态下本函数几乎零开销，可以放心每拍都调
  */
void APP_Car_Update(void);

/** @brief 是否正在循迹行驶 */
uint8_t APP_Car_IsRunning(void);

/** @brief 当前状态（APP_CarState_t） */
uint8_t APP_Car_GetState(void);

/** @brief 取状态快照指针（只读，不用拷贝） */
const APP_CarStatus_t *APP_Car_GetStatus(void);

/** @brief 状态名（ASCII，给屏幕和串口用） */
const char *APP_Car_StateName(uint8_t state);

/**
  * @brief  ★ 取某个循迹"结果事件"的累计次数
  * @param  evt APP_CAR_EVT_LEFT3 / RIGHT3 / BLACK
  * @retval 从上一次 APP_Car_ResetEventCounts() 以来，该图案出现了几次
  *
  * 【计数规则：图案保持 + 重新武装】
  *   图案必须【连续保持 CAR_EVT_HOLD_MS(80ms)】才算一次 ——
  *   把过线的抖动、车体一晃都滤掉。
  *   计过之后，图案要【连续消失 CAR_EVT_REARM_MS(400ms)】才允许再计 ——
  *   这样"车正压在横线上"只会算一根，不会每 80ms 又数一根。
  *
  * 【为什么用计数而不是时间】
  *   电池掉电压、车速变慢时，用时间切分的路段会走不到位；
  *   而"第几次看到横线/第几次线跑到左边"和速度无关，永远对得上。
  */
uint16_t APP_Car_GetEventCount(uint8_t evt);

/**
  * @brief  ★ 直接设置某个事件计数器的值
  * @param  evt APP_CAR_EVT_LEFT3 / RIGHT3 / BLACK
  * @param  v   要设成的值
  * @note   给"从中间某一步开始跑"用的：脚本里等的是【绝对第几根横线】，
  *         从第 5 步开始跑时，必须先把 BLACK 垫成"前面步骤跑完后的值"，
  *         否则它会一直等一个永远不来的编号。
  *         设完把该通道重新武装，这样下一个图案马上能计上。
  */
void APP_Car_SetEventCount(uint8_t evt, uint16_t v);

/**
  * @brief  清零三个事件计数器（每趟路线开始时调一次）
  * @note   计数器在【待机时也会更新】—— 这样你可以推着车走一遍赛道，
  *         在屏幕上看着计数对不对，不用真的开车。
  *         所以每趟路线开始前必须清零，否则会把上一趟/标定时的数带进来。
  */
void APP_Car_ResetEventCounts(void);


/** @brief 把一行状态打印到串口（★ 频率别太高，115200 下一行要几毫秒） */
void APP_Car_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CAR_H */
