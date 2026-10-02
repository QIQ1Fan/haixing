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
    uint16_t lost_cnt;      /* ★ "全丢黑线"的累计次数 —— 也就是"到了第几个地形" */
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
  * @brief  复位转向环 PID（清积分与微分历史）
  * @note   给路线模块用：盲走穿过地形之后重新压上线的那一刻，
  *         PID 里还留着"进地形之前"的历史，
  *         不清理的话恢复循线的第一拍会猛打一下方向。
  */
void APP_Car_ResetPid(void);

/**
  * @brief  ★ 取"全丢黑线"的累计次数
  * @retval 从上次清零以来，探头阵列【完全看不到黑线】发生了几次
  *
  * 【它为什么有用 —— 用地形编号代替时间/图案】
  *   赛道上的地形（石块 / 草地 / 减速带 / 台阶）都是平铺上去的板子，
  *   会把下面的黑线盖住 —— 每经过一块地形，就必然产生【一次全丢线】。
  *   这些地形在固定赛道上的先后顺序是已知的，于是：
  *
  *       第 1 次全丢线 = 到了第 1 个地形
  *       第 2 次全丢线 = 到了第 2 个地形   ...
  *
  *   路线脚本就能写成"循线 → 等到第 N 次丢线 → 换个跑法过 → 接着循线"，
  *   全程都处在循迹状态，只是在特定编号时用特定方式跑。
  *   ★ 它和时间无关，所以电池掉电压也不影响判断（比时间脚本可靠）。
  */
uint16_t APP_Car_GetLostCount(void);

/**
  * @brief  清零丢线计数（每趟路线开始时调一次）
  * @note   ★ APP_Car_Start() 不会清零它 —— 路线里每一段循线都会调 Start，
  *         如果 Start 顺手清零，计数在路线中途就归零了，判断全乱
  */
void APP_Car_ResetLostCount(void);


/** @brief 把一行状态打印到串口（★ 频率别太高，115200 下一行要几毫秒） */
void APP_Car_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CAR_H */
