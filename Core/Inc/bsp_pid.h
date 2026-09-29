/**
  ******************************************************************************
  * @file    bsp_pid.h
  * @brief   通用增量式 PID 控制器（带 dt、抗积分饱和、微分低通）
  *
  * 【在工程里的位置】 算法积木。它不认识电机、也不认识传感器，
  *                     只做一件事：把"误差"变成"控制量"。
  *
  *   位置误差 ──► BSP_PID_UpdateErr() ──► 转向量 ──► 左右轮差速
  *   转速误差 ──► BSP_PID_Update()    ──► 占空比 ──► 电机 PWM
  *
  * 【上游】 app_car（转向环）、可选的速度环
  * 【下游】 无。纯计算，不碰任何硬件
  *
  * 【本实现和"教科书 PID"的三点区别，都是被实车逼出来的】
  *
  *   1) 带 dt —— 每个控制周期把"距上次调用了多少秒"传进来。
  *      所以主循环偶尔被 OLED 刷屏拖慢几毫秒时，积分和微分会自动按
  *      实际时间缩放，控制量不会突变。若用固定 dt，一次卡顿就会
  *      在微分项上炸出一个尖峰，车会猛地一抖。
  *
  *   2) 条件积分抗饱和 —— 输出已经顶到限幅、而误差还在往同方向推时，
  *      停止累加积分。否则车在急弯里积分会一路涨到天上，
  *      等出了弯道这股"存量"要花很久才泄掉，表现为出弯画龙。
  *
  *   3) 微分低通 —— 循迹误差是 8 路开关量加权平均出来的阶梯信号，
  *      直接微分会得到一串尖脉冲。这里对 D 项做一阶低通（系数
  *      PID_D_FILTER_ALPHA），让它真正可用。
  *
  * 【正负号约定（★ 和 app_car 的转向方向强相关，改动前先看这里）】
  *   误差为正 → 控制量为正。至于"正的控制量往哪边拐"由调用者决定：
  *   app_car 里是 左轮 = 基础速度 + 转向量，所以
  *   控制量为正 → 左轮快、右轮慢 → 车向右转。
  * ============================================================================
  */
#ifndef __BSP_PID_H
#define __BSP_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/** PID 控制器实例。★ 每个被控量一个实例（左右轮各一个，转向环一个） */
typedef struct {
    /* ------------------------- 可调参数 ------------------------- */
    float kp;           /* 比例增益：误差的直接放大倍数，决定"反应有多急" */
    float ki;           /* 积分增益：消除稳态偏差，循迹里一般给很小或 0 */
    float kd;           /* 微分增益：提前量，决定"看到偏差抬头就刹车"的力度 */
    float i_limit;      /* 积分项限幅（绝对值）。防止积分饱和 */
    float out_limit;    /* 输出限幅（绝对值）。必须符合执行器的物理范围 */
    float dead_zone;    /* 误差死区：|误差| 小于它时按 0 处理，专治直道抖动 */
    float d_alpha;      /* D 项一阶低通系数 0~1：越小越平滑、越大越灵敏 */

    /* ------------------------- 内部状态 ------------------------- */
    float   integral;   /* 积分累计值（已经乘过 ki） */
    float   d_filt;     /* 滤波后的微分值（还未乘 kd） */
    float   prev_err;   /* 上一次的误差，用来算微分 */
    uint8_t started;    /* 0 = 还没跑过第一拍。第一拍不算微分，避免冲击 */

    /* ------------------- 观测量（给屏幕/串口看） ------------------- */
    float p_term;       /* 比例项输出 */
    float i_term;       /* 积分项输出 */
    float d_term;       /* 微分项输出 */
    float out;          /* 限幅后的最终输出 */
} BSP_PID_t;

/**
  * @brief  初始化一个 PID 实例
  * @param  pid 实例指针
  * @param  kp/ki/kd 三个增益
  * @note   限幅、死区、滤波系数会被设成保守默认值：
  *         i_limit = 0（不用积分）、out_limit = 0（不限制）、dead_zone = 0、
  *         d_alpha = PID_D_FILTER_ALPHA。
  *         ★ out_limit = 0 有点特殊，意思是"不做限幅"。
  *           如果要用限幅请显式调 BSP_PID_SetLimits()。
  */
void BSP_PID_Init(BSP_PID_t *pid, float kp, float ki, float kd);

/**
  * @brief  只改三个增益，不动积分/微分状态
  * @note   ★ 这个函数存在的意义就是"运行中调参"：菜单里 Kp 一改，
  *         下一个控制周期就生效，而积分存量不会被清掉 ——
  *         如果改成重置整个 PID，每调一次参数车都会顿一下。
  */
void BSP_PID_SetGains(BSP_PID_t *pid, float kp, float ki, float kd);

/** @brief 设置积分项限幅与输出限幅（都用绝对值，传正数） */
void BSP_PID_SetLimits(BSP_PID_t *pid, float i_limit, float out_limit);

/** @brief 设置误差死区（绝对值）。传 0 = 关闭死区 */
void BSP_PID_SetDeadZone(BSP_PID_t *pid, float dead_zone);

/** @brief 设置 D 项低通系数 0~1（0 = 完全滤掉微分，1 = 不滤波） */
void BSP_PID_SetDFilter(BSP_PID_t *pid, float alpha);

/**
  * @brief  清空内部状态（积分归零、微分历史清零）
  * @note   启停切换、从待机进入运行时调用。运行中不要随便调，
  *         否则积分会被反复清零、失去消差能力
  */
void BSP_PID_Reset(BSP_PID_t *pid);

/**
  * @brief  按"设定值 - 实测值"跑一拍 PID
  * @param  pid      实例
  * @param  setpoint 目标值
  * @param  measure  实测值
  * @param  dt_s     距上次调用的实际时间，单位秒
  * @retval 限幅后的控制量
  */
float BSP_PID_Update(BSP_PID_t *pid, float setpoint, float measure, float dt_s);

/**
  * @brief  直接按误差跑一拍 PID —— 循迹环用这个
  * @param  err 误差（正负号约定见文件头）
  * @param  dt_s 实际时间（秒）
  * @retval 限幅后的控制量
  */
float BSP_PID_UpdateErr(BSP_PID_t *pid, float err, float dt_s);

/** @brief 取回各项分量，用于在 OLED 上显示"到底哪一项在起作用" */
void BSP_PID_GetTerms(const BSP_PID_t *pid, float *p, float *i, float *d, float *out);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PID_H */
