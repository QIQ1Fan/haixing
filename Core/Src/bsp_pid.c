/**
  ******************************************************************************
  * @file    bsp_pid.c
  * @brief   通用 PID 实现
  *
  * 【一拍 PID 的完整数据流】
  *
  *   err = setpoint - measure
  *     │
  *     ├─ 死区：|err| < dead_zone 就当 0 ─────────────┐
  *     │                                              │
  *     ├─ P = kp × err ───────────────────────────────┤
  *     │                                              │
  *     ├─ I：integral += ki × err × dt                 │
  *     │      条件积分：输出已饱和且 err 还在往同方向推 │
  *     │      时这一步跳过；然后限幅到 ±i_limit        ├─► out = P + I + D
  *     │                                              │      │
  *     └─ D：d_raw = (err - prev_err) / dt            │      └─► 限幅 ±out_limit
  *            d_filt 一阶低通 → D = kd × d_filt ──────┘            │
  *                                                                 ▼
  *                                                          返回控制量
  *
  * 【为什么所有除法前都要检查 dt】
  *   dt 由调用者从 HAL_GetTick() 算出来。如果两次调用发生在同一毫秒内，
  *   dt 就是 0，直接做除法会得到 inf —— 而 inf 参与运算后会污染
  *   integral 和 d_filt，之后整个 PID 的输出永远是 nan，车彻底失控。
  *   所以这里统一把过小的 dt 兜底成 1ms。
  *
  * 【为什么不用 fabsf()】
  *   本文件刻意不依赖 math.h：Cortex-M3 没有硬件浮点，
  *   库函数会带来额外的运行时依赖，而绝对值只要一行三元表达式。
  *   少一个依赖，换工具链时少一个坑。
  ******************************************************************************
  */
#include "bsp_pid.h"

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/** 浮点绝对值。不用 fabsf() 的理由见文件头 */
static float pid_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/** 对称限幅：把 v 夹到 [-limit, +limit]。limit <= 0 表示"不限幅" */
static float pid_clamp(float v, float limit)
{
    if (limit <= 0.0f) {
        return v;
    }
    if (v > limit) {
        return limit;
    }
    if (v < -limit) {
        return -limit;
    }
    return v;
}

/** 把时间步长兜底到合法范围，杜绝除零 */
static float pid_safe_dt(float dt_s)
{
    if (dt_s < 0.0005f) {
        return 0.001f;      /* 1ms 兜底，理由见文件头 */
    }
    if (dt_s > 0.5f) {
        return 0.5f;        /* 超过半秒的间隔已经不算"控制周期"了，防积分暴走 */
    }
    return dt_s;
}

/* ==========================================================================
 * 配置接口
 * ========================================================================== */

void BSP_PID_Init(BSP_PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /* 保守默认值：
       i_limit = 0 → 不限幅（因为 ki 通常也是 0，不会有积分）
       out_limit = 0 → 不限幅，由调用者按执行器量程设置
       dead_zone = 0 → 不设死区
       d_alpha 取工程默认的低通系数 */
    pid->i_limit   = 0.0f;
    pid->out_limit = 0.0f;
    pid->dead_zone = 0.0f;
    pid->d_alpha   = PID_D_FILTER_ALPHA;

    BSP_PID_Reset(pid);
}

void BSP_PID_SetGains(BSP_PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }

    /* ★ 故意不去动 integral / d_filt / prev_err。
       运行中调参时，如果顺手把状态清了，每次按键车都会顿一下，
       根本没法边跑边看效果。 */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void BSP_PID_SetLimits(BSP_PID_t *pid, float i_limit, float out_limit)
{
    if (pid == NULL) {
        return;
    }
    pid->i_limit   = (i_limit   < 0.0f) ? -i_limit   : i_limit;
    pid->out_limit = (out_limit < 0.0f) ? -out_limit : out_limit;
}

void BSP_PID_SetDeadZone(BSP_PID_t *pid, float dead_zone)
{
    if (pid == NULL) {
        return;
    }
    pid->dead_zone = (dead_zone < 0.0f) ? -dead_zone : dead_zone;
}

void BSP_PID_SetDFilter(BSP_PID_t *pid, float alpha)
{
    if (pid == NULL) {
        return;
    }
    if (alpha < 0.0f) { alpha = 0.0f; }
    if (alpha > 1.0f) { alpha = 1.0f; }
    pid->d_alpha = alpha;
}

void BSP_PID_Reset(BSP_PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->d_filt   = 0.0f;
    pid->prev_err = 0.0f;
    pid->started  = 0u;
    pid->p_term   = 0.0f;
    pid->i_term   = 0.0f;
    pid->d_term   = 0.0f;
    pid->out      = 0.0f;
}

/* ==========================================================================
 * 计算核心
 * ========================================================================== */

float BSP_PID_Update(BSP_PID_t *pid, float setpoint, float measure, float dt_s)
{
    return BSP_PID_UpdateErr(pid, setpoint - measure, dt_s);
}

float BSP_PID_UpdateErr(BSP_PID_t *pid, float err, float dt_s)
{
    float p, d_raw, i_candidate, out_raw, out;

    if (pid == NULL) {
        return 0.0f;
    }

    dt_s = pid_safe_dt(dt_s);

    /* ---- 1. 死区 ----
       误差小于死区就当成"已经在中心线上"。两个好处：
         · 直道上探头在两条线之间来回跳时，输出不会跟着抖
         · 省掉电机对微小扰动的无谓反应，行驶更安静 */
    if (pid->dead_zone > 0.0f) {
        if (pid_abs(err) < pid->dead_zone) {
            err = 0.0f;
        }
    }

    /* ---- 2. 比例项 ---- */
    p = pid->kp * err;

    /* ---- 3. 微分项（带一阶低通）----
       ★ 第一拍不算微分：此时 prev_err 还是初值 0，直接算会得到一个
         "凭空冒出来"的巨大微分，让车一启动就猛地一拐。 */
    if (pid->started != 0u) {
        d_raw = (err - pid->prev_err) / dt_s;
    } else {
        d_raw = 0.0f;
    }
    pid->prev_err = err;
    pid->started  = 1u;

    /* 一阶低通：新值按 alpha 权重融进来。
       alpha 越小越平滑（但滞后越大）。误差是阶梯信号，这一步很关键 */
    pid->d_filt += pid->d_alpha * (d_raw - pid->d_filt);

    /* ---- 4. 积分项（条件积分抗饱和）----
       先"预演"一下：如果现在把新的积分加进去，输出会不会溢出？
       如果会溢出、而且误差的方向还在把输出往同一个方向推，
       那就别加 —— 这就是条件积分。 */
    i_candidate = pid->integral + pid->ki * err * dt_s;
    i_candidate = pid_clamp(i_candidate, pid->i_limit);

    out_raw = p + i_candidate + pid->kd * pid->d_filt;

    if (pid->out_limit > 0.0f) {
        uint8_t sat_high = (out_raw > pid->out_limit);
        uint8_t sat_low  = (out_raw < -pid->out_limit);
        uint8_t push_up  = (pid->ki * err) > 0.0f;

        if ((sat_high && push_up) || (sat_low && !push_up)) {
            /* 输出已经顶死、误差还在往外推：这一拍不更新积分 */
            i_candidate = pid->integral;
        }
    }

    pid->integral = i_candidate;

    /* ---- 5. 合成与限幅 ---- */
    pid->p_term = p;
    pid->i_term = pid->integral;

    /* ★ D 项单独限幅，理由见 bsp_config.h 里 PID_D_LIMIT_RATIO 的详细说明：
       误差跳一格就能让 D 项爆表（采样周期只有 10ms，变化率被放大 100 倍），
       一旦爆表，整个输出就被 D 项一个人顶到限幅，P 项完全不起作用 ——
       控制器退化成"看见误差跳变就往反方向打死"。
       夹到输出的一半之后，D 只负责阻尼，方向仍然由 P 项说了算。 */
    pid->d_term = pid->kd * pid->d_filt;
    if (pid->out_limit > 0.0f) {
        float d_max = pid->out_limit * PID_D_LIMIT_RATIO;

        if (pid->d_term > d_max) {
            pid->d_term = d_max;
        } else if (pid->d_term < -d_max) {
            pid->d_term = -d_max;
        }
    }

    out = pid->p_term + pid->i_term + pid->d_term;
    out = pid_clamp(out, pid->out_limit);

    pid->out = out;
    return out;
}

void BSP_PID_GetTerms(const BSP_PID_t *pid, float *p, float *i, float *d, float *out)
{
    if (pid == NULL) {
        return;
    }
    /* 允许只取其中几项，传 NULL 的就不写 */
    if (p   != NULL) { *p   = pid->p_term; }
    if (i   != NULL) { *i   = pid->i_term; }
    if (d   != NULL) { *d   = pid->d_term; }
    if (out != NULL) { *out = pid->out;    }
}
