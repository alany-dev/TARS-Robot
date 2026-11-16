#include "pid.h"

/*
 * 标准位置式 PID，三个工程化处理：
 *   ① 微分先行：d 项对“反馈 actual”求导，而不是对误差求导。
 *      设定值突变时不会产生 derivative kick（输出尖峰）。
 *   ② 抗积分饱和：输出已经顶到上限、误差还在往同方向推时，停止累加积分。
 *      否则电机堵转几秒后 integral 会累到很大，松开车时先“冲”一下。
 *   ③ 输出限幅：由调用方给 out_min/out_max（本项目 -1000~+1000）。
 */

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    PID_Reset(p);
}

void PID_Reset(PID_t *p)
{
    p->integral    = 0.0f;
    p->prev_actual = 0.0f;
}

float PID_Update(PID_t *p, float setpoint, float actual, float dt)
{
    float err = setpoint - actual;

    /* 先算“如果这次积分也加进去”的候选值，再决定要不要真的加 */
    float i_candidate = p->integral + err * dt;

    /* 微分先行：注意是对 actual 求导，所以符号是负的 */
    float d_term = 0.0f;
    if (dt > 0.0f && p->kd != 0.0f) {
        d_term = -p->kd * (actual - p->prev_actual) / dt;
    }

    // 当前误差（比例） + 过去累计误差（积分）+ 未来误差的趋势（微分）
    float u = p->kp * err + p->ki * i_candidate + d_term;
    float u_clamped = clampf(u, p->out_min, p->out_max);

    /* 抗饱和：已经饱和，且误差还在把输出往饱和方向推，就不积分 */
    int saturated   = (u > p->out_max) || (u < p->out_min);
    int pushing_out = (u > p->out_max && err > 0.0f) ||
                      (u < p->out_min && err < 0.0f);
    if (!(saturated && pushing_out)) {
        p->integral = i_candidate;
    }

    p->prev_actual = actual;
    return u_clamped;
}
