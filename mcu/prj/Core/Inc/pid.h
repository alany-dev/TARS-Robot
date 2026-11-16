#ifndef __PID_H
#define __PID_H

/*
 * 位置式 PID（带抗积分饱和、微分先行）
 *
 * 为什么用位置式而不是增量式：
 *   速度环的输出直接就是电机 PWM（-1000~+1000），位置式直观、好限幅；
 *   增量式更适合“输出是累加量”的场合（如步进电机位置环）。
 */

typedef struct {
    float kp, ki, kd;
    float integral;        /* 积分累加器 */
    float prev_actual;     /* 上次反馈值，用于“微分先行” */
    float out_min, out_max;
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max);
float PID_Update(PID_t *p, float setpoint, float actual, float dt);
void  PID_Reset(PID_t *p);

#endif /* __PID_H */
