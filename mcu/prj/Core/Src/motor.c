#include "motor.h"

/*
 * H 桥电机驱动：TIM1 四通道 PWM，200 Hz，占空比 0~1000
 *
 * 硬件链路：
 *   CCR3/CCR4 → PE13/PE14 → M1 的 SA8870C IN1/IN2 → 左电机
 *   CCR1/CCR2 → PE9 /PE11 → M2 的 SA8870C IN1/IN2 → 右电机
 *
 * 方向控制不是靠 GPIO，而是靠“哪一路有 PWM”：
 *   正转：A=0，B=pulse
 *   反转：B=0，A=|pulse|
 *   停车：A=0，B=0
 */

typedef struct {
    uint32_t ch_a;      /* 该电机的 IN1 通道 */
    uint32_t ch_b;      /* 该电机的 IN2 通道 */
    int16_t  pulse;     /* 当前输出，用于换向检测 */
} Motor_t;

static Motor_t s_motor[2] = {
    { MOTOR_L_CH_A, MOTOR_L_CH_B, 0 },   /* 左轮 M2（PE9/PE11） */
    { MOTOR_R_CH_A, MOTOR_R_CH_B, 0 }    /* 右轮 M1（PE13/PE14） */
};

/* 把目标值夹到 ±MOTOR_PWM_MAX，防止 CCR 超过 ARR 导致 100% 常开 */
static int16_t clamp_pulse(int32_t p)
{
    if (p >  MOTOR_PWM_MAX) p =  MOTOR_PWM_MAX;
    if (p < -MOTOR_PWM_MAX) p = -MOTOR_PWM_MAX;
    return (int16_t)p;
}

void Motor_Init(void)
{
    /* TIM1 和 TIM8 一样是高级定时器，除了 4 个通道的 CCxE，
     * 还有 BDTR.MOE 这个总输出开关，漏了它示波器上一点波形都没有。 */
    __HAL_TIM_MOE_ENABLE(MOTOR_TIM);
    __HAL_TIM_ENABLE(MOTOR_TIM);

    /* 4 个通道各启动一次；之后只改 CCR，不再重复 Start */
    HAL_TIM_PWM_Start(MOTOR_TIM, MOTOR_L_CH_A);
    HAL_TIM_PWM_Start(MOTOR_TIM, MOTOR_L_CH_B);
    HAL_TIM_PWM_Start(MOTOR_TIM, MOTOR_R_CH_A);
    HAL_TIM_PWM_Start(MOTOR_TIM, MOTOR_R_CH_B);

    /* 上电第一件事：确保电机不动。
     * 如果这里不写，CCR 可能保留 CubeMX 里的初值（本项目是 0，但仍建议显式停）。 */
    Motor_StopAll();
}

/* 读板上“电机控制开关”（PD3）。开关关闭时返回 0。
 * 注意：这是输入状态，不是软件输出；开关本身是硬件拨动的。 */
uint8_t Motor_IsSwitchOn(void)
{
    return (HAL_GPIO_ReadPin(MOTOR_SW_GPIO_Port, MOTOR_SW_Pin) == MOTOR_SW_ON_LEVEL)
           ? 1U : 0U;
}

void Motor_SetPulse(MotorId_t id, int16_t pulse)
{
    Motor_t *m = &s_motor[id];
    int16_t p = clamp_pulse(pulse);

    /* 硬件“电机控制开关”关闭时，无论上层给什么速度都强制停车。
     * 这一层放在死区/换向之前，保证开关一关就立刻两路 CCR=0。 */
    if (!Motor_IsSwitchOn()) {
        p = 0;
    }

    /* 死区：占空比太小时电机只会“嗡嗡”响、发热，不会转。
     * 这个阈值要实测（从 0 慢慢加，记下刚好能转的值）。 */
    if (p > 0 && p <  MOTOR_MIN_PULSE) p = 0;
    if (p < 0 && p > -MOTOR_MIN_PULSE) p = 0;

    /* 方向约定：正 pulse = 车前进。左右轮镜像安装时用系数修正。 */
    if (id == MOTOR_LEFT) {
        p = (int16_t)(p * MOTOR_L_SIGN);
    } else {
        p = (int16_t)(p * MOTOR_R_SIGN);
    }

    /* 换向瞬间先把两路都清零，避免出现“两路同时有输出”的中间状态。
     * 虽然 SA8870C 内部有防直通逻辑，软件上保守一点更安全。 */
    if ((m->pulse > 0 && p < 0) || (m->pulse < 0 && p > 0)) {
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_a, 0);
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_b, 0);
    }

    if (p > 0) {
        /* 正转：IN1=0，IN2=PWM */
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_a, 0);
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_b, (uint32_t)p);
    } else if (p < 0) {
        /* 反转：IN2=0，IN1=|PWM| */
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_b, 0);
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_a, (uint32_t)(-p));
    } else {
        /* 停车：两路都 0，电机滑行 */
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_a, 0);
        __HAL_TIM_SET_COMPARE(MOTOR_TIM, m->ch_b, 0);
    }

    m->pulse = p;
}

void Motor_SetBoth(int16_t left, int16_t right)
{
    Motor_SetPulse(MOTOR_LEFT, left);
    Motor_SetPulse(MOTOR_RIGHT, right);
}

void Motor_StopAll(void)
{
    Motor_SetBoth(0, 0);
}

int16_t Motor_GetPulse(MotorId_t id)
{
    return s_motor[id].pulse;
}
