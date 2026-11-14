#include "servo.h"

/*
 * 舵机驱动：TIM8_CH3 (PC8) 硬件 PWM，50 Hz，1 计数 = 1 µs
 *
 * 底层链路（从寄存器到引脚）：
 *   TIM8 计数器 CNT 每 1 µs 加 1，从 0 数到 ARR=19999 后归零
 *   → 比较器把 CNT 与 CCR3 比较：CNT < CCR3 输出有效电平（高）
 *   → 通道 3 输出使能（CCER.CC3E）
 *   → 高级定时器总开关（BDTR.MOE）
 *   → PC8 的复用功能 AF3（GPIO_AF3_TIM8）
 *   → J4 座子的第 2 脚（信号）
 */

static uint16_t s_pulse_us = SERVO_MID_US;

/* 把 v 夹在 [lo, hi] 之间；用 int32_t 是为了能接收负的中间值 */
static uint16_t clamp_u16(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

void Servo_Init(void)
{
    /* ① 高级定时器（TIM1/TIM8）除了每个通道的 CCxE，还有一个“总闸” MOE
     *    （Main Output Enable，在 BDTR 寄存器里）。
     *    只调用 HAL_TIM_PWM_Start 而不开 MOE，会出现“CCR 改了、示波器没波形”
     *    的经典现象。CubeMX 不会替你写这一句。 */
    __HAL_TIM_MOE_ENABLE(SERVO_TIM);
    __HAL_TIM_ENABLE(SERVO_TIM);

    /* ② 启动通道 3 的 PWM 输出：使能 CC3E，计数器开始跑。
     *    只需要调用一次，之后改角度只改 CCR3，不用反复 Start。 */
    if (HAL_TIM_PWM_Start(SERVO_TIM, SERVO_CHANNEL) != HAL_OK) {
        Error_Handler();
    }

    /* ③ 上电先回中位：避免舵机记住上次角度，一上电猛地打向一边 */
    Servo_SetPulseUs(SERVO_MID_US);
}

void Servo_SetPulseUs(uint16_t us)
{
    /* 第一级：电气范围 500~2500 µs，超出会顶到舵机内部机械限位 */
    uint16_t p = clamp_u16((int32_t)us + SERVO_OFFSET_US,
                           SERVO_MIN_US, SERVO_MAX_US);

    /* 第二级：实车机械限位（转向杯/连杆），防止堵转烧舵机 */
    p = clamp_u16(p, SERVO_LEFT_LIMIT_US, SERVO_RIGHT_LIMIT_US);

    s_pulse_us = p;

    /* 因为 PSC=167，1 个计数正好 1 µs，所以这里 CCR 的数值就是脉宽微秒数。
     * CCR3 配了“输出比较预装载”（OCPreload），写入先落到影子寄存器，
     * 下一个更新事件才搬到工作寄存器 —— 改脉宽时不会切出半截脉冲。 */
    __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_CHANNEL, p);
}

void Servo_SetAngle(float deg)
{
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;

    /* 500 µs → 0°，2500 µs → 180°，线性映射：2000 µs / 180° */
    float us = SERVO_MIN_US + deg * (2000.0f / 180.0f);
    Servo_SetPulseUs((uint16_t)(us + 0.5f));   /* +0.5 做四舍五入 */
}

void Servo_SetSteer(float cmd)
{
    /* 给运动学层的归一化接口：-1 = 左满舵，0 = 中位，+1 = 右满舵。
     * 左右行程可能不对称，所以分别按两侧的机械限位映射。 */
    if (cmd >  1.0f) cmd =  1.0f;
    if (cmd < -1.0f) cmd = -1.0f;

    float span = (cmd >= 0.0f)
               ? (float)(SERVO_RIGHT_LIMIT_US - SERVO_MID_US)
               : (float)(SERVO_MID_US - SERVO_LEFT_LIMIT_US);

    Servo_SetPulseUs((uint16_t)(SERVO_MID_US + cmd * span + 0.5f));
}

uint16_t Servo_GetPulseUs(void)
{
    return s_pulse_us;
}
