#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

/* htim1 定义在 main.c，这里只做引用 */
extern TIM_HandleTypeDef htim1;

/* ============================================================================
 * 双路 H 桥电机（SA8870C）映射
 * ----------------------------------------------------------------------------
 * 左后轮 M2：TIM1_CH1/CH2 → PE9  / PE11
 * 右后轮 M1：TIM1_CH3/CH4 → PE13 / PE14
 *
 * 本板没有独立 DIR 方向脚，方向由“哪一路 PWM 有输出”决定：
 *   IN1=0, IN2=PWM  → 正转
 *   IN1=PWM, IN2=0  → 反转
 *   两路都为 0      → 滑行/停车
 *
 * TIM1 是高级定时器，初始化后必须 __HAL_TIM_MOE_ENABLE()，否则没有波形。
 * PSC=839 / ARR=999 → 168 MHz / 840 / 1000 = 200 Hz，占空比 0~1000。
 * ========================================================================== */
#define MOTOR_TIM              (&htim1)
#define MOTOR_L_CH_A           TIM_CHANNEL_1   /* PE9  */
#define MOTOR_L_CH_B           TIM_CHANNEL_2   /* PE11 */
#define MOTOR_R_CH_A           TIM_CHANNEL_3   /* PE13 */
#define MOTOR_R_CH_B           TIM_CHANNEL_4   /* PE14 */

#define MOTOR_PWM_MAX          1000            /* = ARR + 1 */
#define MOTOR_MIN_PULSE        120             /* 死区：低于它电机克服不了静摩擦 */

/* 板上“电机控制开关”接 PD3（MOTOR_SW，输入上拉）。
 * 它是硬件拨动开关，软件只能“读”，不能输出控制。
 * 实测：开关拨到“允许电机”时 PD3 为低电平（闭合接地），
 *       所以有效电平是 GPIO_PIN_RESET；若换板后相反，改回 GPIO_PIN_SET。 */
#define MOTOR_SW_ON_LEVEL      GPIO_PIN_RESET

/* 方向系数：让“正 pulse = 车前进”。装车后实测，某一侧反了就改 -1。 */
#define MOTOR_L_SIGN           (+1)
#define MOTOR_R_SIGN           (+1)

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1
} MotorId_t;

void    Motor_Init(void);
void    Motor_SetPulse(MotorId_t id, int16_t pulse);  /* -1000 ~ +1000 */
uint8_t Motor_IsSwitchOn(void);                        /* 电机控制开关状态 */
void    Motor_SetBoth(int16_t left, int16_t right);
void    Motor_StopAll(void);
int16_t Motor_GetPulse(MotorId_t id);

#endif /* __MOTOR_H */
