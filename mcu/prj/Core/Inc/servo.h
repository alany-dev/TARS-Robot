#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdint.h>

/* ============================================================================
 * 硬件映射：板上 J4 座 → PC8 → TIM8_CH3
 * ----------------------------------------------------------------------------
 * J4   : 控制板上 4 个 3 针舵机座之一（针序 1=5V、2=信号、3=GND）
 * PC8  : STM32F407 的引脚，原理图标注 "SDIO0/TIM8CH3/PC8"
 * TIM8 : 高级控制定时器，挂在 APB2，计数时钟 168 MHz
 * CH3  : TIM8 的第 3 个通道，只有它被引到 PC8（CH1/CH2 在 PC6/PC7，被 USART6 占用）
 *
 * 为什么这样配就能得到 50 Hz、1 µs 分辨率：
 *   PSC = 167  → 168 MHz / (167 + 1) = 1 MHz → 每个计数 1 µs
 *   ARR = 19999 → 20000 个计数 = 20 ms = 50 Hz
 *   所以 CCR3 写 1500，就是高电平 1500 µs，不需要再算占空比。
 * ========================================================================== */
#define SERVO_TIM              (&htim8)
#define SERVO_CHANNEL          TIM_CHANNEL_3

/* 电气范围：S825F-180 / LD-1501MG 都是 500~2500 µs 对应 0~180° */
#define SERVO_MIN_US           500U
#define SERVO_MID_US           1500U
#define SERVO_MAX_US           2500U

/* 机械限位：装到转向杯上以后实测再收紧，防止顶死堵转。
 * 25 kg·cm 舵机堵转电流 2~3 A，顶死几秒就会发热、甚至扫齿。 */
#define SERVO_LEFT_LIMIT_US    1150U
#define SERVO_RIGHT_LIMIT_US   1850U

/* 中位机械偏差：舵盘装偏时用它补，单位 µs，范围建议 ±100 */
#define SERVO_OFFSET_US        0

extern TIM_HandleTypeDef htim8;

void     Servo_Init(void);
void     Servo_SetPulseUs(uint16_t us);   /* 直接给脉宽，单位 µs */
void     Servo_SetAngle(float deg);       /* 0~180° */
void     Servo_SetSteer(float cmd);       /* -1.0 ~ +1.0，给运动学层用 */
uint16_t Servo_GetPulseUs(void);

#endif /* __SERVO_H */
