#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

#include "main.h"
#include "encoder.h"
#include <stdint.h>

/* ============================================================================
 * 双轮速度闭环
 * ----------------------------------------------------------------------------
 * 控制周期：TIM7 PSC=83、ARR=9999
 *   84 MHz / (83+1) / (9999+1) = 100 Hz = 10 ms
 *
 * 数据流：
 *   编码器 CNT → 10 ms 增量 → counts/s → rps → 低通 → PID → Motor_SetPulse
 *
 * 所有计算都在 TIM7 更新中断里完成；中断里绝对不能 printf / HAL_Delay。
 * ========================================================================== */
#define CTRL_PERIOD_S          0.010f      /* 10 ms */

/* 机械参数：★ 必须按实车实测修改 */
#define WHEEL_DIAMETER_M       0.100f      /* 轮径，m（含轮胎） */
#define RPS_LIMIT              2.0f        /* 目标转速上限，转/秒 */

/* PID 起点值：★ 必须整定（先 Kp，再加 Ki，Kd 一般先 0） */
#define PID_KP                 100.0f
#define PID_KI                 0.0f
#define PID_KD                 0.0f
#define PID_OUT_MIN            (-1000.0f)  /* 对应 Motor_SetPulse 的下限 */
#define PID_OUT_MAX            (1000.0f)

/* 测速低通：0.8 = 新值权重 80%。越大响应快、噪声大；越小越平滑、越滞后 */
#define SPEED_LPF_ALPHA        0.8f

/* 心跳保护：超过 300 ms 没有心跳/新指令就停车（接上位机后由协议层喂狗） */
#define HEARTBEAT_TIMEOUT_MS   300U

void    SpeedLoop_Init(void);                 /* 初始化 PID + 启动 TIM7 中断 */
void    SpeedLoop_Tick(void);                 /* 只在 TIM7 更新中断里调用 */
void    SpeedLoop_SetTargetMps(float l, float r);
void    SpeedLoop_SetTargetRps(float l, float r);
void    SpeedLoop_Stop(void);
void    SpeedLoop_Heartbeat(void);            /* 喂狗：本地演示 / 上位机指令时调用 */
void    SpeedLoop_GetActualMps(float *l, float *r);
void    SpeedLoop_GetActualRps(float *l, float *r);
float   SpeedLoop_GetOutput(EncoderId_t id);   /* 调试：本周期 PID 输出 */
uint8_t SpeedLoop_IsFault(void);

#endif /* __SPEED_LOOP_H */
