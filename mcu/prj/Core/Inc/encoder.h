#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include <stdint.h>

/* ============================================================================
 * 左右轮编码器映射（M1 = 右后轮，M2 = 左后轮）
 * ----------------------------------------------------------------------------
 * 右轮 M1：TIM5_CH1/CH2 → PA0 / PA1   （CubeMX Label：ENC_R_A / ENC_R_B）
 * 左轮 M2：TIM2_CH1/CH2 → PA15 / PB3  （CubeMX Label：ENC_L_A / ENC_L_B）
 *
 * 为什么用 TIM2/TIM5：
 *   它们是 F407 上仅有的两个 32 位定时器，ARR 可设 0xFFFFFFFF。
 *   1024 线编码器 4 倍频后每转 4096 计数，乘减速比十几万/圈；
 *   16 位定时器会频繁溢出，32 位可以直接“回绕差值”，不需要溢出中断。
 *
 * 注意：CubeMX 里 TIM2 的 Encoder Mode 必须是 “TI1 and TI2”（TI12）。
 *       如果选成 “TI1 only”，只有 2 倍频，左轮分辨率会只有右轮的一半。
 * ========================================================================== */
#define ENC_L_TIM              (&htim2)   /* 左轮 M2：PA15/PB3 */
#define ENC_R_TIM              (&htim5)   /* 右轮 M1：PA0/PA1  */

/* 车轮转一圈的计数 = 编码器线数 × 4（四倍频）× 减速比。
 * 1024 线、减速比 30 时 = 122880。
 * 实测（2026-11-14，手转一圈三次）：
 *   右轮前进：+123107  → 123107 / 4096 = 30.05，误差 0.18%，直接用 122880
 *   左轮前进：-61419   → 当时 TIM2 还是 TI1（2 倍频），改成 TI12 后应约 123k */
#define ENC_COUNTS_PER_WHEEL_REV   122880.0f

/* 方向系数：让“前进”时累计值增加。
 * 实测：左轮前进计数为负、右轮为正（A/B 相序相反），所以左轮取 -1。 */
#define ENC_L_SIGN             (-1)
#define ENC_R_SIGN             (+1)

typedef enum {
    ENC_LEFT  = 0,
    ENC_RIGHT = 1
} EncoderId_t;

void     Encoder_Init(void);
int32_t  Encoder_ReadDelta(EncoderId_t id);   /* 自上次调用以来的计数增量 */
int64_t  Encoder_GetTotal(EncoderId_t id);    /* 累计位置，可正可负 */
uint32_t Encoder_GetRaw(EncoderId_t id);      /* 直接读 CNT，调试点 */
void     Encoder_Reset(EncoderId_t id);       /* 清零 CNT / 累计值 */

#endif /* __ENCODER_H */
