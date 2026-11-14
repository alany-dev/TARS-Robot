#include "encoder.h"

/*
 * 编码器驱动：TIM2/TIM5 硬件编码器接口 + 32 位回绕差值法
 *
 * 硬件链路：
 *   PA0/PA1  → TIM5_CH1/CH2 → 右轮 M1 编码器 A/B
 *   PA15/PB3 → TIM2_CH1/CH2 → 左轮 M2 编码器 A/B
 *
 * 编码器模式下 CNT 不是由内部时钟驱动，而是由 A/B 两相的边沿驱动：
 *   TI12（Encoder Mode 3）在 A、B 的每个边沿都计数 → 一个 A 周期 4 个计数（4 倍频）
 *   DIR 位由硬件根据相位关系自动更新，所以 CNT 会加也会减。
 *
 * 本驱动不开启任何中断：TIM2/TIM5 是 32 位，直接用“无符号相减再转有符号”
 * 处理回绕，CPU 只在读取时参与。
 */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

typedef struct {
    TIM_HandleTypeDef *htim;   /* 该轮对应的定时器 */
    uint32_t prev_cnt;         /* 上次读到的 CNT */
    int64_t  total;            /* 累计计数，可正可负 */
} Encoder_t;

static Encoder_t s_enc[2] = {
    { ENC_L_TIM, 0, 0 },       /* ENC_LEFT  = 左轮 M2 / TIM2 */
    { ENC_R_TIM, 0, 0 }        /* ENC_RIGHT = 右轮 M1 / TIM5 */
};

void Encoder_Init(void)
{
    for (int i = 0; i < 2; ++i) {
        /* 先清零计数器，避免上电残留的随机值被当成第一次增量 */
        __HAL_TIM_SET_COUNTER(s_enc[i].htim, 0);
        __HAL_TIM_CLEAR_FLAG(s_enc[i].htim, TIM_FLAG_UPDATE);

        /* TIM_CHANNEL_ALL = 同时启用 CH1/CH2 两路编码器输入。
         * 这一步会把 CR1.CEN 置 1，之后 CNT 就跟着 A/B 相走了。 */
        if (HAL_TIM_Encoder_Start(s_enc[i].htim, TIM_CHANNEL_ALL) != HAL_OK) {
            Error_Handler();
        }

        s_enc[i].prev_cnt = 0;
        s_enc[i].total    = 0;
    }
}

int32_t Encoder_ReadDelta(EncoderId_t id)
{
    Encoder_t *e = &s_enc[id];
    uint32_t now = __HAL_TIM_GET_COUNTER(e->htim);

    /* 回绕差值法的核心：
     *   now、prev 都是 uint32_t，相减按模 2^32 进行；
     *   再强制转换成 int32_t，二进制补码会把它解释成“有符号增量”。
     *   例：prev=0xFFFFFF00，now=0x00000100
     *       无符号相减 = 0x00000200 → int32 = +512（正确）
     *   前提：两次采样之间真实增量小于 2^31（10~200 ms 采样远远满足）。 */
    int32_t delta = (int32_t)(now - e->prev_cnt);
    e->prev_cnt = now;

    /* 应用方向系数，让“车轮前进”时返回正值。
     * 实测（2026-11-14）：左轮前进计数为负、右轮为正，所以 ENC_L_SIGN = -1。 */
    int32_t signed_delta = (id == ENC_LEFT)
                         ? (int32_t)(delta * ENC_L_SIGN)
                         : (int32_t)(delta * ENC_R_SIGN);

    e->total += signed_delta;
    return signed_delta;
}

int64_t Encoder_GetTotal(EncoderId_t id)
{
    return s_enc[id].total;
}

uint32_t Encoder_GetRaw(EncoderId_t id)
{
    return __HAL_TIM_GET_COUNTER(s_enc[id].htim);
}

void Encoder_Reset(EncoderId_t id)
{
    __HAL_TIM_SET_COUNTER(s_enc[id].htim, 0);
    s_enc[id].prev_cnt = 0;
    s_enc[id].total    = 0;
}
