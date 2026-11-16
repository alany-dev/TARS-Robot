#include "speed_loop.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include <math.h>

/* htim7 定义在 main.c，这里只引用 */
extern TIM_HandleTypeDef htim7;

/* 每 10 ms 更新一次的目标/反馈/PID 状态 */
typedef struct {
    PID_t   pid;          /* 该轮的 PID 控制器 */
    float   tps_f;        /* 低通后的 counts/s */
    float   rps;          /* 反馈转速，转/秒 */
    float   target_rps;   /* 目标转速，转/秒 */
    float   u;            /* 本周期最终输出（含死区前馈），调试用 */
    uint8_t zero_cnt;     /* 连续“有目标但计数为 0”的周期数，用于断线/堵转检测 */
} Wheel_t;

static Wheel_t s_wheel[2];                 /* 0=左轮，1=右轮 */
static volatile uint32_t s_no_cmd_ticks = 0;
static volatile uint8_t  s_fault = 0;

static float limit_rps(float rps)
{
    if (rps >  RPS_LIMIT) rps =  RPS_LIMIT;
    if (rps < -RPS_LIMIT) rps = -RPS_LIMIT;
    return rps;
}

void SpeedLoop_Init(void)
{
    for (int i = 0; i < 2; ++i) {
        PID_Init(&s_wheel[i].pid, PID_KP, PID_KI, PID_KD,
                 PID_OUT_MIN, PID_OUT_MAX);
        s_wheel[i].tps_f      = 0.0f;
        s_wheel[i].rps        = 0.0f;
        s_wheel[i].target_rps = 0.0f;
        s_wheel[i].zero_cnt   = 0;
    }

    s_no_cmd_ticks = 0;
    s_fault        = 0;

    Motor_StopAll();

    /* CubeMX 只生成 TIM7 的配置，不会替你启动。
     * 这一句才会真正开更新中断，之后每 10 ms 进一次 HAL_TIM_PeriodElapsedCallback。 */
    HAL_TIM_Base_Start_IT(&htim7);
}

void SpeedLoop_SetTargetRps(float l, float r)
{
    s_wheel[0].target_rps = limit_rps(l);
    s_wheel[1].target_rps = limit_rps(r);

    /* 收到新目标 = 系统还活着：清心跳计数和故障标志，
     * 这样按键/上位机重新给速度时能从故障停车状态恢复。 */
    s_no_cmd_ticks = 0;
    s_fault        = 0;

    s_wheel[0].zero_cnt = 0;
    s_wheel[1].zero_cnt = 0;
}

void SpeedLoop_SetTargetMps(float l, float r)
{
    /* v = rps × π × D  →  rps = v / (πD) */
    const float k = 1.0f / (3.14159265f * WHEEL_DIAMETER_M);
    SpeedLoop_SetTargetRps(l * k, r * k);
}

void SpeedLoop_Stop(void)
{
    s_wheel[0].target_rps = 0.0f;
    s_wheel[1].target_rps = 0.0f;
    s_wheel[0].zero_cnt   = 0;
    s_wheel[1].zero_cnt   = 0;

    PID_Reset(&s_wheel[0].pid);
    PID_Reset(&s_wheel[1].pid);

    Motor_StopAll();
}

void SpeedLoop_Heartbeat(void)
{
    s_no_cmd_ticks = 0;
}

void SpeedLoop_GetActualRps(float *l, float *r)
{
    if (l) *l = s_wheel[0].rps;
    if (r) *r = s_wheel[1].rps;
}

void SpeedLoop_GetActualMps(float *l, float *r)
{
    const float k = 3.14159265f * WHEEL_DIAMETER_M;
    if (l) *l = s_wheel[0].rps * k;
    if (r) *r = s_wheel[1].rps * k;
}

uint8_t SpeedLoop_IsFault(void)
{
    return s_fault;
}

float SpeedLoop_GetOutput(EncoderId_t id)
{
    return s_wheel[id].u;
}

/* 由 TIM7 更新中断调用，每 10 ms 一次 */
void SpeedLoop_Tick(void)
{
    /* 硬件“电机控制开关”关闭时：立即停车并清 PID。
     * 如果只让 Motor_SetPulse 内部强制 0，PID 的积分仍会继续累加，
     * 开关一打开就会猛冲一下（积分饱和的典型现象）。 */
    if (!Motor_IsSwitchOn()) {
        SpeedLoop_Stop();
        s_no_cmd_ticks = 0;
        return;
    }

    for (uint8_t i = 0; i < 2; ++i) {
        Wheel_t *w = &s_wheel[i];
        EncoderId_t eid = (i == 0) ? ENC_LEFT : ENC_RIGHT;

        /* Encoder_ReadDelta 已经应用过 ENC_L_SIGN / ENC_R_SIGN，
         * 返回“前进为正”的增量，这里不要再乘一次符号。 */
        int32_t delta = Encoder_ReadDelta(eid);

        /* 10 ms 内的计数增量 → counts/s */
        float tps_new = (float)delta / CTRL_PERIOD_S;

        /* 一阶低通：抑制编码器量化噪声和机械抖动 */
        w->tps_f = SPEED_LPF_ALPHA * tps_new + (1.0f - SPEED_LPF_ALPHA) * w->tps_f;

        /* counts/s → 转/秒 */
        w->rps = w->tps_f / ENC_COUNTS_PER_WHEEL_REV;

        /* PID 输出（-1000~+1000） */
        float u = PID_Update(&w->pid, w->target_rps, w->rps, CTRL_PERIOD_S);

        /* 死区前馈：目标非零时叠加一个最小启动量。
         * 0.15 m/s ≈ 0.48 rps，Kp=250 时 P 只有 120，
         * 正好卡在 MOTOR_MIN_PULSE 死区边缘，电机会一直不转。 */
        if (w->target_rps > 0.01f) {
            u += (float)MOTOR_MIN_PULSE;
        } else if (w->target_rps < -0.01f) {
            u -= (float)MOTOR_MIN_PULSE;
        }
        if (u > PID_OUT_MAX) u = PID_OUT_MAX;
        if (u < PID_OUT_MIN) u = PID_OUT_MIN;
        w->u = u;

        /* 反馈丢失检测：只在本周期输出明显大于死区、但计数仍为 0 时才计数，
         * 并且要持续 1 s，避免启动阶段被死区误判成故障。 */
        if (w->target_rps != 0.0f && delta == 0 &&
            fabsf(u) > (float)(MOTOR_MIN_PULSE + 100)) {
            if (w->zero_cnt < 255) w->zero_cnt++;
            if (w->zero_cnt > 100) s_fault = 1;
        } else {
            w->zero_cnt = 0;
        }

        Motor_SetPulse((MotorId_t)i, (int16_t)u);
    }

    /* 心跳/故障保护：超时或故障 → 停车。
     * 本地演示时由 main 每 100 ms 喂一次心跳；接上位机后由协议层喂。 */
    if (++s_no_cmd_ticks > (HEARTBEAT_TIMEOUT_MS / 10U) || s_fault) {
        SpeedLoop_Stop();
        s_no_cmd_ticks = 0;
    }
}

/* HAL 的 weak 回调：所有定时器更新中断都会进这里，需要按 Instance 区分 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        SpeedLoop_Tick();
    }
}
