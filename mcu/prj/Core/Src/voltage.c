#include "voltage.h"

/* F407 出厂 VREFINT 校准值（30°C、VDDA=3.3V 时的 12 位读数） */
#define VREFINT_CAL_ADDR   ((uint16_t *)0x1FFF7A2A)
#define TEMP_V25           0.76f
#define TEMP_SLOPE         0.0025f

/* hadc1 定义在 main.c */
extern ADC_HandleTypeDef hadc1;

/* 3 通道 × 8 个样本 = 24 个 halfword */
static uint16_t s_dma_buf[24];
static float    s_vbat = 0.0f;
static float    s_vdda = 3.3f;
static float    s_temp = 25.0f;
static uint8_t  s_low_latched = 0;

void Voltage_Init(void)
{
    /* CubeMX 里 ADC1 已经配置成 3 通道扫描：
     *   Rank1 = IN8 (PB0) / Rank2 = VREFINT / Rank3 = TEMPSENSOR
     *   ScanConvMode = Enable、NbrOfConversion = 3
     *   DMAContinuousRequests = Enable（DDS=1）
     * 所以这里只需要启动 DMA，不要再重复 HAL_ADC_Init/ConfigChannel。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma_buf, 24) != HAL_OK) {
        Error_Handler();
    }
}

void Voltage_Task(void)
{
    uint32_t sum_vbat = 0, sum_vref = 0, sum_temp = 0;

    for (int i = 0; i < 8; ++i) {
        sum_vbat += s_dma_buf[i * 3 + 0];
        sum_vref += s_dma_buf[i * 3 + 1];
        sum_temp += s_dma_buf[i * 3 + 2];
    }

    uint16_t raw_vbat = (uint16_t)(sum_vbat / 8);
    uint16_t raw_vref = (uint16_t)(sum_vref / 8);
    uint16_t raw_temp = (uint16_t)(sum_temp / 8);

    if (raw_vref == 0) {
        return;                     /* DMA 还没填满，跳过 */
    }

    /* 用 VREFINT 反推真实 VDDA；校准值无效时退回 1.21 V 典型值 */
    uint16_t cal = *VREFINT_CAL_ADDR;
    float vdda;
    if (cal == 0 || cal == 0xFFFF) {
        vdda = 1.21f * ADC_FULL_SCALE / (float)raw_vref;
    } else {
        vdda = 3.3f * (float)cal / (float)raw_vref;
    }

    float v_now = (float)raw_vbat * vdda / ADC_FULL_SCALE * VBAT_DIVIDER;

    /* 温度：VSENSE(V) → (VSENSE - 0.76) / 0.0025 + 25 */
    float vsense = (float)raw_temp * vdda / ADC_FULL_SCALE;
    float t_now  = (vsense - TEMP_V25) / TEMP_SLOPE + 25.0f;

    /* 第一帧直接赋值，避免上电从 0 爬 */
    if (s_vbat < 1.0f) {
        s_vbat = v_now;
    } else {
        s_vbat = 0.9f * s_vbat + 0.1f * v_now;
    }
    s_vdda = 0.9f * s_vdda + 0.1f * vdda;
    s_temp = 0.9f * s_temp + 0.1f * t_now;

    /* 低压锁存 + 滞回 */
    if (!s_low_latched && s_vbat < VBAT_LOW) {
        s_low_latched = 1;
    } else if (s_low_latched && s_vbat > VBAT_LOW + VBAT_HYST) {
        s_low_latched = 0;
    }
}

float Voltage_Get(void)
{
    return s_vbat;
}

float Voltage_GetVdda(void)
{
    return s_vdda;
}

float Voltage_GetTemp(void)
{
    return s_temp;
}

uint8_t Voltage_IsLow(void)
{
    return s_low_latched;
}

uint8_t Voltage_IsCritical(void)
{
    return (s_vbat > 1.0f && s_vbat < VBAT_CRITICAL) ? 1U : 0U;
}
