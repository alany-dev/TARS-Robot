#ifndef __VOLTAGE_H
#define __VOLTAGE_H

#include "main.h"
#include <stdint.h>

/* ============================================================================
 * 电池电压检测（PB0 / ADC1_IN8，100k+10k 分压）
 * ----------------------------------------------------------------------------
 * V_adc = V_bat / 11；12 位 ADC，VREF=3.3 V
 * 默认按 3S 锂电：10.2 V 低压、9.6 V 严重低压
 * 2S 电池请把 VBAT_LOW / VBAT_CRITICAL 改成 6.8 / 6.4
 * ========================================================================== */
#define VBAT_DIVIDER       11.0f
#define VBAT_LOW           10.2f
#define VBAT_CRITICAL       9.6f
#define VBAT_HYST           0.4f
#define ADC_VREF            3.3f
#define ADC_FULL_SCALE   4095.0f

void    Voltage_Init(void);
void    Voltage_Task(void);          /* 主循环 100 ms 调一次 */
float   Voltage_Get(void);
float   Voltage_GetVdda(void);
float   Voltage_GetTemp(void);
uint8_t Voltage_IsLow(void);
uint8_t Voltage_IsCritical(void);

#endif /* __VOLTAGE_H */
