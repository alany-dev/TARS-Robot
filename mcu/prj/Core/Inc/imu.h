#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include <stdint.h>

/* ============================================================================
 * QMI8658 六轴 IMU（I2C2：PB10=SCL / PB11=SDA，400 kHz）
 * ----------------------------------------------------------------------------
 * 地址：0x6A（SA0 接地）或 0x6B（SA0 接高），IMU_Init 会自动扫描
 * WHO_AM_I(0x00) = 0x05
 *
 * 量程/换算（与 imu.c 的 CTRL2/CTRL3 对应）：
 *   ACC ±8 g     → 4096 LSB/g
 *   GYR ±1024 dps → 32 LSB/(deg/s)
 * 改量程时这两个系数要一起改。
 * ========================================================================== */
#define IMU_ACC_LSB       4096.0f
#define IMU_GYRO_LSB      32.0f

typedef struct {
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* deg/s */
} IMU_Data_t;

uint8_t IMU_Init(void);                 /* 0=成功，1=未找到，2=WHO_AM_I 不对 */
void    IMU_Read(IMU_Data_t *d);
void    IMU_Update(float dt);           /* dt 单位秒，10 ms 传 0.01f */
void    IMU_CalibrateGyro(uint16_t samples);

float   IMU_GetRoll(void);
float   IMU_GetPitch(void);
float   IMU_GetYaw(void);
void    IMU_GetAcc(float *x, float *y, float *z);  /* 调试：最近一次加速度 g */
void    IMU_GetGyro(float *x, float *y, float *z); /* 调试：最近一次角速度 dps */
void    IMU_Dump(void);                 /* 调试：打印 STATUS0 / CTRL / 原始 12 字节 */
uint8_t IMU_GetWhoAmI(void);            /* 调试：实际读到的 WHO_AM_I */
uint32_t IMU_GetErrorCount(void);       /* 调试：I2C 读失败次数 */

#endif /* __IMU_H */
