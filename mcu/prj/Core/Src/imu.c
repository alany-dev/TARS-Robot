#include "imu.h"
#include "Fusion.h"
#include <math.h>
#include <stdio.h>

/*
 * QMI8658 驱动：硬件 I2C2（CubeMX 已生成 hi2c2 / MX_I2C2_Init）
 *
 * 数据流：
 *   I2C 读 0x35 起 12 字节 → int16 六轴 → 换算成 g / dps
 *   → 官方 Fusion AHRS（三轴陀螺 + 三轴加速度）→ 四元数/欧拉角
 *
 * 为什么用 Fusion：
 *   讲义 8.9 的互补滤波只用于理解“陀螺短期 + 加速度长期”的思想；
 *   实际代码用 Seb Madgwick 的 Fusion 库（出厂例程同款），
 *   三轴都参与融合，动态性能比手写互补滤波好。
 */

extern I2C_HandleTypeDef hi2c2;

static uint8_t s_addr = 0x6A;          /* 扫描后可能是 0x6B */
static float s_roll = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw = 0.0f;

static FusionAhrs   s_ahrs;
static FusionOffset s_offset;

static uint8_t  s_whoami = 0;
static uint32_t s_err = 0;
static IMU_Data_t s_last = {0};

/* FusionOffset 的 cal_count 在陀螺 >5 dps 时不会递减，
 * 这里用独立计时做超时兜底，避免姿态永远不更新。 */
static uint32_t s_cal_ms = 0;

/* 芯片轴 → 车体轴（X 前 / Y 左 / Z 上）的安装方向。
 * 实测：平放 az=+0.93；车头抬 30° 时 ay=-0.5；左侧抬 30° 时 ax=+0.5；左转 gz 正。
 * 推导：X_car = -Y_chip，Y_car = +X_chip，Z_car = +Z_chip → -Y+X+Z（屏幕横装 90°） */
#define IMU_AXES_ALIGN   FusionAxesAlignmentNYPXPZ

/* QMI8658 寄存器 */
#define QMI_WHO_AM_I   0x00
#define QMI_CTRL1      0x02
#define QMI_CTRL2      0x03
#define QMI_CTRL3      0x04
#define QMI_CTRL7      0x08      /* 注意：CTRL7 在 0x08，不是 0x06 */
#define QMI_AX_L       0x35

static HAL_StatusTypeDef qmi_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c2, s_addr << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 20);
}

static HAL_StatusTypeDef qmi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2, s_addr << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 20);
}

uint8_t IMU_Init(void)
{
    uint8_t id = 0;

    /* 扫描 0x6A / 0x6B，确认模块在线 */
    if (HAL_I2C_IsDeviceReady(&hi2c2, 0x6A << 1, 3, 20) == HAL_OK) {
        s_addr = 0x6A;
    } else if (HAL_I2C_IsDeviceReady(&hi2c2, 0x6B << 1, 3, 20) == HAL_OK) {
        s_addr = 0x6B;
    } else {
        return 1;
    }

    if (qmi_read(QMI_WHO_AM_I, &id, 1) != HAL_OK) return 1;
    s_whoami = id;
    if (id != 0x05) return 2;

    /* 软复位，等 20 ms */
    qmi_write(0x60, 0xB0);
    HAL_Delay(20);

    /* CTRL1：bit6 ADDR_AI=1，bit5 BE=0（小端），I2C 模式 */
    qmi_write(QMI_CTRL1, 0x40);
    /* CTRL2：aFS=010(±8g)，aODR=0101(235Hz) */
    qmi_write(QMI_CTRL2, 0x25);
    /* CTRL3：gFS=110(±1024dps)，gODR=0101(235Hz) */
    qmi_write(QMI_CTRL3, 0x65);
    /* CTRL7：sys_hs=1 + gEN + aEN */
    qmi_write(QMI_CTRL7, 0x43);
    HAL_Delay(200);

    /* 读回确认使能位 */
    uint8_t c7 = 0;
    if (qmi_read(QMI_CTRL7, &c7, 1) != HAL_OK) return 3;
    if ((c7 & 0x03) != 0x03) return 3;

    /* 初始化 Fusion：采样率 100 Hz（对应 10 ms 调用） */
    FusionAhrsInitialise(&s_ahrs);
    FusionOffsetInitialise(&s_offset, 100);

    return 0;
}

void IMU_Read(IMU_Data_t *d)
{
    uint8_t b[12];

    if (qmi_read(QMI_AX_L, b, 12) != HAL_OK) {
        s_err++;            /* 保留上一次的值，不要在失败时清 0 */
        return;
    }

    int16_t ax = (int16_t)((b[1] << 8) | b[0]);
    int16_t ay = (int16_t)((b[3] << 8) | b[2]);
    int16_t az = (int16_t)((b[5] << 8) | b[4]);
    int16_t gx = (int16_t)((b[7] << 8) | b[6]);
    int16_t gy = (int16_t)((b[9] << 8) | b[8]);
    int16_t gz = (int16_t)((b[11] << 8) | b[10]);

    d->ax = ax / IMU_ACC_LSB;
    d->ay = ay / IMU_ACC_LSB;
    d->az = az / IMU_ACC_LSB;
    d->gx = gx / IMU_GYRO_LSB;
    d->gy = gy / IMU_GYRO_LSB;
    d->gz = gz / IMU_GYRO_LSB;
}

void IMU_CalibrateGyro(uint16_t samples)
{
    float sx = 0, sy = 0, sz = 0;
    uint16_t n = 0;
    uint16_t max_iter;
    IMU_Data_t d;

    if (samples == 0) samples = 1;
    max_iter = (uint16_t)(samples * 2U);

    /* 用「加速度模长≈1g」判断静止，不用陀螺阈值——
     * FusionOffset 自己的 5 dps 阈值在零偏较大时会永远认为"在动"，拒绝校准。 */
    for (uint16_t i = 0; i < max_iter && n < samples; ++i) {
        IMU_Read(&d);
        float amag = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
        if (fabsf(amag - 1.0f) < 0.20f) {
            sx += d.gx;
            sy += d.gy;
            sz += d.gz;
            n++;
        }
        HAL_Delay(2);
    }

    if (n == 0) {
        return;     /* 一直没静止，保留旧零偏 */
    }

    /* 直接把平均零偏写进 FusionOffset 结构，之后每次 FusionOffsetUpdate
     * 都会先减掉它，陀螺接近 0，姿态不再被零偏拖着漂。 */
    s_offset.gyroscopeOffset.axis.x = sx / (float)n;
    s_offset.gyroscopeOffset.axis.y = sy / (float)n;
    s_offset.gyroscopeOffset.axis.z = sz / (float)n;

    /* 校准已完成，跳过 FusionOffset 的等待期 */
    s_offset.timer = 0;
    s_offset.cal_count = 0;
    s_cal_ms = 1500U;
}

void IMU_Update(float dt)
{
    IMU_Data_t d;
    IMU_Read(&d);
    s_last = d;

    FusionVector gyro  = {.axis.x = d.gx, .axis.y = d.gy, .axis.z = d.gz};
    FusionVector accel = {.axis.x = d.ax, .axis.y = d.ay, .axis.z = d.az};

    /* 安装方向对齐：把芯片轴映射到车体轴（X前/Y左/Z上）。
     * 默认 PXPYPZ 表示芯片安装方向与车体一致；
     * 装反/转 90° 时改这里的宏，见讲义 8.10。 */
    gyro  = FusionAxesSwap(gyro,  IMU_AXES_ALIGN);
    accel = FusionAxesSwap(accel, IMU_AXES_ALIGN);

    gyro = FusionOffsetUpdate(&s_offset, gyro);

    /* 校准期最多 1.5 s。
     * FusionOffset 在陀螺任一分量 >5 dps 时直接 return、cal_count 不减，
     * 若板子有轻微振动就会永远停在"校准中"，姿态一直是 0。 */
    if (s_offset.cal_count > 0) {
        s_cal_ms += (uint32_t)(dt * 1000.0f + 0.5f);
        if (s_cal_ms < 1500U) {
            return;
        }
        s_offset.cal_count = 0;     /* 超时强制进入正常融合 */
    }

    /* 官方 AHRS：三轴陀螺 + 三轴加速度，dt 单位秒 */
    FusionAhrsUpdateNoMagnetometer(&s_ahrs, gyro, accel, dt);

    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&s_ahrs));
    s_roll  = euler.angle.roll;
    s_pitch = euler.angle.pitch;
    s_yaw   = euler.angle.yaw;
}

float IMU_GetRoll(void)  { return s_roll; }
float IMU_GetPitch(void) { return s_pitch; }
float IMU_GetYaw(void)   { return s_yaw; }

void IMU_Dump(void)
{
    uint8_t st = 0, c1 = 0, c2 = 0, c3 = 0, c7 = 0;
    uint8_t raw[12] = {0};

    qmi_read(0x2E, &st, 1);          /* STATUS0：bit0 ACC 就绪，bit1 GYRO 就绪 */
    qmi_read(QMI_CTRL1, &c1, 1);
    qmi_read(QMI_CTRL2, &c2, 1);
    qmi_read(QMI_CTRL3, &c3, 1);
    qmi_read(QMI_CTRL7, &c7, 1);
    qmi_read(QMI_AX_L, raw, 12);

    printf("IMU ST=%02X C1=%02X C2=%02X C3=%02X C7=%02X RAW=",
           st, c1, c2, c3, c7);
    for (int i = 0; i < 12; ++i) {
        printf("%02X ", raw[i]);
    }
    printf("\r\n");
}

void IMU_GetAcc(float *x, float *y, float *z)
{
    if (x) *x = s_last.ax;
    if (y) *y = s_last.ay;
    if (z) *z = s_last.az;
}

void IMU_GetGyro(float *x, float *y, float *z)
{
    if (x) *x = s_last.gx;
    if (y) *y = s_last.gy;
    if (z) *z = s_last.gz;
}

uint8_t IMU_GetWhoAmI(void)      { return s_whoami; }
uint32_t IMU_GetErrorCount(void) { return s_err; }
