#include "oled.h"
#include "u8g2.h"
#include "voltage.h"
#include "speed_loop.h"
#include "imu.h"
#include <stdio.h>

/*
 * OLED 仪表盘：0.96 寸 SSD1306 128x64，标准硬件 SPI2
 *
 * 硬件（DISPLAY 8 孔座，屏插前 6 孔）：
 *   1=GND  2=3V3  3=PB13(SPI2_SCK)  4=PC3(SPI2_MOSI)
 *   5=PD14(RES)  6=PD13(DC)    7=PD12(CS)  8=PD11(BLK)
 *
 * CubeMX 配置（prj.ioc）：
 *   SPI2: Master, 2LINES, 8bit, Mode0(CPOL=Low/CPHA=1Edge),
 *         Prescaler=2 (21MBit/s), MSB first, NSS=Soft
 *   DMA:  SPI2_TX @ DMA1_Stream4/Ch0
 *   GPIO: PB13=SPI2_SCK, PC3=SPI2_MOSI (AF5)
 *         PD12=LCD_CS, PD13=LCD_DC, PD14=LCD_RES (GPIO Output, 在 MX_GPIO_Init)
 *
 * 驱动：出厂同款 U8g2；字节收发用硬件 SPI2（HAL_SPI_Transmit），
 *       标准 u8x8 字节回调，见 u8x8_byte_hw_spi()。
 */

extern SPI_HandleTypeDef hspi2;

static u8g2_t s_u8g2;

/* 注意：6 针屏没有 CS 脚（模块内部已拉低），所以不配置 PD12 */
#define OLED_DC_PORT   GPIOD
#define OLED_DC_PIN    GPIO_PIN_13
#define OLED_RES_PORT  GPIOD
#define OLED_RES_PIN   GPIO_PIN_14

/* ---------------------------------------------------------------------------
 * U8x8 硬件 SPI 字节回调（U8g2 官方 STM32 模板写法）
 *   U8G2_MSG_BYTE_SEND → HAL_SPI_Transmit(&hspi2, ...)
 *   DC / CS 仍由 GPIO 控制
 * ------------------------------------------------------------------------- */
static uint8_t u8x8_byte_hw_spi(u8x8_t *u8x8, uint8_t msg,
                                uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
    case U8X8_MSG_BYTE_INIT:
        /* SPI2 已由 MX_SPI2_Init 初始化，这里不用做事 */
        break;

    case U8X8_MSG_BYTE_SET_DC:
        HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN,
                          arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;

    /* 6 针屏无 CS：START/END_TRANSFER 不做片选翻转 */

    case U8X8_MSG_BYTE_SEND:
        /* 标准硬件 SPI 发送：阻塞式，1024 字节 @21MHz < 0.5 ms */
        if (HAL_SPI_Transmit(&hspi2, (uint8_t *)arg_ptr, arg_int, 100) != HAL_OK) {
            return 0;
        }
        break;

    default:
        return 0;
    }
    return 1;
}

/* U8g2 的延时/GPIO 回调：硬件 SPI 下只需处理延时和 RES */
static uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg,
                                   uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
    case U8X8_MSG_DELAY_100NANO:
        __NOP();
        break;
    case U8X8_MSG_DELAY_10MICRO:
        HAL_Delay(1);
        break;
    case U8X8_MSG_DELAY_MILLI:
        HAL_Delay(arg_int);
        break;
    case U8X8_MSG_DELAY_I2C:              /* SPI 不用 */
        break;
    case U8X8_MSG_GPIO_RESET:
        HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN,
                          arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    default:
        u8x8_SetGPIOResult(u8x8, 1);
        break;
    }
    return 1;
}

void OLED_Init(void)
{
    /* 4 线硬件 SPI + 128x64 SSD1306 */
    u8g2_Setup_ssd1306_128x64_noname_f(&s_u8g2, U8G2_R0,
                                       u8x8_byte_hw_spi,
                                       u8x8_gpio_and_delay);
    u8g2_InitDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0);
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SendBuffer(&s_u8g2);
}

void OLED_Dashboard(void)
{
    char line[22];
    float ax, ay, az, gx, gy, gz, vl, vr;

    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tf);   /* 6 行，基线 10/20/30/40/50/60 */

    IMU_GetAcc(&ax, &ay, &az);
    IMU_GetGyro(&gx, &gy, &gz);
    SpeedLoop_GetActualMps(&vl, &vr);

    /* 1 电压 */
    snprintf(line, sizeof(line), "BAT%5.2fV %s",
             Voltage_Get(), Voltage_IsLow() ? "LOW" : "OK");
    u8g2_DrawStr(&s_u8g2, 0, 10, line);

    /* 2 加速度原始值（校准方向看这行） */
    snprintf(line, sizeof(line), "a%5.2f%5.2f%5.2f", ax, ay, az);
    u8g2_DrawStr(&s_u8g2, 0, 20, line);

    /* 3 角速度原始值（看零偏/转向符号） */
    snprintf(line, sizeof(line), "g%5.1f%5.1f%5.1f", gx, gy, gz);
    u8g2_DrawStr(&s_u8g2, 0, 30, line);

    /* 4 姿态角 */
    snprintf(line, sizeof(line), "R%4.0f P%4.0f Y%4.0f",
             IMU_GetRoll(), IMU_GetPitch(), IMU_GetYaw());
    u8g2_DrawStr(&s_u8g2, 0, 40, line);

    /* 5 左右轮速 */
    snprintf(line, sizeof(line), "L%4.2f R%4.2f", vl, vr);
    u8g2_DrawStr(&s_u8g2, 0, 50, line);

    /* 6 距离 + 故障码（第 15 章接超声波后换成实际距离） */
    snprintf(line, sizeof(line), "D--.- F%u E%lu",
             SpeedLoop_IsFault(), (unsigned long)IMU_GetErrorCount());
    u8g2_DrawStr(&s_u8g2, 0, 60, line);

    u8g2_SendBuffer(&s_u8g2);
}
