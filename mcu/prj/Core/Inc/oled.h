#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/*
 * OLED 仪表盘（0.91" SSD1306 128x32，I2C2）
 * 标准实现：出厂 Third_Party/U8g2 + u8g2_porting.c
 * 移植层只把 osDelay/LWMEM 换成裸机 HAL_Delay/静态对象，API 与出厂一致。
 */
void OLED_Init(void);
void OLED_Dashboard(void);          /* 200 ms 调用一次 */

#endif /* __OLED_H */
