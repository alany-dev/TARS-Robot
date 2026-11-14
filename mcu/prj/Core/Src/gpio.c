#include "gpio.h"

/*
 * 按键的外部中断逻辑已迁移到 key.c / key.h：
 *   - HAL_GPIO_EXTI_Callback()  -> key.c
 *   - 单击 / 长按 / 消抖状态机   -> key.c
 *
 * 本文件保留给 CubeMX 后续生成 GPIO 初始化代码使用，请勿在这里重复实现
 * HAL_GPIO_EXTI_Callback()，否则会与 key.c 产生重复定义。
 */
