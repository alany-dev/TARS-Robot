#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include <stdint.h>

/* 消抖时间：两次下降沿间隔小于该值视为抖动 */
#define KEY_DEBOUNCE_MS     20U
/* 长按判定时间 */
#define KEY_LONG_PRESS_MS   800U

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_CLICK,
    KEY_EVENT_LONG_PRESS
} KeyEvent_t;

void       Key_Init(void);
void       Key_Task(void);          /* 主循环每 10 ms 调一次 */
KeyEvent_t Key_GetEvent(void);      /* 取事件，取完自动清 */
uint8_t    Key_IsPressed(void);

#endif /* __KEY_H */
