#include "key.h"

/*
 * 按键驱动（PE0 / KEY1，下降沿外部中断，内部上拉）
 *
 * 设计原则：
 *   1. 中断里只做“最短的事”——消抖 + 记录按下时刻，不 printf、不延时；
 *   2. 释放与长按在 Key_Task() 里用轮询判断（CubeMX 只配了下降沿，
 *      释放不会产生中断）；
 *   3. 对外只暴露事件（单击 / 长按），业务代码不关心电平细节。
 */

static volatile uint32_t s_last_irq_ms = 0;   /* 上次中断时刻，用于消抖 */
static volatile uint32_t s_press_ms    = 0;   /* 本次按下时刻 */
static volatile uint8_t  s_pressed     = 0;   /* 1 = 当前处于按下状态 */
static volatile uint8_t  s_long_sent   = 0;   /* 1 = 本次按下已报过长按 */
static volatile uint8_t  s_event       = KEY_EVENT_NONE;

void Key_Init(void)
{
    s_last_irq_ms = 0;
    s_press_ms    = 0;
    s_pressed     = 0;
    s_long_sent   = 0;
    s_event       = KEY_EVENT_NONE;

    /* GPIO / EXTI / NVIC 已由 CubeMX 的 MX_GPIO_Init() 配好，
       这里只清软件状态，避免上电残留。 */
}

/* 覆盖 HAL 的 __weak 回调；EXTI0_IRQHandler 会调用到这里 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != KEY2_Pin) {
        return;
    }

    uint32_t now = HAL_GetTick();

    /* 软件消抖：机械按键一次按下可能产生多个边沿 */
    if (now - s_last_irq_ms < KEY_DEBOUNCE_MS) {
        return;
    }
    s_last_irq_ms = now;

    /* 确认是“按下”而不是干扰：上拉 + 按下接地，低电平才是真按下 */
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
        s_pressed   = 1;
        s_press_ms  = now;
        s_long_sent = 0;
    }
}

/* 主循环里每 10 ms 调用一次 */
void Key_Task(void)
{
    if (!s_pressed) {
        return;
    }

    uint32_t now = HAL_GetTick();

    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
        /* 已释放：没报过长按才算单击 */
        s_pressed = 0;
        if (!s_long_sent) {
            s_event = KEY_EVENT_CLICK;
        }
    } else if (!s_long_sent && (now - s_press_ms >= KEY_LONG_PRESS_MS)) {
        /* 仍按住且超过长按阈值：只报一次 */
        s_long_sent = 1;
        s_event     = KEY_EVENT_LONG_PRESS;
    }
}

KeyEvent_t Key_GetEvent(void)
{
    KeyEvent_t e = (KeyEvent_t)s_event;
    s_event = KEY_EVENT_NONE;
    return e;
}

uint8_t Key_IsPressed(void)
{
    return s_pressed;
}
