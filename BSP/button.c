#include "button.h"

#define BUTTON_DEBOUNCE_MS  20U

/**
 * @brief  检测 KEY1 是否产生了一次有效按下
 *
 * 消抖后检测下降沿（按下），事件会被锁存，
 * 即使主循环轮询慢（LCD 刷屏期间），也不会漏掉短按。
 * 调用一次返回 1 后事件被清除，不会重复触发。
 */
uint8_t Button_Key1Pressed(void)
{
    static GPIO_PinState last_raw_state = GPIO_PIN_SET;
    static GPIO_PinState stable_state   = GPIO_PIN_SET;
    static uint32_t last_change_ms      = 0U;
    static uint8_t  pending_press       = 0U;   /* 锁存：有未消费的按下事件 */

    GPIO_PinState raw_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

    /* 原始电平变化时重置消抖计时 */
    if (raw_state != last_raw_state) {
        last_raw_state = raw_state;
        last_change_ms = HAL_GetTick();
    }

    /* 消抖窗口内不做判断 */
    if ((HAL_GetTick() - last_change_ms) < BUTTON_DEBOUNCE_MS) {
        /* 即使在消抖中，若之前已有锁存事件，也可以被消费 */
        if (pending_press) {
            pending_press = 0U;
            return 1U;
        }
        return 0U;
    }

    /* 消抖完成，检测稳定电平是否发生了变化 */
    if (stable_state != raw_state) {
        stable_state = raw_state;
        /* 下降沿 = 按下（低有效），锁存此事件 */
        if (stable_state == GPIO_PIN_RESET) {
            pending_press = 1U;
        }
    }

    /* 消费锁存的按下事件 */
    if (pending_press) {
        pending_press = 0U;
        return 1U;
    }

    return 0U;
}
