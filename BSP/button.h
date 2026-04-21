#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <stdint.h>

#include "gpio.h"
#include "stm32h7xx_hal.h"

/* 读取 KEY1 的一次稳定按下沿，按住不重复触发。 */
uint8_t Button_Key1Pressed(void);

#endif
