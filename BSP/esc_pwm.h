#ifndef __ESC_PWM_H__
#define __ESC_PWM_H__

#include <stdint.h>

#define ESC_PWM_MIN_US 1000U
#define ESC_PWM_MAX_US 2000U

void ESC_PWM_Init(void);
void ESC_PWM_Stop(void);
void ESC_PWM_SetPulseUs(uint16_t left_us, uint16_t right_us);
void ESC_PWM_SetAllPulseUs(uint16_t pulse_us);
uint16_t ESC_PWM_GetLeftPulseUs(void);
uint16_t ESC_PWM_GetRightPulseUs(void);

#endif /* __ESC_PWM_H__ */
