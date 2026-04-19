#include "esc_pwm.h"

#include "tim.h"

/* 记录当前左右两路 ESC 的脉宽，供状态显示和调试使用。 */
static uint16_t g_esc_left_us = ESC_PWM_MIN_US;
static uint16_t g_esc_right_us = ESC_PWM_MIN_US;

static uint16_t ESC_PWM_Clamp(uint16_t pulse_us)
{
    if (pulse_us < ESC_PWM_MIN_US) {
        return ESC_PWM_MIN_US;
    }

    if (pulse_us > ESC_PWM_MAX_US) {
        return ESC_PWM_MAX_US;
    }

    return pulse_us;
}

void ESC_PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    ESC_PWM_SetPulseUs(ESC_PWM_MIN_US, ESC_PWM_MIN_US);
}

void ESC_PWM_Stop(void)
{
    ESC_PWM_SetPulseUs(ESC_PWM_MIN_US, ESC_PWM_MIN_US);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
}

void ESC_PWM_SetPulseUs(uint16_t left_us, uint16_t right_us)
{
    g_esc_left_us = ESC_PWM_Clamp(left_us);
    g_esc_right_us = ESC_PWM_Clamp(right_us);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_esc_left_us);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, g_esc_right_us);
}

void ESC_PWM_SetAllPulseUs(uint16_t pulse_us)
{
    ESC_PWM_SetPulseUs(pulse_us, pulse_us);
}

uint16_t ESC_PWM_GetLeftPulseUs(void)
{
    return g_esc_left_us;
}

uint16_t ESC_PWM_GetRightPulseUs(void)
{
    return g_esc_right_us;
}
