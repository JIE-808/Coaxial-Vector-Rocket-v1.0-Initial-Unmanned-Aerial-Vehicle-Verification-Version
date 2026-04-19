/**
 * @file  vbat_adc.c
 * @brief 3S 锂电池电压采集、状态判定与 LCD 显示
 */
#include "vbat_adc.h"
#include "adc.h"
#include "esc_pwm.h"
#include "lcd.h"
#include "tim.h"
#include "usb_printf.h"
#include <stdio.h>

/* 分压电阻：R86 = 100K，R87 = 10K，分压比约为 11:1 */
#define VBAT_DIVIDER_RATIO              11.0f
/* ADC 参考电压 */
#define VBAT_VREF                       3.3f
/* 16 位 ADC 满量程 */
#define VBAT_ADC_MAX                    65535.0f
/* 滑动平均滤波窗口长度 */
#define VBAT_FILTER_SIZE                16U

/*
 * 3S 锂电池电压阈值
 * 满电：     4.20V/Cell -> 12.60V
 * 建议充电： 3.50V/Cell -> 10.50V
 * 致命低压： 3.20V/Cell ->  9.60V
 *
 * 这里把保护阈值定在 9.6V，是为了在接近 3.0V/Cell 深度过放区之前
 * 提前切断控制逻辑，给电池留出保护余量。
 */
#define VBAT_3S_FULL                    12.6f
#define VBAT_3S_RECHARGE                10.5f
#define VBAT_3S_CRITICAL                9.6f
#define VBAT_CRITICAL_CONFIRM_COUNT     3U

/* 橙色（RGB565） */
#define VBAT_COLOR_ORANGE               0xFD20

/* 电池 UI 区域布局参数 */
#define VBAT_UI_CLEAR_Y                 244U
#define VBAT_ICON_X                     14U
#define VBAT_ICON_Y                     248U
#define VBAT_ICON_W                     62U
#define VBAT_ICON_H                     28U
#define VBAT_ICON_CAP_W                 5U
#define VBAT_ICON_CAP_H                 12U
#define VBAT_ICON_INSET                 3U
#define VBAT_PERCENT_X                  88U
#define VBAT_PERCENT_Y                  250U
#define VBAT_VOLTAGE_X                  146U
#define VBAT_VOLTAGE_Y                  250U

static float vbat_buf[VBAT_FILTER_SIZE];
static uint8_t vbat_idx = 0;
static float vbat_voltage = 0.0f;
static uint8_t vbat_critical_count = 0;
static VBAT_State_t vbat_state = VBAT_STATE_NORMAL;
/* 电池静态 UI 只绘制一次，后续只刷新变化区域 */
static uint8_t vbat_ui_initialized = 0;
/* 进入致命低压保护后置 1，主循环和定时控制都会停机 */
static volatile uint8_t vbat_shutdown = 0U;

static float constrain_float(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static float VBAT_ConvertRawToVoltage(uint32_t raw)
{
    /* 按 ADC 采样值和分压比换算回电池整包电压 */
    return ((float)raw / VBAT_ADC_MAX) * VBAT_VREF * VBAT_DIVIDER_RATIO;
}

static uint32_t VBAT_ReadRaw(void)
{
    /* 单次启动 ADC，读取一次原始值 */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return raw;
}

static void VBAT_UpdateState(float sample_voltage)
{
    /* 致命低压采用连续计数确认，避免瞬时毛刺误触发 */
    if (sample_voltage <= VBAT_3S_CRITICAL) {
        if (vbat_critical_count < 0xFFU) {
            vbat_critical_count++;
        }
    } else {
        vbat_critical_count = 0;
    }

    if (vbat_critical_count >= VBAT_CRITICAL_CONFIRM_COUNT) {
        vbat_state = VBAT_STATE_CRITICAL;
    } else if (vbat_voltage <= VBAT_3S_RECHARGE) {
        vbat_state = VBAT_STATE_LOW;
    } else {
        vbat_state = VBAT_STATE_NORMAL;
    }
}

static uint16_t VBAT_GetDisplayColor(void)
{
    /* 低压和致命低压统一使用红色提醒 */
    if (vbat_state == VBAT_STATE_CRITICAL || vbat_state == VBAT_STATE_LOW) {
        return RED;
    }

    if (vbat_voltage >= 11.4f) {
        return GREEN;
    }

    return VBAT_COLOR_ORANGE;
}

static void VBAT_DrawBatteryIcon(uint16_t x, uint16_t y, uint16_t color, float percent)
{
    /* 电池图标由外框、正极帽和内部填充三部分组成 */
    uint16_t inner_x = x + VBAT_ICON_INSET;
    uint16_t inner_y = y + VBAT_ICON_INSET;
    uint16_t inner_w = VBAT_ICON_W - 2U * VBAT_ICON_INSET;
    uint16_t inner_h = VBAT_ICON_H - 2U * VBAT_ICON_INSET;
    uint16_t cap_x = x + VBAT_ICON_W + 1U;
    uint16_t cap_y = y + (VBAT_ICON_H - VBAT_ICON_CAP_H) / 2U;
    uint16_t fill_w = (uint16_t)((percent / 100.0f) * (float)inner_w);

    LCD_DrawRectangle(x, y, x + VBAT_ICON_W, y + VBAT_ICON_H, color);
    LCD_Fill(inner_x, inner_y, inner_x + inner_w - 1U, inner_y + inner_h - 1U, WHITE);
    LCD_Fill(cap_x, cap_y, cap_x + VBAT_ICON_CAP_W, cap_y + VBAT_ICON_CAP_H, color);

    if ((percent > 0.0f) && (fill_w == 0U)) {
        fill_w = 2U;
    }

    if (fill_w > inner_w) {
        fill_w = inner_w;
    }

    if (fill_w > 0U) {
        LCD_Fill(inner_x, inner_y, inner_x + fill_w - 1U, inner_y + inner_h - 1U, color);
    }
}

static void VBAT_DrawStaticUI(void)
{
    if (vbat_ui_initialized) {
        return;
    }

    /* 底部电池显示区域只清一次，避免每次整块重绘造成闪烁 */
    LCD_Fill(0, VBAT_UI_CLEAR_Y, LCD_W, LCD_H, WHITE);
    vbat_ui_initialized = 1;
}

void VBAT_Init(void)
{
    /* ADC 校准，提高电压读数精度 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    /* 先读一次电压，把滤波缓冲区全部预填满 */
    float v = VBAT_ConvertRawToVoltage(VBAT_ReadRaw());
    for (uint32_t i = 0; i < VBAT_FILTER_SIZE; i++) {
        vbat_buf[i] = v;
    }

    vbat_voltage = v;
    vbat_idx = 0;
    vbat_critical_count = 0;
    vbat_ui_initialized = 0;
    VBAT_UpdateState(v);
}

float VBAT_Read(void)
{
    float v = VBAT_ConvertRawToVoltage(VBAT_ReadRaw());
    float sum = 0.0f;

    /* 滑动平均滤波 */
    vbat_buf[vbat_idx] = v;
    vbat_idx = (uint8_t)((vbat_idx + 1U) % VBAT_FILTER_SIZE);

    for (uint32_t i = 0; i < VBAT_FILTER_SIZE; i++) {
        sum += vbat_buf[i];
    }

    vbat_voltage = sum / (float)VBAT_FILTER_SIZE;
    VBAT_UpdateState(v);
    return vbat_voltage;
}

float VBAT_GetVoltage(void)
{
    return vbat_voltage;
}

VBAT_State_t VBAT_GetState(void)
{
    return vbat_state;
}

float VBAT_GetPercent(void)
{
    /* 用建议充电电压到满电电压之间做线性映射 */
    float percent = (vbat_voltage - VBAT_3S_RECHARGE) * 100.0f /
                    (VBAT_3S_FULL - VBAT_3S_RECHARGE);
    return constrain_float(percent, 0.0f, 100.0f);
}

void VBAT_Display(void)
{
    char buf[20];
    float percent = VBAT_GetPercent();
    uint16_t color = VBAT_GetDisplayColor();

    /* 静态框架只初始化一次，避免闪屏 */
    VBAT_DrawStaticUI();
    VBAT_DrawBatteryIcon(VBAT_ICON_X, VBAT_ICON_Y, color, percent);

    /* 仅局部清除动态数字区域，降低 SPI 屏重绘闪烁 */
    LCD_Fill(VBAT_PERCENT_X, VBAT_PERCENT_Y, 136, 278, WHITE);
    snprintf(buf, sizeof(buf), "%3.0f%%  ", percent);
    LCD_ShowString(VBAT_PERCENT_X, VBAT_PERCENT_Y, (const uint8_t *)buf, color, WHITE, 24, 0);

    LCD_Fill(VBAT_VOLTAGE_X, VBAT_VOLTAGE_Y, 238, 278, WHITE);
    snprintf(buf, sizeof(buf), "%4.2fV", vbat_voltage);
    LCD_ShowString(VBAT_VOLTAGE_X, VBAT_VOLTAGE_Y, (const uint8_t *)buf, GREEN, WHITE, 24, 0);
}

void VBAT_ShowCriticalWarning(void)
{
    /* 致命低压时只保留全屏警告，不再显示其他运行信息 */
    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
    LCD_DrawRectangle(4, 4, LCD_W - 5U, LCD_H - 5U, RED);
    LCD_DrawRectangle(6, 6, LCD_W - 7U, LCD_H - 7U, RED);
    LCD_Fill(10, 16, LCD_W - 11U, 56, RED);

    LCD_ShowString(26, 26, (const uint8_t *)"LOW BATTERY!", WHITE, RED, 24, 0);
    LCD_ShowString(28, 92, (const uint8_t *)"CONTROL STOPPED", RED, WHITE, 24, 0);
    LCD_ShowString(36, 136, (const uint8_t *)"CHARGE NOW", RED, WHITE, 24, 0);
    LCD_ShowString(28, 196, (const uint8_t *)"PLEASE RECHARGE", BLACK, WHITE, 16, 0);
    LCD_ShowString(20, 220, (const uint8_t *)"DO NOT KEEP USING", BLACK, WHITE, 16, 0);
}

void VBAT_EnterProtectionMode(void)
{
    if (vbat_shutdown) {
        return;
    }

    vbat_shutdown = 1U;

    /* 停止姿态更新，并关闭所有执行机构输出 */
    HAL_TIM_Base_Stop_IT(&htim4);
    ESC_PWM_Stop();
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

    /* 只保留低压警告页面 */
    USB_Printf_SetEnabled(0U);
    VBAT_ShowCriticalWarning();
}

uint8_t VBAT_IsShutdown(void)
{
    return vbat_shutdown;
}
