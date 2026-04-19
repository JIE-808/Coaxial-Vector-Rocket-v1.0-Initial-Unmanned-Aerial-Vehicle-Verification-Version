/**
 * @file  ultrasonic.c
 * @brief HC-SR04 超声波测距驱动
 *        Trig: PB10 (GPIO)    Echo: PB11 / TIM2_CH4 (Input Capture)
 *
 *        TIM2 已配置: PSC=240-1 → 1MHz (1μs/tick), ARR=20000-1
 *        CH1/CH3 用于 ESC PWM 输出，CH4 用于 Echo 输入捕获，互不干扰。
 */
#include "ultrasonic.h"
#include "tim.h"
#include "main.h"
#include "lcd.h"
#include <stdio.h>

/* ── 硬件定义 ── */
#define TRIG_GPIO_Port   GPIOB
#define TRIG_Pin         GPIO_PIN_10

/* 声速: 343 m/s = 0.0343 cm/μs */
#define SOUND_SPEED_HALF  0.01715f   /* cm per μs (单程) */

/* TIM2 周期 (ARR+1)，用于溢出修正 */
#define TIM2_PERIOD       20000U

/* 超时阈值：触发后 30ms 未完成则判无回波 */
#define ECHO_TIMEOUT_MS   30U

/* HC-SR04 硬件盲区：发射余振未衰减，4cm 以内数据不可信 */
#define US_MIN_DIST_CM    4.0f
/* 4cm 对应脉宽: 4 / 0.01715 ≈ 233μs */
#define US_MIN_PULSE_US   233U

/* ── 滤波配置 ── */
/* 中值滤波窗口大小（奇数），用于剔除跳变/异常值 */
#define US_MEDIAN_SIZE    5U
/* 低通滤波系数：0~1，越小越平滑但响应越慢
 * 0.3 = 新值占 30%，旧值占 70%，适合 100ms 采样周期 */
#define US_LPF_ALPHA      0.3f
/* 异常跳变检测阈值(cm)：单次变化超过此值视为异常，丢弃 */
#define US_SPIKE_THRESH_CM  25.0f

/* ── 内部状态 ── */
typedef enum {
    US_IDLE = 0,       /* 空闲，可触发 */
    US_WAIT_RISE,      /* 等待上升沿 */
    US_WAIT_FALL       /* 等待下降沿 */
} us_state_t;

static volatile us_state_t  state = US_IDLE;
static volatile uint32_t    capture_rise = 0;
static volatile uint32_t    capture_fall = 0;
static volatile uint8_t     capture_done = 0;

static uint32_t trigger_tick = 0;     /* HAL_GetTick() 触发时刻 */
static float    distance_cm  = 0.0f;  /* 滤波后输出值 */
static uint8_t  valid        = 0;

/* ── 滤波内部状态 ── */
static float   median_buf[US_MEDIAN_SIZE];  /* 中值滤波环形缓冲 */
static uint8_t median_idx  = 0;
static uint8_t median_full = 0;             /* 缓冲区是否已填满 */
static float   lpf_out     = 0.0f;          /* 低通滤波输出 */
static float   last_valid_raw = 0.0f;       /* 上一次有效原始值（用于跳变检测）*/
static float   ground_offset  = 0.0f;       /* 地面校零偏移量 */

/* 插入排序取中值（窗口小，性能够用） */
static float CalcMedian(void)
{
    float tmp[US_MEDIAN_SIZE];
    uint8_t n = median_full ? US_MEDIAN_SIZE : median_idx;
    if (n == 0) return 0.0f;

    for (uint8_t i = 0; i < n; i++) tmp[i] = median_buf[i];

    /* 插入排序 */
    for (uint8_t i = 1; i < n; i++) {
        float key = tmp[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return tmp[n / 2];
}

/* ── 公共 API ── */

void Ultrasonic_Init(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    state = US_IDLE;
    valid = 0;
    median_idx  = 0;
    median_full = 0;
    lpf_out     = 0.0f;
    last_valid_raw = 0.0f;
    ground_offset  = 0.0f;
}

void Ultrasonic_Calibrate(void)
{
    /* 以当前滤波输出作为地面基准（0 点）*/
    if (valid && distance_cm > 0.0f) {
        ground_offset = distance_cm;
    }
    /* 若尚无有效读数，保留之前的偏移（不做任何更改）*/
}

void Ultrasonic_Trigger(void)
{
    if (state != US_IDLE) return;   /* 上次测量未结束 */

    capture_done = 0;
    state = US_WAIT_RISE;

    /* 启动 CH4 输入捕获，先捕获上升沿 */
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

    /* 发送 ≥10μs 的 Trig 脉冲（用 TIM2 计数器精确延时） */
    uint32_t t0 = __HAL_TIM_GET_COUNTER(&htim2);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - t0) < 12U) {}
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    trigger_tick = HAL_GetTick();
}

void Ultrasonic_CheckTimeout(void)
{
    /* 处理已完成的捕获 */
    if (capture_done) {
        capture_done = 0;

        uint32_t pulse_us;
        if (capture_fall >= capture_rise) {
            pulse_us = capture_fall - capture_rise;
        } else {
            pulse_us = (TIM2_PERIOD - capture_rise) + capture_fall;
        }

        /* 合理范围: ≥4cm (盲区过滤) ~ 400cm → ≥233μs ~ 23300μs */
        if (pulse_us >= US_MIN_PULSE_US && pulse_us <= 23300U) {
            float raw = (float)pulse_us * SOUND_SPEED_HALF;

            /* 1. 跳变检测：首次有效或变化在阈值内才接受 */
            if (last_valid_raw > 0.0f &&
                (raw - last_valid_raw > US_SPIKE_THRESH_CM ||
                 last_valid_raw - raw > US_SPIKE_THRESH_CM)) {
                /* 异常跳变：丢弃本次，保持上次输出 */
                valid = 1;  /* 保持有效，但不更新数值 */
                return;
            }
            last_valid_raw = raw;

            /* 2. 中值滤波：写入环形缓冲 */
            median_buf[median_idx] = raw;
            median_idx = (uint8_t)((median_idx + 1U) % US_MEDIAN_SIZE);
            if (median_idx == 0) median_full = 1;
            float median_val = CalcMedian();

            /* 3. 低通滤波：指数加权移动平均 */
            if (lpf_out < 1.0f) {
                lpf_out = median_val;  /* 首次直接赋值，避免从0爬升 */
            } else {
                lpf_out = US_LPF_ALPHA * median_val + (1.0f - US_LPF_ALPHA) * lpf_out;
            }

            distance_cm = lpf_out;
            valid = 1;
        } else {
            /* 小于盲区或超量程，丢弃，保留上次有效值但标记无效 */
            valid = 0;
        }
        return;
    }

    /* 超时处理 */
    if (state != US_IDLE) {
        if ((HAL_GetTick() - trigger_tick) >= ECHO_TIMEOUT_MS) {
            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);
            state = US_IDLE;
            valid = 0;
        }
    }
}

float Ultrasonic_GetDistance_cm(void)
{
    float h = distance_cm - ground_offset;
    return h > 0.0f ? h : 0.0f;  /* 相对地面高度，不低于 0 */
}

uint8_t Ultrasonic_IsValid(void)
{
    return valid;
}

void Ultrasonic_Display(void)
{
    char buf[16];
    if (valid) {
        snprintf(buf, sizeof(buf), "D:%5.1fcm", distance_cm);
        LCD_ShowString(148, 140, (const uint8_t *)buf, DARKBLUE, WHITE, 16, 0);
    } else if (distance_cm < US_MIN_DIST_CM && distance_cm > 0.0f) {
        /* 落入盲区 */
        LCD_ShowString(148, 140, (const uint8_t *)"D:<4cm! ", RED, WHITE, 16, 0);
    } else {
        LCD_ShowString(148, 140, (const uint8_t *)"D: ---  ", GRAY, WHITE, 16, 0);
    }
}

/* ── TIM2 CH4 输入捕获中断回调 ── */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_4)
        return;

    if (state == US_WAIT_RISE) {
        /* 上升沿：记录起始时刻，切换为下降沿捕获 */
        capture_rise = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
        state = US_WAIT_FALL;

    } else if (state == US_WAIT_FALL) {
        /* 下降沿：记录结束时刻，停止捕获 */
        capture_fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
        HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_4);
        capture_done = 1;
        state = US_IDLE;
    }
}
