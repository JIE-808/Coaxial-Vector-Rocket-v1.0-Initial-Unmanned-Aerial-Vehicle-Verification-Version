/**
 * @file  altitude_hold.c
 * @brief 超声波定高控制器
 *
 * 控制结构：
 *   遥控器 CH3 油门杆 → 目标高度 (cm)
 *   PID 误差 = 目标高度 - 实测高度
 *   PID 输出 → 油门修正量 (μs)
 *   实际油门 = 悬停基础油门 + PID 输出，限幅后写入 TVC_SetBaseThrottle()
 *
 * 安全策略：
 *   - 超声波无效时保持上一次油门，超过 SONAR_LOST_TIMEOUT_MS 后缓慢降落
 *   - 目标高度杆量低于 DISARM_ALT_CM 时油门归底（落地保护）
 */
#include "altitude_hold.h"
#include "tvc_pid.h"
#include "usb_printf.h"
#include <stdint.h>

/* ── 定高范围配置 ── */
#define ALTHOLD_MIN_CM        0.0f    /* 杆量最低时对应高度 (cm)，0 = 地面 */
#define ALTHOLD_MAX_CM       80.0f    /* 杆量最高时对应高度 (cm)，在超声波量程内 */

/* 杆量低于此死区阈值时强制目标高度=0（地面/落地） */
#define STICK_BOTTOM_SBUS     380
/* 遥控器 SBUS 有效范围 */
#define STICK_MIN_SBUS        300
#define STICK_MAX_SBUS        1700

/* ── 油门配置 ── */
/* 悬停基础油门（μs）：调试时先手动飞一下记录悬停油门再填这里 */
#define HOVER_THROTTLE_US    1480U
/* PID 输出允许的最大调节范围 (μs) */
#define PID_OUTPUT_MAX_US    200.0f
/* 最终油门输出上下限 */
#define THROTTLE_MIN_US      1100U
#define THROTTLE_MAX_US      1900U

/* 目标高度平滑：最大变化率 (cm/每次update)
 * 10Hz × 3cm = 30cm/s，防止目标跳变引起巨大误差 */
#define TARGET_SLEW_CM_PER_UPDATE  3.0f

/* ── 超声波丢失保护 ── */
/* 超声波连续无效超过此时间(ms)后，缓慢降低油门 */
#define SONAR_LOST_TIMEOUT_MS  500U
/* 每次降落步长 (μs/update) */
#define DESCENT_STEP_US         3U

/* ── PID 参数 (待调参) ── */
/* 建议调参顺序：
 *   1. 先单独调 Kp，使高度有明显纠偏趋势但不振荡
 *   2. 再加 Kd，抑制超调
 *   3. 最后加少量 Ki，消除稳态误差
 */
#define ALTHOLD_KP     3.0f    /* 比例：误差 1cm → 3μs 油门修正（保守，防振荡） */
#define ALTHOLD_KI     0.15f   /* 积分：缓慢累积消稳态偏差 */
#define ALTHOLD_KD     5.0f    /* 微分：抑制超调，降低以减少器噪声放大 */
#define ALTHOLD_INTEGRAL_MAX  100.0f  /* 积分限幅 (μs) */

/* ── 内部状态 ── */
static float  g_target_cm       = ALTHOLD_MIN_CM;
static float  g_smooth_target_cm = ALTHOLD_MIN_CM;  /* 平滑后的实际目标 */
static float  g_integral        = 0.0f;
static float  g_prev_error      = 0.0f;
static float  g_pid_output      = 0.0f;
static uint16_t g_last_throttle = HOVER_THROTTLE_US;

static uint32_t g_sonar_lost_ms = 0;    /* 超声波丢失开始时刻 (HAL_GetTick) */
static uint8_t  g_sonar_was_valid = 0;

/* 内联限幅 */
static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ── 外部引用：HAL_GetTick 在 main.h 里通过 stm32h7xx_hal.h 已引入 ── */
#include "main.h"

/* ── 公共 API ── */

void AltHold_Init(void)
{
    g_target_cm      = ALTHOLD_MIN_CM;
    g_smooth_target_cm = ALTHOLD_MIN_CM;
    g_integral       = 0.0f;
    g_prev_error     = 0.0f;
    g_pid_output     = 0.0f;
    g_last_throttle  = HOVER_THROTTLE_US;
    g_sonar_lost_ms  = 0;
    g_sonar_was_valid = 0;
}

void AltHold_SetTargetFromStick(int sbus_throttle)
{
    /* 杆到底死区：强制目标高度 = 0（降落意图） */
    if (sbus_throttle <= STICK_BOTTOM_SBUS) {
        g_target_cm = 0.0f;
        return;
    }
    if (sbus_throttle > STICK_MAX_SBUS) sbus_throttle = STICK_MAX_SBUS;

    /* 死区以上线性映射：STICK_BOTTOM_SBUS~STICK_MAX_SBUS → 0~80cm */
    float ratio = (float)(sbus_throttle - STICK_BOTTOM_SBUS) /
                  (float)(STICK_MAX_SBUS - STICK_BOTTOM_SBUS);
    g_target_cm = ratio * ALTHOLD_MAX_CM;
}

void AltHold_SetTarget_cm(float target_cm)
{
    g_target_cm = clamp(target_cm, ALTHOLD_MIN_CM, ALTHOLD_MAX_CM);
}

float AltHold_GetTarget_cm(void)
{
    return g_smooth_target_cm;
}

void AltHold_ResetIntegral(void)
{
    g_integral   = 0.0f;
    g_prev_error = 0.0f;
    g_smooth_target_cm = 0.0f;
}

float AltHold_GetPIDOutput(void)
{
    return g_pid_output;
}

void AltHold_Update(float actual_cm, uint8_t sonar_valid)
{
    uint32_t now = HAL_GetTick();

    /* ── 落地保护：目标高度 ≤ 2cm（杆在死区内）强制最低油门 ── */
    if (g_target_cm <= 2.0f) {
        g_integral = 0.0f;
        g_pid_output = 0.0f;
        g_smooth_target_cm = 0.0f;
        TVC_SetBaseThrottle(1000);
        g_last_throttle = 1000;
        return;
    }

    /* ── 超声波丢失处理 ── */
    if (!sonar_valid) {
        if (g_sonar_was_valid) {
            g_sonar_lost_ms = now;   /* 记录丢失起始时刻 */
            g_sonar_was_valid = 0;
        }

        if ((now - g_sonar_lost_ms) >= SONAR_LOST_TIMEOUT_MS) {
            /* 超时：缓慢降落 */
            if (g_last_throttle > THROTTLE_MIN_US + DESCENT_STEP_US) {
                g_last_throttle -= DESCENT_STEP_US;
            } else {
                g_last_throttle = THROTTLE_MIN_US;
            }
            TVC_SetBaseThrottle(g_last_throttle);
        }
        /* 500ms 内保持上一次油门不变，等传感器恢复 */
        return;
    }

    g_sonar_was_valid = 1;

    /* ── 目标平滑：限制每次 update 目标变化不超过 3cm，避免跳变 ── */
    float diff = g_target_cm - g_smooth_target_cm;
    if (diff > TARGET_SLEW_CM_PER_UPDATE)
        g_smooth_target_cm += TARGET_SLEW_CM_PER_UPDATE;
    else if (diff < -TARGET_SLEW_CM_PER_UPDATE)
        g_smooth_target_cm -= TARGET_SLEW_CM_PER_UPDATE;
    else
        g_smooth_target_cm = g_target_cm;

    /* ── PID 计算（使用平滑目标） ── */
    float error = g_smooth_target_cm - actual_cm;

    /* 积分（带限幅，防止饱和） */
    g_integral += error;
    g_integral  = clamp(g_integral, -ALTHOLD_INTEGRAL_MAX, ALTHOLD_INTEGRAL_MAX);

    /* 微分（基于误差变化率） */
    float derivative = error - g_prev_error;
    g_prev_error = error;

    /* PID 输出 (μs) */
    g_pid_output = ALTHOLD_KP * error
                 + ALTHOLD_KI * g_integral
                 + ALTHOLD_KD * derivative;
    g_pid_output = clamp(g_pid_output, -PID_OUTPUT_MAX_US, PID_OUTPUT_MAX_US);

    /* 最终油门 = 悬停基础油门 + PID 修正 */
    int32_t throttle = (int32_t)HOVER_THROTTLE_US + (int32_t)g_pid_output;
    if (throttle < (int32_t)THROTTLE_MIN_US) throttle = (int32_t)THROTTLE_MIN_US;
    if (throttle > (int32_t)THROTTLE_MAX_US) throttle = (int32_t)THROTTLE_MAX_US;

    g_last_throttle = (uint16_t)throttle;
    TVC_SetBaseThrottle(g_last_throttle);
}
