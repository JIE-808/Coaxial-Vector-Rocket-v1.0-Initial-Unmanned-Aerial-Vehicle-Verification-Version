/**
 * @file  imu_heat.c
 * @brief BMI088 IMU 恒温 PID 控制
 *        通过 TIM3_CH4 (PB01) 驱动 N-MOS 加热电阻阵列
 */
#include "imu_heat.h"
#include "tim.h"
#include "stm32h7xx_hal.h"
#include <math.h>

/* ==================== 配置参数 ==================== */

/* 目标恒温温度 (°C)
 * BMI088 陀螺仪温漂约 ±0.015°/s/K，温度越稳定漂移越小
 * 50°C 远高于大多数环境温度，且远低于 BMI088 上限 85°C */
#define IMU_HEAT_TARGET_TEMP    50.0f

/* Bang-Bang → PID 切换阈值:
 * 当 err > BANG_THRESH 时全功率快速升温，接近目标后切 PID 精控 */
#define IMU_HEAT_BANG_THRESH    3.0f

/* PID 增益 (仅在接近目标温度时生效) */
#define IMU_HEAT_KP     150.0f    /* 比例: err=2°C → P=300 (3% PWM) */
#define IMU_HEAT_KI       5.0f    /* 积分: 消除稳态误差 */
#define IMU_HEAT_KD     200.0f    /* 微分: 抑制过冲 */

/* 积分限幅 (对应最大 PWM 的 50%，在 PID 阶段有足够余量维持温度) */
#define IMU_HEAT_I_MAX   5000.0f

/* PWM 最大值 (TIM3 Period = 10000-1) */
#define IMU_HEAT_PWM_MAX 9999

/* 温度稳定判定阈值 (°C) */
#define IMU_HEAT_STABLE_THRESH  1.0f

/* ==================== PID 状态 ==================== */
static float pid_integral  = 0.0f;
static float pid_last_temp = 0.0f;   /* 上次温度，用于 D 项 (微分先行) */
static float current_duty  = 0.0f;
static uint8_t is_stable   = 0;
static uint8_t first_call  = 1;
static uint32_t last_tick  = 0;

/* ==================== 内部函数 ==================== */
static void IMU_Heat_SetPWM(uint32_t pulse)
{
    if (pulse > IMU_HEAT_PWM_MAX) pulse = IMU_HEAT_PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);
}

/* ==================== 外部接口 ==================== */
void IMU_Heat_Init(void)
{
    pid_integral  = 0.0f;
    pid_last_temp = 0.0f;
    is_stable     = 0;
    first_call    = 1;

    /* 启动 TIM3 CH4 PWM */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* 初始关闭加热 */
    IMU_Heat_SetPWM(0);
}

void IMU_Heat_Update(float current_temp)
{
    /* 计算 dt (秒) */
    uint32_t now = HAL_GetTick();
    float dt;
    if (first_call) {
        dt = 0.01f;
        pid_last_temp = current_temp;
        first_call = 0;
    } else {
        uint32_t delta_ms = now - last_tick;
        if (delta_ms == 0) delta_ms = 1;
        dt = (float)delta_ms * 0.001f;
    }
    last_tick = now;
    if (dt > 0.5f) dt = 0.5f;  /* 安全限制 */

    float err = IMU_HEAT_TARGET_TEMP - current_temp;

    float output;

    if (err > IMU_HEAT_BANG_THRESH) {
        /* ---- Phase 1: 远离目标，全功率 Bang-Bang 快速升温 ---- */
        output = (float)IMU_HEAT_PWM_MAX;
        /* 预装积分项为稳态维持值的估计 (约 15% PWM)，实现无缝切入 PID */
        pid_integral = (float)IMU_HEAT_PWM_MAX * 0.15f;
    } else {
        /* ---- Phase 2: 接近目标，PID 精确控温 ---- */
        /* P */
        float p_out = IMU_HEAT_KP * err;

        /* I（乘以 dt，积分与调用频率无关） */
        pid_integral += IMU_HEAT_KI * err * dt;
        if (pid_integral >  IMU_HEAT_I_MAX) pid_integral =  IMU_HEAT_I_MAX;
        if (pid_integral < 0.0f)            pid_integral = 0.0f;

        /* D（微分先行：对温度变化率微分而非误差，避免目标突变冲击） */
        float d_out = -IMU_HEAT_KD * (current_temp - pid_last_temp) / dt;

        output = p_out + pid_integral + d_out;
    }
    pid_last_temp = current_temp;

    /* 输出限幅 (单向加热，不可能负输出) */
    if (output < 0.0f) output = 0.0f;
    if (output > (float)IMU_HEAT_PWM_MAX) output = (float)IMU_HEAT_PWM_MAX;

    IMU_Heat_SetPWM((uint32_t)output);
    current_duty = output / (float)IMU_HEAT_PWM_MAX * 100.0f;

    /* 判断是否稳定 */
    is_stable = (fabsf(err) < IMU_HEAT_STABLE_THRESH) ? 1 : 0;
}

float IMU_Heat_GetTarget(void)
{
    return IMU_HEAT_TARGET_TEMP;
}

float IMU_Heat_GetDuty(void)
{
    return current_duty;
}

uint8_t IMU_Heat_IsStable(void)
{
    return is_stable;
}
