#include "tvc_pid.h"

#include "main.h"
#include "tim.h"
#include "esc_pwm.h"

/*
 * TIM1 的计数分辨率为 1us，PWM 周期为 20ms。
 * TIM1_CH1 驱动左侧舵机。
 * TIM1_CH3 驱动右侧舵机。
 */

/* 陀螺仪输入为 rad/s，需转换到 deg/s 与角度量纲一致 */
#define RAD_TO_DEG_TVC  57.295779513f
#define SERVO_LEFT_CENTER_US    1560.0f
#define SERVO_LEFT_UP_US       1000.0f
#define SERVO_LEFT_DOWN_US       2000.0f

#define SERVO_RIGHT_CENTER_US   1510.0f
#define SERVO_RIGHT_DOWN_US      1000.0f
#define SERVO_RIGHT_UP_US      2000.0f

/* 角度到脉宽的近似换算：45deg 对应约 500us。 */
#define SERVO_US_PER_DEG        11.111f

/* 更保守的 TVC 控制参数，优先保证起飞不发散 */
#define ANGLE_DEADBAND_DEG       0.3f     /* 死区回退，保证小角度也能纠偏 */
#define ANGLE_RATE_MAX_DPS      60.0f     /* 外环输出上限回退 */
#define TVC_SERVO_MAX_DEG        7.0f     /* 内环输出：限制最大矢量偏转角 */
#define TVC_SERVO_SLEW_US_S   8000.0f     /* 舵机斜率回退，保证响应速度 */

/*
 * 姿态配平偏移（度）：补偿 IMU 安装角度 / 重心偏移 / 舵机机械中位误差。
 * 正值 = 让机体往该轴正方向倾斜。
 * 调试方法：观察飞行器朝哪个方向飘，就往反方向加 trim。
 *   例：机体往"pitch 正方向"飘 → 把 PITCH_TRIM 往负方向调（如 -1.0）
 *             roll 往正方向飘 → trim 往负方向加
 *   每次调 0.5~1.0 度，逐步逼近。
 */ 
#define PITCH_TRIM_DEG   12.0f
#define ROLL_TRIM_DEG    -4.5f

/*
 * Yaw 差速符号：+delta → 左电机加速、右电机减速。
 * 如果起飞后机体仍然朝一个方向旋转，将此宏从 1 改为 -1 即可翻转。
 */
#define YAW_DIFF_SIGN  1

/*
 * 当前混控约定：
 * pitch: left -= pitch_us, right += pitch_us
 * roll : left -= roll_us,  right -= roll_us
 *
 * roll 方向已经按当前机构方向翻转过。  
 */
/* 外环：角度误差 -> 目标角速度（deg/s）— PI（加小量积分消除稳态漂移） */
static TVC_PID_t g_pitch_angle_pid = {
    .kp = 4.5f,
    .ki = 0.08f,
    .kd = 0.0f,
    .error_sum = 0.0f,
    .integral_max = 30.0f,
    .output_max = ANGLE_RATE_MAX_DPS,
    .target = PITCH_TRIM_DEG
};

static TVC_PID_t g_roll_angle_pid = {
    .kp = 4.5f,
    .ki = 0.08f,
    .kd = 0.0f,
    .error_sum = 0.0f,
    .integral_max = 30.0f,
    .output_max = ANGLE_RATE_MAX_DPS,
    .target = ROLL_TRIM_DEG
};

/* 内环：角速度误差 -> 舵机偏转角（deg）— PI */
static TVC_PID_t g_pitch_rate_pid = {
    .kp = 0.09f,
    .ki = 0.010f,
    .kd = 0.0f,
    .error_sum = 0.0f,
    .integral_max = 100.0f,
    .output_max = TVC_SERVO_MAX_DEG,
    .target = 0.0f
};

static TVC_PID_t g_roll_rate_pid = {
    .kp = 0.09f,
    .ki = 0.010f,
    .kd = 0.0f,
    .error_sum = 0.0f,
    .integral_max = 100.0f,
    .output_max = TVC_SERVO_MAX_DEG,
    .target = 0.0f
};

/* Yaw 角速率环 PID：降低 output_max 减少差速对 roll 的耦合干扰 */
static TVC_PID_t g_yaw_pid = {
    .kp = 0.80f,
    .ki = 0.25f,
    .kd = 0.008f,
    .error_sum = 0.0f,
    .integral_max = 150.0f,
    .output_max = 100.0f,
    .target = 0.0f
};
static float g_yaw_prev_error = 0.0f;

/* 主循环写入的基础油门，由 500Hz ISR 读取并叠加差速 */
static volatile uint16_t g_base_throttle_us = 1000;
static float g_left_cmd_us = SERVO_LEFT_CENTER_US;
static float g_right_cmd_us = SERVO_RIGHT_CENTER_US;

static float ClampFloat(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

void TVC_Init(void)
{
    /* 打开舵机外部 5V 供电。 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    g_left_cmd_us = SERVO_LEFT_CENTER_US;
    g_right_cmd_us = SERVO_RIGHT_CENTER_US;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)g_left_cmd_us);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)g_right_cmd_us);
}

void TVC_Update(float pitch, float roll, float gyro_y, float gyro_x, float gyro_z, float dt)
{
    float pitch_error;
    float roll_error;
    float pitch_rate_target;
    float roll_rate_target;
    float gyro_pitch_dps;
    float gyro_roll_dps;
    float pitch_rate_error;
    float roll_rate_error;
    float pitch_output;
    float roll_output;
    float pitch_us;
    float roll_us;
    float left_us;
    float right_us;
    float max_step_us;

    if (dt <= 0.0f) {
        return;
    }

    gyro_pitch_dps = gyro_y * RAD_TO_DEG_TVC;
    gyro_roll_dps  = gyro_x * RAD_TO_DEG_TVC;

    /* 小角度死区，降低地面与近地效应下的来回抖动 */
    pitch_error = g_pitch_angle_pid.target - pitch;
    roll_error  = g_roll_angle_pid.target - roll;
    if (pitch_error < ANGLE_DEADBAND_DEG && pitch_error > -ANGLE_DEADBAND_DEG) {
        pitch_error = 0.0f;
    }
    if (roll_error < ANGLE_DEADBAND_DEG && roll_error > -ANGLE_DEADBAND_DEG) {
        roll_error = 0.0f;
    }

    /* 外环：角度误差 -> 目标角速度（PI） */
    pitch_rate_target = g_pitch_angle_pid.kp * pitch_error;
    g_pitch_angle_pid.error_sum += pitch_error * dt;
    g_pitch_angle_pid.error_sum = ClampFloat(g_pitch_angle_pid.error_sum,
                                             -g_pitch_angle_pid.integral_max,
                                             g_pitch_angle_pid.integral_max);
    pitch_rate_target += g_pitch_angle_pid.ki * g_pitch_angle_pid.error_sum;
    pitch_rate_target = ClampFloat(pitch_rate_target,
                                   -g_pitch_angle_pid.output_max,
                                   g_pitch_angle_pid.output_max);

    roll_rate_target = g_roll_angle_pid.kp * roll_error;
    g_roll_angle_pid.error_sum += roll_error * dt;
    g_roll_angle_pid.error_sum = ClampFloat(g_roll_angle_pid.error_sum,
                                            -g_roll_angle_pid.integral_max,
                                            g_roll_angle_pid.integral_max);
    roll_rate_target += g_roll_angle_pid.ki * g_roll_angle_pid.error_sum;
    roll_rate_target = ClampFloat(roll_rate_target,
                                  -g_roll_angle_pid.output_max,
                                  g_roll_angle_pid.output_max);

    /* 内环：角速度误差 -> 舵机偏转角（PI） */
    pitch_rate_error = pitch_rate_target - gyro_pitch_dps;
    g_pitch_rate_pid.error_sum += pitch_rate_error * dt;
    g_pitch_rate_pid.error_sum = ClampFloat(g_pitch_rate_pid.error_sum,
                                            -g_pitch_rate_pid.integral_max,
                                            g_pitch_rate_pid.integral_max);
    pitch_output = (g_pitch_rate_pid.kp * pitch_rate_error)
                 + (g_pitch_rate_pid.ki * g_pitch_rate_pid.error_sum);
    pitch_output = ClampFloat(pitch_output,
                              -g_pitch_rate_pid.output_max,
                              g_pitch_rate_pid.output_max);

    roll_rate_error = roll_rate_target - gyro_roll_dps;
    g_roll_rate_pid.error_sum += roll_rate_error * dt;
    g_roll_rate_pid.error_sum = ClampFloat(g_roll_rate_pid.error_sum,
                                           -g_roll_rate_pid.integral_max,
                                           g_roll_rate_pid.integral_max);
    roll_output = (g_roll_rate_pid.kp * roll_rate_error)
                + (g_roll_rate_pid.ki * g_roll_rate_pid.error_sum);
    roll_output = ClampFloat(roll_output,
                             -g_roll_rate_pid.output_max,
                             g_roll_rate_pid.output_max);

    /* 舵机混控输出 */
    pitch_us = pitch_output * SERVO_US_PER_DEG;
    roll_us = roll_output * SERVO_US_PER_DEG;

    left_us = SERVO_LEFT_CENTER_US - pitch_us - roll_us;
    right_us = SERVO_RIGHT_CENTER_US + pitch_us - roll_us;

    left_us = ClampFloat(left_us, SERVO_LEFT_UP_US, SERVO_LEFT_DOWN_US);
    right_us = ClampFloat(right_us, SERVO_RIGHT_DOWN_US, SERVO_RIGHT_UP_US);

    /* 加斜率限制，避免每个2ms周期大幅跳舵 */
    max_step_us = TVC_SERVO_SLEW_US_S * dt;
    g_left_cmd_us += ClampFloat(left_us - g_left_cmd_us, -max_step_us, max_step_us);
    g_right_cmd_us += ClampFloat(right_us - g_right_cmd_us, -max_step_us, max_step_us);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)g_left_cmd_us);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)g_right_cmd_us);

    /* ── Yaw 角速率 PID → 电机差速（仅油门 >1050us 时生效） ── */
    {
        uint16_t base = g_base_throttle_us;

        if (base <= 1050U) {
            g_yaw_pid.error_sum = 0.0f;
            g_yaw_prev_error = 0.0f;
            ESC_PWM_SetPulseUs(base, base);
        } else {
            float yaw_rate_dps = gyro_z * RAD_TO_DEG_TVC;
            float yaw_error = g_yaw_pid.target - yaw_rate_dps;

            g_yaw_pid.error_sum += yaw_error * dt;
            g_yaw_pid.error_sum = ClampFloat(g_yaw_pid.error_sum,
                                             -g_yaw_pid.integral_max,
                                             g_yaw_pid.integral_max);

            float yaw_delta = (g_yaw_pid.kp * yaw_error)
                            + (g_yaw_pid.ki * g_yaw_pid.error_sum);
            /* D 项：跳过首次调用避免突刺 */
            if (g_yaw_prev_error != 0.0f) {
                yaw_delta += g_yaw_pid.kd * (yaw_error - g_yaw_prev_error) / dt;
            }
            g_yaw_prev_error = yaw_error;
            yaw_delta = ClampFloat(yaw_delta,
                                   -g_yaw_pid.output_max,
                                   g_yaw_pid.output_max);
            yaw_delta *= (float)YAW_DIFF_SIGN;

            int32_t left_esc  = (int32_t)base + (int32_t)yaw_delta;
            int32_t right_esc = (int32_t)base - (int32_t)yaw_delta;

            if (left_esc  < 1000) left_esc  = 1000;
            if (left_esc  > 2000) left_esc  = 2000;
            if (right_esc < 1000) right_esc = 1000;
            if (right_esc > 2000) right_esc = 2000;

            ESC_PWM_SetPulseUs((uint16_t)left_esc, (uint16_t)right_esc);
        }
    }
}

/**
 * @brief 舵机归中并清空 PID 积分，在解锁前或低油门时调用
 *        防止解锁瞬间积分饱和冲击。
 */
void TVC_Center(void)
{
    g_pitch_rate_pid.error_sum = 0.0f;
    g_roll_rate_pid.error_sum  = 0.0f;
    g_pitch_angle_pid.error_sum = 0.0f;
    g_roll_angle_pid.error_sum  = 0.0f;
    g_yaw_pid.error_sum        = 0.0f;
    g_yaw_prev_error             = 0.0f;
    g_left_cmd_us = SERVO_LEFT_CENTER_US;
    g_right_cmd_us = SERVO_RIGHT_CENTER_US;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)g_left_cmd_us);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)g_right_cmd_us);
}

void TVC_SetBaseThrottle(uint16_t us)
{
    if (us < 1000U) us = 1000U;
    if (us > 2000U) us = 2000U;
    g_base_throttle_us = us;
}

uint16_t TVC_GetBaseThrottle(void)
{
    return g_base_throttle_us;
}
