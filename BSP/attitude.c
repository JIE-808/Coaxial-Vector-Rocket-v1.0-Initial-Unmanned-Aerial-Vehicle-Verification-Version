/**
 * @file  attitude.c
 * @brief Mahony AHRS + DWT + gyro bias calibration
 */
#include "attitude.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "imu_heat.h"
#include "lcd.h"
#include "vbat_adc.h"
#include "usb_printf.h"
#include "spi.h"
#include <math.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"

#define RAD_TO_DEG  57.295779513f
#define GRAVITY     9.80665f

/* Mahony PI gains */
#define MAHONY_KP       0.8f
#define MAHONY_KI       0.003f

/* Low-pass alpha for accel (0~1, smaller = smoother) */
#define ACC_LP_ALPHA    0.15f

/* Low-pass alpha for output euler angles (0~1, smaller = smoother) */
/* 提高到 0.8 以减小相位延迟，500Hz 下时间常数降至 ~0.5ms */
#define EULER_LP_ALPHA  0.8f

/* Dynamic Kp: fade accel correction when not ~1g (rocket accel phase) */
#define ACC_TRUST_BAND  1.5f   /* |acc_mag - g| within this => full Kp */
#define ACC_REJECT_BAND 4.0f   /* beyond this => Kp = 0 */

/* Gyro outlier rejection threshold (rad/s) — ~300 deg/s */
#define GYRO_SPIKE_THRESH 5.24f
/* Gyro low-pass alpha for spike detection */
#define GYRO_LP_ALPHA     0.2f

/* ==================== state ==================== */
static attitude_data_t att_data = {0};
static uint8_t bmi088_ready = 0;

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
static float gyro_bias[3] = {0};
static float acc_lp[3] = {0};
static float gyro_prev[3] = {0};  /* previous valid gyro for outlier rejection */
static float euler_lp[3] = {0};   /* low-pass filtered euler output */

static uint32_t last_tick = 0;
static uint8_t first_run = 1;

/* ==================== DWT ==================== */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static float DWT_GetDeltaSec(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t delta = now - last_tick;
    last_tick = now;
    return (float)delta / (float)SystemCoreClock;
}

/* ==================== Mahony core ==================== */
static void Mahony_Update(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float acc_sq = ax*ax + ay*ay + az*az;
    if (acc_sq > 0.01f) {
        recipNorm = 1.0f / sqrtf(acc_sq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* estimated gravity from quaternion (DCM 3rd column * 0.5) */
        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = q0*q0 - 0.5f + q3*q3;

        /* error = measured x estimated */
        halfex = ay*halfvz - az*halfvy;
        halfey = az*halfvx - ax*halfvz;
        halfez = ax*halfvy - ay*halfvx;

        /* dynamic gain: trust accel only near 1g, fade out under thrust */
        float acc_mag = sqrtf(acc_sq);
        float acc_err = fabsf(acc_mag - GRAVITY);
        float kp_scale;
        if (acc_err < ACC_TRUST_BAND) {
            kp_scale = 1.0f;
        } else if (acc_err < ACC_REJECT_BAND) {
            kp_scale = 1.0f - (acc_err - ACC_TRUST_BAND) / (ACC_REJECT_BAND - ACC_TRUST_BAND);
        } else {
            kp_scale = 0.0f;
        }

        if (kp_scale > 0.001f) {
            float kp_eff = MAHONY_KP * kp_scale;
            float ki_eff = MAHONY_KI * kp_scale;
            integralFBx += ki_eff * halfex * dt;
            integralFBy += ki_eff * halfey * dt;
            integralFBz += ki_eff * halfez * dt;
            gx += integralFBx + kp_eff * halfex;
            gy += integralFBy + kp_eff * halfey;
            gz += integralFBz + kp_eff * halfez;
        } else {
            /* still apply integral term for continuity */
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
    }

    /* quaternion integration */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}

/* ==================== quat -> euler ==================== */
static void Quat_ToEuler(float *pitch, float *roll, float *yaw)
{
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * RAD_TO_DEG;
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                    1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2),
                    1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}

/* ==================== init ==================== */
uint8_t Attitude_Init(void)
{
    uint8_t error;

    usb_printf("SPI2 State: %d\r\n", hspi2.State);
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    uint8_t tx, rx;
    HAL_StatusTypeDef spi_ret;

    tx = 0x80;
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    tx = 0x55; HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    tx = 0x55; spi_ret = HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    usb_printf("ACC ChipID: 0x%02X (ret=%d)\r\n", rx, spi_ret);

    tx = 0x80;
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    tx = 0x55; spi_ret = HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
    usb_printf("GYRO ChipID: 0x%02X (ret=%d)\r\n", rx, spi_ret);

    error = BMI088_init();
    if (error != BMI088_NO_ERROR) return error;
    bmi088_ready = 1;

    DWT_Init();

    /* gyro bias calibration - keep still for 2.5s */
    usb_printf("Gyro calibrating...\r\n");
    float bias_sum[3] = {0};
    float g[3], a[3], t;
    const int N = 500;
    for (int i = 0; i < N; i++) {
        BMI088_read(g, a, &t);
        bias_sum[0] += g[0];
        bias_sum[1] += g[1];
        bias_sum[2] += g[2];
        HAL_Delay(5);
    }
    gyro_bias[0] = bias_sum[0] / (float)N;
    gyro_bias[1] = bias_sum[1] / (float)N;
    gyro_bias[2] = bias_sum[2] / (float)N;
    usb_printf("Bias: %.5f %.5f %.5f\r\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    /* init quaternion from last accel reading */
    float norm_a = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (norm_a > 0.1f) {
        float ax = a[0]/norm_a, ay = a[1]/norm_a, az = a[2]/norm_a;
        float p = asinf(-ax);
        float r = atan2f(ay, az);
        float cp = cosf(p*0.5f), sp = sinf(p*0.5f);
        float cr = cosf(r*0.5f), sr = sinf(r*0.5f);
        q0 = cp*cr;  q1 = cp*sr;  q2 = sp*cr;  q3 = -sp*sr;
        float nq = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
        q0/=nq; q1/=nq; q2/=nq; q3/=nq;
    }

    acc_lp[0] = a[0]; acc_lp[1] = a[1]; acc_lp[2] = a[2];

    /* init euler low-pass with current quaternion angles */
    Quat_ToEuler(&euler_lp[0], &euler_lp[1], &euler_lp[2]);

    last_tick = DWT->CYCCNT;
    first_run = 1;
    usb_printf("Mahony AHRS init OK\r\n");
    return 0;
}

/* ==================== update ==================== */
void Attitude_Update(void)
{
    if (!bmi088_ready) return;

    float g[3], a[3], temp;
    BMI088_read(g, a, &temp);

    float dt;
    if (first_run) {
        dt = 0.01f;
        last_tick = DWT->CYCCNT;
        first_run = 0;
    } else {
        dt = DWT_GetDeltaSec();
    }
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f)  dt = 0.05f;

    /* accel low-pass */
    acc_lp[0] += ACC_LP_ALPHA * (a[0] - acc_lp[0]);
    acc_lp[1] += ACC_LP_ALPHA * (a[1] - acc_lp[1]);
    acc_lp[2] += ACC_LP_ALPHA * (a[2] - acc_lp[2]);

    /* remove static bias */
    float gx = g[0] - gyro_bias[0];
    float gy = g[1] - gyro_bias[1];
    float gz = g[2] - gyro_bias[2];

    /* gyro outlier rejection: if sudden spike >> previous, use previous */
    if (!first_run) {
        if (fabsf(gx - gyro_prev[0]) > GYRO_SPIKE_THRESH) gx = gyro_prev[0];
        if (fabsf(gy - gyro_prev[1]) > GYRO_SPIKE_THRESH) gy = gyro_prev[1];
        if (fabsf(gz - gyro_prev[2]) > GYRO_SPIKE_THRESH) gz = gyro_prev[2];
    }
    /* low-pass update for next spike detection */
    gyro_prev[0] += GYRO_LP_ALPHA * (gx - gyro_prev[0]);
    gyro_prev[1] += GYRO_LP_ALPHA * (gy - gyro_prev[1]);
    gyro_prev[2] += GYRO_LP_ALPHA * (gz - gyro_prev[2]);

    Mahony_Update(gx, gy, gz, acc_lp[0], acc_lp[1], acc_lp[2], dt);

    float raw_pitch, raw_roll, raw_yaw;
    Quat_ToEuler(&raw_pitch, &raw_roll, &raw_yaw);

    /* euler output low-pass filter to suppress residual noise */
    euler_lp[0] += EULER_LP_ALPHA * (raw_pitch - euler_lp[0]);
    euler_lp[1] += EULER_LP_ALPHA * (raw_roll  - euler_lp[1]);
    euler_lp[2] += EULER_LP_ALPHA * (raw_yaw   - euler_lp[2]);
    att_data.pitch = euler_lp[0];
    att_data.roll  = euler_lp[1];
    att_data.yaw   = euler_lp[2];

    att_data.accel[0] = acc_lp[0];
    att_data.accel[1] = acc_lp[1];
    att_data.accel[2] = acc_lp[2];
    att_data.gyro[0] = gx;
    att_data.gyro[1] = gy;
    att_data.gyro[2] = gz;
    att_data.temp = temp;
    att_data.q[0] = q0; att_data.q[1] = q1;
    att_data.q[2] = q2; att_data.q[3] = q3;
    att_data.gyro_bias[0] = gyro_bias[0] - integralFBx;
    att_data.gyro_bias[1] = gyro_bias[1] - integralFBy;
    att_data.gyro_bias[2] = gyro_bias[2] - integralFBz;
    att_data.dt = dt;
}

/* ==================== get ==================== */
attitude_data_t *Attitude_GetData(void)
{
    return &att_data;
}

uint8_t Attitude_IsReady(void)
{
    return bmi088_ready;
}

/* ==================== display ==================== */
void Attitude_Display(void)
{
    char buf[32];

    LCD_ShowString((LCD_W - ((sizeof("Mahony AHRS") - 1U) * 8U)) / 2U,
                   5,
                   (uint8_t *)"Mahony AHRS",
                   BLUE, WHITE, 16, 0);

    LCD_ShowString(10, 30, (uint8_t *)"PIT", RED, WHITE, 24, 0);
    snprintf(buf, sizeof(buf), "%+7.2f", att_data.pitch);
    LCD_ShowString(50, 30, (uint8_t *)buf, RED, WHITE, 24, 0);
    LCD_ShowString(160, 35, (uint8_t *)"o", RED, WHITE, 16, 0);

    LCD_ShowString(10, 60, (uint8_t *)"ROL", GREEN, WHITE, 24, 0);
    snprintf(buf, sizeof(buf), "%+7.2f", att_data.roll);
    LCD_ShowString(50, 60, (uint8_t *)buf, GREEN, WHITE, 24, 0);
    LCD_ShowString(160, 65, (uint8_t *)"o", GREEN, WHITE, 16, 0);

    LCD_ShowString(10, 90, (uint8_t *)"YAW", BLUE, WHITE, 24, 0);
    snprintf(buf, sizeof(buf), "%+7.2f", att_data.yaw);
    LCD_ShowString(50, 90, (uint8_t *)buf, BLUE, WHITE, 24, 0);
    LCD_ShowString(160, 95, (uint8_t *)"o", BLUE, WHITE, 16, 0);

    LCD_ShowString(10, 120, (uint8_t *)"--------------------", GRAY, WHITE, 16, 0);

    snprintf(buf, sizeof(buf), "dt:%.1fms %dHz", att_data.dt*1000.0f,
             (att_data.dt > 0.001f) ? (int)(1.0f/att_data.dt+0.5f) : 0);
    LCD_ShowString(10, 140, (uint8_t *)buf, BLACK, WHITE, 16, 0);

    snprintf(buf, sizeof(buf), "A%+5.1f %+5.1f %+5.1f", att_data.accel[0], att_data.accel[1], att_data.accel[2]);
    LCD_ShowString(10, 160, (uint8_t *)buf, BLACK, WHITE, 16, 0);

    snprintf(buf, sizeof(buf), "G%+5.2f %+5.2f %+5.2f", att_data.gyro[0], att_data.gyro[1], att_data.gyro[2]);
    LCD_ShowString(10, 180, (uint8_t *)buf, BLACK, WHITE, 16, 0);

    snprintf(buf, sizeof(buf), "T:%.1f/%.0fC %s H:%2.0f%%      ",
             att_data.temp, IMU_Heat_GetTarget(),
             IMU_Heat_IsStable() ? "OK" : "Wait",
             IMU_Heat_GetDuty());
    LCD_ShowString(10, 205, (uint8_t *)buf, MAGENTA, WHITE, 16, 0);
}
