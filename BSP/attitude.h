#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <stdint.h>

/* 姿态数据结构 */
typedef struct {
    float pitch;    /* 俯仰角 (度) */
    float roll;     /* 横滚角 (度) */
    float yaw;      /* 偏航角 (度，统一为 (-180, 180]) */
    float accel[3]; /* 加速度 x,y,z (m/s^2) */
    float gyro[3];  /* 角速度 x,y,z (rad/s) — 已去零偏 */
    float temp;     /* 温度 (°C) */
    float q[4];     /* 姿态四元数 [w,x,y,z] */
    float gyro_bias[3]; /* 陀螺仪零偏估计 (rad/s) */
    float dt;       /* 实际采样间隔 (s) */
    float mag_raw[3];   /* 磁力计原始数据 x,y,z (uT) — RM3100 */
    float mag_yaw;      /* RM3100 磁力计航向角参考 (度，统一为 (-180, 180]) */
    uint8_t mag_valid;  /* 磁力计数据有效标志 */
} attitude_data_t;

/* 函数声明 */
uint8_t Attitude_Init(void);
void Attitude_Update(void);
attitude_data_t *Attitude_GetData(void);
void Attitude_Display(void);
uint8_t Attitude_IsReady(void);

/**
 * @brief  设置磁力计航向角参考，用于修正 Mahony yaw 漂移
 * @param  yaw_deg  RM3100 提供的绝对航向角 (度，函数内部统一折返到 (-180, 180])
 * @param  valid    1 = 数据有效，0 = 无效（暂停修正）
 */
void Attitude_SetMagYaw(float yaw_deg, uint8_t valid);

/**
 * @brief  将任意 yaw 角折返到项目统一范围 (-180, 180]
 */
float Attitude_WrapYaw180(float yaw_deg);

/**
 * @brief  设置磁力计原始磁场数据（用于 ANO 协议上报）
 * @param  mag_x, mag_y, mag_z  磁场分量 (uT)
 */
void Attitude_SetMagRaw(float mag_x, float mag_y, float mag_z);

#endif
