#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include <stdint.h>

/* 姿态数据结构 */
typedef struct {
    float pitch;    /* 俯仰角 (度) */
    float roll;     /* 横滚角 (度) */
    float yaw;      /* 偏航角 (度) */
    float accel[3]; /* 加速度 x,y,z (m/s^2) */
    float gyro[3];  /* 角速度 x,y,z (rad/s) — 已去零偏 */
    float temp;     /* 温度 (°C) */
    float q[4];     /* 姿态四元数 [w,x,y,z] */
    float gyro_bias[3]; /* 陀螺仪零偏估计 (rad/s) */
    float dt;       /* 实际采样间隔 (s) */
} attitude_data_t;

/* 函数声明 */
uint8_t Attitude_Init(void);
void Attitude_Update(void);
attitude_data_t *Attitude_GetData(void);
void Attitude_Display(void);
uint8_t Attitude_IsReady(void);

#endif
