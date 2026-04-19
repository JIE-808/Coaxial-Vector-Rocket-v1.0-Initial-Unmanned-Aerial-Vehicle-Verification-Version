#ifndef __TVC_PID_H__
#define __TVC_PID_H__

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float error_sum;
    float integral_max;
    float output_max;

    float target;
} TVC_PID_t;

void TVC_Init(void);
void TVC_Center(void);
void TVC_Update(float pitch, float roll, float gyro_y, float gyro_x, float gyro_z, float dt);
void TVC_SetBaseThrottle(uint16_t us);
uint16_t TVC_GetBaseThrottle(void);

#endif /* __TVC_PID_H__ */
