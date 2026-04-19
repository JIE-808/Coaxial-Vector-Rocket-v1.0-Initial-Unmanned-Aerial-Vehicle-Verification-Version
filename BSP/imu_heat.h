#ifndef __IMU_HEAT_H__
#define __IMU_HEAT_H__

#include <stdint.h>

/**
 * @brief  初始化 IMU 加热 PID 控制
 *         启动 TIM3_CH4 PWM 输出
 */
void IMU_Heat_Init(void);

/**
 * @brief  PID 恒温控制，每个主循环周期调用一次
 * @param  current_temp  BMI088 当前温度 (°C)
 */
void IMU_Heat_Update(float current_temp);

/**
 * @brief  获取目标温度 (°C)
 */
float IMU_Heat_GetTarget(void);

/**
 * @brief  获取当前 PWM 占空比 (0~100%)
 */
float IMU_Heat_GetDuty(void);

/**
 * @brief  判断温度是否已经稳定（到达目标温度附近）
 * @return 1 = 已稳定, 0 = 未稳定
 */
uint8_t IMU_Heat_IsStable(void);

#endif
