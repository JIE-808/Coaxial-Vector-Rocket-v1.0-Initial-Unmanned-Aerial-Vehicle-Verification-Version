#ifndef __ALTITUDE_HOLD_H__
#define __ALTITUDE_HOLD_H__

#include <stdint.h>

/**
 * @brief  初始化定高控制器，复位 PID 状态
 */
void AltHold_Init(void);

/**
 * @brief  将遥控器油门杆量（SBUS 原始值）映射为目标高度
 *         杆量范围 300~1700 线性映射到 ALTHOLD_MIN_CM ~ ALTHOLD_MAX_CM
 * @param  sbus_throttle  CH3 原始 SBUS 值 (300~1700)
 */
void AltHold_SetTargetFromStick(int sbus_throttle);

/**
 * @brief  直接设置目标高度 (cm)
 */
void AltHold_SetTarget_cm(float target_cm);

/**
 * @brief  获取当前目标高度 (cm)
 */
float AltHold_GetTarget_cm(void);

/**
 * @brief  运行一次定高 PID，计算并写入基础油门
 *         应在主循环中每次超声波数据更新后调用（~100ms 周期）
 * @param  actual_cm     超声波实测高度 (cm)
 * @param  sonar_valid   超声波数据是否有效
 */
void AltHold_Update(float actual_cm, uint8_t sonar_valid);

/**
 * @brief  重置积分项（解锁时调用，防止积分饱和起飞飞走）
 */
void AltHold_ResetIntegral(void);

/**
 * @brief  获取当前 PID 输出的油门修正量 (μs, 可正可负)
 */
float AltHold_GetPIDOutput(void);

#endif /* __ALTITUDE_HOLD_H__ */
