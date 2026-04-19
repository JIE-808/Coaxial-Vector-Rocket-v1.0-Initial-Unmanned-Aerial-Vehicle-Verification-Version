#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include <stdint.h>

/**
 * @brief  初始化 HC-SR04 超声波模块
 *         Trig: PB10 (GPIO Output)
 *         Echo: PB11 / TIM2_CH4 (Input Capture)
 */
void Ultrasonic_Init(void);

/**
 * @brief  发送一次 10μs 触发脉冲并启动输入捕获
 *         建议在主循环中以 ≥60ms 间隔调用
 */
void Ultrasonic_Trigger(void);

/**
 * @brief  处理超时：若触发后长时间未收到回波则重置状态
 *         在主循环中每次都调用即可
 */
void Ultrasonic_CheckTimeout(void);

/**
 * @brief  获取最近一次有效测距结果 (cm)
 */
float Ultrasonic_GetDistance_cm(void);

/**
 * @brief  最近一次测量是否有效
 */
uint8_t Ultrasonic_IsValid(void);

/**
 * @brief  校零：把当前滤波输出记为地面基准（高度=0）
 *         建议在解锁时调用，确保读数稳定后再执行
 */
void Ultrasonic_Calibrate(void);

/**
 * @brief  在 LCD 上显示距离（放在 dt 行右侧）
 */
void Ultrasonic_Display(void);

#endif /* __ULTRASONIC_H__ */
