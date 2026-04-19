#ifndef __VBAT_ADC_H__
#define __VBAT_ADC_H__

#include <stdint.h>

/* 电池状态枚举 */
typedef enum {
    VBAT_STATE_NORMAL = 0,
    VBAT_STATE_LOW,
    VBAT_STATE_CRITICAL
} VBAT_State_t;

/* 初始化电池电压采样模块，需在 MX_ADC1_Init() 之后调用 */
void VBAT_Init(void);

/* 读取并滤波电池电压，建议周期性调用 */
float VBAT_Read(void);

/* 获取最近一次滤波后的整包电压 */
float VBAT_GetVoltage(void);

/* 获取当前电池状态 */
VBAT_State_t VBAT_GetState(void);

/* 按 3S 工作区间估算电量百分比 */
float VBAT_GetPercent(void);

/* 在 LCD 上绘制手机样式的电池图标 */
void VBAT_Display(void);

/* 显示致命低压警告页，仅保留警告信息 */
void VBAT_ShowCriticalWarning(void);

/* 进入致命低压保护模式：停止所有执行机构输出并显示警告 */
void VBAT_EnterProtectionMode(void);

/* 查询是否已进入致命低压保护 */
uint8_t VBAT_IsShutdown(void);

#endif /* __VBAT_ADC_H__ */
