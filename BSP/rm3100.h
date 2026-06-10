/**
 * @file  rm3100.h
 * @brief RM3100 — UART7 DMA+Idle, 115200, PE7(RX)/PE8(TX)
 */
#ifndef __RM3100_H__
#define __RM3100_H__

#include <stdint.h>

typedef struct {
    float mag_x, mag_y, mag_z;
    float yaw;              /* 航向角 (度，统一为 (-180, 180]) */
} RM3100_Data_t;

uint8_t RM3100_Init(void);
void    RM3100_RequestData(void);
void    RM3100_ProcessBuffer(const uint8_t *data, uint16_t size);
uint8_t RM3100_IsDataValid(void);
void    RM3100_GetData(RM3100_Data_t *out);
float   RM3100_GetYaw(void);
void    RM3100_ClearDataValid(void);
void    RM3100_ParsePending(void);    /* 解析已在 ISR 中完成 */

#endif
