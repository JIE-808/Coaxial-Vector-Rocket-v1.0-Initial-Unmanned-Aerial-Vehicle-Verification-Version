/**
 * @file  rm3100.c
 * @brief RM3100 — UART7 DMA+Idle, 115200, PE7(RX)/PE8(TX)
 *
 * Init: 发 AT+FILT=10 (轻度平滑) + AT+PRATE=50 (20Hz 自动回传)
 * Runtime: 纯被动接收 DMA，不主动查询。
 * ISR 回调中逐字节解析，主循环读结果。
 */
#include "rm3100.h"
#include "usart.h"
#include "usb_printf.h"

#define RM3100_LINE_LEN  80

static char     g_line_buf[RM3100_LINE_LEN];
static uint8_t  g_line_idx;
static volatile uint8_t g_data_valid;
static RM3100_Data_t g_data;

/* ── 手写 atof ── */
static float my_atof(const char *s)
{
    float v = 0.0f, f = 0.0f, d = 1.0f;
    int sign = 1;
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') { s++; while (*s >= '0' && *s <= '9') { f = f * 10.0f + (float)(*s - '0'); d *= 10.0f; s++; } }
    return sign * (v + f / d);
}

/* ── 跳过前缀 ── */
static const char *skip_pfx(const char *s, const char *pfx)
{
    while (*pfx && *s == *pfx) { s++; pfx++; }
    return (*pfx == '\0') ? s : NULL;
}

/* ═══ DMA 批量解析（ISR 安全） ═══ */
void RM3100_ProcessBuffer(const uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        uint8_t ch = data[i];
        if (ch == '\n') {
            if (g_line_idx > 10 && g_line_idx < RM3100_LINE_LEN) {
                g_line_buf[g_line_idx] = '\0';
                const char *p = g_line_buf;

                if ((p = skip_pfx(p, "Magx:")) != NULL) {
                    float mx = my_atof(p);
                    while (*p && *p != ',') { p++; }
                    if (*p == ',') { p++; }

                    if ((p = skip_pfx(p, "Magy:")) != NULL) {
                        float my = my_atof(p);
                        while (*p && *p != ',') { p++; }
                        if (*p == ',') { p++; }

                        if ((p = skip_pfx(p, "Magz:")) != NULL) {
                            float mz = my_atof(p);

                            int found_yaw = 0;
                            float yw = 0.0f;
                            while (*p) {
                                if (p[0]==',' && p[1]=='Y' && p[2]=='a' && p[3]=='w' && p[4]==':') {
                                    yw = my_atof(p + 5); found_yaw = 1; break;
                                }
                                p++;
                            }
                            if (found_yaw) {
                                /* 统一规范化到 (-180, 180]，供姿态融合、PID、显示直接使用 */
                                while (yw > 180.0f)  yw -= 360.0f;
                                while (yw <= -180.0f) yw += 360.0f;
                                g_data.mag_x = mx; g_data.mag_y = my;
                                g_data.mag_z = mz; g_data.yaw   = yw;
                                g_data_valid = 1;
                            }
                        }
                    }
                }
            }
            g_line_idx = 0;
        } else if (ch != '\r' && g_line_idx < (RM3100_LINE_LEN - 1)) {
            g_line_buf[g_line_idx++] = (char)ch;
        }
    }
}

/* ═══ Public API ═══ */
uint8_t RM3100_IsDataValid(void)     { return g_data_valid; }
void    RM3100_GetData(RM3100_Data_t *out) { if (out) *out = g_data; }
void    RM3100_ClearDataValid(void)  { g_data_valid = 0; }
void    RM3100_ParsePending(void)    {}
void    RM3100_RequestData(void)     {}  /* 模块自动回传，无需手动查询 */

uint8_t RM3100_Init(void)
{
    g_data_valid = 0;
    g_line_idx   = 0;

    /* 1. 验证连接 */
    usb_printf("[RM3100] Init: AT...\r\n");
    HAL_UART_Transmit(&huart7, (uint8_t *)"AT\r\n", 4, 100);
    HAL_Delay(100);

    /* 2. 轻度平滑滤波 */
    usb_printf("[RM3100] Init: FILT=10...\r\n");
    HAL_UART_Transmit(&huart7, (uint8_t *)"AT+FILT=10\r\n", 13, 100);
    HAL_Delay(100);

    /* 3. 20Hz (50ms) 自动回传 */
    usb_printf("[RM3100] Init: PRATE=50 (20Hz)...\r\n");
    HAL_UART_Transmit(&huart7, (uint8_t *)"AT+PRATE=50\r\n", 14, 100);
    HAL_Delay(100);

    usb_printf("[RM3100] Init OK — 20Hz auto-report, FILT=10\r\n");
    return 0;
}
