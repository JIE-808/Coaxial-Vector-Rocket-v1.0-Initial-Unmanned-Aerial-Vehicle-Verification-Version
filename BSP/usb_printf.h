#ifndef __USB_PRINTF_H__
#define __USB_PRINTF_H__

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* 通过 USB CDC 输出格式化字符串 */
int usb_printf(const char *format, ...);

/* 初始化 USB printf 使用的发送缓冲区 */
void USB_Printf_Init(void);

/* 统一开关 USB 输出，低压保护时会关闭 */
void USB_Printf_SetEnabled(uint8_t enable);

#endif
