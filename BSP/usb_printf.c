#include "usb_printf.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define USB_PRINTF_BUFFER_SIZE  256

/* USB 设备句柄由 USB 设备库提供 */
extern USBD_HandleTypeDef hUsbDeviceHS;

static uint8_t usb_txbuf[USB_PRINTF_BUFFER_SIZE];
/* 统一输出使能标志，低压保护时会被关闭 */
static uint8_t usb_printf_enabled = 1;

int usb_printf(const char *format, ...)
{
    va_list args;
    int len;

    /* 被上层关闭后直接丢弃输出 */
    if (!usb_printf_enabled) {
        return -1;
    }

    /* USB 还未枚举成功时不发送 */
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
        return -1;
    }

    /* 先格式化到本地缓冲区 */
    va_start(args, format);
    len = vsnprintf((char *)usb_txbuf, USB_PRINTF_BUFFER_SIZE - 1, format, args);
    va_end(args);

    if (len < 0 || len >= USB_PRINTF_BUFFER_SIZE) {
        len = USB_PRINTF_BUFFER_SIZE - 1;
    }

    /* 通过 USB CDC 发送 */
    uint8_t result = CDC_Transmit_HS(usb_txbuf, len);
    uint32_t timeout = 100000;
    while (result == USBD_BUSY && timeout--) {
        result = CDC_Transmit_HS(usb_txbuf, len);
    }

    if (result != USBD_OK) {
        return -1;
    }

    return len;
}

void USB_Printf_Init(void)
{
    /* 清空发送缓冲，并默认打开输出 */
    memset(usb_txbuf, 0, USB_PRINTF_BUFFER_SIZE);
    usb_printf_enabled = 1;
}

void USB_Printf_SetEnabled(uint8_t enable)
{
    /* 仅保存使能状态，不主动操作 USB 外设 */
    usb_printf_enabled = enable ? 1U : 0U;
}
