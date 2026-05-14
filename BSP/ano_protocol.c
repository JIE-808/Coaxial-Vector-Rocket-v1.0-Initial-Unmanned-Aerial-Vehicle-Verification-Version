#include "ano_protocol.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define ANO_HEAD     0xAA
#define ANO_BUF_MAX  64

extern USBD_HandleTypeDef hUsbDeviceHS;

static uint8_t ano_buf[ANO_BUF_MAX];

/* 延迟发送缓冲：ISR 中构建帧，主循环中安全发出，避免 USB TxState 抢占冲突 */
static volatile uint8_t ano_tx_pending;
static uint8_t           ano_tx_buf[ANO_BUF_MAX];
static uint16_t          ano_tx_len;

/*
 * V7 协议底层发送：
 *   帧结构: HEAD(0xAA) + D_ADDR + ID + LEN + DATA[] + SC + AC
 *   SC = 从 HEAD 累加到 DATA 末尾，取低 8 位
 *   AC = 每次累加 SC 时，将当前 SC 再累加，取低 8 位
 */
static void ANO_Send_Frame(uint8_t d_addr, uint8_t id,
                           const uint8_t *data, uint8_t len)
{
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
        return;
    }

    uint8_t sc = 0;
    uint8_t ac = 0;
    uint8_t idx = 0;

    /* HEAD */
    ano_buf[idx] = ANO_HEAD;
    sc += ano_buf[idx];  ac += sc;  idx++;

    /* D_ADDR */
    ano_buf[idx] = d_addr;
    sc += ano_buf[idx];  ac += sc;  idx++;

    /* ID */
    ano_buf[idx] = id;
    sc += ano_buf[idx];  ac += sc;  idx++;

    /* LEN */
    ano_buf[idx] = len;
    sc += ano_buf[idx];  ac += sc;  idx++;

    /* DATA */
    for (uint8_t i = 0; i < len; i++) {
        ano_buf[idx] = data[i];
        sc += ano_buf[idx];  ac += sc;  idx++;
    }

    /* SC + AC */
    ano_buf[idx++] = sc;
    ano_buf[idx++] = ac;

    CDC_Transmit_HS(ano_buf, idx);
}

/* 延迟版：构建完整 V7 帧到 ano_tx_buf，只设标志，不调 USB 发送
   安全在 ISR 中使用，避免抢占主循环的 CDC_Transmit_HS */
static void ANO_Send_Frame_Deferred(uint8_t d_addr, uint8_t id,
                                    const uint8_t *data, uint8_t len)
{
    uint8_t sc = 0;
    uint8_t ac = 0;
    uint8_t idx = 0;

    ano_tx_buf[idx] = ANO_HEAD;    sc += ano_tx_buf[idx]; ac += sc; idx++;
    ano_tx_buf[idx] = d_addr;      sc += ano_tx_buf[idx]; ac += sc; idx++;
    ano_tx_buf[idx] = id;          sc += ano_tx_buf[idx]; ac += sc; idx++;
    ano_tx_buf[idx] = len;         sc += ano_tx_buf[idx]; ac += sc; idx++;

    for (uint8_t i = 0; i < len; i++) {
        ano_tx_buf[idx] = data[i];
        sc += ano_tx_buf[idx]; ac += sc; idx++;
    }
    ano_tx_buf[idx++] = sc;
    ano_tx_buf[idx++] = ac;

    ano_tx_len = idx;
    ano_tx_pending = 1U;
}

/* ── 小端写入辅助 ── */
static uint8_t WrI16(uint8_t *buf, int16_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
    return 2;
}

static uint8_t WrI32(uint8_t *buf, int32_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
    buf[2] = (uint8_t)((v >> 16) & 0xFF);
    buf[3] = (uint8_t)((v >> 24) & 0xFF);
    return 4;
}

/* ==================== 0x03 欧拉角姿态帧 ==================== */
void ANO_Send_Attitude(float pitch_deg, float roll_deg, float yaw_deg,
                       uint8_t fusion_sta)
{
    uint8_t data[7];
    uint8_t off = 0;

    off += WrI16(&data[off], (int16_t)(roll_deg  * 100.0f));
    off += WrI16(&data[off], (int16_t)(pitch_deg * 100.0f));
    off += WrI16(&data[off], (int16_t)(yaw_deg   * 100.0f));
    data[off++] = fusion_sta;

    ANO_Send_Frame(ANO_ADDR_PC, ANO_ID_EULER, data, off);
}

/* ==================== 0x01 传感器原始数据帧 ==================== */
void ANO_Send_Senser(const float accel[3], const float gyro[3])
{
#define G_MS2       9.80665f
#define RAD_TO_DEG  57.295779513f

    int16_t ax = (int16_t)(accel[0] / G_MS2 * 1000.0f);
    int16_t ay = (int16_t)(accel[1] / G_MS2 * 1000.0f);
    int16_t az = (int16_t)(accel[2] / G_MS2 * 1000.0f);

    int16_t gx = (int16_t)(gyro[0] * RAD_TO_DEG * 1000.0f);
    int16_t gy = (int16_t)(gyro[1] * RAD_TO_DEG * 1000.0f);
    int16_t gz = (int16_t)(gyro[2] * RAD_TO_DEG * 1000.0f);

    int16_t mx = 0, my = 0, mz = 0;

    uint8_t data[18];
    uint8_t off = 0;

    off += WrI16(&data[off], ax);
    off += WrI16(&data[off], ay);
    off += WrI16(&data[off], az);
    off += WrI16(&data[off], gx);
    off += WrI16(&data[off], gy);
    off += WrI16(&data[off], gz);
    off += WrI16(&data[off], mx);
    off += WrI16(&data[off], my);
    off += WrI16(&data[off], mz);

    ANO_Send_Frame(ANO_ADDR_PC, ANO_ID_SENSER, data, off);
}

/* ==================== 合并发送：0x03 欧拉 + 0x01 传感器（一次 USB 传输）==================== */
/*
 * 核心优化：连续调用 ANO_Send_Frame 时第二次会因 TxState 忙而被静默丢弃。
 * 此函数将两帧拼入同一缓冲区，只调一次 CDC_Transmit_HS，保证两帧都发出。
 * 总长: 13 (Euler) + 24 (Senser) = 37 字节，远在 ano_buf[64] 内。
 */
void ANO_Send_AttitudeAndSenser(float pitch_deg, float roll_deg, float yaw_deg,
                                uint8_t fusion_sta,
                                const float accel[3], const float gyro[3])
{
#define G_MS2       9.80665f
#define RAD_TO_DEG  57.295779513f

    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
        return;

    uint8_t sc, ac;
    uint8_t idx = 0;

    /* ═══ 帧 1: 0x03 欧拉角 ═══ */
    sc = 0; ac = 0;
    ano_buf[idx] = ANO_HEAD;   sc += ano_buf[idx]; ac += sc; idx++;  /* HEAD */
    ano_buf[idx] = ANO_ADDR_PC; sc += ano_buf[idx]; ac += sc; idx++;  /* ADDR */
    ano_buf[idx] = ANO_ID_EULER;sc += ano_buf[idx]; ac += sc; idx++;  /* ID   */
    ano_buf[idx] = 7;          sc += ano_buf[idx]; ac += sc; idx++;  /* LEN=7*/

    int16_t val;
    val = (int16_t)(roll_deg  * 100.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;

    val = (int16_t)(pitch_deg * 100.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;

    val = (int16_t)(yaw_deg   * 100.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;

    ano_buf[idx] = fusion_sta; sc += ano_buf[idx]; ac += sc; idx++;   /* FUSION_STA */
    ano_buf[idx] = sc;        idx++;  /* SC */
    ano_buf[idx] = ac;        idx++;  /* AC */

    /* ═══ 帧 2: 0x01 传感器 ═══ */
    sc = 0; ac = 0;
    ano_buf[idx] = ANO_HEAD;    sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = ANO_ADDR_PC;  sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = ANO_ID_SENSER;sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = 18;          sc += ano_buf[idx]; ac += sc; idx++;  /* LEN=18 */

    /* ACC mg */
    val = (int16_t)(accel[0] / G_MS2 * 1000.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    val = (int16_t)(accel[1] / G_MS2 * 1000.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    val = (int16_t)(accel[2] / G_MS2 * 1000.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;

    /* GYRO mdps */
    val = (int16_t)(gyro[0] * RAD_TO_DEG * 1000.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    val = (int16_t)(gyro[1] * RAD_TO_DEG * 1000.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    val = (int16_t)(gyro[2] * RAD_TO_DEG * 1000.0f);
    ano_buf[idx] = (uint8_t)(val & 0xFF); sc += ano_buf[idx]; ac += sc; idx++;
    ano_buf[idx] = (uint8_t)((val>>8)&0xFF); sc += ano_buf[idx]; ac += sc; idx++;

    /* MAG = 0 */
    for (int i = 0; i < 6; i++) {
        ano_buf[idx] = 0; sc += 0; ac += sc; idx++;
    }

    ano_buf[idx] = sc; idx++;   /* SC */
    ano_buf[idx] = ac; idx++;   /* AC */

    CDC_Transmit_HS(ano_buf, idx);
}

/* ==================== 0x00 ACK 帧（延迟发送版） ==================== */
static void ANO_Send_ACK_Deferred(uint8_t to_addr,
                                   uint8_t rx_id, uint8_t rx_len,
                                   const uint8_t *rx_data)
{
    uint8_t data[64];
    uint8_t off = 0;
    data[off++] = rx_id;
    data[off++] = rx_len;
    for (uint8_t i = 0; i < rx_len; i++)
        data[off++] = rx_data[i];

    ANO_Send_Frame_Deferred(to_addr, ANO_ID_ACK, data, off);
}

/* ==================== V7 帧接收解析 ==================== */
/*
 * 从 USB CDC 收到数据后调用（ISR 上下文）。
 * 用延迟发送缓冲构建响应帧，不直接调 CDC_Transmit_HS。
 *
 * 帧类型处理：
 *   0xE0 命令       → 0x00 ACK
 *   0xE1 参数读取   → 0xE1 回复 param_id + int32(0)
 *   0xE2 参数写入   → 0x00 ACK
 *   其他            → 忽略
 */
void ANO_ProcessRx(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i + 6 <= len; )  /* 最短帧 = 6 (无 DATA) */
    {
        if (data[i] != ANO_HEAD) {
            i++;
            continue;
        }

        uint8_t d_addr = data[i + 1];
        uint8_t id     = data[i + 2];
        uint8_t dlen   = data[i + 3];

        uint32_t frame_len = 4U + dlen + 2U;
        if (i + frame_len > len) {
            i++;
            continue;
        }

        /* 校验 SC */
        uint8_t sc = 0;
        for (uint32_t j = 0; j < 4U + dlen; j++)
            sc += data[i + j];
        if (sc != data[i + 4 + dlen]) {
            i++;
            continue;
        }

        if (d_addr == ANO_ADDR_FC || d_addr == ANO_ADDR_PC) {
            const uint8_t *rxd = &data[i + 4];

            switch (id) {

            case ANO_ID_CMD:   /* 0xE0 — 命令 */
                ANO_Send_ACK_Deferred(d_addr, id, dlen, rxd);
                break;

            case ANO_ID_PARAM_READ:   /* 0xE1 — 参数读取 */
                {
                    /* 回复 0xE1：param_id(uint16 LE) + value(int32 LE=0) = 6 字节 */
                    uint8_t resp[6];
                    uint8_t off = 0;
                    resp[off++] = (dlen >= 1) ? rxd[0] : 0;  /* param_id lo */
                    resp[off++] = (dlen >= 2) ? rxd[1] : 0;  /* param_id hi */
                    off += WrI32(&resp[off], 0);             /* value = 0 */
                    ANO_Send_Frame_Deferred(d_addr, ANO_ID_PARAM_READ, resp, off);
                }
                break;

            case ANO_ID_PARAM_WRITE:  /* 0xE2 — 参数写入 */
                ANO_Send_ACK_Deferred(d_addr, id, dlen, rxd);
                break;

            default:
                /* 0x01/0x03 等数据帧：不回复 */
                break;
            }
        }

        i += frame_len;
    }
}

/* ==================== 延迟发送服务 ==================== */
/* 主循环每次迭代调用，将 ISR 中缓冲的响应帧安全发出 */
void ANO_Service_Deferred(void)
{
    if (!ano_tx_pending)
        return;

    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
        ano_tx_pending = 0U;
        return;
    }

    if (CDC_Transmit_HS(ano_tx_buf, ano_tx_len) == USBD_OK)
        ano_tx_pending = 0U;
}

/* ==================== 初始化 ==================== */
void ANO_Init(void)
{
    memset(ano_buf, 0, sizeof(ano_buf));
    ano_tx_pending = 0U;
    ano_tx_len = 0U;
}
