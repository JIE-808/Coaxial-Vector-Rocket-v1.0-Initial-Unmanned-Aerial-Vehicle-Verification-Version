#ifndef __ANO_PROTOCOL_H__
#define __ANO_PROTOCOL_H__

#include <stdint.h>

/* ── 匿名通信协议 V7 帧定义 ── */

/* 目标地址 */
#define ANO_ADDR_PC     0xFF  /* 广播 / 上位机 */
#define ANO_ADDR_FC     0x05  /* 飞控 */

/* 功能码 */
#define ANO_ID_ACK          0x00  /* 数据校验 / ACK 回复 */
#define ANO_ID_SENSER       0x01  /* 原始传感器数据 (ACC+GYRO+MAG) */
#define ANO_ID_EULER        0x03  /* 飞控姿态：欧拉角（驱动 3D 模型） */
#define ANO_ID_QUAT         0x04  /* 飞控姿态：四元数 */
#define ANO_ID_RCDATA       0x40  /* 遥控器通道数据 */
#define ANO_ID_CMD          0xE0  /* 命令帧 */
#define ANO_ID_PARAM_READ   0xE1  /* 参数读取 请求/响应 */
#define ANO_ID_PARAM_WRITE  0xE2  /* 参数写入 */

/* ── 欧拉角姿态帧 (0x03) ──
 *   DATA 共 7 字节：
 *     ROL  int16  ×100  横滚角
 *     PIT  int16  ×100  俯仰角
 *     YAW  int16  ×100  偏航角
 *     FUSION_STA uint8  融合状态 (0/1)
 *   帧总长 = 1(HEAD) + 1(D_ADDR) + 1(ID) + 1(LEN) + 7(DATA) + 1(SC) + 1(AC) = 13 */
void ANO_Send_Attitude(float pitch_deg, float roll_deg, float yaw_deg,
                       uint8_t fusion_sta);

/* 合并发送 0x03 欧拉角 + 0x01 传感器，一次 USB 传输避免丢帧 */
void ANO_Send_AttitudeAndSenser(float pitch_deg, float roll_deg, float yaw_deg,
                                uint8_t fusion_sta,
                                const float accel[3], const float gyro[3]);

/* ── 传感器原始数据帧 (0x01) ──
 *   DATA 共 18 字节：
 *     ACC_X/Y/Z  int16  (mg = m/s^2 / 9.80665 * 1000)
 *     GYRO_X/Y/Z int16  (mdps = deg/s * 1000)
 *     MAG_X/Y/Z  int16  (无磁力计填 0) */
void ANO_Send_Senser(const float accel[3], const float gyro[3]);

/* 接收处理：从 USB CDC 收到数据后调用，解析 V7 帧并自动回复 ACK */
void ANO_ProcessRx(const uint8_t *data, uint32_t len);

/* 主循环中调用：将 ISR 中缓冲的响应帧安全发出 */
void ANO_Service_Deferred(void);

/* 初始化 */
void ANO_Init(void);

#endif
