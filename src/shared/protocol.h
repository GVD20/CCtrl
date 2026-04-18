#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// --- 命令定义 ---
#define CMD_ADDR_RESET 0xA5
#define CMD_DATA_LOOP 0x8C
#define CMD_DISCONNECT_REPORT 0xE1

// --- 断联原因 ---
#define DISC_REASON_TIMEOUT 0x01
#define DISC_REASON_PROPAGATED 0x02

// --- 设备类型 ---
#define NODE_TYPE_UNKNOWN 0x00
#define NODE_TYPE_ENCODER 0x01
#define NODE_TYPE_HANDLE 0x02

// --- 数据结构 ---

// 1. Encoder 节点 (保持不变)
struct __attribute__((packed)) EncoderPayload {
  uint8_t nodeType; // 0x01
  uint8_t status;   // 0=OK, 1=NoMagnet
  uint16_t rawAngle;
  uint16_t reserved;
};

// 2. Handle 节点 (大幅扩充)
// 总大小: 1+1 + 6(Acc) + 6(Gyr) + 6(Mag) + 1(Btn) + 1(Wheel) + 1(JoyX) +
// 1(JoyY) = 24 Bytes
struct __attribute__((packed)) HandlePayload {
  uint8_t nodeType; // 0x02
  uint8_t status;   // 状态
  int16_t accX, accY, accZ;
  int16_t gyrX, gyrY, gyrZ;
  int16_t magX, magY, magZ;
  uint8_t btnState; // bit0..bit3 四个按钮当前状态位(1=按下)
  uint8_t wheelSeq; // AB相位累计计数(00~FF循环)
  uint8_t joyX;     // 0~100 (3.3V->0, 0V->100)
  uint8_t joyY;     // 0~100 (3.3V->0, 0V->100)
};

// 3. 断联报告帧（链路级，具备传播性）
// [CMD_DISCONNECT_REPORT][nodeId][reasonFlags][seq][crc]
struct __attribute__((packed)) DisconnectReportFrame {
  uint8_t cmd;
  uint8_t nodeId;
  uint8_t reasonFlags;
  uint8_t seq;
  uint8_t crc;
};

// --- CRC8 计算函数 (AVR 优化版) ---
// Poly: x^8 + x^5 + x^4 + 1 (0x31) - standard Dallas/Maxim
inline uint8_t calcCRC8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t extract = *data++;
    for (uint8_t tempI = 8; tempI; tempI--) {
      uint8_t sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

#endif