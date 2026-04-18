# CCtrl Spatial Controller User Manual

[ZHCN](CCtrl_User_Manual_ZHCN.md) | [EN](CCtrl_User_Manual_EN.md)

This document describes the usage and protocol definitions implemented in the current CCtrl repository.

## 1. Project Definition

CCtrl is defined as a spatial controller for orientation, position, key, joystick, and wheel interaction output.

The system has two major firmware parts:
- Node side: ATmega328P, responsible for encoder/handle sampling and link forwarding
- Master side: ESP32-S3, responsible for fusion, calibration, UI, and external output

## 2. Firmware and Runtime Layout

- src/node: 328P node firmware
- src/master: master business logic
- src/ui_runtime: UI runtime entry (dual-core scheduling)
- src/shared: protocol and CRC code

## 3. Communication Links

### 3.1 External Links

- USB CDC (Serial): default 2000000 bps
- RS232 (Serial2): default 115200 bps

### 3.2 Master-to-Node Link

- UART1 (Serial1): 250000 bps
- Link commands:
  - CMD_ADDR_RESET (0xA5): initialization/addressing
  - CMD_DATA_LOOP (0x8C): data polling
  - CMD_DISCONNECT_REPORT (0xE1): disconnect report

## 4. Unified Output Protocol (0x0302)

### 4.1 Frame Structure

```text
SOF(0xA5)
+ data_length(2)
+ seq(1)
+ CRC8(1)
+ cmd_id(2)
+ data[30]
+ CRC16(2)
```

- cmd_id is fixed to 0x0302
- data_length is fixed to 30
- multi-byte fields are little-endian

### 4.2 30-byte Payload Definition

| Offset | Length | Field      | Type       | Description                                |
| -----: | -----: | ---------- | ---------- | ------------------------------------------ |
|      0 |      1 | status_err | uint8      | high nibble status, low nibble error flags |
|   1..2 |      2 | key_flags  | uint16     | KEY1..KEY8 bitmap (low 8 bits used)        |
|      3 |      1 | delta_key  | uint8      | delta trigger key state (same as KEY5)     |
|   4..5 |      2 | wheel_pos  | int16      | accumulated wheel position                 |
|      6 |      1 | joy_x      | uint8      | joystick X, 0..100                         |
|      7 |      1 | joy_y      | uint8      | joystick Y, 0..100                         |
|  8..13 |      6 | pos_h[3]   | float16[3] | position vector                            |
| 14..21 |      8 | att_h[4]   | float16[4] | attitude vector (Euler or Quaternion)      |
|     22 |      1 | mode_flags | uint8      | output interface/pose mode/attitude format |
| 23..28 |      6 | enc_raw[3] | uint16[3]  | calibrated encoder counts                  |
|     29 |      1 | reserved   | uint8      | reserved, currently 0                      |

### 4.3 Key Bit Definitions

status_err:
- status code (high nibble): 0=IDLE, 1=ACTIVE, 2=NODE_DISCONNECT
- error bits (low nibble): 0x01 CRC, 0x02 Timeout, 0x04 Parse, 0x08 Topology

mode_flags:
- bit0: output interface, 0=RS232, 1=USB
- bit1: pose mode, 0=REL, 1=ABS
- bit2: attitude format, 0=Euler, 1=Quaternion

### 4.4 enc_raw Semantics

Although the field is named enc_raw, current firmware outputs calibrated encoder values (offset and direction already applied).

## 5. Output Semantics

### 5.1 Attitude Output

- Euler mode: att_h = [roll, pitch, yaw, 0]
- Quaternion mode: att_h = [w, x, y, z]

### 5.2 Position Output

- REL: relative displacement against reference
- ABS: absolute solved position

### 5.3 Delta Trigger

delta_key (KEY5) acts as the trigger for relative pose/position windows.

## 6. Device-side Configuration and Calibration

Common menu entries:
- Options: switch LinkOut, PoseOut, RotOut
- Joystick Cali: joystick range and deadzone calibration
- Enc Cali: encoder zero/direction calibration with save/reload

Calibration results are stored on-device and applied in runtime output.

## 7. Host Tool

The repository keeps one read-only host preview tool:

```bash
python tools/unified_preview_monitor.py
```

This tool is for data display and simulation preview only.

## 8. Build and Upload

```bash
# 328P node
platformio run -e node_328p

# ESP32-S3 master
platformio run -e master_esp32s3

# Upload master
platformio run -e master_esp32s3 -t upload
```

## 9. Notes

If you integrate this project into production systems, perform your own protocol integration, reliability validation, and edge-case testing.
