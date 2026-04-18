# CCtrl 空间控制器用户手册

[ZHCN](CCtrl_User_Manual_ZHCN.md) | [EN](CCtrl_User_Manual_EN.md)

本文档描述 CCtrl 当前仓库实现对应的使用方法与协议定义。

## 1. 项目定义

CCtrl 被定义为空间控制器，用于输出姿态、位置、按键、摇杆和滚轮等交互数据。

系统由两部分组成：
- 节点侧：ATmega328P，负责编码器/手柄采样与链路转发
- 主控侧：ESP32-S3，负责融合、校准、UI 与对外数据输出

## 2. 固件与运行结构

- `src/node`：328P 节点固件
- `src/master`：主控业务逻辑
- `src/ui_runtime`：主控 UI 运行入口（双核任务调度）
- `src/shared`：通信协议与 CRC 代码


## 3. 通信链路

### 3.1 对外链路

- USB CDC（`Serial`）：默认 2000000 bps
- RS232（`Serial2`）：默认 115200 bps

### 3.2 主控与节点链路

- UART1（`Serial1`）：250000 bps
- 链路命令：
  - `CMD_ADDR_RESET (0xA5)` 初始化/寻址
  - `CMD_DATA_LOOP (0x8C)` 数据轮询
  - `CMD_DISCONNECT_REPORT (0xE1)` 断联上报

## 4. 统一输出协议（0x0302）

### 4.1 帧结构

```text
SOF(0xA5)
+ data_length(2)
+ seq(1)
+ CRC8(1)
+ cmd_id(2)
+ data[30]
+ CRC16(2)
```

- `cmd_id` 固定为 `0x0302`
- `data_length` 固定为 `30`
- 多字节字段采用小端序

### 4.2 30 字节载荷定义

|   偏移 | 长度 | 字段         | 类型         | 说明                           |
| -----: | ---: | ------------ | ------------ | ------------------------------ |
|      0 |    1 | `status_err` | `uint8`      | 高4位状态码，低4位错误位       |
|   1..2 |    2 | `key_flags`  | `uint16`     | KEY1..KEY8 位图（低8位有效）   |
|      3 |    1 | `delta_key`  | `uint8`      | 增量触发键状态（等价于 KEY5）  |
|   4..5 |    2 | `wheel_pos`  | `int16`      | 滚轮累计位置                   |
|      6 |    1 | `joy_x`      | `uint8`      | 摇杆 X，0..100                 |
|      7 |    1 | `joy_y`      | `uint8`      | 摇杆 Y，0..100                 |
|  8..13 |    6 | `pos_h[3]`   | `float16[3]` | 位置向量                       |
| 14..21 |    8 | `att_h[4]`   | `float16[4]` | 姿态向量（欧拉或四元数）       |
|     22 |    1 | `mode_flags` | `uint8`      | 输出接口/姿态位置模式/姿态格式 |
| 23..28 |    6 | `enc_raw[3]` | `uint16[3]`  | 编码器校准后计数               |
|     29 |    1 | `reserved`   | `uint8`      | 保留，当前为0                  |

### 4.3 关键位定义

`status_err`：
- 状态码（高4位）：`0=IDLE`，`1=ACTIVE`，`2=NODE_DISCONNECT`
- 错误位（低4位）：`0x01 CRC`，`0x02 Timeout`，`0x04 Parse`，`0x08 Topology`

`mode_flags`：
- bit0：输出口，`0=RS232`，`1=USB`
- bit1：姿态/位置模式，`0=REL`，`1=ABS`
- bit2：姿态格式，`0=Euler`，`1=Quaternion`

### 4.4 `enc_raw` 语义

字段名虽为 `enc_raw`，但当前实现输出的是校准后的编码器值（已应用 offset 与方向修正）。

## 5. 输出语义

### 5.1 姿态输出

- Euler 模式：`att_h = [roll, pitch, yaw, 0]`
- Quaternion 模式：`att_h = [w, x, y, z]`

### 5.2 位置输出

- REL：相对参考点位移
- ABS：绝对解算位置

### 5.3 增量触发

`delta_key`（KEY5）作为增量触发键，控制相对姿态/位置窗口。

## 6. 设备侧配置与校准

设备菜单提供以下常用项：
- `Options`：切换 LinkOut、PoseOut、RotOut
- `Joystick Cali`：摇杆范围与死区校准
- `Enc Cali`：编码器零位/方向校准，支持保存与重载

校准结果由主控保存并参与运行时解算。

## 7. 上位机工具

当前仓库保留只读预览工具：

```bash
python tools/unified_preview_monitor.py
```

该工具仅用于数据显示与仿真预览。

## 8. 构建与烧录

```bash
# 328P 节点
platformio run -e node_328p

# ESP32-S3 主控
platformio run -e master_esp32s3

# 上传主控
platformio run -e master_esp32s3 -t upload
```

## 9. 兼容与说明

- 如需将本项目接入正式系统，请自行完成协议联调、可靠性验证与边界测试
