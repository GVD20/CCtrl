# CCtrl 嵌入式固件开源数据

[ZHCN](CCtrl_Embedded_Firmware_ZHCN.md) | [EN](CCtrl_Embedded_Firmware_EN.md)

本页汇总 CCtrl 固件相关的开源内容与关键入口。

## 主要文件与目录

- 构建配置：[platformio.ini](../platformio.ini)
- 主控业务固件：[src/master](../src/master)
- 节点固件：[src/node](../src/node)
- UI 运行入口：[src/ui_runtime](../src/ui_runtime)
- 协议与校验公共代码：[src/shared](../src/shared)
- 公共头文件：[include](../include)
- 本地库：[lib](../lib)

## 构建命令

- `platformio run -e node_328p`
- `platformio run -e master_esp32s3`

## 相关文档

- 用户手册（中文）：[CCtrl_User_Manual_ZHCN.md](CCtrl_User_Manual_ZHCN.md)
- User Manual (EN)：[CCtrl_User_Manual_EN.md](CCtrl_User_Manual_EN.md)
