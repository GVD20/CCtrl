# CCtrl 调试工具

[ZHCN](CCtrl_Debug_Tools_ZHCN.md) | [EN](CCtrl_Debug_Tools_EN.md)

本页汇总当前仓库内调试、预览、XR-UART 与 WebXR 联调相关工具入口。

## 主要文件与目录

- 桌面监控与 XR-UART 控制台：
  - [tools/preview_monitor.py](../tools/preview_monitor.py)
- WebXR 工作流说明：
  - [webxr/README.md](../webxr/README.md)
- Quest / ADB / 证书 / 本地服务一体化 TUI：
  - [webxr/WebXR_link.py](../webxr/WebXR_link.py)
- 协议与 CRC 参考实现：
  - [src/shared/protocol.h](../src/shared/protocol.h)
  - [src/shared/crc.h](../src/shared/crc.h)
  - [src/shared/crc.c](../src/shared/crc.c)

## 启动方式

- `python tools/preview_monitor.py`
- `python webxr/WebXR_link.py`

## 说明

- `preview_monitor.py` 现在不是只读预览器，而是桌面侧默认监控与 XR-UART 入口
- `webxr/README.md` 说明 Quest Browser、HTTPS/WSS、本地桥端口和运行时倍率配置
- `WebXR_link.py` 适合串起串口切换、ADB 设备发现、`gnirehtet`、证书和前端服务

## 相关文档

- 用户手册（中文）：[CCtrl_User_Manual_ZHCN.md](CCtrl_User_Manual_ZHCN.md)
- User Manual (EN)：[CCtrl_User_Manual_EN.md](CCtrl_User_Manual_EN.md)
