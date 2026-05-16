# CCtrl WebXR / XR-UART Workflow

这部分是当前仓库内已经落地的 Quest WebXR 前端、PC 本地 HTTPS/WSS 服务，以及 XR-UART 串口桥接工作流说明。

## 目录职责

- `src/main.ts`
  Quest/桌面端 Three.js + WebXR 前端，负责控制器姿态采样、XR 面板交互、手势按键、以及向本地服务发送 `pose_frame`
- `server/index.mjs`
  本地 HTTP/HTTPS + WebSocket 服务，可直接托管 `dist`，并把前端数据转成 96 字节 XR 二进制包后转发到本地桥端口
- `server/runtime-config.json`
  运行时桥接配置，当前用于设置 `bridgeHost`、`bridgePort`、`positionScale`、`rotationScale`
- `scripts/generate-dev-certs.mjs`
  生成本地开发证书，供 Quest Browser 使用 HTTPS/WSS
- `WebXR_link.py`
  面向端到端联调的 Textual 全屏工作流，负责串口切换、ADB 设备发现、`gnirehtet` 联网、证书检查、前端编译和本地服务拉起

## 当前功能

- 桌面预览和沉浸式会话共用一套前端
- 优先请求 `immersive-ar`，失败时回退到 `immersive-vr`
- 右手控制器输出姿态、位置、摇杆、Trigger / Squeeze / A / B / Thumbstick 状态
- 世界空间调试面板、按键面板、状态面板
- 右手 Thumbstick 按下后进行 XR 坐标系参考点调整
- A/B 组合切换面板，可配合 Squeeze 移动面板
- Trigger 映射为 `KEY5`
- 右手手势按键模式可生成 `KEY6` / `KEY7`
- 本地服务提供 `/health`、`/status`、`/runtime-config.json`
- 本地服务会把浏览器数据转发为 XR-UART 二进制包，桥接到本地 TCP 端口

## 数据链路

```text
Quest Browser / Desktop Browser
  -> WebSocket /ws
  -> webxr/server/index.mjs
  -> XR 96-byte relay packet
  -> local bridgeHost:bridgePort
  -> tools/preview_monitor.py 或其他本地 XR-UART bridge consumer
  -> ESP32-S3
```

前端发送的是 JSON `pose_frame`。本地服务负责：

- 维护会话状态和最近一帧摘要
- 统计接收/转发频率
- 应用 `positionScale` / `rotationScale`
- 输出固定 96 字节二进制 XR 包

## 前置依赖

### Node.js

用于前端构建、本地 HTTPS/WSS 服务、证书脚本。

### Python

- `WebXR_link.py` 依赖：`textual`、`rich`、`pyfiglet`
- Ubuntu 打包 / 部署时可直接使用 `requirements-webxr-link.txt`
- 如果要配合桌面监控台使用，还需要根目录 `tools/preview_monitor.py` 的依赖：`PyQt5`、`PyQt-Fluent-Widgets`

### Android / Quest 辅助工具

- Windows 下可直接放在 `webxr/platform-tools/`
- 或者让 `adb`、`gnirehtet` 进入 PATH

> `platform-tools/` 只是本地运行辅助目录，不属于仓库必需源码。

## Ubuntu Bundle

- Ubuntu 即开即用部署说明见 `UBUNTU_DEPLOY_ZHCN.md`
- Ubuntu bundle 打包器见 `ubuntu/package_ubuntu_bundle.py`
- Ubuntu 一键构建脚本见 `ubuntu/build_on_ubuntu.sh`
- `WebXR_link.py` 现支持 `--ui-scale` / `--ui-density` 以适配较小终端

## 常用命令

### 仅构建前端

```bash
npm install
npm run build
```

### 生成开发证书

```bash
npm run certs:generate
```

### 启动本地 HTTPS/WSS 服务

```bash
npm run serve
```

说明：

- `npm run serve` 会启动 `server/index.mjs --serve-dist`
- 若证书存在，则使用 HTTPS + WSS
- 若证书缺失，则会退到 HTTP + WS，但 Quest Browser 不能正常进入同源 WebXR 工作流

### 全流程工作台

```bash
python webxr/WebXR_link.py
```

可选参数：

```bash
python webxr/WebXR_link.py --serial-port COM6 --http-port 8787
python webxr/WebXR_link.py --android-serial <adb-serial>
```

这个全屏工作台会按当前实现处理以下事情：

- 选择并切换 USB 串口进入 XR-UART 模式
- 发现 Android / Quest 设备
- 尝试启动 `gnirehtet`
- 检查或生成 HTTPS 证书
- 首次启动时自动执行 `npm run build`
- 拉起本地 WebXR 服务并展示访问地址

## 与 `preview_monitor.py` 的关系

根目录的 `tools/preview_monitor.py` 现在不是单纯“只读预览器”，而是一站式桌面监控与 XR-UART 控制台。它负责：

- 串口帧解析与状态可视化
- 进入/退出 XR-UART
- 拉起本地 WebXR 服务
- 查询 `/status` 并显示桥接状态、最近帧和倍率配置

如果你已经在用 `preview_monitor.py`，它就是当前仓库默认的桌面入口；`WebXR_link.py` 更偏向 Quest 联调阶段的一体化 CLI/TUI 工作流。

## 运行时配置

`server/runtime-config.json` 当前支持：

```json
{
  "bridgeHost": "127.0.0.1",
  "bridgePort": 8786,
  "positionScale": 1.0,
  "rotationScale": 1.0
}
```

用途：

- `bridgeHost` / `bridgePort`：本地二进制桥目标
- `positionScale`：位置缩放
- `rotationScale`：旋转幅度缩放

前端会轮询 `/runtime-config.json`，用于显示当前倍率状态。

## 调试接口

- `/health`
  返回服务基本存活信息
- `/status`
  返回客户端数量、会话状态、桥接状态、访问地址、最新帧摘要
- `/runtime-config.json`
  返回当前运行时配置

## 说明

- 浏览器侧默认 WebSocket 地址会根据页面协议自动推导；HTTPS 页面会使用 `wss://<host>/ws`
- 如果 Quest Browser 报 `ERR_SSL_PROTOCOL_ERROR`，优先检查证书是否完整以及是否通过 HTTPS 访问
- 手势按键功能当前是源码内硬编码开关，由 `src/main.ts` 中的 `ENABLE_HAND_GESTURE_KEYS` 控制
