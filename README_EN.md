<h1 align="center">CCtrl</h1>

<p align="center">
  <strong>A spatial controller built on ATmega328P node chain and ESP32-S3 master</strong>
</p>

<p align="center">
  <a href="README.md">简体中文</a> · <a href="README_EN.md">English</a>
</p>

<p align="center">
  <img alt="Firmware" src="https://img.shields.io/badge/Firmware-PlatformIO-0A66C2?style=for-the-badge">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-2EA043?style=for-the-badge">
  <img alt="Output" src="https://img.shields.io/badge/Output-RS232%20%7C%20USB-7A3CF0?style=for-the-badge">
</p>

<p align="center">
  <img alt="CCtrl Render" src="hardware/mechanical/renders/CCtrl_Render_Cover.png">
</p>

CCtrl is a spatial controller project for 3D orientation and position interaction. This repository includes embedded firmware, hardware design assets, mechanical models, and host-side tooling for reproduction and customization.

<h5>▌Quick Links</h5>

| Module               | ZHCN                                                                       | EN                                                                     |
| -------------------- | -------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| User Manual          | [CCtrl_User_Manual_ZHCN](docs/CCtrl_User_Manual_ZHCN.md)                   | [CCtrl_User_Manual_EN](docs/CCtrl_User_Manual_EN.md)                   |
| Embedded Firmware    | [CCtrl_Embedded_Firmware_ZHCN](docs/CCtrl_Embedded_Firmware_ZHCN.md)       | [CCtrl_Embedded_Firmware_EN](docs/CCtrl_Embedded_Firmware_EN.md)       |
| Hardware Engineering | [CCtrl_Hardware_Engineering_ZHCN](docs/CCtrl_Hardware_Engineering_ZHCN.md) | [CCtrl_Hardware_Engineering_EN](docs/CCtrl_Hardware_Engineering_EN.md) |
| Mechanical Models    | [CCtrl_Mechanical_Models_ZHCN](docs/CCtrl_Mechanical_Models_ZHCN.md)       | [CCtrl_Mechanical_Models_EN](docs/CCtrl_Mechanical_Models_EN.md)       |
| Debug Tools          | [CCtrl_Debug_Tools_ZHCN](docs/CCtrl_Debug_Tools_ZHCN.md)                   | [CCtrl_Debug_Tools_EN](docs/CCtrl_Debug_Tools_EN.md)                   |

<h5>▌Highlights</h5>

- Distributed architecture with ESP32-S3 master and four ATmega328P nodes over daisy-chained UART.
- 6DoF capture using three magnetic encoders (AS5600) and one 9-axis IMU (ICM20948).
- Dual output interfaces: RS232 and USB.
- Auxiliary inputs: 7 buttons, one 2D joystick, and one wheel.
- Runtime options for REL/ABS position and Euler/Quaternion attitude output.

<h5>▌Repository Structure</h5>

```text
.
├─ include/
├─ lib/
├─ src/
│  ├─ node/
│  ├─ master/
│  ├─ shared/
│  └─ ui_runtime/
├─ tools/
│  └─ unified_preview_monitor.py
├─ docs/
├─ hardware/
│  ├─ electronics/
│  │  ├─ lceda_pro/
│  │  ├─ altium_reference/
│  │  └─ fabrication_reference/
│  └─ mechanical/
│     ├─ cad/
│     ├─ print/
│     └─ renders/
├─ platformio.ini
├─ LICENSE
├─ README.md
├─ README_ZHCN.md
└─ README_EN.md
```

<h5>▌Firmware Build</h5>

```bash
platformio run -e node_328p
platformio run -e master_esp32s3
platformio run -e master_esp32s3 -t upload
```

<h5>▌Host Preview Tool</h5>

```bash
python tools/unified_preview_monitor.py
```

<h5>▌PlatformIO Library Dependencies</h5>

node_328p:
- robtillaart/AS5600
- sparkfun/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library

master_esp32s3:
- frankboesing/FastCRC
- olikraus/U8g2

Local libraries:
- lib/WouoUiLiteGeneralBridge
- lib/WouoUiLiteGeneralOfficial

<h5>▌UI Framework Credit</h5>

This project references and adapts the UI framework from [RQNG/WouoUI: 模仿稚晖君MonoUI风格的超丝滑菜单，使用EC11旋转编码器控制。](https://github.com/RQNG/WouoUI).

<h5>▌License</h5>

This project is released under the MIT License. See [LICENSE](LICENSE).

<h5>▌Maintenance Notice</h5>

This repository may not be actively maintained, and pull requests may not be reviewed in time. If you plan to use it in production, perform your own validation, testing, and code review.

Made with 💗& AI
