# CCtrl Embedded Firmware Open Data

[ZHCN](CCtrl_Embedded_Firmware_ZHCN.md) | [EN](CCtrl_Embedded_Firmware_EN.md)

This page summarizes open-source firmware assets and key entry points.

## Main Files and Directories

- Build configuration: [platformio.ini](../platformio.ini)
- Master firmware: [src/master](../src/master)
- Node firmware: [src/node](../src/node)
- UI runtime entry: [src/ui_runtime](../src/ui_runtime)
- Shared protocol and CRC code: [src/shared](../src/shared)
- Shared headers: [include](../include)
- Local libraries: [lib](../lib)

## Build Commands

- `platformio run -e node_328p`
- `platformio run -e master_esp32s3`

## Related Docs

- User Manual (ZHCN): [CCtrl_User_Manual_ZHCN.md](CCtrl_User_Manual_ZHCN.md)
- User Manual (EN): [CCtrl_User_Manual_EN.md](CCtrl_User_Manual_EN.md)
