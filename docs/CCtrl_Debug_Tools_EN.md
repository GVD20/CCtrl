# CCtrl Debug Tools

[ZHCN](CCtrl_Debug_Tools_ZHCN.md) | [EN](CCtrl_Debug_Tools_EN.md)

This page summarizes the current debug, preview, XR-UART, and WebXR tool entry points in the repository.

## Main Files and Directories

- Desktop monitor and XR-UART control surface:
  - [tools/preview_monitor.py](../tools/preview_monitor.py)
- WebXR workflow overview:
  - [webxr/README.md](../webxr/README.md)
- Quest / ADB / cert / local-service all-in-one TUI:
  - [webxr/WebXR_link.py](../webxr/WebXR_link.py)
- Protocol and CRC reference implementation:
  - [src/shared/protocol.h](../src/shared/protocol.h)
  - [src/shared/crc.h](../src/shared/crc.h)
  - [src/shared/crc.c](../src/shared/crc.c)

## Run

- `python tools/preview_monitor.py`
- `python webxr/WebXR_link.py`

## Notes

- `preview_monitor.py` is no longer a read-only previewer; it is the default desktop monitor and XR-UART entry point
- `webxr/README.md` documents the Quest Browser, HTTPS/WSS, local bridge port, and runtime scaling workflow
- `WebXR_link.py` is the integrated TUI for serial switching, ADB device discovery, `gnirehtet`, certificates, and the frontend service

## Related Docs

- User Manual (ZHCN): [CCtrl_User_Manual_ZHCN.md](CCtrl_User_Manual_ZHCN.md)
- User Manual (EN): [CCtrl_User_Manual_EN.md](CCtrl_User_Manual_EN.md)
