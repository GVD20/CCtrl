# CCtrl Electronics Documentation

[ZHCN](README_ZHCN.md) | [EN](README_EN.md)

Recommended entry:

- [../../docs/CCtrl_Hardware_Engineering_ZHCN.md](../../docs/CCtrl_Hardware_Engineering_ZHCN.md)
- [../../docs/CCtrl_Hardware_Engineering_EN.md](../../docs/CCtrl_Hardware_Engineering_EN.md)

## Directory Overview

- lceda_pro/CCtrl_Main.epro2
  - Original LCSC EDA Pro project (recommended baseline)

- altium_reference/CCtrl_Main_Altium_Export.zip
  - Altium Designer archive exported from LCSC EDA Pro

- fabrication_reference/
  - Fabrication reference files grouped by PCB:
    - main_board/
    - encoder_node/
    - handle_node/
    - joystick/
    - roller/
  - Each folder contains gerber.zip and bom.csv

## Reproduction Notice

- Gerber, BOM, and Altium export files are for reference only.
- Before manufacturing, verify at least:
  - footprint and pad definitions
  - net connectivity and stackup
  - BOM part numbers and substitutes
  - DRC/DFM constraints
