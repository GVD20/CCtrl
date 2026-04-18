# CCtrl 电子硬件资料

[ZHCN](README_ZHCN.md) | [EN](README_EN.md)

推荐入口：

- [../../docs/CCtrl_Hardware_Engineering_ZHCN.md](../../docs/CCtrl_Hardware_Engineering_ZHCN.md)
- [../../docs/CCtrl_Hardware_Engineering_EN.md](../../docs/CCtrl_Hardware_Engineering_EN.md)

## 目录说明

- `lceda_pro/CCtrl_Main.epro2`
  - 立创EDA Pro 原始工程（建议以此为基准）

- `altium_reference/CCtrl_Main_Altium_Export.zip`
  - 由立创EDA Pro 导出的 Altium Designer 工程存档

- `fabrication_reference/`
  - 制造参考文件，按板卡拆分：
    - `main_board/`
    - `encoder_node/`
    - `handle_node/`
    - `joystick/`
    - `roller/`
  - 每个子目录含 `gerber.zip` 与 `bom.csv`

## 复现声明

- Gerber、BOM、Altium 导出内容仅供参考。
- 复现生产前，请自行核对：
  - 封装与焊盘
  - 网络连接与层叠
  - BOM 料号与替代料
  - DRC/DFM 制造约束
