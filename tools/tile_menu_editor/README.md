# TILE Menu Editor

本地 Web 编辑器，用来修改 `include/tile_menu_config.h` 里的:

- 15 个磁贴槽位
- 磁贴名称
- `cmd` 映射值
- 30x30 二值图标
- 槽位顺序调整
- 图片导入并按阈值二值化

## 启动

```powershell
python tools\tile_menu_editor\server.py
```

默认地址:

```text
http://127.0.0.1:8765/?path=include/tile_menu_config.h
```

## 特点

- 不依赖第三方 Python 包
- 直接读写仓库内的头文件
- 图标编辑为 30x30 二值像素
- 可导入普通图片并缩放到 30x30 后二值化
- 可对当前槽位执行上移/下移调整顺序
- 页面会展示打包后的 `PROGMEM` 十六进制字节

## 说明

- 图标显示条件是: 名称非空且 `cmd != 0`
- 隐藏槽位不会在固件磁贴页显示
- 服务只允许读写当前仓库根目录下的文件
