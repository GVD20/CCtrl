# WebXR Ubuntu 即开即用部署方案

这份说明对应当前仓库里的：

- Quest / Browser WebXR 前端
- `webxr/server/index.mjs` 本地 HTTPS/WSS 服务
- `webxr/WebXR_link.py` 串口 / ADB / `gnirehtet` / 证书 / 服务一体化 TUI

目标是把它整理成一个可以在 Ubuntu 上直接解压运行的 bundle。

## 先说结论

当前仓库已经补齐了两部分能力：

1. `webxr/WebXR_link.py`
   现在支持 TUI 缩放和紧凑布局：
   - `--ui-scale 0.85`
   - `--ui-density compact`
   - 也支持环境变量 `CCWEBXR_TUI_SCALE` / `CCWEBXR_TUI_DENSITY`
   - 当终端太小而你没有手动指定时，会自动退到更紧凑的布局

2. `webxr/ubuntu/package_ubuntu_bundle.py`
   这是 Ubuntu bundle 打包器。它会把当前可运行的前后端、Python venv、可选的 Node 运行时、可选的 Linux `adb/gnirehtet` 一起打进一个目录，并可输出 `.tar.gz`。

## 为什么最终包建议在 Ubuntu 上组装

真正“即开即用”的 Ubuntu 包，需要下面这些都是 Linux 版本：

- Python venv
- `node_modules`
- 可选的 Node 运行时
- 可选的 `adb`
- 可选的 `gnirehtet`

这些内容不能直接用当前 Windows 产物替代，所以推荐流程是：

1. 在 Ubuntu 上准备一次运行环境
2. 在 Ubuntu 上执行打包器
3. 得到一个可复制到其他 Ubuntu 机器的 bundle

也就是说：

- Windows 侧负责继续开发源码
- Ubuntu 侧负责组装最终运行包

## 推荐目录结果

打包完成后会得到类似下面的结构：

```text
cc-webxr-ubuntu-bundle/
  webxr/
    WebXR_link.py
    dist/
    server/
    scripts/
    certs/
    src/
    node_modules/
    runtime/
      venv/
      node/                 # 如果你选择一并打包 Node
    platform-tools/         # 如果你选择一并打包 Linux adb/gnirehtet
    cc-webxr.env
    run-webxr-link.sh
    run-webxr-service.sh
    regenerate-certs.sh
```

## Ubuntu 组装步骤

下面是推荐的最稳方案。

### 1. 在 Ubuntu 上准备仓库

把当前仓库同步到 Ubuntu。

### 2. 准备 Python 运行环境

在 `webxr/` 旁边或任意你习惯的位置创建 Ubuntu venv，然后安装固定依赖：

```bash
python3 -m venv .venv-webxr-link
source .venv-webxr-link/bin/activate
python -m pip install --upgrade pip
python -m pip install -r webxr/requirements-webxr-link.txt
```

这份依赖已经固定在：

- [requirements-webxr-link.txt](/abs/path/d:/Dev/PIO/328P-START/webxr/requirements-webxr-link.txt)

### 3. 准备 Node 侧并生成前端

如果你只是要运行而不需要在线开发，建议仍然在 Ubuntu 上做一次：

```bash
cd webxr
npm install
npm run build
```

这样打出来的 `node_modules` 和 `dist` 就是 Ubuntu 原生版本。

如果你想少敲命令，也可以直接用：

```bash
chmod +x webxr/ubuntu/build_on_ubuntu.sh
webxr/ubuntu/build_on_ubuntu.sh
```

它会自动：

- 创建 / 刷新 Ubuntu venv
- 安装 `requirements-webxr-link.txt`
- 执行 `npm install`
- 执行 `npm run build`
- 最后调用 `package_ubuntu_bundle.py`

### 4. 准备可选的 Node 运行时

如果你希望 bundle 解压后不依赖目标机系统 PATH 里的 Node，那么准备一个 Linux Node 目录，例如：

```text
~/runtime/node-vXX-linux-x64/
  bin/node
  bin/npm
  ...
```

打包器会把这个目录整体复制进 bundle 的 `runtime/node/`。

如果你不传 `--node-root`，最终 bundle 会回退到目标机系统 PATH 里的 `node/npm`。

## 5. 准备可选的 Linux adb / gnirehtet

如果你希望 Quest / ADB / `gnirehtet` 也一起打包，就准备一个 Linux 目录，例如：

```text
~/runtime/platform-tools-linux/
  adb
  gnirehtet
  gnirehtet.apk
```

打包器会把它复制到 bundle 的 `webxr/platform-tools/`。

如果你不传这个目录，目标机就需要自己在 PATH 中提供 `adb` / `gnirehtet`。

## 6. 执行打包

在 Ubuntu 上运行：

```bash
python webxr/ubuntu/package_ubuntu_bundle.py \
  --python-venv /path/to/.venv-webxr-link \
  --node-root /path/to/node-linux-root \
  --platform-tools-dir /path/to/platform-tools-linux \
  --output-dir /path/to/out \
  --bundle-name cc-webxr-ubuntu-bundle
```

如果你暂时不打包 Node / ADB / `gnirehtet`，可以只传最小参数：

```bash
python webxr/ubuntu/package_ubuntu_bundle.py \
  --python-venv /path/to/.venv-webxr-link
```

生成后会得到：

- 展开的 bundle 目录
- 对应的 `.tar.gz`

## 目标 Ubuntu 机器如何运行

解压后进入：

```bash
cd cc-webxr-ubuntu-bundle/webxr
```

### 启动一体化 TUI

```bash
./run-webxr-link.sh
```

### 只启动 HTTPS/WSS 服务

```bash
./run-webxr-service.sh
```

### 重新生成证书

```bash
./regenerate-certs.sh
```

## TUI 分辨率太小时怎么调

这是这次专门补进去的能力。

### 方式 1：命令行临时调整

```bash
./run-webxr-link.sh --ui-scale 0.85 --ui-density compact
```

建议经验值：

- 终端偏小：`--ui-scale 0.90`
- 很小：`--ui-scale 0.85 --ui-density compact`
- 特别挤：`--ui-scale 0.75 --ui-density compact`

### 方式 2：改环境文件，长期生效

编辑 bundle 里的：

- `cc-webxr.env`

例如：

```bash
CCWEBXR_TUI_SCALE=0.85
CCWEBXR_TUI_DENSITY=compact
```

### 方式 3：自动紧凑模式

如果你没有手动指定参数，而当前 bash / terminal 的字符尺寸比较小，`WebXR_link.py` 会自动退到更紧凑的布局。

## 我建议你采用的实际部署路径

如果你的目标是“发给另一台 Ubuntu 机器，解压就跑”，我建议按下面做：

1. 在一台 Ubuntu 构建机上准备：
   - Ubuntu Python venv
   - Ubuntu `node_modules`
   - `dist`
   - 可选的 Linux Node 目录
   - 可选的 Linux `adb/gnirehtet`
2. 用 `package_ubuntu_bundle.py` 打成 bundle
3. 把生成的 `.tar.gz` 发给目标机
4. 目标机解压后直接运行 `./run-webxr-link.sh`
5. 如果终端太小，就改 `cc-webxr.env`

## 这次仓库里新增的关键文件

- [WebXR_link.py](/abs/path/d:/Dev/PIO/328P-START/webxr/WebXR_link.py)
  已支持 `--ui-scale` / `--ui-density` 和自动紧凑布局
- [requirements-webxr-link.txt](/abs/path/d:/Dev/PIO/328P-START/webxr/requirements-webxr-link.txt)
  固定了 Python TUI 依赖版本
- [package_ubuntu_bundle.py](/abs/path/d:/Dev/PIO/328P-START/webxr/ubuntu/package_ubuntu_bundle.py)
  负责生成 Ubuntu 即开即用 bundle

## 当前边界

这次我已经把“可执行的 Ubuntu 打包方案”和“TUI 可调分辨率”都落进仓库了。

但要得到真正可运行的 Ubuntu 原生 bundle，最后一步仍然需要在 Ubuntu 上做一次组装，因为：

- Python venv 需要 Ubuntu 版本
- Node 运行时最好是 Ubuntu 版本
- `adb/gnirehtet` 也最好是 Ubuntu 版本

这是平台二进制本身的限制，不是这套脚本的问题。
