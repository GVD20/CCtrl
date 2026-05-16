#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${WEBXR_UBUNTU_VENV:-$ROOT/.venv-ubuntu-bundle}"
OUTPUT_DIR="${WEBXR_UBUNTU_OUT:-$ROOT/out}"
BUNDLE_NAME="${WEBXR_UBUNTU_BUNDLE_NAME:-cc-webxr-ubuntu-bundle}"
NODE_ROOT="${WEBXR_UBUNTU_NODE_ROOT:-}"
PLATFORM_TOOLS_DIR="${WEBXR_UBUNTU_PLATFORM_TOOLS_DIR:-}"

echo "[1/4] Create or refresh Ubuntu venv: $VENV_DIR"
"$PYTHON_BIN" -m venv "$VENV_DIR"
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "$ROOT/requirements-webxr-link.txt"

echo "[2/4] Install Node dependencies"
cd "$ROOT"
npm install

echo "[3/4] Build frontend"
npm run build

echo "[4/4] Assemble bundle"
ARGS=(
  python "$ROOT/ubuntu/package_ubuntu_bundle.py"
  --python-venv "$VENV_DIR"
  --output-dir "$OUTPUT_DIR"
  --bundle-name "$BUNDLE_NAME"
)

if [[ -n "$NODE_ROOT" ]]; then
  ARGS+=(--node-root "$NODE_ROOT")
fi
if [[ -n "$PLATFORM_TOOLS_DIR" ]]; then
  ARGS+=(--platform-tools-dir "$PLATFORM_TOOLS_DIR")
fi

"${ARGS[@]}"

echo "Bundle ready under: $OUTPUT_DIR/$BUNDLE_NAME"
