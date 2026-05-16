#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import stat
import tarfile
import textwrap
import time
from pathlib import Path
from typing import Iterable, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
WEBXR_DIR = SCRIPT_DIR.parent
DEFAULT_OUTPUT_DIR = WEBXR_DIR / "out"

DIRS_TO_COPY = (
    "certs",
    "dist",
    "node_modules",
    "scripts",
    "server",
    "src",
)

FILES_TO_COPY = (
    "UBUNTU_DEPLOY_ZHCN.md",
    "WebXR_link.py",
    "README.md",
    "index.html",
    "package-lock.json",
    "package.json",
    "requirements-webxr-link.txt",
    "tsconfig.json",
    "vite.config.ts",
)

IGNORE_PATTERNS = shutil.ignore_patterns(
    "__pycache__",
    "*.pyc",
    "*.pyo",
    ".DS_Store",
    ".webxr_link_state.json",
    "server.err.log",
    "server.out.log",
)


def ensure_exists(path: Path, label: str) -> None:
    if not path.exists():
        raise SystemExit(f"Missing {label}: {path}")


def ensure_executable(path: Path) -> None:
    mode = path.stat().st_mode
    path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def copy_path(src: Path, dst: Path) -> None:
    if src.is_dir():
        shutil.copytree(src, dst, symlinks=True, ignore=IGNORE_PATTERNS)
    else:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)


def detect_local_platform_tools() -> Optional[Path]:
    candidate = WEBXR_DIR / "platform-tools"
    if not candidate.exists():
        return None
    adb = candidate / "adb"
    gnirehtet = candidate / "gnirehtet"
    if adb.exists() and gnirehtet.exists():
        return candidate
    return None


def write_text(path: Path, content: str, *, executable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8", newline="\n")
    if executable:
        ensure_executable(path)


def env_template() -> str:
    return textwrap.dedent(
        """\
        # Optional TUI tuning for smaller terminals.
        # Examples:
        #   CCWEBXR_TUI_SCALE=0.85
        #   CCWEBXR_TUI_DENSITY=compact
        CCWEBXR_TUI_SCALE=1.0
        CCWEBXR_TUI_DENSITY=normal

        # Node service defaults.
        HOST=0.0.0.0
        PORT=8787

        # Extra certificate SANs when re-issuing certs on the Ubuntu host.
        # CERT_DNS_NAMES=ubuntu-webxr.local
        # CERT_IPS=192.168.1.88
        """
    )


def launcher_text(python_rel: str) -> str:
    return textwrap.dedent(
        f"""\
        #!/usr/bin/env bash
        set -euo pipefail

        ROOT="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
        ENV_FILE="$ROOT/cc-webxr.env"

        if [[ -f "$ENV_FILE" ]]; then
          set -a
          # shellcheck disable=SC1090
          source "$ENV_FILE"
          set +a
        fi

        if [[ -x "$ROOT/runtime/node/bin/node" ]]; then
          export PATH="$ROOT/runtime/node/bin:$PATH"
        fi
        if [[ -d "$ROOT/platform-tools" ]]; then
          export PATH="$ROOT/platform-tools:$PATH"
        fi

        if [[ -f "$ROOT/{python_rel}" ]]; then
          # shellcheck disable=SC1091
          source "$ROOT/runtime/venv/bin/activate"
        fi

        exec python "$ROOT/WebXR_link.py" "$@"
        """
    )


def service_launcher_text() -> str:
    return textwrap.dedent(
        """\
        #!/usr/bin/env bash
        set -euo pipefail

        ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        ENV_FILE="$ROOT/cc-webxr.env"

        if [[ -f "$ENV_FILE" ]]; then
          set -a
          # shellcheck disable=SC1090
          source "$ENV_FILE"
          set +a
        fi

        if [[ -x "$ROOT/runtime/node/bin/node" ]]; then
          export PATH="$ROOT/runtime/node/bin:$PATH"
        fi

        exec node "$ROOT/server/index.mjs" --serve-dist "$@"
        """
    )


def cert_launcher_text() -> str:
    return textwrap.dedent(
        """\
        #!/usr/bin/env bash
        set -euo pipefail

        ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        ENV_FILE="$ROOT/cc-webxr.env"

        if [[ -f "$ENV_FILE" ]]; then
          set -a
          # shellcheck disable=SC1090
          source "$ENV_FILE"
          set +a
        fi

        if [[ -x "$ROOT/runtime/node/bin/node" ]]; then
          export PATH="$ROOT/runtime/node/bin:$PATH"
        fi

        exec node "$ROOT/scripts/generate-dev-certs.mjs" "$@"
        """
    )


def quickstart_text(bundle_name: str, bundled_node: bool, bundled_tools: bool) -> str:
    node_text = "bundled under ./runtime/node" if bundled_node else "expected from system PATH"
    tools_text = "bundled under ./platform-tools" if bundled_tools else "expected from system PATH"
    return textwrap.dedent(
        f"""\
        {bundle_name}
        ==================

        Included runtime:
        - Python venv: ./runtime/venv
        - Node.js: {node_text}
        - adb / gnirehtet: {tools_text}

        Quick start:
        1. Edit ./cc-webxr.env if you need smaller TUI cells.
        2. Run ./run-webxr-link.sh
        3. Or run ./run-webxr-service.sh for the HTTPS/WSS service only.

        TUI tuning:
        - Smaller terminal: set CCWEBXR_TUI_SCALE=0.85
        - Denser layout:   set CCWEBXR_TUI_DENSITY=compact
        """
    )


def create_archive(source_dir: Path, archive_path: Path) -> None:
    if archive_path.exists():
        archive_path.unlink()
    with tarfile.open(archive_path, "w:gz") as tar:
        tar.add(source_dir, arcname=source_dir.name)


def bundle_paths(base: Path, names: Iterable[str]) -> None:
    for name in names:
        src = WEBXR_DIR / name
        ensure_exists(src, name)
        copy_path(src, base / name)


def build_bundle(args: argparse.Namespace) -> Path:
    python_venv = Path(args.python_venv).resolve()
    ensure_exists(python_venv / "bin" / "python", "Ubuntu venv python")

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    bundle_root = output_dir / args.bundle_name
    if bundle_root.exists():
        shutil.rmtree(bundle_root)

    app_root = bundle_root / "webxr"
    runtime_root = app_root / "runtime"
    app_root.mkdir(parents=True, exist_ok=True)
    runtime_root.mkdir(parents=True, exist_ok=True)

    bundle_paths(app_root, DIRS_TO_COPY)
    bundle_paths(app_root, FILES_TO_COPY)

    copy_path(python_venv, runtime_root / "venv")

    bundled_node = False
    if args.node_root:
        node_root = Path(args.node_root).resolve()
        ensure_exists(node_root, "Node runtime root")
        copy_path(node_root, runtime_root / "node")
        bundled_node = True

    bundled_tools = False
    platform_tools_dir: Optional[Path] = None
    if args.platform_tools_dir:
        platform_tools_dir = Path(args.platform_tools_dir).resolve()
    else:
        platform_tools_dir = detect_local_platform_tools()
    if platform_tools_dir is not None:
        ensure_exists(platform_tools_dir, "platform-tools directory")
        copy_path(platform_tools_dir, app_root / "platform-tools")
        bundled_tools = True

    write_text(app_root / "cc-webxr.env.example", env_template())
    write_text(app_root / "cc-webxr.env", env_template())
    write_text(app_root / "run-webxr-link.sh", launcher_text("runtime/venv/bin/python"), executable=True)
    write_text(app_root / "run-webxr-service.sh", service_launcher_text(), executable=True)
    write_text(app_root / "regenerate-certs.sh", cert_launcher_text(), executable=True)
    write_text(
        app_root / "BUNDLE_QUICKSTART.txt",
        quickstart_text(args.bundle_name, bundled_node, bundled_tools),
    )

    manifest = {
        "bundleName": args.bundle_name,
        "createdAtEpochMs": int(time.time() * 1000),
        "pythonVenv": str(python_venv),
        "nodeRootIncluded": bundled_node,
        "platformToolsIncluded": bundled_tools,
        "copiedDirs": list(DIRS_TO_COPY),
        "copiedFiles": list(FILES_TO_COPY),
    }
    write_text(app_root / "bundle-manifest.json", json.dumps(manifest, indent=2) + "\n")

    if args.archive:
        create_archive(bundle_root, output_dir / f"{args.bundle_name}.tar.gz")

    return bundle_root


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create a self-contained Ubuntu WebXR runtime bundle from an Ubuntu-prepared workspace.",
    )
    parser.add_argument(
        "--python-venv",
        required=True,
        help="Ubuntu venv path that already contains the WebXR_link Python dependencies.",
    )
    parser.add_argument(
        "--node-root",
        default="",
        help="Optional portable Node.js root to embed, e.g. ~/node-v20.x-linux-x64",
    )
    parser.add_argument(
        "--platform-tools-dir",
        default="",
        help="Optional Linux adb/gnirehtet directory to copy into ./platform-tools",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Where to place the bundle directory and archive.",
    )
    parser.add_argument(
        "--bundle-name",
        default="cc-webxr-ubuntu-bundle",
        help="Top-level bundle directory name.",
    )
    parser.add_argument(
        "--archive",
        action="store_true",
        default=True,
        help="Also create a .tar.gz archive (default: on).",
    )
    parser.add_argument(
        "--no-archive",
        dest="archive",
        action="store_false",
        help="Skip .tar.gz creation and leave only the expanded directory.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    bundle_root = build_bundle(args)
    print(f"Bundle created: {bundle_root}")
    if args.archive:
        print(f"Archive created: {bundle_root.parent / (bundle_root.name + '.tar.gz')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
