from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


ICON_WIDTH = 30
ICON_HEIGHT = 30
ICON_ROW_BYTES = (ICON_WIDTH + 7) // 8
ICON_BYTES = ICON_ROW_BYTES * ICON_HEIGHT
MAX_ITEMS = 15
DEFAULT_CONFIG_PATH = "include/tile_menu_config.h"

ICON_PATTERN = re.compile(
    r"static const uint8_t kTileIcon(?P<index>\d{2})\[[^\]]+\] PROGMEM = \{(?P<body>.*?)\};",
    re.DOTALL,
)
ITEMS_BLOCK_PATTERN = re.compile(
    r"static const TileMenuConfigEntry kItems\[kMaxItems\] = \{(?P<body>.*?)\};",
    re.DOTALL,
)
ITEM_PATTERN = re.compile(
    r"\{\s*(?P<title>nullptr|\"(?:[^\"\\]|\\.)*\")\s*,\s*"
    r"(?P<icon>nullptr|kTileIcon\d{2})\s*,\s*"
    r"(?P<command>0x[0-9A-Fa-f]{2})\s*\}",
    re.DOTALL,
)
HEX_PATTERN = re.compile(r"0x[0-9A-Fa-f]{1,2}|\d+")


@dataclass
class TileItem:
    index: int
    title: str
    command: int
    icon_bytes: list[int]

    @property
    def visible(self) -> bool:
        return bool(self.title.strip()) and self.command != 0


REPO_ROOT = Path(__file__).resolve().parents[2]
STATIC_ROOT = Path(__file__).resolve().parent


def resolve_repo_path(raw_path: str | None) -> Path:
    candidate = (raw_path or DEFAULT_CONFIG_PATH).strip()
    if not candidate:
        candidate = DEFAULT_CONFIG_PATH

    path = Path(candidate)
    if not path.is_absolute():
        path = (REPO_ROOT / path).resolve()
    else:
        path = path.resolve()

    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise ValueError(f"path must stay under repo root: {REPO_ROOT}") from exc

    return path


def repo_relative(path: Path) -> str:
    return path.relative_to(REPO_ROOT).as_posix()


def decode_c_string(token: str) -> str:
    if token == "nullptr":
        return ""
    content = token[1:-1]
    out: list[str] = []
    i = 0
    while i < len(content):
        ch = content[i]
        if ch != "\\":
            out.append(ch)
            i += 1
            continue

        i += 1
        if i >= len(content):
            out.append("\\")
            break

        esc = content[i]
        if esc == "n":
            out.append("\n")
        elif esc == "r":
            out.append("\r")
        elif esc == "t":
            out.append("\t")
        else:
            out.append(esc)
        i += 1
    return "".join(out)


def encode_c_string(text: str) -> str:
    escaped = text.replace("\\", "\\\\").replace('"', '\\"')
    escaped = escaped.replace("\r", "\\r").replace("\n", "\\n").replace("\t", "\\t")
    return f'"{escaped}"'


def parse_icon_body(body: str) -> list[int]:
    values = [int(token, 0) for token in HEX_PATTERN.findall(body)]
    if len(values) != ICON_BYTES:
        raise ValueError(
            f"icon data size mismatch: expected {ICON_BYTES}, got {len(values)}"
        )
    return [value & 0xFF for value in values]


def blank_icon() -> list[int]:
    return [0] * ICON_BYTES


def parse_header(path: Path) -> list[TileItem]:
    text = path.read_text(encoding="utf-8")

    icons_by_name: dict[str, list[int]] = {}
    for match in ICON_PATTERN.finditer(text):
        icon_name = f"kTileIcon{match.group('index')}"
        icons_by_name[icon_name] = parse_icon_body(match.group("body"))

    items_match = ITEMS_BLOCK_PATTERN.search(text)
    if not items_match:
        raise ValueError("kItems block not found")

    parsed_items = list(ITEM_PATTERN.finditer(items_match.group("body")))
    if len(parsed_items) != MAX_ITEMS:
        raise ValueError(
            f"kItems count mismatch: expected {MAX_ITEMS}, got {len(parsed_items)}"
        )

    result: list[TileItem] = []
    for index, match in enumerate(parsed_items):
        title = decode_c_string(match.group("title"))
        command = int(match.group("command"), 0) & 0xFF
        icon_ref = match.group("icon")
        icon_bytes = icons_by_name.get(f"kTileIcon{index:02d}", blank_icon())
        if icon_ref == "nullptr" and f"kTileIcon{index:02d}" not in icons_by_name:
            icon_bytes = blank_icon()
        result.append(
            TileItem(
                index=index,
                title=title,
                command=command,
                icon_bytes=icon_bytes,
            )
        )
    return result


def icon_bytes_to_rows(icon_bytes: list[int]) -> list[list[int]]:
    rows: list[list[int]] = []
    for y in range(ICON_HEIGHT):
        row: list[int] = []
        base = y * ICON_ROW_BYTES
        for x in range(ICON_WIDTH):
            row.append((icon_bytes[base + (x // 8)] >> (x % 8)) & 0x01)
        rows.append(row)
    return rows


def rows_to_icon_bytes(rows: list[list[int]]) -> list[int]:
    if len(rows) != ICON_HEIGHT:
        raise ValueError(f"icon rows mismatch: expected {ICON_HEIGHT}")

    data = [0] * ICON_BYTES
    for y, row in enumerate(rows):
        if len(row) != ICON_WIDTH:
            raise ValueError(f"icon row width mismatch at y={y}: expected {ICON_WIDTH}")
        for x, value in enumerate(row):
            if int(value) & 0x01:
                data[y * ICON_ROW_BYTES + (x // 8)] |= 1 << (x % 8)
    return data


def normalize_command(value: Any) -> int:
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return 0
        return int(text, 0) & 0xFF
    return int(value) & 0xFF


def normalize_title(value: Any) -> str:
    return str(value or "").strip()


def normalize_rows(value: Any) -> list[list[int]]:
    if not isinstance(value, list):
        raise ValueError("icon pixels must be a 30x30 array")
    rows: list[list[int]] = []
    for row in value:
        if not isinstance(row, list):
            raise ValueError("icon pixels must be a 30x30 array")
        rows.append([1 if int(cell) else 0 for cell in row])
    return rows


def normalize_items(payload: dict[str, Any]) -> list[TileItem]:
    raw_items = payload.get("items")
    if not isinstance(raw_items, list) or len(raw_items) != MAX_ITEMS:
        raise ValueError(f"items must contain exactly {MAX_ITEMS} entries")

    items: list[TileItem] = []
    for index, raw_item in enumerate(raw_items):
        if not isinstance(raw_item, dict):
            raise ValueError("each item must be an object")
        title = normalize_title(raw_item.get("title"))
        command = normalize_command(raw_item.get("command", 0))
        rows = normalize_rows(raw_item.get("pixels", []))
        items.append(
            TileItem(
                index=index,
                title=title,
                command=command,
                icon_bytes=rows_to_icon_bytes(rows),
            )
        )
    return items


def format_icon_bytes(icon_bytes: list[int]) -> str:
    chunks = []
    for i in range(0, len(icon_bytes), 16):
        line = ",".join(f"0x{value:02X}" for value in icon_bytes[i : i + 16])
        chunks.append(f"    {line},")
    return "\n".join(chunks)


def render_header(items: list[TileItem]) -> str:
    lines: list[str] = [
        "#pragma once",
        "",
        "#include <Arduino.h>",
        "",
        "struct TileMenuConfigEntry {",
        "  const char *title;",
        "  const uint8_t *icon;",
        "  uint8_t command;",
        "};",
        "",
        "namespace TileMenuConfig {",
        "",
        f"constexpr uint8_t kMaxItems = {MAX_ITEMS};",
        f"constexpr uint8_t kIconWidth = {ICON_WIDTH};",
        f"constexpr uint8_t kIconHeight = {ICON_HEIGHT};",
        "constexpr size_t kIconBytes = ((kIconWidth + 7U) / 8U) * kIconHeight;",
        "constexpr uint8_t kVisibleIndicatorWidth = 7;",
        "constexpr uint8_t kVisibleIndicatorHeight = 27;",
        "constexpr uint8_t kIconSpacing = 36;",
        "constexpr uint8_t kIndicatorTop = 36;",
        "",
        "// TILE icons use the upstream 30x30 1-bit XBM-style layout.",
        "// Each byte packs 8 pixels, so 30x30 needs 4 bytes per row * 30 rows = 120 bytes.",
        "// Leave title empty or command 0 to hide an item without losing its icon data.",
        "",
    ]

    for item in items:
        lines.append(
            f"static const uint8_t kTileIcon{item.index:02d}[kIconBytes] PROGMEM = {{"
        )
        lines.append(format_icon_bytes(item.icon_bytes))
        lines.append("};")
        lines.append("")

    lines.append("static const TileMenuConfigEntry kItems[kMaxItems] = {")
    for item in items:
        if item.visible:
            lines.append(
                f"    {{{encode_c_string(item.title)}, kTileIcon{item.index:02d}, 0x{item.command:02X}}},"
            )
        else:
            lines.append("    {nullptr, nullptr, 0x00},")
    lines.append("};")
    lines.append("")
    lines.append("} // namespace TileMenuConfig")
    lines.append("")
    return "\n".join(lines)


def config_to_payload(path: Path, items: list[TileItem]) -> dict[str, Any]:
    return {
        "path": repo_relative(path),
        "absolutePath": str(path),
        "iconWidth": ICON_WIDTH,
        "iconHeight": ICON_HEIGHT,
        "maxItems": MAX_ITEMS,
        "items": [
            {
                "index": item.index,
                "title": item.title,
                "command": item.command,
                "visible": item.visible,
                "pixels": icon_bytes_to_rows(item.icon_bytes),
                "iconBytes": item.icon_bytes,
            }
            for item in items
        ],
    }


class TileMenuEditorHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(STATIC_ROOT), **kwargs)

    def log_message(self, format: str, *args: Any) -> None:
        print(f"[tile-editor] {self.address_string()} - {format % args}")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/config":
            self.handle_get_config(parsed)
            return
        if parsed.path == "/":
            self.path = "/index.html"
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/config":
            self.handle_post_config()
            return
        self.send_error(HTTPStatus.NOT_FOUND, "unknown endpoint")

    def read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON body: {exc}") from exc

    def send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def handle_get_config(self, parsed) -> None:
        try:
            query = parse_qs(parsed.query)
            path = resolve_repo_path(query.get("path", [DEFAULT_CONFIG_PATH])[0])
            items = parse_header(path)
            self.send_json({"ok": True, **config_to_payload(path, items)})
        except Exception as exc:
            self.send_json(
                {"ok": False, "error": str(exc)},
                status=HTTPStatus.BAD_REQUEST,
            )

    def handle_post_config(self) -> None:
        try:
            payload = self.read_json_body()
            path = resolve_repo_path(payload.get("path"))
            items = normalize_items(payload)
            header_text = render_header(items)
            path.write_text(header_text, encoding="utf-8", newline="\n")
            self.send_json({"ok": True, **config_to_payload(path, items)})
        except Exception as exc:
            self.send_json(
                {"ok": False, "error": str(exc)},
                status=HTTPStatus.BAD_REQUEST,
            )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Local web editor for include/tile_menu_config.h"
    )
    parser.add_argument("--host", default="127.0.0.1", help="bind host")
    parser.add_argument("--port", type=int, default=8765, help="bind port")
    parser.add_argument(
        "--path",
        default=DEFAULT_CONFIG_PATH,
        help="default repo-relative config path to load",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="parse the target config and print a short summary, then exit",
    )
    return parser


def run_check(path_arg: str) -> int:
    path = resolve_repo_path(path_arg)
    items = parse_header(path)
    visible = sum(1 for item in items if item.visible)
    print(f"path={repo_relative(path)}")
    print(f"items={len(items)} visible={visible}")
    for item in items:
        title = item.title if item.title else "(hidden)"
        print(f"{item.index:02d}: cmd=0x{item.command:02X} title={title}")
    return 0


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.check:
        return run_check(args.path)

    server = ThreadingHTTPServer((args.host, args.port), TileMenuEditorHandler)
    print(
        f"Tile menu editor: http://{args.host}:{args.port}/?path={args.path}"
    )
    print(f"Repo root: {REPO_ROOT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping tile menu editor.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
