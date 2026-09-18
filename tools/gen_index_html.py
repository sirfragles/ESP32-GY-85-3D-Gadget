#!/usr/bin/env python3
"""Regenerates ESP32-GY-85-3D-Gadget/index_html.h from index.html.

The Arduino IDE cannot embed an arbitrary file into a sketch, so the web page
lives in this repo as ESP32-GY-85-3D-Gadget/index.html and is wrapped into a
PROGMEM C string in index_html.h, which the sketch #includes.

Run after every index.html edit, then re-upload the sketch:

    python3 tools/gen_index_html.py
"""
from __future__ import annotations

import hashlib
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SKETCH = ROOT / "ESP32-GY-85-3D-Gadget"
SRC = SKETCH / "index.html"
DST = SKETCH / "index_html.h"
DELIM = "HTML"


def main() -> int:
    if not SRC.is_file():
        print(f"error: {SRC} not found", file=sys.stderr)
        return 1

    html = SRC.read_text(encoding="utf-8").rstrip() + "\n"
    marker = f'){DELIM}"'
    if marker in html:
        print(f"error: raw string marker {marker!r} occurs in {SRC.name}", file=sys.stderr)
        return 1

    sha = hashlib.sha256(html.encode("utf-8")).hexdigest()[:12]
    header = (
        "// Auto-generated from index.html — do not edit by hand.\n"
        "// Regenerate with: python3 tools/gen_index_html.py\n"
        f"// Source: index.html, {len(html)} bytes, sha256 {sha}…\n"
        "#pragma once\n"
        "\n"
        f'const char index_html[] PROGMEM = R"{DELIM}(\n'
        f"{html}"
        f'){DELIM}";\n'
    )

    DST.write_text(header, encoding="utf-8")
    print(f"{DST.relative_to(ROOT)}: embedded {len(html)} bytes of HTML (sha256 {sha}…)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
