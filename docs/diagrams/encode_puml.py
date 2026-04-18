#!/usr/bin/env python3
"""Encode one or more .puml files into PlantUML server URLs.

Usage:
    python3 encode_puml.py <file.puml> [<file2.puml> ...]
    python3 encode_puml.py *.puml
"""

import sys
import zlib
import base64

_B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
_PUML = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_"
_TABLE = str.maketrans(_B64, _PUML)

PLANTUML_BASE = "https://www.plantuml.com/plantuml/svg"


def encode(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        text = f.read()
    # Deflate-compress, strip the 2-byte zlib header and 4-byte checksum trailer
    compressed = zlib.compress(text.encode("utf-8"), 9)[2:-4]
    token = base64.b64encode(compressed).decode("ascii").translate(_TABLE)
    return f"{PLANTUML_BASE}/{token}"


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <file.puml> [<file2.puml> ...]", file=sys.stderr)
        sys.exit(1)
    for path in sys.argv[1:]:
        print(f"# {path}")
        print(encode(path))
        print()


if __name__ == "__main__":
    main()
