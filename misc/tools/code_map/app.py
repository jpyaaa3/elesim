#!/usr/bin/env python3
"""Launch the EleSim live code map."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from misc.tools.code_map.server import serve


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--host", default="127.0.0.1", choices=("127.0.0.1", "::1", "localhost"))
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--token", default="")
    parser.add_argument("--jaeger-url", default="http://127.0.0.1:16686")
    return parser


def main() -> int:
    args = _parser().parse_args()
    serve(args.root, args.host, args.port, args.token, args.jaeger_url)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
