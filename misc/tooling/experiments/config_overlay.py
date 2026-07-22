#!/usr/bin/env python3
"""Create a temporary YAML profile without modifying its base config."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Mapping

import yaml

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.config import load_app_config
from elesim_controller.config.yaml_loader import parse_yaml_value


def _set_dotted(root: dict[str, Any], dotted: str, value: Any) -> None:
    parts = [part for part in dotted.split(".") if part]
    if not parts:
        raise ValueError("override path must not be empty")
    cursor = root
    for part in parts[:-1]:
        child = cursor.setdefault(part, {})
        if not isinstance(child, dict):
            raise ValueError(f"override path collides at {part!r}")
        cursor = child
    cursor[parts[-1]] = value


def write_config_overlay(
    base_path: str | Path,
    output_path: str | Path,
    overrides: Mapping[str, Any],
) -> Path:
    base = Path(base_path).resolve()
    output = Path(output_path).resolve()
    payload: dict[str, Any] = {"schema_version": 1, "extends": str(base)}
    for dotted, value in overrides.items():
        _set_dotted(payload, dotted, value)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    load_app_config(str(output))
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--set", action="append", default=[], metavar="PATH=VALUE")
    args = parser.parse_args()

    overrides: dict[str, Any] = {}
    for item in args.set:
        if "=" not in item:
            parser.error(f"--set expects PATH=VALUE, got {item!r}")
        path, raw = item.split("=", 1)
        overrides[path] = parse_yaml_value(raw)
    write_config_overlay(args.base, args.output, overrides)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
