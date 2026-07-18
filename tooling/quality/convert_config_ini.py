#!/usr/bin/env python3
"""Convert Elesim legacy INI profiles to canonical ownership-oriented YAML."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.config.legacy_ini import load_app_config_from_ini
from elesim_controller.config.yaml_schema import bundle_to_yaml_data


def _write_yaml(path: Path, payload: dict) -> None:
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True, width=100),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_ini")
    parser.add_argument("base_yaml")
    parser.add_argument("--profile", action="append", nargs=3, metavar=("INI", "YAML", "EXTENDS"), default=[])
    args = parser.parse_args()

    base_bundle = load_app_config_from_ini(args.base_ini)
    config_dir = str(Path(args.base_yaml).resolve().parent)
    base_payload = {"schema_version": 1, "extends": None}
    base_payload.update(bundle_to_yaml_data(base_bundle, config_dir=config_dir))
    _write_yaml(Path(args.base_yaml), base_payload)

    for ini_path, yaml_path, extends in args.profile:
        profile_bundle = load_app_config_from_ini(ini_path)
        profile_payload = {"schema_version": 1, "extends": extends}
        profile_payload.update(
            bundle_to_yaml_data(
                profile_bundle,
                baseline=base_bundle,
                config_dir=str(Path(yaml_path).resolve().parent),
            )
        )
        _write_yaml(Path(yaml_path), profile_payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
