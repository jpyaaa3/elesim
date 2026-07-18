#!/usr/bin/env python3
"""Build the controller's immutable IK model from source assembly assets."""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path
from typing import Any

import numpy as np


def json_value(value: Any) -> Any:
    if dataclasses.is_dataclass(value):
        return {
            "__dataclass__": type(value).__name__,
            "value": {
                field.name: json_value(getattr(value, field.name))
                for field in dataclasses.fields(value)
            },
        }
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    raise TypeError(f"unsupported arm model value: {type(value).__name__}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="deployments/controller/config/config.pc.yaml")
    parser.add_argument("--assets", default="model/source/assets")
    parser.add_argument("--output", default="deployments/controller/config/arm_model.json")
    args = parser.parse_args()

    from elesim_model_builder import json_builder
    from elesim_model_builder.context_builder import load_solver_context

    json_builder.DEFAULT_ASSET_ROOT_DIR = str(Path(args.assets).resolve())

    _bundle, context = load_solver_context(args.config)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {"schema_version": 1, "context": json_value(context)},
            ensure_ascii=True,
            indent=2,
            sort_keys=True,
        ) + "\n",
        encoding="utf-8",
    )
    print(output)


if __name__ == "__main__":
    main()
