"""Build the pilot's immutable IK model from source assembly assets."""

from __future__ import annotations

import dataclasses
import json
import tempfile
from pathlib import Path
from typing import Any

import numpy as np


def _json_value(value: Any) -> Any:
    if dataclasses.is_dataclass(value):
        return {
            "__dataclass__": type(value).__name__,
            "value": {
                field.name: _json_value(getattr(value, field.name))
                for field in dataclasses.fields(value)
            },
        }
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    raise TypeError(f"unsupported arm model value: {type(value).__name__}")


def build_arm_model(*, config: Path, assets: Path, output: Path) -> Path:
    """Build one model without writing into the pilot configuration tree."""
    from elesim_model_builder import json_builder
    from elesim_model_builder.context_builder import build_solver_context

    previous_asset_root = json_builder.DEFAULT_ASSET_ROOT_DIR
    try:
        json_builder.DEFAULT_ASSET_ROOT_DIR = str(Path(assets).resolve())
        with tempfile.TemporaryDirectory(prefix="elesim-arm-model-") as workspace:
            _bundle, context = build_solver_context(
                str(Path(config)),
                build_dir=workspace,
            )
    finally:
        json_builder.DEFAULT_ASSET_ROOT_DIR = previous_asset_root

    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(
            {"schema_version": 1, "context": _json_value(context)},
            ensure_ascii=True,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return destination


__all__ = ["build_arm_model"]
