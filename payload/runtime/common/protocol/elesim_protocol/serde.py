"""JSON-safe values shared by the UI and pilot wire contract."""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from .messages import ControlU, ProtocolError, SimQ


def encode_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, (float, np.floating)):
        number = float(value)
        if not math.isfinite(number):
            raise ProtocolError("encoded value contains a non-finite number")
        return number
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, Path):
        return {"__type__": "Path", "value": str(value)}
    if dataclasses.is_dataclass(value):
        return {
            "__type__": type(value).__name__,
            "value": {
                field.name: encode_value(getattr(value, field.name))
                for field in dataclasses.fields(value)
            },
        }
    if isinstance(value, np.ndarray):
        return encode_value(value.tolist())
    if isinstance(value, dict):
        return {str(key): encode_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [encode_value(item) for item in value]
    if hasattr(value, "__dict__"):
        return {
            "__type__": type(value).__name__,
            "value": {
                str(key): encode_value(item)
                for key, item in vars(value).items()
                if not str(key).startswith("_") and is_json_candidate(item)
            },
        }
    return str(value)


def is_json_candidate(value: Any) -> bool:
    return (
        value is None
        or isinstance(value, (bool, int, float, str, Path, list, tuple, dict, np.ndarray))
        or isinstance(value, (np.bool_, np.integer, np.floating))
        or dataclasses.is_dataclass(value)
    )


def decode_value(value: Any) -> Any:
    if isinstance(value, list):
        return [decode_value(item) for item in value]
    if not isinstance(value, dict):
        return value
    kind = value.get("__type__")
    raw = value.get("value")
    if kind == "Path":
        return Path(str(raw))
    if kind == "ControlU" and isinstance(raw, dict):
        return ControlU(**{key: float(item) for key, item in raw.items()})
    if kind == "SimQ" and isinstance(raw, dict):
        return SimQ(**{key: float(item) for key, item in raw.items()})
    if kind and isinstance(raw, dict):
        return SimpleNamespace(**{key: decode_value(item) for key, item in raw.items()})
    return {key: decode_value(item) for key, item in value.items()}


def state_snapshot(state: Any) -> dict[str, Any]:
    return {
        key: encode_value(value)
        for key, value in vars(state).items()
        if not key.startswith("_") and is_json_candidate(value)
    }
