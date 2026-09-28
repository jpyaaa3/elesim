"""Bounded, versioned expression graph for Robot-local IMU correction.

The Pilot owns model files. The Robot receives this value once per selection
and compiles it into fixed-size native instructions; no JSON is parsed in the
control tick.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from .messages import ProtocolError


IMU_MODEL_SCHEMA_VERSION = 1
MAX_IMU_MODEL_NODES = 64
IMU_MODEL_OPCODES = {
    "q": 1, "imu": 2, "const": 3,
    "add": 4, "sub": 5, "mul": 6, "div": 7,
    "neg": 8, "sin": 9, "cos": 10,
}
_BINARY = frozenset({"add", "sub", "mul", "div"})
_UNARY = frozenset({"neg", "sin", "cos"})


def _object(value: object, *, name: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ProtocolError(f"{name} must be an object")
    return {str(key): item for key, item in value.items()}


def _fields(value: Mapping[str, Any], required: set[str], *, name: str) -> None:
    if set(value) != required:
        raise ProtocolError(f"{name} fields must be {sorted(required)}")


def _integer(value: object, low: int, high: int, *, name: str) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ProtocolError(f"{name} must be an integer in {low}..{high}")
    return value


def _identifier(value: object, *, name: str) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= 64:
        raise ProtocolError(f"{name} must contain 1..64 characters")
    if any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-" for character in value):
        raise ProtocolError(f"{name} must be a basename-like identifier")
    if value in {".", ".."} or value.startswith("."):
        raise ProtocolError(f"{name} must be a basename-like identifier")
    return value


@dataclass(frozen=True)
class ImuModelDefinition:
    model_id: str
    version: int
    nodes: tuple[dict[str, Any], ...]
    outputs: tuple[int, int, int, int]
    label: str = ""

    @classmethod
    def from_payload(cls, payload: object, *, catalog_file: bool = False) -> "ImuModelDefinition":
        raw = _object(payload, name="IMU model")
        required = {"schema_version", "id", "version", "program"}
        if catalog_file:
            required.add("label")
        _fields(raw, required, name="IMU model")
        if raw["schema_version"] != IMU_MODEL_SCHEMA_VERSION or type(raw["schema_version"]) is not int:
            raise ProtocolError("unsupported IMU model schema")
        model_id = _identifier(raw["id"], name="IMU model id")
        version = _integer(raw["version"], 1, 65535, name="IMU model version")
        label = ""
        if catalog_file:
            label = raw["label"]
            if not isinstance(label, str) or not 1 <= len(label) <= 80:
                raise ProtocolError("IMU model label must contain 1..80 characters")
        program = _object(raw["program"], name="IMU model program")
        _fields(program, {"nodes", "outputs"}, name="IMU model program")
        nodes_raw = program["nodes"]
        if not isinstance(nodes_raw, list) or not 4 <= len(nodes_raw) <= MAX_IMU_MODEL_NODES:
            raise ProtocolError("IMU model needs 4..64 nodes")
        nodes: list[dict[str, Any]] = []
        for position, item in enumerate(nodes_raw):
            node = _object(item, name="IMU model node")
            op = node.get("op")
            if not isinstance(op, str) or op not in IMU_MODEL_OPCODES:
                raise ProtocolError("unsupported IMU model operation")
            if op == "q":
                _fields(node, {"op", "index"}, name="q node")
                _integer(node["index"], 0, 3, name="q index")
            elif op == "imu":
                _fields(node, {"op", "index"}, name="IMU node")
                _integer(node["index"], 0, 2, name="IMU index")
            elif op == "const":
                _fields(node, {"op", "value"}, name="constant node")
                value = node["value"]
                if type(value) not in {int, float} or not math.isfinite(value) or abs(value) > 1e6:
                    raise ProtocolError("IMU model constant must be finite and within ±1e6")
            elif op in _BINARY:
                _fields(node, {"op", "a", "b"}, name="binary node")
                _integer(node["a"], 0, position - 1, name="node a")
                _integer(node["b"], 0, position - 1, name="node b")
            else:
                _fields(node, {"op", "a"}, name="unary node")
                _integer(node["a"], 0, position - 1, name="node a")
            nodes.append(node)
        outputs = program["outputs"]
        if not isinstance(outputs, list) or len(outputs) != 4:
            raise ProtocolError("IMU model must have four output node indexes")
        result = tuple(_integer(value, 0, len(nodes) - 1, name="output index") for value in outputs)
        return cls(model_id, version, tuple(nodes), result, label)

    @property
    def requires_imu(self) -> bool:
        return any(node["op"] == "imu" for node in self.nodes)

    def command_payload(self) -> dict[str, Any]:
        return {
            "schema_version": IMU_MODEL_SCHEMA_VERSION,
            "id": self.model_id,
            "version": self.version,
            "program": {"nodes": [dict(node) for node in self.nodes], "outputs": list(self.outputs)},
        }

    def catalog_entry(self) -> dict[str, Any]:
        return {
            "id": self.model_id,
            "version": self.version,
            "label": self.label,
            "requires_imu": self.requires_imu,
        }
