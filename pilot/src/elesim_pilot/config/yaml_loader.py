"""Canonical strict YAML configuration loader."""

from __future__ import annotations

import copy
import os
import re
from typing import Any

import yaml

from elesim_pilot.config.schema import AppConfigBundle
from elesim_pilot.config.yaml_schema import ConfigValidationError, build_bundle_from_yaml


class _StrictYamlLoader(yaml.SafeLoader):
    pass


_StrictYamlLoader.yaml_implicit_resolvers = copy.deepcopy(
    yaml.SafeLoader.yaml_implicit_resolvers
)
for _first, _resolvers in list(_StrictYamlLoader.yaml_implicit_resolvers.items()):
    _StrictYamlLoader.yaml_implicit_resolvers[_first] = [
        item for item in _resolvers if item[0] != "tag:yaml.org,2002:bool"
    ]
_StrictYamlLoader.add_implicit_resolver(
    "tag:yaml.org,2002:bool",
    re.compile(r"^(?:true|false)$", re.IGNORECASE),
    list("tTfF"),
)


def _construct_mapping(loader: _StrictYamlLoader, node: yaml.MappingNode, deep: bool = False) -> dict[Any, Any]:
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            mark = key_node.start_mark
            raise ConfigValidationError(
                f"duplicate YAML key {key!r} at line {mark.line + 1}, column {mark.column + 1}"
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_StrictYamlLoader.add_constructor("tag:yaml.org,2002:map", _construct_mapping)


def parse_yaml_value(text: str) -> Any:
    """Parse a standalone value with the same rules as application YAML."""
    try:
        return yaml.load(text, Loader=_StrictYamlLoader)
    except yaml.YAMLError as exc:
        raise ConfigValidationError(f"invalid YAML value: {exc}") from exc


def _read_document(path: str) -> dict[str, Any]:
    try:
        with open(path, "r", encoding="utf-8-sig") as handle:
            data = yaml.load(handle, Loader=_StrictYamlLoader)
    except yaml.YAMLError as exc:
        raise ConfigValidationError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigValidationError(f"{path}: root must be a mapping")
    return data


def _deep_merge(parent: dict[str, Any], child: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(parent)
    for key, value in child.items():
        if key in out and isinstance(out[key], dict) and isinstance(value, dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def _resolve_owned_paths(document: dict[str, Any], *, config_dir: str) -> None:
    path_keys = (
        ("simulation", "assembly", "build_dir"),
        ("simulation", "cameras", "hand_eye", "config"),
        ("vision", "perception", "detector", "detector_config"),
    )
    for keys in path_keys:
        cursor: Any = document
        for key in keys[:-1]:
            if not isinstance(cursor, dict) or key not in cursor:
                break
            cursor = cursor[key]
        else:
            leaf = keys[-1]
            if isinstance(cursor, dict):
                value = cursor.get(leaf)
                if isinstance(value, str) and value and not os.path.isabs(value):
                    cursor[leaf] = os.path.abspath(os.path.join(config_dir, value))


def _read_with_extends(path: str) -> tuple[dict[str, Any], str]:
    root_path = os.path.abspath(path)
    visiting: set[str] = set()

    def collect(current: str) -> dict[str, Any]:
        current_abs = os.path.abspath(current)
        if current_abs in visiting:
            raise ConfigValidationError(f"config extends cycle detected at {current_abs}")
        if not os.path.isfile(current_abs):
            raise FileNotFoundError(f"config file not found: {current_abs}")
        if os.path.splitext(current_abs)[1].lower() not in (".yaml", ".yml"):
            raise ConfigValidationError(f"YAML config may only extend YAML: {current_abs}")
        visiting.add(current_abs)
        document = _read_document(current_abs)
        version = document.pop("schema_version", None)
        if type(version) is not int or version != 1:
            raise ConfigValidationError(f"{current_abs}: schema_version must be integer 1")
        parent_raw = document.pop("extends", None)
        _resolve_owned_paths(document, config_dir=os.path.dirname(current_abs))
        parent: dict[str, Any] = {}
        if parent_raw is not None:
            if not isinstance(parent_raw, str) or not parent_raw.strip():
                raise ConfigValidationError(f"{current_abs}: extends must be a non-empty path or null")
            parent_path = parent_raw if os.path.isabs(parent_raw) else os.path.join(os.path.dirname(current_abs), parent_raw)
            parent = collect(parent_path)
        visiting.remove(current_abs)
        return _deep_merge(parent, document)

    return collect(root_path), os.path.dirname(root_path)


def load_app_config_from_yaml(path: str) -> AppConfigBundle:
    if not path:
        raise FileNotFoundError("config path is empty")
    data, config_dir = _read_with_extends(path)
    return build_bundle_from_yaml(data, config_dir=config_dir)
