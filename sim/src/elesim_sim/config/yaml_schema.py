"""Strict YAML mapping for sim-owned configuration only."""

from __future__ import annotations

import os
import types
from dataclasses import fields, replace
from typing import Any, Mapping, Union, get_args, get_origin, get_type_hints

from elesim_sim.config.defaults import build_mapping_config, default_app_config_bundle
from elesim_sim.config.schema import (
    AppConfigBundle,
    ArmMappingConfig,
    JointLimit,
    SimConfig,
    SimParam,
    SpawnConfig,
    UrdfExportConfig,
)
from elesim_sim.robot.go2.locomotion.config import Go2LocomotionConfig
from elesim_sim.vision.camera_profile import (
    CameraProfileError,
    camera_profile,
    validate_bundle_path,
    validate_calibration_path,
)


class ConfigValidationError(ValueError):
    pass


_COMPONENT_TYPES: dict[str, type[Any]] = {
    "sim_param": SimParam,
    "sim_config": SimConfig,
    "arm_mapping_config": ArmMappingConfig,
    "joint_limit": JointLimit,
    "spawn_config": SpawnConfig,
    "urdf_export_config": UrdfExportConfig,
    "go2_locomotion_config": Go2LocomotionConfig,
}


class _Leaf:
    __slots__ = ("component", "field")

    def __init__(self, component: str, field: str) -> None:
        self.component = component
        self.field = field


_LEAVES: dict[str, _Leaf] = {}
_ASSIGNED: dict[str, set[str]] = {name: set() for name in _COMPONENT_TYPES}


def _field_names(cls: type[Any]) -> set[str]:
    return {item.name for item in fields(cls)}


def _prefixed(cls: type[Any], prefix: str) -> set[str]:
    return {name for name in _field_names(cls) if name.startswith(prefix)}


def _register(
    component: str,
    path: str,
    names: set[str],
    *,
    strip_prefix: str = "",
    aliases: Mapping[str, str] | None = None,
) -> None:
    aliases = dict(aliases or {})
    for name in sorted(names):
        key = aliases.get(
            name,
            name[len(strip_prefix) :] if strip_prefix and name.startswith(strip_prefix) else name,
        )
        dotted = f"{path}.{key}"
        if dotted in _LEAVES:
            raise RuntimeError(f"duplicate YAML config path: {dotted}")
        if name in _ASSIGNED[component]:
            raise RuntimeError(f"duplicate config field: {component}.{name}")
        _LEAVES[dotted] = _Leaf(component, name)
        _ASSIGNED[component].add(name)


_register("sim_param", "simulation.physics", _field_names(SimParam))
_register(
    "sim_config",
    "simulation.runtime",
    {
        "use_gpu",
        "camera_gpu_convert",
        "camera_execution",
        "camera_worker_start_timeout_s",
        "camera_first_frame_timeout_s",
        "enable_viewer",
        "visualizer_max_hz",
        "telemetry_max_hz",
        "floor",
        "use_hardware",
        "use_go2",
    },
)
_register(
    "sim_config",
    "simulation.assembly",
    {"build_dir", "assy_build_json", "urdf_name", "arm_urdf_name"},
)
_register(
    "sim_config",
    "simulation.cameras.hand_eye",
    _prefixed(SimConfig, "sim_camera_") | {"camera_profile", "hand_eye_config"},
    strip_prefix="sim_camera_",
    aliases={"camera_profile": "profile", "hand_eye_config": "config"},
)
_register(
    "sim_config",
    "simulation.cameras.observer",
    _prefixed(SimConfig, "sim_observer_camera_"),
    strip_prefix="sim_observer_camera_",
)
_register(
    "sim_config",
    "simulation.performance",
    _prefixed(SimConfig, "perf_"),
    strip_prefix="perf_",
)

_register("arm_mapping_config", "robot.arm.mapping", _field_names(ArmMappingConfig))
_register("joint_limit", "robot.arm.model", _field_names(JointLimit))
_register(
    "spawn_config",
    "robot.arm.spawn",
    {"pitch", "n_seg", "spawn_xyz", "spawn_euler_deg"},
    aliases={"spawn_xyz": "xyz", "spawn_euler_deg": "euler_deg"},
)
_register(
    "spawn_config",
    "robot.go2.spawn",
    {"go2_spawn_height", "go2_mount_offset_m", "go2_spawn_euler_deg"},
    strip_prefix="go2_",
)
_register(
    "spawn_config",
    "robot.go2.teleop",
    _prefixed(SpawnConfig, "go2_teleop_"),
    strip_prefix="go2_teleop_",
)
_register(
    "spawn_config",
    "world.target",
    _prefixed(SpawnConfig, "sim_target_"),
    strip_prefix="sim_target_",
)
_register(
    "spawn_config",
    "world.debug",
    {"draw_debug_markers"},
    aliases={"draw_debug_markers": "markers"},
)
_register(
    "urdf_export_config",
    "robot.arm.urdf",
    _field_names(UrdfExportConfig),
    aliases={"part_color_rgba_by_name": "colors"},
)

_go2_pitch = _prefixed(Go2LocomotionConfig, "mpc_pitch_trim_")
_go2_payload = _prefixed(Go2LocomotionConfig, "mpc_payload_")
_go2_mpc = _prefixed(Go2LocomotionConfig, "mpc_") - _go2_pitch - _go2_payload
_register(
    "go2_locomotion_config",
    "robot.go2.locomotion.pitch_trim",
    _go2_pitch,
    strip_prefix="mpc_pitch_trim_",
)
_register(
    "go2_locomotion_config",
    "robot.go2.locomotion.payload",
    _go2_payload,
    strip_prefix="mpc_payload_",
)
_register(
    "go2_locomotion_config",
    "robot.go2.locomotion.mpc",
    _go2_mpc,
    strip_prefix="mpc_",
)
_register(
    "go2_locomotion_config",
    "robot.go2.locomotion.general",
    _field_names(Go2LocomotionConfig) - _go2_pitch - _go2_payload - _go2_mpc,
)

for _component, _cls in _COMPONENT_TYPES.items():
    _missing = _field_names(_cls) - _ASSIGNED[_component]
    if _missing:
        raise RuntimeError(f"YAML schema does not map {_component}: {sorted(_missing)}")


_PATH_ALIASES = {
    "simulation.cameras.hand_eye.camera_profile": "simulation.cameras.hand_eye.profile",
}


_TYPE_HINTS = {name: get_type_hints(cls) for name, cls in _COMPONENT_TYPES.items()}


def _coerce_scalar(value: Any, expected: type[Any], path: str) -> Any:
    if expected is bool:
        if type(value) is not bool:
            raise ConfigValidationError(f"{path}: expected boolean")
        return value
    if expected is int:
        if type(value) is not int:
            raise ConfigValidationError(f"{path}: expected integer")
        return value
    if expected is float:
        if type(value) not in (int, float):
            raise ConfigValidationError(f"{path}: expected number")
        return float(value)
    if expected is str:
        if type(value) is not str:
            raise ConfigValidationError(f"{path}: expected string")
        return value
    return value


def _coerce(value: Any, annotation: Any, path: str) -> Any:
    origin = get_origin(annotation)
    args = get_args(annotation)
    if origin in (Union, types.UnionType):
        if value is None and type(None) in args:
            return None
        for choice in args:
            if choice is type(None):
                continue
            try:
                return _coerce(value, choice, path)
            except ConfigValidationError:
                continue
        raise ConfigValidationError(f"{path}: value does not match any allowed type")
    if origin is tuple:
        if not isinstance(value, list):
            raise ConfigValidationError(f"{path}: expected YAML sequence")
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_coerce(item, args[0], f"{path}[{index}]") for index, item in enumerate(value))
        if len(value) != len(args):
            raise ConfigValidationError(f"{path}: expected {len(args)} items, got {len(value)}")
        return tuple(
            _coerce(item, expected, f"{path}[{index}]")
            for index, (item, expected) in enumerate(zip(value, args))
        )
    if origin is dict:
        if not isinstance(value, dict):
            raise ConfigValidationError(f"{path}: expected mapping")
        key_type, value_type = args
        return {
            _coerce(key, key_type, f"{path}.<key>"): _coerce(item, value_type, f"{path}.{key}")
            for key, item in value.items()
        }
    if isinstance(annotation, type):
        return _coerce_scalar(value, annotation, path)
    return value


def _flatten_values(node: Any, prefix: str = "") -> dict[str, Any]:
    if prefix in _LEAVES:
        return {prefix: node}
    if not isinstance(node, dict):
        raise ConfigValidationError(f"{prefix or '<root>'}: expected mapping")
    out: dict[str, Any] = {}
    for raw_key, value in node.items():
        if not isinstance(raw_key, str):
            raise ConfigValidationError(f"{prefix or '<root>'}: keys must be strings")
        path = f"{prefix}.{raw_key}" if prefix else raw_key
        path = _PATH_ALIASES.get(path, path)
        if path not in _LEAVES and not any(leaf.startswith(f"{path}.") for leaf in _LEAVES):
            raise ConfigValidationError(f"{path}: unknown config key")
        out.update(_flatten_values(value, path))
    return out


def build_bundle_from_yaml(data: Mapping[str, Any], *, config_dir: str) -> AppConfigBundle:
    values = _flatten_values(dict(data))
    defaults = default_app_config_bundle()
    updates: dict[str, dict[str, Any]] = {name: {} for name in _COMPONENT_TYPES}
    for path, raw_value in values.items():
        leaf = _LEAVES[path]
        annotation = _TYPE_HINTS[leaf.component][leaf.field]
        updates[leaf.component][leaf.field] = _coerce(raw_value, annotation, path)

    components: dict[str, Any] = {}
    for component in _COMPONENT_TYPES:
        components[component] = replace(getattr(defaults, component), **updates[component])

    sim_config = components["sim_config"]
    execution = str(sim_config.camera_execution).strip().lower()
    if execution not in {"async_process", "sync_legacy"}:
        raise ConfigValidationError(
            "simulation.runtime.camera_execution must be async_process or sync_legacy"
        )
    if float(sim_config.camera_worker_start_timeout_s) <= 0.0:
        raise ConfigValidationError(
            "simulation.runtime.camera_worker_start_timeout_s must be positive"
        )
    if float(sim_config.camera_first_frame_timeout_s) <= 0.0:
        raise ConfigValidationError(
            "simulation.runtime.camera_first_frame_timeout_s must be positive"
        )
    if float(sim_config.visualizer_max_hz) < 0.0:
        raise ConfigValidationError(
            "simulation.runtime.visualizer_max_hz must be non-negative"
        )
    components["sim_config"] = replace(
        sim_config,
        camera_execution=execution,
    )
    sim_config = components["sim_config"]
    resolved: dict[str, str] = {}
    for name in ("build_dir", "hand_eye_config"):
        value = getattr(sim_config, name)
        if value and not os.path.isabs(value):
            resolved[name] = os.path.abspath(os.path.join(config_dir, value))
    if resolved:
        components["sim_config"] = replace(sim_config, **resolved)

    try:
        selected = camera_profile(components["sim_config"].camera_profile)
        validate_bundle_path(selected.name, components["sim_config"].build_dir)
        if components["sim_config"].hand_eye_config:
            validate_calibration_path(selected.name, components["sim_config"].hand_eye_config)
    except CameraProfileError as exc:
        raise ConfigValidationError(str(exc)) from exc

    mapping = build_mapping_config(components["joint_limit"], components["arm_mapping_config"])
    return AppConfigBundle(mapping_config=mapping, **components)


def _yaml_value(value: Any) -> Any:
    if isinstance(value, tuple):
        return [_yaml_value(item) for item in value]
    if isinstance(value, dict):
        return {key: _yaml_value(item) for key, item in value.items()}
    return value


def bundle_to_yaml_data(
    bundle: AppConfigBundle,
    *,
    baseline: AppConfigBundle | None = None,
    config_dir: str | None = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for path, leaf in _LEAVES.items():
        value = getattr(getattr(bundle, leaf.component), leaf.field)
        if baseline is not None and value == getattr(getattr(baseline, leaf.component), leaf.field):
            continue
        if (
            config_dir is not None
            and leaf.component == "sim_config"
            and leaf.field in {"build_dir", "hand_eye_config"}
            and isinstance(value, str)
            and value
            and os.path.isabs(value)
        ):
            value = os.path.relpath(value, config_dir)
        cursor = out
        parts = path.split(".")
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = _yaml_value(value)
    return out
