"""Strict ownership-oriented YAML schema mapped onto runtime dataclasses."""

from __future__ import annotations

import os
import types
from dataclasses import Field, fields, replace
from typing import Any, Mapping, Union, get_args, get_origin, get_type_hints

from elesim_pilot.config.defaults import build_mapping_config, default_app_config_bundle
from elesim_pilot.config.schema import (
    AppConfigBundle,
    ExperimentConfig,
    GazeStabilizerConfig,
    HardwareConfig,
    IkConfig,
    JointLimit,
    PerceptionConfig,
    PickConfig,
    SimConfig,
    SimParam,
    SpawnConfig,
    UrdfExportConfig,
)
from elesim_pilot.vision.perception.camera_profile import (
    CameraProfileError,
    camera_profile,
    validate_bundle_path,
    validate_calibration_path,
)


class ConfigValidationError(ValueError):
    """A YAML config value does not satisfy the canonical schema."""


_COMPONENT_TYPES: dict[str, type[Any]] = {
    "sim_param": SimParam,
    "sim_config": SimConfig,
    "hardware_config": HardwareConfig,
    "joint_limit": JointLimit,
    "spawn_config": SpawnConfig,
    "urdf_export_config": UrdfExportConfig,
    "ik_config": IkConfig,
    "perception_config": PerceptionConfig,
    "pick_config": PickConfig,
    "gaze_stabilizer_config": GazeStabilizerConfig,
    "experiment_config": ExperimentConfig,
}


class _Leaf:
    __slots__ = ("component", "field")

    def __init__(self, component: str, field: str) -> None:
        self.component = component
        self.field = field


_LEAVES: dict[str, _Leaf] = {}
_ASSIGNED: dict[str, set[str]] = {name: set() for name in _COMPONENT_TYPES}


def _register(
    component: str,
    path: str,
    names: list[str] | tuple[str, ...] | set[str],
    *,
    strip_prefix: str = "",
    aliases: Mapping[str, str] | None = None,
) -> None:
    aliases = dict(aliases or {})
    if isinstance(names, set):
        names = sorted(names)
    for name in names:
        key = aliases.get(name, name[len(strip_prefix) :] if strip_prefix and name.startswith(strip_prefix) else name)
        dotted = f"{path}.{key}"
        if dotted in _LEAVES:
            raise RuntimeError(f"duplicate YAML config path: {dotted}")
        if name in _ASSIGNED[component]:
            raise RuntimeError(f"duplicate config field assignment: {component}.{name}")
        _LEAVES[dotted] = _Leaf(component, name)
        _ASSIGNED[component].add(name)


def _field_names(cls: type[Any]) -> set[str]:
    return {field.name for field in fields(cls)}


def _prefixed(cls: type[Any], prefix: str) -> set[str]:
    return {name for name in _field_names(cls) if name.startswith(prefix)}


# Simulation and transport.
_register("sim_param", "simulation.physics", _field_names(SimParam))
_register(
    "sim_config",
    "simulation.runtime",
    {"use_gpu", "enable_viewer", "floor", "use_hardware", "use_go2", "show_all_ports"},
)
_register(
    "sim_config",
    "simulation.assembly",
    {"build_dir", "assy_build_json", "urdf_name", "arm_urdf_name", "rebuild_assembly"},
)
_register(
    "sim_config",
    "simulation.trajectory.default",
    _prefixed(SimConfig, "traj_") - _prefixed(SimConfig, "traj_lji_"),
    strip_prefix="traj_",
)
_register(
    "sim_config",
    "simulation.trajectory.lji",
    _prefixed(SimConfig, "traj_lji_"),
    strip_prefix="traj_lji_",
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
# Arm, world, and generated URDF.
_register("hardware_config", "robot.arm.hardware", _field_names(HardwareConfig))
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
_register("spawn_config", "world.debug", {"draw_debug_markers"}, aliases={"draw_debug_markers": "markers"})
_register(
    "urdf_export_config",
    "robot.arm.urdf",
    _field_names(UrdfExportConfig),
    aliases={"part_color_rgba_by_name": "colors"},
)
_register("ik_config", "robot.arm.ik", _field_names(IkConfig))

# Vision.
_perception_tracking = _prefixed(PerceptionConfig, "track_") | {"reacquire_on_lost"}
_perception_publish = {
    "publish_hz",
    "sim_camera_topic",
}
_perception_detector = {
    "detector_config",
    "detector",
    "target_label",
    "yolo_device",
    "pipeline",
}
_perception_runtime_injected = {
    "sim_camera_dds_settings",
    "sim_camera_source_id",
    "sim_camera_source_boot_id",
}
_perception_runtime = (
    _field_names(PerceptionConfig)
    - _perception_tracking
    - _perception_publish
    - _perception_detector
    - {"camera_profile"}
    - _perception_runtime_injected
)
_register(
    "perception_config",
    "vision.perception.tracking",
    _perception_tracking - {"reacquire_on_lost"},
    strip_prefix="track_",
)
_register("perception_config", "vision.perception.tracking", {"reacquire_on_lost"})
_register("perception_config", "vision.perception.publishing", _perception_publish)
_register("perception_config", "vision.perception.detector", _perception_detector)
_register(
    "perception_config",
    "vision.perception.camera",
    {"camera_profile"},
    aliases={"camera_profile": "profile"},
)
_register("perception_config", "vision.perception.runtime", _perception_runtime)
# These values are filled from the selected DDS peer descriptor at runtime.
# Treating them as mapped would accidentally make process identity writable in
# the static application YAML, so only mark them as intentionally owned here.
_ASSIGNED["perception_config"].update(_perception_runtime_injected)

# Accept the pre-profile spelling while emitting the canonical ``profile``
# field above.  This keeps hand-eye and perception camera configuration names
# consistent without silently accepting an unknown field.
_PATH_ALIASES = {
    "simulation.cameras.hand_eye.camera_profile": "simulation.cameras.hand_eye.profile",
}

# Pick workflow. Prefixes become explicit workflow subsections.
_pick_groups = (
    ("ready_pose_", "behaviors.pick.ready"),
    ("approach_", "behaviors.pick.approach"),
    ("grasp_", "behaviors.pick.grasp"),
    ("sag_", "behaviors.pick.sag"),
    ("lij_", "behaviors.pick.lji"),
    ("look_", "behaviors.pick.look"),
    ("ik_align_", "behaviors.pick.alignment"),
    ("mobile_", "behaviors.pick.mobile"),
)
for _prefix, _path in _pick_groups:
    _register(
        "pick_config",
        _path,
        _prefixed(PickConfig, _prefix) - _ASSIGNED["pick_config"],
        strip_prefix=_prefix,
    )
_register(
    "pick_config",
    "behaviors.pick.lji",
    {"local_img_jacobian_enabled"},
    aliases={"local_img_jacobian_enabled": "enabled"},
)
_register(
    "pick_config",
    "behaviors.pick.general",
    _field_names(PickConfig) - _ASSIGNED["pick_config"],
)

# Gaze workflow.
_gaze_preview = _prefixed(GazeStabilizerConfig, "preview_")
_gaze_center = _prefixed(GazeStabilizerConfig, "center_") | {"uv_gain", "step_scale", "fine_err_max", "fine_settle_scale"}
_gaze_command = _prefixed(GazeStabilizerConfig, "command_ref_")
_register(
    "gaze_stabilizer_config",
    "behaviors.gaze.preview",
    _gaze_preview,
    strip_prefix="preview_",
)
_register(
    "gaze_stabilizer_config",
    "behaviors.gaze.centering",
    _gaze_center,
    strip_prefix="center_",
)
_register(
    "gaze_stabilizer_config",
    "behaviors.gaze.command_reference",
    _gaze_command,
    strip_prefix="command_ref_",
)
_register(
    "gaze_stabilizer_config",
    "behaviors.gaze.general",
    _field_names(GazeStabilizerConfig) - _ASSIGNED["gaze_stabilizer_config"],
)
_register("experiment_config", "experiment", _field_names(ExperimentConfig))


for _component, _cls in _COMPONENT_TYPES.items():
    _missing = _field_names(_cls) - _ASSIGNED[_component]
    if _missing:
        raise RuntimeError(f"YAML schema does not map {_component}: {sorted(_missing)}")


_TYPE_HINTS: dict[str, dict[str, Any]] = {
    name: get_type_hints(cls) for name, cls in _COMPONENT_TYPES.items()
}


def _coerce_scalar(value: Any, expected: type[Any], path: str) -> Any:
    if expected is bool:
        if type(value) is not bool:
            raise ConfigValidationError(f"{path}: expected boolean, got {type(value).__name__}")
        return value
    if expected is int:
        if type(value) is not int:
            raise ConfigValidationError(f"{path}: expected integer, got {type(value).__name__}")
        return value
    if expected is float:
        if type(value) not in (int, float):
            raise ConfigValidationError(f"{path}: expected number, got {type(value).__name__}")
        return float(value)
    if expected is str:
        if type(value) is not str:
            raise ConfigValidationError(f"{path}: expected string, got {type(value).__name__}")
        return value
    return value


def _coerce(value: Any, annotation: Any, path: str) -> Any:
    origin = get_origin(annotation)
    args = get_args(annotation)
    if origin in (Union, types.UnionType):
        if value is None and type(None) in args:
            return None
        failures: list[str] = []
        for choice in args:
            if choice is type(None):
                continue
            try:
                return _coerce(value, choice, path)
            except ConfigValidationError as exc:
                failures.append(str(exc))
        raise ConfigValidationError(f"{path}: value does not match any allowed type")
    if origin in (tuple,):
        if not isinstance(value, list):
            raise ConfigValidationError(f"{path}: expected YAML sequence")
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_coerce(item, args[0], f"{path}[{idx}]") for idx, item in enumerate(value))
        if len(value) != len(args):
            raise ConfigValidationError(f"{path}: expected {len(args)} items, got {len(value)}")
        return tuple(_coerce(item, item_type, f"{path}[{idx}]") for idx, (item, item_type) in enumerate(zip(value, args)))
    if origin in (dict,):
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
        label = prefix or "<root>"
        raise ConfigValidationError(f"{label}: expected mapping")
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
        current = getattr(defaults, component)
        components[component] = replace(current, **updates[component])

    sim_config = components["sim_config"]
    sim_paths: dict[str, str] = {}
    for field_name in ("build_dir", "hand_eye_config"):
        value = getattr(sim_config, field_name)
        if value and not os.path.isabs(value):
            sim_paths[field_name] = os.path.abspath(os.path.join(config_dir, value))
    if sim_paths:
        sim_config = replace(sim_config, **sim_paths)
        components["sim_config"] = sim_config

    try:
        sim_profile = camera_profile(sim_config.camera_profile)
        perception_profile = camera_profile(components["perception_config"].camera_profile)
        if sim_profile.name != perception_profile.name:
            raise ConfigValidationError(
                "simulation.cameras.hand_eye.profile and "
                "vision.perception.camera.profile must select the same camera"
            )
        validate_bundle_path(sim_profile.name, sim_config.build_dir)
        if sim_config.hand_eye_config:
            validate_calibration_path(sim_profile.name, sim_config.hand_eye_config)
    except CameraProfileError as exc:
        raise ConfigValidationError(str(exc)) from exc

    perception_config = components["perception_config"]
    detector_config = str(perception_config.detector_config).strip()
    if detector_config and not os.path.isabs(detector_config):
        components["perception_config"] = replace(
            perception_config,
            detector_config=os.path.abspath(os.path.join(config_dir, detector_config)),
        )

    mapping = build_mapping_config(components["joint_limit"], components["hardware_config"])
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
    """Serialize canonical leaves, optionally emitting only baseline differences."""
    out: dict[str, Any] = {}
    for path, leaf in _LEAVES.items():
        value = getattr(getattr(bundle, leaf.component), leaf.field)
        if baseline is not None:
            base_value = getattr(getattr(baseline, leaf.component), leaf.field)
            if value == base_value:
                continue
        if (
            config_dir is not None
            and (
                (leaf.component == "sim_config" and leaf.field in {"build_dir", "hand_eye_config"})
                or (leaf.component == "perception_config" and leaf.field == "detector_config")
            )
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
