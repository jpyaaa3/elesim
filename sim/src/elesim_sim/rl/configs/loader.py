"""Typed loader for the wrap-grasp RL configuration.

Every tunable the RL stack uses is declared here and sourced from YAML.  The
loader is deliberately strict: an unknown key is an error rather than a
silently ignored typo, because a misspelled reward weight that quietly keeps
its default is indistinguishable from a training bug.
"""

from __future__ import annotations

import collections.abc
import copy
import dataclasses
import re
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Mapping, MutableMapping, Optional, Sequence

import yaml

_CONFIG_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = _CONFIG_DIR / "default.yaml"

SCHEMA_VERSION = 1


class ConfigError(ValueError):
    """Raised for malformed or unknown configuration entries."""


# --------------------------------------------------------------------------
# Leaf sections
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RuntimeConfig:
    backend: str = "gpu"
    torch_device: str = "auto"
    n_envs: int = 256
    seed: int = 0
    deterministic: bool = False


@dataclass(frozen=True)
class Go2Config:
    enable: bool = True
    spawn_xyz: Optional[tuple[float, float, float]] = None
    freeze_legs: bool = True
    base_fixed: bool = True
    leg_pose_rad: tuple[float, float, float] = (0.0, 0.9, -1.8)


@dataclass(frozen=True)
class SceneConfig:
    #: Application config the mechanism's parameters are read from.
    app_config: str = "sim/config/config.yaml"
    dt: float = 0.01
    solver_substeps: int = 2
    max_collision_pairs: int = 1024
    show_viewer: bool = False
    decompose_robot_error_threshold: Optional[float] = 0.15
    model_bundle: str = ""
    urdf_relpath: str = "robot.urdf"
    floor: bool = True
    friction: Optional[float] = None
    env_spacing_m: tuple[float, float] = (3.0, 3.0)
    go2: Go2Config = field(default_factory=Go2Config)


@dataclass(frozen=True)
class ArmLimits:
    #: None on any field means "read it from the application config".  The
    #: mechanism's own numbers are the source of truth; hand-copying them into
    #: this file is what produced four separate disagreements with the real
    #: system (see rl/app_config.py).
    linear_m: Optional[tuple[float, float]] = None
    roll_rad: Optional[tuple[float, float]] = None
    bend_per_node_rad: Optional[float] = None
    #: Coupled cap on the two segment curls:
    #: |theta1_curl_weight * theta1 + theta2| <= curl_limit_per_node_rad.
    #: Not from the application config -- it is a self-collision boundary
    #: measured in the built scene, not a mechanism parameter.  None on the
    #: limit disables the cap.
    theta1_curl_weight: float = 1.5
    curl_limit_per_node_rad: Optional[float] = 1.0647


@dataclass(frozen=True)
class ArmGains:
    kp: float = 200.0
    kv: float = 20.0
    force_range: float = 100.0


@dataclass(frozen=True)
class ArmConfig:
    linear_joint: str = "j_plate_housing"
    roll_joint: str = "j_housing_wedge"
    bend_joints: tuple[str, ...] = ()
    n_seg: int = 5
    linear_axis_sign: float = 1.0
    roll_axis_sign: float = 1.0
    bend_axis_sign: float = 1.0
    limits: ArmLimits = field(default_factory=ArmLimits)
    gains: ArmGains = field(default_factory=ArmGains)
    #: Named UI preset to reset to ("home" or "extend_arm"), converted through
    #: the application's own u-space mapping.  An explicit `home_waypoint`
    #: overrides it.
    home_preset: str = "home"
    home_waypoint: Optional[tuple[float, float, float, float]] = None
    segment2_mid_link: str = "node7"
    arm_link_prefixes: tuple[str, ...] = ("node", "wedge", "housing", "gripper")

    def __post_init__(self) -> None:
        if self.bend_joints and len(self.bend_joints) < 2 * self.n_seg:
            raise ConfigError(
                f"arm.bend_joints has {len(self.bend_joints)} entries but "
                f"n_seg={self.n_seg} needs at least {2 * self.n_seg}"
            )


@dataclass(frozen=True)
class SupportConfig:
    """Fixed box the object stands on.

    Wrapping a cylinder that rests on the ground is not feasible for this arm
    (86 deg peak wrap, measured); raising the object is what makes the task
    possible at all.  The support is a collision body like the floor, so
    hitting it is a non-target collision.
    """

    enable: bool = True
    height_m: float = 0.459
    half_extents_xy: tuple[float, float] = (0.065, 0.065)
    center_xy: tuple[float, float] = (0.500, -0.075)


@dataclass(frozen=True)
class ObjectConfig:
    kind: str = "cylinder"
    radius_m: float = 0.045
    height_m: float = 0.20
    mass_kg: float = 0.30
    pos_xyz: Optional[tuple[float, float, float]] = None
    collision: bool = True
    fixed: bool = False
    #: Extra radii to build alongside `radius_m`, one entity each, so different
    #: environments can hold different sizes.  Empty means a single size.
    radius_choices_m: tuple[float, ...] = ()
    #: Where the sizes an environment is not using wait, and how far apart.
    park_xy_m: tuple[float, float] = (6.0, 6.0)
    park_step_m: float = 0.6

    def radius_choices(self) -> tuple[float, ...]:
        """Every radius the scene builds, `radius_m` first.

        First because that is the one the rest of the config is written against
        -- `object.pos_xyz`, the support column, the curl cap -- and the one a
        single-size run gets.
        """
        rest = tuple(
            float(r) for r in self.radius_choices_m
            if abs(float(r) - float(self.radius_m)) > 1e-9
        )
        return (float(self.radius_m),) + rest


@dataclass(frozen=True)
class SettleConfig:
    mode: str = "fixed"
    joint_vel_thresh: float = 0.05
    object_vel_thresh: float = 0.02
    hold_substeps: int = 5
    min_substeps: int = 10


@dataclass(frozen=True)
class RateLimitConfig:
    linear_m: float = 0.04
    roll_rad: float = 0.30
    theta_rad: float = 0.25


@dataclass(frozen=True)
class MacroStepConfig:
    max_steps: int = 20
    substeps: int = 40
    move_fraction: float = 0.6
    settle: SettleConfig = field(default_factory=SettleConfig)
    rate_limit: RateLimitConfig = field(default_factory=RateLimitConfig)

    def __post_init__(self) -> None:
        if self.settle.min_substeps > self.substeps:
            raise ConfigError(
                "macro_step.settle.min_substeps must not exceed macro_step.substeps"
            )


@dataclass(frozen=True)
class BetaConfig:
    """Residual joint model (backlash + deflection).

    The numbers are placeholders from the paper's manual configuration table.
    `measured` stays False until real identification data replaces them, and
    the training/eval reports surface that flag so a run is never mistaken for
    one calibrated against hardware.
    """

    enable: bool = True
    measured: bool = False
    beta0_deg: float = 0.8
    load_slope_deg_per_kg: float = 0.66
    beta0_jitter_deg: float = 0.2
    directional: bool = True
    estimator_gain: float = 0.8
    estimator_noise_deg: float = 0.15
    apply_bundle_sag_model: bool = False


@dataclass(frozen=True)
class RewardWeights:
    coverage_progress: float = 2.0
    enclosure_progress: float = 1.0
    approach_shaping: float = 0.5
    step_cost: float = -0.05
    non_target_collision: float = -1.0
    object_disturbance: float = -0.5
    object_topple: float = -2.0
    success: float = 5.0


@dataclass(frozen=True)
class CoverageConfig:
    source: str = "contact_span"
    n_bins: int = 180
    radial_band_m: float = 0.04
    link_radius_m: float = 0.035
    interpenetration_tol_m: float = 0.002
    require_caging: bool = False

    def __post_init__(self) -> None:
        if self.source not in ("contact_span", "contact", "proximity"):
            raise ConfigError(
                f"unknown reward.coverage.source: {self.source!r}"
            )


@dataclass(frozen=True)
class DisturbanceConfig:
    deadband_m: float = 0.005
    max_displacement_m: float = 0.06
    max_tilt_rad: float = 0.60


@dataclass(frozen=True)
class SelfContactConfig:
    #: Any arm-against-arm contact counts as a collision.
    all_is_failure: bool = False
    #: Link-name prefixes whose self contact counts even when `all_is_failure`
    #: is off: the rigid base the backbone is mounted on.
    structural_prefixes: tuple[str, ...] = ("housing",)
    #: Whether that contact ends the episode as well as costing the penalty.
    terminates: bool = True


@dataclass(frozen=True)
class RewardConfig:
    weights: RewardWeights = field(default_factory=RewardWeights)
    approach_d0: float = 0.20
    approach_shaping_source: str = "nearest_link"

    def __post_init__(self) -> None:
        if self.approach_shaping_source not in ("nearest_link", "anchor_link"):
            raise ConfigError(
                "unknown reward.approach_shaping_source: "
                f"{self.approach_shaping_source!r}"
            )
    self_contact: SelfContactConfig = field(default_factory=SelfContactConfig)
    coverage: CoverageConfig = field(default_factory=CoverageConfig)
    disturbance: DisturbanceConfig = field(default_factory=DisturbanceConfig)


@dataclass(frozen=True)
class TugConfig:
    trigger_rad: float = 2.0944
    force_scale: float = 1.0
    hold_substeps: int = 120
    max_rel_translation_m: float = 0.03
    max_rel_rotation_rad: float = 0.5


@dataclass(frozen=True)
class LiftConfig:
    trigger_rad: float = 2.0944
    roll_target_rad: float = 0.0
    roll_rate_rad_per_substep: float = 0.015
    #: Substeps between the end of the rotation and the start of the measured
    #: hold.  The lift lays the object down into the coil, so it moves a long
    #: way relative to the arm on purpose; the tolerances below are about
    #: whether it then stays put, and they are anchored after this window.
    settle_substeps: int = 80
    hold_substeps: int = 100
    max_rel_translation_m: float = 0.03
    max_rel_rotation_rad: float = 0.5
    #: The object's lowest point must clear the floor by this much for the
    #: whole measured hold: an object resting on the ground is not being held,
    #: whatever its pose relative to the arm.
    min_clearance_m: float = 0.05
    #: ...and the arm must still be touching it.
    min_object_contacts: int = 2


@dataclass(frozen=True)
class StartPoseConfig:
    """Reverse curriculum on the arm's reset pose."""

    enable: bool = True
    #: Waypoint the episode can start from instead of Home: the open coil
    #: already positioned around the object, which is where a scripted wrap is
    #: at macro step 7.  From here success is three or four steps away.
    near_waypoint: tuple[float, float, float, float] = (-0.077, -1.5708, 0.0, -0.1047)
    #: Interpolation from Home (0) to `near_waypoint` (1), sampled per env.
    t_range: tuple[float, float] = (0.85, 1.0)
    #: Once the success rate over `window` episodes clears `advance_at`, the
    #: range moves `step` towards Home.  None never advances.
    advance_at: Optional[float] = 0.5
    #: ...and if it falls to `retreat_at` the range moves back the same step.
    #: Without this the curriculum is a ratchet: it outran the policy, success
    #: went to zero, and there was no way back -- 130 iterations at exactly
    #: `success 0.0000, phi 4 deg, topple 0.42`.  None never retreats.
    retreat_at: Optional[float] = 0.05
    step: float = 0.1
    window: int = 512
    #: Policy updates that must pass between two moves.  A guard rather than
    #: the fix for the run above, which moved four steps in fifty iterations and
    #: would have passed this: `retreat_at` is what that needed.  It bounds a
    #: real hazard all the same -- at 2048 envs a 512-episode window fills in
    #: about two macro steps, so nothing else stops the range walking several
    #: steps within one iteration if the success rate happens to hold up.
    cooldown_updates: int = 5


@dataclass(frozen=True)
class SuccessConfig:
    #: Keep in step with `success.criterion` in default.yaml.
    #: `resolved_for_curriculum` treats "differs from this default" as "the
    #: caller asked for it explicitly" and lets it win over the stage; if this
    #: drifts from the file, every run looks like an explicit choice and the
    #: stages stop being able to set the criterion at all.
    criterion: str = "lift"
    coverage_target_rad: float = 3.0019
    lift: LiftConfig = field(default_factory=LiftConfig)
    tug: TugConfig = field(default_factory=TugConfig)

    def __post_init__(self) -> None:
        if self.criterion not in ("geometric", "lift", "tug"):
            raise ConfigError(f"unknown success.criterion: {self.criterion!r}")


@dataclass(frozen=True)
class ObsNoiseConfig:
    joint_rad: float = 0.005
    object_pos_m: float = 0.004
    object_rot_rad: float = 0.02
    load_proxy: float = 0.02


@dataclass(frozen=True)
class ActorObsConfig:
    include_joint_estimate: bool = True
    include_object_geometry: bool = True
    include_load_proxy: bool = True
    include_step_index: bool = True
    noise: ObsNoiseConfig = field(default_factory=ObsNoiseConfig)
    delay_steps: tuple[int, int] = (0, 2)


@dataclass(frozen=True)
class CriticObsConfig:
    include_true_joint_state: bool = True
    include_contact_forces: bool = True
    include_true_object_pose: bool = True
    include_coverage: bool = True


@dataclass(frozen=True)
class ObservationConfig:
    actor: ActorObsConfig = field(default_factory=ActorObsConfig)
    critic_privileged: CriticObsConfig = field(default_factory=CriticObsConfig)


@dataclass(frozen=True)
class DomainRandomisationConfig:
    enable: bool = True
    friction: tuple[float, float] = (0.6, 1.2)
    object_mass_kg: tuple[float, float] = (0.15, 0.60)
    object_radius_m: tuple[float, float] = (0.035, 0.060)
    object_pos_jitter_m: tuple[float, float, float] = (0.03, 0.03, 0.01)
    object_yaw_jitter_rad: float = 0.35


@dataclass(frozen=True)
class CurriculumStage:
    randomise_object_pose: bool = False
    randomise_object_radius: bool = False
    approach_shaping: bool = True
    success_criterion: str = "tug"


@dataclass(frozen=True)
class CurriculumConfig:
    stage: int = 1
    stages: Mapping[int, CurriculumStage] = field(default_factory=dict)

    def active(self) -> CurriculumStage:
        try:
            return self.stages[int(self.stage)]
        except KeyError as exc:
            known = sorted(self.stages)
            raise ConfigError(
                f"curriculum.stage={self.stage} is not defined; known stages: {known}"
            ) from exc


@dataclass(frozen=True)
class TrainConfig:
    max_iterations: int = 1500
    save_interval: int = 50
    experiment_name: str = "wrap_grasp"
    run_name: str = ""
    log_dir: str = "sim/rl_runs"
    resume: str = ""
    num_steps_per_env: int = 16
    #: Passed straight through to rsl_rl.  Deliberately untyped: rsl_rl owns
    #: this schema and it moves between versions, so validating it here would
    #: mean maintaining a second copy that silently drifts.
    runner: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class BenchmarkConfig:
    n_envs_sweep: tuple[int, ...] = (256, 1024, 4096, 8192)
    warmup_steps: int = 20
    measure_steps: int = 200
    with_contact: bool = True
    wrap_pose: tuple[float, float, float, float] = (0.0, -1.5708, 0.4712, 0.4712)
    out_path: str = "sim/benchmarks/step_rate.md"


@dataclass(frozen=True)
class PoseOffsetGrid:
    """Offsets from the configured object centre, in metres."""

    x_m: tuple[float, ...] = (-0.03, 0.0, 0.03)
    y_m: tuple[float, ...] = (0.0,)
    yaw_rad: tuple[float, ...] = (-0.3, 0.0, 0.3)


@dataclass(frozen=True)
class EvalConfig:
    episodes_per_condition: int = 20
    pose_offset_grid: PoseOffsetGrid = field(default_factory=PoseOffsetGrid)
    radius_grid_m: tuple[float, ...] = (0.035, 0.045, 0.060)
    render_episodes: int = 1
    out_dir: str = "sim/rl_runs/eval"


@dataclass(frozen=True)
class WrapGraspConfig:
    schema_version: int = SCHEMA_VERSION
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    scene: SceneConfig = field(default_factory=SceneConfig)
    arm: ArmConfig = field(default_factory=ArmConfig)
    support: SupportConfig = field(default_factory=SupportConfig)
    object: ObjectConfig = field(default_factory=ObjectConfig)
    macro_step: MacroStepConfig = field(default_factory=MacroStepConfig)
    beta: BetaConfig = field(default_factory=BetaConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    success: SuccessConfig = field(default_factory=SuccessConfig)
    observation: ObservationConfig = field(default_factory=ObservationConfig)
    domain_randomisation: DomainRandomisationConfig = field(
        default_factory=DomainRandomisationConfig
    )
    curriculum: CurriculumConfig = field(default_factory=CurriculumConfig)
    start_pose: StartPoseConfig = field(default_factory=StartPoseConfig)
    train: TrainConfig = field(default_factory=TrainConfig)
    benchmark: BenchmarkConfig = field(default_factory=BenchmarkConfig)
    eval: EvalConfig = field(default_factory=EvalConfig)

    def resolved_from_app(self) -> "WrapGraspConfig":
        """Fill every unset mechanism value from the application config."""
        from ..app_config import load_mechanism

        mech = load_mechanism(self.scene.app_config)
        limits = self.arm.limits
        limits = dataclasses.replace(
            limits,
            linear_m=limits.linear_m or mech.linear_m,
            roll_rad=limits.roll_rad or mech.roll_rad,
            bend_per_node_rad=(
                limits.bend_per_node_rad
                if limits.bend_per_node_rad is not None
                else mech.bend_per_node_rad
            ),
        )
        home = self.arm.home_waypoint
        if home is None:
            try:
                home = mech.presets[str(self.arm.home_preset)]
            except KeyError as exc:
                known = sorted(mech.presets)
                raise ConfigError(
                    f"arm.home_preset={self.arm.home_preset!r} is not a known UI "
                    f"preset; known: {known}"
                ) from exc
        arm = dataclasses.replace(self.arm, limits=limits, home_waypoint=home)

        go2 = self.scene.go2
        if go2.spawn_xyz is None:
            go2 = dataclasses.replace(
                go2, spawn_xyz=(0.0, 0.0, mech.go2_spawn_height_m)
            )
        scene = dataclasses.replace(self.scene, go2=go2)
        return dataclasses.replace(self, arm=arm, scene=scene)

    def object_center(self) -> tuple[float, float, float]:
        """Reset position of the object centre.

        With `object.pos_xyz` unset the cylinder is seated on the support, so
        the two cannot silently drift apart when the support height changes.
        """
        if self.object.pos_xyz is not None:
            return self.object.pos_xyz
        if not self.support.enable:
            raise ConfigError(
                "object.pos_xyz must be set explicitly when support.enable is false"
            )
        cx, cy = self.support.center_xy
        return (
            float(cx),
            float(cy),
            float(self.support.height_m) + float(self.object.height_m) * 0.5,
        )

    def resolved_for_curriculum(self) -> "WrapGraspConfig":
        """Apply the active curriculum stage on top of the base config.

        The stage is the single switch the spec asks for; it overrides the
        randomisation flags and the success criterion so a stage change never
        requires editing the other sections by hand.
        """
        stage = self.curriculum.active()
        dr = self.domain_randomisation
        if not stage.randomise_object_pose:
            dr = dataclasses.replace(
                dr, object_pos_jitter_m=(0.0, 0.0, 0.0), object_yaw_jitter_rad=0.0
            )
        if not stage.randomise_object_radius:
            r = self.object.radius_m
            dr = dataclasses.replace(dr, object_radius_m=(r, r))
        reward = self.reward
        if not stage.approach_shaping:
            reward = dataclasses.replace(
                reward,
                weights=dataclasses.replace(reward.weights, approach_shaping=0.0),
            )
        # The stage supplies the criterion, but not over an explicit choice.
        #
        # `--set success.criterion=lift` used to be swallowed here: the stage's
        # value replaced it and the run went on with `tug`, reporting a success
        # rate for a test nobody asked for.  It cost three measurements before
        # the pattern was spotted.  A criterion that differs from the schema
        # default is taken as deliberate and kept.
        default_criterion = type(self.success)().criterion
        chosen = (
            self.success.criterion
            if self.success.criterion != default_criterion
            else stage.success_criterion
        )
        success = dataclasses.replace(self.success, criterion=chosen)
        return dataclasses.replace(
            self, domain_randomisation=dr, reward=reward, success=success
        )


# --------------------------------------------------------------------------
# Construction
# --------------------------------------------------------------------------


def _coerce(value: Any, annotation: Any, path: str) -> Any:
    """Convert a YAML value into the annotated dataclass field type."""
    origin = getattr(annotation, "__origin__", None)
    args = getattr(annotation, "__args__", ())

    # Optional[X] -> X or None
    if origin is not None and type(None) in args:
        if value is None:
            return None
        inner = next(a for a in args if a is not type(None))
        return _coerce(value, inner, path)

    if is_dataclass(annotation):
        if not isinstance(value, Mapping):
            raise ConfigError(f"{path}: expected a mapping, got {type(value).__name__}")
        return _build(annotation, value, path)

    if origin is tuple:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ConfigError(f"{path}: expected a sequence")
        # tuple[X, ...] (homogeneous) vs tuple[X, Y, Z] (fixed arity)
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_coerce(v, args[0], f"{path}[{i}]") for i, v in enumerate(value))
        if len(args) != len(value):
            raise ConfigError(
                f"{path}: expected {len(args)} entries, got {len(value)}"
            )
        return tuple(
            _coerce(v, a, f"{path}[{i}]") for i, (v, a) in enumerate(zip(value, args))
        )

    if origin in (collections.abc.Mapping, collections.abc.MutableMapping, dict):
        if not isinstance(value, Mapping):
            raise ConfigError(f"{path}: expected a mapping")
        key_t, val_t = args
        return {
            _coerce(k, key_t, f"{path}.{k}"): _coerce(v, val_t, f"{path}.{k}")
            for k, v in value.items()
        }

    if annotation is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{path}: expected a boolean, got {value!r}")
        return value
    if annotation is int:
        # A digit string counts.  YAML gives mapping keys as integers, but a
        # `--set curriculum.stages.3.approach_shaping=true` reaches here with
        # "3": the override path is split on dots, so every segment arrives as
        # text.  Rejecting it made a whole branch of the config unreachable from
        # the command line.
        if isinstance(value, str) and re.fullmatch(r"[+-]?\d+", value.strip()):
            return int(value.strip())
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{path}: expected an integer, got {value!r}")
        return int(value)
    if annotation is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{path}: expected a number, got {value!r}")
        return float(value)
    if annotation is str:
        if not isinstance(value, str):
            raise ConfigError(f"{path}: expected a string, got {value!r}")
        return value
    return value


def _build(cls: type, data: Mapping[str, Any], path: str = "") -> Any:
    known = {f.name: f for f in fields(cls)}
    unknown = sorted(set(data) - set(known))
    if unknown:
        where = path or cls.__name__
        raise ConfigError(f"{where}: unknown configuration keys {unknown}")
    kwargs: dict[str, Any] = {}
    for name, value in data.items():
        child = f"{path}.{name}" if path else name
        kwargs[name] = _coerce(value, known[name].type, child)
    return cls(**kwargs)


def _existing_key(mapping: Mapping[str, Any], key: str) -> Any:
    """The key already in `mapping` that this path segment names.

    An override path is split on dots, so every segment arrives as text, while
    YAML reads `curriculum.stages`' keys as integers.  Looking up "3" then found
    nothing, so the walk created a *second* entry under the string and the
    original was shadowed -- and the coercion, mapping both to the integer 3,
    kept whichever came last.  A `--set curriculum.stages.3.approach_shaping`
    therefore silently reset that stage's other fields to their defaults, which
    is how a run meant to randomise object size ended up randomising nothing.
    """
    if key in mapping:
        return key
    try:
        numeric = int(key)
    except (TypeError, ValueError):
        return key
    return numeric if numeric in mapping else key


def _deep_merge(base: MutableMapping[str, Any], overlay: Mapping[str, Any]) -> None:
    for key, value in overlay.items():
        if (
            key in base
            and isinstance(base[key], MutableMapping)
            and isinstance(value, Mapping)
        ):
            _deep_merge(base[key], value)
        else:
            base[key] = copy.deepcopy(value)


def _resolve_annotations() -> None:
    """Turn the string annotations from `from __future__ import annotations`
    into real types so `_coerce` can dispatch on them."""
    import typing

    module_ns = dict(globals())
    for cls in list(module_ns.values()):
        if is_dataclass(cls) and getattr(cls, "__module__", "") == __name__:
            hints = typing.get_type_hints(cls, globalns=module_ns)
            for f in fields(cls):
                f.type = hints.get(f.name, f.type)


_resolve_annotations()


def parse_override(text: str) -> tuple[list[str], Any]:
    """Parse a `--set a.b.c=value` token into a key path and a YAML value."""
    if "=" not in text:
        raise ConfigError(f"override must be key=value, got {text!r}")
    key, _, raw = text.partition("=")
    keys = [k for k in key.strip().split(".") if k]
    if not keys:
        raise ConfigError(f"override has an empty key: {text!r}")
    return keys, yaml.safe_load(raw)


def load_config(
    path: Optional[str | Path] = None,
    *,
    overlays: Sequence[str | Path] = (),
    overrides: Sequence[str] = (),
) -> WrapGraspConfig:
    """Load `default.yaml`, then overlay files, then `key=value` overrides."""
    base_path = Path(path) if path else DEFAULT_CONFIG_PATH
    with open(base_path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, MutableMapping):
        raise ConfigError(f"{base_path}: top level must be a mapping")

    for overlay_path in overlays:
        with open(overlay_path, "r", encoding="utf-8") as handle:
            overlay = yaml.safe_load(handle) or {}
        if not isinstance(overlay, Mapping):
            raise ConfigError(f"{overlay_path}: top level must be a mapping")
        _deep_merge(raw, overlay)

    for override in overrides:
        keys, value = parse_override(override)
        cursor: MutableMapping[str, Any] = raw
        for key in keys[:-1]:
            key = _existing_key(cursor, key)
            nxt = cursor.get(key)
            if not isinstance(nxt, MutableMapping):
                nxt = {}
                cursor[key] = nxt
            cursor = nxt
        cursor[_existing_key(cursor, keys[-1])] = value

    version = raw.get("schema_version", SCHEMA_VERSION)
    if version != SCHEMA_VERSION:
        raise ConfigError(
            f"config schema_version {version} != supported {SCHEMA_VERSION}"
        )
    return _build(WrapGraspConfig, raw).resolved_from_app()


def to_dict(cfg: Any) -> Any:
    """Plain-data view of a config, for logging and run reproduction."""
    if is_dataclass(cfg):
        return {f.name: to_dict(getattr(cfg, f.name)) for f in fields(cfg)}
    if isinstance(cfg, Mapping):
        return {k: to_dict(v) for k, v in cfg.items()}
    if isinstance(cfg, tuple):
        return [to_dict(v) for v in cfg]
    return cfg
