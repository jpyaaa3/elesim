"""Configuration values owned by the Genesis deployment."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import elesim_protocol.messages as proto

from elesim_sim.robot.arm.joint_defs import JointLimit
from elesim_sim.robot.go2.locomotion.config import Go2LocomotionConfig


def _default_build_dir() -> Path:
    for root in Path(__file__).resolve().parents:
        candidate = root / "payload/data/models/assemblies/zed-mini"
        if candidate.is_dir():
            return candidate
    return Path("/opt/elesim/data/models/assemblies/zed-mini")


DEFAULT_BUILD_DIR = str(_default_build_dir())


@dataclass(frozen=True)
class SimParam:
    dt: float = 0.01
    substeps: int = 1
    realtime: bool = True
    realtime_factor: float = 1.0
    gravity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    roll_rate: float = float("inf")
    bend_rate: float = float("inf")


@dataclass(frozen=True)
class SimConfig:
    use_gpu: bool = True
    # Keep Genesis backend selection independent from camera post-processing.
    # This switch allows an A/B profile with the same GPU-rendered scene while
    # forcing the legacy host conversion path.
    camera_gpu_convert: bool = True
    # ``async_process`` keeps Genesis camera rendering out of the physics
    # loop.  ``sync_legacy`` is retained only for diagnostics and compatibility
    # with camera unit tests that do not start a render worker.
    camera_execution: str = "async_process"
    camera_worker_start_timeout_s: float = 180.0
    camera_first_frame_timeout_s: float = 30.0
    enable_viewer: bool = True
    # Native desktop viewer refresh cadence.  Zero preserves Genesis' legacy
    # per-physics-step visualizer update; headless profiles leave the viewer
    # disabled and never enter this path.
    visualizer_max_hz: float = 30.0
    telemetry_max_hz: float = 20.0
    floor: bool = True
    use_hardware: bool = False
    use_go2: bool = False

    build_dir: str = DEFAULT_BUILD_DIR
    assy_build_json: str = "blueprint.json"
    urdf_name: str = "robot.urdf"
    arm_urdf_name: str = "arm.urdf"

    camera_profile: str = "zed_mini"
    hand_eye_config: str = ""
    sim_camera_enable: bool = True
    sim_camera_jpeg: bool = True
    sim_camera_jpeg_quality: int = 85
    sim_camera_rgb: bool = True
    sim_camera_depth: bool = True
    sim_camera_max_hz: float = 30.0
    sim_camera_width: int = 640
    sim_camera_height: int = 480
    sim_camera_fov_deg: float = 60.0

    sim_observer_camera_enable: bool = True
    sim_observer_camera_jpeg: bool = True
    sim_observer_camera_jpeg_quality: int = 85
    sim_observer_camera_max_hz: float = 20.0
    sim_observer_camera_width: int = 640
    sim_observer_camera_height: int = 480
    sim_observer_camera_fov_deg: float = 40.0
    sim_observer_camera_pos: tuple[float, float, float] = (3.5, 0.5, 2.5)
    sim_observer_camera_lookat: tuple[float, float, float] = (0.0, 0.0, 0.5)

    perf_log_enable: bool = True
    perf_log_interval_s: float = 2.0
    perf_log_path: str = ""


@dataclass(frozen=True)
class ArmMappingConfig:
    """Only the display-to-canonical mapping needed for reset compatibility."""

    command_direction: tuple[int, int, int, int] = (1, 1, 1, -1)
    linear_u_max_deg: float = 250.0
    linear_u_limit_deg: float = 250.0


@dataclass(frozen=True)
class UrdfExportConfig:
    """Development-rebuild options; production consumes an immutable bundle."""

    robot_name: str = "Robot"
    default_effort: float = 200.0
    default_velocity: float = 3.0
    revolute_effort: Optional[float] = None
    revolute_velocity: Optional[float] = None
    prismatic_effort: Optional[float] = None
    prismatic_velocity: Optional[float] = None
    revolute_damping: float = 0.12
    revolute_friction: float = 0.06
    prismatic_damping: float = 60.0
    prismatic_friction: float = 20.0
    mesh_basename_only: bool = False
    part_color_rgba_by_name: dict[str, tuple[float, float, float, float]] = field(default_factory=dict)


@dataclass(frozen=True)
class SpawnConfig:
    pitch: float = 0.05
    n_seg: Optional[int] = None
    spawn_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    spawn_euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    draw_debug_markers: bool = True

    go2_spawn_height: float = 0.42
    go2_mount_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.08)
    go2_spawn_euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    go2_teleop_vx_mps: float = 0.35
    go2_teleop_vy_mps: float = 0.25
    go2_teleop_wz_radps: float = 0.80

    sim_target_enable: bool = True
    sim_target_xyz: tuple[float, float, float] = (0.8, 0.0, 0.2)
    sim_target_radius: float = 0.025
    #: "sphere" or "cylinder".  A sphere is the perception target this started
    #: as; a standing cylinder is what a whole-arm wrap actually closes around,
    #: and what the wrap-grasp policy was trained against.
    sim_target_shape: str = "sphere"
    #: Cylinder length along z.  Ignored by the sphere.
    sim_target_height: float = 1.1
    #: Override the mass Genesis derives from the geometry.  Left unset
    #: the object weighs what the wrap-grasp training scene built with the
    #: same morph weighed, which is the point of comparison; set it only
    #: to deliberately move away from that.
    sim_target_mass_kg: Optional[float] = None
    sim_target_color_rgba: tuple[float, float, float, float] = (0.85, 0.15, 0.15, 1.0)
    sim_target_collision: bool = True
    sim_target_gravity: bool = False


@dataclass(frozen=True)
class AppConfigBundle:
    sim_param: SimParam
    sim_config: SimConfig
    arm_mapping_config: ArmMappingConfig
    joint_limit: JointLimit
    spawn_config: SpawnConfig
    urdf_export_config: UrdfExportConfig
    go2_locomotion_config: Go2LocomotionConfig
    mapping_config: proto.SimMappingConfig
