"""Configuration values owned by the Genesis deployment."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import elesim_protocol.messages as proto

from elesim_simulator.robot.arm.joint_defs import JointLimit
from elesim_simulator.robot.go2.locomotion.config import Go2LocomotionConfig


DEPLOYMENT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_BUILD_DIR = str(DEPLOYMENT_ROOT / "model")


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
    enable_viewer: bool = True
    floor: bool = True
    use_hardware: bool = False
    use_go2: bool = False

    build_dir: str = DEFAULT_BUILD_DIR
    assy_build_json: str = "blueprint.json"
    urdf_name: str = "robot.urdf"
    arm_urdf_name: str = "arm.urdf"

    hand_eye_config: str = ""
    sim_camera_enable: bool = True
    sim_camera_port: str = "tcp://127.0.0.1:5568"
    sim_camera_jpeg: bool = True
    sim_camera_jpeg_quality: int = 85
    sim_camera_rgb: bool = True
    sim_camera_depth: bool = True
    sim_camera_max_hz: float = 30.0
    sim_camera_width: int = 640
    sim_camera_height: int = 480
    sim_camera_fov_deg: float = 60.0

    sim_observer_camera_enable: bool = True
    sim_observer_camera_port: str = "tcp://127.0.0.1:5569"
    sim_observer_camera_jpeg: bool = True
    sim_observer_camera_jpeg_quality: int = 85
    sim_observer_camera_max_hz: float = 20.0
    sim_observer_camera_width: int = 960
    sim_observer_camera_height: int = 540
    sim_observer_camera_fov_deg: float = 55.0
    sim_observer_camera_pos: tuple[float, float, float] = (0.45, -1.8, 0.55)
    sim_observer_camera_lookat: tuple[float, float, float] = (0.45, 0.0, 0.25)

    perf_log_enable: bool = False
    perf_log_interval_s: float = 2.0
    perf_log_path: str = ""


@dataclass(frozen=True)
class ArmMappingConfig:
    """Only the display-to-canonical mapping needed for reset compatibility."""

    command_direction: tuple[int, int, int, int] = (1, -1, 1, -1)
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
    sim_target_color_rgba: tuple[float, float, float, float] = (0.85, 0.15, 0.15, 1.0)
    sim_target_collision: bool = True
    sim_target_gravity: bool = False

    planned_move_target_enable: bool = True
    planned_move_target_radius: float = 0.02
    planned_move_target_color_rgba: tuple[float, float, float, float] = (0.2, 0.85, 0.35, 0.85)

    planned_move_ghost_enable: bool = True
    planned_move_ghost_color_rgba: tuple[float, float, float, float] = (0.35, 0.55, 0.95, 0.35)
    # The ghost is a pure visual, unlike the real arm's own joint-rate limiter
    # (a.params.roll_rate/bend_rate), which is tuned to real hardware limits --
    # playing a preview back at that same real-world pace reads as sluggish
    # for what's meant to be a quick "does this path look right" check, not an
    # accurate real-time simulation. Scales all of the ghost's per-joint rate
    # bounds (see ghost_playback.build_ghost_stream) uniformly.
    planned_move_ghost_speed_scale: float = 16.0

    # A static wall-with-a-hole obstacle, purely for RRT-avoidance demos --
    # disabled by default so it never appears in an ordinary run. Enabling
    # it here only affects what's *visually spawned*; the Controller's own
    # collision_model.json needs the matching obstacle_boxes separately
    # (see elesim_model_builder.collision_model.build_wall_with_hole_boxes),
    # since the simulator must not import elesim_controller.
    wall_obstacle_enable: bool = False
    wall_obstacle_center_xyz: tuple[float, float, float] = (0.75, 0.0, 0.3)
    wall_obstacle_width_m: float = 0.6
    wall_obstacle_height_m: float = 0.6
    wall_obstacle_thickness_m: float = 0.03
    wall_obstacle_hole_width_m: float = 0.25
    wall_obstacle_hole_height_m: float = 0.25
    wall_obstacle_hole_offset_yz: tuple[float, float] = (0.0, 0.0)
    wall_obstacle_color_rgba: tuple[float, float, float, float] = (0.55, 0.35, 0.2, 1.0)

    # A solid vertical cylindrical obstacle (a post/pillar), purely for RRT-
    # avoidance demos -- disabled by default, same rationale as
    # wall_obstacle_enable. The Controller's own collision_model.json needs
    # the matching obstacle_capsules entry separately (see
    # elesim_model_builder.collision_model.build_cylinder_obstacle_capsule)
    # for the RRT planner to actually avoid it.
    cyl_obstacle_enable: bool = False
    cyl_obstacle_center_xyz: tuple[float, float, float] = (0.55, 0.0, 0.5)
    cyl_obstacle_radius_m: float = 0.1
    cyl_obstacle_height_m: float = 1.0
    cyl_obstacle_color_rgba: tuple[float, float, float, float] = (0.3, 0.3, 0.35, 1.0)


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
