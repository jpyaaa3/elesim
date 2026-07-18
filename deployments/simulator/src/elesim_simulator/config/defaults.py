"""Format-neutral application config defaults and derived values."""

from __future__ import annotations

import elesim_protocol.messages as proto

from elesim_simulator.config.schema import (
    AppConfigBundle,
    ExperimentConfig,
    GazeStabilizerConfig,
    Go2HardwareConfig,
    Go2LocomotionConfig,
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


def build_mapping_config(
    joint_limit: JointLimit,
    hardware_config: HardwareConfig,
) -> proto.SimMappingConfig:
    return proto.SimMappingConfig(
        linear_u_max=float(hardware_config.linear_u_max_deg),
        linear_u_limit=float(hardware_config.linear_u_limit_deg),
        linear_q_min_m=-0.230,
        linear_q_max_m=0.0,
        roll_q_min_rad=joint_limit.roll_min_rad(),
        roll_q_max_rad=joint_limit.roll_max_rad(),
        seg1_q_min_rad=-joint_limit.bend_lim_rad(),
        seg1_q_max_rad=+joint_limit.bend_lim_rad(),
        seg2_q_min_rad=-joint_limit.bend_lim_rad(),
        seg2_q_max_rad=+joint_limit.bend_lim_rad(),
        command_direction=hardware_config.command_direction,
    )


def default_app_config_bundle() -> AppConfigBundle:
    hardware = HardwareConfig()
    limits = JointLimit(roll_min_deg=-90.0, roll_max_deg=90.0, bend_deg=36.0)
    return AppConfigBundle(
        sim_param=SimParam(),
        sim_config=SimConfig(),
        hardware_config=hardware,
        joint_limit=limits,
        spawn_config=SpawnConfig(),
        urdf_export_config=UrdfExportConfig(),
        ik_config=IkConfig(),
        perception_config=PerceptionConfig(),
        pick_config=PickConfig(),
        go2_locomotion_config=Go2LocomotionConfig(),
        go2_hardware_config=Go2HardwareConfig(),
        gaze_stabilizer_config=GazeStabilizerConfig(),
        experiment_config=ExperimentConfig(),
        mapping_config=build_mapping_config(limits, hardware),
    )
