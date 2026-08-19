"""Defaults and derived sim configuration."""

from __future__ import annotations

import elesim_protocol.messages as proto

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


def build_mapping_config(
    joint_limit: JointLimit,
    arm_mapping: ArmMappingConfig,
) -> proto.SimMappingConfig:
    return proto.SimMappingConfig(
        linear_u_max=float(arm_mapping.linear_u_max_deg),
        linear_u_limit=float(arm_mapping.linear_u_limit_deg),
        linear_q_min_m=-0.230,
        linear_q_max_m=0.0,
        roll_q_min_rad=joint_limit.roll_min_rad(),
        roll_q_max_rad=joint_limit.roll_max_rad(),
        seg1_q_min_rad=-joint_limit.bend_lim_rad(),
        seg1_q_max_rad=joint_limit.bend_lim_rad(),
        seg2_q_min_rad=-joint_limit.bend_lim_rad(),
        seg2_q_max_rad=joint_limit.bend_lim_rad(),
        command_direction=arm_mapping.command_direction,
    )


def default_app_config_bundle() -> AppConfigBundle:
    arm_mapping = ArmMappingConfig()
    limits = JointLimit(roll_min_deg=-90.0, roll_max_deg=90.0, bend_deg=36.0)
    return AppConfigBundle(
        sim_param=SimParam(),
        sim_config=SimConfig(),
        arm_mapping_config=arm_mapping,
        joint_limit=limits,
        spawn_config=SpawnConfig(),
        urdf_export_config=UrdfExportConfig(),
        go2_locomotion_config=Go2LocomotionConfig(),
        mapping_config=build_mapping_config(limits, arm_mapping),
    )
