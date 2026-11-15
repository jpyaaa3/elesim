"""Bounded description of actual runtime dynamics; no object pose or authority."""
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
from pathlib import Path
from elesim_sim.robot.arm.actuation import PositionGains
from .physics_options import urdf_options,rigid_options

SUPPORTED_SLEW=(.05,1.,.3,.3)


def effective_command_slew(params):
    actual=(params.bend_rate,params.roll_rate,params.bend_rate,params.bend_rate)
    return [min(limit,float(rate)) for limit,rate in zip(SUPPORTED_SLEW,actual)]


def make_profile(config,params,locomotion,urdf,*,fixed_base=False):
    model=Path(urdf)
    data=dict(schema_version=1,arm_control_mode=config.arm_control_mode,
        dt_s=params.dt,substeps=params.substeps,body_mode='fixed' if fixed_base else 'free',
        gains=asdict(PositionGains(config.arm_kp,config.arm_kv,config.arm_linear_kp,
            config.arm_linear_kv,config.arm_force_limit)),
        friction=dict(arm=config.arm_friction,floor=config.floor_friction,feet=locomotion.mpc_physical_friction),
        urdf_options=urdf_options(),rigid_options=dict(constraint_solver='Newton',**rigid_options()),
        command_slew_per_s=effective_command_slew(params),
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        genesis_version=importlib.metadata.version('genesis-world'))
    data['sha256']=hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return data
