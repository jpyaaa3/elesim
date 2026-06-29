from __future__ import annotations

# Do not import controller / convex_mpc here: host and Jetson tools import
# submodules (e.g. genesis_pin_bridge) without Genesis MPC installed.
# Use explicit imports: engine.robot.go2.mpc.controller, engine.robot.go2.mpc.config, ...

__all__ = [
    "ArmPayloadCompensator",
    "ControlRateInfo",
    "ConvexMpcGenesisController",
    "GenesisPinBridge",
    "Go2MpcConfig",
    "payload_pitch_trim_rad",
]


def __getattr__(name: str):
    if name == "Go2MpcConfig":
        from engine.robot.go2.mpc.config import Go2MpcConfig

        return Go2MpcConfig
    if name == "ControlRateInfo":
        from engine.robot.go2.mpc.control_rate import ControlRateInfo

        return ControlRateInfo
    if name == "ConvexMpcGenesisController":
        from engine.robot.go2.mpc.controller import ConvexMpcGenesisController

        return ConvexMpcGenesisController
    if name == "GenesisPinBridge":
        from engine.robot.go2.mpc.genesis_pin_bridge import GenesisPinBridge

        return GenesisPinBridge
    if name == "ArmPayloadCompensator":
        from engine.robot.go2.mpc.payload_model import ArmPayloadCompensator

        return ArmPayloadCompensator
    if name == "payload_pitch_trim_rad":
        from engine.robot.go2.mpc.payload_model import payload_pitch_trim_rad

        return payload_pitch_trim_rad
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
