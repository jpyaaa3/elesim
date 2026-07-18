from __future__ import annotations

from elesim_protocol import Envelope, SimMappingConfig, make_envelope
from elesim_robot.config import HardwareConfig
from elesim_robot.main import RobotRuntime


class FakeArm:
    def __init__(self) -> None:
        self.target = None
        self.stopped = False

    def command_4dof_deg(self, *target: float) -> None:
        self.target = target

    def stop_arm_velocity(self) -> None:
        self.stopped = True


def command(payload: dict[str, object], *, seq: int = 2, lease: str = "lease-a") -> Envelope:
    return make_envelope(
        "motion_command",
        "controller-a",
        target_id="robot-a",
        payload=payload,
        seq=seq,
        lease_id=lease,
    )


def test_runtime_accepts_only_canonical_q_target() -> None:
    runtime = RobotRuntime(mapping=SimMappingConfig(), hardware_config=HardwareConfig())
    runtime.hw = FakeArm()
    runtime.grant_lease("controller-a", "lease-a")

    ok, reason = runtime.apply(command({"command": "target", "q": [-0.1, 0.0, 0.1, -0.1]}))
    assert (ok, reason) == (True, "target")
    assert runtime.hw.target is not None

    ok, reason = runtime.apply(
        command(
            {"command": "target", "u": {"linear": 10, "roll": 20, "s1": 30, "s2": 40}},
            seq=3,
        )
    )
    assert (ok, reason) == (False, "legacy_u_not_supported")


def test_runtime_rejects_wrong_lease_without_touching_hardware() -> None:
    runtime = RobotRuntime(mapping=SimMappingConfig(), hardware_config=HardwareConfig())
    runtime.hw = FakeArm()
    runtime.grant_lease("controller-a", "lease-a")
    ok, reason = runtime.apply(command({"command": "target", "q": [0, 0, 0, 0]}, lease="wrong"))
    assert (ok, reason) == (False, "lease_mismatch")
    assert runtime.hw.target is None
