from __future__ import annotations

import numpy as np

from elesim_protocol import SimQ
from elesim_sim.telemetry import RuntimeTelemetry


def test_go2_controller_fault_is_published_and_empty_reset_clears_it():
    from types import SimpleNamespace
    emitted = []
    base = SimpleNamespace(get_pos=lambda:np.array([0.,0.,.3]),
        get_quat=lambda:np.array([1.,0.,0.,0.]),get_vel=lambda:np.zeros(3),get_ang=lambda:np.zeros(3))
    entity = SimpleNamespace(get_link=lambda _name:base)
    telemetry = RuntimeTelemetry(emitted.append)
    telemetry.send_go2_base(entity, go2_controller_fault='touchdown timeout')
    assert emitted[-1]['go2_controller_fault'] == 'touchdown timeout'
    telemetry.send_go2_base(entity, go2_controller_fault='')
    assert emitted[-1]['go2_controller_fault'] == ''


def test_runtime_telemetry_uses_canonical_q_and_normalized_direction() -> None:
    emitted: list[dict[str, object]] = []
    telemetry = RuntimeTelemetry(emitted.append)

    telemetry.send_actual_tip(
        np.array([0.7, 0.0, 0.2]),
        np.array([2.0, 0.0, 0.0]),
        arm_q=SimQ(-0.1, 0.2, 0.3, -0.3),
        sim_time_s=1.5,
        sim_step_count=75,
    )

    assert emitted[-1]["q"] == [-0.1, 0.2, 0.3, -0.3]
    assert emitted[-1]["actual_tip"] == [0.7, 0.0, 0.2]
    assert emitted[-1]["actual_tip_dir"] == [1.0, 0.0, 0.0]
    assert emitted[-1]["sim_step_count"] == 75
