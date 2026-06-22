from __future__ import annotations

from engine.go2_mpc.control_rate import ControlRateInfo


def test_effective_rate_capped_by_sim_hz() -> None:
    info = ControlRateInfo.from_sim_dt(0.01, 200.0)
    assert info.sim_hz == 100.0
    assert info.ctrl_hz_config == 200.0
    assert info.ctrl_decim == 1
    assert info.ctrl_hz_effective == 100.0


def test_decimation_when_ctrl_below_sim() -> None:
    info = ControlRateInfo.from_sim_dt(0.01, 50.0)
    assert info.ctrl_decim == 2
    assert info.ctrl_hz_effective == 50.0
