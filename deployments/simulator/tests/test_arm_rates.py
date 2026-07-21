from __future__ import annotations

import pytest

from elesim_protocol import SimMappingConfig
from elesim_simulator.robot.arm.rates import estimate_ideal_sim_rates


def test_ideal_rates_match_existing_hardware_profile_convention() -> None:
    roll, bend = estimate_ideal_sim_rates(SimMappingConfig())
    assert roll == pytest.approx(2.8787, rel=1e-3)
    assert bend == pytest.approx(0.2879, rel=1e-3)
