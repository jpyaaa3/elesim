"""Cross-role/model operating-range integration; protocol gate stays role-free."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from elesim_protocol.messages import (
    ControlU, SimMappingConfig, control_u_to_sim_q, linear_effective_q_bounds,
)

ROOT = Path(__file__).resolve().parents[3]


def test_shipped_models_and_robot_agree_on_linear_limits():
    expected = linear_effective_q_bounds(SimMappingConfig())[0]
    for profile in ("zed-mini", "d435"):
        for name in ("arm.urdf", "robot.urdf"):
            tree = ET.parse(ROOT / "payload/data/models/assemblies" / profile / name)
            joint = tree.find(".//joint[@name='j_plate_housing']")
            assert joint.find("axis").get("xyz") == "1 0 0"
            assert float(joint.find("limit").get("lower")) == pytest.approx(expected)
            assert float(joint.find("limit").get("upper")) == 0.0
    model = json.loads((ROOT / "payload/data/models/arm/default.json").read_text())
    assert model["context"]["linear_min_m"] == pytest.approx(expected)
    robot = yaml.safe_load((ROOT / "payload/config/robot/default.yaml").read_text())
    assert expected == pytest.approx(-0.230)
    assert robot["mapping"]["linear_q_min_m"] == pytest.approx(-0.230)
    assert robot["mapping"]["linear_u_max"] == 259.0
    assert robot["mapping"]["linear_u_limit"] == 259.0


def test_role_defaults_and_rl_share_operating_range():
    from elesim_pilot.config import load_app_config as pilot_config
    from elesim_sim.config import load_app_config as sim_config
    from elesim_robot.config import load_config as robot_config
    from elesim_sim.rl.app_config import load_mechanism

    mappings = [
        SimMappingConfig(),
        pilot_config(ROOT / "payload/config/pilot/config.yaml").mapping_config,
        sim_config(ROOT / "payload/config/sim/config.yaml").mapping_config,
        robot_config(ROOT / "payload/config/robot/default.yaml").mapping,
    ]
    for mapping in mappings:
        assert mapping.linear_u_max == 259.0
        assert mapping.linear_u_limit == 259.0
        assert linear_effective_q_bounds(mapping) == pytest.approx((-0.230, 0.0))
        assert control_u_to_sim_q(ControlU(259, 180, 180, 180), mapping).linear_m == pytest.approx(-0.230)
    assert load_mechanism().linear_m == pytest.approx((-0.230, 0.0))


def test_rl_respects_a_custom_operating_cap(tmp_path):
    from elesim_sim.rl.app_config import load_mechanism
    config = yaml.safe_load((ROOT / "payload/config/sim/config.yaml").read_text())
    config["robot"]["arm"]["mapping"]["linear_u_limit_deg"] = 150.0
    path = tmp_path / "sim.yaml"
    path.write_text(yaml.safe_dump(config))
    assert load_mechanism(path).linear_m == pytest.approx((-0.230 * 150/259, 0.0))
