"""Keep motor angle, generated models and role defaults on one rack scale."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from elesim_protocol.messages import (
    ControlU, SimMappingConfig, control_u_to_sim_q, sim_q_to_control_u,
)

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("degrees, millimetres", [(0.0, 0.0), (15.0, 10.0), (150.0, 100.0), (250.0, 166.66666666666666)])
def test_design_rack_travel(degrees, millimetres):
    config = SimMappingConfig()
    q = control_u_to_sim_q(ControlU(degrees, 180.0, 180.0, 180.0), config)
    assert q.linear_m == pytest.approx(-millimetres / 1000.0)
    assert sim_q_to_control_u(q, config).u_linear == pytest.approx(degrees)


def test_motor_cap_does_not_expand_to_full_rack_length():
    q = control_u_to_sim_q(ControlU(360.0, 180.0, 180.0, 180.0), SimMappingConfig())
    assert q.linear_m == pytest.approx(-0.16666666666666666)
    assert abs(q.linear_m) < 0.29531


def test_shipped_models_and_robot_agree_on_linear_limits():
    expected = SimMappingConfig().linear_q_min_m
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
    assert robot["mapping"]["linear_q_min_m"] == pytest.approx(expected)
