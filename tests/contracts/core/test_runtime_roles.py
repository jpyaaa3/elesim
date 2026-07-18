from __future__ import annotations

import ast
from pathlib import Path

import pytest

from engine.config import load_runtime_role_config
from engine.core.protocol import SimMappingConfig, make_envelope
from engine.vision.sim_camera.remote_control import consume_pose, enqueue
from apps.sim_agent.bridge import SimProtocolBridge


ROOT = next(path for path in Path(__file__).resolve().parents if (path / "AGENTS.md").exists())


def test_runtime_role_configs_are_valid() -> None:
    roles = {
        load_runtime_role_config(path).role
        for path in sorted((ROOT / "configs" / "runtime").glob("*.yaml"))
    }
    assert roles == {"server", "controller", "robot", "sim", "ui"}
    robot = load_runtime_role_config(ROOT / "configs" / "runtime" / "robot.yaml")
    assert robot.camera_enabled is True
    assert robot.streams["rgbd_advertise"]
    sim = load_runtime_role_config(ROOT / "configs" / "runtime" / "sim.yaml")
    assert sim.streams["rgbd_advertise"]
    assert sim.streams["rendered_view"] == "webrtc"


def test_robot_agent_import_boundary() -> None:
    banned = ("apps.host", "apps.sim", "builders", "ui", "engine.pick", "genesis")
    imported: list[str] = []
    for path in (ROOT / "apps" / "robot_agent").glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.append(node.module)
    assert not [name for name in imported if name.startswith(banned)]


def test_remote_camera_orbit_zoom_and_reset() -> None:
    start_pos = (0.0, -2.0, 1.0)
    lookat = (0.0, 0.0, 0.0)
    enqueue("orbit", (0.1, -0.05))
    enqueue("zoom", (-0.1,))
    moved_pos, moved_lookat = consume_pose(
        start_pos,
        lookat,
        reset_pos=start_pos,
        reset_lookat=lookat,
    )
    assert moved_pos != pytest.approx(start_pos)
    assert moved_lookat == pytest.approx(lookat)
    enqueue("reset")
    reset_pos, reset_lookat = consume_pose(
        moved_pos,
        moved_lookat,
        reset_pos=start_pos,
        reset_lookat=lookat,
    )
    assert reset_pos == pytest.approx(start_pos)
    assert reset_lookat == pytest.approx(lookat)


def test_sim_bridge_translates_legacy_target_metadata() -> None:
    bridge = SimProtocolBridge(
        server_endpoint="inproc://unused",
        endpoint_id="sim-a",
        legacy_state_bind="inproc://unused-state",
        legacy_feedback_bind="inproc://unused-feedback",
        mapping=SimMappingConfig(),
    )
    ok, reason = bridge._apply_command(
        {
            "command": "target",
            "target": [1.0, 2.0, 3.0],
            "target_dir": [0.0, 0.0, 1.0],
            "sim_target": [0.8, 0.0, 0.2],
            "go2_sport_pose": "stand_up",
            "go2_obstacles_avoid_enable": True,
        }
    )
    assert (ok, reason) == (True, "target")
    assert bridge.state_meta["ik_target_xyz"] == [1.0, 2.0, 3.0]
    assert bridge.state_meta["ik_target_dir"] == [0.0, 0.0, 1.0]
    assert bridge.state_meta["sim_target_xyz"] == [0.8, 0.0, 0.2]
    assert bridge.state_meta["go2_sport_pose_seq"] == 1
    assert bridge.state_meta["go2_obstacles_avoid_seq"] == 1


def test_sim_bridge_rejects_wrong_lease_and_stale_control() -> None:
    bridge = SimProtocolBridge(
        server_endpoint="inproc://unused",
        endpoint_id="sim-a",
        legacy_state_bind="inproc://unused-state",
        legacy_feedback_bind="inproc://unused-feedback",
        mapping=SimMappingConfig(),
    )
    bridge.controller_id = "controller-a"
    bridge.active_lease = "lease-a"
    accepted = make_envelope(
        "command",
        "controller-a",
        target_id="sim-a",
        payload={"command": "target"},
        seq=2,
        lease_id="lease-a",
    )
    assert bridge._validate_control(accepted) == (True, "accepted")
    assert bridge._validate_control(accepted) == (False, "stale_sequence")
    wrong = make_envelope(
        "command",
        "controller-b",
        target_id="sim-a",
        payload={"command": "target"},
        seq=3,
        lease_id="lease-b",
    )
    assert bridge._validate_control(wrong) == (False, "lease_mismatch")
