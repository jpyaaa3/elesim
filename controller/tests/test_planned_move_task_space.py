from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from elesim_controller.pick import ControlService, PanelState
from elesim_controller.pick.state import HostState
from elesim_controller.robot.arm.iklib import kinematics as ik_kin
from elesim_controller.robot.arm.iklib.solver import load_solver_context
from elesim_controller.robot.arm.mounts.go2_mount import Go2ArmMount
from elesim_protocol import SimQ

CONFIG_PATH = Path(__file__).parents[1] / "config" / "config.yaml"
# config.pc.yaml overrides go2.spawn.mount_offset_m to (0.35, 0.0, 0.08) --
# substantially different from config.yaml's own (0.1, 0.0, 0.07), and is
# what the live deployment (`--config controller/config/config.pc.yaml`)
# actually runs with. The GO2-mounted test below must match it: verifying
# against the wrong mount offset silently checks a different, easier
# problem (confirmed live -- this exact mismatch is why an earlier version
# of this test passed while the real deployment did not).
PC_CONFIG_PATH = Path(__file__).parents[1] / "config" / "config.pc.yaml"


def _wait_for_terminal_phase(service: ControlService, *, timeout_s: float = 20.0) -> dict:
    deadline = time.monotonic() + timeout_s
    status = service.planned_move_status()
    while status.get("phase") in ("idle", "planning") and time.monotonic() < deadline:
        time.sleep(0.02)
        status = service.planned_move_status()
    return status


def _host_state(
    *, go2_base_pos: tuple[float, float, float], go2_base_rpy: tuple[float, float, float]
) -> HostState:
    return HostState(
        connected=True,
        tx_seq=0,
        rx_age_s=0.0,
        device="",
        ports=(),
        torque_enabled=False,
        claw_current=0,
        motor_currents_ma={},
        safety_fault="",
        actual_tip_xyz=None,
        actual_tip_dir=None,
        perceived_object_label="",
        perceived_object_confidence=0.0,
        perceived_object_camera_xyz=None,
        perceived_center_uv=None,
        perceived_scale=None,
        perceived_timestamp_s=0.0,
        go2_vel=(0.0, 0.0, 0.0),
        reply_ok=True,
        reply_reason="",
        q=SimQ(0.0, 0.0, 0.0, 0.0),
        u=None,
        go2_base_pos=go2_base_pos,
        go2_base_rpy=go2_base_rpy,
        go2_leg_q=None,
    )


class _FakeClient:
    """Stands in for ``ControlClient`` -- just enough for
    ``start_planned_move_generate_task_space`` to fold a live GO2 base pose
    into the IK context, the same way the real deployment does."""

    def __init__(self, host_state: HostState) -> None:
        self._host_state = host_state

    def refresh_state(self) -> HostState:
        return self._host_state


def _go2_mounted_service(*, go2_base_pos, go2_base_rpy, current_q) -> ControlService:
    """Build a ``ControlService`` wired up the same way
    ``elesim_controller.runtime.build_control_runtime`` does for a real
    GO2-mounted deployment -- a real ``Go2ArmMount`` (not ``None``) and a
    client that reports a live GO2 base pose, so the arm's IK context folds
    in the *real* mount offset (``go2.spawn.mount_offset_m``, ~35cm forward
    of GO2's own base) instead of the arm's standalone spawn assumption.

    This distinction matters: a target that looks reachable/collision-free
    against the standalone spawn frame can be a completely different
    (and much harder, or outright infeasible) problem once GO2's own body
    -- especially its head, mounted only ~1cm from the arm's own base --
    is folded in and checked for real (confirmed live: this exact gap
    previously let mount-offset-blind verification pass a target that
    the real deployment could never reach without hitting GO2's head).
    """
    bundle, ik_context = load_solver_context(str(PC_CONFIG_PATH))
    mount = Go2ArmMount.from_context(
        use_go2=bool(bundle.sim_config.use_go2),
        spawn_xyz=bundle.spawn_config.spawn_xyz,
        go2_spawn_height=float(bundle.spawn_config.go2_spawn_height),
        go2_spawn_euler_deg=bundle.spawn_config.go2_spawn_euler_deg,
        mount_offset_body_m=bundle.spawn_config.go2_mount_offset_m,
        ik_context=ik_context,
    )
    host_state = _host_state(go2_base_pos=go2_base_pos, go2_base_rpy=go2_base_rpy)
    service = ControlService(
        PanelState(),
        client=_FakeClient(host_state),
        ik_context=ik_context,
        config_path=str(PC_CONFIG_PATH),
        go2_arm_mount=mount,
    )
    service.state.set_q(*current_q)
    return service


def test_start_planned_move_generate_task_space_reaches_planned_phase_for_a_reachable_target() -> None:
    """Regression test: this crashed live with ``AttributeError: 'ControlService'
    object has no attribute '_collision_model'`` -- the collision model only
    lives on ``self._planned_move``, not on ``ControlService`` itself. Exercise
    the real entry point end to end (through the background worker thread,
    same as production) rather than just the pieces it calls, so a wiring
    mistake like that one actually fails a test next time.
    """
    _, ik_context = load_solver_context(str(CONFIG_PATH))
    target_xyz = tuple(float(v) for v in ik_kin._forward_grasp_world(ik_context, np.zeros(4)))

    service = ControlService(PanelState(), config_path=str(CONFIG_PATH))
    assert service._planned_move.collision_model is not None

    service.start_planned_move_generate_task_space(target_xyz=target_xyz)
    # Regression: must flip to "planning" synchronously, before the (possibly
    # long) IK-seed search that precedes the actual generate() call -- see
    # PlannedMoveExecutor.mark_planning()'s docstring. Otherwise the UI's
    # status line sits stale (often "idle") for that whole search, reading
    # as "Generate did nothing" even though a request was genuinely accepted.
    assert service.planned_move_status().get("phase") != "idle"

    status = _wait_for_terminal_phase(service)

    assert status.get("phase") == "planned", status
    assert int(status.get("waypoint_count", 0)) > 0


def test_start_planned_move_generate_task_space_clears_the_demo_wall_when_go2_mounted(
    monkeypatch,
) -> None:
    """Regression test for a live failure chain (in order of discovery):

    1. A colliding IK branch was accepted because only one seed was tried.
    2. The wall's hole margin turned out to be within normal proxy-capsule
       slop -- fixed with a small ``environment_clearance_m`` tolerance.
    3. The *real* live failure turned out to be neither of the above: every
       earlier verification (including this test, originally) built the IK
       context WITHOUT a ``Go2ArmMount``, so it silently checked the arm
       against a standalone spawn frame instead of its real GO2-mounted one.
       Once GO2's own head (parked ~1cm from the arm's base) is folded in
       for real, a wall placed too close to GO2 (world x=0.5) has *zero*
       collision-free solutions even across hundreds of random IK seeds --
       confirmed via a direct sweep before landing on x=0.75, which has a
       majority success rate under the same sweep.
    """
    demo_wall_path = Path(__file__).parents[1] / "config" / "collision_model.demo_wall.json"
    monkeypatch.setenv("ELESIM_COLLISION_MODEL", str(demo_wall_path))

    service = _go2_mounted_service(
        go2_base_pos=(-0.003975092899054289, 1.3227255521996995e-06, 0.2993321120738983),
        go2_base_rpy=(1.166906006286593e-06, 0.002761290661858151, -7.83087879756522e-06),
        current_q=(-0.23, 0.0, -0.3316, 0.4712),
    )
    assert service._planned_move.collision_model is not None
    assert len(service._planned_move.collision_model.obstacle_boxes) == 4

    service.start_planned_move_generate_task_space(target_xyz=(0.75, -0.012, 0.3))
    # Generous timeout: RRT now runs a much finer step_size/collision_check_resolution
    # (needed to not step over a 3cm-thick wall -- see rrt.py's RrtConfig) plus up to
    # 60 IK seed attempts, both of which make a single generate() call noticeably
    # slower than before; this is unseeded (real) randomness, so worst-case duration
    # varies run to run.
    status = _wait_for_terminal_phase(service, timeout_s=90.0)

    assert status.get("phase") == "planned", status


def test_start_planned_move_generate_task_space_fails_cleanly_without_a_collision_model() -> None:
    _, ik_context = load_solver_context(str(CONFIG_PATH))
    service = ControlService(PanelState(), ik_context=ik_context, config_path=None)
    assert service._planned_move.collision_model is None

    service.start_planned_move_generate_task_space(target_xyz=(0.3, 0.0, 0.2))
    status = _wait_for_terminal_phase(service)

    assert status.get("phase") == "failed"
    assert status.get("message") == "no_collision_model"
