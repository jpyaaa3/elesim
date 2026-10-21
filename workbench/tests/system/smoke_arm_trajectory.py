"""Three-process real-DDS trajectory execution probe; no hardware or CUDA.

The planner is a deterministic fixture. Real cuRobo is independently exercised
by smoke_curobo.py. This probe checks operator dispatch, target-owned leases,
timed canonical q delivery, measured completion and replay rejection.
"""
from __future__ import annotations

import json
import multiprocessing as mp
import queue
import time
import traceback
import uuid
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from elesim_protocol import (DdsRuntimeSettings, EndpointDescriptor, PeerClient,
    SimMappingConfig, decode_value)

ROOT = Path(__file__).resolve().parents[3]
START = [-.1, 0., 0., 0.]
GOAL = [-.095, 0., 0., 0.]


def settings(system):
    return DdsRuntimeSettings(system_id=system, domain_id=179,
        rmw_implementation="rmw_cyclonedds_cpp", security_profile="trusted-network")


def context():
    from elesim_pilot.robot.arm.joint_defs import JointLimit
    raw = json.loads((ROOT/"payload/data/models/arm/default.json").read_text())["context"]
    raw["limit"] = JointLimit(**raw["limit"]["value"])
    raw["base_world_T"] = np.eye(4)
    return raw


class FixturePlanner:
    def __init__(self, model_dir, *, world):
        from elesim_pilot.robot.arm.iklib.kinematics import _ReachModel
        c = context()
        self.model = _ReachModel(context=c, limit=c["limit"])
        self.bounds = np.asarray(json.loads((Path(model_dir)/"metadata.json").read_text())["q_bounds"])

    def forward(self, q):
        return {"position": self.model.grasp_position(q).tolist()}

    def plan_to_target(self, start, object_center, **kwargs):
        approach = self.model.grasp_position(GOAL)
        assert np.linalg.norm(approach-object_center) < 1e-6
        return GOAL, NS(positions=np.linspace(start, GOAL, 11), dt_s=.05), approach

    def validate_path(self, positions, **kwargs):
        assert len(positions) >= 2


class FixtureService:
    gaze_config = {}
    sim_reset_in_progress = False
    imu_models = []

    def __init__(self, link, connection):
        self.link, self.connection = link, connection

    def _ik_context_for_host(self, host): return context()
    def _visual_busy(self): return False
    def _pick_busy(self): return False
    def _gaze_busy(self): return False
    def pick_e2e_running(self): return False
    def wrap_grasp_running(self): return False
    def wrap_grasp_result(self): return ""
    def refresh_host_state(self): return self.link.get_state()
    def has_client(self): return True
    def current_control_u(self): return self.link.get_state().u
    def control_mapping(self): return self.link.cfg
    def _pick_config_effective(self): return {}
    def imu_model_status(self): return {}
    @property
    def available_endpoints(self): return self.connection.endpoints
    @property
    def active_endpoint(self): return self.connection.active_target
    @property
    def arm_trajectory(self): return self.runner.status()
    def guard_operator_intent(self, operation, name): self.runner.guard_operator(operation, name)
    def plan_arm_to_target(self): return self.runner.plan_arm_to_target()
    def execute_arm_trajectory(self, plan_id): return self.runner.execute_arm_trajectory(plan_id)
    def cancel_arm_trajectory(self): return self.runner.cancel_arm_trajectory()


def robot(system, stop, results):
    peer = None
    try:
        peer = PeerClient(EndpointDescriptor("trajectory-robot", "robot", ("motion.arm",)),
                          settings=settings(system))
        owner, lease, q, count = "", "", START, 0
        while not stop.is_set():
            peer.heartbeat()
            for m in peer.receive(timeout_ms=10):
                if m.message_type == "lease_granted":
                    owner, lease = m.payload["pilot_id"], m.lease_id
                elif m.message_type == "motion_command":
                    assert m.lease_id == lease and m.source_id == owner
                    assert m.payload["command"] == "target"
                    assert m.payload["source"] == "curobo"
                    assert not {"claw_closed", "go2_vel"} & set(m.payload)
                    q, count = m.payload["q"], count+1
                elif m.message_type == "lease_revoked":
                    owner, lease = "", ""
            if owner:
                peer.send("telemetry", target_id=owner, lease_id=lease, payload={
                    "q": q, "q_source": "measured", "measurement_monotonic_s": time.monotonic(),
                    "torque_enabled": True, "claw_closed": False, "safety_fault": "",
                    "imu_model": {"id": "identity", "version": 1}})
        results.put(("robot", {"count": count, "q": q}))
    except BaseException:
        results.put(("error", traceback.format_exc()))
    finally:
        if peer is not None: peer.close()


def pilot(system, stop, results, world):
    connection = None
    try:
        from elesim_pilot.arm_trajectory import ArmTrajectory
        from elesim_pilot.connection import PilotConnection
        from elesim_pilot.operator import OperatorDispatcher
        from elesim_pilot.pick.client import ControlClient
        from elesim_pilot.pick.state import PanelState
        mapping = SimMappingConfig()
        link = ControlClient(cfg=mapping)
        connection = PilotConnection(pilot_id="trajectory-pilot", initial_target="trajectory-robot",
            mapping=mapping, state_sink=link, dds_settings=settings(system))
        link.attach_sender(connection.submit)
        state = PanelState()
        service = FixtureService(link, connection)
        state.set_target(*FixturePlanner(ROOT/"payload/data/models/planning/zed-mini", world={}).forward(GOAL)["position"])
        runner = ArmTrajectory(connection, service, state,
            model_dir=ROOT/"payload/data/models/planning/zed-mini", world_path=world,
            planner_factory=FixturePlanner)
        service.runner = runner
        connection.control_tick, connection.motion_guard = runner.tick, runner.allow_motion
        connection.on_closing = runner.cancel_arm_trajectory
        connection.operator_handler = OperatorDispatcher(state, service).handle
        connection.start()
        stop.wait(35)
    except BaseException:
        results.put(("error", traceback.format_exc()))
    finally:
        if connection is not None: connection.close()


def ui(system, results):
    peer = None
    try:
        peer = PeerClient(EndpointDescriptor("trajectory-ui", "ui", ()), settings=settings(system))
        def call(operation, name="", args=None):
            request = uuid.uuid4().hex
            deadline, sent = time.monotonic()+8, False
            while time.monotonic() < deadline:
                peer.heartbeat()
                if not sent:
                    discovery = peer.send("discover", payload={})
                    if any(d.get("endpoint_id") == "trajectory-pilot" for d in discovery.payload.get("endpoints", [])):
                        peer.send("operator_intent", target_id="trajectory-pilot", payload={
                            "request_id": request, "operation": operation, "name": name,
                            "args": args or [], "kwargs": {}})
                        sent = True
                for m in peer.receive(timeout_ms=10):
                    if m.message_type == "operator_result" and m.payload.get("request_id") == request:
                        return m.payload
            raise TimeoutError(name or operation)
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            view = call("view_snapshot")
            status = decode_value(view["result"])["service"]["arm_trajectory"]
            if status.get("current_xyz") is not None: break
        else: raise TimeoutError("measured start")
        assert call("service_call", "plan_arm_to_target")["ok"]
        while time.monotonic() < deadline:
            view = decode_value(call("view_snapshot")["result"])
            status = view["service"]["arm_trajectory"]
            if status["phase"] == "ready": break
            assert status["phase"] != "failed", status
        else: raise TimeoutError("plan")
        plan_id = status["plan_id"]
        assert call("service_call", "execute_arm_trajectory", [plan_id])["ok"]
        blocked = call("service_call", "home_controls")
        assert not blocked["ok"] and "workflow active" in blocked["error"]
        while time.monotonic() < deadline:
            view = decode_value(call("view_snapshot")["result"])
            status = view["service"]["arm_trajectory"]
            if status["phase"] == "completed": break
            assert status["phase"] != "failed", status
        else: raise TimeoutError("completion")
        assert not call("service_call", "execute_arm_trajectory", [plan_id])["ok"]
        results.put(("ui", status))
    except BaseException:
        results.put(("error", traceback.format_exc()))
    finally:
        if peer is not None: peer.close()


def main():
    import tempfile
    ctx = mp.get_context("spawn")
    stop, results = ctx.Event(), ctx.Queue()
    system = "trajectory_"+uuid.uuid4().hex[:10]
    with tempfile.TemporaryDirectory() as tmp:
        world = Path(tmp)/"world.yml"
        world.write_text("{}")
        workers = [ctx.Process(target=robot, args=(system, stop, results)),
                   ctx.Process(target=pilot, args=(system, stop, results, str(world))),
                   ctx.Process(target=ui, args=(system, results))]
        for p in workers: p.start()
        observed = {}
        try:
            key, value = results.get(timeout=30)
            assert key == "ui", value
            observed[key] = value
        finally:
            stop.set()
            for p in workers:
                p.join(5)
                if p.is_alive(): p.terminate(); p.join(2)
        while True:
            try: key, value = results.get_nowait()
            except queue.Empty: break
            assert key != "error", value
            observed[key] = value
        # Volatile best-effort motion may lose the first writer/reader sample.
        assert observed.get("robot", {}).get("count", 0) >= 8, observed
        assert np.max(np.abs(np.asarray(observed["robot"]["q"])-GOAL)) <= .001
        assert all(p.exitcode == 0 for p in workers)
        print(json.dumps(observed, indent=2))


def cuda_plan_probe():
    """Real CUDA worker smoke with fixture feedback; never creates a DDS peer."""
    import tempfile
    from elesim_pilot.arm_trajectory import ArmTrajectory
    from elesim_pilot.pick.state import PanelState
    from elesim_protocol import SimQ
    mapping = SimMappingConfig()
    host = NS(connected=True, q=SimQ(*START), torque_enabled=False, safety_fault="",
              reply_ok=True, claw_closed=False, go2_vel=(0., 0., 0.))
    def forbidden(*args, **kwargs):
        raise AssertionError("planning-only probe attempted motion")
    connection = NS(active_target="fixture-robot", lease_id="fixture-lease", mapping=mapping,
        endpoints=[{"endpoint_id": "fixture-robot", "role": "robot", "instance_id": "fixture-boot"}],
        telemetry_received_at=time.monotonic(),
        trajectory_telemetry={"imu_model": {"id": "identity", "version": 1}},
        state_sink=NS(get_state=lambda: host), clear_pending_motion=lambda: None,
        send_trajectory_q=forbidden, release_trajectory_route=forbidden)
    state = PanelState()
    service = FixtureService(NS(cfg=mapping, get_state=lambda: host), connection)
    target = FixturePlanner(ROOT/"payload/data/models/planning/zed-mini", world={}).forward(GOAL)["position"]
    state.set_target(*target)
    with tempfile.TemporaryDirectory() as tmp:
        world = Path(tmp)/"world.yml"
        world.write_text("{}")
        runner = ArmTrajectory(connection, service, state,
            model_dir=ROOT/"payload/data/models/planning/zed-mini", world_path=world)
        service.runner = runner
        started = time.monotonic()
        runner.plan_arm_to_target()
        deadline = started+240
        while runner.status()["phase"] == "planning" and time.monotonic() < deadline:
            runner.tick()
            time.sleep(.05)
        status = runner.status()
        assert status["phase"] == "ready", status
        model = FixturePlanner(ROOT/"payload/data/models/planning/zed-mini", world={}).model
        error = float(np.linalg.norm(model.grasp_position(status["goal_q"])-target))
        assert error <= .001
        status.update(elapsed_s=time.monotonic()-started, goal_error_m=error,
                      scope="real CUDA planning worker; fixture feedback; no DDS or motion")
        print(json.dumps(status, indent=2))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cuda-plan-only", action="store_true")
    args = parser.parse_args()
    cuda_plan_probe() if args.cuda_plan_only else main()
