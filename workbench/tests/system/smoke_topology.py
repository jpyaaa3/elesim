#!/usr/bin/env python3
"""Four-process Router-free DDS topology smoke test.

This test deliberately uses separate OS processes and the generated ROSIDL
package.  Missing ROS 2 or ``elesim_interfaces`` is a failed required gate,
not a successful skip; the generated Developer environment is the canonical
place to execute the real RMW path.
"""

from __future__ import annotations

import importlib.util
import multiprocessing as mp
import os
import queue
import sys
import time
import traceback
from collections.abc import Callable
from typing import Any

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_OPERATOR_CONTROL,
    DdsRuntimeSettings,
    EndpointDescriptor,
    OpenSimulationSessionRequest,
    PeerClient,
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationStatusPayload,
    WebRtcSignalPayload,
)


TIMEOUT_S = float(os.environ.get("ELESIM_TOPOLOGY_TIMEOUT_S", "20"))
EXPECTED = {
    "pilot:ack",
    "pilot:operator",
    "pilot:selected",
    "robot:lease",
    "robot:motion",
    "sim:command",
    "sim:session",
    "sim:webrtc",
    "ui:answer",
    "ui:opened",
    "ui:operator",
    "ui:result",
    "ui:status",
}


def _settings() -> DdsRuntimeSettings:
    return DdsRuntimeSettings(
        system_id=os.environ.get("ELESIM_TOPOLOGY_SYSTEM_ID", "elesim_smoke"),
        domain_id=int(os.environ.get("ELESIM_TOPOLOGY_DOMAIN_ID", "177")),
        rmw_implementation=os.environ.get(
            "ELESIM_TOPOLOGY_RMW",
            "rmw_cyclonedds_cpp",
        ),
        security_profile="trusted-network",
        heartbeat_timeout_s=4.0,
    )


def _report(results: Any, value: str) -> None:
    results.put(str(value))


def _wait_barrier(barrier: Any) -> None:
    barrier.wait(timeout=TIMEOUT_S)


def _run_child(role: str, results: Any, target: Callable[[], None]) -> None:
    try:
        target()
    except BaseException as exc:
        _report(results, f"error:{role}:{exc!r}\n{traceback.format_exc()}")


def _robot_process(barrier: Any, stop: Any, results: Any) -> None:
    def run() -> None:
        peer = PeerClient(
            EndpointDescriptor(
                "robot-smoke",
                "robot",
                (CAPABILITY_MOTION_ARM,),
            ),
            settings=_settings(),
        )
        _wait_barrier(barrier)
        try:
            while not stop.is_set():
                peer.heartbeat()
                for message in peer.receive(timeout_ms=20):
                    if message.message_type == "lease_granted":
                        _report(results, "robot:lease")
                    elif message.message_type == "motion_command":
                        _report(results, "robot:motion")
                        peer.send(
                            "ack",
                            target_id=message.source_id,
                            lease_id=message.lease_id,
                            payload={
                                "reply_to": message.message_id,
                                "ok": True,
                                "reason": "smoke",
                            },
                        )
                        peer.send(
                            "telemetry",
                            target_id=message.source_id,
                            lease_id=message.lease_id,
                            payload={
                                "q": [-0.1, 0.0, 0.0, 0.0],
                                "q_source": "measured",
                                "torque_enabled": False,
                            },
                        )
        finally:
            peer.close()

    _run_child("robot", results, run)


def _controller_process(barrier: Any, stop: Any, results: Any) -> None:
    def run() -> None:
        peer = PeerClient(
            EndpointDescriptor(
                "pilot-smoke",
                "pilot",
                (CAPABILITY_OPERATOR_CONTROL,),
            ),
            settings=_settings(),
        )
        _wait_barrier(barrier)
        selected = False
        lease_id = ""
        last_selection_at = 0.0
        last_motion_at = 0.0
        motion_attempts = 0
        try:
            while not stop.is_set():
                peer.heartbeat()
                now = time.monotonic()
                if not selected and now - last_selection_at >= 1.0:
                    discovery = peer.send(
                        "discover",
                        payload={"role": "robot"},
                    )
                    endpoints = list((discovery.payload or {}).get("endpoints", []))
                    if any(
                        endpoint.get("endpoint_id") == "robot-smoke"
                        for endpoint in endpoints
                    ):
                        peer.send(
                            "select_target",
                            payload={"target_id": "robot-smoke"},
                        )
                        last_selection_at = now
                for message in peer.receive(timeout_ms=20):
                    if message.message_type == "target_selected":
                        selected = True
                        lease_id = message.lease_id
                        _report(results, "pilot:selected")
                    elif message.message_type == "operator_intent":
                        request_id = str((message.payload or {}).get("request_id", ""))
                        peer.send(
                            "operator_result",
                            target_id=message.source_id,
                            payload={
                                "request_id": request_id,
                                "ok": True,
                                "result": {"pong": True},
                            },
                        )
                        _report(results, "pilot:operator")
                    elif message.message_type == "ack":
                        _report(results, "pilot:ack")
                # Motion deliberately uses volatile best-effort keep-last-1
                # delivery.  Exercise the bounded command stream used by the
                # runtime instead of assuming that one first sample can arrive
                # before a newly-created motion writer/reader pair matches.
                if (
                    selected
                    and motion_attempts < 20
                    and now - last_motion_at >= 0.1
                ):
                    peer.send(
                        "motion_command",
                        target_id="robot-smoke",
                        lease_id=lease_id,
                        payload={"command": "torque_off"},
                    )
                    motion_attempts += 1
                    last_motion_at = now
        finally:
            peer.close()

    _run_child("pilot", results, run)


def _sim_process(barrier: Any, stop: Any, results: Any) -> None:
    def run() -> None:
        peer = PeerClient(
            EndpointDescriptor("sim-smoke", "sim", ()),
            settings=_settings(),
        )
        _wait_barrier(barrier)
        active_session = ""
        ui_id = ""
        status_sent = False
        try:
            while not stop.is_set():
                peer.heartbeat()
                for message in peer.receive(timeout_ms=20):
                    if message.message_type == "simulation_session_granted":
                        active_session = message.lease_id
                        ui_id = str((message.payload or {}).get("ui_id", ""))
                        _report(results, "sim:session")
                    elif message.message_type == "simulation_command":
                        command = SimulationCommandRequest.from_payload(
                            message.payload or {}
                        )
                        peer.send(
                            "simulation_result",
                            target_id=message.source_id,
                            lease_id=message.lease_id,
                            payload=SimulationResultPayload(
                                request_id=command.request_id,
                                session_id=command.session_id,
                                command=command.command,
                                ok=True,
                                reason="smoke",
                            ).to_payload(),
                        )
                        _report(results, "sim:command")
                    elif message.message_type == "webrtc_signal":
                        offer = WebRtcSignalPayload.from_payload(
                            message.payload or {}
                        )
                        peer.send(
                            "webrtc_signal",
                            target_id=message.source_id,
                            lease_id=message.lease_id,
                            payload=WebRtcSignalPayload(
                                session_id=offer.session_id,
                                stream=offer.stream,
                                signal="answer",
                                sdp="v=0\r\ns=elesim-smoke\r\n",
                                type="answer",
                            ).to_payload(),
                        )
                        _report(results, "sim:webrtc")
                if active_session and ui_id and not status_sent:
                    peer.send(
                        "simulation_status",
                        lease_id=active_session,
                        payload=SimulationStatusPayload(
                            epoch=1,
                            paused=False,
                            speed=1.0,
                            debug_visible=False,
                            sim_time_s=0.0,
                        ).to_payload(),
                    )
                    status_sent = True
        finally:
            peer.close()

    _run_child("sim", results, run)


def _ui_process(barrier: Any, stop: Any, results: Any) -> None:
    def run() -> None:
        peer = PeerClient(
            EndpointDescriptor("ui-smoke", "ui", ()),
            settings=_settings(),
        )
        _wait_barrier(barrier)
        operator_sent = False
        open_sent = False
        session_id = ""
        simulation_sent = False
        try:
            while not stop.is_set():
                peer.heartbeat()
                discovery = peer.send("discover", payload={})
                endpoints = {
                    str(endpoint.get("endpoint_id", ""))
                    for endpoint in (discovery.payload or {}).get("endpoints", [])
                }
                if "pilot-smoke" in endpoints and not operator_sent:
                    peer.send(
                        "operator_intent",
                        target_id="pilot-smoke",
                        payload={
                            "request_id": "operator-smoke",
                            "operation": "view_snapshot",
                            "name": "",
                            "args": [],
                            "kwargs": {},
                        },
                    )
                    operator_sent = True
                if "sim-smoke" in endpoints and not open_sent:
                    peer.send(
                        "open_simulation_session",
                        payload=OpenSimulationSessionRequest(
                            request_id="open-smoke",
                            sim_id="sim-smoke",
                            streams=("observer",),
                        ).to_payload(),
                    )
                    open_sent = True
                for message in peer.receive(timeout_ms=20):
                    if message.message_type == "operator_result":
                        _report(results, "ui:operator")
                    elif message.message_type == "simulation_session_opened":
                        session_id = message.lease_id
                        _report(results, "ui:opened")
                    elif message.message_type == "simulation_result":
                        SimulationResultPayload.from_payload(message.payload or {})
                        _report(results, "ui:result")
                    elif message.message_type == "simulation_status":
                        SimulationStatusPayload.from_payload(message.payload or {})
                        _report(results, "ui:status")
                    elif message.message_type == "webrtc_signal":
                        answer = WebRtcSignalPayload.from_payload(
                            message.payload or {}
                        )
                        if answer.signal == "answer":
                            _report(results, "ui:answer")
                if session_id and not simulation_sent:
                    peer.send(
                        "simulation_command",
                        target_id="sim-smoke",
                        lease_id=session_id,
                        payload=SimulationCommandRequest(
                            request_id="command-smoke",
                            session_id=session_id,
                            command="pause",
                            arguments={},
                        ).to_payload(),
                    )
                    peer.send(
                        "webrtc_signal",
                        target_id="sim-smoke",
                        lease_id=session_id,
                        payload=WebRtcSignalPayload(
                            session_id=session_id,
                            stream="observer",
                            signal="offer",
                            sdp="v=0\r\ns=elesim-smoke\r\n",
                            type="offer",
                        ).to_payload(),
                    )
                    simulation_sent = True
        finally:
            peer.close()

    _run_child("ui", results, run)


def _runtime_available() -> bool:
    try:
        return (
            importlib.util.find_spec("rclpy") is not None
            and importlib.util.find_spec("elesim_interfaces.msg") is not None
        )
    except ModuleNotFoundError:
        return False


def main() -> int:
    if not _runtime_available():
        print(
            "ERROR: ROS 2 rclpy and generated elesim_interfaces are required; "
            "run this required smoke test in the generated Developer environment.",
            file=sys.stderr,
        )
        return 2

    context = mp.get_context("spawn")
    barrier = context.Barrier(4)
    stop = context.Event()
    results = context.Queue()
    processes = (
        context.Process(
            target=_robot_process,
            args=(barrier, stop, results),
            name="elesim-smoke-robot",
        ),
        context.Process(
            target=_controller_process,
            args=(barrier, stop, results),
            name="elesim-smoke-pilot",
        ),
        context.Process(
            target=_sim_process,
            args=(barrier, stop, results),
            name="elesim-smoke-sim",
        ),
        context.Process(
            target=_ui_process,
            args=(barrier, stop, results),
            name="elesim-smoke-ui",
        ),
    )
    for process in processes:
        process.start()

    observed: set[str] = set()
    errors: list[str] = []
    deadline = time.monotonic() + TIMEOUT_S
    try:
        while time.monotonic() < deadline and not EXPECTED <= observed:
            try:
                item = str(results.get(timeout=0.25))
            except queue.Empty:
                if any(
                    process.exitcode not in {None, 0}
                    for process in processes
                ):
                    break
                continue
            if item.startswith("error:"):
                errors.append(item)
            else:
                observed.add(item)
    finally:
        stop.set()
        for process in processes:
            process.join(timeout=5.0)
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(timeout=2.0)

    for process in processes:
        if process.exitcode not in {0, None}:
            errors.append(f"{process.name} exited with {process.exitcode}")
    missing = sorted(EXPECTED - observed)
    if errors or missing:
        if missing:
            errors.append("missing observations: " + ", ".join(missing))
        raise RuntimeError("\n".join(errors))
    print("Router-free four-process DDS topology smoke passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
