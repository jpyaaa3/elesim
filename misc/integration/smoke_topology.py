#!/usr/bin/env python3
from __future__ import annotations

import multiprocessing as mp
import socket
import threading
import time
from queue import Empty

from elesim_protocol import (
    OPERATOR_VIEW_SCHEMA_VERSION,
    CAPABILITY_MOTION_ARM,
    CAPABILITY_OPERATOR_CONTROL,
    CAPABILITY_STREAM_HAND_EYE_PREVIEW,
    CAPABILITY_STREAM_OBSERVER,
    EndpointClient,
    EndpointDescriptor,
    MEDIA_KIND_RGB,
    MEDIA_SECURITY_DTLS_SRTP,
    MEDIA_TRANSPORT_WEBRTC,
    MediaStreamDescriptor,
    OpenSimulationSessionRequest,
    OperatorViewSnapshot,
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationSessionGrantedPayload,
    SimulationSessionOpenedPayload,
    SimulationStatusPayload,
    WebRtcSignalPayload,
)
from elesim_router.main import RoutingServer


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def router_process(endpoint: str, stop: mp.Event) -> None:
    server = RoutingServer(endpoint, heartbeat_timeout_s=2.0)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    stop.wait(15.0)
    server.close()
    thread.join(timeout=2.0)


def target_process(role: str, endpoint_id: str, endpoint: str, stop: mp.Event, results: mp.Queue) -> None:
    streams = {}
    capabilities = [CAPABILITY_MOTION_ARM]
    if role == "simulator":
        capabilities.extend(
            (CAPABILITY_STREAM_OBSERVER, CAPABILITY_STREAM_HAND_EYE_PREVIEW)
        )
        streams = {
            name: MediaStreamDescriptor(
                transport=MEDIA_TRANSPORT_WEBRTC,
                media_kind=MEDIA_KIND_RGB,
                endpoint=f"webrtc://{endpoint_id}/{name}",
                security=MEDIA_SECURITY_DTLS_SRTP,
            )
            for name in ("observer", "hand_eye_preview")
        }
    client = EndpointClient(
        endpoint,
        EndpointDescriptor(
            endpoint_id,
            role,
            tuple(capabilities),
            streams=streams,
            instance_id=f"{endpoint_id}-smoke",
        ),
    )
    results.put(f"{role}_registered")
    simulation_session_id = ""
    simulation_ui_id = ""
    try:
        while not stop.is_set():
            client.heartbeat()
            for message in client.receive(timeout_ms=50):
                if message.message_type == "simulation_session_granted":
                    granted = SimulationSessionGrantedPayload.from_payload(message.payload)
                    simulation_session_id = granted.session_id
                    simulation_ui_id = granted.ui_id
                    results.put("simulator_session_granted")
                    continue
                if message.message_type == "webrtc_signal":
                    signal = WebRtcSignalPayload.from_payload(message.payload)
                    if signal.signal == "offer" and signal.session_id == simulation_session_id:
                        answer = WebRtcSignalPayload(
                            session_id=signal.session_id,
                            stream=signal.stream,
                            signal="answer",
                            sdp=f"answer-{signal.stream}",
                            type="answer",
                        )
                        client.send(
                            "webrtc_signal",
                            target_id=simulation_ui_id,
                            payload=answer.to_payload(),
                            lease_id=simulation_session_id,
                        )
                    continue
                if message.message_type == "simulation_command":
                    command = SimulationCommandRequest.from_payload(message.payload)
                    result = SimulationResultPayload(
                        request_id=command.request_id,
                        session_id=command.session_id,
                        command=command.command,
                        ok=True,
                        reason="applied",
                    )
                    client.send(
                        "simulation_result",
                        target_id=simulation_ui_id,
                        payload=result.to_payload(),
                        lease_id=simulation_session_id,
                    )
                    client.send(
                        "simulation_status",
                        payload=SimulationStatusPayload(
                            epoch=0,
                            paused=command.command == "pause",
                            speed=1.0,
                            debug_visible=True,
                            sim_time_s=1.0,
                        ).to_payload(),
                    )
                    results.put("simulator_command")
                    continue
                if message.message_type == "motion_command":
                    q = (message.payload or {}).get("q")
                    if isinstance(q, list) and len(q) == 4:
                        results.put(f"{role}_motion")
                        client.send(
                            "ack",
                            target_id=message.source_id,
                            lease_id=message.lease_id,
                            payload={"reply_to": message.message_id, "ok": True},
                        )
    finally:
        client.close()


def controller_process(endpoint: str, stop: mp.Event, results: mp.Queue) -> None:
    client = EndpointClient(
        endpoint,
        EndpointDescriptor(
            "controller-main",
            "controller",
            (CAPABILITY_OPERATOR_CONTROL,),
            instance_id="controller-main-smoke",
        ),
    )
    selected = False
    lease_id = ""
    try:
        while not stop.is_set():
            client.heartbeat()
            if not selected:
                client.send("select_target", payload={"target_id": "sim-default"})
            for message in client.receive(timeout_ms=50):
                if message.message_type == "target_selected":
                    lease_id = str((message.payload or {}).get("lease_id", ""))
                    selected = bool(lease_id)
                    if selected:
                        results.put("controller_selected")
                        client.send(
                            "motion_command",
                            target_id="sim-default",
                            lease_id=lease_id,
                            payload={"command": "target", "q": [-0.1, 0.0, 0.1, -0.1]},
                        )
                elif message.message_type == "operator_intent":
                    intent = message.payload or {}
                    request_id = str(intent.get("request_id", ""))
                    if str(intent.get("operation", "")) == "view_snapshot":
                        result = {
                            "schema_version": OPERATOR_VIEW_SCHEMA_VERSION,
                            "state": {"pick_running": False},
                            "service": {
                                "has_client": True,
                                "current_host_state": {
                                    "connected": False,
                                    "rx_age_s": -1.0,
                                    "host_state_age_s": -1.0,
                                },
                            },
                        }
                    else:
                        result = {"ready": True}
                    client.send(
                        "operator_result",
                        target_id=message.source_id,
                        payload={"request_id": request_id, "ok": True, "result": result},
                    )
            if not selected:
                time.sleep(0.1)
    finally:
        client.close()


def ui_process(endpoint: str, stop: mp.Event, results: mp.Queue) -> None:
    client = EndpointClient(
        endpoint,
        EndpointDescriptor("ui-main", "ui", (), instance_id="ui-main-smoke"),
    )
    simulator = EndpointClient(
        endpoint,
        EndpointDescriptor(
            "ui-main-simulator",
            "ui",
            (),
            instance_id="ui-main-simulator-smoke",
        ),
    )
    request_id = "smoke-request"
    sent = False
    simulation_open_sent = False
    simulation_session_id = ""
    simulation_result_received = False
    simulation_status_received = False
    answered_streams: set[str] = set()
    operator_result_received = False
    try:
        while not stop.is_set():
            client.heartbeat()
            simulator.heartbeat()
            if not sent:
                time.sleep(0.3)
                client.send(
                    "operator_intent",
                    target_id="controller-main",
                    payload={"request_id": request_id, "operation": "view_snapshot"},
                )
                sent = True
            if not simulation_open_sent:
                opened = OpenSimulationSessionRequest(
                    request_id="simulation-open-smoke",
                    simulator_id="sim-default",
                    streams=("observer", "hand_eye_preview"),
                )
                simulator.send(
                    "open_simulation_session",
                    payload=opened.to_payload(),
                )
                simulation_open_sent = True
            for message in client.receive(timeout_ms=50):
                body = message.payload or {}
                if message.message_type == "operator_result" and body.get("request_id") == request_id:
                    view = OperatorViewSnapshot.from_payload(body.get("result"))
                    host = view.service.get("current_host_state", {})
                    if host.get("rx_age_s") != -1.0 or host.get("host_state_age_s") != -1.0:
                        raise RuntimeError(f"unexpected initial host age: {host!r}")
                    results.put("ui_result")
                    operator_result_received = True
            for message in simulator.receive(timeout_ms=20):
                if message.message_type == "simulation_session_opened":
                    opened = SimulationSessionOpenedPayload.from_payload(message.payload)
                    simulation_session_id = opened.session_id
                    for stream in opened.streams:
                        offer = WebRtcSignalPayload(
                            session_id=opened.session_id,
                            stream=stream,
                            signal="offer",
                            sdp=f"offer-{stream}",
                            type="offer",
                        )
                        simulator.send(
                            "webrtc_signal",
                            target_id=opened.simulator_id,
                            payload=offer.to_payload(),
                            lease_id=opened.session_id,
                        )
                    command = SimulationCommandRequest(
                        request_id="simulation-command-smoke",
                        session_id=opened.session_id,
                        command="pause",
                        arguments={},
                    )
                    simulator.send(
                        "simulation_command",
                        target_id=opened.simulator_id,
                        payload=command.to_payload(),
                        lease_id=opened.session_id,
                    )
                elif message.message_type == "webrtc_signal":
                    signal = WebRtcSignalPayload.from_payload(message.payload)
                    if signal.session_id == simulation_session_id and signal.signal == "answer":
                        answered_streams.add(signal.stream)
                        if answered_streams == {"observer", "hand_eye_preview"}:
                            results.put("ui_dual_webrtc")
                elif message.message_type == "simulation_result":
                    result = SimulationResultPayload.from_payload(message.payload)
                    simulation_result_received = result.ok
                    if result.ok:
                        results.put("ui_simulation_result")
                elif message.message_type == "simulation_status":
                    status = SimulationStatusPayload.from_payload(message.payload)
                    simulation_status_received = status.paused
                    if status.paused:
                        results.put("ui_simulation_status")
            if (
                operator_result_received
                and simulation_result_received
                and simulation_status_received
                and answered_streams == {"observer", "hand_eye_preview"}
            ):
                return
    finally:
        simulator.close()
        client.close()


def main() -> None:
    endpoint = f"tcp://127.0.0.1:{free_port()}"
    stop = mp.Event()
    results: mp.Queue = mp.Queue()
    processes = [
        mp.Process(target=router_process, args=(endpoint, stop), name="router"),
        mp.Process(target=controller_process, args=(endpoint, stop, results), name="controller"),
        mp.Process(target=target_process, args=("robot", "robot-go2", endpoint, stop, results), name="robot"),
        mp.Process(target=target_process, args=("simulator", "sim-default", endpoint, stop, results), name="simulator"),
        mp.Process(target=ui_process, args=(endpoint, stop, results), name="ui"),
    ]
    for process in processes:
        process.start()
    expected = {
        "robot_registered",
        "simulator_registered",
        "controller_selected",
        "simulator_motion",
        "ui_result",
        "simulator_session_granted",
        "simulator_command",
        "ui_dual_webrtc",
        "ui_simulation_result",
        "ui_simulation_status",
    }
    seen: set[str] = set()
    deadline = time.monotonic() + 10.0
    try:
        while expected - seen and time.monotonic() < deadline:
            try:
                seen.add(str(results.get(timeout=0.25)))
            except Empty:
                pass
        missing = expected - seen
        if missing:
            raise RuntimeError(f"distributed smoke missing events: {sorted(missing)}; seen={sorted(seen)}")
        print("five-process topology smoke passed")
    finally:
        stop.set()
        for process in processes:
            process.join(timeout=3.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)


if __name__ == "__main__":
    main()
