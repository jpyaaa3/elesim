#!/usr/bin/env python3
from __future__ import annotations

import multiprocessing as mp
import socket
import threading
import time
from queue import Empty

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_OPERATOR_CONTROL,
    EndpointClient,
    EndpointDescriptor,
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
    client = EndpointClient(
        endpoint,
        EndpointDescriptor(endpoint_id, role, (CAPABILITY_MOTION_ARM,)),
    )
    results.put(f"{role}_registered")
    try:
        while not stop.is_set():
            client.heartbeat()
            for message in client.receive(timeout_ms=50):
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
        EndpointDescriptor("controller-main", "controller", (CAPABILITY_OPERATOR_CONTROL,)),
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
                    request_id = str((message.payload or {}).get("request_id", ""))
                    client.send(
                        "operator_result",
                        target_id=message.source_id,
                        payload={"request_id": request_id, "ok": True, "result": {"ready": True}},
                    )
            if not selected:
                time.sleep(0.1)
    finally:
        client.close()


def ui_process(endpoint: str, stop: mp.Event, results: mp.Queue) -> None:
    client = EndpointClient(endpoint, EndpointDescriptor("ui-main", "ui", ()))
    request_id = "smoke-request"
    sent = False
    try:
        while not stop.is_set():
            client.heartbeat()
            if not sent:
                time.sleep(0.3)
                client.send(
                    "operator_intent",
                    target_id="controller-main",
                    payload={"request_id": request_id, "operation": "snapshot"},
                )
                sent = True
            for message in client.receive(timeout_ms=50):
                body = message.payload or {}
                if message.message_type == "operator_result" and body.get("request_id") == request_id:
                    results.put("ui_result")
                    return
    finally:
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
