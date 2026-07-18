"""Adapter between the existing ControlService client contract and protocol v2."""

from __future__ import annotations

import threading
import time
import queue
from typing import Any, Optional

import zmq

from engine.core.distributed import EndpointClient
from engine.core.protocol import EndpointDescriptor, Envelope


class LegacyControlBridge:
    def __init__(
        self,
        *,
        local_endpoint: str,
        server_endpoint: str,
        controller_id: str,
        initial_target: str = "",
    ) -> None:
        self.local_endpoint = str(local_endpoint)
        self.server_endpoint = str(server_endpoint)
        self.controller_id = str(controller_id)
        self.initial_target = str(initial_target)
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.active_target = ""
        self.lease_id = ""
        self.endpoints: list[dict[str, Any]] = []
        self.last_state: dict[str, Any] = {"t": "state", "connected": False}
        self.outbox: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
        self.on_target_selected: Optional[Any] = None

    def send_camera_input(self, command: str, values: tuple[float, ...] = ()) -> None:
        self.outbox.put(("camera_input", {"command": str(command), "values": [float(value) for value in values]}))

    def select_target(self, target_id: str) -> None:
        self.outbox.put(("select_target", {"target_id": str(target_id)}))

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, name="control-v2-bridge", daemon=True)
        self.thread.start()
        if not self.ready.wait(timeout=3.0):
            raise RuntimeError("control bridge failed to bind its local endpoint")

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)

    def _run(self) -> None:
        context = zmq.Context.instance()
        local = context.socket(zmq.ROUTER)
        local.setsockopt(zmq.LINGER, 0)
        local.bind(self.local_endpoint)
        endpoint = EndpointClient(
            self.server_endpoint,
            EndpointDescriptor(self.controller_id, "controller", ("pick", "gaze", "ik", "yolo")),
        )
        poller = zmq.Poller()
        poller.register(local, zmq.POLLIN)
        poller.register(endpoint.socket, zmq.POLLIN)
        clients: set[bytes] = set()
        self.ready.set()
        requested_initial = False
        try:
            while not self.stop_event.is_set():
                endpoint.heartbeat()
                events = dict(poller.poll(50))
                if local in events:
                    identity, message = local.recv_multipart()
                    clients.add(identity)
                    self._from_local(endpoint, local, identity, message)
                if endpoint.socket in events:
                    for message in endpoint.receive():
                        self._from_server(endpoint, local, clients, message)
                        if message.message_type == "registered":
                            endpoint.send("list_endpoints")
                if self.initial_target and self.endpoints and not requested_initial:
                    if any(item.get("endpoint_id") == self.initial_target for item in self.endpoints):
                        endpoint.send("select_target", payload={"target_id": self.initial_target})
                        requested_initial = True
                while True:
                    try:
                        outgoing_type, payload = self.outbox.get_nowait()
                    except queue.Empty:
                        break
                    if outgoing_type == "select_target":
                        endpoint.send("select_target", payload=payload)
                    elif self.active_target and self.lease_id:
                        endpoint.send(
                            outgoing_type,
                            target_id=self.active_target,
                            payload=payload,
                            lease_id=self.lease_id,
                        )
        finally:
            endpoint.close()
            local.close(0)

    def _from_local(
        self,
        endpoint: EndpointClient,
        socket: zmq.Socket,
        identity: bytes,
        data: bytes,
    ) -> None:
        try:
            message = __import__("json").loads(data.decode("utf-8"))
        except Exception:
            socket.send_multipart([identity, __import__("json").dumps({"t": "ack", "ok": False, "reason": "json"}).encode()])
            return
        message_type = str(message.get("t", ""))
        if message_type == "hello":
            socket.send_multipart([identity, __import__("json").dumps(self.last_state).encode()])
            return
        if message_type == "list_endpoints":
            endpoint.send("list_endpoints")
            return
        if message_type == "select_target":
            endpoint.send("select_target", payload={"target_id": str(message.get("target_id", ""))})
            return
        if message_type == "estop" and self.active_target:
            endpoint.send(
                "command",
                target_id=self.active_target,
                payload={"command": "estop", **message},
            )
            return
        if not self.active_target or not self.lease_id:
            reply = {"t": "ack", "ok": False, "reason": "no_active_target"}
            socket.send_multipart([identity, __import__("json").dumps(reply).encode()])
            return
        command = dict(message)
        command["command"] = message_type
        endpoint.send(
            "command",
            target_id=self.active_target,
            payload=command,
            lease_id=self.lease_id,
        )

    def _from_server(
        self,
        endpoint: EndpointClient,
        socket: zmq.Socket,
        clients: set[bytes],
        message: Envelope,
    ) -> None:
        payload = dict(message.payload or {})
        if message.message_type == "endpoint_list":
            self.endpoints = list(payload.get("endpoints", []))
            return
        if message.message_type == "target_selected":
            self.active_target = str(payload.get("target_id", ""))
            self.lease_id = str(payload.get("lease_id", ""))
            descriptor = next(
                (item for item in self.endpoints if item.get("endpoint_id") == self.active_target),
                None,
            )
            callback = self.on_target_selected
            if callback is not None and descriptor is not None:
                try:
                    callback(dict(descriptor))
                except Exception as exc:
                    print(f"[control_agent] target stream setup failed: {exc}")
            self._broadcast(socket, clients, {"t": "ack", "ok": True, "reason": "target_selected"})
            return
        if message.message_type in {"target_lost", "target_released"}:
            self.active_target = ""
            self.lease_id = ""
        if message.message_type == "state":
            self.last_state = {"t": "state", "ts": time.time(), **payload}
            self._broadcast(socket, clients, self.last_state)
        elif message.message_type == "ack":
            self._broadcast(socket, clients, {"t": "ack", "ts": time.time(), **payload})
        elif message.message_type == "error":
            self._broadcast(
                socket,
                clients,
                {"t": "ack", "ts": time.time(), "ok": False, "reason": str(payload.get("reason", "error"))},
            )

    @staticmethod
    def _broadcast(socket: zmq.Socket, clients: set[bytes], message: dict[str, Any]) -> None:
        data = __import__("json").dumps(message, separators=(",", ":")).encode("utf-8")
        for identity in tuple(clients):
            socket.send_multipart([identity, data])
