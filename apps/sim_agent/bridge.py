"""Protocol-v2 adapter for the Genesis runtime's local state sockets."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Optional

import zmq

from engine.core.distributed import EndpointClient
from engine.core.protocol import (
    DEFAULT_START_CONTROL_U,
    EndpointDescriptor,
    Envelope,
    control_u_to_sim_q,
    pack_state,
)


class SimProtocolBridge:
    def __init__(
        self,
        *,
        server_endpoint: str,
        endpoint_id: str,
        legacy_state_bind: str,
        legacy_feedback_bind: str,
        mapping: Any,
        streams: Optional[dict[str, str]] = None,
        webrtc_offer_handler: Optional[Any] = None,
    ) -> None:
        self.server_endpoint = str(server_endpoint)
        self.endpoint_id = str(endpoint_id)
        self.legacy_state_bind = str(legacy_state_bind)
        self.legacy_feedback_bind = str(legacy_feedback_bind)
        self.mapping = mapping
        self.streams = dict(streams or {})
        self.webrtc_offer_handler = webrtc_offer_handler
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.active_lease = ""
        self.controller_id = ""
        self.last_control_seq = -1
        self.current_u = DEFAULT_START_CONTROL_U
        self.current_q = control_u_to_sim_q(self.current_u, self.mapping)
        self.state_meta: dict[str, Any] = {}

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, name="sim-v2-bridge", daemon=True)
        self.thread.start()
        if not self.ready.wait(timeout=3.0):
            raise RuntimeError("sim protocol bridge failed to start")

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)

    def _run(self) -> None:
        context = zmq.Context.instance()
        state_pub = context.socket(zmq.PUB)
        state_pub.setsockopt(zmq.LINGER, 0)
        state_pub.setsockopt(zmq.SNDHWM, 1)
        state_pub.bind(self.legacy_state_bind)
        feedback_pull = context.socket(zmq.PULL)
        feedback_pull.setsockopt(zmq.LINGER, 0)
        feedback_pull.bind(self.legacy_feedback_bind)
        client = EndpointClient(
            self.server_endpoint,
            EndpointDescriptor(
                self.endpoint_id,
                "sim",
                ("arm", "go2", "rgb", "depth", "rendered_view"),
                streams=self.streams,
            ),
        )
        poller = zmq.Poller()
        poller.register(client.socket, zmq.POLLIN)
        poller.register(feedback_pull, zmq.POLLIN)
        self.ready.set()
        try:
            while not self.stop_event.is_set():
                client.heartbeat()
                events = dict(poller.poll(20))
                if client.socket in events:
                    for message in client.receive():
                        if message.message_type == "lease_granted":
                            self.controller_id = str((message.payload or {}).get("controller_id", ""))
                            self.active_lease = message.lease_id
                            self.last_control_seq = -1
                        elif message.message_type == "lease_revoked":
                            self.controller_id = ""
                            self.active_lease = ""
                            self.last_control_seq = -1
                            self.state_meta["go2_vel"] = [0.0, 0.0, 0.0]
                            self._publish_state(state_pub)
                        elif message.message_type == "command":
                            command = str((message.payload or {}).get("command", ""))
                            ok, reason = self._validate_control(message, allow_estop=command == "estop")
                            if ok:
                                ok, reason = self._apply_command(message.payload or {})
                            self._publish_state(state_pub)
                            client.send(
                                "ack",
                                target_id=message.source_id,
                                payload={"reply_to": message.message_id, "ok": ok, "reason": reason},
                                lease_id=message.lease_id,
                            )
                        elif message.message_type == "camera_input":
                            ok, _reason = self._validate_control(message)
                            if ok:
                                from engine.vision.sim_camera.remote_control import enqueue

                                payload = message.payload or {}
                                enqueue(str(payload.get("command", "")), payload.get("values", ()))
                        elif message.message_type == "webrtc_signal" and self.webrtc_offer_handler is not None:
                            payload = message.payload or {}
                            if str(payload.get("signal", "")) == "offer":
                                answer = self.webrtc_offer_handler(str(payload.get("sdp", "")), str(payload.get("type", "offer")))
                                client.send(
                                    "webrtc_signal",
                                    target_id=message.source_id,
                                    payload={"signal": "answer", **answer},
                                )
                if feedback_pull in events:
                    while True:
                        try:
                            feedback = feedback_pull.recv_json(flags=zmq.NOBLOCK)
                        except zmq.Again:
                            break
                        if self.controller_id:
                            client.send(
                                "state",
                                target_id=self.controller_id,
                                payload={
                                    **pack_state(u=self.current_u, q=self.current_q),
                                    **{key: value for key, value in feedback.items() if key != "t"},
                                },
                                lease_id=self.active_lease,
                            )
        finally:
            client.close()
            feedback_pull.close(0)
            state_pub.close(0)

    def _validate_control(self, message: Envelope, *, allow_estop: bool = False) -> tuple[bool, str]:
        if allow_estop:
            return True, "accepted"
        if message.source_id != self.controller_id or message.lease_id != self.active_lease:
            return False, "lease_mismatch"
        if message.seq <= self.last_control_seq:
            return False, "stale_sequence"
        self.last_control_seq = message.seq
        return True, "accepted"

    def _apply_command(self, payload: dict[str, Any]) -> tuple[bool, str]:
        command = str(payload.get("command", ""))
        if command == "target":
            raw_u = payload.get("u")
            if isinstance(raw_u, dict):
                cls = type(self.current_u)
                self.current_u = cls(
                    float(raw_u.get("linear", self.current_u.u_linear)),
                    float(raw_u.get("roll", self.current_u.u_roll)),
                    float(raw_u.get("s1", self.current_u.u_s1)),
                    float(raw_u.get("s2", self.current_u.u_s2)),
                )
                self.current_q = control_u_to_sim_q(self.current_u, self.mapping)
            raw_q = payload.get("q")
            if isinstance(raw_q, dict):
                cls_q = type(self.current_q)
                self.current_q = cls_q(
                    float(raw_q.get("linear_m", self.current_q.linear_m)),
                    float(raw_q.get("roll_rad", self.current_q.roll_rad)),
                    float(raw_q.get("theta1_rad", self.current_q.theta1_rad)),
                    float(raw_q.get("theta2_rad", self.current_q.theta2_rad)),
                )
            meta = {
                key: value
                for key, value in payload.items()
                if key not in {"command", "t", "ts", "seq", "source", "q", "u"}
            }
            aliases = {
                "target": "ik_target_xyz",
                "target_dir": "ik_target_dir",
                "sim_target": "sim_target_xyz",
                "go2_obstacles_avoid_enable": "go2_obstacles_avoid_enabled",
            }
            for source, destination in aliases.items():
                if source in meta:
                    meta[destination] = meta.pop(source)
            if "go2_sport_pose" in meta:
                meta["go2_sport_pose_seq"] = int(self.state_meta.get("go2_sport_pose_seq", 0)) + 1
            if "go2_obstacles_avoid_enabled" in meta:
                meta["go2_obstacles_avoid_seq"] = int(self.state_meta.get("go2_obstacles_avoid_seq", 0)) + 1
            self.state_meta.update(meta)
            return True, "target"
        if command == "sim_reset":
            self.state_meta["sim_reset_seq"] = int(self.state_meta.get("sim_reset_seq", 0)) + 1
            return True, "sim_reset"
        if command in {"torque_on", "torque_off", "estop"}:
            self.state_meta["torque_enabled"] = command == "torque_on"
            if command in {"torque_off", "estop"}:
                self.state_meta["go2_vel"] = [0.0, 0.0, 0.0]
            return True, command
        return False, "unsupported_command"

    def _publish_state(self, socket: zmq.Socket) -> None:
        message = pack_state(u=self.current_u, q=self.current_q, **self._pack_state_kwargs())
        message.update(self.state_meta)
        socket.send(json.dumps(message, separators=(",", ":")).encode("utf-8"))

    def _pack_state_kwargs(self) -> dict[str, Any]:
        allowed = {
            "torque_enabled", "claw_closed", "ik_target_xyz", "ik_target_dir", "sag_model",
            "go2_vel", "sim_target_xyz", "debug_markers", "sim_reset_seq",
            "go2_sport_pose", "go2_sport_pose_seq", "go2_obstacles_avoid_enabled",
            "go2_obstacles_avoid_seq",
        }
        return {key: value for key, value in self.state_meta.items() if key in allowed}
