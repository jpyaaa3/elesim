#!/usr/bin/env python3
"""Jetson-side motor, GO2, sensor and RGBD endpoint."""

from __future__ import annotations

import argparse
import threading
import time
from typing import Any, Optional

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_MOTION_GO2,
    CAPABILITY_STREAM_RGBD,
    EndpointClient,
    EndpointDescriptor,
    Envelope,
    SimQ,
    sim_q_to_motor_deg,
)
from elesim_robot.arm.dynamixel import DXL_CURRENT_UNIT_MA, load_hardware, tick_to_deg_0_360
from elesim_robot.camera.publisher import RgbdPublisher
from elesim_robot.camera.realsense import RealSenseCamera
from elesim_robot.camera.types import RgbdFrame, RgbdIntrinsics
from elesim_robot.config import load_config
from elesim_robot.go2 import create_go2_bridge_if_enabled
from elesim_robot.tracing import configure_tracing, shutdown_tracing, span


class RobotRuntime:
    def __init__(self, *, mapping: Any, hardware_config: Any, device: str = "", go2_bridge: Any = None) -> None:
        self.mapping = mapping
        self.hardware_config = hardware_config
        self.device = str(device)
        self.hw: Any = None
        self.go2 = go2_bridge
        self.torque_enabled = False
        self.active_lease = ""
        self.controller_id = ""
        self.last_command_at = 0.0
        self.last_seq = -1
        self.deadman_s = 0.5
        self.safety_fault = ""

    def open(self) -> None:
        if self.device:
            self.hw, _direction = load_hardware(self.device, hardware_cfg=self.hardware_config)
            self.hw.open()
        if self.go2 is not None:
            self.go2.start()

    def close(self) -> None:
        self.stop_motion()
        if self.go2 is not None:
            self.go2.stop()
        if self.hw is not None:
            self.hw.close()

    def grant_lease(self, controller_id: str, lease_id: str) -> None:
        self.stop_motion()
        self.controller_id = str(controller_id)
        self.active_lease = str(lease_id)
        self.last_seq = -1

    def revoke_lease(self) -> None:
        self.stop_motion()
        self.controller_id = ""
        self.active_lease = ""
        self.last_seq = -1

    def stop_motion(self) -> None:
        if self.go2 is not None:
            self.go2.set_velocity(0.0, 0.0, 0.0)
            self.go2.tick_cmd()
        if self.hw is not None:
            try:
                self.hw.stop_arm_velocity()
            except Exception:
                pass

    def tick(self) -> None:
        now = time.monotonic()
        if self.go2 is not None:
            if self.last_command_at and now - self.last_command_at > self.deadman_s:
                self.go2.set_velocity(0.0, 0.0, 0.0)
            self.go2.tick_cmd(now)

    def apply(self, envelope: Envelope) -> tuple[bool, str]:
        payload = envelope.payload or {}
        command = str(payload.get("command", ""))
        if command == "estop":
            self.stop_motion()
            if self.hw is not None:
                self.hw.torque_off_all()
            self.torque_enabled = False
            return True, "estop"
        if envelope.lease_id != self.active_lease or envelope.source_id != self.controller_id:
            return False, "lease_mismatch"
        if envelope.seq <= self.last_seq:
            return False, "stale_sequence"
        self.last_seq = envelope.seq
        self.last_command_at = time.monotonic()
        try:
            if command == "target":
                if "go2_vel" in payload:
                    velocity = payload["go2_vel"]
                    if self.go2 is not None and isinstance(velocity, (list, tuple)) and len(velocity) == 3:
                        self.go2.set_velocity(*(float(value) for value in velocity))
                if "claw_closed" in payload and self.hw is not None:
                    claw_deg = 180.0 if bool(payload["claw_closed"]) else 0.0
                    self.hw.command_claw_deg(claw_deg)
                if "go2_sport_pose" in payload and self.go2 is not None:
                    self.go2.call_sport_pose(str(payload["go2_sport_pose"]))
                if "go2_obstacles_avoid_enable" in payload and self.go2 is not None:
                    self.go2.set_obstacles_avoid(bool(payload["go2_obstacles_avoid_enable"]))
                if "u" in payload:
                    return False, "legacy_u_not_supported"
                if "q" not in payload:
                    return True, "target_meta"
                return self._target(payload)
            if command == "torque_on":
                if self.hw is not None:
                    self.hw.set_operating_modes()
                    self.hw.set_profiles()
                    self.hw.torque_on_all()
                self.torque_enabled = True
                return True, "torque_on"
            if command == "torque_off":
                self.stop_motion()
                if self.hw is not None:
                    self.hw.torque_off_all()
                self.torque_enabled = False
                return True, command
            if command == "claw":
                if self.hw is not None:
                    self.hw.command_claw_deg(float(payload.get("degrees", 0.0)))
                return True, "claw"
            if command == "go2_velocity":
                if self.go2 is not None:
                    self.go2.set_velocity(
                        float(payload.get("vx", 0.0)),
                        float(payload.get("vy", 0.0)),
                        float(payload.get("wz", 0.0)),
                    )
                return True, "go2_velocity"
            return False, "unsupported_command"
        except Exception as exc:
            self.safety_fault = repr(exc)
            self.stop_motion()
            return False, self.safety_fault

    def _target(self, payload: dict[str, Any]) -> tuple[bool, str]:
        if "u" in payload:
            return False, "legacy_u_not_supported"
        if "q" in payload:
            raw_q = payload["q"]
            if isinstance(raw_q, (list, tuple)) and len(raw_q) == 4:
                q = SimQ(*(float(value) for value in raw_q))
            else:
                return False, "bad_q"
        else:
            return False, "missing_target"
        motor = sim_q_to_motor_deg(q, self.mapping)
        if self.hw is not None:
            self.hw.command_4dof_deg(motor.u_linear, motor.u_roll, motor.u_s1, motor.u_s2)
        return True, "target"

    def state(self) -> dict[str, object]:
        result: dict[str, object] = {
            "device": self.device,
            "torque_enabled": self.torque_enabled,
            "safety_fault": self.safety_fault,
        }
        if self.hw is not None:
            try:
                ticks = self.hw.get_present_positions()
                result["motor_positions_raw"] = {str(key): int(value) for key, value in ticks.items()}
                result["motor_positions_deg"] = {
                    str(key): tick_to_deg_0_360(int(value), int(self.hw.direction.get(key, 1)))
                    for key, value in ticks.items()
                }
                currents = {
                    int(key): int(round(self.hw.get_present_current(int(key)) * DXL_CURRENT_UNIT_MA))
                    for key in self.hw.ids
                }
                result["motor_currents_ma"] = {str(key): value for key, value in currents.items()}
                over_limit = {
                    key: value
                    for key, value in currents.items()
                    if abs(value) > int(self.hardware_config.current_limit_ma)
                }
                if over_limit:
                    self.stop_motion()
                    self.hw.torque_off_all()
                    self.torque_enabled = False
                    self.safety_fault = f"motor current limit exceeded: {over_limit}"
                    result["safety_fault"] = self.safety_fault
            except Exception as exc:
                result["read_error"] = repr(exc)
        if self.go2 is not None:
            sample = self.go2.latest_state()
            if sample is not None:
                result["go2"] = {
                    "position": list(sample.pos),
                    "rpy": list(sample.rpy),
                    "linear_velocity_body": list(sample.lin_vel_body),
                    "angular_velocity": list(sample.ang_vel_body),
                    "timestamp": sample.timestamp_s,
                }
        return result


class CameraPublisherThread:
    def __init__(self, endpoint: str, *, width: int, height: int, fps: int) -> None:
        self.endpoint = str(endpoint)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, name="robot-rgbd", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)

    def _run(self) -> None:
        camera = RealSenseCamera(color_width=self.width, color_height=self.height, fps=self.fps)
        publisher = RgbdPublisher(self.endpoint, use_jpeg=True, jpeg_quality=75, send_depth=True)
        seq = 0
        try:
            camera.start()
            while not self.stop_event.is_set():
                source = camera.capture(timeout_ms=1000)
                seq += 1
                publisher.publish(
                    RgbdFrame(
                        color_bgr=source.color_bgr,
                        depth_raw=source.depth_raw,
                        depth_scale=source.depth_scale,
                        intrinsics=RgbdIntrinsics(
                            source.intrinsics.fx,
                            source.intrinsics.fy,
                            source.intrinsics.cx,
                            source.intrinsics.cy,
                            source.intrinsics.width,
                            source.intrinsics.height,
                        ),
                        seq=seq,
                        ts=time.time(),
                    )
                )
        finally:
            camera.stop()
            publisher.close()


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim physical robot agent")
    parser.add_argument("--config", default="config/default.yaml")
    parser.add_argument("--server", default="")
    parser.add_argument("--id", default="")
    parser.add_argument("--device", default="")
    parser.add_argument("--rgbd-bind", default="")
    parser.add_argument("--rgbd-advertise", default="")
    parser.add_argument("--camera", action=argparse.BooleanOptionalAction, default=None)
    args = parser.parse_args()
    config = load_config(args.config)
    server_endpoint = str(args.server).strip() or config.server_endpoint
    endpoint_id = str(args.id).strip() or config.endpoint_id
    camera_enabled = config.camera.enabled if args.camera is None else bool(args.camera)
    rgbd_bind = str(args.rgbd_bind).strip() or config.camera.bind
    advertised = str(args.rgbd_advertise).strip() or config.camera.advertise or rgbd_bind
    capabilities = [CAPABILITY_MOTION_ARM]
    if camera_enabled:
        capabilities.append(CAPABILITY_STREAM_RGBD)
    if config.use_go2:
        capabilities.append(CAPABILITY_MOTION_GO2)
    client = EndpointClient(
        server_endpoint,
        EndpointDescriptor(
            endpoint_id,
            "robot",
            tuple(capabilities),
            streams={"rgbd": advertised} if camera_enabled else {},
        ),
    )
    go2 = create_go2_bridge_if_enabled(config.go2, use_go2=config.use_go2)
    runtime = RobotRuntime(
        mapping=config.mapping,
        hardware_config=config.arm,
        device=str(args.device).strip() or config.device,
        go2_bridge=go2,
    )
    camera = CameraPublisherThread(
        rgbd_bind,
        width=config.camera.width,
        height=config.camera.height,
        fps=config.camera.fps,
    ) if camera_enabled else None
    runtime.open()
    if camera is not None:
        camera.start()
    last_state = 0.0
    try:
        while True:
            client.heartbeat()
            for message in client.receive(timeout_ms=20):
                if message.message_type == "lease_granted":
                    runtime.grant_lease(str((message.payload or {}).get("controller_id", "")), message.lease_id)
                elif message.message_type == "lease_revoked":
                    runtime.revoke_lease()
                elif message.message_type == "motion_command":
                    ok, reason = runtime.apply(message)
                    client.send(
                        "ack",
                        target_id=message.source_id,
                        payload={"reply_to": message.message_id, "ok": ok, "reason": reason},
                        lease_id=message.lease_id,
                    )
            runtime.tick()
            now = time.monotonic()
            if runtime.controller_id and now - last_state >= 0.1:
                last_state = now
                client.send(
                    "telemetry",
                    target_id=runtime.controller_id,
                    payload=runtime.state(),
                    lease_id=runtime.active_lease,
                )
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.stop()
        runtime.close()
        client.close()


def main() -> None:
    configure_tracing("elesim-robot-agent")
    try:
        with span("robot_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
