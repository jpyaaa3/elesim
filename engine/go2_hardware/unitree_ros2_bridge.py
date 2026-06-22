from __future__ import annotations

import threading
import time
from typing import Any, Optional

from engine.go2_hardware.config import Go2HardwareConfig
from engine.go2_hardware.odom_parser import OdomSample, odom_msg_to_sample
from engine.go2_hardware.sport_api import (
    API_MOVE,
    API_STOP_MOVE,
    build_move_parameter,
    shutdown_api_id,
    stand_api_id,
    velocity_below_deadband,
)


class UnitreeRos2Bridge:
    """Jetson-side bridge: elesim go2_vel -> unitree_ros2 Sport API; /odom -> base state."""

    def __init__(self, cfg: Go2HardwareConfig) -> None:
        self._cfg = cfg
        self._lock = threading.Lock()
        self._latest: Optional[OdomSample] = None
        self._target_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._cmd_period = 1.0 / max(float(cfg.cmd_hz), 1.0)
        self._t_last_cmd = 0.0
        self._started = False
        self._stop_event = threading.Event()
        self._spin_thread: Optional[threading.Thread] = None
        self._node: Any = None
        self._pub: Any = None
        self._Request: Any = None

    def start(self) -> None:
        if self._started:
            return
        self._import_ros()
        self._node = self._RosNode("elesim_go2_bridge")
        self._pub = self._node.create_publisher(self._Request, str(self._cfg.sport_request_topic), 10)
        self._node.create_subscription(
            self._Odometry,
            str(self._cfg.odom_topic),
            self._on_odom,
            10,
        )
        self._stop_event.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop, name="go2-ros2-spin", daemon=True)
        self._spin_thread.start()
        self._started = True
        stand_id = stand_api_id(self._cfg.stand_on_start)
        if stand_id is not None:
            self._publish_api(stand_id, "")
        print(
            "[go2_bridge] started | sport=%s odom=%s cmd_hz=%.1f"
            % (self._cfg.sport_request_topic, self._cfg.odom_topic, float(self._cfg.cmd_hz))
        )

    def stop(self) -> None:
        if not self._started:
            return
        shutdown_id = shutdown_api_id(self._cfg.shutdown_mode)
        if shutdown_id is not None:
            try:
                self._publish_api(shutdown_id, "")
            except Exception:
                pass
        self._stop_event.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._node = None
        self._pub = None
        self._started = False
        print("[go2_bridge] stopped")

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._target_vel = (float(vx), float(vy), float(wz))
        if self._should_stop(vx, vy, wz):
            if bool(self._cfg.stop_on_zero_vel):
                self._publish_api(API_STOP_MOVE, "")
            return
        self._publish_move(vx, vy, wz)
        self._t_last_cmd = time.time()

    def tick_cmd(self, now_s: Optional[float] = None) -> None:
        if not self._started:
            return
        now = float(now_s if now_s is not None else time.time())
        with self._lock:
            vx, vy, wz = self._target_vel
        if self._should_stop(vx, vy, wz):
            return
        if (now - self._t_last_cmd) < self._cmd_period:
            return
        self._publish_move(vx, vy, wz)
        self._t_last_cmd = now

    def latest_state(self) -> Optional[OdomSample]:
        with self._lock:
            return self._latest

    def _should_stop(self, vx: float, vy: float, wz: float) -> bool:
        return velocity_below_deadband(vx, vy, wz, float(self._cfg.vel_deadband))

    def _publish_move(self, vx: float, vy: float, wz: float) -> None:
        self._publish_api(API_MOVE, build_move_parameter(vx, vy, wz))

    def _publish_api(self, api_id: int, parameter: str) -> None:
        if self._pub is None or self._Request is None:
            return
        msg = self._Request()
        try:
            msg.header.identity.api_id = int(api_id)
        except Exception:
            try:
                msg.identity.api_id = int(api_id)
            except Exception:
                pass
        msg.parameter = str(parameter)
        self._pub.publish(msg)

    def _on_odom(self, msg: Any) -> None:
        try:
            stamp = msg.header.stamp
            ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            ts = time.time()
        pose = msg.pose.pose
        twist = msg.twist.twist
        sample = odom_msg_to_sample(
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation_quat_xyzw=(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            lin_vel_world=(twist.linear.x, twist.linear.y, twist.linear.z),
            ang_vel_world=(twist.angular.x, twist.angular.y, twist.angular.z),
            timestamp_s=ts,
            offset_xyz=self._cfg.world_frame_offset_xyz,
            yaw_deg=float(self._cfg.world_frame_yaw_deg),
        )
        with self._lock:
            self._latest = sample

    def _import_ros(self) -> None:
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.node import Node
            from unitree_api.msg import Request
        except ImportError as exc:
            raise RuntimeError(
                "ROS2 Humble deps missing for GO2 bridge (rclpy, nav_msgs, unitree_api): "
                f"{exc}"
            ) from exc
        self._rclpy = rclpy
        self._Odometry = Odometry
        self._Request = Request
        self._RosNode = Node

    def _spin_loop(self) -> None:
        try:
            if not self._rclpy.ok():
                self._rclpy.init()
            while not self._stop_event.is_set() and self._rclpy.ok():
                self._rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception as exc:
            print(f"[go2_bridge] spin failed: {exc}")
        finally:
            try:
                if self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass


def create_go2_bridge_if_enabled(
    cfg: Go2HardwareConfig,
    *,
    use_go2: bool,
) -> Optional[UnitreeRos2Bridge]:
    if not cfg.is_active(use_go2=use_go2):
        return None
    return UnitreeRos2Bridge(cfg)
