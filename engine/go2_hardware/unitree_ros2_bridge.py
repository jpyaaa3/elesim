from __future__ import annotations

import threading
import time
from dataclasses import replace
from typing import Any, Optional, Tuple

from engine.go2_hardware.config import Go2HardwareConfig
from engine.go2_hardware.obstacles_avoid_api import build_obstacles_avoid_parameter
from engine.go2_hardware.ros_env import bootstrap_ros_python_path, ros_import_hint
from engine.go2_hardware.odom_parser import OdomSample, odom_msg_to_sample
from engine.go2_hardware.lowstate_parser import lowstate_leg_q_genesis_order
from engine.go2_hardware.sport_state_parser import sportmodestate_to_sample
from engine.go2_hardware.sport_api import (
    API_MOVE,
    API_STOP_MOVE,
    build_move_parameter,
    fill_unitree_request,
    shutdown_api_id,
    gait_on_start_api_id,
    sport_pose_api_id,
    stand_api_id,
    velocity_below_deadband,
)
from engine.go2_hardware.vel_feedback import Go2VelFeedbackGains, compute_feedback_cmd


def _ros_topic(name: str) -> str:
    topic = str(name).strip()
    if not topic:
        return topic
    if not topic.startswith("/"):
        topic = "/" + topic
    return topic


def _age_s(now_s: float, last_s: float) -> Optional[float]:
    if last_s <= 0.0:
        return None
    return max(0.0, float(now_s) - float(last_s))


def _fmt_age(now_s: float, last_s: float) -> str:
    age = _age_s(now_s, last_s)
    if age is None:
        return "never"
    return f"{age:.3f}s ago"


class UnitreeRos2Bridge:
    """Jetson-side bridge: elesim go2_vel -> unitree_ros2 Sport API; pose topic -> base state."""

    def __init__(self, cfg: Go2HardwareConfig) -> None:
        self._cfg = cfg
        self._pose_source = str(cfg.pose_source).strip().lower()
        self._lock = threading.Lock()
        self._latest: Optional[OdomSample] = None
        self._latest_leg_q: Optional[Tuple[float, ...]] = None
        self._target_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_move_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._cmd_period = 1.0 / max(float(cfg.cmd_hz), 1.0)
        self._t_last_cmd = 0.0
        self._started = False
        self._stop_event = threading.Event()
        self._spin_thread: Optional[threading.Thread] = None
        self._node: Any = None
        self._pub: Any = None
        self._obstacles_pub: Any = None
        self._Request: Any = None
        self._sport_request_topic = _ros_topic(cfg.sport_request_topic)
        self._obstacles_request_topic = _ros_topic(cfg.obstacles_avoid_request_topic)
        self._obstacles_avoid_api_id = int(cfg.obstacles_avoid_api_id)
        self._pose_topic = (
            _ros_topic(cfg.sport_state_topic)
            if self._pose_source == "sportmodestate"
            else _ros_topic(cfg.odom_topic)
        )
        self._lowstate_topic = _ros_topic(cfg.lowstate_topic)
        self._pose_rx_count = 0
        self._lowstate_rx_count = 0
        self._api_pub_count = 0
        self._t_last_pose_rx = 0.0
        self._t_last_lowstate_rx = 0.0
        self._t_last_status_log = 0.0
        self._last_api_id = 0
        self._last_api_parameter = ""
        self._spin_ok = False
        self._spin_error = ""
        self._we_inited_rclpy = False

    def start(self) -> None:
        if self._started:
            return
        self._import_ros()
        self._ensure_rclpy_init()
        self._node = self._RosNode("elesim_go2_bridge")
        self._pub = self._node.create_publisher(self._Request, self._sport_request_topic, 10)
        self._obstacles_pub = self._node.create_publisher(self._Request, self._obstacles_request_topic, 10)
        if self._pose_source == "sportmodestate":
            self._node.create_subscription(
                self._SportModeState,
                self._pose_topic,
                self._on_sportmodestate,
                10,
            )
        else:
            self._node.create_subscription(
                self._Odometry,
                self._pose_topic,
                self._on_odom,
                10,
            )
        if bool(self._cfg.leg_sync):
            self._node.create_subscription(
                self._LowState,
                self._lowstate_topic,
                self._on_lowstate,
                10,
            )
        self._stop_event.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop, name="go2-ros2-spin", daemon=True)
        self._spin_thread.start()
        self._started = True
        stand_id = stand_api_id(self._cfg.stand_on_start)
        if stand_id is not None:
            self._publish_api(stand_id, "")
        gait_id = gait_on_start_api_id(self._cfg.gait_on_start)
        if gait_id is not None:
            self._publish_api(gait_id, "")
        print(
            "[go2_bridge] started | sport=%s pose=%s(%s) leg_sync=%s lowstate=%s cmd_hz=%.1f status_log=%.1fs gait_on_start=%s vel_fb=%s"
            % (
                self._sport_request_topic,
                self._pose_source,
                self._pose_topic,
                bool(self._cfg.leg_sync),
                self._lowstate_topic if bool(self._cfg.leg_sync) else "-",
                float(self._cfg.cmd_hz),
                float(self._cfg.status_log_interval_s),
                str(self._cfg.gait_on_start).strip().lower(),
                bool(self._cfg.vel_feedback_enable),
            )
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
        self._obstacles_pub = None
        self._started = False
        if self._we_inited_rclpy:
            try:
                if self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass
            self._we_inited_rclpy = False
        print("[go2_bridge] stopped")

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._target_vel = (float(vx), float(vy), float(wz))
        if self._should_stop(vx, vy, wz):
            self._last_move_vel = (0.0, 0.0, 0.0)
            if bool(self._cfg.stop_on_zero_vel):
                self._publish_api(API_STOP_MOVE, "")
            return
        cmd_vx, cmd_vy, cmd_wz = self._cmd_vel_for_target(vx, vy, wz)
        self._publish_move(cmd_vx, cmd_vy, cmd_wz)
        self._t_last_cmd = time.time()

    def call_sport_pose(self, pose: str) -> None:
        api_id = sport_pose_api_id(pose)
        if api_id is None:
            raise ValueError(f"unknown GO2 sport pose: {pose}")
        with self._lock:
            self._target_vel = (0.0, 0.0, 0.0)
        if bool(self._cfg.stop_on_zero_vel):
            self._publish_api(API_STOP_MOVE, "")
        self._publish_api(int(api_id), "")
        self._t_last_cmd = time.time()
        print("[go2_bridge] sport_pose=%s api_id=%d" % (str(pose).strip().lower(), int(api_id)))

    def set_obstacles_avoid(self, enabled: bool) -> None:
        parameter = build_obstacles_avoid_parameter(enable=bool(enabled))
        self._publish_obstacles_avoid_api(int(self._obstacles_avoid_api_id), parameter)
        print(
            "[go2_bridge] obstacles_avoid enable=%s api_id=%d topic=%s"
            % (bool(enabled), int(self._obstacles_avoid_api_id), self._obstacles_request_topic)
        )

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
        cmd_vx, cmd_vy, cmd_wz = self._cmd_vel_for_target(vx, vy, wz)
        self._publish_move(cmd_vx, cmd_vy, cmd_wz)
        self._t_last_cmd = now

    def latest_state(self) -> Optional[OdomSample]:
        with self._lock:
            return self._latest

    def maybe_log_status(self, now_s: Optional[float] = None) -> None:
        interval = float(self._cfg.status_log_interval_s)
        if interval <= 0.0:
            return
        now = float(now_s if now_s is not None else time.time())
        with self._lock:
            if self._t_last_status_log > 0.0 and (now - self._t_last_status_log) < interval:
                return
            self._t_last_status_log = now
            pose_rx = int(self._pose_rx_count)
            lowstate_rx = int(self._lowstate_rx_count)
            api_pub = int(self._api_pub_count)
            t_pose = float(self._t_last_pose_rx)
            t_low = float(self._t_last_lowstate_rx)
            target_vel = tuple(float(v) for v in self._target_vel)
            move_vel = tuple(float(v) for v in self._last_move_vel)
            sample = self._latest
            leg_q = self._latest_leg_q
            last_api_id = int(self._last_api_id)
            last_api_parameter = str(self._last_api_parameter)
            spin_ok = bool(self._spin_ok)
            spin_error = str(self._spin_error)

        pose_age = _fmt_age(now, t_pose)
        low_age = _fmt_age(now, t_low)
        spin_state = "ok" if spin_ok else ("failed" if spin_error else "starting")

        if sample is None:
            pos_txt = "none"
            rpy_txt = "none"
            lin_txt = "none"
            ang_txt = "none"
        else:
            pos_txt = "(%.3f, %.3f, %.3f)" % (sample.pos[0], sample.pos[1], sample.pos[2])
            rpy_txt = "(%.3f, %.3f, %.3f)" % (sample.rpy[0], sample.rpy[1], sample.rpy[2])
            lin_txt = "(%.3f, %.3f, %.3f)" % (
                sample.lin_vel_body[0],
                sample.lin_vel_body[1],
                sample.lin_vel_body[2],
            )
            ang_txt = "(%.3f, %.3f, %.3f)" % (
                sample.ang_vel_body[0],
                sample.ang_vel_body[1],
                sample.ang_vel_body[2],
            )

        if leg_q is not None and len(leg_q) == 12:
            legs_txt = "12 joints FL=(%.2f,%.2f,%.2f)" % (leg_q[0], leg_q[1], leg_q[2])
        elif leg_q is not None:
            legs_txt = f"{len(leg_q)} joints"
        else:
            legs_txt = "none"

        warn = ""
        if pose_rx == 0:
            warn = f" | WARNING: no pose on {self._pose_topic}"
        elif t_pose > 0.0 and (now - t_pose) > max(1.0, interval * 2.0):
            warn = f" | WARNING: pose stale ({pose_age})"
        if bool(self._cfg.leg_sync) and lowstate_rx == 0:
            warn += f" | WARNING: no lowstate on {self._lowstate_topic}"

        print(
            "[go2_bridge] status | spin=%s | pub=%s pose_rx=%d(%s) lowstate_rx=%d(%s) api_pub=%d"
            % (spin_state, self._sport_request_topic, pose_rx, pose_age, lowstate_rx, low_age, api_pub)
        )
        print(
            "[go2_bridge]   target_vel=%s move_vel=%s | pos=%s rpy=%s | vel_body=%s ang_body=%s | legs=%s"
            % (
                "(%.2f, %.2f, %.2f)" % target_vel,
                "(%.2f, %.2f, %.2f)" % move_vel,
                pos_txt,
                rpy_txt,
                lin_txt,
                ang_txt,
                legs_txt,
            )
        )
        if last_api_id > 0:
            param_short = last_api_parameter if len(last_api_parameter) <= 80 else (last_api_parameter[:77] + "...")
            print("[go2_bridge]   last_api id=%d param=%s%s" % (last_api_id, param_short, warn))
        else:
            print("[go2_bridge]   last_api=none%s" % warn)
        if spin_error:
            print("[go2_bridge]   spin_error=%s" % spin_error)

    def _should_stop(self, vx: float, vy: float, wz: float) -> bool:
        return velocity_below_deadband(vx, vy, wz, float(self._cfg.vel_deadband))

    def _vel_feedback_gains(self) -> Go2VelFeedbackGains:
        return Go2VelFeedbackGains(
            kp_vx=float(self._cfg.vel_feedback_kp_vx),
            kp_vy=float(self._cfg.vel_feedback_kp_vy),
            kp_wz=float(self._cfg.vel_feedback_kp_wz),
            max_vx=float(self._cfg.vel_feedback_max_vx),
            max_vy=float(self._cfg.vel_feedback_max_vy),
            max_wz=float(self._cfg.vel_feedback_max_wz),
            max_corr_vx=float(self._cfg.vel_feedback_max_corr_vx),
            max_corr_vy=float(self._cfg.vel_feedback_max_corr_vy),
            max_corr_wz=float(self._cfg.vel_feedback_max_corr_wz),
            axis_deadband=float(self._cfg.vel_deadband),
        )

    def _cmd_vel_for_target(self, vx: float, vy: float, wz: float) -> tuple[float, float, float]:
        target = (float(vx), float(vy), float(wz))
        if not bool(self._cfg.vel_feedback_enable):
            self._last_move_vel = target
            return target
        sample = self.latest_state()
        if sample is None:
            self._last_move_vel = target
            return target
        cmd = compute_feedback_cmd(
            target[0],
            target[1],
            target[2],
            sample.lin_vel_body[0],
            sample.lin_vel_body[1],
            sample.ang_vel_body[2],
            gains=self._vel_feedback_gains(),
        )
        self._last_move_vel = cmd
        return cmd

    def _publish_move(self, vx: float, vy: float, wz: float) -> None:
        self._publish_api(API_MOVE, build_move_parameter(vx, vy, wz))

    def _publish_api(self, api_id: int, parameter: str) -> None:
        self._publish_request(self._pub, int(api_id), str(parameter), identity_id=0, noreply=True)

    def _publish_obstacles_avoid_api(self, api_id: int, parameter: str) -> None:
        self._publish_request(self._obstacles_pub, int(api_id), str(parameter), identity_id=1, noreply=False)

    def _publish_request(
        self,
        pub: Any,
        api_id: int,
        parameter: str,
        *,
        identity_id: int,
        noreply: bool,
    ) -> None:
        if pub is None or self._Request is None:
            return
        msg = self._Request()
        fill_unitree_request(
            msg,
            api_id=int(api_id),
            parameter=str(parameter),
            identity_id=int(identity_id),
            noreply=bool(noreply),
        )
        pub.publish(msg)
        with self._lock:
            self._api_pub_count += 1
            self._last_api_id = int(api_id)
            self._last_api_parameter = str(parameter)

    def _on_odom(self, msg: Any) -> None:
        now = time.time()
        try:
            stamp = msg.header.stamp
            ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            ts = now
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
            self._pose_rx_count += 1
            self._t_last_pose_rx = now
            self._latest = self._attach_leg_q(sample)

    def _attach_leg_q(self, sample: OdomSample) -> OdomSample:
        if self._latest_leg_q is not None:
            return replace(sample, leg_q=self._latest_leg_q)
        return sample

    def _on_sportmodestate(self, msg: Any) -> None:
        now = time.time()
        try:
            sample = sportmodestate_to_sample(
                msg,
                offset_xyz=self._cfg.world_frame_offset_xyz,
                yaw_deg=float(self._cfg.world_frame_yaw_deg),
            )
        except Exception as exc:
            print(f"[go2_bridge] sportmodestate parse failed: {exc}")
            return
        with self._lock:
            self._pose_rx_count += 1
            self._t_last_pose_rx = now
            self._latest = self._attach_leg_q(sample)

    def _on_lowstate(self, msg: Any) -> None:
        now = time.time()
        try:
            leg_q = lowstate_leg_q_genesis_order(msg)
        except Exception as exc:
            print(f"[go2_bridge] lowstate parse failed: {exc}")
            return
        with self._lock:
            self._lowstate_rx_count += 1
            self._t_last_lowstate_rx = now
            self._latest_leg_q = leg_q
            if self._latest is not None:
                self._latest = replace(self._latest, leg_q=leg_q)

    def _import_ros(self) -> None:
        bootstrap_ros_python_path(config_workspace=str(self._cfg.ros_workspace))
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.node import Node
            from unitree_api.msg import Request
            from unitree_go.msg import LowState, SportModeState
        except ImportError as exc:
            raise RuntimeError(ros_import_hint(config_workspace=str(self._cfg.ros_workspace))) from exc
        self._rclpy = rclpy
        self._Odometry = Odometry
        self._SportModeState = SportModeState
        self._LowState = LowState
        self._Request = Request
        self._RosNode = Node

    def _ensure_rclpy_init(self) -> None:
        if self._rclpy.ok():
            return
        self._rclpy.init()
        self._we_inited_rclpy = True

    def _spin_loop(self) -> None:
        try:
            self._spin_ok = True
            while not self._stop_event.is_set() and self._rclpy.ok():
                self._rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception as exc:
            self._spin_error = str(exc)
            print(f"[go2_bridge] spin failed: {exc}")
        finally:
            self._spin_ok = False


def create_go2_bridge_if_enabled(
    cfg: Go2HardwareConfig,
    *,
    use_go2: bool,
) -> Optional[UnitreeRos2Bridge]:
    if not cfg.is_active(use_go2=use_go2):
        return None
    return UnitreeRos2Bridge(cfg)
