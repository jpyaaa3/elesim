"""Process-isolated, latest-only Genesis camera rendering.

The physics Scene must not be rendered from a helper thread: Genesis owns the
Scene and its device context, and the synchronous camera API can force a GPU
readback.  This module therefore owns a small visual-only Scene in a spawned
process.  The parent sends bounded state snapshots and receives frame metadata
through latest-only queues; pixel buffers stay in fixed shared-memory slots.

The module deliberately has no top-level Genesis import.  This keeps the
parent-side scheduler usable by unit tests and prevents a failed render worker
from making the Sim control process un-importable.
"""

from __future__ import annotations

from dataclasses import dataclass
import multiprocessing as mp
from multiprocessing.queues import Queue
import queue
import threading
import time
from typing import Any, Callable, Mapping, Optional

import numpy as np

from elesim_sim.vision.sim_camera.types import SimCameraFrame, SimCameraIntrinsics


@dataclass(frozen=True)
class CameraRenderSpec:
    """Immutable recipe used to construct the visual-only Genesis Scene."""

    urdf_path: str
    robot_pos: tuple[float, float, float]
    robot_euler_deg: tuple[float, float, float]
    requires_jac_and_ik: bool
    use_gpu: bool
    gpu_convert: bool
    dt: float
    gravity: tuple[float, float, float]
    substeps: int
    floor: bool
    hand_eye_config: str = ""
    hand_eye_width: int = 640
    hand_eye_height: int = 480
    hand_eye_fov_deg: float = 60.0
    hand_eye_rgb: bool = True
    hand_eye_depth: bool = True
    observer_width: int = 640
    observer_height: int = 480
    observer_fov_deg: float = 40.0
    observer_pos: tuple[float, float, float] = (3.5, 0.5, 2.5)
    observer_lookat: tuple[float, float, float] = (0.0, 0.0, 0.5)
    mock_assets: tuple[str, ...] = ()
    target_enable: bool = False
    target_xyz: tuple[float, float, float] = (0.8, 0.0, 0.2)
    target_radius: float = 0.025
    target_color_rgba: tuple[float, float, float, float] = (0.85, 0.15, 0.15, 1.0)
    target_gravity: bool = False

    def __post_init__(self) -> None:
        if not str(self.urdf_path).strip():
            raise ValueError("camera render worker requires a URDF path")
        if int(self.hand_eye_width) <= 0 or int(self.hand_eye_height) <= 0:
            raise ValueError("hand-eye render dimensions must be positive")
        if int(self.observer_width) <= 0 or int(self.observer_height) <= 0:
            raise ValueError("observer render dimensions must be positive")


@dataclass(frozen=True)
class CameraStateSnapshot:
    """Small, serializable state sample applied by the render process."""

    epoch: int
    sim_step: int
    sim_time_s: float
    arm_q: Optional[tuple[float, float, float, float]] = None
    robot_q: Optional[tuple[float, ...]] = None
    robot_q_indices: Optional[tuple[int, ...]] = None
    root_pos: Optional[tuple[float, float, float]] = None
    root_quat_wxyz: Optional[tuple[float, float, float, float]] = None
    mock_asset_id: str = ""
    mock_position: Optional[tuple[float, float, float]] = None
    mock_euler_deg: Optional[tuple[float, float, float]] = None
    target_position: Optional[tuple[float, float, float]] = None
    observer_pos: Optional[tuple[float, float, float]] = None
    observer_lookat: Optional[tuple[float, float, float]] = None


class SharedRgbdMailbox:
    """One fixed-size latest-only RGB-D slot shared by two processes."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        color: Any,
        depth: Any,
        lock: Any,
        sequence: Any,
        captured_at: Any,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self._color = color
        self._depth = depth
        self._lock = lock
        self._sequence = sequence
        self._captured_at = captured_at

    @classmethod
    def create(
        cls,
        context: mp.context.BaseContext,
        *,
        width: int,
        height: int,
    ) -> "SharedRgbdMailbox":
        width_i = int(width)
        height_i = int(height)
        if width_i <= 0 or height_i <= 0:
            raise ValueError("RGB-D mailbox dimensions must be positive")
        return cls(
            width=width_i,
            height=height_i,
            color=context.RawArray("B", width_i * height_i * 3),
            depth=context.RawArray("H", width_i * height_i),
            lock=context.Lock(),
            sequence=context.Value("Q", 0),
            captured_at=context.Value("d", 0.0),
        )

    @property
    def color_shape(self) -> tuple[int, int, int]:
        return self.height, self.width, 3

    @property
    def depth_shape(self) -> tuple[int, int]:
        return self.height, self.width

    def publish(
        self,
        color: np.ndarray,
        depth: np.ndarray,
        *,
        captured_at: float,
    ) -> int:
        color_arr = np.asarray(color)
        depth_arr = np.asarray(depth)
        if color_arr.dtype != np.uint8 or color_arr.shape != self.color_shape:
            raise ValueError(
                f"RGB mailbox requires uint8 {self.color_shape}, "
                f"got {color_arr.dtype} {tuple(color_arr.shape)}"
            )
        if depth_arr.dtype != np.uint16 or depth_arr.shape != self.depth_shape:
            raise ValueError(
                f"depth mailbox requires uint16 {self.depth_shape}, "
                f"got {depth_arr.dtype} {tuple(depth_arr.shape)}"
            )
        color_arr = np.ascontiguousarray(color_arr)
        depth_arr = np.ascontiguousarray(depth_arr)
        with self._lock:
            color_target = np.frombuffer(self._color, dtype=np.uint8).reshape(self.color_shape)
            depth_target = np.frombuffer(self._depth, dtype=np.uint16).reshape(self.depth_shape)
            np.copyto(color_target, color_arr, casting="no")
            np.copyto(depth_target, depth_arr, casting="no")
            self._sequence.value += 1
            self._captured_at.value = float(captured_at)
            return int(self._sequence.value)

    def latest(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray], int, float]:
        with self._lock:
            sequence = int(self._sequence.value)
            captured_at = float(self._captured_at.value)
            if sequence <= 0:
                return None, None, 0, captured_at
            color = np.frombuffer(self._color, dtype=np.uint8).reshape(self.color_shape).copy()
            depth = np.frombuffer(self._depth, dtype=np.uint16).reshape(self.depth_shape).copy()
        return color, depth, sequence, captured_at


def _make_urdf_morph(
    gs: Any,
    urdf_path: str,
    pos: tuple[float, float, float],
    euler_deg: tuple[float, float, float],
    *,
    fixed: bool,
    requires_jac_and_ik: bool,
) -> Any:
    common = dict(
        file=str(urdf_path),
        pos=tuple(float(v) for v in pos),
        euler=tuple(float(v) for v in euler_deg),
        fixed=bool(fixed),
        prioritize_urdf_material=True,
        merge_fixed_links=not bool(requires_jac_and_ik),
        requires_jac_and_IK=bool(requires_jac_and_ik),
    )
    try:
        return gs.morphs.URDF(**common, default_armature=0.0)
    except TypeError:
        common.pop("requires_jac_and_IK", None)
        common.pop("merge_fixed_links", None)
        try:
            return gs.morphs.URDF(**common, default_armature=0.0)
        except TypeError:
            return gs.morphs.URDF(
                file=str(urdf_path),
                pos=tuple(float(v) for v in pos),
                euler=tuple(float(v) for v in euler_deg),
                fixed=bool(fixed),
            )


def _apply_snapshot(
    entity: Any,
    mock_entities: Mapping[str, Any],
    target_entity: Any,
    observer: Any,
    snapshot: CameraStateSnapshot,
) -> bool:
    """Apply one latest state and report whether the observer pose changed."""

    observer_pose_changed = False
    if snapshot.robot_q is not None:
        q = np.asarray(snapshot.robot_q, dtype=float).reshape(-1)
        try:
            if snapshot.robot_q_indices is None:
                entity.set_dofs_position(q)
            else:
                entity.set_dofs_position(
                    q,
                    dofs_idx_local=list(snapshot.robot_q_indices),
                )
        except Exception:
            # A render replica may have a different fixed-link DOF layout than
            # the physics entity.  Its visual arm remains useful if the shared
            # arm subset can still be applied.
            if snapshot.robot_q_indices is not None:
                try:
                    entity.set_dofs_position(
                        q,
                        dofs_idx_local=list(snapshot.robot_q_indices),
                    )
                except Exception:
                    pass
    if snapshot.root_pos is not None:
        try:
            entity.set_pos(np.asarray(snapshot.root_pos, dtype=float).reshape(3))
        except Exception:
            pass
    if snapshot.root_quat_wxyz is not None:
        try:
            entity.set_quat(np.asarray(snapshot.root_quat_wxyz, dtype=float).reshape(4))
        except Exception:
            pass

    active = str(snapshot.mock_asset_id).strip()
    position = snapshot.mock_position
    euler = snapshot.mock_euler_deg
    for name, mock in mock_entities.items():
        try:
            mock.set_pos(
                position
                if name == active and position is not None
                else (0.0, 0.0, -100.0)
            )
            if name == active and euler is not None:
                from scipy.spatial.transform import Rotation as Rot

                quat_xyzw = Rot.from_euler("xyz", euler, degrees=True).as_quat()
                mock.set_quat(
                    np.asarray(
                        [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                        dtype=float,
                    )
                )
        except Exception:
            pass

    if target_entity is not None and snapshot.target_position is not None:
        try:
            target_entity.set_pos(np.asarray(snapshot.target_position, dtype=float).reshape(3))
            zero_velocity = getattr(target_entity, "zero_all_dofs_velocity", None)
            if callable(zero_velocity):
                zero_velocity()
        except Exception:
            pass

    if observer is not None and (
        snapshot.observer_pos is not None or snapshot.observer_lookat is not None
    ):
        try:
            previous_pos = tuple(float(v) for v in observer.pos)
            previous_lookat = tuple(float(v) for v in observer.lookat)
            pos = snapshot.observer_pos or tuple(float(v) for v in observer.pos)
            lookat = snapshot.observer_lookat or tuple(float(v) for v in observer.lookat)
            observer.pos = tuple(float(v) for v in pos)
            observer.lookat = tuple(float(v) for v in lookat)
            observer_pose_changed = (
                observer.pos != previous_pos or observer.lookat != previous_lookat
            )
            observer._set_camera_pose()
        except Exception:
            pass
    return observer_pose_changed


def _camera_render_process_main(
    spec: CameraRenderSpec,
    streams: tuple[str, ...],
    mailboxes: Mapping[str, SharedRgbdMailbox],
    commands: Queue,
    results: Queue,
    ready: Any,
    stop: Any,
) -> None:
    """Spawn target; all Genesis imports and device work stay here."""

    try:
        import genesis as gs
        from elesim_sim.vision.sim_camera.mount import Node9EyeInHandCamera, ObserverCamera

        backend = gs.gpu if bool(spec.use_gpu) else gs.cpu
        init_kwargs = {"backend": backend, "logging_level": "warning"}
        try:
            if bool(spec.use_gpu):
                init_kwargs["performance_mode"] = True
            gs.init(**init_kwargs)
        except TypeError:
            init_kwargs.pop("performance_mode", None)
            gs.init(**init_kwargs)

        gravity = tuple(float(v) for v in spec.gravity)
        try:
            sim_options = gs.options.SimOptions(
                dt=float(spec.dt),
                gravity=gravity,
                substeps=int(spec.substeps),
            )
        except TypeError:
            sim_options = gs.options.SimOptions(dt=float(spec.dt), gravity=gravity)
        # This process is a visual replica, never a physics authority.  Turn
        # off collision/joint-limit construction so its startup does not
        # duplicate the expensive solver broad phase of the main scene.
        try:
            rigid_options = gs.options.RigidOptions(
                enable_collision=False,
                enable_self_collision=False,
                enable_joint_limit=False,
                disable_constraint=True,
            )
        except TypeError:
            rigid_options = None
        scene_kwargs = {
            "sim_options": sim_options,
            "viewer_options": gs.options.ViewerOptions(
                camera_pos=tuple(float(v) for v in spec.observer_pos),
                camera_lookat=tuple(float(v) for v in spec.observer_lookat),
                camera_fov=float(spec.observer_fov_deg),
                refresh_rate=1,
            ),
            "show_viewer": False,
        }
        if rigid_options is not None:
            scene_kwargs["rigid_options"] = rigid_options
        scene = gs.Scene(**scene_kwargs)
        if bool(spec.floor):
            scene.add_entity(gs.morphs.Plane())
        target_entity = None
        if bool(spec.target_enable):
            sphere_kwargs = {
                "radius": max(0.01, float(spec.target_radius)),
                "pos": tuple(float(v) for v in spec.target_xyz),
                "fixed": not bool(spec.target_gravity),
                # The target is visual-only in this process.  The authoritative
                # collision target remains in the physics scene.
                "collision": False,
            }
            try:
                sphere = gs.morphs.Sphere(**sphere_kwargs)
            except TypeError:
                sphere_kwargs.pop("collision", None)
                sphere = gs.morphs.Sphere(**sphere_kwargs)
            target_entity = scene.add_entity(
                sphere,
                surface=gs.surfaces.Rough(color=tuple(float(v) for v in spec.target_color_rgba)),
            )
        entity = scene.add_entity(
            _make_urdf_morph(
                gs,
                spec.urdf_path,
                spec.robot_pos,
                spec.robot_euler_deg,
                # Root pose is supplied by every snapshot, so a fixed visual
                # replica avoids allocating a dynamic base while retaining
                # the URDF joint DOFs used to pose the arm.
                fixed=True,
                requires_jac_and_ik=bool(spec.requires_jac_and_ik),
            )
        )

        mock_entities: dict[str, Any] = {}
        for asset in spec.mock_assets:
            path = str(asset)
            asset_id = path.rsplit("/", 1)[-1].rsplit("\\", 1)[-1].removesuffix(".obj")
            mock_entities[asset_id] = scene.add_entity(
                gs.morphs.Mesh(
                    file=path,
                    pos=(0.0, 0.0, -100.0),
                    fixed=True,
                    collision=False,
                )
            )

        eye = None
        if "hand_eye_preview" in streams and str(spec.hand_eye_config).strip():
            eye = Node9EyeInHandCamera.create(
                scene,
                res=(int(spec.hand_eye_width), int(spec.hand_eye_height)),
                fov_deg=float(spec.hand_eye_fov_deg),
            )
        observer = None
        if "observer" in streams:
            observer = ObserverCamera.create(
                scene,
                res=(int(spec.observer_width), int(spec.observer_height)),
                fov_deg=float(spec.observer_fov_deg),
                pos=tuple(float(v) for v in spec.observer_pos),
                lookat=tuple(float(v) for v in spec.observer_lookat),
            )
        scene.build()
        if eye is not None:
            eye.bind(entity, hand_eye_path=str(spec.hand_eye_config))
        results.put({"type": "ready", "ok": True})
        ready.set()

        capture_logged: set[str] = set()
        error_logged: set[str] = set()

        while not stop.is_set():
            try:
                command = commands.get(timeout=0.1)
            except queue.Empty:
                continue
            if command is None:
                break
            snapshot, requested = command
            if not isinstance(snapshot, CameraStateSnapshot):
                continue
            # Drain older snapshots.  A render worker never catches up on a
            # stale queue; it always applies the newest state available.
            requested_names = list(str(name) for name in requested)
            while True:
                try:
                    newer = commands.get_nowait()
                except queue.Empty:
                    break
                if newer is None:
                    stop.set()
                    break
                if isinstance(newer, tuple) and len(newer) == 2:
                    snapshot, requested = newer
                    requested_names.extend(str(name) for name in requested)
            if stop.is_set():
                break
            observer_pose_changed = _apply_snapshot(
                entity, mock_entities, target_entity, observer, snapshot
            )
            for stream in tuple(dict.fromkeys(requested_names)):
                try:
                    if stream not in capture_logged:
                        capture_logged.add(stream)
                        print(
                            f"[sim-camera-worker] capture start stream={stream}",
                            flush=True,
                        )
                    started = time.perf_counter()
                    if stream == "hand_eye_preview" and eye is not None:
                        frame = eye.capture(
                            arm_q=snapshot.arm_q,
                            rgb_enabled=bool(spec.hand_eye_rgb),
                            depth_enabled=bool(spec.hand_eye_depth),
                            prefer_gpu=bool(spec.gpu_convert),
                        )
                    elif stream == "observer" and observer is not None:
                        frame = observer.capture(
                            rgb_enabled=True,
                            depth_enabled=False,
                            prefer_gpu=bool(spec.gpu_convert),
                            force_render=observer_pose_changed,
                        )
                    else:
                        continue
                    mailbox = mailboxes[stream]
                    render_ms = 1000.0 * max(0.0, time.perf_counter() - started)
                    sequence = mailbox.publish(
                        np.asarray(frame.color_bgr, dtype=np.uint8),
                        np.asarray(frame.depth_raw, dtype=np.uint16),
                        captured_at=float(frame.ts),
                    )
                    results.put_nowait(
                        {
                            "type": "frame",
                            "stream": stream,
                            "sequence": int(sequence),
                            "epoch": int(snapshot.epoch),
                            "step": int(snapshot.sim_step),
                            "sim_time_s": float(snapshot.sim_time_s),
                            "ts": float(frame.ts),
                            "render_ms": render_ms,
                            "depth_scale": float(frame.depth_scale),
                            "intrinsics": (
                                float(frame.intrinsics.fx),
                                float(frame.intrinsics.fy),
                                float(frame.intrinsics.cx),
                                float(frame.intrinsics.cy),
                                int(frame.intrinsics.width),
                                int(frame.intrinsics.height),
                            ),
                            "arm_q": snapshot.arm_q,
                            "camera_world_origin": frame.camera_world_origin,
                            "camera_world_look": frame.camera_world_look,
                            "camera_world_right": frame.camera_world_right,
                        }
                    )
                except queue.Full:
                    # The mailbox already contains the newest pixels; the
                    # parent only needs the newest metadata notification.
                    pass
                except Exception as exc:
                    if stream not in error_logged:
                        error_logged.add(stream)
                        print(
                            f"[sim-camera-worker] capture failed stream={stream}: {exc}",
                            flush=True,
                        )
                    try:
                        results.put_nowait(
                            {
                                "type": "error",
                                "stream": stream,
                                "error": str(exc)[:512] or type(exc).__name__,
                            }
                        )
                    except queue.Full:
                        pass
    except BaseException as exc:
        try:
            results.put_nowait({"type": "ready", "ok": False, "error": str(exc)[:512]})
        except Exception:
            pass
    finally:
        ready.set()


class CameraRenderWorker:
    """Parent-side non-blocking proxy for the visual-only render process."""

    def __init__(
        self,
        spec: CameraRenderSpec,
        streams: Mapping[str, tuple[int, int]],
        on_frame: Callable[[str, SimCameraFrame], None],
        *,
        context: Optional[mp.context.BaseContext] = None,
    ) -> None:
        if not streams:
            raise ValueError("camera render worker requires at least one stream")
        if not callable(on_frame):
            raise TypeError("camera render worker on_frame must be callable")
        self.spec = spec
        self.streams = tuple(str(name) for name in streams)
        self._context = context or mp.get_context("spawn")
        self.mailboxes = {
            name: SharedRgbdMailbox.create(
                self._context,
                width=int(size[0]),
                height=int(size[1]),
            )
            for name, size in streams.items()
        }
        self._commands = self._context.Queue(maxsize=2)
        self._results = self._context.Queue(maxsize=32)
        self._ready = self._context.Event()
        self._stop = self._context.Event()
        self._process = self._context.Process(
            target=_camera_render_process_main,
            args=(
                self.spec,
                self.streams,
                self.mailboxes,
                self._commands,
                self._results,
                self._ready,
                self._stop,
            ),
            name="elesim-sim-camera-render",
            daemon=True,
        )
        self._on_frame = on_frame
        self._receiver: Optional[threading.Thread] = None
        self._started = False
        self._closed = False
        self._ready_ok = threading.Event()
        self._failure = ""
        self._epoch = 0
        self._submitted = 0
        self._dropped = 0
        self._completed = {name: 0 for name in self.streams}
        self._last_sequence = {name: 0 for name in self.streams}
        self._render_count = {name: 0 for name in self.streams}
        self._render_sum_ms = {name: 0.0 for name in self.streams}
        self._render_max_ms = {name: 0.0 for name in self.streams}
        self._lock = threading.Lock()

    @property
    def process(self) -> mp.Process:
        return self._process

    @property
    def ready(self) -> bool:
        return bool(self._ready_ok.is_set() and self._process.is_alive())

    @property
    def failure(self) -> str:
        with self._lock:
            return str(self._failure)

    def start(self, *, timeout_s: float = 180.0, wait: bool = True) -> None:
        if self._started:
            if not self.ready:
                raise RuntimeError(self.failure or "camera render worker is not ready")
            return
        self._process.start()
        self._started = True
        self._receiver = threading.Thread(
            target=self._receive_loop,
            name="sim-camera-render-receiver",
            daemon=True,
        )
        self._receiver.start()
        if not bool(wait):
            return
        if not self.wait_ready(timeout_s=float(timeout_s)):
            self._failure = "camera render worker startup timed out"
            self.close()
            raise RuntimeError(self._failure)
        if not self.ready:
            failure = self.failure or "camera render worker failed during startup"
            self.close()
            raise RuntimeError(failure)

    def wait_ready(self, *, timeout_s: float = 180.0) -> bool:
        """Wait for scene construction; safe to call outside the physics loop."""

        if not self._started:
            return False
        if not self._ready_ok.wait(max(0.1, float(timeout_s))):
            with self._lock:
                self._failure = "camera render worker startup timed out"
            return False
        return bool(self.ready)

    def submit(self, snapshot: CameraStateSnapshot, requested: tuple[str, ...]) -> bool:
        if not self.ready or self._closed:
            return False
        names = tuple(name for name in requested if name in self.mailboxes)
        if not names:
            return False
        try:
            self._commands.put_nowait((snapshot, names))
        except queue.Full:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                pass
            try:
                self._commands.put_nowait((snapshot, names))
            except queue.Full:
                with self._lock:
                    self._dropped += 1
                return False
            with self._lock:
                self._dropped += 1
        with self._lock:
            self._submitted += 1
            self._epoch = max(self._epoch, int(snapshot.epoch))
        return True

    def bump_epoch(self) -> int:
        with self._lock:
            self._epoch += 1
            return self._epoch

    def wait_for_frame(self, stream: str, *, timeout_s: float = 2.0) -> bool:
        name = str(stream)
        deadline = time.monotonic() + max(0.01, float(timeout_s))
        while time.monotonic() < deadline:
            with self._lock:
                if int(self._completed.get(name, 0)) > 0:
                    return True
            if not self._process.is_alive():
                return False
            time.sleep(0.005)
        with self._lock:
            return int(self._completed.get(name, 0)) > 0

    def diagnostics(self) -> dict[str, Any]:
        with self._lock:
            return {
                "ready": self.ready,
                "alive": bool(self._process.is_alive()) if self._started else False,
                "failure": self._failure,
                "submitted": int(self._submitted),
                "dropped": int(self._dropped),
                "completed": dict(self._completed),
                "last_sequence": dict(self._last_sequence),
                "render_avg_ms": {
                    name: (
                        float(self._render_sum_ms[name]) / max(1, int(self._render_count[name]))
                    )
                    for name in self.streams
                },
                "render_max_ms": dict(self._render_max_ms),
                "epoch": int(self._epoch),
            }

    def close(self, *, timeout_s: float = 3.0) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        try:
            self._commands.put_nowait(None)
        except Exception:
            pass
        if self._started and self._process.is_alive():
            self._process.join(max(0.1, float(timeout_s)))
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(1.0)
        if self._receiver is not None:
            self._receiver.join(1.0)
        for channel in (self._commands, self._results):
            try:
                channel.close()
                channel.join_thread()
            except Exception:
                pass
        self._started = False

    def _receive_loop(self) -> None:
        while not self._closed:
            try:
                message = self._results.get(timeout=0.1)
            except queue.Empty:
                if self._started and not self._process.is_alive() and not self._ready_ok.is_set():
                    with self._lock:
                        self._failure = "camera render worker exited during startup"
                    self._ready_ok.set()
                continue
            if not isinstance(message, dict):
                continue
            kind = str(message.get("type", ""))
            if kind == "ready":
                if not bool(message.get("ok", False)):
                    with self._lock:
                        self._failure = str(
                            message.get("error")
                            or "camera render worker startup failed"
                        )
                self._ready_ok.set()
                continue
            if kind == "error":
                with self._lock:
                    self._failure = str(message.get("error") or "camera render failed")[:512]
                continue
            if kind != "frame":
                continue
            stream = str(message.get("stream", ""))
            mailbox = self.mailboxes.get(stream)
            if mailbox is None:
                continue
            message_epoch = int(message.get("epoch", 0))
            with self._lock:
                current_epoch = int(self._epoch)
            if message_epoch < current_epoch:
                continue
            color, depth, sequence, captured_at = mailbox.latest()
            expected = int(message.get("sequence", 0))
            if color is None or depth is None or sequence != expected:
                # A newer frame overwrote this metadata before the receiver
                # copied it.  The next notification represents the latest
                # slot; dropping this stale one preserves coherence.
                continue
            intr = message.get("intrinsics")
            if not isinstance(intr, (tuple, list)) or len(intr) != 6:
                continue
            frame = SimCameraFrame(
                color_bgr=color,
                depth_raw=depth,
                depth_scale=float(message.get("depth_scale", 0.001)),
                intrinsics=SimCameraIntrinsics(
                    fx=float(intr[0]),
                    fy=float(intr[1]),
                    cx=float(intr[2]),
                    cy=float(intr[3]),
                    width=int(intr[4]),
                    height=int(intr[5]),
                ),
                seq=sequence,
                ts=float(message.get("ts", captured_at)),
                arm_q=message.get("arm_q"),
                camera_world_origin=message.get("camera_world_origin"),
                camera_world_look=message.get("camera_world_look"),
                camera_world_right=message.get("camera_world_right"),
            )
            with self._lock:
                self._completed[stream] = int(self._completed.get(stream, 0)) + 1
                self._last_sequence[stream] = sequence
                render_ms = max(0.0, float(message.get("render_ms", 0.0)))
                self._render_count[stream] = int(self._render_count.get(stream, 0)) + 1
                self._render_sum_ms[stream] = (
                    float(self._render_sum_ms.get(stream, 0.0)) + render_ms
                )
                self._render_max_ms[stream] = max(
                    float(self._render_max_ms.get(stream, 0.0)), render_ms
                )
            try:
                self._on_frame(stream, frame)
            except Exception as exc:
                with self._lock:
                    self._failure = f"frame callback: {str(exc)[:480]}"


__all__ = [
    "CameraRenderSpec",
    "CameraStateSnapshot",
    "CameraRenderWorker",
    "SharedRgbdMailbox",
]
