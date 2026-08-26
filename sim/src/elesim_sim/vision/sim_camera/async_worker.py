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

from dataclasses import dataclass, replace
import multiprocessing as mp
from multiprocessing.queues import Queue
import queue
import threading
import time
import xml.etree.ElementTree as ET
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
    robot_joint_names: tuple[str, ...] = ()
    # The hand-eye camera only needs the arm visual tree.  When a GO2+arm
    # runtime is active, using the combined URDF for both cameras makes a
    # cold visual scene unnecessarily large and couples hand-eye startup to
    # the observer's full robot scene.
    hand_eye_urdf_path: str = ""
    hand_eye_robot_pos: Optional[tuple[float, float, float]] = None
    hand_eye_robot_euler_deg: Optional[tuple[float, float, float]] = None
    hand_eye_robot_joint_names: tuple[str, ...] = ()

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
    robot_joint_positions: Optional[tuple[float, ...]] = None
    root_pos: Optional[tuple[float, float, float]] = None
    root_quat_wxyz: Optional[tuple[float, float, float, float]] = None
    hand_eye_robot_joint_positions: Optional[tuple[float, ...]] = None
    hand_eye_root_pos: Optional[tuple[float, float, float]] = None
    hand_eye_root_quat_wxyz: Optional[tuple[float, float, float, float]] = None
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
        # The camera replica is never stepped and is never a collision
        # authority.  Loading the URDF collision meshes here needlessly
        # duplicates the largest part of the physical scene and, on Genesis
        # versions that still build collision metadata when the rigid solver
        # is disabled, makes the first camera frame take tens of seconds.
        collision=False,
        prioritize_urdf_material=True,
        merge_fixed_links=not bool(requires_jac_and_ik),
        requires_jac_and_IK=bool(requires_jac_and_ik),
        default_armature=0.0,
    )
    return gs.morphs.URDF(**common)


def _genesis_init_kwargs(gs: Any, *, use_gpu: bool) -> dict[str, Any]:
    """Use dynamic arrays for the visual replica to minimize cold startup.

    The authoritative physics process keeps ``performance_mode=True`` for
    steady-state stepping.  The camera replica never steps physics, so paying
    for a second scene-specific static-kernel compilation only delays startup
    and competes with the physics build on the same GPU.
    """

    return {
        "backend": gs.gpu if bool(use_gpu) else gs.cpu,
        "logging_level": "warning",
    }


def movable_urdf_joint_names(urdf_path: str) -> tuple[str, ...]:
    """Return the stable one-DOF joint order shared by both Genesis scenes."""

    root = ET.parse(str(urdf_path)).getroot()
    names = tuple(
        str(joint.attrib.get("name", "")).strip()
        for joint in root.findall(".//joint")
        if str(joint.attrib.get("type", "fixed")).strip().lower() != "fixed"
    )
    if not names or any(not name for name in names) or len(set(names)) != len(names):
        raise ValueError("camera render URDF must contain unique named movable joints")
    return names


def resolve_single_dof_indices(
    entity: Any, joint_names: tuple[str, ...]
) -> tuple[int, ...]:
    """Resolve named URDF joints without assuming equal floating-base layouts."""

    indices: list[int] = []
    for name in joint_names:
        joint = entity.get_joint(name)
        raw = np.asarray(joint.dofs_idx_local, dtype=int).reshape(-1)
        if raw.size != 1:
            raise RuntimeError(
                f"camera render joint '{name}' has {raw.size} DOFs; exactly one is required"
            )
        indices.append(int(raw[0]))
    return tuple(indices)


def _apply_snapshot(
    entity: Any,
    robot_dof_indices: tuple[int, ...],
    mock_entities: Mapping[str, Any],
    target_entity: Any,
    observer: Any,
    snapshot: CameraStateSnapshot,
    *,
    joint_positions: Optional[tuple[float, ...]] = None,
) -> bool:
    """Apply one latest state and report whether the observer pose changed."""

    observer_pose_changed = False
    positions = (
        snapshot.robot_joint_positions
        if joint_positions is None
        else joint_positions
    )
    if positions is not None:
        q = np.asarray(positions, dtype=float).reshape(-1)
        if q.size != len(robot_dof_indices):
            raise RuntimeError(
                "camera render joint snapshot size mismatch: "
                f"{q.size} values for {len(robot_dof_indices)} joints"
            )
        entity.set_dofs_position(q, dofs_idx_local=list(robot_dof_indices))
    if snapshot.root_pos is not None:
        entity.set_pos(np.asarray(snapshot.root_pos, dtype=float).reshape(3))
    if snapshot.root_quat_wxyz is not None:
        entity.set_quat(np.asarray(snapshot.root_quat_wxyz, dtype=float).reshape(4))

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
    frame_results: Mapping[str, Queue],
    ready: Any,
    stop: Any,
) -> None:
    """Spawn target; all Genesis imports and device work stay here."""

    try:
        import genesis as gs
        from elesim_sim.vision.sim_camera.mount import Node9EyeInHandCamera, ObserverCamera

        gs.init(**_genesis_init_kwargs(gs, use_gpu=bool(spec.use_gpu)))

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
        robot_dof_indices = resolve_single_dof_indices(entity, spec.robot_joint_names)
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
            hand_eye_only = (
                "hand_eye_preview" in streams and "observer" not in streams
            )
            applied_snapshot = snapshot
            joint_positions = snapshot.robot_joint_positions
            if hand_eye_only:
                if snapshot.hand_eye_robot_joint_positions is not None:
                    joint_positions = snapshot.hand_eye_robot_joint_positions
                if snapshot.hand_eye_root_pos is not None:
                    applied_snapshot = replace(
                        applied_snapshot,
                        root_pos=snapshot.hand_eye_root_pos,
                    )
                if snapshot.hand_eye_root_quat_wxyz is not None:
                    applied_snapshot = replace(
                        applied_snapshot,
                        root_quat_wxyz=snapshot.hand_eye_root_quat_wxyz,
                    )
            _apply_snapshot(
                entity,
                robot_dof_indices,
                mock_entities,
                target_entity,
                observer,
                applied_snapshot,
                joint_positions=joint_positions,
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
                            force_render=True,
                        )
                    elif stream == "observer" and observer is not None:
                        frame = observer.capture(
                            rgb_enabled=True,
                            depth_enabled=False,
                            prefer_gpu=bool(spec.gpu_convert),
                            force_render=True,
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
                    _put_latest_frame_result(
                        frame_results[stream],
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
                        },
                    )
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


def _put_latest_frame_result(channel: Queue, message: Mapping[str, Any]) -> bool:
    """Keep one latest metadata notification beside a latest pixel mailbox.

    Pixels and their metadata must advance together.  Dropping the newest
    notification when a shared-memory slot has already advanced leaves the
    receiver with only stale sequence numbers; under sustained rendering that
    can starve publication until rendering pauses.  Each stream has its own
    one-item queue so replacing an old notification cannot starve the other
    camera stream.
    """

    value = dict(message)
    try:
        channel.put_nowait(value)
        return True
    except queue.Full:
        pass
    try:
        channel.get_nowait()
    except queue.Empty:
        # multiprocessing.Queue may report the full semaphore just before its
        # feeder makes the item readable.  A later frame will retry without
        # ever blocking the render process.
        return False
    try:
        channel.put_nowait(value)
        return True
    except queue.Full:
        return False


@dataclass
class _CameraProcessState:
    """IPC state for one independent camera renderer."""

    stream: str
    commands: Queue
    results: Queue
    frame_results: Queue
    ready: Any
    stop: Any
    process: Any


class CameraRenderWorker:
    """Parent-side proxy for independent latest-only camera processes.

    Genesis does not provide a safe way to render two cameras concurrently
    from one scene/context.  Keeping one process per stream prevents a slow
    hand-eye RGB-D pass from starving the observer pass (and vice versa).
    Each process still has its own two-item latest-only command queue.
    """

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
        self._processes: dict[str, _CameraProcessState] = {}
        for name in self.streams:
            stream_spec = self._spec_for_stream(name)
            commands = self._context.Queue(maxsize=2)
            results = self._context.Queue(maxsize=32)
            frame_results = self._context.Queue(maxsize=1)
            ready = self._context.Event()
            stop = self._context.Event()
            process = self._context.Process(
                target=_camera_render_process_main,
                args=(
                    stream_spec,
                    (name,),
                    {name: self.mailboxes[name]},
                    commands,
                    results,
                    {name: frame_results},
                    ready,
                    stop,
                ),
                name=f"elesim-sim-camera-{name}",
                daemon=True,
            )
            self._processes[name] = _CameraProcessState(
                stream=name,
                commands=commands,
                results=results,
                frame_results=frame_results,
                ready=ready,
                stop=stop,
                process=process,
            )
        self._on_frame = on_frame
        self._receiver: Optional[threading.Thread] = None
        self._started = False
        self._closed = False
        self._ready_ok = threading.Event()
        self._ready_streams: set[str] = set()
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

    def _spec_for_stream(self, stream: str) -> CameraRenderSpec:
        """Return the smallest visual scene needed by ``stream``."""

        name = str(stream)
        if name != "hand_eye_preview" or not str(self.spec.hand_eye_urdf_path).strip():
            return self.spec
        path = str(self.spec.hand_eye_urdf_path)
        names = tuple(self.spec.hand_eye_robot_joint_names)
        if not names:
            names = movable_urdf_joint_names(path)
        return replace(
            self.spec,
            urdf_path=path,
            robot_pos=(
                tuple(float(v) for v in self.spec.hand_eye_robot_pos)
                if self.spec.hand_eye_robot_pos is not None
                else self.spec.robot_pos
            ),
            robot_euler_deg=(
                tuple(float(v) for v in self.spec.hand_eye_robot_euler_deg)
                if self.spec.hand_eye_robot_euler_deg is not None
                else self.spec.robot_euler_deg
            ),
            robot_joint_names=names,
            # The hand-eye view is attached to the arm, but it must retain the
            # same visual world as the authoritative scene.  In particular,
            # removing the floor/target here produces a perfectly valid
            # constant clear-colour frame, which looks like a dead camera and
            # makes perception impossible.  The arm-only optimization is
            # still useful because it omits the GO2 body and legs; shared
            # floor, target and mock meshes remain part of the view.
        )

    @property
    def process(self) -> mp.Process:
        # Backwards-compatible representative process.  New callers that
        # need lifecycle detail should use ``processes``/diagnostics; there
        # can now be one process per stream.
        return next(iter(self._processes.values())).process

    @property
    def processes(self) -> Mapping[str, mp.Process]:
        return {name: state.process for name, state in self._processes.items()}

    @property
    def ready(self) -> bool:
        return bool(
            self._ready_ok.is_set()
            and all(state.process.is_alive() for state in self._processes.values())
        )

    @property
    def failure(self) -> str:
        with self._lock:
            return str(self._failure)

    def start(self, *, timeout_s: float = 180.0, wait: bool = True) -> None:
        if self._started:
            if bool(wait) and not self.ready:
                raise RuntimeError(self.failure or "camera render worker is not ready")
            return
        for state in self._processes.values():
            state.process.start()
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
        if self._closed:
            return False
        names = tuple(name for name in requested if name in self.mailboxes)
        if not names:
            return False
        accepted = False
        dropped_any = False
        for name in names:
            state = self._processes[name]
            if not self._stream_ready(name):
                continue
            try:
                state.commands.put_nowait((snapshot, (name,)))
                accepted = True
            except queue.Full:
                try:
                    state.commands.get_nowait()
                except queue.Empty:
                    pass
                try:
                    state.commands.put_nowait((snapshot, (name,)))
                    accepted = True
                    dropped_any = True
                except queue.Full:
                    dropped_any = True
        with self._lock:
            if accepted:
                self._submitted += 1
            if dropped_any:
                self._dropped += 1
            self._epoch = max(self._epoch, int(snapshot.epoch))
        return accepted

    def _stream_ready(self, stream: str) -> bool:
        state = self._processes.get(str(stream))
        return bool(
            state is not None
            and state.ready.is_set()
            and state.process.is_alive()
            and not state.stop.is_set()
        )

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
            if not self._stream_ready(name):
                return False
            time.sleep(0.005)
        with self._lock:
            return int(self._completed.get(name, 0)) > 0

    def diagnostics(self) -> dict[str, Any]:
        with self._lock:
            return {
                "ready": self.ready,
                # Keep the historical aggregate boolean for callers that use
                # this diagnostics field as a health check.  Per-stream
                # lifecycle is exposed separately now that each camera has
                # its own process.
                "alive": bool(
                    self._started
                    and all(state.process.is_alive() for state in self._processes.values())
                ),
                "alive_streams": {
                    name: bool(state.process.is_alive()) if self._started else False
                    for name, state in self._processes.items()
                },
                "ready_streams": tuple(sorted(self._ready_streams)),
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
        for state in self._processes.values():
            state.stop.set()
            try:
                state.commands.put_nowait(None)
            except Exception:
                pass
        for state in self._processes.values():
            if self._started and state.process.is_alive():
                state.process.join(max(0.1, float(timeout_s)))
            if state.process.is_alive():
                state.process.terminate()
                state.process.join(1.0)
        if self._receiver is not None:
            self._receiver.join(1.0)
        channels = []
        for state in self._processes.values():
            channels.extend((state.commands, state.results, state.frame_results))
        for channel in channels:
            try:
                channel.close()
                channel.join_thread()
            except Exception:
                pass
        self._started = False

    def _receive_loop(self) -> None:
        while not self._closed:
            messages: list[tuple[str, Any]] = []
            for stream, state in self._processes.items():
                try:
                    messages.append((stream, state.results.get_nowait()))
                except queue.Empty:
                    pass
                try:
                    messages.append((stream, state.frame_results.get_nowait()))
                except queue.Empty:
                    pass
            if not messages:
                dead_before_ready = (
                    self._started
                    and any(
                        not state.process.is_alive()
                        and not state.ready.is_set()
                        for state in self._processes.values()
                    )
                    and not self._ready_ok.is_set()
                )
                if dead_before_ready:
                    with self._lock:
                        self._failure = "camera render worker exited during startup"
                    self._ready_ok.set()
                time.sleep(0.005)
                continue
            for stream, message in messages:
                self._receive_message(message, stream_hint=stream)

    def _receive_message(self, message: Any, *, stream_hint: str = "") -> None:
        if not isinstance(message, dict):
            return
        kind = str(message.get("type", ""))
        if kind == "ready":
            if not bool(message.get("ok", False)):
                with self._lock:
                    self._failure = str(
                        message.get("error") or "camera render worker startup failed"
                    )
                self._ready_ok.set()
                return
            if stream_hint:
                with self._lock:
                    self._ready_streams.add(str(stream_hint))
                    all_ready = self._ready_streams == set(self.streams)
                if all_ready:
                    self._ready_ok.set()
            else:
                self._ready_ok.set()
            return
        if kind == "error":
            with self._lock:
                self._failure = str(message.get("error") or "camera render failed")[:512]
            return
        if kind != "frame":
            return
        stream = str(message.get("stream", ""))
        mailbox = self.mailboxes.get(stream)
        if mailbox is None:
            return
        message_epoch = int(message.get("epoch", 0))
        with self._lock:
            current_epoch = int(self._epoch)
        if message_epoch < current_epoch:
            return
        color, depth, sequence, captured_at = mailbox.latest()
        expected = int(message.get("sequence", 0))
        if color is None or depth is None or sequence != expected:
            # A newer frame overwrote this metadata before the receiver
            # copied it.  The next notification represents the latest slot;
            # dropping this stale one preserves coherence.
            return
        intr = message.get("intrinsics")
        if not isinstance(intr, (tuple, list)) or len(intr) != 6:
            return
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
    "movable_urdf_joint_names",
    "resolve_single_dof_indices",
]
