from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np

from elesim_simulator.vision.sim_camera.calibration import load_hand_eye_transform
from elesim_simulator.vision.sim_camera.types import SimCameraFrame, SimCameraIntrinsics

# RealSense optical (+X right, +Y down, +Z look) -> Genesis/OpenGL camera (+X right, +Y up, -Z look).
_OPTICAL_TO_GENESIS_CAMERA = np.diag([1.0, -1.0, -1.0, 1.0]).astype(np.float64)
_OPTICAL_FROM_GENESIS_CAMERA = np.linalg.inv(_OPTICAL_TO_GENESIS_CAMERA)


def load_hand_eye_offset_T(hand_eye_path: str | Path) -> np.ndarray:
    T, _meta = load_hand_eye_transform(hand_eye_path)
    return T


def hand_eye_to_genesis_attach_T(hand_eye_path: str | Path) -> np.ndarray:
    """Map hand-eye optical extrinsics to Genesis ``camera.attach`` offset."""
    return load_hand_eye_offset_T(hand_eye_path) @ _OPTICAL_TO_GENESIS_CAMERA


def intrinsics_from_fov(*, width: int, height: int, fov_deg: float) -> SimCameraIntrinsics:
    w = int(width)
    h = int(height)
    fov = float(np.radians(fov_deg))
    fx = (w * 0.5) / max(np.tan(fov * 0.5), 1e-6)
    fy = fx
    return SimCameraIntrinsics(fx=float(fx), fy=float(fy), cx=w * 0.5, cy=h * 0.5, width=w, height=h)


@dataclass
class Node9EyeInHandCamera:
    """Genesis debug camera attached to arm node9 (hand-eye extrinsics)."""

    camera: Any
    intrinsics: SimCameraIntrinsics
    depth_scale: float = 0.001
    _seq: int = 0
    _arm_entity: Any = None
    _hand_eye_path: str = ""
    _parent_link: str = "node9"

    @classmethod
    def create(
        cls,
        scene,
        *,
        res: tuple[int, int] = (640, 480),
        fov_deg: float = 60.0,
    ) -> Node9EyeInHandCamera:
        """Register camera before ``scene.build()``."""
        w, h = int(res[0]), int(res[1])
        camera = scene.add_camera(
            res=(w, h),
            fov=float(fov_deg),
            GUI=False,
            debug=False,
        )
        intr = intrinsics_from_fov(width=w, height=h, fov_deg=fov_deg)
        return cls(camera=camera, intrinsics=intr)

    def bind(
        self,
        arm_entity,
        *,
        hand_eye_path: str,
        parent_link: str = "node9",
    ) -> None:
        """Attach to arm link after ``scene.build()``."""
        link = arm_entity.get_link(str(parent_link))
        offset_T = hand_eye_to_genesis_attach_T(hand_eye_path)
        self.camera.attach(link, offset_T)
        self._arm_entity = arm_entity
        self._hand_eye_path = str(hand_eye_path)
        self._parent_link = str(parent_link)
        print(
            f"[sim_camera] attached to {parent_link} | res={self.intrinsics.width}x{self.intrinsics.height} "
            f"from {hand_eye_path}"
        )

    def camera_axes_world(self, *, axis_len_m: float = 0.08) -> Optional[tuple[tuple[float, float, float], ...]]:
        """Optical-frame origin / look / right from the live Genesis camera (matches render POV)."""
        try:
            from elesim_simulator.vision.sim_camera.pose import camera_axes_from_genesis_camera_object

            if hasattr(self.camera, "move_to_attach"):
                self.camera.move_to_attach()
            return camera_axes_from_genesis_camera_object(self.camera, axis_len_m=float(axis_len_m))
        except Exception as exc:
            print(f"[sim_camera] camera_axes_world failed: {exc}")
            return None

    def _camera_pose_world(self) -> Optional[tuple[tuple[float, float, float], ...]]:
        return self.camera_axes_world()

    @classmethod
    def attach(
        cls,
        scene,
        arm_entity,
        *,
        hand_eye_path: str,
        res: tuple[int, int] = (640, 480),
        fov_deg: float = 60.0,
        parent_link: str = "node9",
    ) -> Node9EyeInHandCamera:
        """Legacy helper when scene is not built yet (creates + binds immediately)."""
        cam = cls.create(scene, res=res, fov_deg=fov_deg)
        cam.bind(arm_entity, hand_eye_path=hand_eye_path, parent_link=parent_link)
        return cam

    def capture(
        self,
        *,
        arm_q: Optional[tuple[float, float, float, float]] = None,
        ts: Optional[float] = None,
        rgb_enabled: bool = True,
        depth_enabled: bool = True,
    ) -> SimCameraFrame:
        import time

        target_w = int(self.intrinsics.width)
        target_h = int(self.intrinsics.height)
        rgb = depth = None
        if bool(rgb_enabled) or bool(depth_enabled):
            rgb, depth, _, _ = self.camera.render(rgb=bool(rgb_enabled), depth=bool(depth_enabled))

        if bool(rgb_enabled) and rgb is not None:
            if hasattr(rgb, "cpu"):
                rgb = rgb.cpu().numpy()
            rgb_np = np.asarray(rgb)
            if rgb_np.dtype != np.uint8:
                rgb_f = rgb_np.astype(np.float32, copy=False)
                if float(np.nanmax(rgb_f)) <= 1.0:
                    rgb_np = np.clip(rgb_f * 255.0, 0.0, 255.0).astype(np.uint8)
                else:
                    rgb_np = np.clip(rgb_f, 0.0, 255.0).astype(np.uint8)
            if rgb_np.ndim == 3 and rgb_np.shape[-1] >= 3:
                color_bgr = np.ascontiguousarray(rgb_np[..., :3][:, :, ::-1], dtype=np.uint8)
            else:
                color_bgr = np.ascontiguousarray(rgb_np, dtype=np.uint8)
        else:
            color_bgr = np.zeros((target_h, target_w, 3), dtype=np.uint8)

        if bool(depth_enabled) and depth is not None:
            if hasattr(depth, "cpu"):
                depth = depth.cpu().numpy()
            depth_np = np.asarray(depth, dtype=float)
            if depth_np.ndim == 3:
                depth_np = depth_np[..., 0]
            depth_m = np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)
            depth_mm = np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)
        else:
            depth_mm = np.zeros((target_h, target_w), dtype=np.uint16)

        if color_bgr.shape[0] != target_h or color_bgr.shape[1] != target_w:
            import cv2

            color_bgr = cv2.resize(
                color_bgr,
                (target_w, target_h),
                interpolation=cv2.INTER_LINEAR,
            )
        if depth_mm.shape[0] != target_h or depth_mm.shape[1] != target_w:
            import cv2

            depth_mm = cv2.resize(
                depth_mm,
                (target_w, target_h),
                interpolation=cv2.INTER_NEAREST,
            )

        self._seq += 1
        cam_origin = cam_look = cam_right = None
        pose = self._camera_pose_world()
        if pose is not None:
            cam_origin, cam_look, cam_right = pose
        return SimCameraFrame(
            color_bgr=color_bgr,
            depth_raw=depth_mm,
            depth_scale=float(self.depth_scale),
            intrinsics=self.intrinsics,
            seq=int(self._seq),
            ts=float(time.time() if ts is None else ts),
            arm_q=arm_q,
            camera_world_origin=cam_origin,
            camera_world_look=cam_look,
            camera_world_right=cam_right,
        )


@dataclass
class ObserverCamera:
    """Operator-controlled Genesis camera for the remote scene view."""

    camera: Any
    intrinsics: SimCameraIntrinsics
    pos: tuple[float, float, float]
    lookat: tuple[float, float, float]
    depth_scale: float = 0.001
    _seq: int = 0
    _pose_warned: bool = False
    _reset_pos: Optional[tuple[float, float, float]] = None
    _reset_lookat: Optional[tuple[float, float, float]] = None

    @classmethod
    def create(
        cls,
        scene,
        *,
        res: tuple[int, int] = (960, 540),
        fov_deg: float = 55.0,
        pos: tuple[float, float, float] = (0.45, -1.8, 0.55),
        lookat: tuple[float, float, float] = (0.45, 0.0, 0.25),
    ) -> "ObserverCamera":
        """Register camera before ``scene.build()``."""
        w, h = int(res[0]), int(res[1])
        pos_t = tuple(float(x) for x in pos)
        lookat_t = tuple(float(x) for x in lookat)
        try:
            camera = scene.add_camera(
                res=(w, h),
                pos=pos_t,
                lookat=lookat_t,
                fov=float(fov_deg),
                GUI=False,
                debug=False,
            )
        except TypeError:
            camera = scene.add_camera(
                res=(w, h),
                fov=float(fov_deg),
                GUI=False,
                debug=False,
            )
        intr = intrinsics_from_fov(width=w, height=h, fov_deg=fov_deg)
        return cls(camera=camera, intrinsics=intr, pos=pos_t, lookat=lookat_t)

    def _set_camera_pose(self) -> None:
        if self._reset_pos is None:
            self._reset_pos = self.pos
            self._reset_lookat = self.lookat
        if not hasattr(self.camera, "set_pose"):
            return
        try:
            self.camera.set_pose(pos=self.pos, lookat=self.lookat)
            return
        except TypeError:
            try:
                self.camera.set_pose(self.pos, self.lookat)
                return
            except Exception as exc:
                if not self._pose_warned:
                    self._pose_warned = True
                    print(f"[sim_camera] observer pose update failed: {exc}")
        except Exception as exc:
            if not self._pose_warned:
                self._pose_warned = True
                print(f"[sim_camera] observer pose update failed: {exc}")

    def apply_operator_command(self, command: str, arguments: dict[str, Any]) -> None:
        """Apply a validated camera command on the Genesis owner thread."""

        import math

        if self._reset_pos is None:
            self._reset_pos = self.pos
            self._reset_lookat = self.lookat
        name = str(command)
        if name == "reset_view":
            self.pos = self._reset_pos
            self.lookat = self._reset_lookat or self.lookat
            self._set_camera_pose()
            return
        eye = np.asarray(self.pos, dtype=float)
        target = np.asarray(self.lookat, dtype=float)
        offset = eye - target
        radius = max(float(np.linalg.norm(offset)), 0.05)
        if name == "orbit":
            yaw = math.atan2(float(offset[1]), float(offset[0])) - float(arguments["dx"]) * math.pi
            pitch = math.asin(float(np.clip(offset[2] / radius, -0.98, 0.98)))
            pitch += float(arguments["dy"]) * math.pi * 0.5
            pitch = float(np.clip(pitch, -1.45, 1.45))
            offset = radius * np.array(
                [math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)]
            )
            eye = target + offset
        elif name == "zoom":
            radius = float(np.clip(radius * math.exp(float(arguments["delta"]) * 1.5), 0.08, 20.0))
            eye = target + offset / max(float(np.linalg.norm(offset)), 1e-9) * radius
        elif name == "pan":
            forward = target - eye
            forward /= max(float(np.linalg.norm(forward)), 1e-9)
            right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
            right /= max(float(np.linalg.norm(right)), 1e-9)
            up = np.cross(right, forward)
            shift = (-float(arguments["dx"]) * right + float(arguments["dy"]) * up) * radius
            eye += shift
            target += shift
        else:
            raise ValueError(f"unsupported observer camera command: {name}")
        self.pos = tuple(float(value) for value in eye)
        self.lookat = tuple(float(value) for value in target)
        self._set_camera_pose()

    def _camera_pose_world(self) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        origin = np.asarray(self.pos, dtype=float).reshape(3)
        target = np.asarray(self.lookat, dtype=float).reshape(3)
        look = target - origin
        look_norm = float(np.linalg.norm(look))
        if look_norm <= 1e-9:
            look = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            look = look / look_norm * 0.08
        up = np.array([0.0, 0.0, 1.0], dtype=float)
        right = np.cross(look, up)
        right_norm = float(np.linalg.norm(right))
        if right_norm <= 1e-9:
            right = np.array([0.08, 0.0, 0.0], dtype=float)
        else:
            right = right / right_norm * 0.08
        return (
            tuple(float(x) for x in origin),
            tuple(float(x) for x in look),
            tuple(float(x) for x in right),
        )

    def capture(
        self,
        *,
        ts: Optional[float] = None,
        rgb_enabled: bool = True,
        depth_enabled: bool = False,
    ) -> SimCameraFrame:
        import time

        target_w = int(self.intrinsics.width)
        target_h = int(self.intrinsics.height)
        self._set_camera_pose()
        rgb = depth = None
        if bool(rgb_enabled) or bool(depth_enabled):
            rgb, depth, _, _ = self.camera.render(rgb=bool(rgb_enabled), depth=bool(depth_enabled))

        if bool(rgb_enabled) and rgb is not None:
            if hasattr(rgb, "cpu"):
                rgb = rgb.cpu().numpy()
            rgb_np = np.asarray(rgb)
            if rgb_np.dtype != np.uint8:
                rgb_f = rgb_np.astype(np.float32, copy=False)
                if float(np.nanmax(rgb_f)) <= 1.0:
                    rgb_np = np.clip(rgb_f * 255.0, 0.0, 255.0).astype(np.uint8)
                else:
                    rgb_np = np.clip(rgb_f, 0.0, 255.0).astype(np.uint8)
            if rgb_np.ndim == 3 and rgb_np.shape[-1] >= 3:
                color_bgr = np.ascontiguousarray(rgb_np[..., :3][:, :, ::-1], dtype=np.uint8)
            else:
                color_bgr = np.ascontiguousarray(rgb_np, dtype=np.uint8)
        else:
            color_bgr = np.zeros((target_h, target_w, 3), dtype=np.uint8)

        if bool(depth_enabled) and depth is not None:
            if hasattr(depth, "cpu"):
                depth = depth.cpu().numpy()
            depth_np = np.asarray(depth, dtype=float)
            if depth_np.ndim == 3:
                depth_np = depth_np[..., 0]
            depth_m = np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)
            depth_mm = np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)
        else:
            depth_mm = np.zeros((target_h, target_w), dtype=np.uint16)

        if color_bgr.shape[0] != target_h or color_bgr.shape[1] != target_w:
            import cv2

            color_bgr = cv2.resize(
                color_bgr,
                (target_w, target_h),
                interpolation=cv2.INTER_LINEAR,
            )
        if depth_mm.shape[0] != target_h or depth_mm.shape[1] != target_w:
            import cv2

            depth_mm = cv2.resize(
                depth_mm,
                (target_w, target_h),
                interpolation=cv2.INTER_NEAREST,
            )

        self._seq += 1
        cam_origin, cam_look, cam_right = self._camera_pose_world()
        return SimCameraFrame(
            color_bgr=color_bgr,
            depth_raw=depth_mm,
            depth_scale=float(self.depth_scale),
            intrinsics=self.intrinsics,
            seq=int(self._seq),
            ts=float(time.time() if ts is None else ts),
            arm_q=None,
            camera_world_origin=cam_origin,
            camera_world_look=cam_look,
            camera_world_right=cam_right,
        )
