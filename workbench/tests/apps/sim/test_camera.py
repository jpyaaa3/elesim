from __future__ import annotations

from pathlib import Path

import numpy as np
from elesim_protocol import RgbdIntrinsicsSample, RgbdSample

from elesim_sim.vision.sim_camera.mount import (
    _OPTICAL_FROM_GENESIS_CAMERA,
    Node9EyeInHandCamera,
    ObserverCamera,
    ObserverViewState,
    genesis_drag_zoom_delta,
    genesis_scroll_zoom_delta,
    hand_eye_to_genesis_attach_T,
    load_hand_eye_offset_T,
)
from elesim_sim.vision.sim_camera.pose import (
    _link_world_transform,
    camera_axes_from_genesis_camera_object,
)
from elesim_sim.vision.sim_camera.publisher import SimCameraPublisher
from elesim_sim.vision.sim_camera.subscriber import SimCameraSubscriber
from elesim_sim.vision.sim_camera.types import SimCameraFrame, SimCameraIntrinsics


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
CONFIG_DIR = REPO_ROOT / "payload" / "config" / "sim"
CALIBRATION_DIR = REPO_ROOT / "payload" / "data" / "calibration" / "cameras"


def test_hand_eye_capture_converts_rgb_and_depth() -> None:
    class Camera:
        @staticmethod
        def render(**_kwargs):
            rgb = np.full((3, 4, 3), 0.5, dtype=np.float32)
            depth = np.full((3, 4), 1.25, dtype=np.float32)
            return rgb, depth, None, None

    intrinsics = SimCameraIntrinsics(
        fx=4.0,
        fy=4.0,
        cx=2.0,
        cy=1.5,
        width=4,
        height=3,
    )
    camera = Node9EyeInHandCamera(camera=Camera(), intrinsics=intrinsics)

    frame = camera.capture(prefer_gpu=False)

    assert frame.color_bgr.shape == (3, 4, 3)
    assert frame.color_bgr.dtype == np.uint8
    assert frame.depth_raw.shape == (3, 4)
    assert frame.depth_raw.dtype == np.uint16
    assert np.all(frame.depth_raw == 1250)


class _FakeTensor:
    def __init__(self, data) -> None:
        self._data = np.asarray(data, dtype=float)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._data


class _FakeLink:
    def __init__(self, pos, quat_wxyz) -> None:
        self._pos = _FakeTensor(pos)
        self._quat = _FakeTensor(quat_wxyz)

    def get_pos(self):
        return self._pos

    def get_quat(self):
        return self._quat


def test_hand_eye_optical_axes_match_camera_calibration() -> None:
    calibration = CALIBRATION_DIR / "zed_mini.hand_eye.json"
    transform = load_hand_eye_offset_T(calibration)
    rotation = transform[:3, :3]
    np.testing.assert_allclose(rotation[:, 2], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(rotation[:, 0], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(rotation[:, 1], [0.0, 0.0, 1.0], atol=1e-6)


def test_genesis_camera_object_matches_link_attachment() -> None:
    calibration = CALIBRATION_DIR / "zed_mini.hand_eye.json"
    link = _FakeLink([0.2, 0.0, 0.5], [1.0, 0.0, 0.0, 0.0])
    world_genesis = _link_world_transform(link) @ hand_eye_to_genesis_attach_T(calibration)

    class _FakeCamera:
        def get_transform(self):
            return world_genesis

    origin, look, right = camera_axes_from_genesis_camera_object(_FakeCamera(), axis_len_m=0.1)
    world_optical = world_genesis @ _OPTICAL_FROM_GENESIS_CAMERA
    np.testing.assert_allclose(origin, world_optical[:3, 3], atol=1e-6)
    np.testing.assert_allclose(look, world_optical[:3, :3] @ [0.0, 0.0, 0.1], atol=1e-6)
    np.testing.assert_allclose(right, world_optical[:3, :3] @ [0.1, 0.0, 0.0], atol=1e-6)


def test_hand_eye_pose_warning_is_reported_once(capsys) -> None:
    class _BrokenCamera:
        def move_to_attach(self) -> None:
            raise RuntimeError("camera pose unavailable")

    eye = Node9EyeInHandCamera(
        camera=_BrokenCamera(),
        intrinsics=SimCameraIntrinsics(100.0, 100.0, 1.0, 1.0, 2, 2),
    )

    assert eye.camera_axes_world() is None
    assert eye.camera_axes_world() is None

    output = capsys.readouterr().out
    assert output.count("camera_axes_world failed") == 1


def test_sim_camera_pub_sub_roundtrip() -> None:
    channel: dict[str, object] = {}

    class _Publisher:
        bound_endpoint = "/elesim/sim/rgbd/frame"
        published = 0
        dropped = 0

        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def publish(self, frame) -> bool:
            channel["frame"] = RgbdSample(
                color_bgr=frame.color_bgr,
                depth_raw=frame.depth_raw,
                depth_scale=frame.depth_scale,
                intrinsics=RgbdIntrinsicsSample(
                    fx=frame.intrinsics.fx,
                    fy=frame.intrinsics.fy,
                    cx=frame.intrinsics.cx,
                    cy=frame.intrinsics.cy,
                    width=frame.intrinsics.width,
                    height=frame.intrinsics.height,
                ),
                seq=frame.seq,
                ts=frame.ts,
                source_id="sim-a",
                source_boot_id="boot-a",
                arm_q=frame.arm_q,
            )
            self.published += 1
            return True

        def close(self) -> None:
            pass

    class _Subscriber:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def connect(self) -> None:
            pass

        def recv_latest(self, *, timeout_ms: int):
            del timeout_ms
            return channel.pop("frame", None)

        def close(self) -> None:
            pass

    endpoint = "/elesim/sim/rgbd/frame"
    publisher = SimCameraPublisher(
        endpoint,
        endpoint_id="sim-a",
        publisher_factory=_Publisher,
    )
    subscriber = SimCameraSubscriber(
        endpoint,
        endpoint_id="pilot-a",
        subscriber_factory=_Subscriber,
    )
    subscriber.connect()
    try:
        height, width = 48, 64
        frame = SimCameraFrame(
            color_bgr=np.zeros((height, width, 3), dtype=np.uint8),
            depth_raw=np.full((height, width), 500, dtype=np.uint16),
            depth_scale=0.001,
            intrinsics=SimCameraIntrinsics(
                fx=50.0,
                fy=50.0,
                cx=32.0,
                cy=24.0,
                width=width,
                height=height,
            ),
            seq=1,
            ts=123.0,
            arm_q=(0.0, 0.0, 0.0, 0.0),
        )
        assert publisher.publish(frame)
        received = subscriber.recv_latest(timeout_ms=100)
        assert received is not None
        assert received.color_bgr.shape == (height, width, 3)
        assert received.depth_raw.shape == (height, width)
        assert received.seq == 1
        assert received.arm_q == (0.0, 0.0, 0.0, 0.0)
    finally:
        publisher.close()
        subscriber.close()


def test_observer_camera_uses_genesis_scroll_ratio() -> None:
    assert genesis_scroll_zoom_delta(1.0) < 0.0
    assert genesis_scroll_zoom_delta(-1.0) > 0.0
    assert genesis_drag_zoom_delta(0.0, height=540.0) == 0.0


def test_observer_camera_pole_clamp_and_cad_screen_pan() -> None:
    class _Camera:
        def set_pose(self, *, pos, lookat, up):
            self.pos = pos
            self.lookat = lookat
            self.up = up

    camera = _Camera()
    observer = ObserverCamera(
        camera=camera,
        intrinsics=SimCameraIntrinsics(
            fx=1.0, fy=1.0, cx=480.0, cy=270.0, width=960, height=540
        ),
        pos=(0.45, -1.8, 0.55),
        lookat=(0.45, 0.0, 0.25),
    )
    observer.apply_operator_command("orbit", {"dx": 0.0, "dy": 10.0})
    offset = np.asarray(observer.pos) - np.asarray(observer.lookat)
    elevation = np.arctan2(offset[2], np.linalg.norm(offset[:2]))
    assert abs(float(elevation)) <= np.radians(89.0) + 1e-6
    assert camera.up == (0.0, 0.0, 1.0)
    eye_before = np.asarray(observer.pos)
    target_before = np.asarray(observer.lookat)
    forward = target_before - eye_before
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    observer.apply_operator_command("pan", {"dx": 0.1, "dy": 0.1})

    eye_shift = np.asarray(observer.pos) - eye_before
    target_shift = np.asarray(observer.lookat) - target_before
    np.testing.assert_allclose(eye_shift, target_shift)
    assert float(np.dot(eye_shift, right)) > 0.0
    assert float(np.dot(eye_shift, up)) < 0.0


def test_observer_horizontal_drag_keeps_one_screen_direction_above_and_below() -> None:
    def horizontal_motion(look_z: float) -> float:
        observer = ObserverViewState.create(
            res=(960, 540),
            pos=(0.0, 0.0, 0.0),
            lookat=(1.0, 0.0, look_z),
        )
        before = np.asarray(observer.lookat) - np.asarray(observer.pos)
        before /= np.linalg.norm(before)
        right = np.cross(before, np.array([0.0, 0.0, 1.0]))
        right /= np.linalg.norm(right)

        observer.apply_operator_command("orbit", {"dx": 0.05, "dy": 0.0})

        after = np.asarray(observer.lookat) - np.asarray(observer.pos)
        after /= np.linalg.norm(after)
        return float(np.dot(after - before, right))

    assert horizontal_motion(-0.7) > 0.0
    assert horizontal_motion(0.7) > 0.0
