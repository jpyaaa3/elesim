from __future__ import annotations

import shutil
import time
from pathlib import Path

import numpy as np
from zmq.auth import create_certificates

from elesim_controller.vision.sim_camera.subscriber import SimCameraSubscriber
from elesim_protocol import CurveClientConfig, CurveServerConfig
from elesim_simulator.vision.sim_camera.publisher import SimCameraPublisher
from elesim_simulator.vision.sim_camera.types import (
    SimCameraFrame,
    SimCameraIntrinsics,
)


def _certificate(directory: Path, name: str) -> tuple[Path, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    create_certificates(str(directory), name)
    return directory / f"{name}.key", directory / f"{name}.key_secret"


def test_simulator_rgbd_curve_stream_accepts_controller_and_rejects_other_keys(
    tmp_path: Path,
) -> None:
    server_public, server_secret = _certificate(tmp_path / "server", "sim-media")
    controller_public, controller_secret = _certificate(
        tmp_path / "clients", "controller"
    )
    _attacker_public, attacker_secret = _certificate(tmp_path / "clients", "other")
    authorized = tmp_path / "authorized"
    authorized.mkdir()
    shutil.copy2(controller_public, authorized / controller_public.name)

    publisher = SimCameraPublisher(
        "tcp://127.0.0.1:*",
        use_jpeg=False,
        curve=CurveServerConfig.from_file(server_secret),
        curve_client_keys_dir=authorized,
    )
    controller = SimCameraSubscriber(
        publisher.bound_endpoint,
        use_jpeg=False,
        curve=CurveClientConfig.from_files(
            client_secret_file=controller_secret,
            server_public_file=server_public,
        ),
    )
    other = SimCameraSubscriber(
        publisher.bound_endpoint,
        use_jpeg=False,
        curve=CurveClientConfig.from_files(
            client_secret_file=attacker_secret,
            server_public_file=server_public,
        ),
    )
    color = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)
    depth = np.arange(12, dtype=np.uint16).reshape(3, 4)
    frame = SimCameraFrame(
        color_bgr=color,
        depth_raw=depth,
        depth_scale=0.001,
        intrinsics=SimCameraIntrinsics(100.0, 101.0, 2.0, 1.5, 4, 3),
        seq=7,
        ts=12.5,
        arm_q=(-0.1, 0.2, 0.3, -0.4),
    )
    controller.connect()
    other.connect()

    received = None
    unauthorized_frames = []
    deadline = time.monotonic() + 3.0
    try:
        while received is None and time.monotonic() < deadline:
            publisher.publish(frame)
            received = controller.recv_latest(timeout_ms=50)
            unauthorized_frames.append(other.recv_latest(timeout_ms=5))
            time.sleep(0.01)

        assert received is not None
        np.testing.assert_array_equal(received.color_bgr, color)
        np.testing.assert_array_equal(received.depth_raw, depth)
        assert received.seq == 7
        assert received.arm_q == (-0.1, 0.2, 0.3, -0.4)
        assert all(value is None for value in unauthorized_frames)
    finally:
        controller.close()
        other.close()
        publisher.close()
