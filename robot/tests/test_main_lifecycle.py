from __future__ import annotations

from types import SimpleNamespace

import pytest

import elesim_robot.main as robot_main


def _config(*, camera_enabled: bool) -> SimpleNamespace:
    return SimpleNamespace(
        endpoint_id="robot-a",
        camera=SimpleNamespace(
            enabled=camera_enabled,
            topic="/elesim/robot/rgbd",
            width=640,
            height=480,
            fps=30,
        ),
        dds=SimpleNamespace(security_profile="trusted-network"),
        rgbd=SimpleNamespace(format="encoded-rgbd-v1"),
        use_go2=True,
        mapping=object(),
        arm=object(),
        safety=SimpleNamespace(telemetry_period_s=0.1),
        device="",
        go2=object(),
    )


def _patch_startup(monkeypatch: pytest.MonkeyPatch, *, camera_enabled: bool) -> None:
    args = SimpleNamespace(
        config="unused.yaml",
        id="",
        device="",
        rgbd_topic="",
        camera=None,
    )
    parser = SimpleNamespace(parse_args=lambda: args)
    monkeypatch.setattr(robot_main, "_parser", lambda: parser)
    monkeypatch.setattr(
        robot_main,
        "load_config",
        lambda _path: _config(camera_enabled=camera_enabled),
    )
    monkeypatch.setattr(
        robot_main,
        "create_go2_client_if_enabled",
        lambda _config, _safety, *, use_go2: object() if use_go2 else None,
    )


class FakeClient:
    latest: "FakeClient | None" = None

    def __init__(self, *_args, **_kwargs) -> None:
        type(self).latest = self
        self.node = SimpleNamespace(identity=SimpleNamespace(boot_id="boot-a"))
        self.close_count = 0

    def close(self) -> None:
        self.close_count += 1


def test_bridge_start_failure_closes_runtime_and_dds_peer(monkeypatch: pytest.MonkeyPatch) -> None:
    _patch_startup(monkeypatch, camera_enabled=False)

    class BridgeUnavailableRuntime:
        latest: "BridgeUnavailableRuntime | None" = None

        def __init__(self, **_kwargs) -> None:
            type(self).latest = self
            self.close_count = 0

        def open(self) -> None:
            raise RuntimeError("bridge unavailable")

        def close(self) -> None:
            self.close_count += 1

    monkeypatch.setattr(robot_main, "PeerClient", FakeClient)
    monkeypatch.setattr(robot_main, "RobotRuntime", BridgeUnavailableRuntime)

    with pytest.raises(RuntimeError, match="bridge unavailable"):
        robot_main._run()

    assert BridgeUnavailableRuntime.latest is not None
    assert BridgeUnavailableRuntime.latest.close_count == 1
    assert FakeClient.latest is not None
    assert FakeClient.latest.close_count == 1


def test_camera_start_failure_stops_camera_and_closes_runtime_and_peer(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_startup(monkeypatch, camera_enabled=True)

    class OpenRuntime:
        latest: "OpenRuntime | None" = None

        def __init__(self, **_kwargs) -> None:
            type(self).latest = self
            self.open_count = 0
            self.close_count = 0

        def open(self) -> None:
            self.open_count += 1

        def close(self) -> None:
            self.close_count += 1

    class FailingCamera:
        latest: "FailingCamera | None" = None

        def __init__(self, *_args, **_kwargs) -> None:
            type(self).latest = self
            self.start_count = 0
            self.stop_count = 0

        def start(self) -> None:
            self.start_count += 1
            raise RuntimeError("camera start failed")

        def stop(self) -> None:
            self.stop_count += 1

    monkeypatch.setattr(robot_main, "PeerClient", FakeClient)
    monkeypatch.setattr(robot_main, "RobotRuntime", OpenRuntime)
    monkeypatch.setattr(robot_main, "CameraPublisherThread", FailingCamera)

    with pytest.raises(RuntimeError, match="camera start failed"):
        robot_main._run()

    assert FailingCamera.latest is not None
    assert FailingCamera.latest.start_count == 1
    assert FailingCamera.latest.stop_count == 1
    assert OpenRuntime.latest is not None
    assert OpenRuntime.latest.open_count == 1
    assert OpenRuntime.latest.close_count == 1
    assert FakeClient.latest is not None
    assert FakeClient.latest.close_count == 1


def test_transient_dds_loss_keeps_robot_safety_loop_alive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _patch_startup(monkeypatch, camera_enabled=False)

    class RuntimeWithSafetyLoop:
        latest: "RuntimeWithSafetyLoop | None" = None

        def __init__(self, **_kwargs) -> None:
            type(self).latest = self
            self.active_lease = "lease-1"
            self.pilot_id = "pilot-a"
            self.revoke_count = 0
            self.tick_count = 0
            self.close_count = 0

        def open(self) -> None:
            return None

        def revoke_lease(self) -> None:
            self.revoke_count += 1
            self.active_lease = ""
            self.pilot_id = ""

        def tick(self) -> None:
            self.tick_count += 1

        def close(self) -> None:
            self.close_count += 1

    class FlakyClient(FakeClient):
        receive_count = 0

        def heartbeat(self) -> None:
            return None

        def receive(self, *, timeout_ms: int = 0):
            del timeout_ms
            type(self).receive_count += 1
            if type(self).receive_count == 1:
                raise robot_main.DdsTransportError("target peer disappeared")
            raise KeyboardInterrupt

    monkeypatch.setattr(robot_main, "PeerClient", FlakyClient)
    monkeypatch.setattr(robot_main, "RobotRuntime", RuntimeWithSafetyLoop)

    robot_main._run()

    assert RuntimeWithSafetyLoop.latest is not None
    assert RuntimeWithSafetyLoop.latest.revoke_count == 1
    assert RuntimeWithSafetyLoop.latest.tick_count >= 1
    assert RuntimeWithSafetyLoop.latest.close_count == 1
