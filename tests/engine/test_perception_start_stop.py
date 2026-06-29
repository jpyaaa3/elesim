from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.core.config_loader import PerceptionConfig
from engine.behaviors.pick.actions import ControlService
from engine.vision.perception.capture import PerceptionCapture, PerceptionSnapshot
from engine.behaviors.pick.state import PanelState


class _FakeCapture:
    def __init__(self, *, running: bool = False) -> None:
        self._running = bool(running)
        self.stop_calls = 0

    def is_running(self) -> bool:
        return bool(self._running)

    def stop(self, *, timeout_s: float = 5.0) -> bool:
        self.stop_calls += 1
        self._running = False
        return True

    def start(self) -> None:
        self._running = True


class TestPerceptionStartStop(unittest.TestCase):
    def _service(self) -> ControlService:
        svc = ControlService(PanelState(), client=MagicMock())
        return svc

    def test_stop_does_not_clear_newer_capture(self) -> None:
        svc = self._service()
        old = _FakeCapture(running=True)
        new = _FakeCapture()
        svc._perception_capture = old

        stop_done = threading.Event()

        def _slow_stop(*, timeout_s: float = 5.0) -> bool:
            stop_done.wait(timeout=1.0)
            old._running = False
            return True

        old.stop = _slow_stop  # type: ignore[method-assign]

        def _stop_old() -> None:
            svc.stop_perception_capture()

        t = threading.Thread(target=_stop_old, daemon=True)
        t.start()

        svc._perception_capture = new
        svc._perception_capture_epoch = 2
        stop_done.set()
        t.join(timeout=2.0)

        self.assertIs(svc._perception_capture, new)
        self.assertEqual(int(svc._perception_capture_epoch), 2)

    def test_stale_snapshot_callback_ignored(self) -> None:
        svc = self._service()
        svc._perception_capture_epoch = 3
        svc._on_perception_snapshot(
            PerceptionSnapshot(
                running=False,
                failed=False,
                status_msg="stopped",
                frame_idx=0,
                label="",
                confidence=0.0,
                p_camera=None,
                p_world=None,
                last_update_s=0.0,
            ),
            capture_epoch=2,
        )
        self.assertFalse(bool(svc.state.perception_running))

        svc._on_perception_snapshot(
            PerceptionSnapshot(
                running=True,
                failed=False,
                status_msg="live",
                frame_idx=1,
                label="obj",
                confidence=0.9,
                p_camera=(0.0, 0.0, 0.5),
                p_world=(0.1, 0.0, 1.0),
                last_update_s=0.0,
            ),
            capture_epoch=3,
        )
        self.assertTrue(bool(svc.state.perception_running))
        self.assertEqual(str(svc.state.perception_status_msg), "live")

    @patch("engine.behaviors.pick.actions.PerceptionCapture", autospec=True)
    def test_start_replaces_stuck_running_capture(self, cap_cls: MagicMock) -> None:
        svc = self._service()
        stuck = _FakeCapture(running=True)
        stuck.stop = lambda *, timeout_s=5.0: False  # type: ignore[method-assign]
        svc._perception_capture = stuck

        created = MagicMock()
        cap_cls.return_value = created

        svc.start_perception_capture(config=PerceptionConfig(mode="mock"))

        self.assertIsNot(svc._perception_capture, stuck)
        self.assertIs(svc._perception_capture, created)
        created.start.assert_called_once()

    def test_live_capture_waits_through_transient_frame_miss(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(mode="sim"),
            publish_fn=lambda **kwargs: (0.0, 0.0, 0.5),
        )
        calls = 0

        class _Cam:
            def capture(self, *args, **kwargs):
                nonlocal calls
                calls += 1
                if calls == 1:
                    raise RuntimeError("no sim camera frame received")
                return "frame"

        frame = cap._capture_live_frame(_Cam())

        self.assertEqual(frame, "frame")
        self.assertFalse(bool(cap.snapshot().failed))
        self.assertIn("waiting for camera frame", str(cap.snapshot().status_msg))


if __name__ == "__main__":
    unittest.main()
