from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "payload").is_dir())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.config import PerceptionConfig
from elesim_pilot.vision.perception.capture import PerceptionCapture, TrackerPhase


def _make_det(*, shift_x: int = 0) -> SimpleNamespace:
    mask = np.zeros((48, 64), dtype=np.uint8)
    mask[16:32, 24 + shift_x : 40 + shift_x] = 255
    return SimpleNamespace(
        mask=mask,
        bbox_xyxy=(24 + shift_x, 16, 40 + shift_x, 32),
        label="ball",
        confidence=0.95,
    )


class _FakeTracker:
    backend_name = "fake-csrt"

    def __init__(self, *, drift_px: int = 2) -> None:
        self.initialized = False
        self._bbox = (24, 16, 40, 32)
        self._drift = int(drift_px)
        self._fail_after: int | None = None
        self._updates = 0

    def init(self, _frame, bbox) -> bool:
        self._bbox = tuple(int(v) for v in bbox)
        self.initialized = True
        return True

    def update(self, _frame):
        self._updates += 1
        if self._fail_after is not None and self._updates > self._fail_after:
            return None
        x0, y0, x1, y1 = self._bbox
        x0 += self._drift
        x1 += self._drift
        self._bbox = (x0, y0, x1, y1)
        return self._bbox

    def reset(self) -> None:
        self.initialized = False
        self._updates = 0


class _Frame:
    color_bgr = np.zeros((48, 64, 3), dtype=np.uint8)
    depth_raw = np.ones((48, 64), dtype=np.uint16) * 500
    depth_scale = 0.001

    class _Intrinsics:
        fx = fy = 600.0
        cx = 32.0
        cy = 24.0
        width = 64
        height = 48

    intrinsics = _Intrinsics()


class TestYoloSegPipeline(unittest.TestCase):
    def _run_loop(
        self,
        *,
        cap: PerceptionCapture,
        det_schedule: list,
        aux_tracker: _FakeTracker | None = None,
        track_aux_csrt: bool = True,
        track_lost_frames: int = 5,
        reacquire_on_lost: bool = True,
    ) -> None:
        frame_count = 0

        class _Cam:
            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def capture(self):
                nonlocal frame_count
                frame_count += 1
                if frame_count >= len(det_schedule) + 1:
                    cap._stop_event.set()
                return _Frame()

        def _list_dets(_detector, _color):
            idx = min(frame_count - 1, len(det_schedule) - 1)
            item = det_schedule[idx]
            return [] if item is None else [item]

        def _pick(dets, _label):
            return dets[0] if dets else None

        def _measure(_det, **kwargs):
            return np.array([0.01, 0.0, 0.5], dtype=float)

        def _build_obs(**kwargs):
            return SimpleNamespace(
                label="ball",
                confidence=0.95,
                p_camera_object=np.array([0.01, 0.0, 0.5]),
            )

        cap._config = PerceptionConfig(
            pipeline="yolo_seg",
            publish_hz=100.0,
            track_lost_frames=int(track_lost_frames),
            track_aux_csrt=bool(track_aux_csrt),
            track_coast_max_frames=12,
            reacquire_on_lost=bool(reacquire_on_lost),
        )
        cap._run_camera_yolo_seg(
            detector=MagicMock(),
            detector_cfg={"mask_erode_px": 0, "z_min_m": 0.1, "z_max_m": 2.0},
            measure_detection=_measure,
            build_camera_observation=_build_obs,
            list_frame_detections=_list_dets,
            pick_target_detection=_pick,
            model_class_names=lambda _d: [],
            RealSenseCamera=_Cam,
            show_preview=False,
            draw_detection_overlay=lambda *a, **k: None,
            show_preview_fn=lambda *a, **k: -1,
            target_label="ball",
            publish_period=0.0,
            normalized_detection_center_uv=lambda det, **k: (0.0, 0.0),
            detection_scale=lambda det, **k: 0.05,
            aux_tracker=aux_tracker,
        )

    def test_normalize_pipeline(self) -> None:
        self.assertEqual(PerceptionCapture._normalize_pipeline("yolo_seg"), "yolo_seg")
        self.assertEqual(PerceptionCapture._normalize_pipeline("yolo-seg"), "yolo_seg")
        self.assertEqual(PerceptionCapture._normalize_pipeline("yolo_only"), "yolo_seg")
        self.assertEqual(PerceptionCapture._normalize_pipeline("search_track"), "search_track")

    def test_uses_yolo_mask_pipeline(self) -> None:
        self.assertTrue(PerceptionCapture._uses_yolo_mask_pipeline("yolo_seg"))
        self.assertFalse(PerceptionCapture._uses_yolo_mask_pipeline("search_track"))

    def test_yolo_seg_loop_increments_track_ok(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(pipeline="yolo_seg", publish_hz=100.0, track_lost_frames=5),
            publish_fn=lambda **kwargs: (0.0, 0.0, 0.5),
        )
        det = _make_det()
        self._run_loop(cap=cap, det_schedule=[det, det, det], track_aux_csrt=False)

        snap = cap.snapshot()
        self.assertEqual(str(snap.tracker_phase), TrackerPhase.TRACK.value)
        self.assertGreaterEqual(int(snap.track_ok_frames), 2)
        self.assertEqual(str(snap.tracker_backend), "yolo-seg")

    def test_yolo_seg_coast_maintains_track_ok_on_csrt_miss(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(
                pipeline="yolo_seg",
                track_aux_csrt=True,
                track_coast_max_frames=12,
                track_lost_frames=5,
            ),
            publish_fn=lambda **kwargs: (0.0, 0.0, 0.5),
        )
        det = _make_det()
        tracker = _FakeTracker(drift_px=2)
        self._run_loop(
            cap=cap,
            det_schedule=[det, det, None, None, det],
            aux_tracker=tracker,
        )

        snap = cap.snapshot()
        self.assertEqual(str(snap.tracker_phase), TrackerPhase.TRACK.value)
        self.assertGreaterEqual(int(snap.track_ok_frames), 3)
        self.assertIn("csrt", str(snap.tracker_backend).lower())

    def test_yolo_seg_dual_miss_resets_track_ok(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(
                pipeline="yolo_seg",
                track_aux_csrt=True,
                track_lost_frames=5,
            ),
            publish_fn=lambda **kwargs: (0.0, 0.0, 0.5),
        )
        det = _make_det()
        tracker = _FakeTracker(drift_px=2)
        tracker._fail_after = 0
        self._run_loop(
            cap=cap,
            det_schedule=[det, det, None, None],
            aux_tracker=tracker,
        )

        snap = cap.snapshot()
        self.assertEqual(int(snap.track_ok_frames), 0)

    def test_yolo_seg_target_lost_does_not_mark_camera_failed(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(
                pipeline="yolo_seg",
                track_aux_csrt=False,
                track_lost_frames=1,
                reacquire_on_lost=False,
            ),
            publish_fn=lambda **kwargs: (0.0, 0.0, 0.5),
        )
        det = _make_det()
        self._run_loop(
            cap=cap,
            det_schedule=[det, None, None],
            track_aux_csrt=False,
            track_lost_frames=1,
            reacquire_on_lost=False,
        )

        snap = cap.snapshot()
        self.assertFalse(bool(snap.failed))

    def test_coast_publish_mask_not_full_rectangle(self) -> None:
        published_masks: list[np.ndarray] = []

        def _publish(**kwargs):
            return (0.0, 0.0, 0.5)

        cap = PerceptionCapture(
            PerceptionConfig(pipeline="yolo_seg", track_aux_csrt=True),
            publish_fn=_publish,
        )
        original_publish = cap._publish_observation

        def _capture_publish(*args, **kwargs):
            det = kwargs.get("det")
            if det is not None and getattr(det, "mask", None) is not None:
                published_masks.append(np.asarray(det.mask))
            return original_publish(*args, **kwargs)

        cap._publish_observation = _capture_publish  # type: ignore[method-assign]

        det = _make_det()
        tracker = _FakeTracker(drift_px=1)
        self._run_loop(
            cap=cap,
            det_schedule=[det, None, None],
            aux_tracker=tracker,
        )

        self.assertGreaterEqual(len(published_masks), 2)
        for mask in published_masks[1:]:
            nz = int(np.count_nonzero(mask))
            h, w = mask.shape
            self.assertLess(nz, h * w)


if __name__ == "__main__":
    unittest.main()
