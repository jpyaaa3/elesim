"""Background camera/mock perception loop for the control panel."""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

from engine.config_loader import PerceptionConfig

_PICK_PLACE_ROOT = Path(__file__).resolve().parents[2] / "addons" / "autonomous_pick_place_app"
_PREVIEW_WINDOW = "elesim_perception"


class TrackerPhase(str, Enum):
    SEARCH = "search"
    TRACK = "track"
    LOST = "lost"


def _ensure_pick_place_path() -> Path:
    root = _PICK_PLACE_ROOT.resolve()
    root_s = str(root)
    if root_s not in sys.path:
        sys.path.insert(0, root_s)
    return root


@dataclass(frozen=True)
class PerceptionSnapshot:
    running: bool
    failed: bool
    status_msg: str
    frame_idx: int
    label: str
    confidence: float
    p_camera: Optional[tuple[float, float, float]]
    p_world: Optional[tuple[float, float, float]]
    last_update_s: float
    tracker_phase: str = TrackerPhase.SEARCH.value
    track_ok_frames: int = 0


class PerceptionCapture:
    """Runs detection in a worker thread; publishes via ``publish_fn``."""

    def __init__(
        self,
        config: PerceptionConfig,
        *,
        publish_fn: Callable[..., Optional[tuple[float, float, float]]],
        on_snapshot: Optional[Callable[[PerceptionSnapshot], None]] = None,
    ) -> None:
        self._config = config
        self._publish_fn = publish_fn
        self._on_snapshot = on_snapshot
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._snapshot = PerceptionSnapshot(
            running=False,
            failed=False,
            status_msg="idle",
            frame_idx=0,
            label="",
            confidence=0.0,
            p_camera=None,
            p_world=None,
            last_update_s=0.0,
            tracker_phase=TrackerPhase.SEARCH.value,
            track_ok_frames=0,
        )

    def snapshot(self) -> PerceptionSnapshot:
        with self._lock:
            return self._snapshot

    def tracker_phase(self) -> str:
        return str(self.snapshot().tracker_phase)

    def track_ok_frames(self) -> int:
        return int(self.snapshot().track_ok_frames)

    def _set_snapshot(self, **kwargs: Any) -> None:
        with self._lock:
            fields = {f.name for f in PerceptionSnapshot.__dataclass_fields__.values()}  # type: ignore[attr-defined]
            data = {k: v for k, v in kwargs.items() if k in fields}
            self._snapshot = PerceptionSnapshot(**{**self._snapshot.__dict__, **data})
        if self._on_snapshot is not None:
            self._on_snapshot(self.snapshot())

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running():
            return
        self._stop_event.clear()
        self._set_snapshot(
            running=True,
            failed=False,
            status_msg="starting",
            frame_idx=0,
            tracker_phase=TrackerPhase.SEARCH.value,
            track_ok_frames=0,
        )
        self._thread = threading.Thread(target=self._run, name="perception-capture", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None
        self._set_snapshot(running=False, status_msg="stopped")

    def _run(self) -> None:
        _ensure_pick_place_path()
        try:
            from main import (  # type: ignore[import-not-found]
                build_camera_observation,
                detection_scale,
                measure_detection,
                normalized_detection_center_uv,
                resolve_detector_cfg,
                run_mock_frame,
                _list_frame_detections,
                _model_class_names,
                _pick_target_detection,
            )
            from perception.detector import create_detector, load_detector_config  # type: ignore[import-not-found]
            from perception.preview import (  # type: ignore[import-not-found]
                close_preview,
                draw_detection_overlay,
                show_preview as render_preview_frame,
            )
            from perception.realsense_camera import RealSenseCamera, RealSenseUnavailableError  # type: ignore[import-not-found]
            from perception.visual_tracker import BboxTracker, detection_from_bbox  # type: ignore[import-not-found]
            from perception.yolo_detector import YoloUnavailableError  # type: ignore[import-not-found]
        except Exception as exc:
            self._set_snapshot(running=False, failed=True, status_msg=f"import failed: {exc}")
            return

        cfg = self._config
        config_path = cfg.resolved_detector_config_path()
        if not config_path.is_file():
            self._set_snapshot(
                running=False,
                failed=True,
                status_msg=f"detector config not found: {config_path}",
            )
            return

        try:
            file_cfg = load_detector_config(str(config_path))
            detector_cfg = resolve_detector_cfg(
                file_cfg,
                detector_cli=str(cfg.detector),
                target_label_cli=str(cfg.target_label),
                yolo_device_cli=(str(cfg.yolo_device) if cfg.yolo_device else None),
                mode=str(cfg.mode),
            )
            detector = create_detector(detector_cfg)
        except (RealSenseUnavailableError, YoloUnavailableError) as exc:
            self._set_snapshot(running=False, failed=True, status_msg=str(exc))
            return
        except Exception as exc:
            self._set_snapshot(running=False, failed=True, status_msg=f"detector init failed: {exc}")
            return

        target_label = str(detector_cfg.get("target_label", "") or "")
        enable_preview = bool(cfg.show_preview)
        publish_period = (1.0 / float(cfg.publish_hz)) if float(cfg.publish_hz) > 0 else 0.0
        mode = str(cfg.mode).strip().lower()
        use_search_track = str(cfg.pipeline).strip().lower() in ("search_track", "search-track", "track")

        common = dict(
            detector=detector,
            detector_cfg=detector_cfg,
            measure_detection=measure_detection,
            build_camera_observation=build_camera_observation,
            normalized_detection_center_uv=normalized_detection_center_uv,
            detection_scale=detection_scale,
            list_frame_detections=_list_frame_detections,
            pick_target_detection=_pick_target_detection,
            model_class_names=_model_class_names,
            show_preview=enable_preview,
            draw_detection_overlay=draw_detection_overlay,
            show_preview_fn=render_preview_frame,
            target_label=target_label,
            detection_from_bbox=detection_from_bbox,
            BboxTracker=BboxTracker,
        )

        try:
            if mode == "mock":
                if use_search_track:
                    self._run_mock_search_track(
                        run_mock_frame=run_mock_frame,
                        publish_period=publish_period,
                        **common,
                    )
                else:
                    self._run_mock(
                        run_mock_frame=run_mock_frame,
                        **common,
                    )
            elif use_search_track:
                self._run_camera_search_track(
                    RealSenseCamera=RealSenseCamera,
                    publish_period=publish_period,
                    **common,
                )
            else:
                self._run_camera_yolo_only(
                    RealSenseCamera=RealSenseCamera,
                    publish_period=publish_period,
                    **common,
                )
        except RealSenseUnavailableError as exc:
            self._set_snapshot(running=False, failed=True, status_msg=f"RealSense: {exc}")
        except Exception as exc:
            self._set_snapshot(running=False, failed=True, status_msg=str(exc))
        finally:
            if show_preview:
                from perception.preview import close_preview  # type: ignore[import-not-found]

                close_preview(_PREVIEW_WINDOW)

    def _publish_observation(
        self,
        *,
        obs: Any,
        det: Any,
        image_width: int,
        image_height: int,
        detection_scale_fn: Any,
        normalized_center_uv_fn: Any,
        status_msg: str,
    ) -> Optional[tuple[float, float, float]]:
        p_cam = np.asarray(obs.p_camera_object, dtype=float).reshape(3)
        uv = normalized_center_uv_fn(det, image_width=image_width, image_height=image_height)
        scale = detection_scale_fn(det, image_width=image_width, image_height=image_height)
        p_world = self._publish_fn(
            object_camera_xyz=(float(p_cam[0]), float(p_cam[1]), float(p_cam[2])),
            label=str(obs.label),
            confidence=float(obs.confidence),
            image_center_uv=uv,
            image_scale=float(scale),
        )
        self._set_snapshot(
            label=str(obs.label),
            confidence=float(obs.confidence),
            p_camera=(float(p_cam[0]), float(p_cam[1]), float(p_cam[2])),
            p_world=p_world,
            last_update_s=float(time.time()),
            status_msg=str(status_msg),
            failed=False,
        )
        return p_world

    def _process_detection(
        self,
        *,
        frame: Any,
        det: Any,
        detector_cfg: dict,
        measure_detection: Any,
        build_camera_observation: Any,
        detection_scale_fn: Any,
        normalized_center_uv_fn: Any,
        status_msg: str,
    ) -> Optional[tuple[float, float, float]]:
        p_camera = measure_detection(
            det,
            depth_raw=frame.depth_raw,
            intrinsics=frame.intrinsics,
            depth_scale=frame.depth_scale,
            detector_cfg=detector_cfg,
        )
        if p_camera is None:
            return None
        obs = build_camera_observation(
            detection_label=det.label,
            confidence=det.confidence,
            p_camera_object=p_camera,
        )
        img_h, img_w = frame.color_bgr.shape[:2]
        return self._publish_observation(
            obs=obs,
            det=det,
            image_width=img_w,
            image_height=img_h,
            detection_scale_fn=detection_scale_fn,
            normalized_center_uv_fn=normalized_center_uv_fn,
            status_msg=status_msg,
        )

    def _run_camera_search_track(self, **kwargs: Any) -> None:
        detector = kwargs["detector"]
        detector_cfg = kwargs["detector_cfg"]
        measure_detection = kwargs["measure_detection"]
        build_camera_observation = kwargs["build_camera_observation"]
        list_frame_detections = kwargs["list_frame_detections"]
        pick_target_detection = kwargs["pick_target_detection"]
        model_class_names = kwargs["model_class_names"]
        RealSenseCamera = kwargs["RealSenseCamera"]
        show_preview = kwargs["show_preview"]
        draw_detection_overlay = kwargs["draw_detection_overlay"]
        show_preview_fn = kwargs["show_preview_fn"]
        target_label = kwargs["target_label"]
        publish_period = kwargs["publish_period"]
        normalized_detection_center_uv = kwargs["normalized_detection_center_uv"]
        detection_scale = kwargs["detection_scale"]
        detection_from_bbox = kwargs["detection_from_bbox"]
        BboxTracker = kwargs["BboxTracker"]

        cfg = self._config
        lost_limit = max(1, int(cfg.track_lost_frames))
        reacquire = bool(cfg.reacquire_on_lost)
        tracker = BboxTracker(tracker_type=str(cfg.tracker))

        phase = TrackerPhase.SEARCH
        lost_streak = 0
        track_ok = 0
        frame_idx = 0
        tracked_label = target_label
        all_dets: list = []

        self._set_snapshot(status_msg="searching (YOLO)", tracker_phase=phase.value)

        with RealSenseCamera() as cam:
            while not self._stop_event.is_set():
                t0 = time.time()
                frame = cam.capture()
                img_h, img_w = frame.color_bgr.shape[:2]
                det = None
                status = phase.value
                p_camera = None
                p_world = None

                if phase == TrackerPhase.SEARCH:
                    all_dets = list_frame_detections(detector, frame.color_bgr)
                    yolo_det = pick_target_detection(all_dets, target_label)
                    if yolo_det is not None:
                        if tracker.init(frame.color_bgr, yolo_det.bbox_xyxy):
                            tracked_label = str(yolo_det.label)
                            phase = TrackerPhase.TRACK
                            lost_streak = 0
                            track_ok = 0
                            det = yolo_det
                            status = "track init"
                            p_world = self._process_detection(
                                frame=frame,
                                det=det,
                                detector_cfg=detector_cfg,
                                measure_detection=measure_detection,
                                build_camera_observation=build_camera_observation,
                                detection_scale_fn=detection_scale,
                                normalized_center_uv_fn=normalized_detection_center_uv,
                                status_msg="track init",
                            )
                            if p_world is not None:
                                p_camera = self.snapshot().p_camera
                                track_ok = 1
                        else:
                            status = "tracker init failed"
                    else:
                        status = "searching"

                elif phase == TrackerPhase.TRACK:
                    bbox = tracker.update(frame.color_bgr)
                    if bbox is not None:
                        lost_streak = 0
                        track_ok += 1
                        det = detection_from_bbox(
                            bbox,
                            image_width=img_w,
                            image_height=img_h,
                            label=tracked_label,
                            confidence=0.95,
                        )
                        status = f"tracking ({tracker.backend_name})"
                        p_world = self._process_detection(
                            frame=frame,
                            det=det,
                            detector_cfg=detector_cfg,
                            measure_detection=measure_detection,
                            build_camera_observation=build_camera_observation,
                            detection_scale_fn=detection_scale,
                            normalized_center_uv_fn=normalized_detection_center_uv,
                            status_msg=status,
                        )
                        if p_world is not None:
                            p_camera = self.snapshot().p_camera
                    else:
                        lost_streak += 1
                        status = f"track lost ({lost_streak}/{lost_limit})"
                        if lost_streak >= lost_limit:
                            phase = TrackerPhase.LOST if reacquire else TrackerPhase.SEARCH
                            tracker.reset()
                            track_ok = 0
                            if not reacquire:
                                self._set_snapshot(
                                    failed=True,
                                    status_msg="track lost",
                                    tracker_phase=TrackerPhase.LOST.value,
                                )

                elif phase == TrackerPhase.LOST:
                    if reacquire:
                        phase = TrackerPhase.SEARCH
                        status = "reacquiring"
                        lost_streak = 0
                    all_dets = []

                if show_preview:
                    vis = draw_detection_overlay(
                        frame.color_bgr,
                        det,
                        status=status,
                        target_label=target_label,
                        frame_idx=frame_idx,
                        p_camera=p_camera,
                        p_world=np.asarray(p_world) if p_world is not None else None,
                        all_detections=all_dets if phase == TrackerPhase.SEARCH else [],
                        model_classes=model_class_names(detector) if phase == TrackerPhase.SEARCH else [],
                    )
                    key = show_preview_fn(_PREVIEW_WINDOW, vis)
                    if key in (ord("q"), 27):
                        self._set_snapshot(status_msg="preview quit")
                        break

                self._set_snapshot(
                    frame_idx=frame_idx,
                    status_msg=status,
                    tracker_phase=phase.value,
                    track_ok_frames=int(track_ok),
                )
                frame_idx += 1
                if publish_period > 0:
                    elapsed = time.time() - t0
                    sleep_s = max(0.0, publish_period - elapsed)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

        self._set_snapshot(running=False, status_msg="stopped")

    def _run_camera_yolo_only(self, **kwargs: Any) -> None:
        detector = kwargs["detector"]
        detector_cfg = kwargs["detector_cfg"]
        measure_detection = kwargs["measure_detection"]
        build_camera_observation = kwargs["build_camera_observation"]
        list_frame_detections = kwargs["list_frame_detections"]
        pick_target_detection = kwargs["pick_target_detection"]
        model_class_names = kwargs["model_class_names"]
        RealSenseCamera = kwargs["RealSenseCamera"]
        show_preview = kwargs["show_preview"]
        draw_detection_overlay = kwargs["draw_detection_overlay"]
        show_preview_fn = kwargs["show_preview_fn"]
        target_label = kwargs["target_label"]
        publish_period = kwargs["publish_period"]
        normalized_detection_center_uv = kwargs["normalized_detection_center_uv"]
        detection_scale = kwargs["detection_scale"]

        self._set_snapshot(status_msg="camera live (yolo)", tracker_phase=TrackerPhase.SEARCH.value)
        frame_idx = 0
        with RealSenseCamera() as cam:
            while not self._stop_event.is_set():
                t0 = time.time()
                frame = cam.capture()
                all_dets = list_frame_detections(detector, frame.color_bgr)
                det = pick_target_detection(all_dets, target_label)
                status = "searching"
                p_camera = None
                p_world = None
                if det is not None:
                    p_world = self._process_detection(
                        frame=frame,
                        det=det,
                        detector_cfg=detector_cfg,
                        measure_detection=measure_detection,
                        build_camera_observation=build_camera_observation,
                        detection_scale_fn=detection_scale,
                        normalized_center_uv_fn=normalized_detection_center_uv,
                        status_msg="detected",
                    )
                    if p_world is not None:
                        status = "detected"
                        p_camera = self.snapshot().p_camera
                if show_preview:
                    vis = draw_detection_overlay(
                        frame.color_bgr,
                        det,
                        status=status,
                        target_label=target_label,
                        frame_idx=frame_idx,
                        p_camera=p_camera,
                        p_world=np.asarray(p_world) if p_world is not None else None,
                        all_detections=all_dets,
                        model_classes=model_class_names(detector),
                    )
                    key = show_preview_fn(_PREVIEW_WINDOW, vis)
                    if key in (ord("q"), 27):
                        break
                self._set_snapshot(frame_idx=frame_idx, status_msg=status)
                frame_idx += 1
                if publish_period > 0:
                    time.sleep(max(0.0, publish_period - (time.time() - t0)))
        self._set_snapshot(running=False, status_msg="stopped")

    def _run_mock(self, **kwargs: Any) -> None:
        detector = kwargs["detector"]
        detector_cfg = kwargs["detector_cfg"]
        measure_detection = kwargs["measure_detection"]
        build_camera_observation = kwargs["build_camera_observation"]
        run_mock_frame = kwargs["run_mock_frame"]
        show_preview = kwargs["show_preview"]
        draw_detection_overlay = kwargs["draw_detection_overlay"]
        show_preview_fn = kwargs["show_preview_fn"]
        target_label = kwargs["target_label"]
        normalized_detection_center_uv = kwargs["normalized_detection_center_uv"]
        detection_scale = kwargs["detection_scale"]

        if self._stop_event.is_set():
            return
        self._set_snapshot(status_msg="mock capture")
        color, depth, intrinsics, depth_scale = run_mock_frame(detector_cfg)

        class _MockFrame:
            pass

        mf = _MockFrame()
        mf.color_bgr = color
        mf.depth_raw = depth
        mf.intrinsics = intrinsics
        mf.depth_scale = depth_scale

        det = detector.detect(color)
        if det is None:
            self._set_snapshot(running=False, failed=True, status_msg="mock: no detection")
            return
        p_world = self._process_detection(
            frame=mf,
            det=det,
            detector_cfg=detector_cfg,
            measure_detection=measure_detection,
            build_camera_observation=build_camera_observation,
            detection_scale_fn=detection_scale,
            normalized_center_uv_fn=normalized_detection_center_uv,
            status_msg="mock detected",
        )
        if show_preview and p_world is not None:
            vis = draw_detection_overlay(
                color,
                det,
                status="detected",
                target_label=target_label,
                frame_idx=0,
                p_camera=self.snapshot().p_camera,
                p_world=np.asarray(p_world),
            )
            show_preview_fn(_PREVIEW_WINDOW, vis)
            time.sleep(0.05)
        self._set_snapshot(running=False, status_msg="mock done")

    def _run_mock_search_track(self, **kwargs: Any) -> None:
        detector = kwargs["detector"]
        detector_cfg = kwargs["detector_cfg"]
        measure_detection = kwargs["measure_detection"]
        build_camera_observation = kwargs["build_camera_observation"]
        run_mock_frame = kwargs["run_mock_frame"]
        show_preview = kwargs["show_preview"]
        draw_detection_overlay = kwargs["draw_detection_overlay"]
        show_preview_fn = kwargs["show_preview_fn"]
        target_label = kwargs["target_label"]
        normalized_detection_center_uv = kwargs["normalized_detection_center_uv"]
        detection_scale = kwargs["detection_scale"]
        detection_from_bbox = kwargs["detection_from_bbox"]
        BboxTracker = kwargs["BboxTracker"]
        publish_period = float(kwargs.get("publish_period", 0.0) or 0.0)

        color, depth, intrinsics, depth_scale = run_mock_frame(detector_cfg)

        class _MockFrame:
            pass

        mf = _MockFrame()
        mf.color_bgr = color
        mf.depth_raw = depth
        mf.intrinsics = intrinsics
        mf.depth_scale = depth_scale

        det = detector.detect(color)
        if det is None:
            self._set_snapshot(running=False, failed=True, status_msg="mock: no detection")
            return

        tracker = BboxTracker(tracker_type=str(self._config.tracker))
        if not tracker.init(color, det.bbox_xyxy):
            self._set_snapshot(running=False, failed=True, status_msg="mock: tracker init failed")
            return

        img_h, img_w = color.shape[:2]
        self._process_detection(
            frame=mf,
            det=det,
            detector_cfg=detector_cfg,
            measure_detection=measure_detection,
            build_camera_observation=build_camera_observation,
            detection_scale_fn=detection_scale,
            normalized_center_uv_fn=normalized_detection_center_uv,
            status_msg="mock track",
        )
        self._set_snapshot(tracker_phase=TrackerPhase.TRACK.value, track_ok_frames=1, status_msg="mock tracking")

        frame_idx = 0
        while not self._stop_event.is_set():
            t0 = time.time()
            bbox = tracker.update(color)
            det_track = None
            if bbox is not None:
                det_track = detection_from_bbox(
                    bbox,
                    image_width=img_w,
                    image_height=img_h,
                    label=str(det.label),
                    confidence=0.95,
                )
                self._process_detection(
                    frame=mf,
                    det=det_track,
                    detector_cfg=detector_cfg,
                    measure_detection=measure_detection,
                    build_camera_observation=build_camera_observation,
                    detection_scale_fn=detection_scale,
                    normalized_center_uv_fn=normalized_detection_center_uv,
                    status_msg="mock tracking",
                )
            if show_preview:
                vis = draw_detection_overlay(
                    color,
                    det_track or det,
                    status="mock tracking",
                    target_label=target_label,
                    frame_idx=frame_idx,
                    p_camera=self.snapshot().p_camera,
                )
                key = show_preview_fn(_PREVIEW_WINDOW, vis)
                if key in (ord("q"), 27):
                    break
            self._set_snapshot(frame_idx=frame_idx, track_ok_frames=frame_idx + 1)
            frame_idx += 1
            if publish_period > 0:
                time.sleep(max(0.0, publish_period - (time.time() - t0)))
            else:
                time.sleep(0.05)

        self._set_snapshot(running=False, status_msg="stopped")
