"""Background camera/mock perception loop for the control panel."""

from __future__ import annotations

import json
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Optional, Tuple

import numpy as np

if TYPE_CHECKING:
    from engine.config_loader import PerceptionConfig

_PICK_PLACE_ROOT = Path(__file__).resolve().parents[2] / "addons" / "autonomous_pick_place_app"
_PREVIEW_WINDOW = "elesim_perception"


def default_perception_capture_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "logs" / "perception_capture"


def save_perception_frame_bundle(
    *,
    out_dir: Path,
    color_bgr: np.ndarray,
    overlay_bgr: Optional[np.ndarray] = None,
    depth_raw: Optional[np.ndarray] = None,
    meta: Optional[dict[str, Any]] = None,
    stem: Optional[str] = None,
) -> Path:
    import cv2

    out_dir.mkdir(parents=True, exist_ok=True)
    tag = str(stem or time.strftime("%Y%m%d_%H%M%S"))
    color_path = out_dir / f"{tag}_color.jpg"
    cv2.imwrite(str(color_path), np.ascontiguousarray(color_bgr, dtype=np.uint8))
    if overlay_bgr is not None:
        overlay_path = out_dir / f"{tag}_overlay.jpg"
        cv2.imwrite(str(overlay_path), np.ascontiguousarray(overlay_bgr, dtype=np.uint8))
    if depth_raw is not None and np.asarray(depth_raw).size > 0:
        depth = np.asarray(depth_raw)
        depth_path = out_dir / f"{tag}_depth.png"
        cv2.imwrite(str(depth_path), depth.astype(np.uint16))
        depth_vis_path = out_dir / f"{tag}_depth_vis.jpg"
        d = depth.astype(np.float32)
        if float(np.nanmax(d)) > 0.0:
            vis = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX)
        else:
            vis = np.zeros_like(d, dtype=np.float32)
        cv2.imwrite(str(depth_vis_path), vis.astype(np.uint8))
    if meta is not None:
        meta_path = out_dir / f"{tag}_meta.json"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
    return color_path


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


def _parse_mock_world_xyz(detector_cfg: dict[str, Any]) -> Optional[tuple[float, float, float]]:
    """Optional fixed world position for mock perception (bypasses hand-eye transform)."""
    raw = detector_cfg.get("mock_world_xyz", None)
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        return None
    try:
        return (float(raw[0]), float(raw[1]), float(raw[2]))
    except (TypeError, ValueError):
        return None


def load_mock_world_xyz_from_detector_path(path: str | Path) -> Optional[tuple[float, float, float]]:
    """Read ``mock_world_xyz`` from a detector JSON file, if present."""
    p = Path(path)
    if not p.is_file():
        return None
    try:
        import json

        with open(p, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        if isinstance(cfg, dict):
            return _parse_mock_world_xyz(cfg)
    except Exception:
        return None
    return None


def _bbox_tracker_from_config(cfg: "PerceptionConfig") -> Any:
    from engine.vision.perception.visual_tracker import BboxTracker, CsrtTrackerTuning  # type: ignore[import-not-found]

    tuning = CsrtTrackerTuning(
        psr_threshold=float(cfg.track_csrt_psr_threshold),
        scale_lr=float(cfg.track_csrt_scale_lr),
        histogram_lr=float(cfg.track_csrt_histogram_lr),
        padding=float(cfg.track_csrt_padding),
        scale_step=float(cfg.track_csrt_scale_step),
        bbox_smooth_alpha=float(cfg.track_bbox_smooth_alpha),
    )
    return BboxTracker(tracker_type=str(cfg.tracker), csrt_tuning=tuning)


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
    depth_valid: bool = True
    image_scale: float = 0.0
    bbox_wh: tuple[int, int] = (0, 0)
    tracker_backend: str = ""
    center_uv: Optional[tuple[float, float]] = None


class PerceptionCapture:
    """Runs detection in a worker thread; publishes via ``publish_fn``."""

    _warned_missing_sim_pose: bool = False

    @staticmethod
    def _normalize_pipeline(pipeline: str) -> str:
        p = str(pipeline).strip().lower().replace("-", "_")
        if p in ("search_track", "track"):
            return "search_track"
        if p in ("yolo_seg", "yolo_only", "yolo"):
            return "yolo_seg"
        return p

    @staticmethod
    def _uses_yolo_mask_pipeline(pipeline: str) -> bool:
        return PerceptionCapture._normalize_pipeline(pipeline) == "yolo_seg"

    def __init__(
        self,
        config: "PerceptionConfig",
        *,
        publish_fn: Callable[..., Optional[tuple[float, float, float]]],
        on_snapshot: Optional[Callable[[PerceptionSnapshot], None]] = None,
        target_uv_fn: Optional[Callable[[], Tuple[float, float]]] = None,
        mock_world_xyz_fn: Optional[Callable[[], Optional[tuple[float, float, float]]]] = None,
    ) -> None:
        self._config = config
        self._publish_fn = publish_fn
        self._on_snapshot = on_snapshot
        self._target_uv_fn = target_uv_fn
        self._mock_world_xyz_fn = mock_world_xyz_fn
        self._stop_event = threading.Event()
        self._refresh_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._cached_color: Optional[np.ndarray] = None
        self._cached_overlay: Optional[np.ndarray] = None
        self._cached_depth: Optional[np.ndarray] = None
        self._cached_frame_idx: int = -1
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

    def has_cached_frame(self) -> bool:
        with self._frame_lock:
            return self._cached_color is not None

    def _update_frame_cache(
        self,
        color_bgr: np.ndarray,
        *,
        overlay_bgr: Optional[np.ndarray] = None,
        depth_raw: Optional[np.ndarray] = None,
        frame_idx: int = 0,
    ) -> None:
        with self._frame_lock:
            self._cached_color = np.ascontiguousarray(color_bgr, dtype=np.uint8).copy()
            self._cached_overlay = (
                None
                if overlay_bgr is None
                else np.ascontiguousarray(overlay_bgr, dtype=np.uint8).copy()
            )
            self._cached_depth = None if depth_raw is None else np.asarray(depth_raw).copy()
            self._cached_frame_idx = int(frame_idx)

    def save_cached_frames(
        self,
        out_dir: Path,
        *,
        extra_meta: Optional[dict[str, Any]] = None,
    ) -> Optional[Path]:
        with self._frame_lock:
            if self._cached_color is None:
                return None
            color = self._cached_color.copy()
            overlay = None if self._cached_overlay is None else self._cached_overlay.copy()
            depth = None if self._cached_depth is None else self._cached_depth.copy()
            frame_idx = int(self._cached_frame_idx)
        snap = self.snapshot()
        meta: dict[str, Any] = {
            "frame_idx": frame_idx,
            "label": snap.label,
            "confidence": float(snap.confidence),
            "status": snap.status_msg,
            "tracker_phase": snap.tracker_phase,
            "center_uv": snap.center_uv,
            "p_camera": snap.p_camera,
            "p_world": snap.p_world,
            "image_scale": float(snap.image_scale),
            "bbox_wh": list(snap.bbox_wh),
        }
        if extra_meta:
            meta.update(extra_meta)
        return save_perception_frame_bundle(
            out_dir=out_dir,
            color_bgr=color,
            overlay_bgr=overlay,
            depth_raw=depth,
            meta=meta,
        )

    def tracker_phase(self) -> str:
        return str(self.snapshot().tracker_phase)

    def track_ok_frames(self) -> int:
        return int(self.snapshot().track_ok_frames)

    def _preview_uv_overlay(self) -> tuple[Optional[tuple[float, float]], Optional[tuple[float, float]]]:
        target_uv: Optional[tuple[float, float]] = None
        if self._target_uv_fn is not None:
            try:
                tu, tv = self._target_uv_fn()
                target_uv = (float(tu), float(tv))
            except Exception:
                target_uv = None
        snap = self.snapshot()
        return target_uv, snap.center_uv

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
        if self._thread is not None and not self._thread.is_alive():
            self._thread = None
        self._stop_event.clear()
        self._refresh_event.clear()
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

    def stop(self, *, timeout_s: float = 5.0) -> bool:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=max(float(timeout_s), 0.1))
            if thread.is_alive():
                self._set_snapshot(running=True, status_msg="stopping")
                return False
        self._thread = None
        self._refresh_event.clear()
        self._set_snapshot(running=False, status_msg="stopped")
        return True

    def request_refresh(self) -> bool:
        if not self.is_running():
            return False
        self._refresh_event.set()
        self._set_snapshot(status_msg="refresh requested (YOLO)")
        return True

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
            from engine.vision.perception.visual_tracker import BboxTracker, detection_from_bbox  # type: ignore[import-not-found]
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
            print(f"[perception] start failed: {exc}")
            return
        except Exception as exc:
            self._set_snapshot(running=False, failed=True, status_msg=f"detector init failed: {exc}")
            print(f"[perception] detector init failed: {exc}")
            return

        target_label = str(detector_cfg.get("target_label", "") or "")
        enable_preview = bool(cfg.show_preview)
        publish_period = (1.0 / float(cfg.publish_hz)) if float(cfg.publish_hz) > 0 else 0.0
        mode = str(cfg.mode).strip().lower()
        pipeline_kind = self._normalize_pipeline(str(cfg.pipeline))
        use_search_track = pipeline_kind == "search_track"
        use_yolo_mask = pipeline_kind == "yolo_seg"

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
                elif use_yolo_mask:
                    self._run_mock_yolo_seg(
                        run_mock_frame=run_mock_frame,
                        publish_period=publish_period,
                        **common,
                    )
                else:
                    self._run_mock(
                        run_mock_frame=run_mock_frame,
                        **common,
                    )
            elif mode == "sim":
                from perception.sim_rendered_camera import SimRenderedCamera  # type: ignore[import-not-found]

                sim_cam_cls = lambda: SimRenderedCamera(  # noqa: E731
                    endpoint=str(cfg.sim_camera_port),
                    use_jpeg=bool(cfg.sim_camera_jpeg),
                )
                if use_search_track:
                    self._run_camera_search_track(
                        RealSenseCamera=sim_cam_cls,
                        publish_period=publish_period,
                        **common,
                    )
                elif use_yolo_mask:
                    self._run_camera_yolo_seg(
                        RealSenseCamera=sim_cam_cls,
                        publish_period=publish_period,
                        **common,
                    )
                else:
                    self._run_camera_yolo_seg(
                        RealSenseCamera=sim_cam_cls,
                        publish_period=publish_period,
                        **common,
                    )
            elif use_search_track:
                self._run_camera_search_track(
                    RealSenseCamera=RealSenseCamera,
                    publish_period=publish_period,
                    **common,
                )
            elif use_yolo_mask:
                self._run_camera_yolo_seg(
                    RealSenseCamera=RealSenseCamera,
                    publish_period=publish_period,
                    **common,
                )
            else:
                self._run_camera_yolo_seg(
                    RealSenseCamera=RealSenseCamera,
                    publish_period=publish_period,
                    **common,
                )
        except RealSenseUnavailableError as exc:
            self._set_snapshot(running=False, failed=True, status_msg=f"RealSense: {exc}")
        except Exception as exc:
            msg = str(exc)
            self._set_snapshot(running=False, failed=True, status_msg=msg)
            print(f"[perception] worker failed: {msg}")
        finally:
            if enable_preview:
                close_preview(_PREVIEW_WINDOW)

    def _is_mock_mode(self) -> bool:
        return str(self._config.mode).strip().lower() == "mock"

    def _resolve_mock_world(self, detector_cfg: dict[str, Any]) -> Optional[tuple[float, float, float]]:
        # Only bypass hand-eye with a fixed world pose in mock perception mode.
        if not self._is_mock_mode():
            return None
        if self._mock_world_xyz_fn is not None:
            try:
                raw = self._mock_world_xyz_fn()
                if raw is not None and len(raw) == 3:
                    return (float(raw[0]), float(raw[1]), float(raw[2]))
            except Exception:
                pass
        return _parse_mock_world_xyz(detector_cfg)

    def _fallback_p_camera(self, detector_cfg: dict) -> tuple[float, float, float]:
        snap = self.snapshot()
        if snap.p_camera is not None:
            return (
                float(snap.p_camera[0]),
                float(snap.p_camera[1]),
                float(snap.p_camera[2]),
            )
        z_nom = float(detector_cfg.get("z_nom_m", 0.35))
        return (0.0, 0.0, max(0.05, z_nom))

    def _effective_mask_erode_px(
        self,
        detector_cfg: dict[str, Any],
        *,
        det: Any,
        detection_scale_fn: Any,
        image_width: int,
        image_height: int,
    ) -> int:
        erode_px = int(detector_cfg.get("mask_erode_px", 0) or 0)
        if erode_px <= 0:
            return 0
        cfg = self._config
        prox = float(cfg.track_proximity_scale)
        if prox <= 1e-6:
            return erode_px
        scale = float(self.snapshot().image_scale)
        if scale <= 1e-6 and det is not None:
            scale = float(
                detection_scale_fn(det, image_width=int(image_width), image_height=int(image_height))
            )
        if scale >= prox:
            return int(max(0, int(cfg.track_proximity_mask_erode_px)))
        return erode_px

    def _refine_detection_for_publish(
        self,
        det: Any,
        detector_cfg: dict[str, Any],
        *,
        detection_scale_fn: Any,
        image_width: int,
        image_height: int,
    ) -> Any:
        erode_px = self._effective_mask_erode_px(
            detector_cfg,
            det=det,
            detection_scale_fn=detection_scale_fn,
            image_width=int(image_width),
            image_height=int(image_height),
        )
        if erode_px <= 0:
            return det
        _ensure_pick_place_path()
        from engine.vision.perception.detection_utils import refine_detection_mask  # type: ignore[import-not-found]

        return refine_detection_mask(det, erode_px=int(erode_px))

    @staticmethod
    def _world_from_sim_frame(frame: Any, p_camera: np.ndarray) -> Optional[tuple[float, float, float]]:
        origin = getattr(frame, "camera_world_origin", None)
        look = getattr(frame, "camera_world_look", None)
        right = getattr(frame, "camera_world_right", None)
        if origin is None or look is None or right is None:
            return None
        try:
            from engine.vision.sim_camera.pose import camera_point_to_world_from_axes

            p_w = camera_point_to_world_from_axes(origin, look, right, p_camera)
            return (float(p_w[0]), float(p_w[1]), float(p_w[2]))
        except Exception:
            return None

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
        depth_valid: bool = True,
        detector_cfg: Optional[dict[str, Any]] = None,
        scale_override: Optional[float] = None,
        frame: Any = None,
    ) -> Optional[tuple[float, float, float]]:
        p_cam = np.asarray(obs.p_camera_object, dtype=float).reshape(3)
        uv = normalized_center_uv_fn(det, image_width=image_width, image_height=image_height)
        scale = float(detection_scale_fn(det, image_width=image_width, image_height=image_height))
        if scale_override is not None:
            scale = float(max(scale, float(scale_override)))
        msg = str(status_msg)
        if not depth_valid:
            msg = f"{msg} | depth invalid (uv/scale only)"
        mock_world = self._resolve_mock_world(detector_cfg or {})
        if mock_world is not None:
            msg = f"{msg} | mock_world_xyz"
        else:
            sim_world = self._world_from_sim_frame(frame, p_cam) if frame is not None else None
            if sim_world is not None:
                msg = f"{msg} | sim_frame_pose"
            elif str(self._config.mode).strip().lower() == "sim" and not getattr(
                PerceptionCapture, "_warned_missing_sim_pose", False
            ):
                PerceptionCapture._warned_missing_sim_pose = True
                print(
                    "[perception] sim mode but camera frame has no pose metadata; "
                    "restart sim.py after update (world coords will use host FK until then)"
                )
        object_world = mock_world
        if object_world is None and frame is not None:
            object_world = self._world_from_sim_frame(frame, p_cam)
        cam_origin = getattr(frame, "camera_world_origin", None) if frame is not None else None
        cam_look = getattr(frame, "camera_world_look", None) if frame is not None else None
        cam_right = getattr(frame, "camera_world_right", None) if frame is not None else None
        p_world = self._publish_fn(
            object_camera_xyz=(float(p_cam[0]), float(p_cam[1]), float(p_cam[2])),
            label=str(obs.label),
            confidence=float(obs.confidence),
            image_center_uv=uv,
            image_scale=scale,
            depth_valid=bool(depth_valid),
            object_world=object_world,
            camera_world_origin=cam_origin,
            camera_world_look=cam_look,
            camera_world_right=cam_right,
        )
        x0, y0, x1, y1 = det.bbox_xyxy
        bbox_wh = (int(max(0, x1 - x0)), int(max(0, y1 - y0)))
        snap_extra: dict[str, Any] = dict(
            label=str(obs.label),
            confidence=float(obs.confidence),
            p_camera=(float(p_cam[0]), float(p_cam[1]), float(p_cam[2])),
            p_world=p_world,
            last_update_s=float(time.time()),
            status_msg=msg,
            failed=False,
            depth_valid=bool(depth_valid),
            image_scale=scale,
            bbox_wh=bbox_wh,
            center_uv=(float(uv[0]), float(uv[1])),
        )
        self._set_snapshot(**snap_extra)
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
        img_h, img_w = frame.color_bgr.shape[:2]
        det_raw = det
        det = self._refine_detection_for_publish(
            det,
            detector_cfg,
            detection_scale_fn=detection_scale_fn,
            image_width=int(img_w),
            image_height=int(img_h),
        )
        scale_raw = float(
            detection_scale_fn(det_raw, image_width=int(img_w), image_height=int(img_h))
        )
        scale_refined = float(
            detection_scale_fn(det, image_width=int(img_w), image_height=int(img_h))
        )
        scale_override = float(max(scale_raw, scale_refined))
        p_camera = measure_detection(
            det,
            depth_raw=frame.depth_raw,
            intrinsics=frame.intrinsics,
            depth_scale=frame.depth_scale,
            detector_cfg=detector_cfg,
        )
        depth_valid = p_camera is not None
        if not depth_valid:
            p_camera = np.asarray(self._fallback_p_camera(detector_cfg), dtype=float)
        obs = build_camera_observation(
            detection_label=det.label,
            confidence=det.confidence,
            p_camera_object=p_camera,
        )
        return self._publish_observation(
            obs=obs,
            det=det,
            image_width=img_w,
            image_height=img_h,
            detection_scale_fn=detection_scale_fn,
            normalized_center_uv_fn=normalized_center_uv_fn,
            status_msg=status_msg,
            depth_valid=depth_valid,
            detector_cfg=detector_cfg,
            scale_override=scale_override,
            frame=frame,
        )

    def _track_needs_redetect(
        self,
        *,
        track_ok: int,
        current_scale: float,
        bbox_area: int,
        init_bbox_area: int,
        last_scale: Optional[float],
        scale_stale_streak: int,
    ) -> tuple[bool, int]:
        cfg = self._config
        min_frames = max(1, int(cfg.track_watchdog_min_frames))
        if int(track_ok) < min_frames:
            return False, int(scale_stale_streak)
        stale_streak = int(scale_stale_streak)
        if float(current_scale) < float(cfg.track_scale_min):
            return True, stale_streak
        shrink_ratio = float(cfg.track_bbox_shrink_ratio)
        if int(init_bbox_area) > 0 and int(bbox_area) < int(init_bbox_area * shrink_ratio):
            return True, stale_streak
        eps = float(cfg.track_scale_stale_eps)
        if last_scale is not None and abs(float(current_scale) - float(last_scale)) < eps:
            stale_streak += 1
        else:
            stale_streak = 0
        if stale_streak >= max(1, int(cfg.track_redetect_stale_frames)):
            return True, stale_streak
        return False, stale_streak

    def _try_track_redetect(
        self,
        *,
        frame: Any,
        tracker: Any,
        detector: Any,
        target_label: str,
        list_frame_detections: Any,
        pick_target_detection: Any,
        detection_scale: Any,
        detection_init_bbox: Any,
        bbox_xyxy_area: Any,
        measure_detection: Any,
        build_camera_observation: Any,
        normalized_detection_center_uv: Any,
        detector_cfg: dict,
        img_w: int,
        img_h: int,
        current_scale: float,
        init_bbox_area: int,
        scale_stale_streak: int = 0,
    ) -> tuple[bool, int, Optional[Any], Optional[Any], str]:
        """YOLO refresh while staying in TRACK. Returns reinited, init_area, det, p_world, suffix."""
        cfg = self._config
        all_dets = list_frame_detections(detector, frame.color_bgr)
        yolo_det = pick_target_detection(all_dets, target_label)
        if yolo_det is None:
            return False, int(init_bbox_area), None, None, "redetect miss"
        new_scale = float(
            detection_scale(yolo_det, image_width=int(img_w), image_height=int(img_h))
        )
        min_scale = float(cfg.track_scale_min)
        grow_ratio = float(max(cfg.track_redetect_grow_ratio, 1.0))
        stale_need = max(1, int(cfg.track_redetect_stale_frames)) // 2
        if int(scale_stale_streak) >= stale_need:
            grow_ratio = float(max(cfg.track_redetect_grow_ratio_stale, 1.0))
        if new_scale <= max(float(current_scale) * grow_ratio, min_scale * 0.85):
            return False, int(init_bbox_area), None, None, "redetect skip (small)"
        init_bbox = detection_init_bbox(
            yolo_det,
            image_width=int(img_w),
            image_height=int(img_h),
            padding=float(cfg.track_init_bbox_padding),
        )
        if not tracker.init(frame.color_bgr, init_bbox):
            err = str(tracker.last_init_error).strip()
            suffix = "redetect init fail"
            if err:
                suffix += f": {err}"
            return False, int(init_bbox_area), None, None, suffix
        new_area = int(bbox_xyxy_area(init_bbox))
        p_world = self._process_detection(
            frame=frame,
            det=yolo_det,
            detector_cfg=detector_cfg,
            measure_detection=measure_detection,
            build_camera_observation=build_camera_observation,
            detection_scale_fn=detection_scale,
            normalized_center_uv_fn=normalized_detection_center_uv,
            status_msg=f"track redetect ({tracker.backend_name}) scale {new_scale:.3f}",
        )
        return True, new_area, yolo_det, p_world, f"redetect ok scale={new_scale:.3f}"

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

        _ensure_pick_place_path()
        from engine.vision.perception.detection_utils import bbox_xyxy_area, detection_init_bbox  # type: ignore[import-not-found]

        cfg = self._config
        lost_limit = max(1, int(cfg.track_lost_frames))
        reacquire = bool(cfg.reacquire_on_lost)
        tracker = _bbox_tracker_from_config(cfg)

        phase = TrackerPhase.SEARCH
        lost_streak = 0
        track_ok = 0
        frame_idx = 0
        tracked_label = target_label
        all_dets: list = []
        init_bbox_area = 0
        scale_stale_streak = 0
        last_scale: Optional[float] = None

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
                manual_refresh = self._refresh_event.is_set()
                if manual_refresh:
                    self._refresh_event.clear()
                    tracker.reset()
                    phase = TrackerPhase.SEARCH
                    lost_streak = 0
                    track_ok = 0
                    scale_stale_streak = 0
                    init_bbox_area = 0
                    last_scale = None
                    status = "refreshing (YOLO)"

                if phase == TrackerPhase.SEARCH:
                    all_dets = list_frame_detections(detector, frame.color_bgr)
                    yolo_det = pick_target_detection(all_dets, target_label)
                    if yolo_det is not None:
                        det = yolo_det
                        p_world = self._process_detection(
                            frame=frame,
                            det=det,
                            detector_cfg=detector_cfg,
                            measure_detection=measure_detection,
                            build_camera_observation=build_camera_observation,
                            detection_scale_fn=detection_scale,
                            normalized_center_uv_fn=normalized_detection_center_uv,
                            status_msg="yolo detected",
                        )
                        if p_world is not None:
                            p_camera = self.snapshot().p_camera
                        init_bbox = detection_init_bbox(
                            yolo_det,
                            image_width=img_w,
                            image_height=img_h,
                            padding=float(cfg.track_init_bbox_padding),
                        )
                        if tracker.init(frame.color_bgr, init_bbox):
                            tracked_label = str(yolo_det.label)
                            phase = TrackerPhase.TRACK
                            lost_streak = 0
                            track_ok = 1
                            init_bbox_area = int(bbox_xyxy_area(init_bbox))
                            scale_stale_streak = 0
                            last_scale = float(self.snapshot().image_scale)
                            status = f"track init ({tracker.backend_name})"
                        else:
                            err = str(tracker.last_init_error).strip()
                            status = "tracker init failed (yolo ok)"
                            if err:
                                status += f": {err}"
                    else:
                        status = "refresh miss" if manual_refresh else "searching"

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
                        current_scale = float(self.snapshot().image_scale)
                        bbox_area = int(bbox_xyxy_area(bbox))
                        need_redetect, scale_stale_streak = self._track_needs_redetect(
                            track_ok=int(track_ok),
                            current_scale=current_scale,
                            bbox_area=bbox_area,
                            init_bbox_area=int(init_bbox_area),
                            last_scale=last_scale,
                            scale_stale_streak=int(scale_stale_streak),
                        )
                        last_scale = current_scale
                        if need_redetect:
                            reinited, init_bbox_area, redet, redet_world, suffix = (
                                self._try_track_redetect(
                                    frame=frame,
                                    tracker=tracker,
                                    detector=detector,
                                    target_label=target_label,
                                    list_frame_detections=list_frame_detections,
                                    pick_target_detection=pick_target_detection,
                                    detection_scale=detection_scale,
                                    detection_init_bbox=detection_init_bbox,
                                    bbox_xyxy_area=bbox_xyxy_area,
                                    measure_detection=measure_detection,
                                    build_camera_observation=build_camera_observation,
                                    normalized_detection_center_uv=normalized_detection_center_uv,
                                    detector_cfg=detector_cfg,
                                    img_w=img_w,
                                    img_h=img_h,
                                    current_scale=current_scale,
                                    init_bbox_area=int(init_bbox_area),
                                    scale_stale_streak=int(scale_stale_streak),
                                )
                            )
                            if reinited:
                                track_ok = 1
                                scale_stale_streak = 0
                                last_scale = float(self.snapshot().image_scale)
                                if redet is not None:
                                    det = redet
                                if redet_world is not None:
                                    p_world = redet_world
                                    p_camera = self.snapshot().p_camera
                                status = f"tracking ({tracker.backend_name}) | {suffix}"
                            else:
                                status = f"{status} | {suffix}"
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

                snap = self.snapshot()
                target_uv, center_uv = self._preview_uv_overlay()
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
                    image_scale=float(snap.image_scale),
                    bbox_wh=tuple(snap.bbox_wh),
                    tracker_phase=str(phase.value),
                    tracker_backend=str(tracker.backend_name) if tracker.initialized else "",
                    target_uv=target_uv,
                    center_uv=center_uv,
                )
                self._update_frame_cache(
                    frame.color_bgr,
                    overlay_bgr=vis,
                    depth_raw=getattr(frame, "depth_raw", None),
                    frame_idx=frame_idx,
                )
                if show_preview:
                    key = show_preview_fn(_PREVIEW_WINDOW, vis)
                    if key in (ord("q"), 27):
                        self._set_snapshot(status_msg="preview quit")
                        break

                self._set_snapshot(
                    frame_idx=frame_idx,
                    status_msg=status,
                    tracker_phase=phase.value,
                    track_ok_frames=int(track_ok),
                    tracker_backend=str(tracker.backend_name) if tracker.initialized else "",
                )
                frame_idx += 1
                if publish_period > 0:
                    elapsed = time.time() - t0
                    sleep_s = max(0.0, publish_period - elapsed)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

        self._set_snapshot(running=False, status_msg="stopped")

    def _run_camera_yolo_seg(self, **kwargs: Any) -> None:
        """YOLO-seg every publish tick; CSRT coasts shifted YOLO mask when YOLO misses."""
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

        cfg = self._config
        lost_limit = max(1, int(cfg.track_lost_frames))
        reacquire = bool(cfg.reacquire_on_lost)
        aux_csrt = bool(cfg.track_aux_csrt)
        coast_max = max(1, int(cfg.track_coast_max_frames))
        tracker = kwargs.get("aux_tracker")
        if tracker is None and aux_csrt:
            tracker = _bbox_tracker_from_config(cfg)
        backend = "yolo-seg+csrt" if aux_csrt else "yolo-seg"

        _ensure_pick_place_path()
        from engine.vision.perception.detection_utils import (  # type: ignore[import-not-found]
            bbox_xyxy_area,
            detection_center_pixel,
            detection_init_bbox,
            detection_mask_coast_aligned,
        )
        from engine.vision.perception.visual_tracker import detection_from_bbox  # type: ignore[import-not-found]

        phase = TrackerPhase.SEARCH
        lost_streak = 0
        track_ok = 0
        coast_streak = 0
        frame_idx = 0
        all_dets: list = []
        last_good_det: Any = None
        last_anchor_center: Optional[tuple[float, float]] = None
        tracked_label = target_label
        init_bbox_area = 0
        last_scale: Optional[float] = None
        scale_stale_streak = 0

        self._set_snapshot(status_msg=f"camera live ({backend})", tracker_phase=phase.value)

        with RealSenseCamera() as cam:
            while not self._stop_event.is_set():
                t0 = time.time()
                manual_refresh = self._refresh_event.is_set()
                if manual_refresh:
                    self._refresh_event.clear()
                    if tracker is not None:
                        tracker.reset()
                    phase = TrackerPhase.SEARCH
                    lost_streak = 0
                    track_ok = 0
                    coast_streak = 0
                    last_good_det = None
                    last_anchor_center = None
                    init_bbox_area = 0
                    last_scale = None
                    scale_stale_streak = 0

                frame = cam.capture()
                img_h, img_w = frame.color_bgr.shape[:2]
                if phase == TrackerPhase.LOST and reacquire:
                    phase = TrackerPhase.SEARCH
                all_dets = list_frame_detections(detector, frame.color_bgr)
                yolo_det = pick_target_detection(all_dets, target_label)
                det = yolo_det
                status = "searching"
                p_camera = None
                p_world = None

                if yolo_det is not None:
                    lost_streak = 0
                    coast_streak = 0
                    track_ok += 1
                    phase = TrackerPhase.TRACK
                    last_good_det = yolo_det
                    last_anchor_center = detection_center_pixel(
                        yolo_det, image_width=img_w, image_height=img_h
                    )
                    tracked_label = str(yolo_det.label)
                    publish_det = yolo_det
                    if aux_csrt and tracker is not None:
                        init_bbox = detection_init_bbox(
                            yolo_det,
                            image_width=img_w,
                            image_height=img_h,
                            padding=float(cfg.track_init_bbox_padding),
                        )
                        csrt_bbox: Optional[tuple[int, int, int, int]] = None
                        if not tracker.initialized:
                            if tracker.init(frame.color_bgr, init_bbox):
                                init_bbox_area = int(bbox_xyxy_area(init_bbox))
                                csrt_bbox = init_bbox
                        else:
                            csrt_bbox = tracker.update(frame.color_bgr)
                            if csrt_bbox is None:
                                if tracker.init(frame.color_bgr, init_bbox):
                                    init_bbox_area = int(bbox_xyxy_area(init_bbox))
                                    csrt_bbox = init_bbox
                            elif int(bbox_xyxy_area(init_bbox)) > int(
                                bbox_xyxy_area(csrt_bbox) * 1.12
                            ):
                                if tracker.init(frame.color_bgr, init_bbox):
                                    init_bbox_area = int(bbox_xyxy_area(init_bbox))
                                    csrt_bbox = init_bbox
                        if csrt_bbox is not None and last_anchor_center is not None:
                            yolo_scale = float(
                                detection_scale(
                                    yolo_det, image_width=int(img_w), image_height=int(img_h)
                                )
                            )
                            merged = detection_mask_coast_aligned(
                                yolo_det,
                                csrt_bbox=csrt_bbox,
                                anchor_center=last_anchor_center,
                                image_width=int(img_w),
                                image_height=int(img_h),
                            )
                            if merged is not None:
                                merged_scale = float(
                                    detection_scale(
                                        merged,
                                        image_width=int(img_w),
                                        image_height=int(img_h),
                                    )
                                )
                                if merged_scale > yolo_scale * 1.01:
                                    publish_det = merged
                            else:
                                csrt_det = detection_from_bbox(
                                    csrt_bbox,
                                    image_width=int(img_w),
                                    image_height=int(img_h),
                                    label=tracked_label,
                                    confidence=0.95,
                                )
                                csrt_scale = float(
                                    detection_scale(
                                        csrt_det,
                                        image_width=int(img_w),
                                        image_height=int(img_h),
                                    )
                                )
                                if csrt_scale > yolo_scale * 1.01:
                                    publish_det = csrt_det
                    p_world = self._process_detection(
                        frame=frame,
                        det=publish_det,
                        detector_cfg=detector_cfg,
                        measure_detection=measure_detection,
                        build_camera_observation=build_camera_observation,
                        detection_scale_fn=detection_scale,
                        normalized_center_uv_fn=normalized_detection_center_uv,
                        status_msg="yolo-seg detected",
                    )
                    status = "yolo-seg track" if p_world is not None else "yolo-seg (no depth)"
                    if p_world is not None:
                        p_camera = self.snapshot().p_camera
                    last_scale = float(self.snapshot().image_scale)
                    scale_stale_streak = 0
                elif (
                    aux_csrt
                    and tracker is not None
                    and tracker.initialized
                    and last_good_det is not None
                    and last_anchor_center is not None
                    and phase == TrackerPhase.TRACK
                ):
                    bbox = tracker.update(frame.color_bgr)
                    if bbox is not None:
                        bx0, by0, bx1, by1 = bbox
                        csrt_cx = 0.5 * (float(bx0) + float(bx1))
                        csrt_cy = 0.5 * (float(by0) + float(by1))
                        ax, ay = last_anchor_center
                        shifted = detection_mask_coast_aligned(
                            last_good_det,
                            csrt_bbox=bbox,
                            anchor_center=(ax, ay),
                            image_width=img_w,
                            image_height=img_h,
                        )
                        if shifted is not None:
                            lost_streak = 0
                            track_ok += 1
                            coast_streak += 1
                            det = shifted
                            coast_status = f"yolo-seg coast ({coast_streak}/{coast_max})"
                            p_world = self._process_detection(
                                frame=frame,
                                det=shifted,
                                detector_cfg=detector_cfg,
                                measure_detection=measure_detection,
                                build_camera_observation=build_camera_observation,
                                detection_scale_fn=detection_scale,
                                normalized_center_uv_fn=normalized_detection_center_uv,
                                status_msg=coast_status,
                            )
                            status = coast_status if p_world is not None else f"{coast_status} (no depth)"
                            if p_world is not None:
                                p_camera = self.snapshot().p_camera
                            current_scale = float(self.snapshot().image_scale)
                            need_redetect, scale_stale_streak = self._track_needs_redetect(
                                track_ok=int(track_ok),
                                current_scale=current_scale,
                                bbox_area=int(bbox_xyxy_area(bbox)),
                                init_bbox_area=int(init_bbox_area),
                                last_scale=last_scale,
                                scale_stale_streak=int(scale_stale_streak),
                            )
                            last_scale = current_scale
                            if coast_streak >= coast_max or need_redetect:
                                reinited, init_bbox_area, redet, redet_world, suffix = (
                                    self._try_track_redetect(
                                        frame=frame,
                                        tracker=tracker,
                                        detector=detector,
                                        target_label=target_label,
                                        list_frame_detections=list_frame_detections,
                                        pick_target_detection=pick_target_detection,
                                        detection_scale=detection_scale,
                                        detection_init_bbox=detection_init_bbox,
                                        bbox_xyxy_area=bbox_xyxy_area,
                                        measure_detection=measure_detection,
                                        build_camera_observation=build_camera_observation,
                                        normalized_detection_center_uv=normalized_detection_center_uv,
                                        detector_cfg=detector_cfg,
                                        img_w=img_w,
                                        img_h=img_h,
                                        current_scale=current_scale,
                                        init_bbox_area=int(init_bbox_area),
                                        scale_stale_streak=int(scale_stale_streak),
                                    )
                                )
                                if reinited:
                                    coast_streak = 0
                                    scale_stale_streak = 0
                                    last_scale = float(self.snapshot().image_scale)
                                    if redet is not None:
                                        det = redet
                                        last_good_det = redet
                                        last_anchor_center = detection_center_pixel(
                                            redet, image_width=img_w, image_height=img_h
                                        )
                                    if redet_world is not None:
                                        p_world = redet_world
                                        p_camera = self.snapshot().p_camera
                                    status = f"{status} | {suffix}"
                                elif coast_streak >= coast_max:
                                    lost_streak += 1
                                    coast_streak = 0
                                    track_ok = 0
                                    if lost_streak >= lost_limit:
                                        phase = TrackerPhase.LOST if reacquire else TrackerPhase.SEARCH
                                        status = "lost" if phase == TrackerPhase.LOST else "searching"
                                        if tracker is not None:
                                            tracker.reset()
                                    else:
                                        status = f"coast expired ({lost_streak}/{lost_limit})"
                        else:
                            lost_streak += 1
                            track_ok = 0
                            coast_streak = 0
                            if lost_streak >= lost_limit:
                                phase = TrackerPhase.LOST if reacquire else TrackerPhase.SEARCH
                                status = "lost" if phase == TrackerPhase.LOST else "searching"
                                if tracker is not None:
                                    tracker.reset()
                            else:
                                status = f"coast shift fail ({lost_streak}/{lost_limit})"
                    else:
                        lost_streak += 1
                        track_ok = 0
                        coast_streak = 0
                        if lost_streak >= lost_limit:
                            phase = TrackerPhase.LOST if reacquire else TrackerPhase.SEARCH
                            status = "lost" if phase == TrackerPhase.LOST else "searching"
                            if tracker is not None:
                                tracker.reset()
                        else:
                            status = f"csrt lost ({lost_streak}/{lost_limit})"
                else:
                    lost_streak += 1
                    track_ok = 0
                    coast_streak = 0
                    if lost_streak >= lost_limit:
                        phase = TrackerPhase.LOST if reacquire else TrackerPhase.SEARCH
                        status = "lost" if phase == TrackerPhase.LOST else "searching"
                        if tracker is not None:
                            tracker.reset()
                    elif phase == TrackerPhase.TRACK:
                        status = f"track miss ({lost_streak}/{lost_limit})"

                if manual_refresh and yolo_det is None:
                    status = "refresh miss"

                target_uv, center_uv = self._preview_uv_overlay()
                snap = self.snapshot()
                vis = draw_detection_overlay(
                    frame.color_bgr,
                    det,
                    status=status,
                    target_label=target_label,
                    frame_idx=frame_idx,
                    p_camera=p_camera if p_camera is not None else snap.p_camera,
                    p_world=np.asarray(p_world) if p_world is not None else None,
                    all_detections=all_dets,
                    model_classes=model_class_names(detector),
                    image_scale=float(snap.image_scale),
                    bbox_wh=tuple(snap.bbox_wh),
                    tracker_phase=str(phase.value),
                    tracker_backend=(
                        str(tracker.backend_name)
                        if tracker is not None and tracker.initialized
                        else backend
                    ),
                    target_uv=target_uv,
                    center_uv=center_uv,
                )
                self._update_frame_cache(
                    frame.color_bgr,
                    overlay_bgr=vis,
                    depth_raw=getattr(frame, "depth_raw", None),
                    frame_idx=frame_idx,
                )
                if show_preview:
                    key = show_preview_fn(_PREVIEW_WINDOW, vis)
                    if key in (ord("q"), 27):
                        self._set_snapshot(status_msg="preview quit")
                        break

                self._set_snapshot(
                    frame_idx=frame_idx,
                    status_msg=status,
                    tracker_phase=phase.value,
                    track_ok_frames=int(track_ok),
                    tracker_backend=(
                        str(tracker.backend_name)
                        if tracker is not None and tracker.initialized
                        else backend
                    ),
                    failed=phase == TrackerPhase.LOST and not reacquire,
                )
                frame_idx += 1
                if publish_period > 0:
                    elapsed = time.time() - t0
                    sleep_s = max(0.0, publish_period - elapsed)
                    if sleep_s > 0:
                        time.sleep(sleep_s)

        self._set_snapshot(running=False, status_msg="stopped")

    def _run_camera_yolo_only(self, **kwargs: Any) -> None:
        """Legacy alias: same as yolo_seg (mask every frame + TRACK semantics)."""
        self._run_camera_yolo_seg(**kwargs)

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
        target_uv, center_uv = self._preview_uv_overlay()
        vis = draw_detection_overlay(
            color,
            det,
            status="detected",
            target_label=target_label,
            frame_idx=0,
            p_camera=self.snapshot().p_camera,
            p_world=np.asarray(p_world) if p_world is not None else None,
            target_uv=target_uv,
            center_uv=center_uv,
        )
        self._update_frame_cache(color, overlay_bgr=vis, depth_raw=depth, frame_idx=0)
        if show_preview and p_world is not None:
            show_preview_fn(_PREVIEW_WINDOW, vis)
            time.sleep(0.05)
        self._set_snapshot(running=False, status_msg="mock done")

    def _run_mock_yolo_seg(self, **kwargs: Any) -> None:
        """Mock loop: YOLO/mask every tick; optional CSRT coast on detector miss."""
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
        publish_period = float(kwargs.get("publish_period", 0.0) or 0.0)
        mock_tracker = kwargs.get("mock_tracker")

        color, depth, intrinsics, depth_scale = run_mock_frame(detector_cfg)
        img_h, img_w = color.shape[:2]

        class _MockFrame:
            pass

        mf = _MockFrame()
        mf.color_bgr = color
        mf.depth_raw = depth
        mf.intrinsics = intrinsics
        mf.depth_scale = depth_scale

        cfg = self._config
        lost_limit = max(1, int(cfg.track_lost_frames))
        aux_csrt = bool(cfg.track_aux_csrt)
        coast_max = max(1, int(cfg.track_coast_max_frames))
        backend = "yolo-seg+csrt" if aux_csrt else "yolo-seg"

        _ensure_pick_place_path()
        from engine.vision.perception.detection_utils import (  # type: ignore[import-not-found]
            detection_center_pixel,
            detection_init_bbox,
            detection_mask_translated,
        )

        phase = TrackerPhase.SEARCH
        lost_streak = 0
        track_ok = 0
        coast_streak = 0
        frame_idx = 0
        last_good_det: Any = None
        last_anchor_center: Optional[tuple[float, float]] = None
        tracker = mock_tracker

        self._set_snapshot(status_msg=f"mock {backend}", tracker_phase=phase.value)

        while not self._stop_event.is_set():
            t0 = time.time()
            yolo_det = detector.detect(color)
            det = yolo_det
            status = "searching"
            p_camera = None
            p_world = None

            if yolo_det is not None:
                lost_streak = 0
                coast_streak = 0
                track_ok += 1
                phase = TrackerPhase.TRACK
                last_good_det = yolo_det
                last_anchor_center = detection_center_pixel(
                    yolo_det, image_width=img_w, image_height=img_h
                )
                if aux_csrt and tracker is not None:
                    init_bbox = detection_init_bbox(
                        yolo_det,
                        image_width=img_w,
                        image_height=img_h,
                        padding=float(cfg.track_init_bbox_padding),
                    )
                    tracker.init(color, init_bbox)
                p_world = self._process_detection(
                    frame=mf,
                    det=yolo_det,
                    detector_cfg=detector_cfg,
                    measure_detection=measure_detection,
                    build_camera_observation=build_camera_observation,
                    detection_scale_fn=detection_scale,
                    normalized_center_uv_fn=normalized_detection_center_uv,
                    status_msg="mock yolo-seg",
                )
                status = "mock yolo-seg track"
                if p_world is not None:
                    p_camera = self.snapshot().p_camera
            elif (
                aux_csrt
                and tracker is not None
                and getattr(tracker, "initialized", False)
                and last_good_det is not None
                and last_anchor_center is not None
                and phase == TrackerPhase.TRACK
            ):
                bbox = tracker.update(color)
                if bbox is not None:
                    bx0, by0, bx1, by1 = bbox
                    csrt_cx = 0.5 * (float(bx0) + float(bx1))
                    csrt_cy = 0.5 * (float(by0) + float(by1))
                    ax, ay = last_anchor_center
                    shifted = detection_mask_translated(
                        last_good_det,
                        dx=int(round(csrt_cx - ax)),
                        dy=int(round(csrt_cy - ay)),
                        image_width=img_w,
                        image_height=img_h,
                    )
                    if shifted is not None:
                        lost_streak = 0
                        track_ok += 1
                        coast_streak += 1
                        det = shifted
                        coast_status = f"mock coast ({coast_streak}/{coast_max})"
                        p_world = self._process_detection(
                            frame=mf,
                            det=shifted,
                            detector_cfg=detector_cfg,
                            measure_detection=measure_detection,
                            build_camera_observation=build_camera_observation,
                            detection_scale_fn=detection_scale,
                            normalized_center_uv_fn=normalized_detection_center_uv,
                            status_msg=coast_status,
                        )
                        status = coast_status
                        if p_world is not None:
                            p_camera = self.snapshot().p_camera
                        if coast_streak >= coast_max:
                            lost_streak += 1
                            coast_streak = 0
                            track_ok = 0
                    else:
                        lost_streak += 1
                        track_ok = 0
                        coast_streak = 0
                else:
                    lost_streak += 1
                    track_ok = 0
                    coast_streak = 0
                if lost_streak >= lost_limit:
                    phase = TrackerPhase.SEARCH
                    if tracker is not None:
                        tracker.reset()
                    status = "mock searching"
            else:
                lost_streak += 1
                track_ok = 0
                coast_streak = 0
                if lost_streak >= lost_limit:
                    phase = TrackerPhase.SEARCH
                status = "mock searching"

            target_uv, center_uv = self._preview_uv_overlay()
            vis = draw_detection_overlay(
                color,
                det,
                status=status,
                target_label=target_label,
                frame_idx=frame_idx,
                p_camera=p_camera,
                p_world=np.asarray(p_world) if p_world is not None else None,
                target_uv=target_uv,
                center_uv=center_uv,
                tracker_phase=str(phase.value),
                tracker_backend=(
                    str(getattr(tracker, "backend_name", backend))
                    if tracker is not None and getattr(tracker, "initialized", False)
                    else backend
                ),
            )
            self._update_frame_cache(
                color,
                overlay_bgr=vis,
                depth_raw=depth,
                frame_idx=frame_idx,
            )
            if show_preview:
                key = show_preview_fn(_PREVIEW_WINDOW, vis)
                if key in (ord("q"), 27):
                    break

            self._set_snapshot(
                frame_idx=frame_idx,
                status_msg=status,
                tracker_phase=phase.value,
                track_ok_frames=int(track_ok),
                tracker_backend=(
                    str(getattr(tracker, "backend_name", backend))
                    if tracker is not None and getattr(tracker, "initialized", False)
                    else backend
                ),
            )
            frame_idx += 1
            if publish_period > 0:
                time.sleep(max(0.0, publish_period - (time.time() - t0)))
            else:
                time.sleep(0.05)

        self._set_snapshot(running=False, status_msg="stopped")

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

        _ensure_pick_place_path()
        from engine.vision.perception.detection_utils import detection_init_bbox  # type: ignore[import-not-found]

        img_h, img_w = color.shape[:2]
        tracker = _bbox_tracker_from_config(self._config)
        init_bbox = detection_init_bbox(
            det,
            image_width=img_w,
            image_height=img_h,
            padding=float(self._config.track_init_bbox_padding),
        )
        if not tracker.init(color, init_bbox):
            self._set_snapshot(running=False, failed=True, status_msg="mock: tracker init failed")
            return
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
            target_uv, center_uv = self._preview_uv_overlay()
            vis = draw_detection_overlay(
                color,
                det_track or det,
                status="mock tracking",
                target_label=target_label,
                frame_idx=frame_idx,
                p_camera=self.snapshot().p_camera,
                target_uv=target_uv,
                center_uv=center_uv,
            )
            self._update_frame_cache(
                color,
                overlay_bgr=vis,
                depth_raw=depth,
                frame_idx=frame_idx,
            )
            if show_preview:
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
