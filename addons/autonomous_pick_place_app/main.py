#!/usr/bin/env python3
"""Detect object 3D position in camera frame; elesim converts to world using mount config."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

_APP_ROOT = Path(__file__).resolve().parent
if str(_APP_ROOT) not in sys.path:
    sys.path.insert(0, str(_APP_ROOT))

from observation import CameraObservation
from perception.depth_pose import CameraIntrinsics, estimate_object_position_camera
from perception.detector import DetectionResult, ObjectDetector, create_detector, load_detector_config
from perception.preview import close_preview, draw_detection_overlay, show_preview
from perception.realsense_camera import RealSenseCamera, RealSenseUnavailableError
from perception.target_tracker import TargetTracker, TrackPacket
from perception.yolo_detector import YoloUnavailableError

from elesim_bridge.host_client import HostPublishError, publish_perceived_object

_DEFAULT_HOST_ENDPOINT = "tcp://127.0.0.1:5555"

_PREVIEW_WINDOW = "autonomous_pick_place"


def _format_vec(v: np.ndarray) -> str:
    a = np.asarray(v, dtype=float).reshape(3)
    return f"[{a[0]:+.4f}, {a[1]:+.4f}, {a[2]:+.4f}]"


def _format_bbox(bbox: tuple[int, int, int, int]) -> str:
    x0, y0, x1, y1 = bbox
    return f"[{x0}, {y0}, {x1}, {y1}]"


def _list_frame_detections(detector: ObjectDetector, color_bgr: np.ndarray) -> list[DetectionResult]:
    list_fn = getattr(detector, "list_detections", None)
    if callable(list_fn):
        return list(list_fn(color_bgr))
    det = detector.detect(color_bgr)
    return [det] if det is not None else []


def _pick_target_detection(
    dets: list[DetectionResult],
    target_label: str,
) -> Optional[DetectionResult]:
    if not dets:
        return None
    key = target_label.strip().lower()
    if not key:
        return max(dets, key=lambda d: float(d.confidence))
    matches = [d for d in dets if d.label.strip().lower() == key]
    if not matches:
        return None
    return max(matches, key=lambda d: float(d.confidence))


def _model_class_names(detector: ObjectDetector) -> list[str]:
    names = getattr(detector, "class_names", None)
    if names is None:
        return []
    return [str(x) for x in names]


def _format_detection_summary(dets: list[DetectionResult]) -> str:
    return ", ".join(f"{d.label}@{float(d.confidence):.2f}" for d in dets)


def build_camera_observation(
    *,
    detection_label: str,
    confidence: float,
    p_camera_object: np.ndarray,
) -> CameraObservation:
    return CameraObservation(
        label=detection_label,
        confidence=float(confidence),
        p_camera_object=np.asarray(p_camera_object, dtype=float).reshape(3),
        timestamp=time.time(),
    )


def resolve_detector_cfg(
    file_cfg: dict[str, Any],
    *,
    detector_cli: str,
    target_label_cli: str | None,
    yolo_device_cli: str | None,
    mode: str,
) -> dict[str, Any]:
    cfg = dict(file_cfg)
    det = str(detector_cli).strip().lower()

    if det == "yolo":
        cfg["type"] = "yolo"
    elif det != "config":
        cfg["type"] = det

    if target_label_cli:
        cfg["target_label"] = str(target_label_cli).strip()

    if yolo_device_cli is not None and str(yolo_device_cli).strip() != "":
        cfg["device"] = str(yolo_device_cli).strip()

    if mode == "mock" and str(cfg.get("type", "")).lower() == "yolo":
        fallback = str(cfg.get("mock_fallback_type", "mock_center")).strip().lower() or "mock_center"
        print(f"[Detector] mock mode: YOLO disabled, using '{fallback}' detector")
        cfg["type"] = fallback

    return cfg


def run_mock_frame(detector_cfg: dict) -> tuple[np.ndarray, np.ndarray, CameraIntrinsics, float]:
    import cv2

    w, h = 640, 480
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:, :] = (40, 120, 40)
    cx, cy = w // 2, h // 2
    cv2.circle(color, (cx, cy), 40, (0, 0, 220), -1)

    intrinsics = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0, width=w, height=h)
    depth_scale = 0.001
    z_m = float(detector_cfg.get("mock_depth_m", 0.65))
    depth_raw = np.zeros((h, w), dtype=np.uint16)
    depth_raw[:, :] = int(round(z_m / depth_scale))
    return color, depth_raw, intrinsics, depth_scale


def measure_detection(
    det: DetectionResult,
    *,
    depth_raw: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float,
    detector_cfg: dict[str, Any],
) -> Optional[np.ndarray]:
    try:
        return estimate_object_position_camera(
            det.mask,
            depth_raw,
            intrinsics,
            depth_scale,
            z_min_m=float(detector_cfg.get("z_min_m", 0.15)),
            z_max_m=float(detector_cfg.get("z_max_m", 2.5)),
        )
    except RuntimeError:
        return None


def print_observation(obs: CameraObservation, *, det: DetectionResult) -> None:
    print(f"[Detector] label={obs.label} confidence={obs.confidence:.3f} bbox={_format_bbox(det.bbox_xyxy)}")
    print(f"[Camera]   p_camera_object = {_format_vec(obs.p_camera_object)} m  (optical: x=right y=down z=look)")


def _print_world_object(p_world: tuple[float, float, float] | None) -> None:
    if p_world is None:
        return
    print(f"[World]  p_world_object = {_format_vec(np.asarray(p_world))} m")


def _track_payload_from_packet(packet: TrackPacket) -> dict[str, Any]:
    out: dict[str, Any] = {
        "track_state": str(packet.track_state),
        "track_confidence": float(packet.track_confidence),
        "bbox_xyxy": list(packet.bbox_xyxy),
        "center_uv": list(packet.center_uv),
        "depth_valid_ratio": float(packet.depth_valid_ratio),
        "lost_count": int(packet.lost_count),
    }
    if packet.mu_camera is not None:
        mu = np.asarray(packet.mu_camera, dtype=float).reshape(3)
        out["mu_camera"] = [float(mu[0]), float(mu[1]), float(mu[2])]
    if packet.sigma_camera is not None:
        sig = np.asarray(packet.sigma_camera, dtype=float).reshape(3)
        out["sigma_camera"] = [float(sig[0]), float(sig[1]), float(sig[2])]
    return out


def _publish_point_for_packet(packet: TrackPacket) -> Optional[np.ndarray]:
    if packet.p_camera is not None and np.all(np.isfinite(packet.p_camera)):
        return np.asarray(packet.p_camera, dtype=float).reshape(3)
    if packet.mu_camera is not None and np.all(np.isfinite(packet.mu_camera)):
        return np.asarray(packet.mu_camera, dtype=float).reshape(3)
    return None


def _print_track_packet(packet: TrackPacket) -> None:
    p = _publish_point_for_packet(packet)
    p_txt = _format_vec(p) if p is not None else "-"
    mu_txt = _format_vec(packet.mu_camera) if packet.mu_camera is not None else "-"
    sig_txt = _format_vec(packet.sigma_camera) if packet.sigma_camera is not None else "-"
    print(
        f"[Track] state={packet.track_state} conf={packet.track_confidence:.2f} "
        f"depth_valid={packet.depth_valid_ratio:.2f} lost={packet.lost_count} "
        f"p_cam={p_txt} mu={mu_txt} sigma={sig_txt}"
    )


def run_camera_session(
    *,
    detector: ObjectDetector,
    detector_cfg: dict[str, Any],
    verbose_debug: bool,
    show: bool,
    until_found: bool,
    stop_on_detect: bool,
    publish_host: bool,
    host_endpoint: str,
    publish_hz: float,
) -> int:
    import cv2

    target_label = str(detector_cfg.get("target_label", "") or "")
    model_classes = _model_class_names(detector)
    publish_period = (1.0 / float(publish_hz)) if float(publish_hz) > 0 else 0.0
    frame_idx = 0
    found_once = False
    last_det_summary = ""
    tracker = TargetTracker.from_config(detector_cfg)
    h_img = 480
    w_img = 640

    if show:
        cv2.namedWindow(_PREVIEW_WINDOW, cv2.WINDOW_NORMAL)

    print(
        "[Camera] live capture started (YOLO lock + ROI depth tracker)"
        + (" (retry until detection)" if until_found else " (exit if first frame has no target)")
        + ("; stop after first detection" if stop_on_detect else "; runs until q/ESC")
        + ("; preview: q/ESC to quit" if show else "")
    )

    try:
        with RealSenseCamera() as cam:
            while True:
                t0 = time.time()
                frame = cam.capture()
                h_img = int(frame.intrinsics.height)
                w_img = int(frame.intrinsics.width)

                det: Optional[DetectionResult] = None
                all_dets: list[DetectionResult] = []
                if tracker.needs_yolo():
                    all_dets = _list_frame_detections(detector, frame.color_bgr)
                    det = _pick_target_detection(all_dets, target_label)
                    if det is not None:
                        if tracker.try_lock(det, width=w_img, height=h_img):
                            print(
                                f"[Track] YOLO lock label={det.label} bbox={_format_bbox(det.bbox_xyxy)} "
                                f"-> TRACKING_3D"
                            )

                packet = tracker.update(
                    depth_raw=frame.depth_raw,
                    intrinsics=frame.intrinsics,
                    depth_scale=frame.depth_scale,
                )
                status = str(packet.track_state).lower()
                p_camera = _publish_point_for_packet(packet)

                if tracker.needs_yolo():
                    det_summary = _format_detection_summary(all_dets)
                    if det_summary != last_det_summary:
                        if all_dets:
                            print(f"[Detector] frame={frame_idx} detections: {det_summary}")
                        elif frame_idx == 0 or last_det_summary:
                            print(f"[Detector] frame={frame_idx} no detections above confidence")
                        if target_label and all_dets and det is None:
                            print(
                                f"[Detector] target {target_label!r} not in frame "
                                f"(model classes: {', '.join(model_classes[:20])}"
                                + ("..." if len(model_classes) > 20 else "")
                                + ")"
                            )
                        last_det_summary = det_summary

                if show:
                    vis_det = det
                    if vis_det is None and packet.bbox_xyxy != (0, 0, 0, 0):
                        x0, y0, x1, y1 = packet.bbox_xyxy
                        mask = np.zeros((h_img, w_img), dtype=np.uint8)
                        mask[y0 : y1 + 1, x0 : x1 + 1] = 255
                        vis_det = DetectionResult(
                            mask=mask,
                            bbox_xyxy=packet.bbox_xyxy,
                            label=str(packet.label or target_label or "track"),
                            confidence=float(packet.track_confidence),
                        )
                    vis = draw_detection_overlay(
                        frame.color_bgr,
                        vis_det,
                        status=f"{status} conf={packet.track_confidence:.2f}",
                        target_label=target_label,
                        frame_idx=frame_idx,
                        p_camera=p_camera,
                        p_world=None,
                        all_detections=all_dets,
                        model_classes=model_classes,
                    )
                    key = show_preview(_PREVIEW_WINDOW, vis)
                    if key in (ord("q"), 27):
                        print("[Camera] quit by user")
                        return 0

                if packet.should_publish_to_host and p_camera is not None:
                    if not found_once:
                        print("[Camera] tracker active (publish)")
                        found_once = True
                    _print_track_packet(packet)
                    if verbose_debug:
                        print(f"[Debug]  frame={frame_idx}")
                    if publish_host:
                        try:
                            p_world = publish_perceived_object(
                                endpoint=host_endpoint,
                                object_camera_xyz=p_camera,
                                label=str(packet.label or target_label),
                                track=_track_payload_from_packet(packet),
                            )
                            _print_world_object(p_world)
                        except HostPublishError as exc:
                            print(f"[Host] publish warning: {exc}", file=sys.stderr)
                    if stop_on_detect:
                        return 0

                if not until_found and not found_once:
                    msg = "[Detector] no object in frame"
                    if target_label:
                        msg += f" (target_label={target_label!r})"
                    if all_dets:
                        msg += f"; saw: {_format_detection_summary(all_dets)}"
                    print(msg, file=sys.stderr)
                    return 1

                frame_idx += 1
                if publish_period > 0:
                    elapsed = time.time() - t0
                    sleep_s = max(0.0, publish_period - elapsed)
                    if sleep_s > 0:
                        time.sleep(sleep_s)
    finally:
        if show:
            close_preview(_PREVIEW_WINDOW)

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Perceive object 3D position in camera frame; elesim sim converts to world."
    )
    parser.add_argument("--detector-config", required=True, help="detector JSON")
    parser.add_argument(
        "--detector",
        choices=("config", "yolo", "hsv", "roi", "mock", "mock_center"),
        default="config",
        help="Detector backend; 'config' uses JSON type field (default)",
    )
    parser.add_argument(
        "--target-label",
        default=None,
        help="YOLO class name filter (e.g. white_cup). Overrides JSON target_label.",
    )
    parser.add_argument(
        "--yolo-device",
        default=None,
        help="YOLO GPU device: 0|1|2, cuda:1, or cpu. Overrides JSON 'device'.",
    )
    parser.add_argument(
        "--mode",
        choices=("camera", "mock"),
        default="mock",
        help="camera: RealSense D435i; mock: synthetic frame",
    )
    parser.add_argument(
        "--verbose-debug",
        action="store_true",
        help="Print extra debug fields",
    )
    parser.add_argument(
        "--publish-host",
        action="store_true",
        help="Send p_camera_object to host; host uses hand_eye_config for world marker",
    )
    parser.add_argument(
        "--host-endpoint",
        default=_DEFAULT_HOST_ENDPOINT,
        help=f"host.py ZMQ ROUTER address (default: {_DEFAULT_HOST_ENDPOINT})",
    )
    parser.add_argument(
        "--stop-on-detect",
        action="store_true",
        help="Exit after first successful detection (default: keep running)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Exit on first frame with no target (default camera: retry until found)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Disable OpenCV preview window",
    )
    parser.add_argument(
        "--publish-hz",
        type=float,
        default=10.0,
        help="Max camera loop rate when detecting (Hz)",
    )
    args = parser.parse_args()

    file_cfg = load_detector_config(args.detector_config)
    detector_cfg = resolve_detector_cfg(
        file_cfg,
        detector_cli=args.detector,
        target_label_cli=args.target_label,
        yolo_device_cli=args.yolo_device,
        mode=args.mode,
    )

    host_endpoint = str(args.host_endpoint).strip() or _DEFAULT_HOST_ENDPOINT
    if args.publish_host:
        print(f"[Host] publish endpoint: {host_endpoint}")

    try:
        detector = create_detector(detector_cfg)

        if args.mode == "camera":
            return run_camera_session(
                detector=detector,
                detector_cfg=detector_cfg,
                verbose_debug=args.verbose_debug,
                show=not args.no_show,
                until_found=not args.once,
                stop_on_detect=bool(args.stop_on_detect),
                publish_host=bool(args.publish_host),
                host_endpoint=host_endpoint,
                publish_hz=float(args.publish_hz),
            )

        color, depth, intrinsics, depth_scale = run_mock_frame(detector_cfg)
        det = detector.detect(color)
        if det is None:
            print("[Error] mock detector found no object", file=sys.stderr)
            return 1
        tracker = TargetTracker.from_config(detector_cfg)
        h_img = int(intrinsics.height)
        w_img = int(intrinsics.width)
        tracker.try_lock(det, width=w_img, height=h_img)
        packet = tracker.update(depth_raw=depth, intrinsics=intrinsics, depth_scale=depth_scale)
        p_camera = _publish_point_for_packet(packet)
        if p_camera is None:
            print("[Error] tracker produced no camera point on mock frame", file=sys.stderr)
            return 1
        obs = build_camera_observation(
            detection_label=str(packet.label or det.label),
            confidence=float(packet.track_confidence),
            p_camera_object=p_camera,
        )
        print_observation(obs, det=det)
        _print_track_packet(packet)
        if args.publish_host:
            p_world = publish_perceived_object(
                endpoint=host_endpoint,
                object_camera_xyz=p_camera,
                label=obs.label,
                track=_track_payload_from_packet(packet),
            )
            _print_world_object(p_world)
        return 0

    except RealSenseUnavailableError as exc:
        print(f"[Error] RealSense unavailable: {exc}", file=sys.stderr)
        print("  Install: pip install pyrealsense2", file=sys.stderr)
        return 1
    except YoloUnavailableError as exc:
        print(f"[Error] YOLO unavailable: {exc}", file=sys.stderr)
        print("  Install: pip install ultralytics", file=sys.stderr)
        return 1
    except HostPublishError as exc:
        print(f"[Error] host publish failed: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"[Error] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
