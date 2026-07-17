#!/usr/bin/env python3
"""Preview the Genesis sim camera stream in a separate OpenCV window."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config import load_app_config


def _depth_preview(depth_raw: np.ndarray) -> np.ndarray:
    if depth_raw is None or depth_raw.size == 0:
        return np.zeros((1, 1, 3), dtype=np.uint8)
    depth = np.asarray(depth_raw, dtype=np.uint16)
    valid = depth[depth > 0]
    if valid.size == 0:
        gray = np.zeros(depth.shape, dtype=np.uint8)
    else:
        near = float(np.percentile(valid, 2.0))
        far = float(np.percentile(valid, 98.0))
        if far <= near:
            far = near + 1.0
        normalized = np.clip((depth.astype(np.float32) - near) / (far - near), 0.0, 1.0)
        gray = (255.0 * (1.0 - normalized)).astype(np.uint8)
        gray[depth == 0] = 0
    import cv2

    return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)


def _fit_image(image: np.ndarray, *, scale: float) -> np.ndarray:
    scale = float(scale)
    if scale <= 0.0 or abs(scale - 1.0) < 1e-6:
        return image
    import cv2

    h, w = image.shape[:2]
    return cv2.resize(image, (max(1, int(w * scale)), max(1, int(h * scale))), interpolation=cv2.INTER_AREA)


def _compose(color_bgr: np.ndarray, depth_raw: np.ndarray, *, show_depth: bool, scale: float) -> np.ndarray:
    import cv2

    color = np.ascontiguousarray(color_bgr, dtype=np.uint8)
    if show_depth:
        depth = _depth_preview(depth_raw)
        if depth.shape[:2] != color.shape[:2]:
            depth = cv2.resize(depth, (color.shape[1], color.shape[0]), interpolation=cv2.INTER_NEAREST)
        image = np.concatenate([color, depth], axis=1)
    else:
        image = color
    return _fit_image(image, scale=scale)


def _draw_status(image: np.ndarray, *, text: str) -> None:
    import cv2

    cv2.rectangle(image, (0, 0), (image.shape[1], 24), (0, 0, 0), thickness=-1)
    cv2.putText(
        image,
        text,
        (8, 17),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )


def _blank(width: int, height: int, *, text: str, scale: float) -> np.ndarray:
    image = np.zeros((max(1, int(height)), max(1, int(width)), 3), dtype=np.uint8)
    _draw_status(image, text=text)
    return _fit_image(image, scale=scale)


def main() -> int:
    ap = argparse.ArgumentParser(description="Show the Genesis sim camera stream in a preview window.")
    ap.add_argument("--config", default=str(ROOT / "configs/config.yaml"))
    ap.add_argument("--endpoint", default="", help="Override sim camera ZMQ endpoint.")
    ap.add_argument("--jpeg", dest="jpeg", action="store_true", default=None, help="Decode color as JPEG.")
    ap.add_argument("--raw", dest="jpeg", action="store_false", help="Decode color as raw BGR bytes.")
    ap.add_argument("--depth", dest="depth", action="store_true", help="Show RGB plus depth colormap.")
    ap.add_argument("--no-depth", dest="depth", action="store_false", help=argparse.SUPPRESS)
    ap.set_defaults(depth=False)
    ap.add_argument("--scale", type=float, default=1.0, help="Preview window scale.")
    ap.add_argument("--timeout-ms", type=int, default=500)
    ap.add_argument("--window", default="Sim Camera Preview")
    args = ap.parse_args()

    bundle = load_app_config(str(args.config))
    sim_cfg = bundle.sim_config
    endpoint = str(args.endpoint).strip() or str(sim_cfg.sim_camera_port)
    use_jpeg = bool(sim_cfg.sim_camera_jpeg) if args.jpeg is None else bool(args.jpeg)
    expected_w = max(1, int(sim_cfg.sim_camera_width))
    expected_h = max(1, int(sim_cfg.sim_camera_height))

    try:
        import cv2
        from engine.vision.sim_camera.subscriber import SimCameraSubscriber
    except Exception as exc:
        print(f"[sim_camera_preview] OpenCV and pyzmq are required: {exc}")
        return 2

    sub = SimCameraSubscriber(endpoint, use_jpeg=use_jpeg)
    last_frame_t = 0.0
    last_seq = -1
    fps = 0.0
    frames = 0
    fps_t0 = time.time()

    print(
        f"[sim_camera_preview] connecting {endpoint} jpeg={use_jpeg} "
        f"depth={bool(args.depth)}; press q or Esc to quit"
    )
    try:
        cv2.namedWindow(str(args.window), cv2.WINDOW_NORMAL)
        while True:
            frame = sub.recv_latest(timeout_ms=int(args.timeout_ms))
            now = time.time()
            if frame is None:
                image = _blank(
                    expected_w,
                    expected_h,
                    text=f"Waiting for sim camera | {endpoint}",
                    scale=float(args.scale),
                )
            else:
                frames += 1
                if now - fps_t0 >= 0.5:
                    fps = frames / max(now - fps_t0, 1e-6)
                    fps_t0 = now
                    frames = 0
                last_frame_t = now
                last_seq = int(frame.seq)
                image = _compose(
                    frame.color_bgr,
                    frame.depth_raw,
                    show_depth=bool(args.depth),
                    scale=float(args.scale),
                )
                age_ms = max(0.0, (now - float(frame.ts)) * 1000.0) if frame.ts else 0.0
                _draw_status(image, text=f"seq={last_seq} fps={fps:4.1f} age={age_ms:4.0f} ms")

            if frame is None and last_frame_t > 0.0:
                _draw_status(image, text=f"No new frame | last seq={last_seq} | {now - last_frame_t:.1f}s ago")
            cv2.imshow(str(args.window), image)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"[sim_camera_preview] failed to open preview window: {exc}")
        return 1
    finally:
        sub.close()
        try:
            cv2.destroyWindow(str(args.window))
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
