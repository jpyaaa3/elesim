#!/usr/bin/env python3
"""Preview one typed DDS RGB-D topic in an OpenCV window."""

from __future__ import annotations

import argparse
import time

import numpy as np

from elesim_protocol import DdsRgbdSubscriber


def _depth_preview(depth_raw: np.ndarray) -> np.ndarray:
    import cv2

    depth = np.asarray(depth_raw)
    valid = depth[np.isfinite(depth) & (depth > 0)]
    if valid.size == 0:
        gray = np.zeros(depth.shape, dtype=np.uint8)
    else:
        near = float(np.percentile(valid, 2.0))
        far = max(near + 1e-6, float(np.percentile(valid, 98.0)))
        normalized = np.clip(
            (depth.astype(np.float32) - near) / (far - near),
            0.0,
            1.0,
        )
        gray = (255.0 * (1.0 - normalized)).astype(np.uint8)
        gray[depth <= 0] = 0
    return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/elesim/sim_default/rgbd/frame",
        help="Absolute DDS RGB-D topic.",
    )
    parser.add_argument("--source-id", default="")
    parser.add_argument("--source-boot", default="")
    parser.add_argument("--depth", action="store_true")
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--timeout-ms", type=int, default=500)
    parser.add_argument("--window", default="Elesim DDS RGB-D")
    args = parser.parse_args()

    try:
        import cv2

        subscriber = DdsRgbdSubscriber(
            args.topic,
            endpoint_id="rgbd-preview",
            expected_source_id=args.source_id,
            expected_boot_id=args.source_boot,
        )
    except Exception as exc:
        print(f"[sim_camera_preview] ROS 2 interfaces/OpenCV unavailable: {exc}")
        return 2

    frames = 0
    fps = 0.0
    fps_started = time.monotonic()
    print(
        f"[sim_camera_preview] DDS topic={args.topic} "
        f"source={args.source_id or '*'}; press q or Esc to quit"
    )
    try:
        cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
        while True:
            sample = subscriber.recv_latest(timeout_ms=args.timeout_ms)
            if sample is None:
                image = np.zeros((240, 424, 3), dtype=np.uint8)
                status = "waiting for DDS RGB-D sample"
            else:
                frames += 1
                elapsed = time.monotonic() - fps_started
                if elapsed >= 0.5:
                    fps = frames / elapsed
                    frames = 0
                    fps_started = time.monotonic()
                image = np.ascontiguousarray(sample.color_bgr, dtype=np.uint8)
                if args.depth:
                    depth = _depth_preview(sample.depth_raw)
                    image = np.concatenate((image, depth), axis=1)
                if args.scale > 0.0 and abs(args.scale - 1.0) > 1e-6:
                    height, width = image.shape[:2]
                    image = cv2.resize(
                        image,
                        (
                            max(1, int(width * args.scale)),
                            max(1, int(height * args.scale)),
                        ),
                        interpolation=cv2.INTER_AREA,
                    )
                age_ms = max(0.0, (time.time() - sample.ts) * 1000.0)
                status = (
                    f"{sample.source_id} seq={sample.seq} "
                    f"fps={fps:.1f} age={age_ms:.0f}ms"
                )
            cv2.rectangle(
                image,
                (0, 0),
                (image.shape[1], 24),
                (0, 0, 0),
                thickness=-1,
            )
            cv2.putText(
                image,
                status,
                (8, 17),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (235, 235, 235),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(args.window, image)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        subscriber.close()
        try:
            cv2.destroyWindow(args.window)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
