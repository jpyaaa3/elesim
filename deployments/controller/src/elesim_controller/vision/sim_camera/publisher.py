from __future__ import annotations

import json
from typing import Optional

import zmq
from elesim_controller.observability.tracing import sampled_traced

import elesim_protocol as protocol
from elesim_controller.vision.sim_camera.types import SimCameraFrame


class SimCameraPublisher:
    """Sim-side PUB for eye-in-hand frames (multipart: meta JSON, color, depth)."""

    def __init__(
        self,
        endpoint: str,
        *,
        use_jpeg: bool = True,
        jpeg_quality: int = 85,
        send_depth: bool = True,
    ) -> None:
        self.endpoint = str(endpoint)
        self.use_jpeg = bool(use_jpeg)
        self.jpeg_quality = int(jpeg_quality)
        self.send_depth = bool(send_depth)
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.SNDHWM, 1)
        self._sock.bind(self.endpoint)
        self.published = 0
        self.dropped = 0
        print(f"[sim_camera] publisher bound {self.endpoint} jpeg={self.use_jpeg} depth={self.send_depth}")

    @sampled_traced("camera.sim.publish", sample_key="camera.sim.publish", every=60, kind="producer")
    def publish(self, frame: SimCameraFrame) -> bool:
        meta = json.dumps(frame.to_meta_dict()).encode("utf-8")
        color_bytes = self._encode_color(frame)
        parts = [meta, color_bytes]
        if self.send_depth:
            parts.append(frame.depth_raw.tobytes())
        try:
            self._sock.send_multipart(parts, flags=zmq.NOBLOCK)
            self.published += 1
            return True
        except zmq.Again:
            self.dropped += 1
            return False
        except Exception:
            self.dropped += 1
            return False

    def _encode_color(self, frame: SimCameraFrame) -> bytes:
        if not self.use_jpeg:
            return frame.color_bgr.tobytes()
        try:
            import cv2

            ok, buf = cv2.imencode(
                ".jpg",
                frame.color_bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if ok:
                return buf.tobytes()
        except Exception:
            pass
        return frame.color_bgr.tobytes()

    def close(self) -> None:
        try:
            self._sock.close(0)
        except Exception:
            pass
