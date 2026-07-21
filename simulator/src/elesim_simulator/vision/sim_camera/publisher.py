from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import zmq
from zmq.auth.thread import ThreadAuthenticator
from elesim_simulator.observability.tracing import sampled_traced

import elesim_protocol as protocol
from elesim_protocol import (
    CurveServerConfig,
    configure_curve_server,
    require_curve_server_auth,
)
from elesim_simulator.vision.sim_camera.types import SimCameraFrame


class SimCameraPublisher:
    """Sim-side PUB for eye-in-hand frames (multipart: meta JSON, color, depth)."""

    def __init__(
        self,
        endpoint: str,
        *,
        use_jpeg: bool = True,
        jpeg_quality: int = 85,
        send_depth: bool = True,
        curve: Optional[CurveServerConfig] = None,
        curve_client_keys_dir: str | Path | None = None,
        allow_insecure_remote: bool = False,
    ) -> None:
        self.endpoint = str(endpoint)
        self.use_jpeg = bool(use_jpeg)
        self.jpeg_quality = int(jpeg_quality)
        self.send_depth = bool(send_depth)
        authorized_dir = (
            None
            if curve_client_keys_dir is None or not str(curve_client_keys_dir).strip()
            else Path(curve_client_keys_dir).expanduser().resolve()
        )
        require_curve_server_auth(
            self.endpoint,
            curve_enabled=curve is not None,
            authorized_clients=authorized_dir is not None,
            allow_insecure_remote=bool(allow_insecure_remote),
        )
        if authorized_dir is not None and not authorized_dir.is_dir():
            raise FileNotFoundError(f"media client key directory is missing: {authorized_dir}")
        self._owns_context = authorized_dir is not None
        self._ctx = zmq.Context() if self._owns_context else zmq.Context.instance()
        self._authenticator: Optional[ThreadAuthenticator] = None
        if authorized_dir is not None:
            self._authenticator = ThreadAuthenticator(self._ctx)
            self._authenticator.start()
            self._authenticator.configure_curve(domain="*", location=str(authorized_dir))
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.SNDHWM, 1)
        if curve is not None:
            configure_curve_server(self._sock, curve)
        self._sock.bind(self.endpoint)
        self.published = 0
        self.dropped = 0
        print(
            f"[sim_camera] publisher bound {self.bound_endpoint} "
            f"jpeg={self.use_jpeg} depth={self.send_depth}"
        )

    @property
    def bound_endpoint(self) -> str:
        return self._sock.getsockopt(zmq.LAST_ENDPOINT).decode("utf-8")

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
        if self._authenticator is not None:
            self._authenticator.stop()
            self._authenticator = None
        if self._owns_context:
            self._ctx.term()
