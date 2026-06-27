from __future__ import annotations

import json
from typing import Optional

import numpy as np
import zmq

from engine.vision.sim_camera.types import SimCameraFrame, SimCameraIntrinsics


class SimCameraSubscriber:
    """Ctrl-side SUB: latest-frame-only sim camera relay."""

    def __init__(self, endpoint: str, *, use_jpeg: bool = True) -> None:
        self.endpoint = str(endpoint)
        self.use_jpeg = bool(use_jpeg)
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVHWM, 2)
        self._connected = False

    def connect(self) -> None:
        if self._connected:
            return
        self._sock.connect(self.endpoint)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._connected = True

    def close(self) -> None:
        try:
            self._sock.close(0)
        except Exception:
            pass
        self._connected = False

    def recv_latest(self, *, timeout_ms: int = 500) -> Optional[SimCameraFrame]:
        if not self._connected:
            self.connect()
        poller = zmq.Poller()
        poller.register(self._sock, zmq.POLLIN)
        events = dict(poller.poll(timeout=int(timeout_ms)))
        if self._sock not in events:
            return None
        latest_parts: Optional[list[bytes]] = None
        while True:
            try:
                latest_parts = self._sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
        if latest_parts is None or len(latest_parts) < 2:
            return None
        parts = latest_parts
        meta = json.loads(parts[0].decode("utf-8"))
        color_bytes = parts[1]
        depth_bytes = parts[2] if len(parts) > 2 else b""
        w = int(meta.get("width", 640))
        h = int(meta.get("height", 480))
        color_bgr = self._decode_color(color_bytes, width=w, height=h)
        ch, cw = int(color_bgr.shape[0]), int(color_bgr.shape[1])
        depth_raw = self._decode_depth(depth_bytes, width=cw, height=ch, meta_w=w, meta_h=h)
        intr = SimCameraIntrinsics(
            fx=float(meta.get("fx", cw * 0.5)),
            fy=float(meta.get("fy", ch * 0.5)),
            cx=float(meta.get("cx", cw * 0.5)),
            cy=float(meta.get("cy", ch * 0.5)),
            width=cw,
            height=ch,
        )
        arm_q_raw = meta.get("arm_q", None)
        arm_q = None
        if isinstance(arm_q_raw, (list, tuple)) and len(arm_q_raw) == 4:
            arm_q = tuple(float(x) for x in arm_q_raw)

        def _vec3(key: str) -> Optional[tuple[float, float, float]]:
            raw = meta.get(key, None)
            if isinstance(raw, (list, tuple)) and len(raw) == 3:
                return (float(raw[0]), float(raw[1]), float(raw[2]))
            return None

        return SimCameraFrame(
            color_bgr=color_bgr,
            depth_raw=depth_raw,
            depth_scale=float(meta.get("depth_scale", 0.001)),
            intrinsics=intr,
            seq=int(meta.get("seq", 0)),
            ts=float(meta.get("ts", 0.0)),
            arm_q=arm_q,
            camera_world_origin=_vec3("camera_world_origin"),
            camera_world_look=_vec3("camera_world_look"),
            camera_world_right=_vec3("camera_world_right"),
        )

    def _decode_depth(
        self,
        data: bytes,
        *,
        width: int,
        height: int,
        meta_w: int,
        meta_h: int,
    ) -> np.ndarray:
        w = int(width)
        h = int(height)
        if not data:
            return np.zeros((h, w), dtype=np.uint16)
        depth_flat = np.frombuffer(data, dtype=np.uint16)
        need = w * h
        if int(depth_flat.size) == need:
            return depth_flat.reshape(h, w)
        meta_need = int(meta_w) * int(meta_h)
        if int(depth_flat.size) == meta_need:
            depth_raw = depth_flat.reshape(int(meta_h), int(meta_w))
            if (int(meta_h), int(meta_w)) != (h, w):
                import cv2

                depth_raw = cv2.resize(depth_raw, (w, h), interpolation=cv2.INTER_NEAREST)
            return depth_raw
        inferred = self._infer_depth_shape(int(depth_flat.size), target_w=w, target_h=h)
        if inferred is not None:
            src_h, src_w = inferred
            import cv2

            depth_raw = depth_flat.reshape(int(src_h), int(src_w))
            return cv2.resize(depth_raw, (w, h), interpolation=cv2.INTER_NEAREST)
        raise ValueError(
            f"depth buffer size mismatch: got {int(depth_flat.size)} px, "
            f"expected {need} (color {w}x{h}) or {meta_need} (meta {meta_w}x{meta_h})"
        )

    @staticmethod
    def _infer_depth_shape(flat_size: int, *, target_w: int, target_h: int) -> Optional[tuple[int, int]]:
        n = int(flat_size)
        if n <= 0:
            return None
        target_ar = float(target_h) / max(float(target_w), 1.0)
        best: Optional[tuple[int, int]] = None
        best_score = float("inf")
        lo = max(1, int(target_w) // 2)
        hi = max(lo, int(target_w) * 4)
        for tw in range(lo, min(n, hi) + 1):
            if n % tw != 0:
                continue
            th = n // tw
            score = abs((float(th) / max(float(tw), 1.0)) - target_ar)
            if score < best_score:
                best_score = score
                best = (int(th), int(tw))
        return best

    def _decode_color(self, data: bytes, *, width: int, height: int) -> np.ndarray:
        if self.use_jpeg:
            try:
                import cv2

                arr = np.frombuffer(data, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    img = np.ascontiguousarray(img, dtype=np.uint8)
                    if int(img.shape[0]) != int(height) or int(img.shape[1]) != int(width):
                        img = cv2.resize(
                            img,
                            (int(width), int(height)),
                            interpolation=cv2.INTER_LINEAR,
                        )
                    return img
            except Exception:
                pass
        expected = int(width) * int(height) * 3
        if len(data) == expected:
            return np.frombuffer(data, dtype=np.uint8).reshape(int(height), int(width), 3).copy()
        return np.zeros((int(height), int(width), 3), dtype=np.uint8)
