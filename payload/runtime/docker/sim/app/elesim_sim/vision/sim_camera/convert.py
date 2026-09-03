"""Camera tensor/array conversion with an optional CUDA fast path.

Genesis may return either NumPy arrays or Torch tensors.  The old camera path
always moved a tensor to the host before doing normalization, channel
reordering and resizing.  That made a GPU render pay for a synchronization and
then made the CPU perform work that can safely be done on the device.

This module deliberately owns only pixel conversion.  It does not import
Genesis, DDS or WebRTC, and it keeps a NumPy path for CPU renders or an
explicitly selected CPU conversion.  A CUDA tensor never silently falls back
to that path: doing so would synchronize the render stream and recreate the
frame-rate collapse this module is meant to avoid.  A timing sink is
intentionally tiny so the runtime can account for conversion stages without
coupling this module to its perf logger.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional

import numpy as np


TimingSink = Callable[[str, float], None]


def _emit(sink: Optional[TimingSink], name: str, started: float) -> None:
    if sink is None:
        return
    try:
        sink(str(name), max(0.0, time.perf_counter() - float(started)))
    except Exception:
        # Profiling must never make a camera capture fail.
        pass


def _as_numpy(value: Any) -> np.ndarray:
    """Convert a CPU-side array/tensor without changing its value semantics."""

    current = value
    detach = getattr(current, "detach", None)
    if callable(detach):
        current = detach()
    cpu = getattr(current, "cpu", None)
    if callable(cpu):
        current = cpu()
    numpy = getattr(current, "numpy", None)
    if callable(numpy):
        current = numpy()
    return np.asarray(current)


def _is_cuda_tensor(value: Any) -> bool:
    # ``is_cuda`` is preferred over checking for ``cpu``: fake tensors and
    # NumPy-compatible wrappers often expose a cpu() method but are already on
    # the host.  Torch remains a lazy dependency here so CPU-only installs do
    # not import it at module import time.
    return bool(getattr(value, "is_cuda", False)) and callable(
        getattr(value, "detach", None)
    )


def _cuda_rgb_to_bgr(
    value: Any,
    *,
    target_width: int,
    target_height: int,
    normalized_float: Optional[bool],
    timing_sink: Optional[TimingSink],
) -> np.ndarray:
    """Convert a CUDA RGB tensor and transfer one final BGR image to the host.

    Genesis normally returns HxWx3 float tensors in the [0, 1] range.  The
    Genesis' floating render output is normalized.  That boundary passes
    ``normalized_float=True`` so conversion remains on-device until the one
    image transfer required by DDS/WebRTC.  ``None`` keeps range auto-detection
    for other callers, at the cost of one scalar synchronization.
    """

    try:
        import torch
        import torch.nn.functional as functional
    except (ImportError, OSError) as exc:
        raise RuntimeError(
            "CUDA camera conversion requires a working PyTorch CUDA install"
        ) from exc

    tensor = value.detach()
    if tensor.ndim != 3 or int(tensor.shape[-1]) < 3:
        raise ValueError(
            "CUDA RGB camera output must have shape HxWx3 or HxWx4"
        )

    if tensor.dtype == torch.uint8:
        pixels = tensor[..., :3]
    else:
        work = tensor.to(dtype=torch.float32)
        # Match NumPy's eventual clipping while keeping non-finite values from
        # poisoning the reduction.  +inf is represented above the normalized
        # range so normalized finite values are not accidentally scaled.
        work = torch.nan_to_num(work, nan=0.0, posinf=255.0, neginf=0.0)
        is_normalized = normalized_float
        if is_normalized is None:
            is_normalized = bool(
                float(torch.amax(work).item()) <= 1.0 if work.numel() else True
            )
        if is_normalized:
            work = work * 255.0
        pixels = torch.clamp(work, 0.0, 255.0).to(dtype=torch.uint8)
        pixels = pixels[..., :3]

    pixels = pixels.flip(-1).contiguous()
    if int(pixels.shape[0]) != int(target_height) or int(pixels.shape[1]) != int(target_width):
        resize_started = time.perf_counter()
        # Interpolate in float so uint8 values do not overflow.  The result is
        # rounded/clipped before the host transfer and is equivalent to the
        # previous OpenCV interpolation within one quantization level.
        image = pixels.permute(2, 0, 1).unsqueeze(0).to(dtype=torch.float32)
        image = functional.interpolate(
            image,
            size=(int(target_height), int(target_width)),
            mode="bilinear",
            align_corners=False,
        )
        pixels = (
            torch.clamp(torch.round(image), 0.0, 255.0)
            .to(dtype=torch.uint8)[0]
            .permute(1, 2, 0)
            .contiguous()
        )
        _emit(timing_sink, "rgb_resize", resize_started)

    transfer_started = time.perf_counter()
    result = pixels.cpu().numpy()
    _emit(timing_sink, "rgb_transfer", transfer_started)
    return np.ascontiguousarray(result, dtype=np.uint8)


def _cuda_depth_to_uint16(
    value: Any,
    *,
    target_width: int,
    target_height: int,
    timing_sink: Optional[TimingSink],
) -> np.ndarray:
    """Convert a CUDA depth tensor in metres to the wire-format uint16 image."""

    try:
        import torch
        import torch.nn.functional as functional
    except (ImportError, OSError) as exc:
        raise RuntimeError(
            "CUDA camera conversion requires a working PyTorch CUDA install"
        ) from exc

    tensor = value.detach()
    if tensor.ndim == 3:
        if int(tensor.shape[-1]) != 1:
            raise ValueError("CUDA depth camera output must have shape HxW or HxWx1")
        tensor = tensor[..., 0]
    if tensor.ndim != 2:
        raise ValueError("CUDA depth camera output must have shape HxW or HxWx1")

    work = tensor.to(dtype=torch.float32)
    work = torch.nan_to_num(work, nan=0.0, posinf=0.0, neginf=0.0)
    depth = torch.clamp(work * 1000.0, 0.0, 65535.0).to(dtype=torch.uint16)
    if int(depth.shape[0]) != int(target_height) or int(depth.shape[1]) != int(target_width):
        resize_started = time.perf_counter()
        image = depth.unsqueeze(0).unsqueeze(0).to(dtype=torch.float32)
        image = functional.interpolate(
            image,
            size=(int(target_height), int(target_width)),
            mode="nearest",
        )
        depth = torch.clamp(torch.round(image), 0.0, 65535.0).to(dtype=torch.uint16)[0, 0]
        _emit(timing_sink, "depth_resize", resize_started)

    transfer_started = time.perf_counter()
    result = depth.cpu().numpy()
    _emit(timing_sink, "depth_transfer", transfer_started)
    return np.ascontiguousarray(result, dtype=np.uint16)


def rgb_to_bgr(
    value: Any,
    *,
    target_width: int,
    target_height: int,
    prefer_gpu: bool = True,
    normalized_float: Optional[bool] = None,
    timing_sink: Optional[TimingSink] = None,
) -> np.ndarray:
    """Return a contiguous uint8 BGR image from a Genesis RGB result."""

    started = time.perf_counter()
    if prefer_gpu and _is_cuda_tensor(value):
        converted = _cuda_rgb_to_bgr(
            value,
            target_width=int(target_width),
            target_height=int(target_height),
            normalized_float=normalized_float,
            timing_sink=timing_sink,
        )
        _emit(timing_sink, "rgb_convert", started)
        return converted

    rgb_np = _as_numpy(value)
    if rgb_np.ndim != 3 or rgb_np.shape[-1] < 3:
        raise ValueError("RGB camera output must have shape HxWx3 or HxWx4")
    if rgb_np.dtype != np.uint8:
        rgb_f = rgb_np.astype(np.float32, copy=False)
        if float(np.nanmax(rgb_f)) <= 1.0:
            rgb_np = np.clip(rgb_f * 255.0, 0.0, 255.0).astype(np.uint8)
        else:
            rgb_np = np.clip(rgb_f, 0.0, 255.0).astype(np.uint8)
    color_bgr = np.ascontiguousarray(rgb_np[..., :3][:, :, ::-1], dtype=np.uint8)
    _emit(timing_sink, "rgb_convert", started)
    return color_bgr


def depth_to_uint16(
    value: Any,
    *,
    target_width: int,
    target_height: int,
    prefer_gpu: bool = True,
    timing_sink: Optional[TimingSink] = None,
) -> np.ndarray:
    """Return a contiguous uint16 millimetre depth image."""

    started = time.perf_counter()
    if prefer_gpu and _is_cuda_tensor(value):
        converted = _cuda_depth_to_uint16(
            value,
            target_width=int(target_width),
            target_height=int(target_height),
            timing_sink=timing_sink,
        )
        _emit(timing_sink, "depth_convert", started)
        return converted

    depth_np = np.asarray(_as_numpy(value), dtype=float)
    if depth_np.ndim == 3:
        if depth_np.shape[-1] != 1:
            raise ValueError("depth camera output must have shape HxW or HxWx1")
        depth_np = depth_np[..., 0]
    if depth_np.ndim != 2:
        raise ValueError("depth camera output must have shape HxW or HxWx1")
    depth_m = np.nan_to_num(depth_np, nan=0.0, posinf=0.0, neginf=0.0)
    depth_mm = np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)
    _emit(timing_sink, "depth_convert", started)
    return np.ascontiguousarray(depth_mm, dtype=np.uint16)


def resize_cpu_if_needed(
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    *,
    target_width: int,
    target_height: int,
    timing_sink: Optional[TimingSink] = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Resize only CPU/fallback outputs, preserving legacy OpenCV semantics."""

    if color_bgr.shape[0] != int(target_height) or color_bgr.shape[1] != int(target_width):
        import cv2

        started = time.perf_counter()
        color_bgr = cv2.resize(
            color_bgr,
            (int(target_width), int(target_height)),
            interpolation=cv2.INTER_LINEAR,
        )
        _emit(timing_sink, "rgb_resize", started)
    if depth_mm.shape[0] != int(target_height) or depth_mm.shape[1] != int(target_width):
        import cv2

        started = time.perf_counter()
        depth_mm = cv2.resize(
            depth_mm,
            (int(target_width), int(target_height)),
            interpolation=cv2.INTER_NEAREST,
        )
        _emit(timing_sink, "depth_resize", started)
    return (
        np.ascontiguousarray(color_bgr, dtype=np.uint8),
        np.ascontiguousarray(depth_mm, dtype=np.uint16),
    )


__all__ = [
    "TimingSink",
    "depth_to_uint16",
    "resize_cpu_if_needed",
    "rgb_to_bgr",
]
