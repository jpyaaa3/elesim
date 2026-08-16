from __future__ import annotations

import numpy as np
import pytest

from elesim_sim.vision.sim_camera.convert import (
    depth_to_uint16,
    resize_cpu_if_needed,
    rgb_to_bgr,
)


def _cuda_available() -> bool:
    try:
        import torch
    except Exception:
        return False
    return bool(torch.cuda.is_available())


def test_cpu_conversion_preserves_rgb_channel_order_and_depth_units() -> None:
    rgb = np.array(
        [
            [[0.0, 0.5, 1.0], [1.0, 0.25, 0.0]],
        ],
        dtype=np.float32,
    )
    depth = np.array([[0.0, 1.25]], dtype=np.float32)
    timings: list[tuple[str, float]] = []

    color = rgb_to_bgr(
        rgb,
        target_width=2,
        target_height=1,
        timing_sink=lambda name, value: timings.append((name, value)),
    )
    depth_mm = depth_to_uint16(
        depth,
        target_width=2,
        target_height=1,
        timing_sink=lambda name, value: timings.append((name, value)),
    )

    assert color.dtype == np.uint8
    assert color.shape == (1, 2, 3)
    np.testing.assert_array_equal(
        color,
        np.array([[[255, 127, 0], [0, 63, 255]]], dtype=np.uint8),
    )
    np.testing.assert_array_equal(depth_mm, np.array([[0, 1250]], dtype=np.uint16))
    assert {name for name, _value in timings} == {"rgb_convert", "depth_convert"}


def test_cpu_resize_keeps_color_and_depth_interpolation_contract() -> None:
    color = np.zeros((1, 2, 3), dtype=np.uint8)
    color[0, 1] = [255, 0, 0]
    depth = np.array([[1000, 2000]], dtype=np.uint16)
    names: list[str] = []

    resized_color, resized_depth = resize_cpu_if_needed(
        color,
        depth,
        target_width=4,
        target_height=2,
        timing_sink=lambda name, _value: names.append(name),
    )

    assert resized_color.shape == (2, 4, 3)
    assert resized_depth.shape == (2, 4)
    assert resized_color.dtype == np.uint8
    assert resized_depth.dtype == np.uint16
    assert names == ["rgb_resize", "depth_resize"]


@pytest.mark.skipif(
    not _cuda_available(),
    reason="CUDA device is required for the GPU conversion regression",
)
def test_cuda_conversion_matches_cpu_within_one_uint8_level() -> None:
    import torch

    rgb = torch.tensor(
        [
            [[0.0, 0.5, 1.0], [1.0, 0.25, 0.0]],
            [[0.1, 0.2, 0.3], [0.7, 0.8, 0.9]],
        ],
        dtype=torch.float32,
        device="cuda",
    )
    depth = torch.tensor(
        [[0.0, 1.25], [2.0, float("nan")]],
        dtype=torch.float32,
        device="cuda",
    )
    cpu_color = rgb_to_bgr(
        rgb.cpu().numpy(), target_width=4, target_height=3, prefer_gpu=False
    )
    gpu_color = rgb_to_bgr(
        rgb, target_width=4, target_height=3, prefer_gpu=True
    )
    cpu_depth = depth_to_uint16(
        depth.cpu().numpy(), target_width=4, target_height=3, prefer_gpu=False
    )
    cpu_color, cpu_depth = resize_cpu_if_needed(
        cpu_color,
        cpu_depth,
        target_width=4,
        target_height=3,
    )
    gpu_depth = depth_to_uint16(
        depth, target_width=4, target_height=3, prefer_gpu=True
    )

    assert gpu_color.shape == cpu_color.shape == (3, 4, 3)
    assert gpu_depth.shape == cpu_depth.shape == (3, 4)
    assert int(np.max(np.abs(gpu_color.astype(np.int16) - cpu_color.astype(np.int16)))) <= 1
    np.testing.assert_array_equal(gpu_depth, cpu_depth)
