"""Bounded thread-index capacity fix for the pinned cuRobo v0.7.8 checkout.

The CUDA parallel kernel supports 1024 threads * 512 pairs; the Python
generator allocated only 32 * 512 pair slots. Larger robots fell back to a
serial kernel that leaves stale output/gradients on collision-free queries.
Increase capacity while retaining every pair. The large parallel variants
also need 16-bit loop counters (the 8-bit counters wrap at 128/512 pairs),
bounded unrolling and whole-warp block sizes. The <1024-sphere limit bounds
all pairs below 524288. No collision or gradient check is removed.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

ORIGINAL_SHA256 = "260df6aa18ddf9ae2a7c9dee098613a079d9d8ad5bed92fba394f0515c40599d"
PATCHED_SHA256 = "1d01dbad6ab66ce41b4551d24f40c7d52a5f45ffb3e90cde0cbadbdc2511bfd8"
RELATIVE_PATH = "src/curobo/cuda_robot_model/cuda_robot_generator.py"


def _patch_file(path: Path, original: str, patched: str, replacements) -> None:
    content = path.read_bytes()
    digest = hashlib.sha256(content).hexdigest()
    if digest == patched:
        return
    if digest != original:
        raise RuntimeError("unexpected cuRobo generator source; refusing to patch")
    modified = content
    for before, after in replacements:
        if modified.count(before) != 1:
            raise RuntimeError("cuRobo patch location is not unique")
        modified = modified.replace(before, after, 1)
    if hashlib.sha256(modified).hexdigest() != patched:
        raise RuntimeError("cuRobo patch digest mismatch")
    path.write_bytes(modified)


def patch(source: Path) -> None:
    _patch_file(source / RELATIVE_PATH, ORIGINAL_SHA256, PATCHED_SHA256, [
        (b"        max_checks_per_thread = 512\n        thread_loc =",
         b"        max_checks_per_thread = 16384\n        thread_loc ="),
    ])
    _patch_file(source / "src/curobo/curobolib/cpp/self_collision_kernel.cu",
                "612be710756be5d4b80cf7bb7211df94feab9108b4114c4dabb6773e04c03328",
                "1cbf5fdb24fa4ad468ebe967f637f40b8932cb9ae64fbf512620808c2d319d49", [
        (b"for (uint8_t i = 0; i < ndpt * 2; i++)", b"for (uint16_t i = 0; i < ndpt * 2; i++)"),
        (b"#pragma unroll\n\n      for (uint8_t k = 0; k < ndpt; k++)",
         b"#pragma unroll 1\n\n      for (uint16_t k = 0; k < ndpt; k++)"),
        (b"  int blocksPerGrid   = (batch_size + nbpb - 1) / nbpb;",
         b"  threadsPerBlock = ((threadsPerBlock + 31) / 32) * 32;\n  int blocksPerGrid   = (batch_size + nbpb - 1) / nbpb;"),
    ])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    patch(parser.parse_args().source)
