"""Bounded adapter for the pinned official SEE core; measured observations only."""
from __future__ import annotations

import ctypes
from dataclasses import dataclass
import multiprocessing
from pathlib import Path
import time

import numpy as np

from .floor_calibration import camera_transform


LIBRARY = Path('/opt/elesim/native/libelesim_see.so')


@dataclass(frozen=True)
class SeeOptions:
    radius_m: float = .01
    density_per_m3: int = 1_000_000
    view_distance_m: float = .35

    def __post_init__(self):
        if (not np.isfinite([self.radius_m, self.view_distance_m]).all()
                or not .005 <= self.radius_m <= .05
                or not .16 <= self.view_distance_m <= .8
                or type(self.density_per_m3) is not int
                or not 1 <= self.density_per_m3 <= 100_000_000
                or np.ceil(4*np.pi/3*self.radius_m**3*self.density_per_m3) < 3):
            raise ValueError('invalid SEE density/radius/view distance')


def _native(frames, intrinsics, library, options):
    """Runs only in the disposable worker. All arrays share one fixed frame."""
    points = np.ascontiguousarray(np.concatenate([p for p, _ in frames]), dtype=np.float32)
    counts = np.ascontiguousarray([len(p) for p, _ in frames], dtype=np.int32)
    views = np.ascontiguousarray([np.r_[t[:3, 3], t[:3, 2]] for _, t in frames], dtype=np.float32)
    fx, fy, cx, cy, width, height = intrinsics
    fov_x = np.degrees(np.arctan(cx/fx)+np.arctan((width-cx)/fx))
    fov_y = np.degrees(np.arctan(cy/fy)+np.arctan((height-cy)/fy))
    output = np.empty((4096, 9), dtype=np.float32)
    lib = ctypes.CDLL(str(library))
    float_array = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
    int_array = np.ctypeslib.ndpointer(dtype=np.int32, flags='C_CONTIGUOUS')
    call = lib.elesim_see_proposals
    call.argtypes = [float_array, int_array, float_array, ctypes.c_int,
                     ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float,
                     ctypes.c_float, ctypes.c_int, ctypes.c_float, float_array, ctypes.c_int]
    call.restype = ctypes.c_int
    count = call(points, counts, views, len(frames), int(width), int(height),
                 fov_x, fov_y, options.radius_m, options.density_per_m3, options.view_distance_m, output, len(output))
    if count < 0:
        raise RuntimeError(f'SEE native error {count} (input/parameters/proposal budget/native failure)')
    result = output[:count].astype(float)
    if not np.isfinite(result).all():
        raise RuntimeError('SEE returned non-finite proposals')
    return result


def _worker(connection, frames, intrinsics, library, options):
    try:
        connection.send((True, _native(frames, intrinsics, library, options)))
    except Exception as error:
        connection.send((False, f'{type(error).__name__}: {error}'[:512]))
    finally:
        connection.close()


class SeeBackend:
    def __init__(self, *, library=LIBRARY, timeout_s=20., options=None):
        self.library = Path(library)
        if not self.library.is_file():
            raise RuntimeError('SEE native library is absent; build the Pilot SEE artifact first')
        if not np.isfinite(timeout_s) or not 0 < timeout_s <= 60:
            raise ValueError('invalid SEE deadline')
        self.timeout_s = timeout_s
        self.options = options if options is not None else SeeOptions()
        if not isinstance(self.options, SeeOptions):
            raise TypeError('SEE options must be SeeOptions')

    def propose(self, frames, intrinsics, *, cancelled=lambda: False):
        if not 1 <= len(frames) <= 24:
            raise ValueError('SEE requires 1..24 accepted observations')
        intr = np.asarray(intrinsics, float)
        if (intr.shape != (6,) or not np.isfinite(intr).all() or
                np.any(intr[[0, 1, 4, 5]] <= 0) or
                not 0 <= intr[2] <= intr[4] or not 0 <= intr[3] <= intr[5]):
            raise ValueError('invalid SEE camera intrinsics')
        checked = []
        for points, pose in frames:
            p = np.asarray(points, float)
            if p.ndim != 2 or p.shape[1] != 3 or not 3 <= len(p) <= 20_000 or not np.isfinite(p).all():
                raise ValueError('invalid SEE observed cloud')
            t = camera_transform(pose)
            if np.any(np.linalg.norm(p-t[:3, 3], axis=1) < 1e-6):
                raise ValueError('SEE point coincides with sensor origin')
            checked.append((p, t))
        if cancelled():
            raise InterruptedError('SEE cancelled')
        context = multiprocessing.get_context('spawn')
        parent, child = context.Pipe(duplex=False)
        process = context.Process(target=_worker, args=(child, checked, intr, self.library, self.options), daemon=True)
        started = time.monotonic()
        try:
            process.start()
            child.close()
            while not parent.poll(.02):
                if cancelled():
                    raise InterruptedError('SEE cancelled')
                if time.monotonic()-started >= self.timeout_s:
                    raise TimeoutError('SEE planning deadline exceeded')
                if not process.is_alive():
                    raise RuntimeError(f'SEE worker exited: {process.exitcode}')
            try:
                ok, result = parent.recv()
            except EOFError as error:
                raise RuntimeError('SEE worker closed without a result') from error
            if not ok:
                raise RuntimeError(result)
            return result
        finally:
            child.close()
            parent.close()
            if process.pid is not None:
                process.join(.1)
                if process.is_alive():
                    process.terminate()
                    process.join(1.)
                if process.is_alive():
                    process.kill()
                    process.join(1.)
                process.close()
