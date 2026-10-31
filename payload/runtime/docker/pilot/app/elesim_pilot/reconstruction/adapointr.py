"""Metric AdaPoinTr completion, isolated from Pilot's imports and DDS threads."""
from __future__ import annotations

import hashlib
import math
import multiprocessing as mp
from pathlib import Path
import time

import numpy as np
import yaml

from .pipeline import PipelineCancelled


PROFILES = {
    'pcn': ('PCN.yaml', 16384, [512, 256]),
    'shapenet55': ('ShapeNet-55.yaml', 8192, [512, 256]),
    'projected55': ('Projected_ShapeNet-55_noise.yaml', 8192, [256, 128]),
}


def prepare(points, *, normalization, radius, axis_order, seed):
    points = np.array(points, dtype=np.float64, copy=True)
    if (points.ndim != 2 or points.shape[1] != 3 or not 32 <= len(points) <= 100_000
            or not np.isfinite(points).all()):
        raise ValueError('AdaPoinTr requires 32..100000 finite observed points')
    center = points.mean(axis=0) if normalization != 'native' else np.zeros(3)
    centered = points-center
    extent = float(np.linalg.norm(centered, axis=1).max())
    if extent < 1e-6:
        raise ValueError('AdaPoinTr observed cloud is degenerate')
    scale = extent/radius if normalization == 'observed-radius' else 1.
    normalized = (centered/scale).astype(np.float32)
    if axis_order == 'world-z-up':
        normalized = normalized[:, [0, 2, 1]]
    rng = np.random.default_rng(seed)
    if len(points) >= 2048:
        indices = rng.choice(len(points), 2048, replace=False)
    else:
        indices = np.r_[np.arange(len(points)), rng.choice(len(points), 2048-len(points), replace=True)]
        rng.shuffle(indices)
    return np.ascontiguousarray(normalized[indices]), center, scale


def restore(points, center, scale, *, axis_order, expected_points):
    values = np.asarray(points)
    if values.shape != (expected_points, 3) or not np.isfinite(values).all():
        raise ValueError('AdaPoinTr returned invalid or nonfinite generated points')
    if axis_order == 'world-z-up':
        values = values[:, [0, 2, 1]]
    result = values.astype(np.float64)*scale+center
    if np.any(np.abs(result) > 100):
        raise ValueError('AdaPoinTr generated points exceed the metre scene bounds')
    return result


class AdaPoinTrCompletion:
    api_version, stage, version = 1, 'completion', 'adapointr-metric-v1'
    supports_cancellation = True

    def __init__(self, *, options, assets):
        allowed = {'profile', 'normalization', 'radius', 'axis_order', 'seed', 'timeout_s', 'cache_root'}
        if set(options)-allowed or set(assets) != {'source', 'config', 'weights'}:
            raise ValueError('AdaPoinTr requires source/config/weights assets and explicit supported options')
        self.profile = options.get('profile', 'projected55')
        if self.profile not in PROFILES:
            raise ValueError('unsupported AdaPoinTr profile')
        filename, self.output_points, centers = PROFILES[self.profile]
        config = yaml.safe_load(Path(assets['config']).read_text())
        model = config['model']
        if (Path(config['dataset']['train']['_base_']).name != filename
                or model.get('NAME') != 'AdaPoinTr' or model.get('encoder_type') != 'graph'
                or model.get('num_points') != self.output_points or model.get('center_num') != centers):
            raise ValueError('AdaPoinTr config does not match its declared profile')
        self.normalization = options.get('normalization', 'center-only')
        self.axis_order = options.get('axis_order', 'world-z-up')
        self.radius, self.seed = options.get('radius', .5), options.get('seed', 0)
        self.timeout_s = options.get('timeout_s', 120.)
        if (self.normalization not in {'center-only', 'observed-radius', 'native'}
                or self.axis_order not in {'world-z-up', 'model-native'}
                or not isinstance(self.radius, (int, float)) or not 0 < self.radius <= 1
                or type(self.seed) is not int or not 0 <= self.seed < 2**32
                or not isinstance(self.timeout_s, (int, float)) or not math.isfinite(self.timeout_s)
                or not 0 < self.timeout_s <= 180):
            raise ValueError('invalid AdaPoinTr frame, sampling or timeout policy')
        self.assets = dict(assets)
        hashes = {}
        for name, path in assets.items():
            digest = hashlib.sha256()
            with Path(path).open('rb') as handle:
                while block := handle.read(1024*1024):
                    digest.update(block)
            hashes[name] = digest.hexdigest()
        self.spec = dict(assets=self.assets, hashes=hashes, cache_root=options.get('cache_root'))
        self._process, self._pipe = None, None
        self._metadata = dict(loaded=False, applied=False, profile=self.profile,
            normalization=self.normalization, axis_order=self.axis_order, asset_sha256=hashes)

    def _receive(self, deadline, cancelled):
        while time.monotonic() < deadline:
            if cancelled():
                raise PipelineCancelled('AdaPoinTr completion cancelled')
            if self._pipe.poll(.05):
                message = self._pipe.recv()
                if not message.get('ok'):
                    raise RuntimeError('AdaPoinTr worker: '+message.get('error', 'unknown failure'))
                return message
            if not self._process.is_alive():
                raise RuntimeError(f'AdaPoinTr worker exited ({self._process.exitcode})')
        raise TimeoutError('AdaPoinTr model execution deadline exceeded')

    def _ensure_worker(self, deadline, cancelled):
        if cancelled():
            raise PipelineCancelled('AdaPoinTr completion cancelled')
        if self._process is None:
            from .adapointr_backend import worker_main
            context = mp.get_context('spawn')
            parent, child = context.Pipe()
            self._pipe = parent
            self._process = context.Process(target=worker_main, args=(child, self.spec),
                name='pilot-adapointr', daemon=True)
            self._process.start()
            child.close()
            ready = self._receive(deadline, cancelled)
            self._metadata.update(ready['metadata'], loaded=True)

    def prepare(self, *, cancelled=lambda: False):
        try:
            self._ensure_worker(time.monotonic()+self.timeout_s, cancelled)
        except BaseException:
            self.close()
            raise

    def complete(self, cloud, task, *, cancelled=lambda: False):
        prepared, center, scale = prepare(cloud.observed, normalization=self.normalization,
            radius=self.radius, axis_order=self.axis_order, seed=self.seed)
        deadline = time.monotonic()+self.timeout_s
        began = time.monotonic()
        try:
            self._ensure_worker(deadline, cancelled)
            self._pipe.send(prepared)
            message = self._receive(deadline, cancelled)
            restored = restore(message['predicted'], center, scale,
                axis_order=self.axis_order, expected_points=self.output_points)
            self._metadata.update(applied=True, object_id=cloud.object_id,
                input_points=len(cloud.observed), sampled_points=2048, predicted_points=len(restored),
                center_m=center.tolist(), metres_per_model_unit=scale,
                elapsed_s=time.monotonic()-began, forward_ms=message['forward_ms'])
            return restored
        except BaseException:
            self._metadata['applied'] = False
            self.close()
            raise

    def diagnostics(self):
        return dict(self._metadata)

    def close(self):
        process, pipe = self._process, self._pipe
        if process is not None and process.pid is not None:
            if process.is_alive() and pipe is not None:
                try:
                    pipe.send(None)
                except (OSError, EOFError):
                    pass
            process.join(1.)
            if process.is_alive():
                process.terminate()
                process.join(1.)
            if process.is_alive():
                process.kill()
                process.join(1.)
            if process.is_alive():
                raise TimeoutError('AdaPoinTr worker has not stopped')
        if pipe is not None:
            pipe.close()
        self._process, self._pipe = None, None
