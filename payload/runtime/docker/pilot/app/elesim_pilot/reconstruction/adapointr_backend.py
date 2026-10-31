"""Pinned upstream AdaPoinTr inference in a Pilot-owned CUDA child process.

The source is a hash-verified operator-supplied asset, not a runtime dependency
on this repository's research tree. Upstream's imports are isolated here.
"""
from __future__ import annotations

import hashlib
import importlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import sys
import tarfile
import tempfile
import time
import types


UPSTREAM_REVISION = '4603257ed3db9e7dad349b712e1b2fe0da207015'
SOURCE_ROOT = 'PoinTr-'+UPSTREAM_REVISION


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while block := stream.read(1024*1024):
            result.update(block)
    return result.hexdigest()


def unpack_source(path, expected_sha256, cache_root):
    if digest(path) != expected_sha256:
        raise ValueError('AdaPoinTr source digest changed')
    root = Path(cache_root)
    root.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='source-', dir=root))
    complete = False
    try:
        hashes = {}
        with tarfile.open(path, 'r:gz') as archive:
            members = archive.getmembers()
            if len(members) > 10000 or sum(member.size for member in members) > 128*1024*1024:
                raise ValueError('AdaPoinTr source archive exceeds its bounds')
            for member in members:
                name = PurePosixPath(member.name)
                if (name.is_absolute() or '..' in name.parts or not name.parts
                        or name.parts[0] != SOURCE_ROOT or not (member.isfile() or member.isdir())):
                    raise ValueError('invalid AdaPoinTr source archive member')
                relative = PurePosixPath(*name.parts[1:])
                if (not member.isfile() or not relative.parts
                        or not (relative.suffix == '.py' or relative.name in {'LICENSE', 'LICENSE.txt'})):
                    continue
                if member.size > 2*1024*1024:
                    raise ValueError('AdaPoinTr source member exceeds its bounds')
                target = staging/relative
                target.parent.mkdir(parents=True, exist_ok=True)
                data = archive.extractfile(member).read()
                target.write_bytes(data)
                hashes[str(relative)] = hashlib.sha256(data).hexdigest()
        if not (staging/'models/AdaPoinTr.py').is_file():
            raise ValueError('AdaPoinTr architecture is missing from source asset')
        (staging/'source-manifest.json').write_text(json.dumps(hashes, sort_keys=True))
        complete = True
        return staging
    finally:
        if not complete:
            shutil.rmtree(staging)


class AttrDict(dict):
    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name) from None


def attributes(value):
    if isinstance(value, dict):
        return AttrDict({key: attributes(item) for key, item in value.items()})
    if isinstance(value, list):
        return [attributes(item) for item in value]
    return value


def namespace(name, path=None):
    module = types.ModuleType(name)
    if path is not None:
        module.__path__ = [str(path)]
    sys.modules[name] = module
    return module


def load_model(vendor, config_path, checkpoint_path):
    import torch
    import yaml
    from . import adapointr_ops as ops

    config = attributes(yaml.safe_load(Path(config_path).read_text())['model'])
    namespace('models', vendor/'models')
    namespace('utils', vendor/'utils')
    namespace('extensions', vendor/'extensions')
    loss = namespace('extensions.chamfer_dist')
    class InferenceOnlyLoss(torch.nn.Module):
        def forward(self, *_args):
            raise RuntimeError('training loss is unavailable in AdaPoinTr inference')
    loss.ChamferDistanceL1 = InferenceOnlyLoss
    pointnet = namespace('pointnet2_ops')
    pointnet.pointnet2_utils = types.SimpleNamespace(furthest_point_sample=ops.furthest_point_sample,
        gather_operation=ops.gather_operation, three_nn=ops.unsupported_deformable_operation,
        three_interpolate=ops.unsupported_deformable_operation)
    namespace('timm')
    namespace('timm.models')
    layers = namespace('timm.models.layers')
    class DropPath(torch.nn.Module):
        def __init__(self, drop_prob=0.):
            super().__init__()
            self.drop_prob = drop_prob
        def forward(self, value):
            if not self.training or not self.drop_prob:
                return value
            raise RuntimeError('training DropPath is unavailable in AdaPoinTr inference')
    layers.DropPath, layers.trunc_normal_ = DropPath, torch.nn.init.trunc_normal_
    model = importlib.import_module('models.AdaPoinTr').AdaPoinTr(config)
    raw = torch.load(checkpoint_path, map_location='cpu', weights_only=True)
    state = {key.removeprefix('module.'): value for key, value in raw['base_model'].items()}
    loaded = model.load_state_dict(state, strict=True)
    model = model.eval().cuda()
    return model, dict(parameters=sum(value.numel() for value in model.parameters()),
        missing_keys=loaded.missing_keys, unexpected_keys=loaded.unexpected_keys,
        upstream_revision=UPSTREAM_REVISION, gpu=torch.cuda.get_device_name(0),
        torch_version=str(torch.__version__), backend='upstream AdaPoinTr graph; Triton FPS / Torch gather',
        training=False, tf32=False)


def worker_main(pipe, spec):
    vendor = None
    try:
        import torch
        if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
            raise RuntimeError('AdaPoinTr requires exactly one configured CUDA device')
        torch.set_num_threads(2)
        torch.manual_seed(17)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        for key, path in spec['assets'].items():
            if digest(path) != spec['hashes'][key]:
                raise ValueError('AdaPoinTr asset changed before worker load: '+key)
        cache = spec.get('cache_root') or str(Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))/
                                             'elesim-models/adapointr')
        vendor = unpack_source(spec['assets']['source'], spec['hashes']['source'], cache)
        model, metadata = load_model(vendor, spec['assets']['config'], spec['assets']['weights'])
        pipe.send(dict(ok=True, metadata=metadata))
        with torch.inference_mode():
            while True:
                prepared = pipe.recv()
                if prepared is None:
                    break
                tensor = torch.from_numpy(prepared[None]).cuda()
                began = time.perf_counter()
                predicted = model(tensor)[-1]
                torch.cuda.synchronize()
                elapsed = (time.perf_counter()-began)*1000
                pipe.send(dict(ok=True, predicted=predicted[0].cpu().numpy(), forward_ms=elapsed))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as error:
        try:
            pipe.send(dict(ok=False, error=f'{type(error).__name__}: {error}'[:2048]))
        except (OSError, EOFError):
            pass
    finally:
        pipe.close()
        # Only this worker's private source extraction is owned here. Keep
        # source/checkpoint assets, Triton kernels and all Genesis caches.
        if vendor is not None:
            shutil.rmtree(vendor)
