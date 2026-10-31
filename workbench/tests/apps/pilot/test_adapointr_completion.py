import hashlib
import io
from pathlib import Path
import tarfile
import time

import numpy as np
import pytest
import yaml

from elesim_pilot.reconstruction.adapointr import AdaPoinTrCompletion, prepare, restore
from elesim_pilot.reconstruction.adapointr_backend import SOURCE_ROOT, unpack_source
from elesim_pilot.reconstruction.contracts import ObjectCloud, Task
from elesim_pilot.reconstruction.pipeline import PipelineCancelled


def _inference_fixture(pipe, spec):
    pipe.send(dict(ok=True, metadata={'fixture': True}))
    while True:
        points = pipe.recv()
        if points is None:
            break
        pipe.send(dict(ok=True, predicted=np.tile(points, (4, 1)), forward_ms=.01))
    pipe.close()


def _blocked_inference_fixture(pipe, spec):
    pipe.send(dict(ok=True, metadata={'fixture': True}))
    pipe.recv()
    time.sleep(60.)


def assets(tmp_path):
    config = tmp_path/'model.yaml'
    config.write_text(yaml.safe_dump(dict(model=dict(NAME='AdaPoinTr', encoder_type='graph',
        num_points=8192, center_num=[256, 128]),
        dataset={'train': {'_base_': 'Projected_ShapeNet-55_noise.yaml'}})))
    weights, source = tmp_path/'weights', tmp_path/'source'
    weights.write_bytes(b'test weights')
    source.write_bytes(b'test source')
    return dict(config=str(config), source=str(source), weights=str(weights))


def cloud():
    points = np.random.default_rng(2).normal(size=(300, 3))*.02+[.3, -.2, .7]
    return ObjectCloud('target', 'object', 'target', 'world', points)


@pytest.mark.parametrize('normalization,radius', [('center-only', .5), ('observed-radius', .5), ('native', .5)])
def test_input_sampling_and_axis_inverse_preserve_metric_world_coordinates(normalization, radius):
    observed = cloud().observed
    before = observed.copy()
    prepared, center, scale = prepare(observed, normalization=normalization,
        radius=radius, axis_order='world-z-up', seed=0)
    restored = restore(prepared, center, scale, axis_order='world-z-up', expected_points=2048)
    np.testing.assert_allclose(restored.mean(axis=0), observed.mean(axis=0), atol=.004)
    from scipy.spatial import cKDTree
    assert cKDTree(observed).query(restored)[0].max() < 1e-7
    np.testing.assert_array_equal(observed, before)


def test_completion_reuses_its_owned_process_and_closes_it(tmp_path, monkeypatch):
    import elesim_pilot.reconstruction.adapointr_backend as backend
    monkeypatch.setattr(backend, 'worker_main', _inference_fixture)
    model = AdaPoinTrCompletion(options={}, assets=assets(tmp_path))
    observed = cloud()
    try:
        model.prepare()
        assert model.diagnostics()['loaded'] and not model.diagnostics()['applied']
        first = model.complete(observed, Task('scene', 'target'))
        process = model._process
        second = model.complete(observed, Task('scene', 'target'))
        assert model._process is process and process.is_alive()
        np.testing.assert_array_equal(first, second)
        assert first.shape == (8192, 3)
        assert model.diagnostics()['applied'] and model.diagnostics()['predicted_points'] == 8192
    finally:
        model.close()
    assert not process.is_alive()


def test_cancelled_native_inference_stops_the_owned_process(tmp_path, monkeypatch):
    import elesim_pilot.reconstruction.adapointr_backend as backend
    monkeypatch.setattr(backend, 'worker_main', _blocked_inference_fixture)
    model = AdaPoinTrCompletion(options={}, assets=assets(tmp_path))
    began = time.monotonic()
    with pytest.raises(PipelineCancelled):
        model.complete(cloud(), Task('scene', 'target'), cancelled=lambda: model._metadata['loaded'])
    assert model._process is None and model._pipe is None
    assert not model.diagnostics()['applied'] and time.monotonic()-began < 8.


def test_source_asset_rejects_traversal_and_never_reuses_mutable_source(tmp_path):
    archive = tmp_path/'source.tar.gz'
    def write_archive(name):
        with tarfile.open(archive, 'w:gz') as handle:
            info = tarfile.TarInfo(name)
            data = b'# test fixture\n'
            info.size = len(data)
            handle.addfile(info, io.BytesIO(data))
        return hashlib.sha256(archive.read_bytes()).hexdigest()
    digest = write_archive(SOURCE_ROOT+'/../escape.py')
    with pytest.raises(ValueError, match='archive member'):
        unpack_source(archive, digest, tmp_path/'cache')
    digest = write_archive(SOURCE_ROOT+'/models/AdaPoinTr.py')
    root = unpack_source(archive, digest, tmp_path/'cache')
    (root/'models/AdaPoinTr.py').write_text('tampered')
    fresh = unpack_source(archive, digest, tmp_path/'cache')
    assert fresh != root
    assert (fresh/'models/AdaPoinTr.py').read_bytes() == b'# test fixture\n'
