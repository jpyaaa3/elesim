import hashlib
import json

import pytest

from elesim_pilot.reconstruction.artifacts import default_scene_output_root, scene_publication


def bundle(root):
    root.mkdir(parents=True)
    mesh = b'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n'
    (root/'target.obj').write_bytes(mesh)
    manifest = {'schema_version':1, 'scene_id':'scan-example', 'frame_id':'world', 'units':'m',
        'provenance':{}, 'objects':[{'object_id':'target', 'label':'red object', 'role':'target',
        'mesh_file':'target.obj', 'sha256':hashlib.sha256(mesh).hexdigest(),
        'observed_points':3, 'predicted_points':0}]}
    path = root/'scene.json'
    path.write_text(json.dumps(manifest))
    return path


def test_publication_uses_relative_reference_and_binds_the_exact_sim_boot(tmp_path):
    root = tmp_path/'exchange'
    path = bundle(root/'job'/'scene')
    value = scene_publication(path, root, 'sim-test', 'boot-test')
    assert value == {'scene_reference':'job/scene/scene.json',
        'scene_revision':hashlib.sha256(path.read_bytes()).hexdigest(),
        'sim_id':'sim-test', 'sim_boot_id':'boot-test'}


def test_publication_rejects_a_bundle_outside_the_exchange_root_or_modified_mesh(tmp_path):
    path = bundle(tmp_path/'outside')
    with pytest.raises(ValueError):
        scene_publication(path, tmp_path/'exchange', 'sim-test', 'boot-test')
    (path.parent/'target.obj').write_text('invalid changed mesh')
    with pytest.raises(ValueError, match='digest'):
        scene_publication(path, tmp_path, 'sim-test', 'boot-test')


def test_default_scan_output_is_a_writable_cache_location_not_package_source(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    assert default_scene_output_root() == tmp_path/'scene-imports'
