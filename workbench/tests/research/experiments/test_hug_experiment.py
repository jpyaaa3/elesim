"""Host-only checks: these do not impersonate Genesis or prove grasp success."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from workbench.research.experiments.hug_interpolation import configure as cfg
from workbench.research.experiments.hug_interpolation.experiment import main
from workbench.research.experiments.hug_interpolation.interpolate import CHANNELS, interpolate
from elesim_sim.rl.trajectory import Trajectory


@pytest.mark.parametrize('roll', [-90, 90])
def test_urdf_map_axes_and_total_to_per_node(roll):
    root = ET.parse(cfg.BUNDLE/'robot.urdf').getroot()
    q = .2
    joints = {'j_housing_wedge': math.radians(roll), 'j_wedge_node0': q}
    joints.update({f'j_node{i}_node{i+1}': q for i in range(9)})
    base = cfg.fk(root, 'node0', joints)[:3, 3]
    fourth = cfg.fk(root, 'node4', joints)[:3, 3]
    expected = [.05*sum(math.cos(i*q) for i in range(1, 5)),
                roll/90*.05*sum(math.sin(i*q) for i in range(1, 5)), 0]
    np.testing.assert_allclose(fourth-base, expected, atol=1e-10)
    np.testing.assert_allclose(base, [.343, 0, .242], atol=1e-10)


def test_task_copy_unlocks_only_bend_and_roll_and_protects_camera(tmp_path):
    original = cfg.sha(cfg.BUNDLE/'robot.urdf')
    path = cfg.materialize_robot(tmp_path/'robot.urdf')
    root = ET.parse(path).getroot()
    assert cfg.sha(cfg.BUNDLE/'robot.urdf') == original
    bend = root.find("joint[@name='j_wedge_node0']/limit")
    assert float(bend.get('upper')) == pytest.approx(math.pi/5)
    roll = root.find("joint[@name='j_housing_wedge']/limit")
    assert float(roll.get('lower')) == pytest.approx(-math.pi/2)
    assert root.find("link[@name='camera']/collision/geometry/mesh") is not None
    assert root.find("joint[@name='j_gripper_base_claw_left']").get('type') == 'fixed'
    assert root.find("joint[@name='j_gripper_base_claw_right']").get('type') == 'fixed'
    assert all(Path(m.get('filename')).is_absolute() for m in root.iter('mesh'))
    assert cfg.materialize_robot(path) == path


def pilot(tmp_path):
    c = dict(case_id='anchor', split='train', band=1, roll_deg=-90, diameter_mm=150.,
             xy_mm=[-100., 320.], branch='one', mu_min=.3,
             start_command=[0, -math.pi/2, 0, 0], goal_command=[0, -math.pi/2, .05, -.5])
    data = dict(schema='elesim-hug-pilot', schema_version=1, source_sha256='source',
                source_coordinates={'node0_from_body_center_mm': [0., 144.5]},
                cases=[c], triangles=[{'branch': 'one', 'anchor_ids': ['anchor']}])
    path = tmp_path/'pilot.json'; path.write_text(json.dumps(data))
    return path


def test_configure_world_position_and_immutable_context(tmp_path):
    result = cfg.configure(pilot(tmp_path))
    case = result['cases'][0]
    np.testing.assert_allclose(case['object_xyz_m'], [.5185, .1, .55])
    assert case['start_command'][2] == .3  # away from object, no initial straight-arm overlap
    path = tmp_path/'experiment.json'; cfg.write_json(path, result)
    assert cfg.load_experiment(path) == result
    result['cases'][0]['goal_command'][2] += .01
    path.write_text(json.dumps(result))
    with pytest.raises(ValueError, match='Case set'):
        cfg.load_experiment(path)


@pytest.mark.parametrize('changes', [dict(dt=0), dict(hold_steps=1.5), dict(friction=.2),
                                      dict(height_m=.2), dict(start_bend_rad=1), dict(unknown=1)])
def test_bad_experiment_settings_fail_offline(tmp_path, changes):
    with pytest.raises(ValueError):
        cfg.configure(pilot(tmp_path), changes)


def test_hold_phase_is_constant_after_endpoint_adjustment():
    context = {'model_sha256': 'test'}
    target = dict(case_id='target', xy_mm=[.2, .3], band=1, roll_deg=-90, diameter_mm=150,
                  branch='one', simulation_context=context, start_command=[0, -math.pi/2, .3, 0],
                  goal_command=[0, -math.pi/2, .1, -.4], duration_s=4., motion_end_s=3.)
    anchors = []
    for i, xy in enumerate(([0, 0], [1, 0], [0, 1])):
        end = [0, -math.pi/2, .1+i*.01, -.4-i*.01]
        commands = [target['start_command'], [0, -math.pi/2, .2, -.3], end, end]
        anchors.append(Trajectory([0, 1.5, 3, 4], commands, CHANNELS,
                                  dict(target, xy_mm=xy, case_id=str(i)), {'replay_success': True}))
    result = interpolate(anchors, target, samples=401)
    np.testing.assert_allclose(result.commands[300:], np.tile(target['goal_command'], (101, 1)), atol=1e-10)
    anchors[0].commands[-1, 2] += .1
    with pytest.raises(ValueError, match='hold'):
        interpolate(anchors, target)


def test_configure_cli_never_imports_training_stack(tmp_path):
    destination = tmp_path/'experiment.json'
    assert main(['configure', '--pilot', str(pilot(tmp_path)), '--output', str(destination)]) == 0
    assert cfg.load_experiment(destination)['readiness'] == 'configured-not-physics-tested'


def test_offline_cli_split_routing_with_explicit_test_fixtures(tmp_path):
    # Replay flags here are synthetic fixtures, not experimental evidence.
    cases, triangles = [], []
    for band, roll in ((1, -90), (2, 90)):
        ids = []
        for i, (xy, split) in enumerate([([-100, 300], 'train'), ([-80, 300], 'train'),
                                          ([-100, 320], 'train'), ([-95, 305], 'validation'),
                                          ([-90, 305], 'test')]):
            cid = f'b{band}-{i}'
            cases.append(dict(case_id=cid, split=split, band=band, roll_deg=roll,
                         diameter_mm=150., xy_mm=xy, branch=f'b{band}', mu_min=.3,
                         start_command=[0, math.radians(roll), 0, 0],
                         goal_command=[0, math.radians(roll), .1, -.4]))
            if split == 'train':
                ids.append(cid)
        triangles.append(dict(branch=f'b{band}', anchor_ids=ids))
    data = dict(schema='elesim-hug-pilot', schema_version=1, source_sha256='test-fixture',
                source_coordinates={'node0_from_body_center_mm': [0., 144.5]}, cases=cases, triangles=triangles)
    source = tmp_path/'pilot.json'; cfg.write_json(source, data)
    exp = cfg.configure(source); path = tmp_path/'experiment.json'; cfg.write_json(path, exp)
    anchors = tmp_path/'anchors'
    for case in exp['cases']:
        if case['split'] != 'train':
            continue
        metadata = dict(case, simulation_context=exp['simulation_context'], duration_s=4., motion_end_s=3.)
        Trajectory([0, 3, 4], [case['start_command'], case['goal_command'], case['goal_command']],
                   CHANNELS, metadata, {'replay_success': True}).save(anchors/(case['case_id']+'.json'))
    output = tmp_path/'candidates'
    assert main(['interpolate', '--experiment', str(path), '--anchors', str(anchors),
                 '--split', 'validation', '--output', str(output)]) == 0
    results = [Trajectory.load(p) for p in output.glob('*.json')]
    assert len(results) == 2
    assert all(t.metadata['split'] == 'validation' for t in results)
    assert all(t.diagnostics['physics_validated'] is False for t in results)


def test_bundled_example_is_self_contained_and_has_fixed_splits():
    sample = Path(cfg.__file__).with_name('example_pilot.json')
    exp = cfg.configure(sample)
    assert len(exp['cases']) == 30
    assert {split: sum(c['split'] == split for c in exp['cases'])
            for split in ('train', 'validation', 'test')} == {'train': 6, 'validation': 22, 'test': 2}
    assert all(c['diameter_mm'] == 150 for c in exp['cases'])
    assert exp['source_sha256'] == 'c28a6f9535e8859fa400300903d145ca6d9663ef805db83d04a2fe9f1782b1ee'
    assert exp['example_provenance']['full_map_required'] is False
