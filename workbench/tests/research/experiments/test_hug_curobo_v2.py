"""CPU checks for coordinate, coupling and timing boundaries of V2 integration."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from workbench.research.experiments.hug_reverse_curobo.v2_geometry import (
    budget_search_bounds, contact_release_offsets, diverse_release_starts, native_endpoint_bounds, normalize_commands,
    restore_mesh_urdf, small_release_offsets, task_velocity_limits, urdf_poses,
)

ROOT = Path(__file__).resolve().parents[4]


def test_restore_keeps_exact_coupling_and_excludes_only_plate_collision(tmp_path):
    source = ROOT / 'payload/data/models/planning/zed-mini'
    assembly = ROOT / 'payload/data/models/assemblies/zed-mini'
    target = tmp_path / 'arm.urdf'
    hashes = restore_mesh_urdf(source, assembly, target)
    root = ET.parse(target).getroot()
    assert not root.find("link[@name='plate']").findall('collision')
    assert root.find("link[@name='plate']").findall('visual')
    assert root.find("link[@name='gripper_base']").findall('collision')
    assert root.find("link[@name='camera']").findall('collision')
    assert len(root.findall('joint/mimic')) == 8
    assert hashes and all(len(v) == 64 for v in hashes.values())
    names = json.loads((source / 'metadata.json').read_text())['joint_names']
    q = [-.08, -.9, -.3, .4]
    old = urdf_poses(source / 'arm.urdf', names, q)
    new = urdf_poses(target, names, q)
    assert old.keys() == new.keys()
    for link in old:
        np.testing.assert_allclose(old[link], new[link], atol=1e-12)


def test_mimic_uses_master_once_not_total_bend_conversion(tmp_path):
    path = tmp_path / 'coupled.urdf'
    path.write_text('''<robot name="test"><link name="base"/><link name="one"/><link name="two"/>
      <joint name="master" type="revolute"><parent link="base"/><child link="one"/>
      <axis xyz="0 0 1"/></joint>
      <joint name="slave" type="revolute"><parent link="one"/><child link="two"/>
      <origin xyz="1 0 0"/><axis xyz="0 0 1"/><mimic joint="master" multiplier="1"/></joint></robot>''')
    result = urdf_poses(path, ['master'], [np.pi/2])['two']
    np.testing.assert_allclose(result[:3, 3], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(result[:3, :3], np.diag([-1, -1, 1]), atol=1e-12)


def boundary():
    start = np.array([0, -np.pi/2, -.2, -.5])
    home = np.array([-.12, 0, -.59, .59])
    lower = np.array([-.18, -np.pi/2, -.63, -.63])
    upper = np.array([0, np.pi/2, .63, .63])
    rate = np.array([.0005, .01, .0015, .0015])
    return start, home, lower, upper, rate


def test_float32_endpoints_and_initial_roll_noise_do_not_choose_wrong_direction():
    start, home, lower, upper, rate = boundary()
    raw = np.linspace(start, home, 1000).astype(np.float32)
    raw[:5, 1] = np.float32(start[1])
    raw[2, 1] = np.nextafter(raw[2, 1], np.float32(0))
    commands, timing = normalize_commands(raw, .01, start, home, lower, upper, rate)
    assert commands.shape == (2001, 4)
    np.testing.assert_allclose(commands[0], start, atol=0)
    np.testing.assert_allclose(commands[-1], home, atol=0)
    assert np.all(np.abs(np.diff(commands, axis=0)) <= rate+1e-7)
    assert np.all(np.diff(commands[:, 1]) >= 0)
    assert timing['movement_duration_s'] <= 19


def test_real_reversal_before_unlock_is_rejected_but_after_unlock_is_allowed():
    start, home, lower, upper, rate = boundary()
    raw = np.linspace(start, home, 1000)
    raw[50, 1] = raw[49, 1]-.001
    with pytest.raises(ValueError, match='initial roll reversal'):
        normalize_commands(raw, .01, start, home, lower, upper, rate)
    raw = np.vstack((np.linspace(start, home, 1000), home+[0, .005, 0, 0], home))
    commands, _ = normalize_commands(raw, .01, start, home, lower, upper, rate)
    assert np.max(commands[:, 1]) > 0


def test_long_path_cannot_erase_home_verification_time():
    start, home, lower, upper, rate = boundary()
    with pytest.raises(ValueError, match='one-second Home'):
        normalize_commands([start, home], .01, start, home, lower, upper, rate/100)


def test_release_offsets_preserve_original_and_respect_small_motion_bounds():
    start, _, lower, upper, _ = boundary()
    poses = small_release_offsets(start, lower, upper)
    np.testing.assert_array_equal(poses[0], start)
    assert np.all(poses >= lower) and np.all(poses <= upper)
    delta = poses-start
    assert np.max(np.abs(delta[:, 0])) <= .02+1e-12
    assert np.max(np.abs(delta[:, 1])) <= np.deg2rad(10)+1e-12
    assert np.max(np.abs(delta[:, 2:])) <= np.deg2rad(2)+1e-12
    assert np.all(delta[:, 0] <= 0) and np.all(delta[:, 1] >= 0)


def test_budget_search_bounds_do_not_exclude_any_feasible_intermediate_pose():
    start, home, lower, upper, _ = boundary()
    contract = dict(sim_to_motor_abs_scale=[1500., 360/np.pi, 900/np.pi, 900/np.pi],
                    limits_motor_deg=[200., 220., 600.])
    lo, hi = budget_search_bounds(start, home, lower, upper, contract)
    assert lo[0] == pytest.approx(-.126666667, abs=1e-8)
    assert hi[1] == pytest.approx(np.deg2rad(10), abs=1e-8)
    q = np.random.default_rng(42).uniform(lower, upper, (10000, 4))
    cost = (np.abs(q-start)+np.abs(home-q))*contract['sim_to_motor_abs_scale']
    possible = ((cost[:, 0] <= 200) & (cost[:, 1] <= 220)
                & (cost[:, 2]+cost[:, 3] <= 600))
    assert possible.any()
    assert np.all(q[possible] >= lo) and np.all(q[possible] <= hi)


def test_contact_release_search_retains_original_and_distinct_opening_alternatives():
    start, home, lower, upper, _ = boundary()
    poses = contact_release_offsets(start, home, lower, upper)
    selected = diverse_release_starts(poses, start)
    assert len(selected) == 8 and len(np.unique(selected, axis=0)) == 8
    np.testing.assert_array_equal(selected[0], start)
    assert np.all(selected >= lower) and np.all(selected <= upper)
    assert np.all(selected[:, 2:] >= start[2:])
    assert np.max(selected[:, 1]) == pytest.approx(home[1])
    assert np.min(selected[:, 0]) == pytest.approx(home[0])
    np.testing.assert_allclose(np.max(selected[:, 2:], axis=0), [0, 0])


def test_full_planner_uniform_slowdown_preserves_native_timing_profile():
    start, home, lower, upper, _ = boundary()
    rate = np.abs(home-start)/100
    middle = start+(home-start)*.1
    commands, timing = normalize_commands([start, middle, home], .01,
        start, home, lower, upper, rate, retiming='uniform')
    assert timing['retimed_segment_dt_min_s'] == pytest.approx(.9)
    assert timing['retimed_segment_dt_max_s'] == pytest.approx(.9)
    np.testing.assert_allclose(commands[90], middle, atol=1e-12)
    np.testing.assert_allclose(commands[180], home, atol=1e-12)


def test_full_planner_rejects_excess_uniform_duration_instead_of_changing_phases():
    start, home, lower, upper, _ = boundary()
    raw = [start, start+(home-start)*.1, home]
    rate = np.abs(home-start)*.01/15
    _, timing = normalize_commands(raw, .01, start, home, lower, upper, rate)
    assert timing['movement_duration_s'] == pytest.approx(15.)
    with pytest.raises(ValueError, match='one-second Home'):
        normalize_commands(raw, .01, start, home, lower, upper, rate, retiming='uniform')


def test_task_velocity_mapping_preserves_already_per_node_bends_and_stricter_urdf():
    rate = [.0005, .01, .003, .003]
    np.testing.assert_allclose(task_velocity_limits([.2, 3.5, 1., 1.], rate), [.05, 1., .3, .3])
    np.testing.assert_allclose(task_velocity_limits([.02, .5, .1, .2], rate), [.02, .5, .1, .2])
    with pytest.raises(ValueError):
        task_velocity_limits([.2, 3.5, 1., 1.], [0, .01, .003, .003])


def test_solver_endpoint_slack_does_not_change_command_bounds_or_allow_real_overshoot():
    start, home, lower, upper, rate = boundary()
    saved_lower, saved_upper = lower.copy(), upper.copy()
    lo, hi = native_endpoint_bounds(start, home, lower, upper)
    assert lo[1] == pytest.approx(lower[1]-5e-7)
    assert hi[0] == pytest.approx(5e-8)
    np.testing.assert_array_equal(lower, saved_lower)
    np.testing.assert_array_equal(upper, saved_upper)
    np.testing.assert_array_equal(lo[[0, 2, 3]], lower[[0, 2, 3]])
    np.testing.assert_array_equal(hi[[1, 2, 3]], upper[[1, 2, 3]])
    raw = np.linspace(start, home, 1000)
    raw[1, 0] = .00001
    with pytest.raises(ValueError, match='Command exceeds task bounds'):
        normalize_commands(raw, .01, start, home, lower, upper, rate, retiming='uniform')
