import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from elesim_sim.robot.go2.foot_workspace import foot_angles_body, foot_position_body, project_ground_target, reachable
from elesim_sim.robot.go2.pympc_controller import stance_end_foothold

LO = np.array([-1.0472, -.5236, -2.7227])
HI = np.array([1.0472, 4.5379, -.83776])


@pytest.mark.parametrize('leg', range(4))
def test_inverse_roundtrip(leg):
    angles = np.array([.1 if leg in (0, 2) else -.1, .9, -1.8])
    np.testing.assert_allclose(foot_angles_body(leg, foot_position_body(leg, angles)), angles, atol=1e-10)


def test_rear_target_is_recovered_on_the_ground_after_body_advances():
    base = np.array([.24, 0., .3])
    rotation = np.eye(3)
    target = np.array([-.25, .17, .025])
    assert not reachable(2, target, base, rotation, LO, HI)
    corrected, feasible = project_ground_target(2, target, base, rotation, LO, HI)
    assert feasible
    assert reachable(2, corrected, base, rotation, LO, HI)
    assert corrected[2] == pytest.approx(target[2])
    assert corrected[0] > target[0]


def test_reachable_target_is_unchanged():
    base = np.array([0., 0., .3])
    target = np.array([-.2, .17, .025])
    corrected, feasible = project_ground_target(2, target, base, np.eye(3), LO, HI)
    assert feasible
    np.testing.assert_array_equal(corrected, target)


def test_body_too_high_is_reported_not_falsely_declared_ground_reachable():
    target = np.array([-.2, .17, .025])
    _, feasible = project_ground_target(2, target, np.array([0., 0., .8]), np.eye(3), LO, HI)
    assert not feasible


@pytest.mark.parametrize('leg', range(4))
def test_pitched_rolled_body_ground_projection_stays_within_joint_limits(leg):
    rotation = Rotation.from_euler('xyz', [.15, .2, .3]).as_matrix()
    base = np.array([.1, .1, .3])
    target = base + rotation @ foot_position_body(leg, [.1 if leg in (0,2) else -.1, .9, -1.8])
    target[0] -= .3
    target[2] = .025
    lo = LO.copy()
    if leg < 2: lo[1] = -1.5708
    corrected, feasible = project_ground_target(leg, target, base, rotation, lo, HI)
    assert feasible
    assert reachable(leg, corrected, base, rotation, lo, HI)
    assert corrected[2] == pytest.approx(.025)


def test_analytic_geometry_matches_the_shipped_pinocchio_urdf():
    from pathlib import Path
    pin = pytest.importorskip('pinocchio')
    root = Path(__file__).resolve().parents[6]
    model = pin.buildModelFromUrdf(str(root/'payload/data/models/assemblies/zed-mini/assets/go2/go2.urdf'), pin.JointModelFreeFlyer())
    data = model.createData()
    rng = np.random.default_rng(42)
    for _ in range(10):
        q = np.r_[np.zeros(3), [0, 0, 0, 1], np.zeros(12)]
        for leg, name in enumerate(('FL', 'FR', 'RL', 'RR')):
            angles = np.array([rng.uniform(-.3,.3), rng.uniform(.4,1.3), rng.uniform(-2.,-1.)])
            q[7+3*leg:10+3*leg] = angles
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        for leg, name in enumerate(('FL', 'FR', 'RL', 'RR')):
            np.testing.assert_allclose(foot_position_body(leg, q[7+3*leg:10+3*leg]),
                                       data.oMf[model.getFrameId(name+'_foot')].translation, atol=1e-9)


def test_controller_recovery_bounds_target_motion_and_preserves_ground_height():
    from types import SimpleNamespace
    from elesim_sim.robot.go2.pympc_controller import PyMpcGenesisController
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._bridge = SimpleNamespace(last_q=np.r_[[.24,0,.3],[0,0,0,1],np.zeros(12)])
    c._foot_lower = np.tile(LO+.04,(4,1))
    c._foot_upper = np.tile(HI-.04,(4,1))
    start = np.array([-.25,.17,.025])
    recovered = c._reachable_target(2, start, max_shift=.03)
    assert 0 < np.linalg.norm(recovered-start) <= .030000001
    assert recovered[2] == pytest.approx(start[2])
    for _ in range(20):
        recovered = c._reachable_target(2, recovered, max_shift=.03)
    assert reachable(2, recovered, c._bridge.last_q[:3], np.eye(3), LO+.04, HI-.04)


def test_forward_rear_touchdown_remains_reachable_through_stance_end():
    # Published GPU2 trace at t=4.6 s: this rear target landed, but the body
    # passed it during stance and the calf hit its extension limit at t=5.4.
    base = np.array([.56152445, .12331772, .32730103])
    rotation = Rotation.from_euler('xyz', [-.04439852, .26618691, .06768182]).as_matrix()
    target = np.array([.47353240, .28040362, .025])
    body_velocity = rotation @ np.array([.37531962, -.09266569, -.03856721])
    desired_velocity = np.array([.8, 0., 0.])
    remaining_swing = .12
    stance_duration = .6/2.2
    lower, upper = LO+.04, HI-.04
    result, feasible = stance_end_foothold(
        2, target, base, rotation, body_velocity, desired_velocity,
        remaining_swing, stance_duration, lower, upper,
    )
    forecast_velocity = np.where(abs(desired_velocity) > abs(body_velocity),
                                 desired_velocity, body_velocity)
    forecast_velocity[2] = 0.
    landing_base = base + forecast_velocity*remaining_swing
    end_base = base + forecast_velocity*(remaining_swing+stance_duration+.06)
    assert feasible
    assert result[0] > target[0] + .1
    assert result[2] == pytest.approx(target[2])
    assert reachable(2, result, landing_base, rotation, lower, upper)
    assert reachable(2, result, end_base, rotation, lower, upper)


def test_backward_front_touchdown_remains_reachable_through_stance_end():
    # The same trailing-leg failure appears on the front leg in reverse.
    base = np.array([-.74260318, -.05165738, .29962528])
    rotation = Rotation.from_euler('xyz', [.08051289, -.26374641, .09716753]).as_matrix()
    target = np.array([-.69325040, .09413158, .025])
    # Use the same trace's established reverse speed at t=5.1 s: the
    # low-speed liftoff itself must not assume an immediate -0.8 m/s body.
    body_velocity = np.array([-.64, -.17, 0.])
    desired_velocity = np.array([-.8, 0., 0.])
    remaining_swing = (1-.6)/2.2
    stance_duration = .6/2.2
    lower, upper = LO+.04, HI-.04
    result, feasible = stance_end_foothold(
        0, target, base, rotation, body_velocity, desired_velocity,
        remaining_swing, stance_duration, lower, upper,
    )
    forecast_velocity = np.where(abs(desired_velocity) > abs(body_velocity),
                                 desired_velocity, body_velocity)
    forecast_velocity[2] = 0.
    landing_base = base + forecast_velocity*remaining_swing
    end_base = base + forecast_velocity*(remaining_swing+stance_duration+.06)
    assert feasible
    assert result[0] < target[0] - .1
    assert result[2] == pytest.approx(target[2])
    assert reachable(0, result, landing_base, rotation, lower, upper)
    assert reachable(0, result, end_base, rotation, lower, upper)


def test_slow_body_does_not_place_rear_foot_for_full_command_speed():
    # ba38e6b advanced the next rear foot by 7 cm while base speed was
    # 0.03 m/s. The foot then braked the body and forward progress stopped.
    base = np.array([.14502032, .02931007, .31952688])
    rotation = Rotation.from_euler('xyz', [.03234908, .03885711, .06875277]).as_matrix()
    target = np.array([.06175619, .20981782, .025])
    body_velocity = rotation @ np.array([.02695699, -.00377471, .03388461])
    result, feasible = stance_end_foothold(
        2, target, base, rotation, body_velocity, np.array([.8, 0., 0.]),
        .16, .6/2.2, LO+.04, HI-.04,
    )
    assert feasible
    assert result[0]-target[0] < .025
