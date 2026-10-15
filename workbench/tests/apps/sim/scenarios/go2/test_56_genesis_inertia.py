"""Match the controller model to Genesis, including inferred fixed-link mass."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def model_and_entity():
    pin = pytest.importorskip("pinocchio")
    root = next(p for p in Path(__file__).resolve().parents if (p / "payload").is_dir())
    path = root / "payload/data/models/assemblies/zed-mini/assets/go2/go2.urdf"
    model = pin.buildModelFromUrdf(str(path), pin.JointModelFreeFlyer())
    links = {}
    # Positive, distinct tensors in a nonidentity principal frame make both
    # dropped fixed links and a wrong inertia frame observable independently.
    for i, frame in enumerate(f for f in model.frames if f.type == pin.BODY):
        rotation = Rotation.from_euler("xyz", [.2, -.1, .3 + .01*i])
        desc = SimpleNamespace(mass=.03 + .001*i, inertial_pos=np.array([.003, -.002, .005]),
                               inertial_quat=rotation.as_quat()[[3, 0, 1, 2]],
                               inertia=np.diag([1e-5, 2e-5, 2.5e-5]) * (i+1))
        links[frame.name] = SimpleNamespace(name=frame.name, desc=desc)
    return pin, model, SimpleNamespace(get_link=lambda name: links[name], links=list(links.values()))


@pytest.mark.parametrize("rpy,leg_q", [([0., 0., 0.], [0., .9, -1.8]),
                                     ([.2, -.3, .4], [.1, .5, -1.2])])
def test_model_matches_per_link_mass_com_and_inertia_after_fixed_joint_folding(rpy, leg_q):
    from elesim_sim.robot.go2.mpc.genesis_pin_bridge import synchronize_model_inertias
    pin, model, entity = model_and_entity()
    q = pin.neutral(model)
    q[:3], q[3:7], q[7:] = [.4, -.2, .32], Rotation.from_euler("xyz", rpy).as_quat(), np.tile(leg_q, 4)
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    samples = []
    for link in entity.links:
        placement = data.oMf[model.getFrameId(link.name)]
        desc = link.desc
        rotation = placement.rotation @ Rotation.from_quat(desc.inertial_quat[[1, 2, 3, 0]]).as_matrix()
        samples.append((desc.mass, placement.act(desc.inertial_pos), rotation @ desc.inertia @ rotation.T))
    mass = sum(m for m, _, _ in samples)
    com = sum(m*c for m, c, _ in samples)/mass
    inertia = np.zeros((3, 3))
    for m, c, tensor in samples:
        delta = c-com
        inertia += tensor + m*(np.dot(delta, delta)*np.eye(3)-np.outer(delta, delta))

    synchronize_model_inertias(model, entity)
    pin.ccrba(model, data, q, np.zeros(model.nv))
    assert data.Ig.mass == pytest.approx(mass, abs=1e-12)
    np.testing.assert_allclose(data.com[0], com, atol=1e-12)
    np.testing.assert_allclose(data.Ig.inertia, inertia, atol=1e-12)
    # Repeated synchronization replaces the authored properties; it never
    # adds the Genesis estimate to already counted URDF inertias.
    synchronize_model_inertias(model, entity)
    pin.ccrba(model, data, q, np.zeros(model.nv))
    assert data.Ig.mass == pytest.approx(mass, abs=1e-12)


def test_missing_body_link_does_not_leave_a_partially_synchronized_model():
    from elesim_sim.robot.go2.mpc.genesis_pin_bridge import synchronize_model_inertias
    pin, model, entity = model_and_entity()
    before = [inertia.matrix().copy() for inertia in model.inertias]
    lookup = entity.get_link
    def missing(name):
        if name == "FR_calflower":
            raise KeyError(name)
        return lookup(name)
    entity.get_link = missing
    with pytest.raises(ValueError, match="FR_calflower"):
        synchronize_model_inertias(model, entity)
    for inertia, expected in zip(model.inertias, before):
        np.testing.assert_array_equal(inertia.matrix(), expected)
