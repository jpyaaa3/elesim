from __future__ import annotations

import unittest
import sys
import types
from types import SimpleNamespace

import numpy as np

try:
    import scipy.spatial.transform  # noqa: F401
except ModuleNotFoundError:
    scipy_mod = types.ModuleType("scipy")
    spatial_mod = types.ModuleType("scipy.spatial")
    transform_mod = types.ModuleType("scipy.spatial.transform")

    class _IdentityRotation:
        @classmethod
        def from_quat(cls, _quat):
            return cls()

        def as_matrix(self):
            return np.eye(3, dtype=float)

    transform_mod.Rotation = _IdentityRotation
    spatial_mod.transform = transform_mod
    scipy_mod.spatial = spatial_mod
    sys.modules["scipy"] = scipy_mod
    sys.modules["scipy.spatial"] = spatial_mod
    sys.modules["scipy.spatial.transform"] = transform_mod

from elesim_sim.robot.go2.mpc.payload_model import ArmPayloadCompensator
from scipy.spatial.transform import Rotation


class _FakeLink:
    def __init__(self, name: str, mass: float, pos, *, inertial_pos=None, inertia=None) -> None:
        self.name = name
        self.desc = SimpleNamespace(
            mass=float(mass),
            inertial_pos=(
                np.zeros(3, dtype=float)
                if inertial_pos is None
                else np.asarray(inertial_pos, dtype=float)
            ),
            inertia=None if inertia is None else np.asarray(inertia, dtype=float),
        )
        self._pos = np.asarray(pos, dtype=float)

    def get_pos(self):
        return self._pos

    def get_quat(self):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    def get_vel(self):
        return np.zeros(3, dtype=float)

    def get_ang(self):
        return np.zeros(3, dtype=float)


class _FakeEntity:
    def __init__(self, links: list[_FakeLink]) -> None:
        self.links = links
        self._links_by_name = {link.name: link for link in links}

    def get_link(self, name: str):
        return self._links_by_name[name]


class _BulkEntity(_FakeEntity):
    def __init__(self, links):
        super().__init__(links)
        self.calls = []
        self.states = {}
        for index, link in zip([11, 3, 8, 19], links):
            link.idx_local = index
            self.states[index] = [np.asarray(getattr(link, f'get_{name}')()).copy()
                                  for name in ('pos', 'quat', 'vel', 'ang')]

    def _read(self, field, indices):
        self.calls.append((field, list(indices)))
        return np.array([self.states[int(index)][field] for index in indices])

    def get_links_pos(self, links_idx_local):
        return self._read(0, links_idx_local)

    def get_links_quat(self, links_idx_local):
        return self._read(1, links_idx_local)

    def get_links_vel(self, links_idx_local):
        return self._read(2, links_idx_local)

    def get_links_ang(self, links_idx_local):
        return self._read(3, links_idx_local)


class PayloadModelTests(unittest.TestCase):
    def test_bulk_payload_reads_four_selected_arrays_with_identical_physics(self):
        links = [_FakeLink('base', 10., [0., 0., 0.]),
                 _FakeLink('plate', .15, [.2, .1, .4], inertial_pos=[.01, .02, -.03],
                           inertia=np.diag([.002, .003, .004])),
                 _FakeLink('node9', .05, [.3, -.2, .5], inertial_pos=[-.02, .03, .01],
                           inertia=np.diag([.003, .004, .005])),
                 _FakeLink('massless', 0., [9., 9., 9.])]
        for i, link in enumerate(links):
            rotation = Rotation.from_euler('xyz', [.1*i, -.2*i, .3*i])
            quat = rotation.as_quat()[[3, 0, 1, 2]]
            link.get_quat = lambda q=quat: q
            link.get_vel = lambda i=i: np.array([.1*i, -.2*i, .3*i])
            link.get_ang = lambda i=i: np.array([-.3*i, .1*i, .2*i])
            link.desc.inertial_quat = Rotation.from_euler('x', .2*i).as_quat()[[3, 0, 1, 2]]
        names = {'plate', 'node9', 'massless'}
        reference = ArmPayloadCompensator(_FakeEntity(links), link_names=names).measure()
        entity = _BulkEntity(links)
        def scalar_read_is_forbidden():
            raise AssertionError('payload used a per-link GPU readback')
        for link in links:
            for name in ('pos', 'quat', 'vel', 'ang'):
                setattr(link, f'get_{name}', scalar_read_is_forbidden)
        actual = ArmPayloadCompensator(entity, link_names=names).measure()
        self.assertEqual(entity.calls, [(field, [3, 8]) for field in range(4)])
        self.assertEqual(actual.mass_kg, reference.mass_kg)
        for name in ('com_world', 'vel_world', 'inertia_world'):
            np.testing.assert_allclose(getattr(actual, name), getattr(reference, name), atol=1e-12)

    def test_bulk_payload_retains_position_difference_velocity_and_reset(self):
        entity = _BulkEntity([_FakeLink('plate', 2., [0., 0., .2])])
        payload = ArmPayloadCompensator(entity)
        payload.measure(dt=.02)
        entity.states[11][0][0] += .002
        np.testing.assert_allclose(payload.measure(dt=.02).vel_world, [.1, 0., 0.])
        payload.reset()
        entity.states[11][0][0] += .5
        np.testing.assert_array_equal(payload.measure(dt=.02).vel_world, np.zeros(3))

    def test_bulk_payload_rejects_invalid_array_instead_of_using_scalar_fallback(self):
        entity = _BulkEntity([_FakeLink('plate', 2., [0., 0., .2])])
        entity.get_links_quat = lambda **kw: np.array([[1., 0., 0.]])
        with self.assertRaisesRegex(ValueError, 'quat'):
            ArmPayloadCompensator(entity).measure()

    def test_payload_rotates_principal_inertia_into_link_then_world_frame(self) -> None:
        link = _FakeLink("plate", 2.0, [1.0, 0.0, 0.0], inertia=np.diag([.2, .3, .4]))
        principal = Rotation.from_euler("xyz", [.3, -.4, .7])
        world = Rotation.from_euler("xyz", [-.2, .6, -.1])
        link.desc.inertial_quat = principal.as_quat()[[3, 0, 1, 2]]
        link.get_quat = lambda: world.as_quat()[[3, 0, 1, 2]]
        rotation = (world * principal).as_matrix()

        snapshot = ArmPayloadCompensator(_FakeEntity([link])).measure()

        np.testing.assert_allclose(snapshot.inertia_world,
                                   rotation @ link.desc.inertia @ rotation.T, atol=1e-12)

    def test_stale_payload_link_velocity_uses_com_position_difference(self) -> None:
        link = _FakeLink("plate", 2.0, [0.0, 0.0, 0.0])
        payload = ArmPayloadCompensator(_FakeEntity([link]))

        payload.measure(dt=0.02)
        link._pos[0] += 0.002
        moving = payload.measure(dt=0.02)

        assert moving is not None
        self.assertTrue(np.allclose(moving.vel_world, [0.1, 0.0, 0.0]))

        payload.reset()
        link._pos[0] += 0.5
        after_reset = payload.measure(dt=0.02)
        assert after_reset is not None
        self.assertTrue(np.allclose(after_reset.vel_world, np.zeros(3)))

    def test_payload_measurement_reads_genesis_link_description_inertia(self) -> None:
        expected_inertia = np.diag([0.2, 0.3, 0.4])
        entity = _FakeEntity(
            [
                _FakeLink(
                    "plate",
                    2.0,
                    [1.0, 0.0, 0.0],
                    inertial_pos=[0.5, 0.0, 0.0],
                    inertia=expected_inertia,
                )
            ]
        )

        snap = ArmPayloadCompensator(entity).measure()

        self.assertIsNotNone(snap)
        assert snap is not None
        self.assertEqual(snap.mass_kg, 2.0)
        self.assertTrue(np.allclose(snap.com_world, np.array([1.5, 0.0, 0.0])))
        self.assertTrue(np.allclose(snap.inertia_world, expected_inertia))

    def test_payload_measurement_filters_merged_go2_links(self) -> None:
        entity = _FakeEntity(
            [
                _FakeLink("base", 10.0, [0.0, 0.0, 0.0]),
                _FakeLink("plate", 2.0, [1.0, 0.0, 0.0]),
                _FakeLink("node9", 3.0, [2.0, 0.0, 0.0]),
            ]
        )

        payload = ArmPayloadCompensator(entity, link_names={"plate", "node9"})
        snap = payload.measure()

        self.assertIsNotNone(snap)
        assert snap is not None
        self.assertEqual(snap.mass_kg, 5.0)
        self.assertTrue(np.allclose(snap.com_world, np.array([1.6, 0.0, 0.0])))
        self.assertTrue(np.allclose(payload.measure_com_body(entity), np.array([1.6, 0.0, 0.0])))

    def test_payload_measurement_without_filter_keeps_legacy_whole_entity_behavior(self) -> None:
        entity = _FakeEntity(
            [
                _FakeLink("base", 10.0, [0.0, 0.0, 0.0]),
                _FakeLink("plate", 2.0, [1.0, 0.0, 0.0]),
            ]
        )

        payload = ArmPayloadCompensator(entity)
        snap = payload.measure()

        self.assertIsNotNone(snap)
        assert snap is not None
        self.assertEqual(snap.mass_kg, 12.0)


if __name__ == "__main__":
    unittest.main()
