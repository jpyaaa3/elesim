from __future__ import annotations

import unittest
import sys
import types

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

from elesim_simulator.robot.go2.mpc.payload_model import ArmPayloadCompensator


class _FakeLink:
    def __init__(self, name: str, mass: float, pos) -> None:
        self.name = name
        self.inertial_mass = float(mass)
        self.inertial_pos = np.zeros(3, dtype=float)
        self.inertial_i = None
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


class PayloadModelTests(unittest.TestCase):
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
