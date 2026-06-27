from __future__ import annotations

import json
import unittest

import engine.protocol as proto
from engine.go2.hardware.obstacles_avoid_api import (
    API_OBSTACLES_AVOID_SWITCH,
    build_obstacles_avoid_parameter,
)
from engine.go2.hardware.sport_api import fill_unitree_request


class TestObstaclesAvoidApi(unittest.TestCase):
    def test_build_parameter(self) -> None:
        self.assertEqual(json.loads(build_obstacles_avoid_parameter(enable=False)), {"enable": False})
        self.assertEqual(json.loads(build_obstacles_avoid_parameter(enable=True)), {"enable": True})
        self.assertEqual(API_OBSTACLES_AVOID_SWITCH, 1001)

    def test_fill_request_obstacles_style(self) -> None:
        class _Policy:
            priority = -1
            noreply = True

        class _Lease:
            id = -1

        class _Identity:
            id = -1
            api_id = -1

        class _Header:
            identity = _Identity()
            lease = _Lease()
            policy = _Policy()

        class _Request:
            header = _Header()
            parameter = ""
            binary = [1]

        req = _Request()
        fill_unitree_request(
            req,
            api_id=1001,
            parameter='{"enable":false}',
            identity_id=1,
            noreply=False,
        )
        self.assertEqual(req.header.identity.id, 1)
        self.assertEqual(req.header.identity.api_id, 1001)
        self.assertFalse(req.header.policy.noreply)

    def test_unpack_enable(self) -> None:
        self.assertTrue(proto.unpack_go2_obstacles_avoid_enable(True))
        self.assertFalse(proto.unpack_go2_obstacles_avoid_enable(False))
        self.assertTrue(proto.unpack_go2_obstacles_avoid_enable("on"))
        self.assertFalse(proto.unpack_go2_obstacles_avoid_enable("off"))


if __name__ == "__main__":
    unittest.main()
