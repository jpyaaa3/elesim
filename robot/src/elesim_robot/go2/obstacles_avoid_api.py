from __future__ import annotations

import json

# Unitree GO2 /api/obstacles_avoid/request switch API (user-verified on Jetson).
API_OBSTACLES_AVOID_SWITCH = 1001


def build_obstacles_avoid_parameter(*, enable: bool) -> str:
    return json.dumps({"enable": bool(enable)}, separators=(",", ":"))
