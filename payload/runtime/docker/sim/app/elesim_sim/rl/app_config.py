"""Read the mechanism's parameters from the application config.

Every value that describes the *robot* rather than the RL setup -- joint
ranges, the u-space mapping, the quadruped's spawn height, the UI's pose
presets -- lives in ``payload/config/sim/config.yaml`` and is loaded here rather than
copied into the RL config by hand.

Hand-copying produced four separate disagreements with the real system:

* ``bend_axis_sign`` taken from ``runtime.JointLayout`` (-1) double-negated the
  bend, folding the arm through the quadruped's own trunk: 74 mm of penetration
  at 495 N for the UI's Home preset.
* ``command_direction`` defaulted to ``(1,1,1,1)`` while the app uses
  ``(1,1,1,-1)``, which flips segment 2 -- Home came out as a C-shape instead
  of the S-shape it actually is.
* the roll range was set to +/-180 deg against a real +/-90 deg, so pose sweeps
  spent half their samples on unreachable configurations.
* the quadruped spawn height was taken first from an analysis document (0.32),
  then "corrected" to the schema default (0.42), while the app config says
  0.32 -- the first value had been right.

None of those are visible as errors in the RL config on its own; they only show
up against the mechanism.  So the mechanism is the source of truth.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

_REPO_ROOT = next(root for root in Path(__file__).resolve().parents if (root / "AGENTS.md").is_file())
DEFAULT_APP_CONFIG = "payload/config/sim/config.yaml"

#: UI pose presets, in slider units, from pilot.pick.actions.
#: ``home_controls`` and ``extend_arm_controls`` are the two buttons on the
#: 4-DoF control panel.
UI_PRESETS: dict[str, dict[str, float]] = {
    "home": {"u_linear": 180.0, "u_roll": 180.0, "u_s1": 10.0, "u_s2": 10.0},
    "extend_arm": {"u_linear": 15.0, "u_roll": 180.0, "u_s1": 180.0, "u_s2": 180.0},
}


@dataclass(frozen=True)
class MechanismFacts:
    """What the application config says about the arm and its carrier."""

    linear_m: tuple[float, float]
    roll_rad: tuple[float, float]
    bend_per_node_rad: float
    go2_spawn_height_m: float
    go2_mount_offset_m: tuple[float, float, float]
    node_pitch_m: float
    command_direction: tuple[int, int, int, int]
    presets: dict[str, tuple[float, float, float, float]]
    source: str

    def describe(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "linear_m": list(self.linear_m),
            "roll_deg": [math.degrees(v) for v in self.roll_rad],
            "bend_per_node_deg": math.degrees(self.bend_per_node_rad),
            "go2_spawn_height_m": self.go2_spawn_height_m,
            "go2_mount_offset_m": list(self.go2_mount_offset_m),
            "node_pitch_m": self.node_pitch_m,
            "command_direction": list(self.command_direction),
            "presets_deg": {
                name: [
                    round(q[0], 4),
                    round(math.degrees(q[1]), 2),
                    round(math.degrees(q[2]), 2),
                    round(math.degrees(q[3]), 2),
                ]
                for name, q in self.presets.items()
            },
        }


def load_mechanism(path: Optional[str | Path] = None) -> MechanismFacts:
    """Load joint ranges, the u-space mapping and the UI presets from the app."""
    from elesim_protocol.messages import ControlU, control_u_to_sim_q
    from elesim_sim.config import load_app_config

    config_path = Path(path) if path else _REPO_ROOT / DEFAULT_APP_CONFIG
    if not config_path.is_absolute():
        config_path = (_REPO_ROOT / config_path).resolve()
    bundle = load_app_config(str(config_path))

    limit = bundle.joint_limit
    mapping = bundle.mapping_config
    spawn = bundle.spawn_config

    presets: dict[str, tuple[float, float, float, float]] = {}
    for name, u in UI_PRESETS.items():
        # Converted with the *app's* mapping, so command_direction is whatever
        # the real system uses rather than the protocol default.
        q = control_u_to_sim_q(ControlU(**u), mapping)
        presets[name] = (
            float(q.linear_m),
            float(q.roll_rad),
            float(q.theta1_rad),
            float(q.theta2_rad),
        )

    return MechanismFacts(
        linear_m=(float(mapping.linear_q_min_m), float(mapping.linear_q_max_m)),
        roll_rad=(float(limit.roll_min_rad()), float(limit.roll_max_rad())),
        bend_per_node_rad=float(limit.bend_lim_rad()),
        go2_spawn_height_m=float(spawn.go2_spawn_height),
        go2_mount_offset_m=tuple(float(v) for v in spawn.go2_mount_offset_m),
        node_pitch_m=float(spawn.pitch),
        command_direction=tuple(int(v) for v in mapping.command_direction),
        presets=presets,
        source=str(config_path),
    )
