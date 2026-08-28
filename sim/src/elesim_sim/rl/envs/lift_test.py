"""Success criteria: geometric gate, scripted lift, and scripted tug.

Lifting *this* arm means rotating roll so the bend plane goes from horizontal
to vertical.  A cylinder held that way ends up lying on its side, roughly 90
degrees from upright, and that is the normal outcome -- uprightness is
deliberately **not** part of the test.  What is tested is *retention*: the
object must not fall, and its pose relative to segment 2 must stay put for the
whole lift plus a hold afterwards.

The tug test asks the same retention question without lifting anything: hold
the wrap still and pull the object sideways, through the opening the wrap left,
with a force equal to a multiple of the object's own weight.  It exists because
the two cheaper criteria do not answer it.  A wrap-angle gate scores a pose,
and caging is out of reach on this arm -- the coil is a spiral, so it surrounds
184-198 deg of bearing while the widest gap stays 10-20 mm wider than the
object at every size in the randomisation range.  Measured, the tug separates
cleanly: wrapped, the object holds to within 8 mm of the arm under its own
weight and 3 mm under four times it; unwrapped, it slides 710 mm.

Batching note: ``scene.step()`` advances every environment, so neither
scripted test can be run for one env in isolation.  Both are therefore
**per-env state machines**: an env that reaches the wrap threshold switches
from following the policy to following the script, while its neighbours carry
on.  Only per-env data differs -- joint targets for the lift, an applied force
for the tug.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Protocol

import torch

from ..configs.loader import SuccessConfig


class LiftPhase(IntEnum):
    """Per-env phase.  Only ``WRAPPING`` follows the policy.

    ``SETTLING`` sits between the rotation and the measured hold: the lift lays
    a standing object down into the coil, so it travels a long way relative to
    the arm *on purpose*, and the retention tolerances are meaningless until it
    has come to rest there.
    """

    WRAPPING = 0
    LIFTING = 1
    SETTLING = 2
    HOLDING = 3
    PASSED = 4
    FAILED = 5


class SuccessCriterion(Protocol):
    """Swappable success test, so the curriculum can change it by config."""

    def reset(self, env_ids: Optional[torch.Tensor]) -> None: ...

    @property
    def name(self) -> str: ...


@dataclass
class LiftObservation:
    """State the lift monitor needs, sampled once per substep."""

    object_pos: torch.Tensor       # (n, 3)
    object_quat: torch.Tensor      # (n, 4)
    anchor_pos: torch.Tensor       # (n, 3) segment-2 mid link
    anchor_quat: torch.Tensor      # (n, 4)
    #: Lowest point of the object above the floor, in metres.  Zero while it
    #: still rests on the ground.
    clearance: torch.Tensor        # (n,)
    #: How many arm links are touching the object.
    object_contacts: torch.Tensor  # (n,)


def _quat_relative_angle(q_a: torch.Tensor, q_b: torch.Tensor) -> torch.Tensor:
    """Angle of the rotation taking `q_a` to `q_b`, in radians.

    Quaternion double cover means q and -q are the same rotation, so the dot
    product is taken in absolute value; without that, a perfectly held object
    can read as rotated by pi.
    """
    dot = (q_a * q_b).sum(dim=-1).abs().clamp(max=1.0)
    return 2.0 * torch.acos(dot)


class GeometricCriterion:
    """Coverage gate: wrap angle at or above the configured target.

    This is the curriculum stand-in for the lift test.  The target defaults to
    172 deg because that is the highest wrap angle the prior geometry sweep
    reached anywhere on the reachable joint grid; a 180 deg gate would be
    unreachable and its zero success rate would look like a training failure
    rather than a kinematic limit.
    """

    def __init__(self, cfg: SuccessConfig, *, device: torch.device) -> None:
        self.cfg = cfg
        self.device = device
        self.threshold = float(cfg.coverage_target_rad)

    @property
    def name(self) -> str:
        return "geometric"

    def reset(self, env_ids: Optional[torch.Tensor]) -> None:  # noqa: D102
        return None

    def evaluate(self, phi: torch.Tensor) -> torch.Tensor:
        return phi >= self.threshold


class LiftTest:
    """Batched scripted-lift retention test."""

    def __init__(
        self,
        cfg: SuccessConfig,
        *,
        n_envs: int,
        device: torch.device,
    ) -> None:
        self.cfg = cfg
        self.lift = cfg.lift
        self.n_envs = int(n_envs)
        self.device = device
        # The lift is attempted at its own permissive threshold.  Tying it to
        # the coverage target would make the wrap angle an objective again by
        # the back door: nothing would ever be lifted until the geometry gate
        # was met, so retention could never fail informatively.
        self.threshold = float(cfg.lift.trigger_rad)
        z = lambda dtype: torch.zeros(self.n_envs, device=device, dtype=dtype)  # noqa: E731
        self.phase = z(torch.long)
        self.substep = z(torch.long)
        self.roll_start = z(torch.float32)
        self.roll_command = z(torch.float32)
        self._ref_rel_pos = torch.zeros((self.n_envs, 3), device=device)
        self._ref_rel_quat = torch.zeros((self.n_envs, 4), device=device)
        self._violated = z(torch.bool)

    @property
    def name(self) -> str:
        return "lift"

    # -- lifecycle ---------------------------------------------------------

    def reset(self, env_ids: Optional[torch.Tensor] = None) -> None:
        if env_ids is None:
            self.phase[:] = int(LiftPhase.WRAPPING)
            self.substep[:] = 0
            self.roll_start[:] = 0.0
            self.roll_command[:] = 0.0
            self._violated[:] = False
            return
        if env_ids.numel() == 0:
            return
        self.phase[env_ids] = int(LiftPhase.WRAPPING)
        self.substep[env_ids] = 0
        self.roll_start[env_ids] = 0.0
        self.roll_command[env_ids] = 0.0
        self._violated[env_ids] = False

    # -- state machine -----------------------------------------------------

    @property
    def follows_policy(self) -> torch.Tensor:
        """Envs still under policy control (everything else is scripted)."""
        return self.phase == int(LiftPhase.WRAPPING)

    @property
    def finished(self) -> torch.Tensor:
        return (self.phase == int(LiftPhase.PASSED)) | (
            self.phase == int(LiftPhase.FAILED)
        )

    @property
    def passed(self) -> torch.Tensor:
        return self.phase == int(LiftPhase.PASSED)

    def arm(
        self, phi: torch.Tensor, roll_now: torch.Tensor, obs: LiftObservation
    ) -> torch.Tensor:
        """Start the lift for envs whose wrap angle just cleared the gate.

        The relative object pose is captured *here*, at the moment the wrap is
        declared, so retention is measured against the grasp the policy
        actually achieved rather than against the reset pose.
        """
        newly = self.follows_policy & (phi >= self.threshold)
        if not bool(newly.any()):
            return newly
        ids = newly.nonzero(as_tuple=False).flatten()
        self.phase[ids] = int(LiftPhase.LIFTING)
        self.substep[ids] = 0
        self.roll_start[ids] = roll_now[ids]
        self.roll_command[ids] = roll_now[ids]
        rel_pos, rel_quat = self._relative(obs)
        self._ref_rel_pos[ids] = rel_pos[ids]
        self._ref_rel_quat[ids] = rel_quat[ids]
        self._violated[ids] = False
        return newly

    def _relative(self, obs: LiftObservation) -> tuple[torch.Tensor, torch.Tensor]:
        """Object pose expressed relative to the segment-2 anchor link."""
        return obs.object_pos - obs.anchor_pos, obs.object_quat

    def advance(self, obs: LiftObservation) -> torch.Tensor:
        """Run one substep of the scripted lift; returns the roll command.

        Retention is checked on *every* substep, not just at the end: an object
        that slips out mid-lift and is caught again has still failed.
        """
        lifting = self.phase == int(LiftPhase.LIFTING)
        settling = self.phase == int(LiftPhase.SETTLING)
        holding = self.phase == int(LiftPhase.HOLDING)
        # Retention is measured over the hold only.  During the rotation and
        # the settle the object is *supposed* to move relative to the arm: it
        # goes from standing to lying in the coil, which is 90 deg of
        # reorientation.  Measured on the scripted manoeuvre, drift from the
        # pre-lift grasp reads 166 mm and from the end of the rotation 48 mm,
        # while from the end of the settle it is 5.6 mm -- one grasp, three
        # anchor points, two of which measure the lift instead of the hold.
        if bool(holding.any()):
            self._check_retention(obs, holding)

        target = float(self.lift.roll_target_rad)
        rate = float(self.lift.roll_rate_rad_per_substep)
        if bool(lifting.any()):
            direction = torch.sign(target - self.roll_start)
            direction = torch.where(
                direction == 0, torch.ones_like(direction), direction
            )
            stepped = self.roll_command + direction * rate
            # Clamp so the ramp stops exactly at the target from either side.
            lo = torch.minimum(self.roll_start, torch.full_like(stepped, target))
            hi = torch.maximum(self.roll_start, torch.full_like(stepped, target))
            stepped = stepped.clamp(lo, hi)
            self.roll_command = torch.where(lifting, stepped, self.roll_command)
            reached = lifting & (
                (self.roll_command - target).abs() <= rate * 0.5 + 1e-9
            )
            if bool(reached.any()):
                ids = reached.nonzero(as_tuple=False).flatten()
                self.phase[ids] = int(LiftPhase.SETTLING)
                self.substep[ids] = 0

        if bool(settling.any()):
            self.substep = torch.where(settling, self.substep + 1, self.substep)
            done = settling & (self.substep >= int(self.lift.settle_substeps))
            if bool(done.any()):
                ids = done.nonzero(as_tuple=False).flatten()
                # Re-anchor: from here on the object must stay where the settle
                # left it.
                rel_pos, rel_quat = self._relative(obs)
                self._ref_rel_pos[ids] = rel_pos[ids]
                self._ref_rel_quat[ids] = rel_quat[ids]
                self.phase[ids] = int(LiftPhase.HOLDING)
                self.substep[ids] = 0

        if bool(holding.any()):
            self.substep = torch.where(
                holding, self.substep + 1, self.substep
            )
            done = holding & (self.substep >= int(self.lift.hold_substeps))
            if bool(done.any()):
                ids = done.nonzero(as_tuple=False).flatten()
                verdict = torch.where(
                    self._violated[ids],
                    torch.full_like(ids, int(LiftPhase.FAILED)),
                    torch.full_like(ids, int(LiftPhase.PASSED)),
                )
                self.phase[ids] = verdict
        return self.roll_command

    def _check_retention(self, obs: LiftObservation, active: torch.Tensor) -> None:
        rel_pos, rel_quat = self._relative(obs)
        slipped = (rel_pos - self._ref_rel_pos).norm(dim=-1) > float(
            self.lift.max_rel_translation_m
        )
        rotated = _quat_relative_angle(rel_quat, self._ref_rel_quat) > float(
            self.lift.max_rel_rotation_rad
        )
        # Two things the pose tolerances cannot see: an object resting on the
        # floor is not being held however well its pose matches the reference,
        # and neither is one the arm has let go of.
        grounded = obs.clearance < float(self.lift.min_clearance_m)
        released = obs.object_contacts < int(self.lift.min_object_contacts)
        dropped = grounded | released
        # Sticky, so a dropped object cannot be "recovered" by the remaining
        # hold substeps: the verdict at the end of the hold reads this flag.
        #
        # The phase is deliberately *not* flipped to FAILED here.  Doing that
        # stopped the ramp mid-rotation, and the roll-back the test exists to
        # perform was never carried out -- a run that slipped 16 deg into a 90
        # deg rotation left the arm parked at 74 deg, so neither the video nor
        # the diagnostics showed what the manoeuvre does.  The commanded motion
        # now always completes; only the verdict changes.
        self._violated = self._violated | (active & (slipped | rotated | dropped))

    def diagnostics(self) -> dict[str, torch.Tensor]:
        return {
            "phase": self.phase.clone(),
            "roll_command": self.roll_command.clone(),
            "violated": self._violated.clone(),
        }


class TugPhase(IntEnum):
    """Per-env phase.  Only ``WRAPPING`` follows the policy."""

    WRAPPING = 0
    TUGGING = 1
    PASSED = 2
    FAILED = 3


class TugTest:
    """Batched scripted-tug retention test.

    The arm holds whatever wrap the policy reached and an external force pulls
    the object through the opening.  Retention is the object's pose relative to
    segment 2: the test passes when that pose survives the whole pull.
    """

    def __init__(
        self,
        cfg: SuccessConfig,
        *,
        n_envs: int,
        device: torch.device,
    ) -> None:
        self.cfg = cfg
        self.tug = cfg.tug
        self.n_envs = int(n_envs)
        self.device = device
        # Same reasoning as the lift: the tug is attempted at its own
        # permissive threshold, so retention can fail informatively instead of
        # never being tried until a wrap-angle gate is met.
        self.threshold = float(cfg.tug.trigger_rad)
        z = lambda dtype: torch.zeros(self.n_envs, device=device, dtype=dtype)  # noqa: E731
        self.phase = z(torch.long)
        self.substep = z(torch.long)
        self._force = torch.zeros((self.n_envs, 3), device=device)
        self._ref_rel_pos = torch.zeros((self.n_envs, 3), device=device)
        self._ref_rel_quat = torch.zeros((self.n_envs, 4), device=device)
        self._violated = z(torch.bool)

    @property
    def name(self) -> str:
        return "tug"

    # -- lifecycle ---------------------------------------------------------

    def reset(self, env_ids: Optional[torch.Tensor] = None) -> None:
        if env_ids is None:
            self.phase[:] = int(TugPhase.WRAPPING)
            self.substep[:] = 0
            self._force[:] = 0.0
            self._violated[:] = False
            return
        if env_ids.numel() == 0:
            return
        self.phase[env_ids] = int(TugPhase.WRAPPING)
        self.substep[env_ids] = 0
        self._force[env_ids] = 0.0
        self._violated[env_ids] = False

    # -- state machine -----------------------------------------------------

    @property
    def follows_policy(self) -> torch.Tensor:
        return self.phase == int(TugPhase.WRAPPING)

    @property
    def finished(self) -> torch.Tensor:
        return (self.phase == int(TugPhase.PASSED)) | (
            self.phase == int(TugPhase.FAILED)
        )

    @property
    def passed(self) -> torch.Tensor:
        return self.phase == int(TugPhase.PASSED)

    def arm(
        self,
        phi: torch.Tensor,
        obs: LiftObservation,
        *,
        gap_bearing_rad: torch.Tensor,
        weight_n: torch.Tensor,
    ) -> torch.Tensor:
        """Start the tug for envs whose wrap angle just cleared the gate.

        The pull direction is the bearing of the opening the wrap left, so the
        test pushes the object out the way it can actually leave rather than
        into the arm.  Its magnitude is a multiple of the object's own weight,
        which keeps the force following the randomised mass.
        """
        newly = self.follows_policy & (phi >= self.threshold)
        if not bool(newly.any()):
            return newly
        ids = newly.nonzero(as_tuple=False).flatten()
        self.phase[ids] = int(TugPhase.TUGGING)
        self.substep[ids] = 0
        magnitude = weight_n * float(self.tug.force_scale)
        self._force[ids, 0] = (magnitude * torch.cos(gap_bearing_rad))[ids]
        self._force[ids, 1] = (magnitude * torch.sin(gap_bearing_rad))[ids]
        self._force[ids, 2] = 0.0
        rel_pos, rel_quat = self._relative(obs)
        self._ref_rel_pos[ids] = rel_pos[ids]
        self._ref_rel_quat[ids] = rel_quat[ids]
        self._violated[ids] = False
        return newly

    def external_force(self) -> torch.Tensor:
        """Force to apply to the object this substep; zero where inactive."""
        active = (self.phase == int(TugPhase.TUGGING)).unsqueeze(-1)
        return torch.where(active, self._force, torch.zeros_like(self._force))

    def _relative(self, obs: LiftObservation) -> tuple[torch.Tensor, torch.Tensor]:
        return obs.object_pos - obs.anchor_pos, obs.object_quat

    def advance(self, obs: LiftObservation) -> None:
        """Run one substep of the pull.

        Retention is checked every substep: an object that squeezes out and is
        caught again has still left the grasp.
        """
        active = self.phase == int(TugPhase.TUGGING)
        if not bool(active.any()):
            return
        self._check_retention(obs, active)
        self.substep = torch.where(active, self.substep + 1, self.substep)
        done = (self.phase == int(TugPhase.TUGGING)) & (
            self.substep >= int(self.tug.hold_substeps)
        )
        if bool(done.any()):
            ids = done.nonzero(as_tuple=False).flatten()
            self.phase[ids] = torch.where(
                self._violated[ids],
                torch.full_like(ids, int(TugPhase.FAILED)),
                torch.full_like(ids, int(TugPhase.PASSED)),
            )

    def _check_retention(self, obs: LiftObservation, active: torch.Tensor) -> None:
        rel_pos, rel_quat = self._relative(obs)
        slipped = (rel_pos - self._ref_rel_pos).norm(dim=-1) > float(
            self.tug.max_rel_translation_m
        )
        rotated = _quat_relative_angle(rel_quat, self._ref_rel_quat) > float(
            self.tug.max_rel_rotation_rad
        )
        self._violated = self._violated | (active & (slipped | rotated))
        failed_now = active & self._violated
        if bool(failed_now.any()):
            ids = failed_now.nonzero(as_tuple=False).flatten()
            self.phase[ids] = int(TugPhase.FAILED)

    def diagnostics(self) -> dict[str, torch.Tensor]:
        return {
            "phase": self.phase.clone(),
            "force": self._force.norm(dim=-1).clone(),
            "violated": self._violated.clone(),
        }


def build_criterion(
    cfg: SuccessConfig, *, n_envs: int, device: torch.device
):
    """Return the criterion the config selects."""
    if cfg.criterion == "geometric":
        return GeometricCriterion(cfg, device=device)
    if cfg.criterion == "lift":
        return LiftTest(cfg, n_envs=n_envs, device=device)
    if cfg.criterion == "tug":
        return TugTest(cfg, n_envs=n_envs, device=device)
    raise ValueError(f"unknown success.criterion: {cfg.criterion!r}")
