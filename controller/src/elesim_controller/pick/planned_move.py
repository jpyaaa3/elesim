"""Operator-triggered, collision-checked point-to-point arm move.

Split into two operator-visible phases so the UI can show the planned path
before committing the arm to it:

- ``generate()`` plans a joint-space path with RRT-Connect
  (``robot/arm/planning/rrt.py``), shortcuts it, and publishes the
  remaining waypoints as "planned_waypoint" debug markers (rendered by
  Simulator, see ``elesim_simulator.runtime.resolve_host_marker``) without
  moving the arm.
- ``execute()`` time-parameterizes the *previously generated* path
  (``robot/arm/planning/trajectory.py``) and streams it through the
  existing motion_command channel via ``ControlClient.send_target_values``
  -- the same wire path every other phase already uses, so no protocol
  change is needed beyond the operator allowlist entries. Each waypoint's
  marker is cleared as the arm passes it; every marker is cleared
  unconditionally when execution ends, whether it finished, failed, or was
  cancelled.

If ``generate()`` fails because the start or goal pose is itself in
collision, it publishes diagnostic markers instead of clearing everything:
the (invalid) target/current tip position plus the two conflicting links'
positions (see ``_collision_diagnostic_markers``), so the operator can see
*where* the conflict is rather than just a status string.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np

from elesim_controller.pick.control_ownership import ControlOwner, ControlOwnership, ControlOwnershipError
from elesim_controller.robot.arm.iklib import kinematics as ik_kin
from elesim_controller.robot.arm.planning.collision import (
    CollisionModel,
    CollisionResult,
    check_configuration,
    simplify_go2_to_bounding_box,
)
from elesim_controller.robot.arm.planning.rrt import (
    RrtConfig,
    make_collision_validity_fn,
    maximize_clearance,
    plan_rrt_connect,
    shortcut_path,
)
from elesim_controller.robot.arm.planning.trajectory import JointRateLimits, resample, time_parameterize

SendDebugMarkers = Callable[[list[dict[str, Any]]], None]

WAYPOINT_MARKER_COLOR = [1.0, 0.55, 0.0, 0.9]
WAYPOINT_MARKER_RADIUS = 0.02

# Diagnostic markers shown instead of clearing everything when generate()
# fails because the start or goal pose itself is in collision -- lets the
# operator see *where* on the arm the conflict is, rather than just a
# status string, without needing to guess-and-check slider values.
COLLISION_MARKER_COLOR = [0.95, 0.1, 0.1, 0.95]
COLLISION_MARKER_RADIUS = 0.03

# How close the *real, observed* tip must get to a waypoint's position
# before its marker clears -- a coarse "passed through here" threshold, not
# a precision target.
WAYPOINT_REACHED_TOLERANCE_M = 0.03
# Safeguard only: force-clear a waypoint once its commanded arrival time has
# been exceeded by this factor *or* by this many extra seconds, whichever
# is later, so a marker can't linger forever if real tracking (feedback
# missing, or sag/compliance) never converges within
# WAYPOINT_REACHED_TOLERANCE_M. The flat extra-seconds floor matters most
# for waypoints with a short *commanded* duration -- JointRateLimits is
# derived from Dynamixel profile-velocity registers (a rated, not measured,
# speed), and real cable-sag tracking can lag well behind that estimate;
# scaling a already-short arrival time alone leaves little absolute margin
# for that gap. Deliberately generous: this is a last-resort, not the
# primary "reached" signal (that's WAYPOINT_REACHED_TOLERANCE_M above).
STUCK_WAYPOINT_TIMEOUT_SCALE = 3.0
STUCK_WAYPOINT_MIN_EXTRA_S = 5.0
# arrival_times (and the commanded stream's own real-time schedule) assume
# the arm tracks commands at its rated speed in real time, but a heavily
# loaded sim (GPU rendering, physics) can fall well behind real time --
# confirmed live via HostState.sim_realtime_factor reading ~0.125-0.18 (the
# sim advancing only 1 simulated second per ~6-8 real seconds). This alone
# explains two symptoms: a waypoint clearing by timeout_fallback long before
# the arm physically got there (arrival_times is in simulated seconds, not
# real ones), and the commanded stream racing through every intermediate
# waypoint's q within the first few real seconds while the real arm is still
# far behind -- confirmed live: an intermediate waypoint missed by 16cm and
# only cleared via timeout_fallback, immediately followed by the *final*
# waypoint clearing via a genuine 2mm position match, because the reference
# had already moved past the intermediate point and on to the goal long
# before the (slow) real arm got anywhere near it. Both the timeout
# threshold and the commanded stream's own advancement are scaled by the
# *measured* sim_realtime_factor to correct for this -- floored so a brief
# telemetry glitch (or a value of exactly 0.0, which SimScene
# .sim_realtime_factor returns before the first sim step) can't blow the
# timeout up to an unusable size or freeze the stream's advancement entirely.
MIN_SIM_REALTIME_FACTOR = 0.05

# Random shortcutting (see shortcut_path) replaces a multi-waypoint detour
# with a straight segment whenever *any* valid one exists -- by construction
# that's usually the tightest, most direct connection it can find, which
# tends to hug obstacle boundaries rather than stay centered in a narrow
# opening (confirmed live: a path threading a wall's hole visibly scraped
# along the wall face after shortcutting). The RRT tree-growth validity_fn
# needs a permissive (even slightly negative) environment clearance just to
# find *a* path through a tight opening at all -- but the shortcut pass runs
# after a valid path already exists, so it can afford to require real
# positive margin instead, trading a few extra waypoints for a path that
# stays away from obstacle surfaces rather than grazing them.
SHORTCUT_MIN_ENVIRONMENT_CLEARANCE_M = 0.015
# See the shortcut_path call site for why these differ from
# RrtConfig's own (much finer/larger-budget) tree-growth defaults.
SHORTCUT_COLLISION_CHECK_RESOLUTION = 0.02
SHORTCUT_ITERATIONS = 80


@dataclass(frozen=True)
class PlannedMoveOutcome:
    success: bool
    reason: str = ""


@dataclass(frozen=True)
class PlannedMoveStatus:
    """Operator-visible snapshot of the generate/execute lifecycle."""

    phase: str = "idle"  # idle | planning | planned | executing | done | failed | cancelled
    message: str = ""
    waypoint_count: int = 0


COLLISION_WIREFRAME_LINE_RADIUS = 0.003

# Pairs of corner indices (see _box_world_corners' sign ordering) that form
# the box's 12 edges: two corners are connected iff they differ along
# exactly one axis (i.e. their index differs in exactly one bit, since the
# corner order below is sx-major/sy-mid/sz-minor -> index = 4*sx+2*sy+sz).
_BOX_EDGE_INDICES = [(i, i ^ 1) for i in range(8) if i < (i ^ 1)]
_BOX_EDGE_INDICES += [(i, i ^ 2) for i in range(8) if i < (i ^ 2)]
_BOX_EDGE_INDICES += [(i, i ^ 4) for i in range(8) if i < (i ^ 4)]


def _box_world_corners(center: np.ndarray, half_extents: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """The box's 8 corners in world space, keeping its real orientation."""
    signs = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])
    corners_local = signs * np.asarray(half_extents, dtype=float).reshape(1, 3)
    return np.asarray(center, dtype=float).reshape(1, 3) + corners_local @ np.asarray(rot, dtype=float).T


def _link_shape_markers(
    model: CollisionModel, context: Mapping[str, Any], link_name: str, q: Sequence[float], *, key: str
) -> list[dict[str, Any]]:
    """Markers showing one FK link's *real* collision-proxy extent (line for a
    capsule, a 12-edge wireframe for each of a link's box(es)), not just a
    single point -- empty if ``link_name`` isn't a real FK link
    (``go2_collision_check`` reports the GO2 side as a capsule *label*, e.g.
    ``"go2_chassis"``, with no FK pose to look up). A box's wireframe is
    drawn as 12 individual "capsule" (thin line) markers along its real
    corners, not a single axis-aligned "box" marker -- ``draw_debug_box``
    only accepts axis-aligned world bounds, so re-enclosing a *rotated* box
    in one would silently inflate it back out to (up to) its bounding
    sphere's size, which is exactly wrong for links far out on the chain
    (gripper_base/claws) that accumulate a lot of rotation from upstream
    bend/roll -- confirmed live, it rendered as a huge, badly misoriented
    box relative to the actual mesh. A link can have more than one box
    (see ``LinkBox``); each gets its own set of 12 edges with distinct keys.
    """
    link_tf = ik_kin._forward_link_tf(dict(context), q)
    if link_name not in link_tf:
        return []
    pos, rot = link_tf[link_name]
    base = {"name": "planned_waypoint", "frame": "world", "color": list(COLLISION_MARKER_COLOR)}
    if model.is_box(link_name):
        markers = []
        for idx, (center, half_extents, box_rot) in enumerate(model.world_boxes(link_name, pos, rot)):
            corners = _box_world_corners(center, half_extents, box_rot)
            for edge_idx, (a, b) in enumerate(_BOX_EDGE_INDICES):
                markers.append(
                    {
                        **base,
                        "key": f"{key}_{idx}_edge{edge_idx}",
                        "shape": "capsule",
                        "p0": corners[a].tolist(),
                        "p1": corners[b].tolist(),
                        "radius": COLLISION_WIREFRAME_LINE_RADIUS,
                    }
                )
        return markers
    p0, p1, radius = model.world_capsule(link_name, pos, rot)
    return [{**base, "key": key, "shape": "capsule", "p0": p0.tolist(), "p1": p1.tolist(), "radius": float(radius)}]


def _collision_diagnostic_markers(
    *,
    context: Mapping[str, Any],
    model: CollisionModel,
    q: Sequence[float],
    result: CollisionResult,
) -> list[dict[str, Any]]:
    """Markers showing *where* an invalid start/goal pose conflicts, so a failed
    ``generate()`` still gives the operator something to look at instead of just
    clearing every marker and reporting a bare status string. The two
    conflicting links are drawn as their real capsule/box extent (see
    ``_link_shape_markers``), not a single point -- a point alone doesn't
    convey which direction or how large the actual proxy volume is."""
    tip = ik_kin._forward_grasp_world(dict(context), q)
    markers: list[dict[str, Any]] = [
        {
            "name": "planned_waypoint",
            "key": "collision_target",
            "frame": "world",
            "pos": [float(tip[0]), float(tip[1]), float(tip[2])],
            "color": list(COLLISION_MARKER_COLOR),
            "radius": WAYPOINT_MARKER_RADIUS,
        }
    ]
    for slot, link_name in (("a", result.link_a), ("b", result.link_b)):
        markers.extend(_link_shape_markers(model, context, link_name, q, key=f"collision_link_{slot}"))
    return markers


# Resolution for _first_violation_along_path's straight-line joint-space
# sweep -- coarser than this risks stepping clean over a shallow, real
# first-contact point (e.g. a gap of a few tenths of a millimetre) and
# reporting a deeper, more confusing later state instead.
FIRST_VIOLATION_SWEEP_STEPS = 100


def _first_violation_along_path(
    *,
    context: Mapping[str, Any],
    model: CollisionModel,
    current_q: np.ndarray,
    target_q: np.ndarray,
    go2_pos: Optional[Sequence[float]],
    go2_rpy_rad: Optional[Sequence[float]],
    leg_q: Optional[Sequence[float]] = None,
    steps: int = FIRST_VIOLATION_SWEEP_STEPS,
    environment_clearance_m: Optional[float] = None,
) -> tuple[np.ndarray, CollisionResult]:
    """Walk the straight joint-space line from ``current_q`` to ``target_q`` and
    return the *first* pose (and its violating pair) that isn't collision-free.

    A "goal_in_collision" report on its own only says the endpoint is bad --
    it doesn't say *where along the way* things first went wrong, which can
    look confusing when the reported pair (e.g. a mid-chain node vs the base
    plate) seems to imply earlier links must have already been overlapping
    without ever being reported (they're excluded as near-constant, adjacent-
    by-construction pairs -- see ``discover_always_colliding_pairs``). This
    finds the actual first real violation instead of just the endpoint's.
    """
    for step in range(steps + 1):
        q = current_q + (step / steps) * (target_q - current_q)
        result = check_configuration(
            context=context,
            q=q,
            model=model,
            go2_pos=go2_pos,
            go2_rpy_rad=go2_rpy_rad,
            leg_q=leg_q,
            environment_clearance_m=environment_clearance_m,
        )
        if not result.ok:
            return q, result
    # Every sampled point on the line was clear -- can only happen if the
    # endpoint's own violation is too shallow/narrow for this resolution to
    # land on. Fall back to the endpoint itself so there's still something
    # to report.
    result = check_configuration(
        context=context,
        q=target_q,
        model=model,
        go2_pos=go2_pos,
        go2_rpy_rad=go2_rpy_rad,
        leg_q=leg_q,
        environment_clearance_m=environment_clearance_m,
    )
    return target_q, result


def _waypoint_markers(context: Mapping[str, Any], waypoints: Sequence[np.ndarray]) -> list[dict[str, Any]]:
    markers: list[dict[str, Any]] = []
    for idx, q in enumerate(waypoints):
        pos = ik_kin._forward_grasp_world(dict(context), q)
        markers.append(
            {
                "name": "planned_waypoint",
                "key": str(idx),
                "frame": "world",
                "pos": [float(pos[0]), float(pos[1]), float(pos[2])],
                "color": list(WAYPOINT_MARKER_COLOR),
                "radius": WAYPOINT_MARKER_RADIUS,
            }
        )
    return markers


class PlannedMoveExecutor:
    """Owns one generate -> execute lifecycle for a collision-checked joint-space move."""

    def __init__(
        self,
        *,
        ownership: ControlOwnership,
        ik_context: Mapping[str, Any],
        collision_model: Optional[CollisionModel] = None,
        rates: JointRateLimits = JointRateLimits(),
        tick_hz: float = 20.0,
        rrt_config: RrtConfig = RrtConfig(),
    ) -> None:
        self._ownership = ownership
        self._ik_context = dict(ik_context)
        self._collision_model = collision_model
        self._rates = rates
        self._tick_hz = float(tick_hz)
        self._rrt_config = rrt_config
        self._lock = threading.RLock()
        self._status = PlannedMoveStatus()
        self._waypoints: list[np.ndarray] = []
        self._plan_context: Mapping[str, Any] = self._ik_context

    @property
    def available(self) -> bool:
        return self._collision_model is not None

    @property
    def collision_model(self) -> Optional[CollisionModel]:
        return self._collision_model

    def status(self) -> PlannedMoveStatus:
        with self._lock:
            return self._status

    def preview_waypoints(self) -> list[np.ndarray]:
        """The joint-space waypoints from the most recent ``generate()`` -- read-only,
        for a ghost-preview playback of the planned path. Empty before the first
        successful ``generate()`` and after ``execute()`` clears them in its own
        ``finally``."""
        with self._lock:
            return list(self._waypoints)

    def _set_status(self, **changes: Any) -> PlannedMoveStatus:
        with self._lock:
            self._status = replace(self._status, **changes)
            return self._status

    def mark_planning(self) -> None:
        """Report that a generate request has been accepted and is starting,
        before ``generate()`` itself is necessarily reachable yet.

        ``start_planned_move_generate_task_space`` runs a potentially long
        IK-seed search (up to dozens of attempts, each with its own collision
        check) *before* it can call ``generate()`` -- without this, the UI's
        status line sits at whatever it showed before the click (often
        "idle") for that entire search, which reads as "Generate did nothing"
        even though a request was genuinely accepted and is in flight.
        """
        self._set_status(phase="planning", message="", waypoint_count=0)

    def fail(self, *, reason: str, message: str = "") -> PlannedMoveOutcome:
        """Report a failure that happened before/without a ``generate()`` call
        (e.g. task-space IK not converging) through the same ``phase="failed"``
        status convention ``generate()`` itself uses."""
        self._set_status(phase="failed", message=message or reason, waypoint_count=0)
        return PlannedMoveOutcome(False, reason)

    def mark_cancelled(self) -> PlannedMoveOutcome:
        """Report a cancellation that happened before ``generate()`` could get
        far enough to report it itself -- e.g. a Cancel click during task-space
        generate's IK-seed search, which precedes any ``generate()`` call. Same
        ``phase="cancelled"`` convention ``generate()``/``execute()`` use."""
        self._set_status(phase="cancelled", message="cancelled", waypoint_count=0)
        return PlannedMoveOutcome(False, "cancelled")

    def generate(
        self,
        *,
        current_q: Sequence[float],
        target_q: Sequence[float],
        send_debug_markers: SendDebugMarkers,
        context: Optional[Mapping[str, Any]] = None,
        go2_pos: Optional[Sequence[float]] = None,
        go2_rpy_rad: Optional[Sequence[float]] = None,
        leg_q: Optional[Sequence[float]] = None,
        environment_clearance_m: Optional[float] = None,
        cancel: Optional[threading.Event] = None,
    ) -> PlannedMoveOutcome:
        """Plan a path and publish it as debug markers. Does not move the arm.

        ``context`` should already have the arm's *current* base pose folded
        in (see ``_ControllerContextActions._with_current_arm_base``) when
        the arm is GO2-mounted -- the constructor's ``ik_context`` alone only
        reflects the static spawn assumption, which is the wrong frame for
        both FK-derived marker positions and GO2-body collision checking
        once GO2 has moved from that spawn pose. Falls back to the static
        context if none is given (fixed-base deployments). ``leg_q`` (GO2's
        live 12-value leg joint vector) is independently optional -- see
        ``elesim_controller.robot.arm.planning.collision.go2_leg_world_shapes``;
        without it the arm is still checked against GO2's torso/head, just
        not its legs. ``environment_clearance_m`` overrides the environment-
        obstacle check's tolerance -- see ``check_configuration``.

        ``cancel``, if given, lets a Cancel click interrupt a long-running
        search (RRT can take up to ``RrtConfig.max_iters`` iterations) rather
        than forcing the operator to wait it out -- reported as
        ``phase="cancelled"``, the same convention ``execute()`` already uses.
        """
        plan_context = dict(context) if context is not None else self._ik_context
        with self._lock:
            self._waypoints = []
            self._plan_context = plan_context
        if self._collision_model is None:
            self._set_status(phase="failed", message="no_collision_model", waypoint_count=0)
            return PlannedMoveOutcome(False, "no_collision_model")
        # GO2's torso/head/legs are collapsed into a single body-frame box
        # (at this call's frozen leg_q) for every check below -- ~15 GO2
        # shapes (and the per-leg forward kinematics needed to place them)
        # down to 1, at the cost of some conservatism (the merged box can
        # cover real empty space around/between individual leg shapes). See
        # simplify_go2_to_bounding_box's docstring for the full tradeoff.
        model = simplify_go2_to_bounding_box(self._collision_model, leg_q=leg_q)

        self._set_status(phase="planning", message="", waypoint_count=0)
        validity_fn = make_collision_validity_fn(
            context=plan_context,
            model=model,
            go2_pos=go2_pos,
            go2_rpy_rad=go2_rpy_rad,
            leg_q=leg_q,
            environment_clearance_m=environment_clearance_m,
        )
        current_arr = np.asarray(current_q, dtype=float).reshape(4)
        target_arr = np.asarray(target_q, dtype=float).reshape(4)
        worst_clearance = min(
            check_configuration(
                context=plan_context,
                q=current_arr + (step / 10.0) * (target_arr - current_arr),
                model=model,
                go2_pos=go2_pos,
                go2_rpy_rad=go2_rpy_rad,
                leg_q=leg_q,
                environment_clearance_m=environment_clearance_m,
            ).min_clearance_m
            for step in range(11)
        )
        print(
            f"[planned_move] straight-line worst clearance between current_q and target_q: "
            f"{worst_clearance:.4f}m ({'BLOCKED, RRT must detour' if worst_clearance < 0 else 'clear, direct connect expected'})"
        )

        plan = plan_rrt_connect(
            start_q=current_q,
            goal_q=target_q,
            context=plan_context,
            validity_fn=validity_fn,
            config=self._rrt_config,
            cancel=cancel,
        )
        print(f"[planned_move] plan_rrt_connect: success={plan.success} reason={plan.reason!r} raw_waypoints={len(plan.waypoints)}")
        if plan.reason == "cancelled":
            self._set_status(phase="cancelled", message="cancelled", waypoint_count=0)
            send_debug_markers([])
            return PlannedMoveOutcome(False, "cancelled")
        if not plan.success:
            self._set_status(phase="failed", message=plan.reason, waypoint_count=0)
            if plan.reason == "start_in_collision":
                bad_q = current_arr
                result = check_configuration(
                    context=plan_context,
                    q=bad_q,
                    model=model,
                    go2_pos=go2_pos,
                    go2_rpy_rad=go2_rpy_rad,
                    leg_q=leg_q,
                    environment_clearance_m=environment_clearance_m,
                )
            elif plan.reason == "goal_in_collision":
                # Report the *first* violation encountered while moving toward
                # the goal, not the (possibly much deeper) violation at the
                # goal itself -- see _first_violation_along_path.
                bad_q, result = _first_violation_along_path(
                    context=plan_context,
                    model=model,
                    current_q=current_arr,
                    target_q=target_arr,
                    go2_pos=go2_pos,
                    go2_rpy_rad=go2_rpy_rad,
                    leg_q=leg_q,
                    environment_clearance_m=environment_clearance_m,
                )
            else:
                bad_q, result = None, None
            if result is not None:
                print(f"[planned_move] {plan.reason}: {result.link_a!r} vs {result.link_b!r} gap={result.min_clearance_m:.4f}m")
                send_debug_markers(_collision_diagnostic_markers(context=plan_context, model=model, q=bad_q, result=result))
            else:
                send_debug_markers([])
            return PlannedMoveOutcome(False, plan.reason)

        shortcut_validity_fn = make_collision_validity_fn(
            context=plan_context,
            model=model,
            go2_pos=go2_pos,
            go2_rpy_rad=go2_rpy_rad,
            leg_q=leg_q,
            environment_clearance_m=SHORTCUT_MIN_ENVIRONMENT_CLEARANCE_M,
        )
        waypoints = shortcut_path(
            plan.waypoints,
            context=plan_context,
            validity_fn=shortcut_validity_fn,
            # Finer than shortcut_path's own 0.05 default -- needed so a candidate
            # straight-line replacement can't skip clean over a thin obstacle (see
            # RrtConfig.collision_check_resolution's docstring) -- but the RRT
            # tree-growth resolution (0.01) made every one of shortcut's 200
            # iterations re-check long spans at that resolution, measured at
            # ~28s on a real 37-waypoint path; 0.02 (already used elsewhere in
            # this codebase for the same thin-wall concern, see
            # test_shortcut_path_never_introduces_a_wall_violation) is a much
            # cheaper compromise that's still finer than the original default.
            resolution=SHORTCUT_COLLISION_CHECK_RESOLUTION,
            iterations=SHORTCUT_ITERATIONS,
            seed=self._rrt_config.seed,
            cancel=cancel,
        )
        print(f"[planned_move] after shortcut: {len(waypoints)} waypoints")
        waypoints = maximize_clearance(
            waypoints,
            context=plan_context,
            model=model,
            go2_pos=go2_pos,
            go2_rpy_rad=go2_rpy_rad,
            leg_q=leg_q,
            environment_clearance_m=environment_clearance_m,
            cancel=cancel,
        )
        print(f"[planned_move] after clearance smoothing: {len(waypoints)} waypoints")
        # maximize_clearance nudges each interior waypoint independently toward
        # whatever locally maximizes *its own* margin, with no notion of the
        # path's overall shape -- confirmed live: this can turn a reasonably
        # direct path into a visibly wiggly/zigzagging one even though every
        # point individually reports good clearance. Re-running shortcut
        # (same +margin requirement) straightens that back out, since it will
        # only accept a direct replacement that *also* keeps the margin --
        # confirmed live on the same scenario: 25 wiggly waypoints collapsed
        # back to 3 direct ones, clearances unchanged (~2cm throughout).
        waypoints = shortcut_path(
            waypoints,
            context=plan_context,
            validity_fn=shortcut_validity_fn,
            resolution=SHORTCUT_COLLISION_CHECK_RESOLUTION,
            iterations=SHORTCUT_ITERATIONS,
            seed=self._rrt_config.seed,
            cancel=cancel,
        )
        print(f"[planned_move] after re-shortcut: {len(waypoints)} waypoints")
        if cancel is not None and cancel.is_set():
            self._set_status(phase="cancelled", message="cancelled", waypoint_count=0)
            send_debug_markers([])
            return PlannedMoveOutcome(False, "cancelled")
        with self._lock:
            self._waypoints = waypoints
        # waypoints[0] is the current pose -- nothing to visualize there.
        send_debug_markers(_waypoint_markers(plan_context, waypoints[1:]))
        go2_checked = go2_pos is not None and go2_rpy_rad is not None
        message = "ready to execute" if go2_checked else "ready to execute (GO2-body check disabled: no telemetry)"
        self._set_status(phase="planned", message=message, waypoint_count=len(waypoints) - 1)
        return PlannedMoveOutcome(True, "planned")

    def execute(
        self,
        *,
        client: Any,
        cancel: threading.Event,
        send_debug_markers: SendDebugMarkers,
    ) -> PlannedMoveOutcome:
        """Stream the path from the most recent successful ``generate()``."""
        with self._lock:
            waypoints = list(self._waypoints)
            plan_context = self._plan_context
        if not waypoints:
            outcome = PlannedMoveOutcome(False, "nothing_generated")
            self._set_status(phase="failed", message=outcome.reason, waypoint_count=0)
            return outcome

        trajectory = time_parameterize(waypoints, rates=self._rates)
        stream = resample(trajectory, tick_hz=self._tick_hz)
        if not stream:
            outcome = PlannedMoveOutcome(False, "empty_trajectory")
            self._set_status(phase="failed", message=outcome.reason, waypoint_count=0)
            send_debug_markers([])
            return outcome

        try:
            self._ownership.acquire(ControlOwner.PLANNED_MOVE)
        except ControlOwnershipError as exc:
            outcome = PlannedMoveOutcome(False, f"ownership_denied: {exc}")
            self._set_status(phase="failed", message=outcome.reason)
            return outcome

        self._set_status(phase="executing", message="", waypoint_count=len(waypoints) - 1)
        # Marker clearing must track the *real, observed* arm, not the
        # commanded stream's assumed schedule -- the real arm (cable sag,
        # inertia) tracks the commanded trajectory with lag, so a
        # time-scheduled clear can fire well before the arm is actually
        # anywhere near that point. arrival_times is kept only as a
        # stuck-safeguard: if the real tip never gets within tolerance
        # (feedback missing, or sag prevents full convergence), a waypoint
        # is still force-cleared once its commanded time is well past, so a
        # marker can't linger forever.
        waypoint_positions = [
            np.asarray(ik_kin._forward_grasp_world(dict(plan_context), q), dtype=float) for q in waypoints
        ]
        arrival_times = [sample.t_s for sample in trajectory.samples]
        next_waypoint_idx = 1  # trajectory.samples[0] is the start pose, already "reached"
        dt = 1.0 / self._tick_hz
        outcome = PlannedMoveOutcome(True, "completed")
        try:
            tick_idx = 0
            # Fractional index into `stream`, advanced each real tick by the
            # *measured* sim_realtime_factor rather than always by 1 -- see
            # MIN_SIM_REALTIME_FACTOR's docstring. `tick_idx`/`t` stay tied to
            # real wall-clock time throughout (needed for the stuck-safeguard
            # and for `time.sleep` pacing); `stream_pos` is the only thing
            # that tracks how far the commanded reference has *actually*
            # been able to advance given the sim's real speed.
            stream_pos = 0.0
            held_pose_logged = False
            while True:
                if cancel.is_set():
                    outcome = PlannedMoveOutcome(False, "cancelled")
                    break
                stream_idx = min(int(stream_pos), len(stream) - 1)
                q = stream[stream_idx]
                stream_exhausted = stream_idx >= len(stream) - 1
                # Once the commanded stream is exhausted, keep holding the
                # final pose and keep polling real feedback -- the commanded
                # schedule is derived from rated (not measured) joint speeds,
                # so the real arm can still be well short of the goal after
                # the last sample is sent. Without this, waypoints still
                # unconfirmed by real feedback got force-cleared the instant
                # streaming stopped, regardless of where the arm actually was.
                if stream_exhausted and not held_pose_logged:
                    held_pose_logged = True
                    print(
                        f"[planned_move] commanded stream exhausted at tick={tick_idx} with "
                        f"{len(waypoint_positions) - next_waypoint_idx} waypoint(s) not yet confirmed by "
                        "real feedback -- holding last pose and waiting for arrival or safeguard timeout"
                    )
                client.send_target_values(
                    linear_m=float(q[0]),
                    roll_rad=float(q[1]),
                    theta1_rad=float(q[2]),
                    theta2_rad=float(q[3]),
                    source="planned_move",
                    # Every other target-sending call site in client.py passes
                    # force=True; this one didn't, so each tick's command was
                    # coalesced into a single last-write-wins pending slot
                    # instead of going straight out -- a real, if partial,
                    # contributor to commands lagging behind the intended
                    # per-tick schedule during streamed execution.
                    force=True,
                )
                t = tick_idx * dt
                host_state = client.refresh_state() if hasattr(client, "refresh_state") else None
                actual_tip = getattr(host_state, "actual_tip_xyz", None) if host_state is not None else None
                actual_tip_arr = None if actual_tip is None else np.asarray(actual_tip, dtype=float)
                # host_state.q is the *simulator's own reported* applied 4dof q
                # (SimMover.current_4dof_q(), echoed back as telemetry "q" with
                # q_source="simulated") -- comparing it against the q we just
                # commanded tells us whether an unconverged actual_tip is a
                # real command-delivery/tracking problem (reported_q stays far
                # from q) or a stale/wrong feedback computation (reported_q
                # already matches q but actual_tip doesn't reflect it).
                reported_q = getattr(host_state, "q", None) if host_state is not None else None
                # sim_realtime_factor is sim-seconds-per-wall-second (SimScene.
                # sim_realtime_factor -- 1.0 means the sim keeps pace with the
                # clock). SimMover's rate limiter passes a *fixed* dt=params.dt
                # to RateLimiter.step every call, assuming each call represents
                # exactly that much elapsed time -- if the sim is actually
                # running below realtime (heavy rendering/GPU load), each call
                # covers *more* real wall-clock time than dt assumes, so the
                # effective real-time joint speed is bend_rate * realtime_factor,
                # not bend_rate. A value well under 1.0 here would explain a
                # bend axis that tracks at a fraction of its configured rate
                # even though roll/linear (much faster, so still "fast enough"
                # despite the same slowdown) look fine.
                realtime_factor = getattr(host_state, "sim_realtime_factor", None) if host_state is not None else None
                effective_realtime_factor = (
                    1.0
                    if realtime_factor is None or realtime_factor <= 0.0
                    else max(float(realtime_factor), MIN_SIM_REALTIME_FACTOR)
                )
                # Advance the commanded reference only as fast as the sim can
                # actually realize it -- at effective_realtime_factor=1 this
                # is exactly "+1 index per real tick" (the old behavior).
                stream_pos += effective_realtime_factor
                if tick_idx == 0:
                    print(
                        f"[planned_move] execute tick0: host_state={'none' if host_state is None else 'ok'} "
                        f"actual_tip_xyz={actual_tip} reported_q={reported_q} commanded_q={np.round(q, 4).tolist()} "
                        f"sim_realtime_factor={realtime_factor}"
                    )
                while next_waypoint_idx < len(waypoint_positions):
                    dist = (
                        None
                        if actual_tip_arr is None
                        else float(np.linalg.norm(actual_tip_arr - waypoint_positions[next_waypoint_idx]))
                    )
                    reached_by_position = dist is not None and dist <= WAYPOINT_REACHED_TOLERANCE_M
                    effective_arrival_s = arrival_times[next_waypoint_idx] / effective_realtime_factor
                    reached_by_timeout = t >= max(
                        STUCK_WAYPOINT_TIMEOUT_SCALE * effective_arrival_s,
                        effective_arrival_s + STUCK_WAYPOINT_MIN_EXTRA_S,
                    )
                    if not (reached_by_position or reached_by_timeout):
                        break
                    print(
                        f"[planned_move] clearing waypoint {next_waypoint_idx} at tick={tick_idx} t={t:.3f}s "
                        f"target_pos={np.round(waypoint_positions[next_waypoint_idx], 4).tolist()} "
                        f"actual_tip={None if actual_tip_arr is None else np.round(actual_tip_arr, 4).tolist()} "
                        f"dist={dist if dist is None else round(dist, 4)} "
                        f"reason={'position' if reached_by_position else 'timeout_fallback'} "
                        f"commanded_q={np.round(q, 4).tolist()} reported_q={reported_q} "
                        f"sim_realtime_factor={realtime_factor}"
                    )
                    next_waypoint_idx += 1
                    send_debug_markers(_waypoint_markers(plan_context, waypoints[next_waypoint_idx:]))
                    self._set_status(waypoint_count=len(waypoints) - next_waypoint_idx)
                self._ownership.heartbeat(ControlOwner.PLANNED_MOVE)
                if stream_exhausted and next_waypoint_idx >= len(waypoint_positions):
                    break
                tick_idx += 1
                time.sleep(dt)
        finally:
            self._ownership.release(ControlOwner.PLANNED_MOVE)
            send_debug_markers([])
            with self._lock:
                self._waypoints = []

        if outcome.reason == "cancelled":
            phase = "cancelled"
        elif outcome.success:
            phase = "done"
        else:
            phase = "failed"
        self._set_status(phase=phase, message=outcome.reason, waypoint_count=0)
        return outcome


__all__ = ["PlannedMoveExecutor", "PlannedMoveOutcome", "PlannedMoveStatus"]
