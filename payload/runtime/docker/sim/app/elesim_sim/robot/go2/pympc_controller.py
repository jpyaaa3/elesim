"""Genesis GO2 torque controller using Quadruped-PyMPC's acados GRF solve.

This backend does not import the old convex gait, robot model or leg controller.
The source profile selects this experimental backend; gait acceptance remains
separate from successful solver initialization.
"""

from __future__ import annotations

from dataclasses import replace
from functools import cached_property
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from elesim_sim.robot.go2.locomotion.kinematics import (
    GO2_LEG_JOINTS,
    HIP_OFFSET_BODY,
    NOMINAL_FOOT_OFFSET_BODY,
    Go2KinematicsModel,
)
from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper
from elesim_sim.robot.go2.locomotion.pose import JointPoseTransition, smoothstep_quintic
from elesim_sim.robot.go2.locomotion.types import Go2Command, LegId
from elesim_sim.robot.go2.mpc.contact_diagnostics import GenesisContactDiagnostics
from elesim_sim.robot.go2.mpc.control_rate import ControlRateInfo
from elesim_sim.robot.go2.mpc.genesis_pin_bridge import GenesisPinBridge, synchronize_model_inertias
from elesim_sim.robot.go2.mpc.payload_model import ArmPayloadCompensator
from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, PyMpcInput, project_support_motor_limits
from elesim_sim.robot.go2.foot_workspace import project_ground_target, reachable
from elesim_sim.simulation.genesis.utils import to_numpy_1d
from elesim_sim.simulation.genesis.control_state import EntityControlStateReader


_LEGS = (LegId.FL, LegId.FR, LegId.RL, LegId.RR)
_PHASE_OFFSETS = np.array((0.0, 0.5, 0.5, 0.0))
_SWING_TOUCHDOWN_DEPTH_M = .008
# Current footholds target the flat floor. A 22 mm foot sphere with a 2 mm
# local offset cannot support the body once its frame clears 30 mm.
_MAX_GROUND_FOOT_HEIGHT_M = .03
_MIN_SUPPORT_FORCE_N = 10.0
# Retain leading hip height and lower the trailing body reference by 1 cm.
# This is a body reference trim, not a change to the ground/contact geometry.
_WALK_TRAILING_CROUCH_M = .01


def commanded_progress_velocity(actual_velocity, commanded_velocity):
    """Bound existing horizontal progress to the shaped operator intent.

    Keep vertical/sideways motion, opposite motion and overspeed damped.
    A released or purely rotational command supplies no translation reference.
    """
    actual = np.asarray(actual_velocity, dtype=float)
    command = np.asarray(commanded_velocity, dtype=float)
    if (actual.shape != (3,) or command.shape != (3,)
            or not np.isfinite(actual).all() or not np.isfinite(command).all()):
        raise ValueError('commanded progress needs finite three-dimensional velocities')
    speed = float(np.linalg.norm(command[:2]))
    result = np.zeros(3)
    if speed > 1e-9:
        direction = command[:2]/speed
        result[:2] = direction*np.clip(np.dot(actual[:2], direction), 0., speed)
    return result


def trailing_crouch_reference(sample, *, command_vx, crouch_m):
    """Keep leading hip reference height while slightly bending the rear."""
    if not np.isfinite(command_vx) or not np.isfinite(crouch_m) or not 0. <= crouch_m <= .03:
        raise ValueError('trailing crouch needs finite velocity and 0..0.03 m drop')
    if crouch_m == 0. or command_vx == 0.:
        return sample
    signed_drop = crouch_m*np.clip(command_vx/.2, -1., 1.)
    position = sample.desired_com_position.copy()
    rpy = sample.desired_rpy.copy()
    position[2] -= .5*abs(signed_drop)
    rpy[1] -= np.arcsin(signed_drop/(2.*.1934))
    return replace(sample, desired_com_position=position, desired_rpy=rpy)


def landing_velocity_reference(sample, *, blocked, actual_velocity, command_world, horizon_s):
    """Hold bounded existing progress while waiting for physical landing.

    Contact confirmation still owns support and new liftoffs. Holding the
    command must not repeatedly request a full stop at every load handover.
    No acceleration toward an unachieved speed or sideways drift is added.
    """
    if not blocked:
        return sample
    velocity = commanded_progress_velocity(actual_velocity, command_world)
    position = sample.desired_com_position.copy()
    position[:2] = sample.com_position[:2]+horizon_s*velocity[:2]
    return replace(sample, desired_com_position=position, desired_com_velocity=velocity)


def swing_damping_reference_torque(jacobian, joint_velocity, measured_velocity,
                                   desired_velocity, *, damping, velocity_limits):
    """Cancel damping of reference motion, retaining damping of tracking error.

    Foot velocities and the leg Jacobian are world aligned. Subtract the
    measured base contribution before solving for the reference joint speed.
    The minimum-norm solve leaves unresolved joint directions damped.
    """
    jacobian = np.asarray(jacobian, dtype=float)
    joint_velocity = np.asarray(joint_velocity, dtype=float)
    measured_velocity = np.asarray(measured_velocity, dtype=float)
    desired_velocity = np.asarray(desired_velocity, dtype=float)
    limits = np.asarray(velocity_limits, dtype=float)
    if (jacobian.shape != (3, 3) or any(value.shape != (3,) for value in
            (joint_velocity, measured_velocity, desired_velocity, limits))
            or not all(np.all(np.isfinite(value)) for value in
                (jacobian, joint_velocity, measured_velocity, desired_velocity, limits))
            or np.any(limits <= 0.) or not np.isfinite(damping) or damping < 0.):
        raise ValueError("swing damping needs finite 3D inputs and positive velocity limits")
    base_velocity = measured_velocity - jacobian @ joint_velocity
    reference = np.linalg.lstsq(jacobian, desired_velocity - base_velocity, rcond=1e-3)[0]
    # A common scale preserves the resolved direction. Clipping each joint
    # independently can inject a null-space command at a singular pose.
    reference /= max(1., float(np.max(np.abs(reference) / limits)))
    return float(damping) * reference


def support_actuator_constraints(jacobians, raw_torque_bias, auxiliary_torque,
                                  torque_limits, actuation_scales):
    """Map applied GRF into the two existing motor clipping stages.

    Freeze measured leg Jacobians over this short horizon. The first box is
    before the startup gain; the second includes that gain and auxiliary PD.
    Passive Genesis damping is outside the motor command and is not a motor
    torque offset. No actuator bound is raised.
    """
    jacobians = np.asarray(jacobians, dtype=float)
    raw_bias = np.asarray(raw_torque_bias, dtype=float)
    auxiliary = np.asarray(auxiliary_torque, dtype=float)
    limits = np.asarray(torque_limits, dtype=float)
    gains = np.asarray(actuation_scales, dtype=float)
    if (jacobians.shape != (4, 3, 3) or any(value.shape != (12,) for value in
            (raw_bias, auxiliary, limits)) or gains.ndim != 1 or len(gains) < 1
            or not all(np.isfinite(value).all() for value in
                       (jacobians, raw_bias, auxiliary, limits, gains))
            or np.any(limits <= 0.) or np.any((gains <= 0.) | (gains > 1.))):
        raise ValueError('support actuator constraints need finite leg inputs, positive limits and gains')
    shape = (len(gains), 4, 3, 3)
    final_maps = np.broadcast_to(-jacobians.transpose(0, 2, 1), shape)
    maps = np.concatenate((final_maps/gains[:, None, None, None], final_maps), axis=2)
    raw_bias = np.broadcast_to(raw_bias.reshape(4, 3), shape[:-1])
    final_bias = gains[:, None, None]*raw_bias + auxiliary.reshape(4, 3)
    offsets = np.concatenate((raw_bias, final_bias), axis=2)
    limits = np.tile(limits.reshape(4, 3), (len(gains), 1, 2))
    return maps, offsets, limits


def solver_update_stride(sim_dt: float, prediction_dt: float, *, backend: str) -> int:
    """Refresh GPU forces within one prediction stage; retain CPU cadence."""
    if not np.isfinite([sim_dt, prediction_dt]).all() or min(sim_dt, prediction_dt) <= 0.:
        raise ValueError("solver update intervals must be finite and positive")
    if backend == "acados":
        return max(1, int(round(1./(25.*sim_dt))))
    if backend != "jax_mppi":
        raise ValueError(f"unknown PyMPC backend: {backend}")
    return max(1, int(np.floor(prediction_dt/sim_dt + 1e-9)))


class ContactGatedGait:
    """Admit new liftoffs only after their remaining support actually lands."""

    def __init__(self):
        self.time = 0.0
        self.phase_rate = 0.0
        self.blocked = True
        self.confirmed = np.zeros(4, dtype=bool)
        self.contact_age = np.zeros(4)
        self.loss_age = np.zeros(4)
        self.missing_age = np.zeros(4)
        self.search = np.zeros(4, dtype=bool)
        self.load = np.zeros(4)
        self._contact_window = []
        self._previous_planned = np.ones(4, dtype=bool)
        self._swinging = np.zeros(4, dtype=bool)
        self._finished_swing = np.zeros(4, dtype=bool)
        self.swing_elapsed = np.zeros(4)
        self.swing_progress = np.zeros(4)
        self.swing_remaining = np.zeros(4)
        self.observed_normal = np.zeros(4)
        self.observed_heights = np.zeros(4)

    def prime(self, normal_forces, *, dt):
        """Observe the already loaded feet while position control holds ready."""
        normal = np.asarray(normal_forces, dtype=float)
        if normal.shape != (4,) or not np.all(np.isfinite(normal)):
            raise RuntimeError("invalid foot contact force feedback")
        self.contact_age = np.where(normal > 5.0, self.contact_age + dt, 0.0)
        self.confirmed = self.contact_age >= 0.04 - 1e-9
        # Existing support must not be unloaded and ramped again at startup.
        self.load = self.confirmed.astype(float)
        self.blocked = not bool(np.all(self.confirmed))

    def update(self, normal_forces, *, dt, gait_hz, duty, horizon_dt, horizon, settling,
               foot_heights, support_handover=False):
        normal = np.asarray(normal_forces, dtype=float)
        if normal.shape != (4,) or not np.all(np.isfinite(normal)):
            raise RuntimeError("invalid foot contact force feedback")
        heights = np.asarray(foot_heights, dtype=float)
        if heights.shape != (4,) or not np.all(np.isfinite(heights)):
            raise RuntimeError("invalid foot height feedback")
        self.observed_normal = normal.copy()
        self.observed_heights = heights.copy()
        # Observe loss BEFORE advancing the clock across the next liftoff.
        old_planned = contact_schedule(self.time, gait_hz=gait_hz, duty=duty,
                                       dt=horizon_dt, horizon=horizon)[:, 0].astype(bool)
        airborne = heights > _MAX_GROUND_FOOT_HEIGHT_M
        loss_now = airborne | ((normal <= 2.0) & (self.loss_age + dt >= 0.08 - 1e-9))
        if support_handover and duty > .5:
            loaded = self.confirmed & (self.load >= 1.-1e-9) & ~loss_now
            proposed = contact_schedule(self.time+dt, gait_hz=gait_hz, duty=duty,
                                        dt=horizon_dt, horizon=horizon)[:, 0].astype(bool)
            if np.any(old_planned & ~proposed):
                # Before crossing a liftoff, every foot that will remain in
                # support needs full measured loading. Departing feet may
                # already be unloaded; they do not own the next support set.
                supported = bool(np.all(loaded[proposed]))
            elif np.all(old_planned):
                # Retain the landing ramp's original pause. Only the incoming
                # diagonal can release it; an unloaded departing foot must
                # not then prevent that verified pair from taking over.
                incoming_offset = 0. if (self.time*gait_hz) % 1. < .5 else .5
                incoming = _PHASE_OFFSETS == incoming_offset
                supported = bool(np.all(loaded[incoming]))
            else:
                supported = bool(not self.blocked and np.all(loaded[old_planned]))
            self.phase_rate = float(supported and not settling)
        else:
            self.phase_rate = float(not self.blocked and not settling
                                    and not np.any(old_planned & loss_now))
        if self.phase_rate:
            self.time += dt
        schedule = contact_schedule(
            self.time, gait_hz=gait_hz, duty=duty, dt=horizon_dt, horizon=horizon,
        )
        if settling:
            schedule.fill(1.0)
        nominal = schedule[:, 0].astype(bool)
        liftoff = self._previous_planned & ~nominal
        self._previous_planned = nominal.copy()
        self._swinging |= liftoff
        self.swing_elapsed[self._swinging] += dt
        self.swing_elapsed[liftoff] = 0.0
        duration = (1.0 - duty) / gait_hz
        finished = self._swinging & (self.swing_elapsed >= duration - 1e-9)
        self._swinging &= ~finished
        self._finished_swing = (self._finished_swing | finished) & ~nominal
        self.swing_progress = np.clip(self.swing_elapsed / duration, 0.0, 1.0)
        self.swing_remaining = np.where(self._swinging, np.maximum(0., duration - self.swing_elapsed), 0.)
        # Holding new liftoffs must not suspend feet already in the air.
        planned = (nominal | self._finished_swing) & ~self._swinging
        # Liftoff explicitly clears the old contact. During stance tolerate a
        # brief unloading impulse, but never retain a visibly airborne foot.
        self.confirmed &= planned
        self.loss_age = np.where(planned & (normal <= 2.0), self.loss_age + dt, 0.0)
        lost = (self.loss_age >= 0.08 - 1e-9) | airborne
        self.confirmed &= ~lost
        # Genesis impulse contacts may alternate loaded/unloaded ticks at
        # 50 Hz. Require 40 ms of evidence within 80 ms, not uninterrupted
        # force. Clear all old evidence for a deliberately lifted foot.
        ground_contact = planned & ~airborne & (normal > 5.0)
        self._contact_window.append(ground_contact.astype(float) * dt)
        self._contact_window = self._contact_window[-max(1, int(np.ceil(.08 / dt))):]
        for evidence in self._contact_window:
            evidence[~planned | airborne] = 0.0
        self.contact_age = np.sum(self._contact_window, axis=0)
        self.confirmed |= ground_contact & (self.contact_age >= 0.04 - 1e-9)
        # Load age is independent of instantaneous contact force. A brief
        # unloading must not repeatedly restart the landing ramp.
        self.load = np.where(self.confirmed, np.minimum(1.0, self.load + dt / 0.06), 0.0)
        self.search = planned & ~self.confirmed
        self.missing_age = np.where(self.search, self.missing_age + dt, 0.0)
        if np.any(self.missing_age > 0.30):
            legs = ",".join(_LEGS[i].value for i in np.flatnonzero(self.search))
            raise RuntimeError(f"touchdown timeout after 0.30s: {legs}")
        self.blocked = bool(np.any(planned & (self.load < 1.0)))
        support = (planned & self.confirmed).astype(float)
        if self.blocked or np.any(self._finished_swing):
            # Do not promise future support or lift the old pair while waiting.
            schedule[:] = support[:, None]
        else:
            schedule[:, 0] = support
        return schedule


def planned_footholds(feet, candidates, contacts, previous_contacts, touchdowns) -> np.ndarray:
    """Give MPC the same touchdown held by the swing controller until landing."""
    result = np.asarray(feet, dtype=float).copy()
    swinging = np.asarray(contacts) == 0
    continuing = swinging & (np.asarray(previous_contacts) == 0)
    result[swinging] = np.asarray(candidates)[swinging]
    result[continuing] = np.asarray(touchdowns)[continuing]
    return result


def future_touchdown_times(contacts, *, gait_time: float, gait_hz: float) -> np.ndarray:
    """Nominal time to recontact only for a stance leg that cycles in horizon."""
    support = np.asarray(contacts, dtype=float)
    if support.ndim != 2 or support.shape[0] != 4 or gait_hz <= 0:
        raise ValueError("future touchdown needs four contact rows and positive gait rate")
    phase = (float(gait_time) * gait_hz + _PHASE_OFFSETS) % 1.0
    recontact = np.any((support[:, 1:] > support[:, :-1]), axis=1)
    return np.where((support[:, 0] == 1.) & recontact, (1. - phase) / gait_hz, 0.)


def swing_foot_reference(progress, start, touchdown, *, duration_s, height_m=0.06,
                         phase_rate=1.0, touchdown_depth_m=0.0):
    """Position and matching world velocity for the bounded swing trajectory."""
    progress = float(np.clip(progress, 0.0, 1.0))
    smooth = progress * progress * (3.0 - 2.0 * progress)
    endpoint = np.asarray(touchdown, dtype=float).copy()
    # A small actuator penetration creates downward load at the support
    # plane. The latched MPC foothold remains on the nominal ground surface.
    endpoint[2] -= float(touchdown_depth_m)
    delta = endpoint - np.asarray(start)
    target = np.asarray(start) + smooth * delta
    target[2] += height_m * 16.0 * progress**2 * (1.0 - progress)**2
    velocity = 6.0 * progress * (1.0 - progress) / duration_s * delta
    velocity[2] += height_m * 32.0 * progress * (1.0 - progress) * (1.0 - 2.0 * progress) / duration_s
    return target, velocity * phase_rate


def contact_schedule(time_s: float, *, gait_hz: float, duty: float, dt: float, horizon: int) -> np.ndarray:
    """FL/RR and FR/RL alternate; a column is one NMPC horizon step."""
    if gait_hz <= 0 or not 0 < duty < 1 or dt <= 0 or horizon < 2:
        raise ValueError("invalid trot contact schedule parameters")
    phase = (time_s + np.arange(horizon) * dt) * gait_hz
    return (((phase[None, :] + _PHASE_OFFSETS[:, None]) % 1.0) < duty).astype(float)


def touchdown_offsets_body(
    nominal_foot_offsets: np.ndarray,
    *,
    command_body: np.ndarray,
    yaw_rate: float,
    half_stance_s: float,
    placement_scale: float,
) -> np.ndarray:
    """Return nominal foot centers plus scaled stride and yaw prediction."""
    feet = np.asarray(nominal_foot_offsets, dtype=float)
    command = np.asarray(command_body, dtype=float).reshape(-1)
    if feet.ndim != 2 or feet.shape[1] != 3 or command.shape != (3,):
        raise ValueError("touchdown offsets need Nx3 feet and a 3D body command")
    if not np.all(np.isfinite(feet)) or not np.all(np.isfinite(command)):
        raise ValueError("touchdown offsets and command must be finite")

    yaw_lead = Rotation.from_euler(
        "z", float(half_stance_s) * float(yaw_rate)
    ).as_matrix()
    offsets = feet @ yaw_lead.T
    # Scale the commanded stride while retaining each leg's true nominal
    # lateral stance width. Scaling the hip pivot itself pulls touchdown points
    # inboard because the URDF thigh joint sits 95.5 mm outboard of each pivot.
    offsets[:, :2] += float(half_stance_s) * float(placement_scale) * command[None, :2]
    return offsets


def touchdown_candidates_world(base_position, placements, *, yaw_rotation,
                               velocity_world, desired_velocity_world,
                               remaining_swing_s=0.0,
                               travel_correction_world=None) -> np.ndarray:
    """Predict horizontal body travel before touchdown, then add stance lead."""
    candidates = np.asarray(base_position) + np.asarray(placements) @ yaw_rotation.T
    remaining = np.broadcast_to(np.asarray(remaining_swing_s, dtype=float), (len(candidates),))
    if not np.all(np.isfinite(remaining)) or np.any(remaining < 0.0):
        raise ValueError("remaining swing time must be finite and nonnegative")
    travel = remaining[:, None] * np.asarray(velocity_world, dtype=float)[None, :2]
    if travel_correction_world is not None:
        correction = np.asarray(travel_correction_world, dtype=float)
        if correction.shape != travel.shape or not np.isfinite(correction).all():
            raise ValueError("touchdown travel correction must be finite Nx2")
        travel += correction
    # Bound the added prediction, including diagonal motion, so a velocity
    # spike cannot move the landing target arbitrarily far away. This is an
    # incremental bound, not a full kinematic reachability guarantee.
    travel *= np.minimum(1.0, 0.15 / np.maximum(np.linalg.norm(travel, axis=1), 1e-12))[:, None]
    candidates[:, :2] += travel
    # Land toward excess body motion to brake it. Limit the correction so a
    # transient velocity estimate cannot drag a foot outside its local stride.
    candidates[:, :2] += np.clip(
        0.1 * (np.asarray(velocity_world)[:2] - np.asarray(desired_velocity_world)[:2]),
        -0.04, 0.04,
    )
    return candidates


def accelerating_touchdown_candidates(
    base_position, nominal_offsets, *, yaw_rotation, velocity_world,
    command_shaper: Go2CommandShaper, yaw_rate: float, remaining_swing_s,
    stance_duration_s: float, placement_scale: float, body_height_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Plan GPU liftoff/recontact using the held command's acceleration.

    Use one integrated trajectory for travel to landing and the lead over the
    first nominal half of support, preserving the established cruise stride.
    End-of-support reach also covers the existing 60 ms load handover and
    restoration of ready body height;
    neither a moving swing endpoint nor vertical foot travel is introduced.
    """
    base = np.asarray(base_position, dtype=float)
    feet = np.asarray(nominal_offsets, dtype=float)
    times = np.broadcast_to(np.asarray(remaining_swing_s, dtype=float), (len(feet),))
    if not np.isfinite(times).all() or np.any(times < 0.):
        raise ValueError("remaining swing time must be finite and nonnegative")
    support_s = float(stance_duration_s) + .06
    half_s = .5 * float(stance_duration_s)
    current = command_shaper.current
    command = np.array([current.vx, current.vy, 0.])
    yaw = np.asarray(yaw_rotation, dtype=float)
    desired = yaw @ command
    velocity = np.asarray(velocity_world, dtype=float)
    placements = touchdown_offsets_body(
        feet, command_body=np.zeros(3), yaw_rate=yaw_rate,
        half_stance_s=.5*float(stance_duration_s), placement_scale=placement_scale,
    )
    to_landing = np.array([command_shaper.preview_planar_displacement(t) for t in times])
    to_middle = np.array([command_shaper.preview_planar_displacement(t+half_s) for t in times])
    to_end = np.array([command_shaper.preview_planar_displacement(t+support_s) for t in times])
    placements[:, :2] += float(placement_scale) * (to_middle-to_landing)
    correction = np.c_[to_landing-times[:, None]*command[:2], np.zeros(len(feet))] @ yaw.T
    candidates = touchdown_candidates_world(
        base, placements, yaw_rotation=yaw, velocity_world=velocity,
        desired_velocity_world=desired, remaining_swing_s=times,
        travel_correction_world=correction[:, :2],
    )
    candidates[:, 2] = .025
    # Keep excess measured travel in the forecast, but include the upcoming
    # acceleration instead of extrapolating today's low shaped speed forever.
    forecast = np.where(np.abs(desired) > np.abs(velocity), desired, velocity).copy()
    forecast[2] = 0.
    landing = base + times[:, None]*forecast + correction
    end_correction = np.c_[to_end-(times+support_s)[:, None]*command[:2],
                           np.zeros(len(feet))] @ yaw.T
    end = base + (times+support_s)[:, None]*forecast + end_correction
    landing[:, 2] = end[:, 2] = max(float(base[2]), float(body_height_m))
    return candidates, landing, end


def stance_end_foothold(
    leg: int, target: np.ndarray, base_position: np.ndarray,
    base_rotation: np.ndarray, body_velocity_world: np.ndarray,
    desired_velocity_world: np.ndarray, remaining_swing_s: float,
    stance_duration_s: float, lower: np.ndarray, upper: np.ndarray,
    *, predicted_base_positions=None,
) -> tuple[np.ndarray, bool]:
    """Keep a new touchdown in reach through its predicted support interval.

    A target can be easy to reach when the foot first lands, yet trail beyond
    the calf extension limit as the body travels over it. Use the larger of
    measured and requested horizontal speed for that travel estimate. The
    extra 60 ms covers the contact gate's landing load ramp, which holds the
    gait clock. During initial forward/backward acceleration, blend the
    forecast toward commanded speed only after measured progress is present;
    otherwise distant goals can brake the body before it starts walking.
    The GPU supplies acceleration/height-aware landing and end positions
    explicitly, overriding this legacy constant-velocity estimate.
    Only plan this at liftoff; a planted foot stays fixed in world.
    """
    velocity = np.asarray(body_velocity_world, dtype=float)
    desired = np.asarray(desired_velocity_world, dtype=float)
    forecast = np.where(np.abs(desired) > np.abs(velocity), desired, velocity).copy()
    body_command = np.asarray(base_rotation, dtype=float).T @ desired
    command_speed = float(np.linalg.norm(desired[:2]))
    if (command_speed > .2
            and abs(body_command[0]) > 1.5 * abs(body_command[1])):
        direction = desired[:2] / command_speed
        actual_progress = float(np.dot(velocity[:2], direction))
        requested_progress = float(np.dot(forecast[:2], direction))
        blend = float(np.clip((actual_progress - .15) / .20, 0., 1.))
        forecast[:2] -= (1. - blend) * (requested_progress - actual_progress) * direction
    forecast[2] = 0.0
    base = np.asarray(base_position, dtype=float)
    landing_base = base + forecast * float(remaining_swing_s)
    end_base = base + forecast * (float(remaining_swing_s) + float(stance_duration_s) + .06)
    if predicted_base_positions is not None:
        predicted = np.asarray(predicted_base_positions, dtype=float)
        if predicted.shape != (2, 3) or not np.isfinite(predicted).all():
            raise ValueError("foothold prediction needs finite landing/end body positions")
        landing_base, end_base = predicted
    projected, feasible = project_ground_target(
        leg, target, end_base, base_rotation, lower, upper)
    if feasible and reachable(leg, projected, landing_base, base_rotation, lower, upper):
        return projected, True
    return np.asarray(target, dtype=float).copy(), False


def payload_aware_com_height(
    current_com_z: float, nominal_com_z: float, *, payload_active: bool,
    base_height: float,
) -> float:
    """Account for raised payload COM while retaining body height feedback."""
    nominal = float(nominal_com_z)
    if not payload_active:
        return nominal
    return nominal + float(current_com_z) - float(base_height)


class PyMpcGenesisController:
    def __init__(
        self,
        entity,
        *,
        dt: float,
        config,
        arm_entity=None,
        arm_link_names: set[str] | None = None,
        metrics=None,
        command_source: str = "teleop",
        go2_urdf_path: str | Path | None = None,
        timing_sink=None,
        solver_factory=None,
    ) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise RuntimeError("Pinocchio is required by the PyMPC GO2 backend") from exc
        if go2_urdf_path is None:
            from elesim_sim.model_bundle import resolve_model_bundle

            go2_urdf_path = resolve_model_bundle() / "assets/go2/go2.urdf"
        urdf = Path(go2_urdf_path).expanduser().resolve()
        if not urdf.is_file():
            raise FileNotFoundError(f"GO2 PyMPC URDF not found: {urdf}")
        self._pin = pin
        self._model = pin.buildModelFromUrdf(str(urdf), pin.JointModelFreeFlyer())
        if self._model.nq != 19 or self._model.nv != 18:
            raise RuntimeError("GO2 PyMPC expects a floating base and twelve leg DOFs")
        if tuple(self._model.names[2:]) != GO2_LEG_JOINTS:
            raise RuntimeError("GO2 PyMPC Pinocchio joint order differs from Genesis")
        self._frames = tuple(self._model.getFrameId(f"{leg.value}_foot") for leg in _LEGS)
        if any(frame >= self._model.nframes for frame in self._frames):
            raise RuntimeError("GO2 PyMPC URDF is missing a foot frame")
        synchronize_model_inertias(self._model, entity)
        self._data = self._model.createData()
        self._foot_lower = self._model.lowerPositionLimit[7:19].reshape(4, 3).copy() + .04
        self._foot_upper = self._model.upperPositionLimit[7:19].reshape(4, 3).copy() - .04
        self._swing_velocity_limits = np.asarray(self._model.velocityLimit[6:18], dtype=float).copy()
        if (not np.all(np.isfinite(self._swing_velocity_limits))
                or np.any(self._swing_velocity_limits <= 0.)):
            raise RuntimeError("GO2 swing joint velocity limits must be finite and positive")
        self._nominal_foot_offsets_body = np.asarray(
            [NOMINAL_FOOT_OFFSET_BODY[leg] for leg in _LEGS], dtype=float
        )
        # Slight hip abduction: widen each side by 1 cm without increasing
        # forward stride or rotating the foot frame. Keep front/rear symmetry.
        self._nominal_foot_offsets_body[:, 1] += (
            np.sign(self._nominal_foot_offsets_body[:, 1]) * 0.01
        )
        self._entity = entity
        self._kin = Go2KinematicsModel.from_entity(entity)
        self._leg_dof_idxs = list(self._kin.all_leg_dof_idx)
        self._bridge = GenesisPinBridge(entity, self._leg_dof_idxs)
        self._state_reader = EntityControlStateReader.for_entity(entity)
        self._payload = (
            ArmPayloadCompensator(
                arm_entity,
                mass_override_kg=float(config.payload_mass_kg),
                link_names=arm_link_names,
            )
            if arm_entity is not None and bool(config.payload_enable)
            else None
        )
        self._dt = float(dt)
        self._config = config
        self._pose_transition = JointPoseTransition(float(config.pose_transition_s))
        self._pose_transition.reset(self._kin.stand_q)
        self._pose_stage = "stand"
        self._metrics = metrics
        self._contact_diagnostics = (
            GenesisContactDiagnostics(entity, cadence_steps=max(1, round(0.1 / self._dt)))
            if metrics is not None else None
        )
        self._command_source = str(command_source)
        self._timing_sink = timing_sink
        self._rate_info = ControlRateInfo.from_sim_dt(self._dt, float(config.ctrl_hz))
        self._solver = PyMpcForceSolver(
            horizon=12,
            dt=0.02,
            friction=float(config.optimization_friction),
            max_normal_force_n=float(config.fz_max_n),
            solver_factory=solver_factory,
            backend=config.solver_backend,
            gpu_samples=config.gpu_samples,
            gpu_iterations=config.gpu_iterations,
            gpu_seed=config.gpu_seed,
        )
        self._solve_stride = solver_update_stride(
            self._dt, self._solver.dt, backend=self._solver.backend,
        )
        self._tau_lim = self._read_torque_limits()
        self._cmd = Go2Command()
        self._command_shaper = Go2CommandShaper(
            linear_accel_mps2=float(config.command_accel_mps2),
            yaw_accel_radps2=float(config.command_yaw_accel_radps2),
            stop_dwell_s=float(config.stop_dwell_s),
        )
        self._arm_q = (0.0, 0.0, 0.0, 0.0)
        self._sim_time = 0.0
        if self._contact_diagnostics is not None:
            self._contact_diagnostics.reset()
        self._active = False
        self._ready_until = 0.0
        self._walk_started_s: float | None = None
        self._settling = False
        self._torque_mode_active = False
        self._faulted = False
        self._step_i = 0
        self._force_requested = np.zeros((4, 3))
        self._forces = np.zeros((4, 3))
        self._tau_filt = np.zeros(12)
        self._tau_hold = np.zeros(12)
        self._tau_limited = np.zeros(12)
        self._tau_raw = np.zeros(12)
        self._joint_gravity = np.zeros(12)
        self._base_velocity_world = np.zeros(3)
        self._swing_starts = np.zeros((4, 3))
        self._touchdowns = np.zeros((4, 3))
        self._touchdown_depths = np.zeros(4)
        self._last_contacts = np.ones(4)
        self._contact_gait = ContactGatedGait()
        self._search_targets = np.zeros((4, 3))
        self._search_min_z = np.zeros(4)
        self._was_searching = np.zeros(4, dtype=bool)
        self._set_stand_actuation()
        self._entity.set_dofs_position(self._kin.stand_q, dofs_idx_local=self._leg_dof_idxs)
        self._apply_physics_params()
        if metrics is not None:
            metrics.set_tau_limits(self._tau_lim)
            metrics.set_control_rate_info(self._rate_info)
        print(f"[go2_pympc] backend={self._solver.backend} solve_hz=25 torque_hz="
              f"{self._rate_info.sim_hz:.1f} (experimental)")

    @property
    def control_rate_info(self) -> ControlRateInfo:
        return self._rate_info

    @property
    def tau_hold(self) -> np.ndarray:
        return self._tau_hold.copy()

    @cached_property
    def _leg_control_dofs(self) -> range | list[int]:
        """Keep static contiguous writes on Genesis' slice path.

        A list stages a GPU index array on every control call. A range selects
        the same DOFs without that allocation. Retain arbitrary joint order
        and the original list used by CPU kinematics and state indexing.
        """
        indices = self._leg_dof_idxs
        if indices and indices[0] >= 0 and all(
            index == indices[0] + offset for offset, index in enumerate(indices)
        ):
            return range(indices[0], indices[0] + len(indices))
        return indices

    @cached_property
    def _as_force_tensor(self):
        # Genesis' zero-copy setter can copy a CPU Tensor directly. NumPy
        # instead takes its extended-broadcast fallback and stages a device
        # value. Keep the original CPU float64 values and write every tick.
        from torch import from_numpy

        return from_numpy

    def _read_torque_limits(self) -> np.ndarray:
        lower, upper = self._entity.get_dofs_force_range(dofs_idx_local=self._leg_dof_idxs)
        lo, hi = to_numpy_1d(lower), to_numpy_1d(upper)
        if lo.shape != (12,) or hi.shape != (12,):
            raise RuntimeError("Genesis GO2 torque limits must have twelve values")
        limits = np.minimum(np.abs(lo), np.abs(hi)) * float(self._config.torque_safety_scale)
        if not np.all(np.isfinite(limits)) or np.any(limits <= 0):
            raise RuntimeError("Genesis GO2 torque limits are invalid")
        return limits

    def _apply_physics_params(self) -> None:
        self._entity.set_friction(float(self._config.physical_friction))
        for stem, value in (
            ("armature", self._config.joint_armature),
            ("damping", self._config.joint_damping),
            ("frictionloss", self._config.joint_frictionloss),
        ):
            expected = np.full(12, float(value))
            getattr(self._entity, f"set_dofs_{stem}")(
                expected, dofs_idx_local=self._leg_dof_idxs
            )
            actual = to_numpy_1d(
                getattr(self._entity, f"get_dofs_{stem}")(
                    dofs_idx_local=self._leg_dof_idxs
                )
            )
            if actual.shape != (12,) or not np.allclose(actual, expected, atol=1e-7):
                raise RuntimeError(f"Genesis GO2 {stem} differs from configured value")

    def _set_stand_actuation(self) -> None:
        self._entity.set_dofs_damping(
            np.full(12, float(self._config.joint_damping)), dofs_idx_local=self._leg_dof_idxs
        )
        self._entity.set_dofs_kp(
            np.full(12, float(self._config.stand_kp)), dofs_idx_local=self._leg_dof_idxs
        )
        self._entity.set_dofs_kv(
            np.full(12, float(self._config.stand_kv)), dofs_idx_local=self._leg_dof_idxs
        )

    def _set_ready_actuation(self) -> None:
        self._entity.set_dofs_damping(
            np.full(12, float(self._config.joint_damping)), dofs_idx_local=self._leg_dof_idxs
        )
        self._entity.set_dofs_kp(
            np.full(12, float(self._config.ready_kp)), dofs_idx_local=self._leg_dof_idxs
        )
        self._entity.set_dofs_kv(
            np.full(12, float(self._config.ready_kv)), dofs_idx_local=self._leg_dof_idxs
        )

    def _current_leg_pose(self) -> np.ndarray:
        try:
            pose = to_numpy_1d(
                self._entity.get_dofs_position(dofs_idx_local=self._leg_dof_idxs)
            )
            if pose.shape == (12,) and np.all(np.isfinite(pose)):
                return pose
        except Exception:
            pass
        current = self._pose_transition.current
        if current is not None and current.shape == (12,):
            return current
        return np.asarray(self._kin.stand_q, dtype=float).copy()

    def _begin_ready_pose(self) -> None:
        # No samples are taken during position control. Do not differentiate
        # across that gap or reuse the previous walking horizon.
        self._bridge.reset()
        if self._payload is not None:
            self._payload.reset()
        self._solver.reset()
        self._contact_gait = ContactGatedGait()
        self._was_searching.fill(False)
        self._pose_transition.begin(self._current_leg_pose(), self._kin.ready_q)
        self._pose_stage = "to_ready"
        self._active = True
        self._walk_started_s = None
        self._settling = False
        self._torque_mode_active = False
        self._ready_until = 0.0
        self._set_ready_actuation()

    def _begin_stand_pose(self) -> None:
        self._pose_transition.begin(self._current_leg_pose(), self._kin.stand_q)
        self._pose_stage = "to_stand"
        self._active = False
        self._walk_started_s = None
        self._settling = False
        self._torque_mode_active = False
        self._step_i = 0
        self._force_requested.fill(0.0)
        self._forces.fill(0.0)
        self._tau_filt.fill(0.0)
        self._tau_hold.fill(0.0)
        self._tau_limited.fill(0.0)
        self._tau_raw.fill(0.0)
        self._joint_gravity.fill(0.0)
        self._base_velocity_world = np.zeros(3)
        self._swing_starts.fill(0.0)
        self._touchdowns.fill(0.0)
        self._last_contacts.fill(1.0)
        self._set_stand_actuation()

    def _body_ready_to_stand(self) -> bool:
        """Retain zero-command balance control until actual motion settles."""
        q, dq = self._bridge.last_q, self._bridge.last_dq
        if q is None or dq is None or not np.isfinite(np.r_[q, dq]).all():
            return False
        tilt = Rotation.from_quat(q[3:7]).as_euler("xyz")[:2]
        return bool(np.linalg.norm(dq[:2]) < 0.08
                    and np.linalg.norm(dq[3:6]) < 0.3
                    and np.max(np.abs(tilt)) < 0.15)

    def _gait_time(self) -> float:
        return self._contact_gait.time

    def _walk_elapsed(self) -> float:
        return (0.0 if self._walk_started_s is None
                else max(0.0, self._sim_time - self._walk_started_s))

    def _command_scale(self) -> float:
        if self._walk_started_s is None:
            return 0.0
        if self._solver.backend == 'jax_mppi':
            # The shaper starts at physical walking, so a second multiplier
            # would bypass its acceleration bound while that multiplier rises.
            return 1.0
        ramp_s = max(1e-3, float(self._config.command_ramp_s))
        return smoothstep_quintic((self._sim_time - self._walk_started_s) / ramp_s)

    def _set_torque_actuation(self) -> None:
        self._entity.set_dofs_kp(np.zeros(12), dofs_idx_local=self._leg_dof_idxs)
        self._entity.set_dofs_kv(np.zeros(12), dofs_idx_local=self._leg_dof_idxs)
        # Genesis FORCE mode bypasses actuator KV. Use the passive term so
        # configured motor damping is evaluated at every physics substep.
        self._entity.set_dofs_damping(
            np.full(12, float(self._config.joint_damping) + float(self._config.leg_kv_damping)),
            dofs_idx_local=self._leg_dof_idxs,
        )
        self._swing_damping_mask = np.zeros(4, dtype=bool)

    def _set_swing_damping(self, free_swing: np.ndarray) -> None:
        """Keep substep damping in stance/search; track swing velocity actively."""
        free_swing = np.asarray(free_swing, dtype=bool)
        if free_swing.shape != (4,):
            raise ValueError("swing damping needs one mask value per leg")
        if np.array_equal(free_swing, getattr(self, "_swing_damping_mask", None)):
            return
        damping = (float(self._config.joint_damping)
                   + float(self._config.leg_kv_damping) * np.repeat(~free_swing, 3))
        self._entity.set_dofs_damping(damping, dofs_idx_local=self._leg_dof_idxs)
        self._swing_damping_mask = free_swing.copy()

    def set_command(self, cmd: Go2Command) -> None:
        self._command_shaper.set_target(cmd)

    def set_arm_q(self, arm_q: tuple[float, float, float, float]) -> None:
        self._arm_q = tuple(float(x) for x in arm_q)

    def reset(self) -> None:
        self._contact_gait = ContactGatedGait()
        self._was_searching.fill(False)
        self._search_targets.fill(0.0)
        self._search_min_z.fill(0.0)
        self._touchdown_depths.fill(0.0)
        self._cmd = Go2Command()
        self._command_shaper.reset()
        self._sim_time = 0.0
        if self._contact_diagnostics is not None:
            self._contact_diagnostics.reset()
        self._active = False
        self._ready_until = 0.0
        self._walk_started_s = None
        self._settling = False
        self._torque_mode_active = False
        self._pose_stage = "stand"
        self._pose_transition.reset(self._kin.stand_q)
        self._faulted = False
        self._step_i = 0
        self._force_requested.fill(0.0)
        self._forces.fill(0.0)
        self._tau_filt.fill(0.0)
        self._tau_hold.fill(0.0)
        self._tau_limited.fill(0.0)
        self._tau_raw.fill(0.0)
        self._joint_gravity.fill(0.0)
        self._swing_starts.fill(0.0)
        self._touchdowns.fill(0.0)
        self._last_contacts.fill(1.0)
        self._set_stand_actuation()
        self._entity.set_dofs_position(self._kin.stand_q, dofs_idx_local=self._leg_dof_idxs)
        self._entity.zero_all_dofs_velocity()
        self._bridge.reset()
        if self._payload is not None:
            self._payload.reset()
        self._solver.reset()

    def _predict_touchdowns(self, base, yaw_rotation, velocity, desired_world,
                           command_body, yaw_rate, remaining_swing_s, stance_duration_s):
        if self._solver.backend == 'jax_mppi':
            candidates, landing, end = accelerating_touchdown_candidates(
                base, self._nominal_foot_offsets_body, yaw_rotation=yaw_rotation,
                velocity_world=velocity, command_shaper=self._command_shaper,
                yaw_rate=yaw_rate, remaining_swing_s=remaining_swing_s,
                stance_duration_s=stance_duration_s,
                placement_scale=float(self._config.foot_placement_scale),
                body_height_m=float(self._config.z_pos_des_m),
            )
            return candidates, np.stack((landing, end), axis=1)
        placements = touchdown_offsets_body(
            self._nominal_foot_offsets_body, command_body=command_body,
            yaw_rate=yaw_rate, half_stance_s=.5*stance_duration_s,
            placement_scale=float(self._config.foot_placement_scale),
        )
        candidates = touchdown_candidates_world(
            base, placements, yaw_rotation=yaw_rotation, velocity_world=velocity,
            desired_velocity_world=desired_world, remaining_swing_s=remaining_swing_s,
        )
        candidates[:, 2] = .025
        return candidates, None

    def _sample(self) -> tuple[PyMpcInput, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pin = self._pin
        reader = getattr(self, '_state_reader', None)
        state_args = {} if reader is None else dict(snapshot=reader.read())
        q, dq = self._bridge.read_pin_q_dq(dt=self._dt, **state_args)
        rot = getattr(self._bridge, 'base_rotation', None)
        self._bridge.last_q = q.copy()
        self._bridge.last_dq = dq.copy()
        self._joint_gravity = np.asarray(
            pin.computeGeneralizedGravity(self._model, self._data, q), dtype=float
        )[6:18].copy()
        pin.forwardKinematics(self._model, self._data, q, dq)
        pin.updateFramePlacements(self._model, self._data)
        pin.computeJointJacobians(self._model, self._data, q)
        pin.centerOfMass(self._model, self._data, q, dq)
        pin.ccrba(self._model, self._data, q, dq)
        com = np.asarray(self._data.com[0], dtype=float).copy()
        vel = np.asarray(self._data.vcom[0], dtype=float).copy()
        if self._payload is not None:
            state = SimpleNamespace(data=self._data, pos_com_world=com, vel_com_world=vel)
            self._payload.apply(state, dt=self._dt, **state_args)
            com = np.asarray(state.pos_com_world, dtype=float).copy()
            vel = np.asarray(state.vel_com_world, dtype=float).copy()
        if rot is None:
            rot = Rotation.from_quat(q[3:7])
        rot_m = rot.as_matrix()
        rpy = rot.as_euler("xyz")
        feet = np.array([self._data.oMf[frame].translation for frame in self._frames])
        foot_vel = np.array([
            pin.getFrameVelocity(self._model, self._data, frame, pin.LOCAL_WORLD_ALIGNED).linear
            for frame in self._frames
        ])
        jacobians = np.array([
            pin.getFrameJacobian(self._model, self._data, frame, pin.LOCAL_WORLD_ALIGNED)[:3, 6 + i * 3 : 9 + i * 3]
            for i, frame in enumerate(self._frames)
        ])
        mass = float(self._data.Ig.mass)
        inertia_body = rot_m.T @ np.asarray(self._data.Ig.inertia, dtype=float) @ rot_m
        command_scale = self._command_scale()
        vx = float(self._cmd.vx) * command_scale
        vy = float(self._cmd.vy) * command_scale
        yaw_rate = float(self._cmd.yaw_rate) * command_scale
        cmd_body = np.array((vx, vy, 0.0))
        # WASD requests a horizontal body-frame velocity; body pitch/roll must
        # not turn a forward command into a requested vertical velocity.
        yaw_rot = Rotation.from_euler("z", float(rpy[2])).as_matrix()
        cmd_world = yaw_rot @ cmd_body
        horizon_t = 0.5 * self._solver.horizon * self._solver.dt
        ref_com = com + horizon_t * cmd_world
        ref_com[2] = payload_aware_com_height(
            float(com[2]),
            float(self._config.z_pos_des_m),
            payload_active=self._payload is not None,
            base_height=float(q[2]),
        )
        ref_rpy = rpy.copy()
        ref_rpy[0:2] = 0.0
        ref_rpy[2] += horizon_t * yaw_rate
        # Actual support feedback is required even with CSV diagnostics off.
        contacts = self._contact_gait.update(
            self._normal_contact_forces(**state_args), dt=self._dt,
            gait_hz=float(self._config.gait_hz),
            duty=float(self._config.gait_duty),
            horizon_dt=self._solver.dt,
            horizon=self._solver.horizon,
            settling=self._balance_only(),
            foot_heights=feet[:, 2],
            support_handover=self._solver.backend == 'jax_mppi',
        )
        if self._contact_gait.blocked:
            cmd_world = np.zeros(3)
            ref_com[:2] = com[:2]
            ref_rpy[2] = rpy[2]
            yaw_rate = 0.0
        stance_duration_s = float(self._config.gait_duty / self._config.gait_hz)
        base_velocity_world = rot_m @ dq[:3]
        self._base_velocity_world = base_velocity_world.copy()
        candidates, predicted_bases = self._predict_touchdowns(
            q[:3], yaw_rot,
            base_velocity_world if self._solver.backend == 'jax_mppi' else vel,
            cmd_world, cmd_body, yaw_rate, self._contact_gait.swing_remaining,
            stance_duration_s,
        )
        swing_previous = self._swing_previous_contacts(contacts[:, 0])
        for i in range(4):
            if contacts[i, 0] == 0 and swing_previous[i] == 1:
                projected, feasible = stance_end_foothold(
                    i, candidates[i], q[:3], rot_m, base_velocity_world, cmd_world,
                    self._contact_gait.swing_remaining[i], stance_duration_s,
                    self._foot_lower[i], self._foot_upper[i],
                    predicted_base_positions=(None if predicted_bases is None else predicted_bases[i]),
                )
                candidates[i] = (projected if feasible
                                 else self._reachable_target(i, candidates[i]))
        footholds = planned_footholds(
            feet, candidates, contacts[:, 0], swing_previous, self._touchdowns,
        )
        preview = footholds.copy()
        if self._solver.backend == "jax_mppi":
            future_s = future_touchdown_times(
                contacts, gait_time=self._gait_time(), gait_hz=float(self._config.gait_hz),
            )
            if np.any(future_s):
                future, future_bases = self._predict_touchdowns(
                    q[:3], yaw_rot, base_velocity_world, cmd_world, cmd_body,
                    yaw_rate, future_s, stance_duration_s,
                )
                for i in np.flatnonzero(future_s):
                    predicted_base = future_bases[i, 0]
                    projected, feasible = stance_end_foothold(
                        int(i), future[i], q[:3], rot_m, base_velocity_world,
                        cmd_world, future_s[i], stance_duration_s,
                        self._foot_lower[i], self._foot_upper[i],
                        predicted_base_positions=future_bases[i],
                    )
                    if not feasible:
                        projected, feasible = project_ground_target(
                            int(i), future[i], predicted_base, rot_m,
                            self._foot_lower[i], self._foot_upper[i],
                        )
                    preview[i] = projected if feasible else future[i]
        sample = PyMpcInput(
            com_position=com,
            com_velocity=vel,
            rpy=rpy,
            angular_velocity_body=dq[3:6],
            feet_world=feet,
            desired_com_position=ref_com,
            desired_com_velocity=cmd_world,
            desired_rpy=ref_rpy,
            desired_angular_velocity_body=np.array((0.0, 0.0, yaw_rate)),
            footholds_world=footholds,
            preview_footholds_world=preview,
            contacts=contacts,
            support_loads=self._contact_gait.load.copy(),
            actuation_scales=self._actuation_scale_horizon(),
            support_force_floor_n=(_MIN_SUPPORT_FORCE_N if self._solver.backend == "jax_mppi" else 0.),
            landing_search=self._contact_gait.search.copy(),
            hip_offsets_com_body=(np.array([HIP_OFFSET_BODY[leg] for leg in _LEGS])
                                  + (rot_m.T @ (q[:3] - com))[None, :]),
            mass_kg=mass,
            inertia_body=inertia_body,
            simulation_time_s=self._sim_time,
        )
        return self._walking_reference(sample), feet, foot_vel, jacobians, dq[6:18]

    def _walking_reference(self, sample: PyMpcInput) -> PyMpcInput:
        if self._solver.backend != 'jax_mppi':
            return sample
        sample = trailing_crouch_reference(sample, command_vx=float(self._cmd.vx),
                                           crouch_m=_WALK_TRAILING_CROUCH_M)
        command_world = Rotation.from_euler('z', sample.rpy[2]).apply(
            [self._cmd.vx, self._cmd.vy, 0.])
        return landing_velocity_reference(sample, blocked=self._contact_gait.blocked,
            actual_velocity=self._base_velocity_world, command_world=command_world,
            horizon_s=.5*self._solver.horizon*self._solver.dt)

    def _reachable_target(self, leg: int, target: np.ndarray, *, max_shift=None) -> np.ndarray:
        q = self._bridge.last_q
        projected, feasible = project_ground_target(
            leg, target, q[:3], Rotation.from_quat(q[3:7]).as_matrix(),
            self._foot_lower[leg], self._foot_upper[leg],
        )
        if not feasible:
            return target.copy()  # Contact timeout remains authoritative.
        delta = projected - target
        if max_shift is not None:
            delta *= min(1., max_shift / max(np.linalg.norm(delta), 1e-12))
        return target + delta

    def _normal_contact_forces(self, *, snapshot=None) -> np.ndarray:
        if snapshot is None:
            forces = to_numpy_1d(self._entity.get_links_net_contact_force()).reshape(-1, 3)
        else:
            if snapshot.entity is not self._entity:
                raise ValueError('GO2 contact snapshot belongs to another entity')
            forces = snapshot.contact_force
        indices = getattr(self, '_contact_link_idxs', None)
        if indices is None:
            indices = [self._entity.get_link(f"{leg.value}_foot").idx_local for leg in _LEGS]
            self._contact_link_idxs = indices
        return forces[indices, 2].copy()

    def _torques(self, sample: PyMpcInput, feet: np.ndarray, foot_vel: np.ndarray,
                 jacobians: np.ndarray, joint_vel: np.ndarray) -> np.ndarray:
        contacts = sample.contacts[:, 0]
        auxiliary = self._aux_pd_torque(joint_vel)
        auxiliary += self._stance_damping_reference(sample, jacobians)
        static_gravity = bool(
            self._settling and np.all(contacts == 1.)
            and np.all(self._contact_gait.load >= 1.-1e-9)
            and np.linalg.norm(sample.com_velocity) < .08
            and np.linalg.norm(sample.angular_velocity_body) < .15
            and np.max(np.abs(joint_vel)) < .5
            and self._solver.backend == "jax_mppi"
        )
        swing_previous = self._swing_previous_contacts(contacts)
        support_changed = not np.array_equal(contacts, self._last_contacts)
        fresh_distribution = support_changed or (self._solver.backend == 'jax_mppi' and self._step_i == 0)
        # A force solution is valid only for the support set it was solved for.
        # In particular a newly grounded foot must not reuse its swing zero.
        if not np.any(contacts):
            self._force_requested.fill(0.0)
            self._forces.fill(0.0)
        elif self._step_i % self._solve_stride == 0 or support_changed:
            started = time.perf_counter()
            solver_sample = sample
            if self._solver.backend == 'jax_mppi':
                raw_bias = -.5*joint_vel + (self._joint_gravity if static_gravity else 0.)
                support_dofs = np.repeat(contacts == 1., 3)
                gains = (sample.actuation_scales if sample.actuation_scales is not None else
                         np.full(sample.contacts.shape[1], self._torque_scale()))
                maps, offsets, limits = support_actuator_constraints(jacobians,
                    np.where(support_dofs, raw_bias, 0.), np.where(support_dofs, auxiliary, 0.),
                    self._tau_lim, gains)
                solver_sample = replace(sample, support_torque_maps=maps,
                    support_torque_offsets=offsets, support_torque_limits=limits)
            self._force_requested = self._solver.solve(solver_sample)
            alpha = float(np.clip(self._config.force_filter_alpha, 0.05, 1.0))
            history = self._forces
            if self._solver.backend == 'jax_mppi' and not fresh_distribution:
                # The old force was feasible at an older pose/bias. Keep the
                # configured filter, but blend two currently feasible forces.
                history = project_support_motor_limits(np, history,
                    self._contact_gait.load*gains[0], maps[0], offsets[0], limits[0])
            # The previous distribution balances a different support set.
            # Blending it across liftoff loses its removed foot's force/moment;
            # at touchdown it also mixes the new support with a swing zero.
            # Start the new support phase with its freshly solved distribution.
            filtered = (
                self._force_requested.copy() if fresh_distribution
                else alpha * self._force_requested + (1.0 - alpha) * history
            )
            mu = float(self._config.optimization_friction)
            fz_max = float(self._config.fz_max_n)
            for i in range(4):
                if contacts[i] == 0:
                    filtered[i] = 0.0
                    continue
                fz = float(np.clip(filtered[i, 2], 0.0, fz_max))
                filtered[i, 2] = fz
                filtered[i, :2] = np.clip(filtered[i, :2], -mu * fz, mu * fz)
            self._forces = filtered
            if self._timing_sink is not None:
                self._timing_sink("go2_pympc_solve", time.perf_counter() - started)
        contacts = sample.contacts[:, 0]
        tau = np.zeros(12)
        duty = float(self._config.gait_duty)
        free_swing = (contacts == 0.) & ~self._contact_gait.search
        if self._solver.backend == "jax_mppi":
            self._set_swing_damping(free_swing)
        else:
            free_swing = np.zeros(4, dtype=bool)
        # Direct gravity feedforward regressed dynamic walking. Use inverse
        # statics only during a quiet, fully loaded four-foot stop, where the
        # measured force bias otherwise leaves a residual body tilt.
        for i in range(4):
            sl = slice(i * 3, i * 3 + 3)
            if contacts[i] == 1:
                tau[sl] = -jacobians[i].T @ (
                    self._forces[i] * self._contact_gait.load[i])
                if static_gravity:
                    tau[sl] += self._joint_gravity[sl]
            elif self._contact_gait.search[i]:
                if not self._was_searching[i]:
                    # A late swing must keep trying to reach its ground target.
                    # Resetting the target to the measured airborne foot both
                    # brakes the descent and can leave the entire 4 cm search
                    # above the floor. A newly lost stance has no active swing
                    # target, so retain its measured XY and height instead.
                    finished_swing = (
                        self._last_contacts[i] == 0
                        and self._contact_gait.swing_progress[i] >= 1.0
                    )
                    self._search_targets[i] = self._touchdowns[i] if finished_swing else feet[i]
                    self._search_min_z[i] = self._search_targets[i, 2] - .04
                    if finished_swing:
                        self._search_targets[i, 2] -= self._touchdown_depths[i]
                    print(f"[go2_pympc] touchdown search leg={_LEGS[i].value} "
                          f"sim_time={self._sim_time:.3f} foot_z={feet[i, 2]:.3f}")
                # Search below the retained reference, with bounded force.
                descent = min(
                    0.15 * self._dt,
                    max(0.0, self._search_targets[i, 2] - self._search_min_z[i]),
                )
                self._search_targets[i, 2] -= descent
                desired_vel = np.array((0.0, 0.0, -descent / self._dt))
                previous_target = self._search_targets[i].copy()
                self._search_targets[i] = self._reachable_target(
                    i, previous_target, max_shift=1.5 * self._dt,
                )
                desired_vel += (self._search_targets[i] - previous_target) / self._dt
                task_force = (1000.0 * (self._search_targets[i] - feet[i])
                              + 20.0 * (desired_vel - foot_vel[i]))
                norm = np.linalg.norm(task_force)
                task_force *= min(1.0, 60.0 / max(norm, 1e-12))
                tau[sl] = jacobians[i].T @ task_force
                if self._solver.backend == 'jax_mppi':
                    tau[sl] += self._search_damping_reference(
                        jacobians[i], joint_vel[sl], foot_vel[i],
                        self._swing_velocity_limits[sl])
            else:
                if swing_previous[i] == 1:
                    self._swing_starts[i] = feet[i]
                    self._touchdowns[i] = sample.footholds_world[i]
                    # The forward/reverse trajectory needs earlier loading;
                    # the tested lateral trajectory retains its ground end.
                    # Latch the blend so changing intent mid-swing cannot
                    # move the foot's endpoint vertically before landing.
                    vx, vy = float(self._cmd.vx), float(self._cmd.vy)
                    self._touchdown_depths[i] = (
                        _SWING_TOUCHDOWN_DEPTH_M * vx*vx / max(vx*vx + vy*vy, 1e-12)
                    )
                progress = self._contact_gait.swing_progress[i]
                target, desired_vel = swing_foot_reference(
                    progress, self._swing_starts[i], self._touchdowns[i],
                    duration_s=(1.0 - duty) / float(self._config.gait_hz),
                    phase_rate=float(self._contact_gait._swinging[i]),
                    touchdown_depth_m=self._touchdown_depths[i],
                )
                task_force = 1000.0 * (target - feet[i]) + 20.0 * (desired_vel - foot_vel[i])
                tau[sl] = jacobians[i].T @ task_force
                if free_swing[i]:
                    moved_damping = float(self._config.leg_kv_damping)
                    tau[sl] -= moved_damping * joint_vel[sl]
                    tau[sl] += swing_damping_reference_torque(
                        jacobians[i], joint_vel[sl], foot_vel[i], desired_vel,
                        damping=(.5 + moved_damping + float(self._config.joint_damping)),
                        velocity_limits=self._swing_velocity_limits[sl],
                    )
            tau[sl] -= 0.5 * joint_vel[sl]
        self._was_searching = self._contact_gait.search.copy()
        self._last_contacts = contacts.copy()
        if not np.all(np.isfinite(tau)):
            raise RuntimeError("PyMPC produced nonfinite joint torque")
        self._tau_raw = tau.copy()
        tau = np.clip(tau, -self._tau_lim, self._tau_lim)
        swing_dofs = np.repeat(contacts == 0, 3)
        # Stance damping must not fight the commanded swing velocity. The
        # swing task already supplies its own velocity feedback.
        auxiliary[swing_dofs] = 0.0
        tau = tau * self._torque_scale() + auxiliary
        self._tau_limited = np.clip(tau, -self._tau_lim, self._tau_lim)
        alpha = float(np.clip(self._config.tau_filter_alpha, 0.05, 1.0))
        # All stance legs may redistribute load at a contact change. Do not
        # reintroduce the obsolete support distribution through torque history.
        self._tau_filt = (
            self._tau_limited.copy() if fresh_distribution
            else alpha * self._tau_limited + (1.0 - alpha) * self._tau_filt
        )
        # Do not drag the preceding support torque through a short swing.
        # Preserve the same local force/torque limits in both support modes.
        self._tau_filt[swing_dofs] = self._tau_limited[swing_dofs]
        return self._tau_filt.copy()

    def _swing_previous_contacts(self, contacts: np.ndarray) -> np.ndarray:
        """Replan a new liftoff even when that foot was searching beforehand."""
        previous = self._last_contacts.copy()
        if self._solver.backend == 'jax_mppi':
            restarted = (np.asarray(contacts) == 0.) & ~self._contact_gait.search & self._was_searching
            previous[restarted] = 1.
        return previous

    def _stance_damping_reference(self, sample: PyMpcInput, jacobians: np.ndarray, *,
                                  reference_solver=None) -> np.ndarray:
        """Damp stance errors without braking existing commanded translation.

        Keep the original damping gains and substep damping. Only the bounded
        commanded component receives a matching reference torque; vertical,
        rotational and cross-command motion retain their existing damping.
        Contact confirmation gates the reference. Landing load scales the
        support force, while damping and its reference remain unscaled.
        Final torque limits and filtering still apply.
        """
        reference = np.zeros(12)
        if self._solver.backend != 'jax_mppi' or self._settling:
            return reference
        command_world = Rotation.from_euler('z', sample.rpy[2]).apply(
            [self._cmd.vx, self._cmd.vy, 0.])
        progress = commanded_progress_velocity(self._base_velocity_world, command_world)
        if not np.any(progress):
            return reference
        damping = (.5*self._torque_scale()+float(self._config.joint_damping)
                   +float(self._config.leg_kv_damping)+float(self._config.aux_kv))
        active = np.flatnonzero(sample.contacts[:, 0] == 1.)
        if not len(active):
            return reference
        if reference_solver is not None:
            for i in active:
                sl = slice(3*i, 3*i+3)
                reference[sl] = reference_solver(
                    jacobians[i], np.zeros(3), progress, np.zeros(3), damping=damping,
                    velocity_limits=self._swing_velocity_limits[sl])
            return reference
        matrices = np.asarray(jacobians[active], dtype=float)
        limits = np.asarray(self._swing_velocity_limits.reshape(4, 3)[active], dtype=float)
        if (matrices.shape != (len(active), 3, 3) or limits.shape != (len(active), 3)
                or not np.isfinite(matrices).all() or not np.isfinite(limits).all()
                or not np.isfinite(progress).all() or np.any(limits <= 0.)
                or not np.isfinite(damping) or damping < 0.):
            raise ValueError('stance damping needs finite 3D inputs and positive velocity limits')
        zero = np.zeros((len(active), 3))
        rhs = zero - (progress - np.einsum('nij,nj->ni', matrices, zero))
        # Share preparation, keeping the original per-leg LAPACK solve and
        # singular-value cutoff. A batched SVD changes answers at that cutoff.
        resolved = np.stack([np.linalg.lstsq(j, b, rcond=1e-3)[0]
                             for j, b in zip(matrices, rhs)])
        scale = np.maximum(1., np.max(np.abs(resolved) / limits, axis=1))
        resolved /= scale[:, None]
        reference.reshape(4, 3)[active] = float(damping) * resolved
        return reference

    def _search_damping_reference(self, jacobian, joint_velocity, measured_velocity,
                                  velocity_limits, *,
                                  reference_solver=swing_damping_reference_torque):
        """Damp ground-relative foot motion while the body moves over landing.

        Keep the existing passive damping and 60 N task-force bound. Remove
        only the measured base contribution from joint drag; the foot's
        ground-relative motion is still damped about zero, including descent.
        The task controller alone requests the bounded search descent.
        """
        return reference_solver(jacobian, joint_velocity, measured_velocity, np.zeros(3),
            # This reference is added before the active torque ramp; passive
            # damping remains fully present at every Genesis substep.
            damping=(.5+(float(self._config.joint_damping)
                         +float(self._config.leg_kv_damping))/self._torque_scale()),
            velocity_limits=velocity_limits)

    def _balance_only(self) -> bool:
        """Keep four-foot support until the startup torque ramp can carry a trot."""
        return bool(self._settling or (
            self._solver.backend == 'jax_mppi' and self._torque_scale() < 1.-1e-9))

    def _actuation_scale_horizon(self) -> np.ndarray | None:
        if self._solver.backend != 'jax_mppi':
            return None
        elapsed = self._walk_elapsed()
        return np.array([self._torque_scale(elapsed_s=elapsed+stage*self._solver.dt)
                         for stage in range(self._solver.horizon)])

    def _torque_scale(self, *, elapsed_s: float | None = None) -> float:
        warmup_s = max(0.0, float(self._config.torque_warmup_s))
        ramp_s = max(1e-3, float(self._config.torque_ramp_s))
        elapsed = self._walk_elapsed() if elapsed_s is None else float(elapsed_s)
        if not np.isfinite(elapsed) or elapsed < 0.:
            raise ValueError('torque ramp elapsed time must be finite and nonnegative')
        if elapsed < warmup_s:
            return 0.35
        return float(min(1.0, 0.35 + 0.65 * ((elapsed - warmup_s) / ramp_s)))

    def _aux_pd_torque(self, joint_vel: np.ndarray) -> np.ndarray:
        warmup_s = max(0.0, float(self._config.torque_warmup_s))
        ramp_s = max(1e-3, float(self._config.torque_ramp_s))
        elapsed = self._walk_elapsed() - warmup_s
        tau = -float(self._config.aux_kv) * np.asarray(joint_vel, dtype=float)
        if 0.0 < elapsed < ramp_s:
            q_leg = to_numpy_1d(
                self._entity.get_dofs_position(dofs_idx_local=self._leg_dof_idxs)
            )
            if q_leg.shape != (12,) or not np.all(np.isfinite(q_leg)):
                raise RuntimeError("PyMPC auxiliary PD received invalid leg position")
            kp_scale = float(1.0 - elapsed / ramp_s)
            tau += (
                float(self._config.aux_kp)
                * kp_scale
                * (np.asarray(self._kin.ready_q, dtype=float) - q_leg)
            )
        return tau

    def _record_contact_metrics(self, sample: PyMpcInput) -> None:
        if self._metrics is None or self._contact_diagnostics is None:
            return
        actuation = 1. if sample.actuation_scales is None else float(sample.actuation_scales[0])
        diagnostic = self._contact_diagnostics.sample(
            step_index=self._step_i,
            elapsed_s=(self._dt if self._step_i == 0 else
                       self._contact_diagnostics.cadence_steps * self._dt),
            stance={leg: bool(sample.contacts[i, 0]) for i, leg in enumerate(_LEGS)},
            desired_grf_world={leg: self._forces[i] * self._contact_gait.load[i] * actuation
                               for i, leg in enumerate(_LEGS)},
            physical_mu=float(self._config.physical_friction),
        )
        if diagnostic is not None:
            self._metrics.sample_contact(
                diagnostic, sim_time_s=self._sim_time,
                raw_grf=self._force_requested, tau_raw=self._tau_raw,
                tau_limited=self._tau_limited, tau_applied=self._tau_hold,
            )

    def step(self) -> None:
        self._step()
        if self._metrics is not None:
            torque = self._torque_mode_active and not self._faulted
            self._metrics.record_torque_step(recomputed=torque, hold=False)
            self._metrics.sample_go2(
                go2_entity=self._entity,
                go2_cmd=(self._cmd.vx, self._cmd.vy, self._cmd.yaw_rate),
                command_source=self._command_source, arm_q=self._arm_q,
                tau=self._tau_hold if torque else None,
                torque_update_flag=torque, torque_hold_flag=False,
                wall_time_s=time.time(), sim_time_s=self._sim_time,
                control_rate_info=self._rate_info,
                mpc_base_velocity_body=self._bridge.last_dq[:6] if torque else None,
            )

    def _step(self) -> None:
        self._sim_time += self._dt
        self._cmd = self._command_shaper.update(self._dt)
        gpu = self._solver.backend == 'jax_mppi'
        if gpu:
            stop_ready = self._command_shaper.braking_ready(time_s=self._sim_time,
                threshold=float(self._config.command_idle_threshold))
        else:
            stop_ready = self._command_shaper.stop_ready(
                time_s=self._sim_time,
                threshold=float(self._config.command_idle_threshold),
                gait_hz=float(self._config.gait_hz),
                gait_duty=float(self._config.gait_duty),
            )
        target_idle = self._command_shaper.target.is_idle(
            float(self._config.command_idle_threshold)
        )
        if self._faulted:
            pose, _ = self._pose_transition.update(self._dt)
            self._entity.control_dofs_position(
                pose, dofs_idx_local=self._leg_control_dofs
            )
            return

        if gpu and stop_ready and self._pose_stage == "walk":
            # Freeze new liftoffs now. The contact gate keeps existing swings
            # moving and still requires their real touchdown/load evidence.
            self._settling = True
            stop_ready = (bool(np.all(self._contact_gait.confirmed))
                          and not self._contact_gait.blocked and self._body_ready_to_stand())
        if not gpu and stop_ready and self._pose_stage == "walk":
            stop_ready = bool(np.all(self._contact_gait.confirmed)) and not self._contact_gait.blocked
        if not gpu and stop_ready and self._pose_stage == "walk":
            stop_ready = bool(np.all(contact_schedule(
                self._gait_time(), gait_hz=float(self._config.gait_hz),
                duty=float(self._config.gait_duty), dt=self._dt, horizon=2,
            )[:, 0])) if self._config.gait_duty > 0.5 else stop_ready
        if not target_idle:
            self._settling = False
        if not gpu and stop_ready and self._pose_stage == "walk":
            # Finish the current swing, then balance on four feet while the
            # body brakes. Continuing to trot prevents a quiet stand handoff.
            self._settling = True
            stop_ready = self._body_ready_to_stand()
        if stop_ready and self._pose_stage in {"to_ready", "ready_hold", "walk"}:
            self._begin_stand_pose()
        elif self._pose_stage == "to_stand" and not target_idle:
            self._begin_ready_pose()
        elif self._pose_stage == "stand" and not target_idle:
            self._begin_ready_pose()

        if self._pose_stage == "stand":
            self._entity.control_dofs_position(
                self._kin.stand_q, dofs_idx_local=self._leg_control_dofs
            )
            return

        if self._pose_stage in {"to_ready", "to_stand"}:
            pose, done = self._pose_transition.update(self._dt)
            self._entity.control_dofs_position(pose, dofs_idx_local=self._leg_control_dofs)
            if done and self._pose_stage == "to_ready":
                self._pose_stage = "ready_hold"
                self._ready_until = self._sim_time + float(self._config.ready_pose_s)
            elif done:
                self._pose_stage = "stand"
            return

        if self._pose_stage == "ready_hold":
            self._entity.control_dofs_position(
                self._kin.ready_q, dofs_idx_local=self._leg_control_dofs
            )
            self._contact_gait.prime(self._normal_contact_forces(), dt=self._dt)
            if self._sim_time >= self._ready_until and not target_idle:
                self._pose_stage = "walk"
                self._walk_started_s = self._sim_time
                if self._solver.backend == 'jax_mppi':
                    held = self._command_shaper.target
                    self._command_shaper.reset()
                    self._command_shaper.set_target(held)
                    self._cmd = Go2Command()
                self._was_searching.fill(False)
                self._step_i = 0
                if self._contact_diagnostics is not None:
                    self._contact_diagnostics.reset()
            return

        if self._pose_stage != "walk":
            raise RuntimeError(f"unexpected PyMPC pose stage: {self._pose_stage}")

        if not self._torque_mode_active:
            self._set_torque_actuation()
            self._torque_mode_active = True
        try:
            sample, feet, foot_vel, jacobians, joint_vel = self._sample()
            self._tau_hold = self._torques(sample, feet, foot_vel, jacobians, joint_vel)
        except Exception as exc:
            self._pose_transition.begin(self._current_leg_pose(), self._kin.stand_q)
            self._faulted = True
            self._active = False
            self._pose_stage = "fault"
            self._walk_started_s = None
            self._settling = False
            self._torque_mode_active = False
            self._tau_hold.fill(0.0)
            self._force_requested.fill(0.0)
            self._forces.fill(0.0)
            self._tau_filt.fill(0.0)
            self._tau_limited.fill(0.0)
            self._tau_raw.fill(0.0)
            self._set_stand_actuation()
            pose, _ = self._pose_transition.update(self._dt)
            self._entity.control_dofs_position(pose, dofs_idx_local=self._leg_control_dofs)
            print(f"[go2_pympc] fault; latched safe stand until reset: {exc}")
            return
        self._entity.control_dofs_force(
            self._as_force_tensor(self._tau_hold), dofs_idx_local=self._leg_control_dofs
        )
        self._record_contact_metrics(sample)
        self._step_i += 1
