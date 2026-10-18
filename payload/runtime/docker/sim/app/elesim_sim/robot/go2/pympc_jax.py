# Adapted SRBD equations and cost weights from Quadruped-PyMPC
# commit 814cd3e8f5733a6652b0df30420b75882a48b0be.
# BSD 3-Clause License
# 
# Copyright (c) 2025, DLS Lab at Istituto Italiano di Tecnologia, Italy
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""GPU sampling MPC for the shared PyMPC state/force boundary.

The SRBD/MPPI formulation follows Quadruped-PyMPC, but this kernel owns its
configuration and takes payload properties as runtime arrays. It does not use
upstream's global configuration, fixed-inertia JIT closure or CPU fallback.
JAX is imported only when this explicitly selected backend is constructed.
"""
from __future__ import annotations

import os
import time

import numpy as np

from .pympc_solver import LEGS, project_support_motor_limits


def _support_extension_cost(array, hip_to_foot_body, support):
    """Softly retain 20 mm reach before the GO2 calf's safe extension limit."""
    # The 95.5 mm hip-to-thigh offset rotates with abduction. Removing its
    # squared length leaves the radius of the two 213 mm sagittal links.
    radius = array.sqrt(array.maximum(array.sum(hip_to_foot_body**2, axis=-1)-.0955**2, 1e-12))
    soft_limit = 2.*.213*np.cos(.5*(-.83776-.04))-.02
    excess = array.maximum((radius-soft_limit)/.02, 0.)
    return 200.*array.sum(array.asarray(support)*excess**2)


def _bounded_integer(name: str, value: int, lower: int, upper: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or not lower <= value <= upper:
        raise ValueError(f"{name} must be an integer in [{lower}, {upper}]")
    return int(value)


class _MppiInputLayout:
    """One owned host packet; slicing/reshaping is compiled into the GPU solve."""

    def __init__(self, horizon: int) -> None:
        # x, reference, feet, contacts, loads, mass, inertia, inverse,
        # use_mean, elapsed, hips, geometry, force_floor, landing_search,
        # motor_maps, motor_offsets, motor_limits. Mean/key stay on device.
        shapes = ((12,), (12,), (horizon, 4, 3), (horizon, 4), (horizon, 4), (),
                  (3, 3), (3, 3), (), (), (4, 3), (), (), (4,),
                  (horizon, 4, 6, 3), (horizon, 4, 6), (horizon, 4, 6))
        offset, parts = 0, []
        for shape in shapes:
            end = offset + int(np.prod(shape))
            parts.append((slice(offset, end), shape))
            offset = end
        self.parts = tuple(parts)

    def pack(self, values) -> np.ndarray:
        if len(values) != len(self.parts):
            raise ValueError('MPPI input packet requires all 17 host fields')
        # A fresh array avoids aliasing caller inputs or an in-flight upload.
        return np.concatenate([np.asarray(value, dtype=np.float32).reshape(shape).reshape(-1)
                               for value, (_, shape) in zip(values, self.parts)])

    def unpack(self, packet):
        return tuple(packet[span].reshape(shape) for span, shape in self.parts)


class JaxMppiSolver:
    """Bounded parallel force rollouts; no training or runtime downloads."""

    def __init__(self, *, horizon: int, dt: float, friction: float,
                 max_normal_force_n: float, samples: int, iterations: int,
                 seed: int) -> None:
        self.horizon = _bounded_integer("horizon", horizon, 2, 64)
        self.samples = _bounded_integer("gpu_samples", samples, 64, 32768)
        self.iterations = _bounded_integer("gpu_iterations", iterations, 1, 8)
        self.seed = _bounded_integer("gpu_seed", seed, 0, 2**31 - 1)
        if not np.isfinite([dt, friction, max_normal_force_n]).all() or min(dt, friction, max_normal_force_n) <= 0:
            raise ValueError("GPU MPC dt, friction and force limit must be finite and positive")
        self.dt = float(dt)
        # Genesis and its render workers share this device. Never reserve the
        # JAX default 75% pool. Respect an explicit operator allocator policy.
        os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
        os.environ.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.15")
        try:
            import jax
            import jax.numpy as jnp
        except ImportError as exc:
            raise RuntimeError("jax_mppi requires the Sim/dev JAX CUDA dependencies; rebuild the image") from exc
        try:
            devices = jax.devices("gpu")
        except Exception as exc:
            raise RuntimeError("jax_mppi requires a working CUDA GPU; CPU fallback is disabled") from exc
        if not devices:
            raise RuntimeError("jax_mppi requires a working CUDA GPU; CPU fallback is disabled")
        # Respect the container visibility/order in both inherit and specific
        # modes. One solve uses the first visible GPU; additional GPUs need
        # not be hidden and are not used for distributed rollout here.
        self.device = devices[0]
        if self.device.platform != "gpu":
            raise RuntimeError("jax_mppi did not select a GPU")
        self._jax, self._jnp = jax, jnp
        self._input_layout = _MppiInputLayout(self.horizon)
        self._optimize = jax.jit(
            self._make_packed_optimizer(friction, max_normal_force_n), device=self.device,
        )
        self.reset()
        began = time.perf_counter()
        # Compile and synchronize before the controller can admit a command.
        feet = np.array([[.2, .12, 0], [.2, -.12, 0], [-.2, .12, 0], [-.2, -.12, 0]])
        state = dict(position=np.array([0, 0, .3]), linear_velocity=np.zeros(3),
                     orientation=np.zeros(3), angular_velocity=np.zeros(3))
        reference = {"ref_" + key: value for key, value in state.items()}
        for i, leg in enumerate(LEGS):
            state[f"foot_{leg}"] = feet[i]
            reference[f"ref_foot_{leg}"] = feet[i:i + 1]
        self.compute_control(state, reference, np.ones((4, horizon)), mass=18.,
                             inertia=np.diag([.2, .5, .5]).reshape(9), elapsed_s=0.)
        self.warmup_s = time.perf_counter() - began
        self.reset()
        print(f"[go2_pympc] GPU={self.device.device_kind} device={self.device.id} "
              f"visible_gpus={len(devices)} samples={samples} "
              f"iterations={iterations} JIT warmup={self.warmup_s:.3f}s")

    def reset(self) -> None:
        self._mean = None
        self._zero_mean = self._jax.device_put(
            np.zeros((self.horizon, 4, 3), dtype=np.float32), self.device)
        self._key = self._jax.device_put(np.array([0, self.seed], dtype=np.uint32), self.device)

    def _make_packed_optimizer(self, friction: float, force_cap: float):
        optimize = self._make_optimizer(friction, force_cap)
        layout, jp = self._input_layout, self._jnp

        def packed_optimize(packet, mean, key):
            (x, reference, feet, contacts, loads, mass, inertia, inverse,
             use_mean, elapsed, hips, geometry, force_floor, landing_search,
             motor_maps, motor_offsets, motor_limits) = layout.unpack(packet)
            return optimize(x, reference, feet, contacts, loads, mass, inertia, inverse,
                            mean, use_mean.astype(jp.bool_), elapsed, key, hips, geometry,
                            force_floor, landing_search, motor_maps, motor_offsets, motor_limits)

        return packed_optimize

    def _make_optimizer(self, friction: float, force_cap: float):
        jax, jp = self._jax, self._jnp
        horizon, dt = self.horizon, self.dt
        samples, iterations = self.samples, self.iterations
        # Three force knots keep exploration correlated across the horizon.
        knot_time = np.linspace(0., 2., horizon, dtype=np.float32)
        left = np.minimum(knot_time.astype(np.int32), 1)
        blend = (knot_time - left).astype(np.float32)[:, None, None]

        def rotation(rpy):
            r, p, y = rpy
            sr, sp, sy = jp.sin(r), jp.sin(p), jp.sin(y)
            cr, cp, cy = jp.cos(r), jp.cos(p), jp.cos(y)
            return jp.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]])

        def angle_error(a, b):
            return jp.arctan2(jp.sin(a-b), jp.cos(a-b))

        def project(force, contacts, loads, force_floor, motor_maps, motor_offsets, motor_limits):
            # The torque controller applies the ramped force. Retain a minimum
            # support request in the rollout itself, including freshly confirmed
            # landings; a post-solve clamp would predict a different wrench.
            minimum = jp.where(loads > 0.,
                               jp.minimum(force_floor/jp.maximum(loads, 1e-6), force_cap), 0.)
            z = jp.clip(force[..., 2], minimum, force_cap)
            xy = jp.clip(force[..., :2], -friction*z[..., None], friction*z[..., None])
            bounded = jp.concatenate((xy, z[..., None]), axis=-1) * contacts[..., None]
            return project_support_motor_limits(jp, bounded, loads,
                                                 motor_maps, motor_offsets, motor_limits)

        def derivative(x, force, feet, mass, inertia, inverse):
            r, p = x[6:8]
            w = x[9:12]
            sr, cr = jp.sin(r), jp.cos(r)
            # Reject excessive tilt in scoring, while keeping rejected rollout
            # arithmetic finite near the Euler singularity.
            cp = jp.maximum(jp.cos(p), .05)
            rate = jp.array([w[0] + jp.sin(p)/cp*(sr*w[1]+cr*w[2]),
                             cr*w[1]-sr*w[2], (sr*w[1]+cr*w[2])/cp])
            torque = jp.sum(jp.cross(feet-x[:3], force), axis=0)
            angular = inverse @ (rotation(x[6:9]).T @ torque - jp.cross(w, inertia @ w))
            linear = jp.sum(force, axis=0)/mass + jp.array([0., 0., -9.81])
            return jp.concatenate((x[3:6], linear, rate, angular))

        def score(x0, reference, feet, contacts, loads, mass, inertia, inverse, hips, geometry,
                  force_floor, landing_search, motor_maps, motor_offsets, motor_limits, forces):
            def step(carry, values):
                x, total = carry
                force, foot, contact, load, maps, offsets, limits = values
                # Every seed, candidate and weighted update was projected
                # before scoring. Score that exact force: reapplying a normal
                # floor here could change a motor-limited force a second time.
                requested = force
                # The torque controller scales an already capped GRF by this
                # landing load. Score the force that actually reaches the
                # Jacobian, for both translation and rotation.
                applied = requested*load[:, None]
                mid = x + .5*dt*derivative(x, applied, foot, mass, inertia, inverse)
                x = x + dt*derivative(mid, applied, foot, mass, inertia, inverse)
                error = x-reference
                error = error.at[6:9].set(angle_error(x[6:9], reference[6:9]))
                # Horizontal tracking must not buy speed by launching or
                # tipping the body out of the legs' support workspace.
                q = jp.array([0., 0., 200000., 2000., 2000., 8000., 8000., 8000., 0., 200., 200., 2000.])
                state_cost = jp.sum(q*error**2)
                nominal_z = mass*9.81/jp.maximum(jp.sum(contact*load), 1.)
                force_error = requested - contact[:, None]*jp.array([0., 0., nominal_z])
                effort = jp.sum(force_error**2*jp.array([.001, .001, .0001]))
                body_feet = (foot-x[:3]) @ rotation(x[6:9])
                # Missing contact cannot supply a GRF, but its ground target
                # must stay reachable while the body balances on other feet.
                reach = jp.maximum(contact*load, landing_search)
                extension = geometry*_support_extension_cost(jp, body_feet-hips, reach)
                motor_torque = jp.einsum('ijk,ik->ij', maps, applied)+offsets
                impossible_motor = jp.any((jp.abs(motor_torque) > limits+1e-4)
                                          & (contact[:, None] > 0.))
                invalid = jp.any(jp.abs(x[6:8]) > 1.2) | ~jp.all(jp.isfinite(x)) | impossible_motor
                cost = jp.where(invalid, jp.inf, state_cost + effort + extension)
                return (x, total + dt*cost), None
            (_, total), _ = jax.lax.scan(
                step, (x0, jp.float32(0.)),
                (forces, feet, contacts, loads, motor_maps, motor_offsets, motor_limits))
            return total

        batch_score = jax.vmap(score, in_axes=(None,)*15 + (0,))

        def initial_forces(x, reference, feet, contacts, loads, mass, inertia, force_floor,
                           motor_maps, motor_offsets, motor_limits):
            # A regularized wrench distribution seeds sampling with a useful
            # support force, including at a changed support set or payload.
            acceleration = (reference[:3]-x[:3])*jp.array([0., 0., 60.]) + (reference[3:6]-x[3:6])*8.
            force = mass*(acceleration + jp.array([0., 0., 9.81]))
            angular = angle_error(reference[6:9], x[6:9])*jp.array([40., 40., 10.]) + (reference[9:12]-x[9:12])*8.
            torque = rotation(x[6:9]) @ (inertia @ angular + jp.cross(x[9:12], inertia @ x[9:12]))
            wrench = jp.concatenate((force, torque))
            def distribute(foot, contact, load, maps, offsets, limits):
                arms = foot-x[:3]
                def cross_matrix(v):
                    a, b, c = v
                    return jp.array([[0., -c, b], [c, 0., -a], [-b, a, 0.]])
                top = jp.tile(jp.eye(3), (1, 4))
                bottom = jax.vmap(cross_matrix)(arms).transpose(1, 0, 2).reshape(3, 12)
                matrix = jp.concatenate((top, bottom), axis=0)*jp.repeat(contact*load, 3)[None, :]
                allocation = matrix.T @ jp.linalg.solve(matrix @ matrix.T + .001*jp.eye(6), jp.eye(6))
                f = allocation @ wrench
                return project(f.reshape(4, 3), contact, load, force_floor,
                               maps, offsets, limits), allocation.reshape(4, 3, 6)
            return jax.vmap(distribute)(feet, contacts, loads, motor_maps, motor_offsets, motor_limits)

        def optimize(x, reference, feet, contacts, loads, mass, inertia, inverse, mean, use_mean, elapsed, key,
                     hips, geometry, force_floor, landing_search, motor_maps, motor_offsets, motor_limits):
            seed, allocation = initial_forces(x, reference, feet, contacts, loads, mass, inertia, force_floor,
                                              motor_maps, motor_offsets, motor_limits)
            index = jp.minimum(jp.arange(horizon, dtype=jp.float32) + elapsed/dt, horizon-1.)
            lo = jp.floor(index).astype(jp.int32)
            hi = jp.minimum(lo+1, horizon-1)
            fraction = (index-lo)[:, None, None]
            shifted = mean[lo]*(1.-fraction) + mean[hi]*fraction
            mean = jp.where(use_mean, .8*shifted + .2*seed, seed)
            mean = project(mean, contacts, loads, force_floor, motor_maps, motor_offsets, motor_limits)
            def improve(_, carry):
                mean, key = carry
                key, noise_key = jax.random.split(key)
                noise = jax.random.normal(noise_key, ((samples+1)//2, 3, 6), dtype=jp.float32)
                # Paired perturbations reduce stochastic force/yaw bias.
                noise = jp.concatenate((noise, -noise), axis=0)[:samples]
                # Explore net body wrench, then distribute it over stance
                # feet. Independent foot noise wastes samples in cancelling
                # forces and introduces large unwanted roll/yaw torques.
                noise = noise*jp.array([24., 24., 40., 3., 3., 3.])
                knot_blend = blend[..., 0]
                wrench_noise = noise[:, left]*(1.-knot_blend) + noise[:, left+1]*knot_blend
                perturbation = jp.einsum("hijq,shq->shij", allocation, wrench_noise)
                candidates = project(mean[None]+perturbation, contacts[None], loads, force_floor,
                                     motor_maps, motor_offsets, motor_limits)
                # Preserve both the incumbent and fresh feedback seed.
                candidates = candidates.at[0].set(mean).at[1].set(seed)
                costs = batch_score(x, reference, feet, contacts, loads, mass, inertia, inverse,
                                    hips, geometry, force_floor, landing_search,
                                    motor_maps, motor_offsets, motor_limits, candidates)
                finite = jp.isfinite(costs)
                best = jp.min(jp.where(finite, costs, jp.inf))
                weights = jp.where(finite, jp.exp(-jp.clip(costs-best, 0., 80.)/1.), 0.)
                weighted = jp.sum(candidates*weights[:, None, None, None], axis=0)/jp.maximum(jp.sum(weights), 1e-20)
                weighted = project(weighted, contacts, loads, force_floor,
                                   motor_maps, motor_offsets, motor_limits)
                # Keep an improving weighted update. Selecting a single lucky
                # noisy rollout injects force/yaw jitter into the physical gait.
                weighted_cost = score(x, reference, feet, contacts, loads, mass, inertia, inverse,
                                      hips, geometry, force_floor, landing_search,
                                      motor_maps, motor_offsets, motor_limits, weighted)
                # The fresh feedback seed is a deterministic candidate too.
                # Do not discard it when averaging noisy trajectories makes
                # a worse solution than that available recovery trajectory.
                seed_better = costs[1] < costs[0]
                incumbent = jp.where(seed_better, seed, mean)
                incumbent_cost = jp.minimum(costs[0], costs[1])
                result = jp.where(weighted_cost <= incumbent_cost, weighted, incumbent)
                return result, key
            mean, key = jax.lax.fori_loop(0, iterations, improve, (mean, key))
            cost = score(x, reference, feet, contacts, loads, mass, inertia, inverse,
                         hips, geometry, force_floor, landing_search,
                         motor_maps, motor_offsets, motor_limits, mean)
            return mean, key, cost
        return optimize

    def compute_control(self, state, reference, contacts, *, mass, inertia, elapsed_s):
        fields = ("position", "linear_velocity", "orientation", "angular_velocity")
        origin = np.asarray(state["position"], dtype=float)
        x = np.concatenate([state[name] for name in fields])
        ref = np.concatenate([reference["ref_"+name] for name in fields])
        # Rebase in float64 before device conversion. Large world coordinates
        # must not quantize foot lever arms in the float32 rollout.
        x[:3] -= origin
        ref[:3] -= origin
        x, ref = x.astype(np.float32), ref.astype(np.float32)
        feet = (np.array([state[f"foot_{leg}"] for leg in LEGS])-origin).astype(np.float32)
        targets_world = np.array([reference[f"ref_foot_{leg}"][0] for leg in LEGS])
        targets = (targets_world-origin).astype(np.float32)
        preview_world = np.array([reference.get(f"preview_foot_{leg}",
                                   reference[f"ref_foot_{leg}"])[0] for leg in LEGS])
        preview = (preview_world-origin).astype(np.float32)
        contact = np.asarray(contacts.T, dtype=np.float32)
        loads = np.asarray(reference.get("support_loads", contact), dtype=np.float32)
        # A foot keeps its measured support position until its first swing;
        # after that, the controller's latched touchdown owns its next stance.
        has_swung = np.maximum.accumulate(1.-contact, axis=0).astype(bool)
        feet_horizon = np.where(has_swung[..., None], preview[None], feet[None])
        inertia = np.asarray(inertia, dtype=np.float32).reshape(3, 3)
        hips = reference.get("hip_offsets_com_body")
        geometry = np.float32(hips is not None)
        hips = np.zeros((4, 3), dtype=np.float32) if hips is None else np.asarray(hips, dtype=np.float32)
        force_floor = np.float32(reference.get("support_force_floor_n", 0.))
        landing_search = np.asarray(reference.get("landing_search", np.zeros(4)), dtype=np.float32)
        motor_names = ('support_torque_maps', 'support_torque_offsets', 'support_torque_limits')
        present = [name in reference for name in motor_names]
        if any(present) and not all(present):
            raise ValueError('support motor constraints require maps, offsets and limits together')
        motor_maps = np.asarray(reference.get(motor_names[0], np.zeros((self.horizon, 4, 6, 3))), dtype=np.float32)
        motor_offsets = np.asarray(reference.get(motor_names[1], np.zeros((self.horizon, 4, 6))), dtype=np.float32)
        motor_limits = np.asarray(reference.get(motor_names[2], np.ones((self.horizon, 4, 6))), dtype=np.float32)
        if (motor_maps.shape != (self.horizon, 4, 6, 3)
                or motor_offsets.shape != (self.horizon, 4, 6)
                or motor_limits.shape != (self.horizon, 4, 6)
                or not all(np.isfinite(value).all() for value in (motor_maps, motor_offsets, motor_limits))
                or np.any(motor_limits <= 0.)):
            raise ValueError('support motor constraints must have finite horizon shapes and positive limits')
        mean = self._mean
        fresh = mean is None or elapsed_s >= self.horizon*self.dt
        if mean is None:
            mean = self._zero_mean
        values = (x, ref, feet_horizon, contact, loads, np.float32(mass), inertia,
                  np.linalg.inv(inertia), np.bool_(not fresh), np.float32(elapsed_s),
                  hips, geometry, force_floor, landing_search, motor_maps, motor_offsets, motor_limits)
        packet = self._jax.device_put(self._input_layout.pack(values), self.device)
        solution, key, cost = self._optimize(packet, mean, self._key)
        # D2H is intentional: Genesis torque conversion is on CPU. Waiting
        # here includes GPU execution/transfer in the caller's solve timing.
        forces = np.asarray(solution)
        cost_value = float(cost)
        if not np.isfinite(forces).all() or not np.isfinite(cost_value):
            raise RuntimeError("GPU MPC produced no finite force trajectory")
        self._mean, self._key = solution, key
        self.last_cost = cost_value
        return forces[0].reshape(12), targets_world, np.zeros(24), 0
