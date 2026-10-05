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

from .pympc_solver import LEGS


def _bounded_integer(name: str, value: int, lower: int, upper: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or not lower <= value <= upper:
        raise ValueError(f"{name} must be an integer in [{lower}, {upper}]")
    return int(value)


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
        if len(devices) != 1:
            raise RuntimeError("jax_mppi requires exactly one visible GPU; select one in EleSim GPU settings")
        self.device = devices[0]
        if self.device.platform != "gpu":
            raise RuntimeError("jax_mppi did not select a GPU")
        self._jax, self._jnp = jax, jnp
        self._optimize = jax.jit(
            self._make_optimizer(friction, max_normal_force_n), device=self.device,
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
        print(f"[go2_pympc] GPU={self.device.device_kind} samples={samples} "
              f"iterations={iterations} JIT warmup={self.warmup_s:.3f}s")

    def reset(self) -> None:
        self._mean = None
        self._key = self._jax.device_put(np.array([0, self.seed], dtype=np.uint32), self.device)

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

        def project(force, contacts):
            z = jp.clip(force[..., 2], 0., force_cap)
            xy = jp.clip(force[..., :2], -friction*z[..., None], friction*z[..., None])
            return jp.concatenate((xy, z[..., None]), axis=-1) * contacts[..., None]

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

        def score(x0, reference, feet, contacts, mass, inertia, inverse, forces):
            def step(carry, values):
                x, total = carry
                force, foot, contact = values
                force = project(force, contact)
                mid = x + .5*dt*derivative(x, force, foot, mass, inertia, inverse)
                x = x + dt*derivative(mid, force, foot, mass, inertia, inverse)
                error = x-reference
                error = error.at[6:9].set(angle_error(x[6:9], reference[6:9]))
                q = jp.array([0., 0., 1500., 200., 200., 200., 500., 500., 0., 20., 20., 50.])
                state_cost = jp.sum(q*error**2)
                nominal_z = mass*9.81/jp.maximum(jp.sum(contact), 1.)
                force_error = force - contact[:, None]*jp.array([0., 0., nominal_z])
                effort = jp.sum(force_error**2*jp.array([.001, .001, .0001]))
                invalid = jp.any(jp.abs(x[6:8]) > 1.2) | ~jp.all(jp.isfinite(x))
                cost = jp.where(invalid, jp.inf, state_cost + effort)
                return (x, total + dt*cost), None
            (_, total), _ = jax.lax.scan(step, (x0, jp.float32(0.)), (forces, feet, contacts))
            return total

        batch_score = jax.vmap(score, in_axes=(None, None, None, None, None, None, None, 0))

        def initial_forces(x, reference, feet, contacts, mass, inertia):
            # A regularized wrench distribution seeds sampling with a useful
            # support force, including at a changed support set or payload.
            acceleration = (reference[:3]-x[:3])*jp.array([0., 0., 60.]) + (reference[3:6]-x[3:6])*8.
            force = mass*(acceleration + jp.array([0., 0., 9.81]))
            angular = angle_error(reference[6:9], x[6:9])*jp.array([40., 40., 10.]) + (reference[9:12]-x[9:12])*8.
            torque = rotation(x[6:9]) @ (inertia @ angular + jp.cross(x[9:12], inertia @ x[9:12]))
            wrench = jp.concatenate((force, torque))
            def distribute(foot, contact):
                arms = foot-x[:3]
                def cross_matrix(v):
                    a, b, c = v
                    return jp.array([[0., -c, b], [c, 0., -a], [-b, a, 0.]])
                top = jp.tile(jp.eye(3), (1, 4))
                bottom = jax.vmap(cross_matrix)(arms).transpose(1, 0, 2).reshape(3, 12)
                matrix = jp.concatenate((top, bottom), axis=0)*jp.repeat(contact, 3)[None, :]
                f = matrix.T @ jp.linalg.solve(matrix @ matrix.T + .001*jp.eye(6), wrench)
                return project(f.reshape(4, 3), contact)
            return jax.vmap(distribute)(feet, contacts)

        def optimize(x, reference, feet, contacts, mass, inertia, inverse, mean, use_mean, elapsed, key):
            seed = initial_forces(x, reference, feet, contacts, mass, inertia)
            index = jp.minimum(jp.arange(horizon, dtype=jp.float32) + elapsed/dt, horizon-1.)
            lo = jp.floor(index).astype(jp.int32)
            hi = jp.minimum(lo+1, horizon-1)
            fraction = (index-lo)[:, None, None]
            shifted = mean[lo]*(1.-fraction) + mean[hi]*fraction
            mean = jp.where(use_mean, .8*shifted + .2*seed, seed)
            mean = project(mean, contacts)
            def improve(_, carry):
                mean, key = carry
                key, noise_key = jax.random.split(key)
                noise = jax.random.normal(noise_key, (samples, 3, 4, 3), dtype=jp.float32)
                noise = noise*jp.array([12., 12., 24.])
                perturbation = noise[:, left]*(1.-blend) + noise[:, left+1]*blend
                candidates = project(mean[None]+perturbation, contacts[None])
                # Preserve both the incumbent and fresh feedback seed.
                candidates = candidates.at[0].set(mean).at[1].set(seed)
                costs = batch_score(x, reference, feet, contacts, mass, inertia, inverse, candidates)
                finite = jp.isfinite(costs)
                best = jp.min(jp.where(finite, costs, jp.inf))
                weights = jp.where(finite, jp.exp(-jp.clip(costs-best, 0., 80.)/1.), 0.)
                weighted = jp.sum(candidates*weights[:, None, None, None], axis=0)/jp.maximum(jp.sum(weights), 1e-20)
                # Averaging can worsen a nonlinear rollout; retain the better
                # candidate after explicitly scoring the weighted trajectory.
                weighted_cost = score(x, reference, feet, contacts, mass, inertia, inverse, weighted)
                winner = candidates[jp.argmin(jp.where(finite, costs, jp.inf))]
                result = jp.where(weighted_cost <= best, weighted, winner)
                return result, key
            mean, key = jax.lax.fori_loop(0, iterations, improve, (mean, key))
            cost = score(x, reference, feet, contacts, mass, inertia, inverse, mean)
            return mean, key, cost
        return optimize

    def compute_control(self, state, reference, contacts, *, mass, inertia, elapsed_s):
        fields = ("position", "linear_velocity", "orientation", "angular_velocity")
        x = np.concatenate([state[name] for name in fields]).astype(np.float32)
        ref = np.concatenate([reference["ref_"+name] for name in fields]).astype(np.float32)
        feet = np.array([state[f"foot_{leg}"] for leg in LEGS], dtype=np.float32)
        targets = np.array([reference[f"ref_foot_{leg}"][0] for leg in LEGS], dtype=np.float32)
        contact = np.asarray(contacts.T, dtype=np.float32)
        # A foot keeps its measured support position until its first swing;
        # after that, the controller's latched touchdown owns its next stance.
        has_swung = np.maximum.accumulate(1.-contact, axis=0).astype(bool)
        feet_horizon = np.where(has_swung[..., None], targets[None], feet[None])
        inertia = np.asarray(inertia, dtype=np.float32).reshape(3, 3)
        mean = self._mean
        fresh = mean is None or elapsed_s >= self.horizon*self.dt
        if mean is None:
            mean = np.zeros((self.horizon, 4, 3), dtype=np.float32)
        values = (x, ref, feet_horizon, contact, np.float32(mass), inertia,
                  np.linalg.inv(inertia), mean, np.bool_(not fresh), np.float32(elapsed_s), self._key)
        args = [self._jax.device_put(value, self.device) for value in values]
        solution, key, cost = self._optimize(*args)
        # D2H is intentional: Genesis torque conversion is on CPU. Waiting
        # here includes GPU execution/transfer in the caller's solve timing.
        forces = np.asarray(solution)
        cost_value = float(cost)
        if not np.isfinite(forces).all() or not np.isfinite(cost_value):
            raise RuntimeError("GPU MPC produced no finite force trajectory")
        self._mean, self._key = solution, key
        self.last_cost = cost_value
        return forces[0].reshape(12), targets, np.zeros(24), 0
