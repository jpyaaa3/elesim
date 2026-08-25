"""Azimuthal wrap coverage around the object.

The wrap angle Phi is the angular extent of the object's circumference that
the arm surrounds.  It is measured about the object's **current** centre and
axis, not the reset pose: an object that has been nudged must not credit the
policy with coverage it no longer has.

Phi is the largest *contiguous* occupied arc, not the total occupied measure.
Two links on opposite sides with a gap between them do not cage anything, and
summing their bins would score that as if they did.

Reference point for the numbers this produces: the geometry sweep in
``misc/analysis/wrap_grasp`` (motion_planning branch) reached a peak of
Phi = 172 deg over the reachable (theta1, theta2) grid and never attained
180 deg.  ``success.coverage_target_rad`` defaults to 172 deg for that reason.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import torch

_TWO_PI = 2.0 * math.pi


def quat_to_axis(quat: torch.Tensor, *, local_axis: int = 2) -> torch.Tensor:
    """Rotate a body axis into world coordinates.

    Genesis quaternions are ``(w, x, y, z)``.  For an upright cylinder the
    interesting axis is local +z, which is why that is the default.
    """
    q = quat.to(torch.float32)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    if local_axis == 2:
        return torch.stack(
            (
                2.0 * (x * z + w * y),
                2.0 * (y * z - w * x),
                1.0 - 2.0 * (x * x + y * y),
            ),
            dim=-1,
        )
    if local_axis == 0:
        return torch.stack(
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y + w * z),
                2.0 * (x * z - w * y),
            ),
            dim=-1,
        )
    return torch.stack(
        (
            2.0 * (x * y - w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + w * x),
        ),
        dim=-1,
    )


def _orthonormal_basis(axis: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Two unit vectors spanning the plane perpendicular to `axis`.

    The helper vector is chosen away from `axis` so the cross product never
    degenerates, which it would if we always used world +z and the object
    happened to lie down -- exactly what the lift test makes it do.
    """
    a = axis / axis.norm(dim=-1, keepdim=True).clamp_min(1e-9)
    helper = torch.zeros_like(a)
    # Pick the world axis the object axis is least aligned with.
    least = a.abs().argmin(dim=-1, keepdim=True)
    helper.scatter_(-1, least, 1.0)
    u = torch.cross(a, helper, dim=-1)
    u = u / u.norm(dim=-1, keepdim=True).clamp_min(1e-9)
    v = torch.cross(a, u, dim=-1)
    v = v / v.norm(dim=-1, keepdim=True).clamp_min(1e-9)
    return u, v


def max_circular_run(occupied: torch.Tensor) -> torch.Tensor:
    """Longest contiguous run of True on a circle, per row.

    Vectorised: the row is doubled so a run that crosses the wrap-around point
    becomes contiguous, run lengths are computed with a cumsum/cummax trick,
    and the result is capped at the bin count so a fully occupied row reports
    exactly one full turn rather than two.
    """
    if occupied.numel() == 0:
        return torch.zeros(occupied.shape[0], device=occupied.device, dtype=torch.long)
    n_bins = occupied.shape[-1]
    doubled = torch.cat((occupied, occupied), dim=-1).to(torch.float32)
    cum = torch.cumsum(doubled, dim=-1)
    # Freeze the cumulative count at every gap, then subtract the most recent
    # frozen value: what remains is the run length ending at each position.
    gaps = torch.where(doubled == 0, cum, torch.zeros_like(cum))
    baseline = torch.cummax(gaps, dim=-1).values
    runs = cum - baseline
    return runs.amax(dim=-1).clamp(max=float(n_bins)).to(torch.long)


@dataclass
class CoverageResult:
    """Wrap geometry for one reading."""

    phi_rad: torch.Tensor        # (n_envs,) largest contiguous covered arc
    occupied_bins: torch.Tensor  # (n_envs, n_bins) bool
    n_near_links: torch.Tensor   # (n_envs,) links inside the scoring band
    min_surface_dist: torch.Tensor  # (n_envs,) closest link-to-surface distance
    gap_rad: torch.Tensor        # (n_envs,) widest uncovered arc
    gap_width_m: torch.Tensor    # (n_envs,) free opening across that gap
    caged: torch.Tensor          # (n_envs,) bool: opening narrower than object


class CoverageMeter:
    """Computes wrap coverage from batched link and object poses."""

    def __init__(
        self,
        *,
        n_bins: int,
        radial_band_m: float,
        device: torch.device,
        max_fill_rad: float = math.pi / 2.0,
    ) -> None:
        if n_bins < 4:
            raise ValueError("coverage.n_bins must be at least 4")
        self.n_bins = int(n_bins)
        self.radial_band_m = float(radial_band_m)
        self.device = device
        self.bin_width = _TWO_PI / self.n_bins
        self.max_fill_rad = float(max_fill_rad)


    def _occupancy(self, angle: torch.Tensor, near: torch.Tensor) -> torch.Tensor:
        """Bins the arm body occupies, filled along the chain.

        Marking only the bin each link centre falls into under-reports badly:
        with 72 bins and ten sparse nodes a genuine half wrap scores a single
        bin.  The arm is one continuous body, so the arc *between* two adjacent
        links is occupied too -- provided both ends are near the object.  Links
        must therefore arrive in chain order (see `LinkIndex.arm_chain`).

        Each gap is filled along the shorter of the two arcs joining its ends.
        A pair separated by more than `max_fill_rad` is skipped: that is the
        arm leaving the object and coming back, not wrapping around it.
        """
        n_envs = angle.shape[0]
        bins = (
            torch.arange(self.n_bins, device=self.device, dtype=torch.float32) + 0.5
        ) * self.bin_width
        occupied = torch.zeros(
            (n_envs, self.n_bins), device=self.device, dtype=torch.bool
        )
        if angle.shape[1] == 0:
            return occupied

        # Each near link marks its own bin.
        own = torch.clamp((angle / self.bin_width).to(torch.long), max=self.n_bins - 1)
        rows = torch.arange(n_envs, device=self.device).unsqueeze(-1).expand_as(own)
        if near.any():
            occupied[rows[near], own[near]] = True

        if angle.shape[1] < 2:
            return occupied

        a = angle[:, :-1]
        b = angle[:, 1:]
        pair_ok = near[:, :-1] & near[:, 1:]
        delta = (b - a + math.pi) % _TWO_PI - math.pi          # signed short arc
        span = delta.abs()
        pair_ok = pair_ok & (span <= self.max_fill_rad)
        if not bool(pair_ok.any()):
            return occupied

        direction = torch.sign(delta)
        direction = torch.where(direction == 0, torch.ones_like(direction), direction)
        # Angular travel from `a` towards `b`, measured in the fill direction.
        travel = ((bins.view(1, 1, -1) - a.unsqueeze(-1)) * direction.unsqueeze(-1)) % _TWO_PI
        inside = travel <= span.unsqueeze(-1)
        inside = inside & pair_ok.unsqueeze(-1)
        return occupied | inside.any(dim=1)

    def measure(
        self,
        link_pos: torch.Tensor,
        object_pos: torch.Tensor,
        object_quat: torch.Tensor,
        *,
        radius_m: torch.Tensor,
        height_m: torch.Tensor,
        link_radius_m: float = 0.0,
    ) -> CoverageResult:
        """Measure coverage.

        Parameters
        ----------
        link_pos
            ``(n_envs, n_links, 3)`` arm link origins in world coordinates.
        object_pos, object_quat
            ``(n_envs, 3)`` and ``(n_envs, 4)`` current object pose.
        radius_m, height_m
            ``(n_envs,)`` object dimensions; per-env because the curriculum
            randomises radius.
        link_radius_m
            Effective link thickness, subtracted from the surface distance so a
            capsule touching the object reports ~0 rather than its own radius.
        """
        pos = link_pos.to(self.device, torch.float32)
        centre = object_pos.to(self.device, torch.float32).unsqueeze(1)
        axis = quat_to_axis(object_quat.to(self.device, torch.float32))
        u, v = _orthonormal_basis(axis)

        rel = pos - centre
        along = (rel * axis.unsqueeze(1)).sum(dim=-1)           # (n, L)
        planar_u = (rel * u.unsqueeze(1)).sum(dim=-1)
        planar_v = (rel * v.unsqueeze(1)).sum(dim=-1)
        radial = torch.sqrt(planar_u**2 + planar_v**2)

        r = radius_m.to(self.device, torch.float32).reshape(-1, 1)
        h = height_m.to(self.device, torch.float32).reshape(-1, 1)
        surface_dist = radial - r - float(link_radius_m)

        # A link counts only if it is beside the object's body (within the
        # height extent) and close enough radially to be caging it.
        within_height = along.abs() <= (h * 0.5 + self.radial_band_m)
        within_band = surface_dist <= self.radial_band_m
        near = within_height & within_band & (radial > 1e-6)

        angle = torch.atan2(planar_v, planar_u) % _TWO_PI
        occupied = self._occupancy(angle, near)

        runs = max_circular_run(occupied)
        # A run of k bins covers somewhere between (k-1) and k bin widths, so
        # reporting k would over-credit coverage by up to one bin.  Round down:
        # the success gate must never be met by a quantisation artefact.
        phi = (runs - 1).clamp_min(0).to(torch.float32) * self.bin_width

        # Caging: a wrap angle says nothing on its own.  The object escapes
        # unless the free opening left by the widest gap is narrower than the
        # object itself, which is the gate the prior geometry sweep applied.
        gap_bins = max_circular_run(~occupied)
        gap = gap_bins.to(torch.float32) * self.bin_width
        wrap_radius = torch.where(
            near, radial, torch.zeros_like(radial)
        ).sum(dim=-1) / near.sum(dim=-1).clamp_min(1).to(radial.dtype)
        chord = 2.0 * wrap_radius * torch.sin((gap * 0.5).clamp(max=math.pi / 2))
        gap_width = (chord - 2.0 * float(link_radius_m)).clamp_min(0.0)
        caged = near.any(dim=-1) & (gap_width < 2.0 * r.squeeze(-1))

        masked = torch.where(near, surface_dist, torch.full_like(surface_dist, 1e6))
        min_dist = masked.amin(dim=-1)
        # No qualifying link: fall back to the raw closest surface distance so
        # approach shaping still has a gradient before the first contact.
        raw_min = surface_dist.amin(dim=-1)
        min_dist = torch.where(near.any(dim=-1), min_dist, raw_min)

        return CoverageResult(
            phi_rad=phi,
            occupied_bins=occupied,
            n_near_links=near.sum(dim=-1),
            min_surface_dist=min_dist,
            gap_rad=gap,
            gap_width_m=gap_width,
            caged=caged,
        )
