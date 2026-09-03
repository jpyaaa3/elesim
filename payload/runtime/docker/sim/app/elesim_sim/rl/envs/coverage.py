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
    gap_bearing_rad: torch.Tensor   # (n_envs,) bearing the object would leave by
    enclosure_rad: torch.Tensor  # (n_envs,) bearing span the arm surrounds
    plane_alignment: torch.Tensor  # (n_envs,) |bend-plane normal . object axis|
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

    def _contact_span(self, hits: torch.Tensor) -> torch.Tensor:
        """Chain positions between the first and last touching link.

        Requires two contacts: one is a poke, not a wrap, and the run it
        would span is a single link wide.
        """
        n_envs, n_links = hits.shape
        idx = torch.arange(n_links, device=self.device).unsqueeze(0)
        first = torch.where(hits, idx, torch.full_like(idx, n_links)).amin(dim=-1)
        last = torch.where(hits, idx, torch.full_like(idx, -1)).amax(dim=-1)
        inside = (idx >= first.unsqueeze(-1)) & (idx <= last.unsqueeze(-1))
        enough = (hits.sum(dim=-1) >= 2).unsqueeze(-1)
        return inside & enough

    def measure(
        self,
        link_pos: torch.Tensor,
        object_pos: torch.Tensor,
        object_quat: torch.Tensor,
        *,
        radius_m: torch.Tensor,
        height_m: torch.Tensor,
        link_radius_m: float = 0.0,
        contact_mask: Optional[torch.Tensor] = None,
        contact_rule: str = "span",
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
        contact_mask
            ``(n_envs, n_links)`` of links actually touching the object, in the
            same chain order as ``link_pos``.

            Proximity alone is a poor stand-in for wrapping.  A hook lying in a
            vertical plane beside an upright cylinder scores 58 deg of azimuth
            under the proximity rule -- links a little apart in bearing,
            bridged by the chain fill -- while a coil that genuinely encircles
            the object scores 66 deg.  An 8 deg edge does not pay for the six
            macro steps of roll the coil costs, so a policy optimising it
            correctly learns to hook.
        contact_rule
            How ``contact_mask`` restricts occupancy.

            ``"strict"`` keeps only the touching links.  Measured against the
            simulator this is too sharp to train on: a rigid cylinder inside a
            continuum coil touches at 2-3 points because the coil is a spiral,
            not a circle, so a real 254 deg wrap scores 0 whenever those points
            are not chain-neighbours.

            ``"span"`` keeps the near links lying between the first and last
            touching link along the chain, and scores zero below two contacts.
            The arm is anchored on the object at both ends of that run and the
            links in between hug it, which is what wrapping means; a hook
            touching at one point spans nothing.
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
        if contact_mask is not None:
            hits = contact_mask.to(self.device, torch.bool)
            if contact_rule == "strict":
                # Contact already implies proximity; the height and radial
                # tests stay as a guard against a stale mask.
                near = near & hits
            elif contact_rule == "span":
                near = near & self._contact_span(hits)
            else:
                raise ValueError(f"unknown contact_rule: {contact_rule!r}")

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

        # Bearing enclosure: how much of the object's circumference the arm gets
        # *around*, ignoring how far away it is.
        #
        # Distance is left out deliberately.  Every other signal here is a
        # distance, and a distance is minimised by poking the object with the
        # nearest link, which is what a policy plateauing at Phi = 27 deg is
        # doing.  Getting the open coil around the object before closing it has
        # to be worth something on its own, and bearing is the part of that
        # which does not require contact: measured along a scripted wrap the
        # enclosure runs 27 -> 118 deg through the roll and 222 -> 300 as the
        # coil comes round, while an arm that extends past the object without
        # enclosing it peaks at 118 and falls back to 92.
        #
        # Computed from the raw bearings rather than the occupancy grid, since
        # the grid only holds links inside the scoring band.
        in_height = within_height & (radial > 1e-6)
        bearing = torch.where(in_height, angle, torch.full_like(angle, float("nan")))
        srt, _ = torch.sort(torch.nan_to_num(bearing, nan=_TWO_PI * 2.0), dim=-1)
        finite = in_height.sum(dim=-1, keepdim=True)
        # Gaps between neighbours, plus the wrap-around gap, over the links that
        # qualify.  Padding sorts to the end, so a row's live entries are its
        # first `finite` columns.
        idx = torch.arange(srt.shape[1], device=self.device).unsqueeze(0)
        live = idx < finite
        first = srt.gather(1, torch.zeros_like(idx[:, :1]))
        last = srt.gather(1, (finite - 1).clamp_min(0))
        gaps = torch.diff(srt, dim=-1)
        gap_live = live[:, 1:] & (idx[:, 1:] < finite)
        gaps = torch.where(gap_live, gaps, torch.zeros_like(gaps))
        wrap_gap = (first + _TWO_PI) - last
        widest = torch.maximum(
            gaps.amax(dim=-1) if gaps.shape[1] else torch.zeros_like(wrap_gap[:, 0]),
            wrap_gap[:, 0],
        )
        enclosure = (_TWO_PI - widest).clamp(min=0.0)
        enclosure = torch.where(
            finite[:, 0] >= 2, enclosure, torch.zeros_like(enclosure)
        )

        # How closely the arm's bend plane matches the object's cross-section.
        #
        # Bearing enclosure alone cannot tell a coil that surrounds the object
        # from one curled beside it in a vertical plane, because projecting to
        # the horizontal throws away exactly that difference: measured, a
        # vertical curl over the robot's own back scores 98 deg of enclosure
        # without going near the object, more than the 34-80 deg a real
        # approach passes through.  Multiplying by this kills that: the wrap
        # reaches 0.90, the vertical curl 0.07 and an arm reaching past the
        # object 0.17.
        #
        # The normal is the summed cross product of consecutive segment
        # vectors, which is the bend-plane normal for a planar chain.  No
        # eigendecomposition -- besides being cheaper, torch.linalg.eigh is not
        # implemented on MPS.  Links must arrive in chain order, which
        # `_occupancy` already requires.
        if pos.shape[1] >= 3:
            seg = pos[:, 1:, :] - pos[:, :-1, :]
            turn = torch.cross(seg[:, :-1, :], seg[:, 1:, :], dim=-1).sum(dim=1)
            normal = turn / turn.norm(dim=-1, keepdim=True).clamp_min(1e-9)
            alignment = (normal * axis).sum(dim=-1).abs().clamp(0.0, 1.0)
        else:
            alignment = torch.zeros_like(enclosure)

        # Circular mean of the unoccupied bearings: the direction the object is
        # least held in, and so the direction a retention test should pull.  A
        # plain mean of bin indices would put the opening at due east whenever
        # it straddles the wrap-around.
        bins = (
            torch.arange(self.n_bins, device=self.device, dtype=torch.float32) + 0.5
        ) * self.bin_width
        free = (~occupied).to(torch.float32)
        weight = free.sum(dim=-1, keepdim=True).clamp_min(1.0)
        gap_bearing = torch.atan2(
            (free * torch.sin(bins).unsqueeze(0)).sum(dim=-1, keepdim=True) / weight,
            (free * torch.cos(bins).unsqueeze(0)).sum(dim=-1, keepdim=True) / weight,
        ).squeeze(-1)

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
            gap_bearing_rad=gap_bearing,
            enclosure_rad=enclosure,
            plane_alignment=alignment,
            caged=caged,
        )
