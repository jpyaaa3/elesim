"""Contact observation and classification.

Genesis reports contacts as batched tensors of *global link indices*, so the
only way to tell "the arm touched the object" from "the arm touched the floor"
is to map those indices back onto bodies.  That mapping comes from
:class:`elesim_sim.rl.scene.LinkIndex`.

Two facts drive the design:

* The quadruped and the arm live in one URDF, so arm/trunk contact is a
  **self** contact rather than an inter-entity one.  Non-target collision
  therefore has to cover self contacts, not just floor contacts.
* Contact must be aggregated over the *whole* substep window of a macro step.
  Sampling only the settled pose misses a link that swung through the floor
  and came back out, which is precisely the failure the collision penalty
  exists to catch.  :class:`ContactAggregator` is what accumulates that.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

import torch


def _isin(values: torch.Tensor, allowed: torch.Tensor) -> torch.Tensor:
    """Batched membership test with an empty-set fast path."""
    if allowed.numel() == 0:
        return torch.zeros_like(values, dtype=torch.bool)
    return torch.isin(values, allowed)


@dataclass
class ContactSnapshot:
    """One reading of the contact buffer, already classified.

    All tensors are ``(n_envs,)`` unless noted.  ``*_force`` values are the
    summed contact-force magnitude, which the critic may observe but the actor
    may not: the real arm has no contact sensing.
    """

    object_touch: torch.Tensor          # bool: any arm-object contact
    object_force: torch.Tensor          # float: summed |force| on the object
    object_link_hits: torch.Tensor      # bool (n_envs, n_arm_links)
    floor_touch: torch.Tensor           # bool: arm or trunk hit the floor
    self_touch: torch.Tensor            # bool: robot self-collision
    go2_touch: torch.Tensor             # bool: arm hit the quadruped body
    max_penetration: torch.Tensor       # float
    overflow: torch.Tensor              # bool: buffer was completely full


class ContactClassifier:
    """Turns raw Genesis contact dicts into :class:`ContactSnapshot`."""

    def __init__(self, scene: Any) -> None:
        if scene.links is None:
            raise RuntimeError("scene must be built before classifying contacts")
        self.scene = scene
        self.device = scene.device
        self._idx = scene.links.as_tensors(self.device)
        arm_sorted = sorted(scene.links.arm)
        self._arm_order = {idx: pos for pos, idx in enumerate(arm_sorted)}
        self.n_arm_links = len(arm_sorted)
        self._arm_lookup = torch.full(
            (max(arm_sorted) + 1 if arm_sorted else 1,),
            -1,
            device=self.device,
            dtype=torch.long,
        )
        for idx, pos in self._arm_order.items():
            self._arm_lookup[idx] = pos

    # -- helpers -----------------------------------------------------------

    def _to_device(self, contacts: Mapping[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        return {k: v.to(self.device) for k, v in contacts.items()}

    def _arm_positions(self, link_ids: torch.Tensor) -> torch.Tensor:
        """Map global link indices to arm-local columns; -1 when not an arm link."""
        safe = link_ids.clamp(min=0, max=self._arm_lookup.numel() - 1)
        mapped = self._arm_lookup[safe]
        return torch.where(link_ids >= 0, mapped, torch.full_like(mapped, -1))

    # -- main entry point --------------------------------------------------

    def classify(self, n_envs: int) -> ContactSnapshot:
        """Query Genesis once and classify every contact row."""
        robot = self.scene.robot
        raw = robot.get_contacts(exclude_self_contact=False)
        c = self._to_device(raw)
        valid = c["valid_mask"]
        link_a = c["link_a"].long()
        link_b = c["link_b"].long()
        force = c["force_a"]
        penetration = c["penetration"]

        obj_idx = self._idx["object"]
        floor_idx = self._idx["floor"]
        arm_idx = self._idx["arm"]
        go2_idx = self._idx["go2"]

        a_obj, b_obj = _isin(link_a, obj_idx), _isin(link_b, obj_idx)
        a_flo, b_flo = _isin(link_a, floor_idx), _isin(link_b, floor_idx)
        a_arm, b_arm = _isin(link_a, arm_idx), _isin(link_b, arm_idx)
        a_go2, b_go2 = _isin(link_a, go2_idx), _isin(link_b, go2_idx)

        # Target contact: an arm link against the object.
        obj_rows = valid & ((a_arm & b_obj) | (a_obj & b_arm))
        # Every non-target class below is deliberately *arm-centric*.  The
        # quadruped stands on the floor and its own adjacent links touch each
        # other; scoring those as policy failures would terminate every
        # episode on the first substep regardless of what the arm did.
        floor_rows = valid & ((a_arm & b_flo) | (a_flo & b_arm))
        go2_rows = valid & ((a_arm & b_go2) | (a_go2 & b_arm))
        # Arm-against-arm self collision.  Genesis already drops the pairs that
        # touch in the neutral configuration, so what remains is real.
        self_rows = valid & a_arm & b_arm

        force_mag = force.norm(dim=-1)
        object_force = (force_mag * obj_rows.to(force_mag.dtype)).sum(dim=-1)

        link_hits = torch.zeros(
            (n_envs, max(self.n_arm_links, 1)), device=self.device, dtype=torch.bool
        )
        if obj_rows.any():
            arm_side = torch.where(a_arm, link_a, link_b)
            cols = self._arm_positions(arm_side)
            rows = torch.arange(n_envs, device=self.device).unsqueeze(-1).expand_as(cols)
            keep = obj_rows & (cols >= 0)
            if keep.any():
                link_hits[rows[keep], cols[keep]] = True

        pen = torch.where(valid, penetration, torch.zeros_like(penetration))
        return ContactSnapshot(
            object_touch=obj_rows.any(dim=-1),
            object_force=object_force,
            object_link_hits=link_hits,
            floor_touch=floor_rows.any(dim=-1),
            self_touch=self_rows.any(dim=-1),
            go2_touch=go2_rows.any(dim=-1),
            max_penetration=pen.abs().amax(dim=-1),
            # A completely full buffer means contacts may have been dropped;
            # the caller logs it rather than silently trusting the reading.
            overflow=valid.all(dim=-1),
        )


@dataclass
class ContactAggregate:
    """Contact facts accumulated across one macro step's substeps."""

    object_touch: torch.Tensor
    object_force_peak: torch.Tensor
    object_link_hits: torch.Tensor
    non_target_collision: torch.Tensor
    floor_touch: torch.Tensor
    self_touch: torch.Tensor
    go2_touch: torch.Tensor
    max_penetration: torch.Tensor
    overflow: torch.Tensor


class ContactAggregator:
    """Accumulates contact state over the substeps of a macro step."""

    def __init__(self, classifier: ContactClassifier, *, n_envs: int) -> None:
        self.classifier = classifier
        self.n_envs = int(n_envs)
        self.device = classifier.device
        self._acc: dict[str, torch.Tensor] = {}
        self.reset()

    def reset(self) -> None:
        n = self.n_envs
        dev = self.device
        cols = max(self.classifier.n_arm_links, 1)
        self._acc = {
            "object_touch": torch.zeros(n, device=dev, dtype=torch.bool),
            "object_force_peak": torch.zeros(n, device=dev, dtype=torch.float32),
            "object_link_hits": torch.zeros((n, cols), device=dev, dtype=torch.bool),
            "floor_touch": torch.zeros(n, device=dev, dtype=torch.bool),
            "self_touch": torch.zeros(n, device=dev, dtype=torch.bool),
            "go2_touch": torch.zeros(n, device=dev, dtype=torch.bool),
            "max_penetration": torch.zeros(n, device=dev, dtype=torch.float32),
            "overflow": torch.zeros(n, device=dev, dtype=torch.bool),
        }

    def accumulate(self) -> ContactSnapshot:
        snap = self.classifier.classify(self.n_envs)
        acc = self._acc
        acc["object_touch"] |= snap.object_touch
        acc["object_force_peak"] = torch.maximum(
            acc["object_force_peak"], snap.object_force
        )
        acc["object_link_hits"] |= snap.object_link_hits
        acc["floor_touch"] |= snap.floor_touch
        acc["self_touch"] |= snap.self_touch
        acc["go2_touch"] |= snap.go2_touch
        acc["max_penetration"] = torch.maximum(
            acc["max_penetration"], snap.max_penetration
        )
        acc["overflow"] |= snap.overflow
        return snap

    def result(self) -> ContactAggregate:
        acc = self._acc
        # "Non-target" is arm contact with anything that is not the object:
        # the floor, the quadruped body, or the arm itself.
        non_target = acc["floor_touch"] | acc["go2_touch"] | acc["self_touch"]
        return ContactAggregate(
            object_touch=acc["object_touch"].clone(),
            object_force_peak=acc["object_force_peak"].clone(),
            object_link_hits=acc["object_link_hits"].clone(),
            non_target_collision=non_target,
            floor_touch=acc["floor_touch"].clone(),
            self_touch=acc["self_touch"].clone(),
            go2_touch=acc["go2_touch"].clone(),
            max_penetration=acc["max_penetration"].clone(),
            overflow=acc["overflow"].clone(),
        )
