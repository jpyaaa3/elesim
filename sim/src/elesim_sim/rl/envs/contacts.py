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
    floor_touch: torch.Tensor           # bool: arm hit the ground plane
    support_touch: torch.Tensor         # bool: arm hit the object's support
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
        #: Link pairs already touching at the reset pose, excluded from the
        #: collision verdict.  The stowed Home pose folds the arm back over the
        #: quadruped's head and mounting plate, so those contacts are a property
        #: of where the episode starts, not of anything the policy did -- left
        #: in, every episode terminates on its first step.  Genesis does the
        #: same thing for its own neutral configuration.
        #:
        #: The exclusion is per pair and applies for the whole episode, so a
        #: genuine later crash between the *same two links* is also forgiven.
        #: That is the cost of the simpler rule; pairs that only ever touch at
        #: Home are unaffected.
        self._excluded_pairs: Optional[torch.Tensor] = None
        self._pair_stride = 0
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

    def _encode_pairs(self, link_a: torch.Tensor, link_b: torch.Tensor) -> torch.Tensor:
        lo = torch.minimum(link_a, link_b)
        hi = torch.maximum(link_a, link_b)
        return lo * self._pair_stride + hi

    def set_baseline_from_current_contacts(self) -> int:
        """Record the pairs touching right now as the excluded baseline.

        Call once, with the arm settled at the reset pose.
        """
        raw = self.scene.robot.get_contacts(exclude_self_contact=False)
        c = self._to_device(raw)
        valid = c["valid_mask"]
        if valid.shape[-1] == 0:
            self._excluded_pairs = None
            return 0
        link_a = c["link_a"].long()
        link_b = c["link_b"].long()
        self._pair_stride = int(max(int(link_a.max()), int(link_b.max())) + 1)
        pairs = self._encode_pairs(link_a, link_b)[valid]
        self._excluded_pairs = torch.unique(pairs)
        return int(self._excluded_pairs.numel())

    def _arm_positions(self, link_ids: torch.Tensor) -> torch.Tensor:
        """Map global link indices to arm-local columns; -1 when not an arm link."""
        safe = link_ids.clamp(min=0, max=self._arm_lookup.numel() - 1)
        mapped = self._arm_lookup[safe]
        return torch.where(link_ids >= 0, mapped, torch.full_like(mapped, -1))

    # -- main entry point --------------------------------------------------

    def _empty_snapshot(self, n_envs: int) -> ContactSnapshot:
        """Snapshot for a step with no detected contacts."""
        dev = self.device
        false_ = torch.zeros(n_envs, device=dev, dtype=torch.bool)
        zero = torch.zeros(n_envs, device=dev, dtype=torch.float32)
        return ContactSnapshot(
            object_touch=false_.clone(),
            object_force=zero.clone(),
            object_link_hits=torch.zeros(
                (n_envs, max(self.n_arm_links, 1)), device=dev, dtype=torch.bool
            ),
            floor_touch=false_.clone(),
            support_touch=false_.clone(),
            self_touch=false_.clone(),
            go2_touch=false_.clone(),
            max_penetration=zero.clone(),
            overflow=false_.clone(),
        )

    def classify(self, n_envs: int) -> ContactSnapshot:
        """Query Genesis once and classify every contact row."""
        robot = self.scene.robot
        raw = robot.get_contacts(exclude_self_contact=False)
        c = self._to_device(raw)
        valid = c["valid_mask"]
        # Genesis sizes the contact buffer to the number of pairs it actually
        # detected, which is zero when nothing is touching.  Reductions over a
        # zero-width dim are not merely awkward -- `amax` raises, and
        # `all(dim=-1)` returns True, which would report a buffer overflow on
        # precisely the steps where there are no contacts at all.
        if valid.shape[-1] == 0:
            return self._empty_snapshot(n_envs)
        link_a = c["link_a"].long()
        link_b = c["link_b"].long()
        force = c["force_a"]
        penetration = c["penetration"]

        if self._excluded_pairs is not None and self._excluded_pairs.numel():
            baseline = torch.isin(
                self._encode_pairs(link_a, link_b), self._excluded_pairs
            )
            # Drop baseline rows from *every* verdict, target contact included:
            # a pair resting together at Home is not news either way.
            valid = valid & ~baseline

        obj_idx = self._idx["object"]
        # The support is a non-target body just like the floor, but it is
        # reported separately: they are in completely different places, so
        # merging them leaves the logs unable to say which one the arm keeps
        # running into.
        floor_idx = self._idx["floor"]
        support_idx = self._idx["support"]
        arm_idx = self._idx["arm"]
        go2_idx = self._idx["go2"]

        a_obj, b_obj = _isin(link_a, obj_idx), _isin(link_b, obj_idx)
        a_flo, b_flo = _isin(link_a, floor_idx), _isin(link_b, floor_idx)
        a_sup, b_sup = _isin(link_a, support_idx), _isin(link_b, support_idx)
        a_arm, b_arm = _isin(link_a, arm_idx), _isin(link_b, arm_idx)
        a_go2, b_go2 = _isin(link_a, go2_idx), _isin(link_b, go2_idx)

        # Target contact: an arm link against the object.
        obj_rows = valid & ((a_arm & b_obj) | (a_obj & b_arm))
        # Every non-target class below is deliberately *arm-centric*.  The
        # quadruped stands on the floor and its own adjacent links touch each
        # other; scoring those as policy failures would terminate every
        # episode on the first substep regardless of what the arm did.
        floor_rows = valid & ((a_arm & b_flo) | (a_flo & b_arm))
        support_rows = valid & ((a_arm & b_sup) | (a_sup & b_arm))
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
            support_touch=support_rows.any(dim=-1),
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
    support_touch: torch.Tensor
    self_touch: torch.Tensor
    go2_touch: torch.Tensor
    max_penetration: torch.Tensor
    overflow: torch.Tensor


class ContactAggregator:
    """Accumulates contact state over the substeps of a macro step."""

    def __init__(
        self,
        classifier: ContactClassifier,
        *,
        n_envs: int,
        self_contact_is_failure: bool = False,
    ) -> None:
        self.classifier = classifier
        self.n_envs = int(n_envs)
        self.self_contact_is_failure = bool(self_contact_is_failure)
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
            "support_touch": torch.zeros(n, device=dev, dtype=torch.bool),
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
        acc["support_touch"] |= snap.support_touch
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
        # the floor, the support, or the quadruped body.
        #
        # Arm-against-arm contact is excluded by default.  Closing a continuum
        # coil tightly enough to wrap is exactly what makes the coil touch
        # itself: measured at this arm, wrapping a 45-55 mm cylinder raises
        # self contact in the same macro step that the wrap reaches 174-188
        # deg.  Counting it as a collision terminates the episode at the
        # moment of success.  The dangerous fold -- neighbouring nodes closing
        # on each other -- is already prevented by the 36 deg per-node joint
        # limit, and Genesis drops the pairs that touch at the neutral pose.
        # It stays logged as `contact/self` either way.
        non_target = acc["floor_touch"] | acc["support_touch"] | acc["go2_touch"]
        if self.self_contact_is_failure:
            non_target = non_target | acc["self_touch"]
        return ContactAggregate(
            object_touch=acc["object_touch"].clone(),
            object_force_peak=acc["object_force_peak"].clone(),
            object_link_hits=acc["object_link_hits"].clone(),
            non_target_collision=non_target,
            floor_touch=acc["floor_touch"].clone(),
            support_touch=acc["support_touch"].clone(),
            self_touch=acc["self_touch"].clone(),
            go2_touch=acc["go2_touch"].clone(),
            max_penetration=acc["max_penetration"].clone(),
            overflow=acc["overflow"].clone(),
        )
