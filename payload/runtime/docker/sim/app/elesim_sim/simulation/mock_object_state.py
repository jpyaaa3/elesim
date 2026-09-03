"""Sim-local lifecycle for a catalog-backed mock object.

This module owns only the planning/mock attachment state.  It does not spawn
Genesis entities and it does not publish DDS messages.  The returned snapshot
uses the field names and bounded values of ``MockObjectStatePayload`` so the
runtime can hand it to the protocol layer without exposing this state object.
"""

from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Any, Sequence

from .mock_objects import (
    MockObjectArtifact,
    MockObjectCatalog,
    MockObjectError,
    project_artifact_xz,
)


class MockObjectStateError(ValueError):
    """An invalid mock object lifecycle transition or target was requested."""


class MockObjectState:
    """Bounded state machine for one Sim-local mock artifact.

    ``revision`` is a process-local lifecycle generation.  Spawn and detach
    advance it, so an operator detach cancels every in-flight solution without
    changing the immutable artifact SHA-256.
    """

    MAX_AVAILABLE_ASSETS = 16
    MAX_SILHOUETTE_POINTS = 64
    DEFAULT_Q_TOLERANCE = 1e-3
    DEFAULT_SETTLE_SAMPLES = 3
    MAX_POSITION_M = 10.0
    MAX_EULER_DEG = 360.0

    def __init__(
        self,
        catalog: MockObjectCatalog,
        *,
        q_tolerance: float = DEFAULT_Q_TOLERANCE,
        settle_samples: int = DEFAULT_SETTLE_SAMPLES,
    ) -> None:
        if not isinstance(catalog, MockObjectCatalog):
            raise TypeError("catalog must be MockObjectCatalog")
        if not math.isfinite(float(q_tolerance)) or q_tolerance <= 0.0:
            raise ValueError("q_tolerance must be finite and positive")
        if type(settle_samples) is not int or settle_samples <= 0:
            raise ValueError("settle_samples must be a positive integer")
        assets = catalog.assets()
        if len(assets) > self.MAX_AVAILABLE_ASSETS:
            raise MockObjectStateError(
                f"catalog exposes more than {self.MAX_AVAILABLE_ASSETS} mock assets"
            )
        self.catalog = catalog
        self._lock = threading.RLock()
        self.q_tolerance = float(q_tolerance)
        self.settle_samples = settle_samples
        self._available_assets = assets
        self._revision = 0
        self._artifact: MockObjectArtifact | None = None
        self._silhouette_xz: tuple[tuple[float, float], ...] = ()
        self._position = (0.0, 0.0, 0.0)
        self._euler_deg = (0.0, 0.0, 0.0)
        self._state = "empty"
        self._solution_id = ""
        self._final_q: tuple[float, ...] | None = None
        self._settle_count = 0
        self._attached = False
        self._reason = ""

    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @staticmethod
    def _finite_vector(value: Sequence[object], size: int, *, name: str) -> tuple[float, ...]:
        if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != size:
            raise MockObjectStateError(f"{name} must contain exactly {size} values")
        try:
            result = tuple(float(item) for item in value)
        except (TypeError, ValueError) as exc:
            raise MockObjectStateError(f"{name} must contain finite numbers") from exc
        if not all(math.isfinite(item) for item in result):
            raise MockObjectStateError(f"{name} must contain finite numbers")
        return result

    @staticmethod
    def _validate_solution_id(value: object) -> str:
        result = str(value or "").strip()
        if not result or len(result) > 128 or any(char.isspace() for char in result):
            raise MockObjectStateError("solution_id must be a bounded non-whitespace string")
        return result

    @staticmethod
    def _sha256(value: object) -> str:
        result = str(value or "").strip().lower()
        if len(result) != 64 or any(char not in "0123456789abcdef" for char in result):
            raise MockObjectStateError("object sha256 must be a 64-character hexadecimal digest")
        return result

    def _empty_transition(self, reason: str) -> None:
        self._artifact = None
        self._silhouette_xz = ()
        self._state = "empty"
        self._solution_id = ""
        self._final_q = None
        self._settle_count = 0
        self._attached = False
        self._reason = reason

    def spawn(
        self,
        asset_id: str | Path,
        position: Sequence[object],
        euler_deg: Sequence[object],
    ) -> dict[str, Any]:
        """Load and make one catalog artifact current in the next generation."""

        with self._lock:
            position_values = self._finite_vector(position, 3, name="position")
            euler_values = self._finite_vector(euler_deg, 3, name="euler_deg")
            if any(abs(value) > self.MAX_POSITION_M for value in position_values):
                raise MockObjectStateError(f"position must remain within ±{self.MAX_POSITION_M:g} m")
            if any(abs(value) > self.MAX_EULER_DEG for value in euler_values):
                raise MockObjectStateError(f"euler_deg must remain within ±{self.MAX_EULER_DEG:g}")
            try:
                artifact = self.catalog.load(asset_id)
                silhouette_xz = project_artifact_xz(artifact, euler_values)
            except (MockObjectError, OSError) as exc:
                raise MockObjectStateError(str(exc)) from exc
            if len(silhouette_xz) > self.MAX_SILHOUETTE_POINTS:
                raise MockObjectStateError("artifact silhouette exceeds the state bound")
            self._revision += 1
            self._artifact = artifact
            self._silhouette_xz = silhouette_xz
            self._position = (position_values[0], position_values[1], position_values[2])
            self._euler_deg = (euler_values[0], euler_values[1], euler_values[2])
            self._state = "spawned"
            self._solution_id = ""
            self._final_q = None
            self._settle_count = 0
            self._attached = False
            self._reason = ""
            return self.snapshot()

    def remove(self) -> dict[str, Any]:
        """Remove the active object while retaining the latest generation."""

        with self._lock:
            self._empty_transition("removed")
            return self.snapshot()

    def begin_execution(
        self,
        solution_id: str,
        revision: int,
        sha256: str,
        final_q: Sequence[object],
    ) -> dict[str, Any]:
        """Fence and start one solution against the exact current artifact."""

        with self._lock:
            return self._accept_execution(solution_id, revision, sha256, final_q, idempotent=False)

    def accept_execution(
        self,
        solution_id: str,
        revision: int,
        sha256: str,
        final_q: Sequence[object],
    ) -> dict[str, Any]:
        """Accept every fenced waypoint, idempotently, before applying its q."""

        with self._lock:
            return self._accept_execution(solution_id, revision, sha256, final_q, idempotent=True)

    def _accept_execution(
        self,
        solution_id: str,
        revision: int,
        sha256: str,
        final_q: Sequence[object],
        *,
        idempotent: bool,
    ) -> dict[str, Any]:
        if self._artifact is None:
            raise MockObjectStateError("cannot execute without an active mock object")
        expected_revision = self._revision_value(revision)
        expected_sha = self._sha256(sha256)
        target = self._finite_vector(final_q, 4, name="final_q")
        identity = self._validate_solution_id(solution_id)
        if self._state == "executing":
            if idempotent and (
                identity == self._solution_id
                and expected_revision == self._revision
                and expected_sha == self._artifact.sha256
                and target == self._final_q
            ):
                return self.snapshot()
            raise MockObjectStateError("a mock execution is already in progress")
        if self._attached:
            raise MockObjectStateError("attached mock object must be detached before execution")
        if expected_revision != self._revision:
            raise MockObjectStateError("stale mock object revision")
        if expected_sha != self._artifact.sha256:
            raise MockObjectStateError("mock object sha256 does not match the active artifact")
        self._solution_id = identity
        self._final_q = target
        self._settle_count = 0
        self._state = "executing"
        self._attached = False
        self._reason = ""
        return self.snapshot()

    def observe_q(self, q: Sequence[object]) -> dict[str, Any]:
        """Observe a joint state and attach after consecutive settled samples."""

        with self._lock:
            observed = self._finite_vector(q, 4, name="q")
            if self._artifact is None:
                raise MockObjectStateError("cannot observe q without an active mock object")
            if self._state != "executing" or self._final_q is None:
                raise MockObjectStateError("no mock execution is in progress")
            if all(abs(left - right) <= self.q_tolerance for left, right in zip(observed, self._final_q)):
                self._settle_count += 1
            else:
                self._settle_count = 0
            if self._settle_count >= self.settle_samples:
                self._state = "attached"
                self._attached = True
                self._reason = "mock posture settled"
            return self.snapshot()

    def detach(self) -> dict[str, Any]:
        """Release the logical attachment without removing the object."""

        with self._lock:
            if self._artifact is None:
                raise MockObjectStateError("cannot detach without an active mock object")
            self._revision += 1
            self._state = "spawned"
            self._solution_id = ""
            self._final_q = None
            self._settle_count = 0
            self._attached = False
            self._reason = "detached"
            return self.snapshot()

    def fail(self, reason: object) -> dict[str, Any]:
        """Surface an application failure without discarding artifact identity."""

        with self._lock:
            if self._artifact is None:
                raise MockObjectStateError("cannot fail without an active mock object")
            message = str(reason or "mock object operation failed").strip()
            self._state = "error"
            self._solution_id = ""
            self._final_q = None
            self._settle_count = 0
            self._attached = False
            self._reason = message[:512]
            return self.snapshot()

    def reset(self) -> dict[str, Any]:
        """Clear the active object without reusing its spawn generation."""

        with self._lock:
            self._empty_transition("reset")
            return self.snapshot()

    def snapshot(self) -> dict[str, Any]:
        """Return a bounded mapping consumable by ``MockObjectStatePayload``."""

        with self._lock:
            artifact = self._artifact
            return {
                "available_assets": tuple(self._available_assets),
                "state": self._state,
                "asset_id": "" if artifact is None else artifact.asset_id,
                "revision": self._revision,
                "sha256": "" if artifact is None else artifact.sha256,
                "position": self._position,
                "euler_deg": self._euler_deg,
                "silhouette_xz": () if artifact is None else self._silhouette_xz,
                "solution_id": self._solution_id,
                "attached": self._attached,
                "reason": self._reason,
            }

    @staticmethod
    def _revision_value(value: object) -> int:
        if type(value) is not int or value < 1:
            raise MockObjectStateError("object revision must be a positive integer")
        return value


__all__ = ["MockObjectState", "MockObjectStateError"]
