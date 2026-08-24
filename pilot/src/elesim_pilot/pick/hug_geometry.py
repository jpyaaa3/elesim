"""EleSim adapter for the vendored target-first ``hug.py`` solver."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from . import hug_reference as reference


class HugGeometryError(ValueError):
    pass


@dataclass(frozen=True)
class HugSettings:
    section_length_m: float = 0.25
    section_length_tolerance_m: float = 0.035
    terminal_ratio: float = 0.155
    boundary_samples: int = 32
    pair_limit: int = 256
    solution_count: int = 3
    time_limit_s: float = 4.0

    def checked(self) -> "HugSettings":
        if not all(
            math.isfinite(value)
            for value in (
                self.section_length_m,
                self.section_length_tolerance_m,
                self.terminal_ratio,
                self.time_limit_s,
            )
        ):
            raise HugGeometryError("hug settings must be finite")
        if self.section_length_m <= 0.0 or self.section_length_tolerance_m < 0.0:
            raise HugGeometryError("section length and tolerance are invalid")
        if not 0.0 <= self.terminal_ratio <= 1.0:
            raise HugGeometryError("terminal ratio must be between zero and one")
        if min(self.boundary_samples, self.pair_limit, self.solution_count) < 1:
            raise HugGeometryError("hug search bounds must be positive")
        if self.time_limit_s <= 0.0:
            raise HugGeometryError("hug time limit must be positive")
        return self


@dataclass(frozen=True)
class HugCandidate:
    mode: str
    turn1: float
    turn2: float
    length: float
    rotation: float
    translation: tuple[float, float]
    contact1: tuple[float, float]
    contact2: tuple[float, float]
    contact1_u: float
    contact2_u: float
    capture_score: float
    section_source: str


def _reference_settings(settings: HugSettings, *, time_limit_s: float) -> reference.Settings:
    return reference.Settings(
        n=settings.terminal_ratio,
        solution_count=settings.solution_count,
        time_limit=time_limit_s,
        wait_for_first=False,
        boundary_levels=(settings.boundary_samples,),
        sdf_resolution=128,
        pair_limits=(settings.pair_limit,),
        contact_grid=5,
        turn_samples=33,
        refinement_levels=1,
    )


def _as_candidate(solution: reference.Solution, source: str) -> HugCandidate:
    return HugCandidate(
        mode=solution.mode,
        turn1=float(solution.gripper.turn1),
        turn2=float(solution.gripper.turn2),
        length=float(solution.length),
        rotation=float(solution.rotation),
        translation=tuple(float(value) for value in solution.translation),
        contact1=tuple(float(value) for value in solution.p1),
        contact2=tuple(float(value) for value in solution.p2),
        contact1_u=float(solution.u1),
        contact2_u=float(solution.u2),
        capture_score=float(solution.score),
        section_source=source,
    )


def _solve(
    vertices: np.ndarray,
    settings: HugSettings,
    *,
    source: str,
    time_limit_s: float,
) -> tuple[HugCandidate, ...]:
    try:
        target = reference.make_target(vertices, sdf_resolution=128)
        solutions = reference.solve_target(
            target,
            _reference_settings(settings, time_limit_s=time_limit_s),
        )
    except reference.DesignError as exc:
        raise HugGeometryError(str(exc)) from exc
    candidates = tuple(
        _as_candidate(solution, source)
        for solution in solutions
        if abs(float(solution.length) - settings.section_length_m)
        <= settings.section_length_tolerance_m
    )
    if not candidates:
        raise HugGeometryError(
            "contact solutions do not fit the fixed 0.25 m arm sections"
        )
    return tuple(
        sorted(
            candidates,
            key=lambda value: (
                value.capture_score,
                -abs(value.length - settings.section_length_m),
            ),
            reverse=True,
        )
    )


def _conservative_radial_section(vertices: np.ndarray, count: int = 32) -> np.ndarray:
    """Return a circular section enclosing every supplied section point."""

    radius = float(np.max(np.linalg.norm(vertices, axis=1)))
    if not math.isfinite(radius) or radius <= 0.0:
        raise HugGeometryError("target cross-section radius is invalid")
    angles = np.linspace(0.0, 2.0 * math.pi, max(16, int(count)), endpoint=False)
    return np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])


def solve_cross_section(
    vertices: Iterable[Iterable[float]],
    settings: HugSettings = HugSettings(),
) -> tuple[HugCandidate, ...]:
    """Solve one XZ section using hug.py contact construction and checks.

    The supplied polygon gets the first bounded budget.  If sharp corners
    produce no solution, the remainder is spent on a conservative circumcircle
    enclosing the complete section.  Both paths run the same contact-pair and
    exact finite-pocket solver; neither falls back to angle heuristics.
    """

    settings = settings.checked()
    raw = np.asarray(tuple(tuple(float(value) for value in point) for point in vertices), dtype=float)
    if raw.ndim != 2 or raw.shape[1:] != (2,) or len(raw) < 3 or not np.isfinite(raw).all():
        raise HugGeometryError("target cross-section must contain finite 2-D vertices")
    centered = raw - np.mean(raw, axis=0)
    exact_budget = max(0.5, settings.time_limit_s * 0.20)
    errors: list[str] = []
    try:
        return _solve(centered, settings, source="exact-xz", time_limit_s=exact_budget)
    except HugGeometryError as exc:
        errors.append(str(exc))
    try:
        return _solve(
            _conservative_radial_section(centered),
            settings,
            source="conservative-circumcircle",
            time_limit_s=max(0.5, settings.time_limit_s - exact_budget),
        )
    except HugGeometryError as exc:
        errors.append(str(exc))
    raise HugGeometryError("; ".join(errors))


__all__ = ["HugCandidate", "HugGeometryError", "HugSettings", "solve_cross_section"]
