from __future__ import annotations

import math

from elesim_pilot.pick.hug_geometry import HugSettings, solve_cross_section


def test_demo_box_section_uses_contact_solver_and_fits_physical_sections() -> None:
    solutions = solve_cross_section(
        ((-0.06, -0.08), (0.06, -0.08), (0.06, 0.08), (-0.06, 0.08))
    )
    best = solutions[0]

    assert best.mode in {"arc2", "stick"}
    assert best.section_source in {"exact-xz", "conservative-circumcircle"}
    assert abs(best.length - 0.25) <= HugSettings().section_length_tolerance_m
    assert best.contact1 != best.contact2
    assert 0.0 <= best.contact1_u <= 1.0
    assert 0.0 <= best.contact2_u <= 1.0
    assert abs(best.turn1) <= math.pi
    assert abs(best.turn2) <= math.pi


def test_solver_result_changes_with_cross_section_geometry() -> None:
    wide = solve_cross_section(
        ((-0.09, -0.05), (0.09, -0.05), (0.09, 0.05), (-0.09, 0.05))
    )[0]
    tall = solve_cross_section(
        ((-0.05, -0.07), (0.05, -0.07), (0.05, 0.07), (-0.05, 0.07))
    )[0]

    assert (wide.contact1, wide.contact2, wide.rotation) != (
        tall.contact1,
        tall.contact2,
        tall.rotation,
    )
