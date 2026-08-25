"""Wrap-grasp reinforcement learning stack (Genesis + rsl_rl).

Import-order note (macOS, conda + pip): three copies of ``libomp.dylib`` live
in this environment -- conda's own, pymeshlab's (pulled in by genesis-world)
and the one bundled inside the pip ``torch`` wheel.  Importing ``torch``
first loads the bundled copy, and the ``numpy`` import that follows brings in
conda's, which trips OpenMP's duplicate-runtime guard and aborts the process:

    OMP: Error #15: Initializing libomp.dylib, but found libomp.dylib
    already initialized.

Importing ``numpy`` first loads conda's copy, and ``torch``'s ``@rpath``
dependency then resolves to that already-loaded library, so only one OpenMP
runtime ends up in the process.  Doing it here means every ``elesim_sim.rl``
entry point gets the safe order without each module having to remember, and
without resorting to ``KMP_DUPLICATE_LIB_OK``, which silences the guard
instead of fixing the duplication.
"""

from __future__ import annotations

import numpy as _numpy  # noqa: F401  # must precede any torch import

__all__ = ["configs", "beta_model", "arm_kinematics", "envs"]
