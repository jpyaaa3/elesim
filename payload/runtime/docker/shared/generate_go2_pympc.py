"""Generate the pinned GO2 nominal acados solver during image construction."""

from pathlib import Path

import quadruped_pympc.config as config
from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import (
    Acados_NMPC_Nominal,
)
import quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal as nominal


config.mpc_params.update(
    horizon=12,
    dt=0.02,
    mu=0.55,
    grf_max=180.0,
    grf_min=0.0,
    use_foothold_optimization=False,
    use_foothold_constraints=False,
)
Acados_NMPC_Nominal()
generated = Path(nominal.__file__).parent / "c_generated_code"
if not (generated / "centroidal_nmpc.json").is_file():
    raise RuntimeError("GO2 PyMPC solver generation produced no JSON")
if not list(generated.glob("*.so")):
    raise RuntimeError("GO2 PyMPC solver generation produced no shared library")
print(f"[go2_pympc] generated acados solver in {generated}")
