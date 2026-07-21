from __future__ import annotations

import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize(
    ("script_name", "source_root", "module"),
    [
        ("run_laptop_stack.sh", "deployments/router/src", "elesim_router.main"),
        ("run_laptop_stack.sh", "deployments/controller/src", "elesim_controller.main"),
        ("run_laptop_stack.sh", "deployments/ui/src", "elesim_ui.main"),
        ("run_sim_worker.sh", "deployments/simulator/src", "elesim_simulator.main"),
        ("run_robot_jetson.sh", "deployments/robot/src", "elesim_robot.main"),
    ],
)
def test_workspace_launcher_runs_the_local_source_package(
    script_name: str,
    source_root: str,
    module: str,
) -> None:
    source = (ROOT / "scripts" / script_name).read_text(encoding="utf-8")

    assert source_root in source
    assert f"-m {module}" in source


@pytest.mark.parametrize(
    "script_name",
    ("run_laptop_stack.sh", "run_sim_worker.sh", "run_robot_jetson.sh"),
)
def test_workspace_launcher_has_valid_bash_syntax(script_name: str) -> None:
    subprocess.run(
        ("bash", "-n", str(ROOT / "scripts" / script_name)),
        check=True,
    )
