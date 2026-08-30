"""Unit tests for the training supervisor.

Both failure modes cost real time.  Not restarting throws away a run that a
resume would have recovered in seconds -- a 2048-env run lost 49 minutes and a
curriculum walked back from 1.0 to 0.6 to one NaN at iteration 207.  Restarting
on a reproducible failure burns a GPU all night repeating it.  The difference
between them is whether the run got past its previous checkpoint, so that is
what these tests pin down.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401
from pathlib import Path  # numpy-before-torch ordering

from elesim_sim.rl.supervise import build_command, latest_checkpoint


def test_the_newest_checkpoint_is_found_numerically_not_alphabetically(tmp_path):
    for n in (50, 100, 200, 1000):
        (tmp_path / f"model_{n}.pt").write_text("x")
    # "model_200.pt" sorts after "model_1000.pt" as a string.
    path, n = latest_checkpoint(tmp_path)
    assert (path.name, n) == ("model_1000.pt", 1000)


def test_an_empty_or_missing_run_directory_reads_as_iteration_zero(tmp_path):
    assert latest_checkpoint(tmp_path) == (None, 0)
    assert latest_checkpoint(tmp_path / "nope") == (None, 0)


def test_unrelated_files_are_ignored(tmp_path):
    (tmp_path / "metadata.json").write_text("{}")
    (tmp_path / "model_50.curriculum.json").write_text("{}")
    (tmp_path / "model_50.pt").write_text("x")
    path, n = latest_checkpoint(tmp_path)
    assert (path.name, n) == ("model_50.pt", 50)


def test_the_command_carries_the_run_settings_through(tmp_path):
    cmd = build_command(
        ["--stamp", "srv", "--set", "runtime.n_envs=2048"],
        resume=None,
        iterations=4000,
    )
    assert cmd[1:3] == ["-m", "elesim_sim.rl.train"]
    assert "--stamp" in cmd and "srv" in cmd
    assert "runtime.n_envs=2048" in cmd
    assert cmd[cmd.index("--iterations") + 1] == "4000"
    assert "--resume" not in cmd


def test_a_restart_replaces_the_iteration_count_rather_than_adding_one(tmp_path):
    """`train --iterations` is how many *more* to run, so a restart has to pass
    the remainder -- and passing it twice would leave argparse taking the first.
    """
    ckpt = tmp_path / "model_200.pt"
    cmd = build_command(
        ["--stamp", "srv", "--iterations", "4000", "--resume", "old.pt"],
        resume=ckpt,
        iterations=3800,
    )
    assert cmd.count("--iterations") == 1
    assert cmd[cmd.index("--iterations") + 1] == "3800"
    assert cmd.count("--resume") == 1
    assert cmd[cmd.index("--resume") + 1] == str(ckpt)
    assert "old.pt" not in cmd


def test_equals_form_flags_are_replaced_too():
    cmd = build_command(
        ["--stamp", "srv", "--iterations=4000", "--resume=old.pt"],
        resume=None,
        iterations=100,
    )
    assert cmd.count("--iterations") == 1
    assert not any(a.startswith("--resume") for a in cmd)
    assert "old.pt" not in cmd


def test_a_seed_checkpoint_is_used_only_until_the_run_has_its_own(tmp_path):
    """`--resume` starts a fresh run directory from another run's checkpoint.

    Once this run writes one, restarts must pick that up instead -- otherwise a
    blow-up would throw away everything since the seed.
    """
    from elesim_sim.rl.supervise import _iteration_of, latest_checkpoint

    seed = tmp_path / "old" / "model_100.pt"
    seed.parent.mkdir()
    seed.write_text("x")
    run = tmp_path / "new"
    run.mkdir()
    assert latest_checkpoint(run) == (None, 0)
    assert _iteration_of(seed) == 100

    (run / "model_150.pt").write_text("x")
    path, n = latest_checkpoint(run)
    assert (path.name, n) == ("model_150.pt", 150)


def test_the_seeds_own_iteration_counts_against_the_total():
    """rsl_rl carries on numbering from the checkpoint it loads."""
    from elesim_sim.rl.supervise import _iteration_of

    assert _iteration_of(Path("model_100.pt")) == 100
    assert _iteration_of(Path("whatever.pt")) == 0
