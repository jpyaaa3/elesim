"""T1 step-rate benchmark (gate G1).

Measures wrap-scene throughput against the parallel-environment count, with
contact enabled, and writes a markdown table.

Genesis initialises once per process and a scene is built once, so each ``N``
in the sweep runs in a **child process**: that is the only way to get an
independent build per point, and it also means a point that exhausts memory
fails on its own instead of taking the whole sweep down.  A failed point is
recorded as failed -- the table never silently omits an ``N`` that did not run.

Run one point::

    python -m elesim_sim.rl.benchmark --n-envs 256

Run the configured sweep and write the report::

    python -m elesim_sim.rl.benchmark
"""

from __future__ import annotations

import argparse
import json
import platform
import resource
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import torch

from .configs.loader import load_config, WrapGraspConfig

_REPO_ROOT = Path(__file__).resolve().parents[4]


@dataclass
class PointResult:
    n_envs: int
    ok: bool
    build_s: Optional[float] = None
    total_steps_per_s: Optional[float] = None
    per_env_steps_per_s: Optional[float] = None
    peak_rss_bytes: Optional[int] = None
    device_alloc_bytes: Optional[int] = None
    contact_buffer: Optional[int] = None
    live_contacts: Optional[int] = None
    error: Optional[str] = None


def _peak_rss_bytes() -> int:
    """Peak resident set size, normalised to bytes.

    ``ru_maxrss`` is bytes on macOS and kilobytes on Linux; getting this wrong
    would misreport memory by 1024x.
    """
    raw = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    return raw if platform.system() == "Darwin" else raw * 1024


def _device_alloc_bytes(device_kind: str) -> Optional[int]:
    import torch

    if device_kind == "cuda" and torch.cuda.is_available():
        return int(torch.cuda.max_memory_allocated())
    if device_kind == "mps" and getattr(torch, "mps", None) is not None:
        getter = getattr(torch.mps, "current_allocated_memory", None)
        if callable(getter):
            return int(getter())
    return None


def environment_report() -> dict[str, Any]:
    """Host and toolchain facts the benchmark table must carry.

    ``torch.version.cuda`` is the runtime CUDA the wheel was built against --
    not whatever ``nvcc`` happens to be on PATH, which can differ.
    """
    import torch

    report: dict[str, Any] = {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor() or platform.machine(),
        "python": platform.python_version(),
        "torch": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "cuda_available": bool(torch.cuda.is_available()),
        "mps_available": bool(
            getattr(torch.backends, "mps", None) is not None
            and torch.backends.mps.is_available()
        ),
    }
    try:
        import genesis as gs

        report["genesis"] = getattr(gs, "__version__", "unknown")
    except Exception:  # pragma: no cover - genesis import is heavy
        report["genesis"] = "unavailable"

    if shutil.which("nvidia-smi"):
        try:
            out = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=name,memory.total,driver_version",
                    "--format=csv,noheader",
                ],
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
            report["nvidia_smi"] = out.stdout.strip() or out.stderr.strip()
        except Exception as exc:
            report["nvidia_smi"] = f"query failed: {exc}"
    else:
        report["nvidia_smi"] = "not present"
    return report


def _drive_into_wrap(cfg: WrapGraspConfig, scene, steps: int = 80) -> None:
    """Ramp the arm into a wrapping pose so contacts are actually live.

    Timing the scene at its neutral pose measures a scene where contact is
    *enabled* but never *happens* -- the contact buffer stays empty and the
    constraint count is at its floor.  A wrap-grasp step rate that never
    touched the object is not the step rate of wrap grasping, so the arm is
    driven to the best pose the workspace probe found before the clock starts.
    """
    from .arm_kinematics import ArmWaypointMapper

    rate = cfg.macro_step.rate_limit
    mapper = ArmWaypointMapper(
        cfg.arm,
        n_envs=scene.n_envs,
        device=scene.device,
        rate_limit=(rate.linear_m, rate.roll_rad, rate.theta_rad, rate.theta_rad),
    )
    target = torch.tensor(
        [float(v) for v in cfg.benchmark.wrap_pose],
        device=scene.device,
        dtype=torch.float32,
    ).view(1, 4).expand(scene.n_envs, 4)
    home = torch.zeros_like(target)
    dofs = list(scene.arm_dofs.all_indices)
    for i in range(steps):
        alpha = float(i + 1) / steps
        scene.robot.control_dofs_position(
            mapper.joint_targets(home + (target - home) * alpha), dofs_idx_local=dofs
        )
        scene.step()


def measure_point(cfg: WrapGraspConfig, n_envs: int) -> PointResult:
    """Build the wrap scene at `n_envs` and time steady-state stepping."""
    import torch

    from .scene import WrapGraspScene

    bench = cfg.benchmark
    try:
        t0 = time.perf_counter()
        scene = WrapGraspScene(cfg, n_envs=n_envs).build()
        build_s = time.perf_counter() - t0

        if bench.with_contact:
            _drive_into_wrap(cfg, scene)

        for _ in range(int(bench.warmup_steps)):
            scene.step()

        # Read contacts once so the measured loop includes the buffer the RL
        # env will actually query every substep.
        contacts = scene.robot.get_contacts(with_entity=scene.object)
        buffer_width = int(contacts["valid_mask"].shape[-1])
        live_contacts = int(contacts["valid_mask"].sum())

        steps = int(bench.measure_steps)
        t0 = time.perf_counter()
        for _ in range(steps):
            scene.step()
        elapsed = time.perf_counter() - t0

        total = steps * n_envs / elapsed
        return PointResult(
            n_envs=n_envs,
            ok=True,
            build_s=build_s,
            total_steps_per_s=total,
            per_env_steps_per_s=steps / elapsed,
            peak_rss_bytes=_peak_rss_bytes(),
            device_alloc_bytes=_device_alloc_bytes(scene.device.type),
            contact_buffer=buffer_width,
            live_contacts=live_contacts,
        )
    except Exception as exc:  # noqa: BLE001 - the failure itself is the datum
        return PointResult(
            n_envs=n_envs, ok=False, error=f"{type(exc).__name__}: {exc}"
        )


def _child_command(n_envs: int, argv_extra: list[str]) -> list[str]:
    return [
        sys.executable,
        "-m",
        "elesim_sim.rl.benchmark",
        "--n-envs",
        str(n_envs),
        "--json",
        *argv_extra,
    ]


def run_sweep(cfg: WrapGraspConfig, argv_extra: list[str]) -> list[PointResult]:
    results: list[PointResult] = []
    for n_envs in cfg.benchmark.n_envs_sweep:
        print(f"[benchmark] n_envs={n_envs} ...", flush=True)
        proc = subprocess.run(
            _child_command(int(n_envs), argv_extra),
            capture_output=True,
            text=True,
            cwd=str(_REPO_ROOT),
            check=False,
        )
        payload: Optional[dict[str, Any]] = None
        for line in reversed(proc.stdout.splitlines()):
            line = line.strip()
            if line.startswith("{"):
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError:
                    continue
                break
        if payload is None:
            tail = (proc.stderr or proc.stdout or "").strip().splitlines()
            reason = tail[-1] if tail else f"exit code {proc.returncode}"
            results.append(
                PointResult(n_envs=int(n_envs), ok=False, error=f"child failed: {reason}")
            )
        else:
            results.append(PointResult(**payload))
        last = results[-1]
        if last.ok:
            print(
                f"[benchmark]   {last.total_steps_per_s:,.0f} steps/s total, "
                f"{last.per_env_steps_per_s:,.1f} per env",
                flush=True,
            )
        else:
            print(f"[benchmark]   FAILED: {last.error}", flush=True)
    return results


def _fmt_bytes(value: Optional[int]) -> str:
    if value is None:
        return "n/a"
    return f"{value / (1024 ** 3):.2f} GiB"


def render_report(
    cfg: WrapGraspConfig, results: list[PointResult], env: dict[str, Any]
) -> str:
    lines: list[str] = []
    lines.append("# T1 - wrap-scene step rate")
    lines.append("")
    lines.append(
        "Throughput of the contact-enabled wrap scene (arm + cylinder + floor + "
        "GO2 trunk) against the parallel-environment count."
    )
    lines.append("")
    lines.append("## Measurement")
    lines.append("")
    lines.append(
        f"- `{cfg.benchmark.warmup_steps}` warm-up steps, then "
        f"`{cfg.benchmark.measure_steps}` timed `scene.step()` calls."
    )
    lines.append(
        "- Each `N` runs in its own child process, so a point that runs out of "
        "memory is recorded as a failure instead of aborting the sweep."
    )
    lines.append(
        f"- Physics `dt = {cfg.scene.dt}` s, solver substeps "
        f"`{cfg.scene.solver_substeps}`, `max_collision_pairs = "
        f"{cfg.scene.max_collision_pairs}`."
    )
    lines.append(
        "- Memory is peak process RSS. On unified-memory hosts (Apple silicon) "
        "that is the honest figure: there is no separate VRAM pool to read, and "
        "GPU-side allocations may not appear in RSS at all, so treat it as a "
        "lower bound."
    )
    lines.append(
        f"- `with_contact = {cfg.benchmark.with_contact}`. When true the arm is "
        "ramped into a wrapping pose first, so the timed loop carries live "
        "contacts. A run with `live contacts = 0` is measuring a scene where "
        "contact is enabled but never happens, and its rate does not describe "
        "wrap grasping."
    )
    lines.append("")
    lines.append("## Environment")
    lines.append("")
    lines.append("| item | value |")
    lines.append("|---|---|")
    for key in (
        "platform",
        "machine",
        "processor",
        "python",
        "torch",
        "torch_cuda",
        "cuda_available",
        "mps_available",
        "genesis",
        "nvidia_smi",
    ):
        lines.append(f"| `{key}` | {env.get(key, 'n/a')} |")
    lines.append("")
    lines.append(
        f"Genesis backend requested: `{cfg.runtime.backend}`. "
        "`torch_cuda` is the CUDA the torch wheel was built against, not `nvcc`."
    )
    lines.append("")
    lines.append("## Results")
    lines.append("")
    lines.append(
        "| N envs | total steps/s | per-env steps/s | build s | peak RSS | "
        "device alloc | contact buffer | live contacts |"
    )
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in results:
        if r.ok:
            lines.append(
                f"| {r.n_envs} | {r.total_steps_per_s:,.0f} | "
                f"{r.per_env_steps_per_s:,.1f} | {r.build_s:,.1f} | "
                f"{_fmt_bytes(r.peak_rss_bytes)} | "
                f"{_fmt_bytes(r.device_alloc_bytes)} | {r.contact_buffer} | "
                f"{r.live_contacts} |"
            )
        else:
            lines.append(
                f"| {r.n_envs} | **failed** | - | - | - | - | - | - |"
            )
    lines.append("")
    failures = [r for r in results if not r.ok]
    if failures:
        lines.append("### Failures")
        lines.append("")
        for r in failures:
            lines.append(f"- `N = {r.n_envs}`: {r.error}")
        lines.append("")
    return "\n".join(lines) + "\n"


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="base config YAML")
    parser.add_argument(
        "--overlay", action="append", default=[], help="overlay YAML (repeatable)"
    )
    parser.add_argument(
        "--set", action="append", default=[], dest="overrides",
        help="config override, e.g. --set benchmark.measure_steps=50",
    )
    parser.add_argument(
        "--n-envs", type=int, default=None,
        help="measure this single point instead of the configured sweep",
    )
    parser.add_argument(
        "--json", action="store_true", help="emit one JSON line (child mode)"
    )
    parser.add_argument("--out", default=None, help="report path override")
    args = parser.parse_args(argv)

    cfg = load_config(args.config, overlays=args.overlay, overrides=args.overrides)

    if args.n_envs is not None:
        result = measure_point(cfg, int(args.n_envs))
        if args.json:
            print(json.dumps(asdict(result)))
        else:
            print(json.dumps(asdict(result), indent=2))
        return 0 if result.ok else 1

    # Forward the same config selection to every child.
    argv_extra: list[str] = []
    if args.config:
        argv_extra += ["--config", args.config]
    for overlay in args.overlay:
        argv_extra += ["--overlay", overlay]
    for override in args.overrides:
        argv_extra += ["--set", override]

    results = run_sweep(cfg, argv_extra)
    env = environment_report()
    report = render_report(cfg, results, env)

    out_path = Path(args.out or cfg.benchmark.out_path)
    if not out_path.is_absolute():
        out_path = _REPO_ROOT / out_path
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding="utf-8")
    print(f"[benchmark] wrote {out_path}")
    return 0 if all(r.ok for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
