"""Execute catalog stages and persist the distinction between completion and acceptance."""
from __future__ import annotations

import importlib.metadata
import json
import os
from pathlib import Path, PurePosixPath
import platform
import signal
import subprocess
import sys
import time

from .catalog import Plan, ROOT, SCHEMA_VERSION
from .evidence import artifacts, check_artifacts, config_sources, digest, inventory, write_json

ROLE_PATHS = {
    "pilot": ("payload/runtime/docker/pilot/app",),
    "sim": ("payload/runtime/docker/sim/app",),
    "workspace": ("payload/runtime/docker/pilot/app", "payload/runtime/docker/sim/app"),
}


def source_identity() -> dict:
    def git(*args):
        result = subprocess.run(["git", *args], cwd=ROOT, text=True, capture_output=True, check=False)
        return result.stdout.strip() if result.returncode == 0 else None
    # Include uncommitted workbench code: HEAD alone cannot identify a modified runner.
    code = {}
    for tree in (ROOT / "workbench/research", ROOT / "workbench/tools/bench"):
        for path in sorted(tree.rglob("*")):
            if path.suffix in (".py", ".json", ".sh") and path.is_file():
                code[path.relative_to(ROOT).as_posix()] = digest(path)
    return {"git_head": git("rev-parse", "HEAD"), "git_status": git("status", "--porcelain"),
            "code_sha256": code, "python": sys.version, "platform": platform.platform(),
            "packages": {name: dist.version for name in ("numpy", "scipy", "torch", "genesis-world", "jax")
                         if (dist := next(iter(importlib.metadata.distributions(name=name)), None))}}


def assess(plan: dict, work: Path) -> dict:
    relative = PurePosixPath(plan["result"])
    if relative.is_absolute() or ".." in relative.parts or "\\" in plan["result"]:
        raise ValueError("result path must stay inside run/work")
    path = work / str(relative)
    if plan["gate"] in ("walking", "performance"):
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        json.dumps(rows, allow_nan=False)
        summaries = [row for row in rows if row.get("kind") == "summary"]
        complete = len(summaries) == plan["parameters"]["repeat"]
        accepted = complete and all(row.get("completed") is True for row in summaries)
        if plan["gate"] == "performance":
            budget = plan["parameters"]["budget_ms"]
            accepted = accepted and all(
                type(row.get("whole_loop_step_ms")) in (int, float) and
                0 < row["whole_loop_step_ms"] <= budget for row in summaries)
        return {"complete": complete, "accepted": bool(accepted), "summaries": summaries}
    result = json.loads(path.read_text())
    json.dumps(result, allow_nan=False)
    if type(result.get("accepted")) is not bool:
        raise ValueError("evaluator did not declare a boolean accepted result")
    return {"complete": result.get("complete", True) is True, "accepted": result["accepted"],
            "result": result}


class LocalExecutor:
    """Owns only the process groups it launches; never discovers or kills services."""
    def __init__(self, *, gpu: str | None = None):
        self.gpu = gpu
        self.processes = []
        self.environment = {}

    def preflight(self, plan: Plan, run: Path) -> dict:
        if any(s.gpu for s in plan.stages):
            if not self.gpu or not self.gpu.startswith("GPU-"):
                raise ValueError("GPU stages require an explicit --gpu UUID (or use --install Compose execution)")
            result = subprocess.run(["nvidia-smi", "--query-gpu=index,uuid", "--format=csv,noheader"],
                                    text=True, capture_output=True, check=True)
            matches = [line.split(",")[0].strip() for line in result.stdout.splitlines()
                       if line.split(",")[-1].strip() == self.gpu]
            if len(matches) != 1:
                raise ValueError("requested GPU UUID is not present")
            visible = [line for line in result.stdout.splitlines() if line.strip()]
            if plan.scope.get("cameras") and len(visible) != 1:
                raise ValueError("camera benchmarks need GPU-scoped Compose/dev execution; other GPUs remain visible")
            self.environment.update(CUDA_VISIBLE_DEVICES=self.gpu,
                                    EGL_DEVICE_ID="0" if len(visible) == 1 else matches[0])
        return {"kind": "local", "gpu_uuid": self.gpu, "isolation": "process only; use Compose for truth mount isolation"}

    def command(self, stage, plan: Plan, run: Path, fingerprints: dict) -> tuple[list[str], dict]:
        bindings = {**plan.inputs, "run": str(run / "work")}
        if "checkpoint" in fingerprints:
            bindings["checkpoint_sha256"] = fingerprints["checkpoint"]["."]["sha256"]
        args = [a.format_map(bindings) for a in stage.args]
        env = os.environ.copy()
        env.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="2", PYTHONUNBUFFERED="1",
                   PYTHONDONTWRITEBYTECODE="1", **self.environment)
        if stage.gpu and self.gpu:
            env["ELESIM_BENCH_GPU"] = self.gpu
        roots = ("payload/runtime/common/protocol", *ROLE_PATHS[stage.role], "model")
        env["PYTHONPATH"] = os.pathsep.join((str(ROOT), *(str(ROOT / p) for p in roots)))
        wrapper = [sys.executable, "-m", "workbench.research.support.worker", "--metadata",
                   str(run / "work/metadata" / f"{stage.name}.json"), "--module", stage.module,
                   "--role", stage.role]
        if "config" in stage.inputs and stage.role in ("sim", "workspace"):
            wrapper += ["--simulation-config", bindings["config"]]
        return [*wrapper, "--", *args], env

    def launch(self, command: list, env: dict, log: Path):
        with log.open("w", encoding="utf-8") as stream:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process

    def cleanup(self) -> dict:
        stopped = []
        for process in self.processes:
            # A worker may have exited while its camera child still lives.
            # Only the process group created by this executor is addressed.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
            deadline = time.monotonic()+5
            while True:
                process.poll()  # Reap the parent even if only an orphan child remains.
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    break
                if time.monotonic() >= deadline:
                    os.killpg(process.pid, signal.SIGKILL)
                    break
                time.sleep(.05)
            process.wait(timeout=5)
            stopped.append(process.pid)
        return {"own_processes_stopped": stopped}


def _input_fingerprints(plan: Plan, run: Path) -> dict:
    inputs = {name: Path(value) for name, value in plan.inputs.items()}
    for name, path in inputs.items():
        if not path.exists():
            raise ValueError(f"missing input {name}: {path}")
        if run.is_relative_to(path) or path.is_relative_to(run):
            raise ValueError("inputs and run directory must not overlap")
        if name == "truth" and any(path.is_relative_to(other) or other.is_relative_to(path)
                                   for key, other in inputs.items() if key != "truth"):
            raise ValueError("private truth must be separate from inference inputs")
    fingerprints = {name: inventory(path) for name, path in inputs.items()}
    for name in ("config", "pilot_config"):
        if name in inputs:
            for parent in config_sources(inputs[name])[1:]:
                fingerprints[name]["extends:" + str(parent)] = {
                    "bytes": parent.stat().st_size, "sha256": digest(parent)}
    return fingerprints


def _run_group(stages, plan, run, fingerprints, executor) -> list[dict]:
    running, records = [], []
    for stage in stages:
        command, env = executor.command(stage, plan, run, fingerprints)
        began = time.monotonic()
        process = executor.launch(command, env, run / "logs" / f"{stage.name}.log")
        running.append((stage, process, began))
        records.append({"name": stage.name, "phase": stage.phase, "command": command,
                        "timeout_s": stage.timeout_s})
    pending = set(range(len(running)))
    while pending:
        for index in sorted(pending):
            stage, process, began = running[index]
            code = process.poll()
            elapsed = time.monotonic() - began
            if elapsed > stage.timeout_s and code is None:
                raise TimeoutError(f"stage timed out: {stage.name}")
            if code is not None:
                records[index].update(exit_code=code, wall_s=elapsed)
                pending.remove(index)
                if code and not (code == 1 and len(stages) == 1):
                    raise RuntimeError(f"stage failed: {stage.name} (exit {code})")
        if pending:
            time.sleep(.05)
    return records


def run(plan: Plan, destination: Path, executor=None) -> dict:
    destination = destination.expanduser().resolve()
    for value in plan.inputs.values():
        path = Path(value).resolve()
        if destination.is_relative_to(path) or path.is_relative_to(destination):
            raise ValueError("inputs and run directory must not overlap")
    destination.mkdir(parents=True, exist_ok=False)
    (destination / "logs").mkdir()
    work = destination / "work"
    work.mkdir()
    (work / "metadata").mkdir()
    for stage in plan.stages:
        for relative in stage.writes:
            (work / relative).mkdir(parents=True, exist_ok=True)
    # Read mounts used concurrently by the producer and consumer must exist.
    for stage in plan.stages:
        if not stage.group:
            continue
        for relative in stage.reads:
            (work / relative).mkdir(parents=True, exist_ok=True)
    document = plan.document()
    write_json(destination / "plan.json", document)
    executor = executor or LocalExecutor()
    started = time.monotonic()
    manifest = {"schema_version": SCHEMA_VERSION, "benchmark": plan.name, "family": plan.family,
                "scope": plan.scope, "complete": False, "accepted": False,
                "stages": [], "timings": {}, "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    phase_start = started
    try:
        manifest["source"] = source_identity()
        fingerprints = _input_fingerprints(plan, destination)
        manifest["inputs"] = fingerprints
        manifest["executor"] = executor.preflight(plan, destination)
        manifest["timings"]["preflight_s"] = time.monotonic() - phase_start
        queue = list(plan.stages)
        while queue:
            stage = queue.pop(0)
            group = [stage]
            if stage.group:
                while queue and queue[0].group == stage.group:
                    group.append(queue.pop(0))
            phase_start = time.monotonic()
            records = _run_group(group, plan, destination, fingerprints, executor)
            manifest["stages"].extend(records)
            key = stage.phase + "_s"
            manifest["timings"][key] = manifest["timings"].get(key, 0.) + time.monotonic() - phase_start
            if any(record["exit_code"] for record in records) and queue:
                raise RuntimeError(f"worker failed before evaluation: {stage.name}")
        decision = assess(document, work)
        phase_start = time.monotonic()
        try:
            after = _input_fingerprints(plan, destination)
        except (ValueError, OSError) as exc:
            raise ValueError("benchmark inputs changed during execution") from exc
        if after != fingerprints:
            raise ValueError("benchmark inputs changed during execution")
        manifest["timings"]["input_recheck_s"] = time.monotonic()-phase_start
        for record in manifest["stages"]:
            if record["exit_code"] == 1:
                metadata = work / "metadata" / f"{record['name']}.json"
                if not metadata.is_file() or json.loads(metadata.read_text()).get("exit") != 1:
                    raise RuntimeError(f"worker crashed rather than completing evaluation: {record['name']}")
        # Exit 1 is an expected criteria failure only for the final evaluator/probe.
        if decision["accepted"] and any(r["exit_code"] != 0 for r in manifest["stages"]):
            raise ValueError("worker failed despite claiming acceptance")
        manifest.update(complete=decision["complete"], accepted=decision["complete"] and decision["accepted"])
        manifest["decision"] = decision
    except (Exception, KeyboardInterrupt) as exc:
        manifest["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        phase_start = time.monotonic()
        try:
            manifest["cleanup"] = executor.cleanup()
        except Exception as exc:
            manifest.update(complete=False, accepted=False, cleanup_error=str(exc))
        manifest["timings"]["cleanup_s"] = time.monotonic() - phase_start
        phase_start = time.monotonic()
        manifest["artifacts"] = artifacts(destination)
        manifest["timings"]["collect_s"] = time.monotonic() - phase_start
        manifest["timings"]["total_wall_s"] = time.monotonic() - started
        write_json(destination / "manifest.json", manifest)
        report(destination)
    return manifest


def verify(run: Path) -> dict:
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported benchmark manifest version")
    if (type(manifest.get("complete")) is not bool or type(manifest.get("accepted")) is not bool or
            (manifest["accepted"] and not manifest["complete"])):
        raise ValueError("manifest completion/acceptance must be consistent booleans")
    check_artifacts(run, manifest)
    if manifest.get("complete"):
        decision = assess(json.loads((run / "plan.json").read_text()), run / "work")
        if not decision["complete"] or decision["accepted"] != manifest["accepted"]:
            raise ValueError("saved acceptance differs from re-evaluation")
    return manifest


def exit_code(manifest: dict) -> int:
    return 0 if manifest.get("complete") and manifest.get("accepted") else (1 if manifest.get("complete") else 2)


def report(run: Path) -> str:
    manifest = verify(run)
    state = "PASS" if exit_code(manifest) == 0 else "FAIL" if manifest.get("complete") else "INCOMPLETE"
    text = (f"{manifest['benchmark']}: {state}\n"
            f"complete={manifest['complete']} accepted={manifest['accepted']}\n"
            f"scope={json.dumps(manifest['scope'], ensure_ascii=False)}\n"
            f"timings={json.dumps(manifest['timings'])}\n")
    if manifest.get("error"):
        text += manifest["error"] + "\n"
    (run / "report.txt").write_text(text, encoding="utf-8")
    return text
