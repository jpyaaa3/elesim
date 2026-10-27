"""Capture the executing environment before entering a benchmark calculation worker."""
from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
from enum import Enum
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import runpy
import subprocess
import sys
import time


def config_document(value):
    """Snapshot resolved limits, representing intentional infinities as text."""
    if is_dataclass(value):
        return config_document(asdict(value))
    if isinstance(value, dict):
        return {str(k): config_document(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [config_document(v) for v in value]
    if isinstance(value, (Path, Enum)):
        return str(value) if isinstance(value, Path) else config_document(value.value)
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--module", required=True)
    parser.add_argument("--role", choices=("pilot", "sim", "workspace"), required=True)
    parser.add_argument("--simulation-config", type=Path)
    parser.add_argument("args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not args.module.startswith("workbench.research."):
        parser.error("only repository research workers are supported")
    roles = ("pilot", "sim") if args.role == "workspace" else (args.role,)
    sources = {}
    for package in ("elesim_protocol", *(f"elesim_{role}" for role in roles)):
        spec = importlib.util.find_spec(package)
        if spec and spec.submodule_search_locations:
            root = Path(next(iter(spec.submodule_search_locations)))
            sources[package] = {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted(root.rglob("*.py"))}
    versions = {}
    for name in ("numpy", "scipy", "torch", "genesis-world", "jax", "jaxlib"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    gpu = None
    if os.environ.get("ELESIM_BENCH_GPU"):
        probe = subprocess.run(["nvidia-smi", "--query-gpu=uuid,name,driver_version", "--format=csv,noheader"],
                               capture_output=True, text=True, timeout=15, check=True)
        gpu = probe.stdout.strip().splitlines()
        if len(gpu) != 1 or gpu[0].split(",")[0].strip() != os.environ["ELESIM_BENCH_GPU"]:
            # On a native host nvidia-smi still sees all GPUs even when CUDA is
            # restricted. Match the explicit UUID; Compose must expose one.
            matches = [row for row in gpu if row.split(",")[0].strip() == os.environ["ELESIM_BENCH_GPU"]]
            if len(matches) != 1:
                raise RuntimeError("benchmark GPU UUID not observed")
            gpu = matches
    record = {"python": sys.version, "platform": platform.platform(), "packages": versions,
              "runtime_source_sha256": sources, "gpu": gpu, "module": args.module,
              "environment": {k: os.environ.get(k) for k in (
                  "CUDA_VISIBLE_DEVICES", "EGL_DEVICE_ID", "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")},
              "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    entry = importlib.util.find_spec(args.module)
    record["entry_source_sha256"] = hashlib.sha256(Path(entry.origin).read_bytes()).hexdigest()
    record["worker_source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.metadata.parent.mkdir(parents=True, exist_ok=True)

    def save():
        args.metadata.write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    save()
    sys.argv = [args.module, *(args.args[1:] if args.args[:1] == ["--"] else args.args)]
    began = time.monotonic()
    try:
        if args.simulation_config and "--help" not in sys.argv:
            from elesim_sim.config import load_app_config
            record["simulation_config"] = {}
            for mode in (None, "remote"):
                bundle = load_app_config(args.simulation_config, mode=mode)
                record["simulation_config"][mode or "default"] = config_document(bundle)
                if bundle.sim_config.use_hardware:
                    raise RuntimeError("simulation benchmark refuses use_hardware=true")
        runpy.run_module(args.module, run_name="__main__")
    except BaseException as exc:
        record["exit"] = exc.code if isinstance(exc, SystemExit) else type(exc).__name__
        raise
    else:
        record["exit"] = 0
    finally:
        record["worker_wall_s"] = time.monotonic()-began
        save()


if __name__ == "__main__":
    main()
