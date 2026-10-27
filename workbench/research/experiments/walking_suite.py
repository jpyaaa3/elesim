"""Data-driven DDS gaze trials in a fresh, loopback-only simulation system.

Run through workbench.tools.bench. This worker never joins an installed runtime
graph, starts physical Robot, or stops an existing service.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import time
import uuid

ROOT = Path(__file__).resolve().parents[3]
SUITES = Path(__file__).with_name("walking_suites.json")


def suite_cases(name):
    document = json.loads(SUITES.read_text())
    if document["schema_version"] != 1:
        raise ValueError("unsupported walking scenario version")
    return document["suites"][name]


def runtime_configs(output, token):
    """Fresh identities and loopback prevent a bench lease reaching a normal target."""
    import yaml
    paths = {}
    source_topic = f"/elesim/bench_{token}/sim/rgbd/frame"
    broker_topic = f"/elesim/bench_{token}/pilot/rgbd/frame"
    for role in ("pilot", "sim"):
        document = yaml.safe_load((ROOT / f"payload/config/{role}/runtime.yaml").read_text())
        endpoint = f"{role}-{token}"
        document["runtime"]["endpoint_id"] = endpoint
        if role == "pilot":
            document["runtime"]["active_target"] = f"sim-{token}"
        else:
            document["runtime"]["mirror_robot_id"] = ""
            document["runtime"]["streams"] = {"rgbd_topic": source_topic,
                "observer": f"webrtc://{endpoint}/observer", "hand_eye_preview": f"webrtc://{endpoint}/hand_eye_preview"}
        document["rgbd"]["wire"]["topic"] = broker_topic
        for name in ("right_topic", "left_preview_topic"):
            document["rgbd"]["wire"].pop(name, None)
        document["dds"].update(system_id=f"bench-{token}", node_key=endpoint,
            domain_id=179, discovery_mode="multicast", static_peers=[], network_interface="lo",
            vendor_config="", security_profile="trusted-network", keystore="", enclave="")
        path = output / f"{role}-runtime.yaml"
        path.write_text(yaml.safe_dump(document, sort_keys=False))
        paths[role] = path
    return paths


@contextmanager
def trial_environment(directory, token):
    previous_cwd, previous_env = Path.cwd(), os.environ.copy()
    try:
        for key in tuple(os.environ):
            if key.startswith(("ELESIM_DDS_", "ROS_SECURITY_")) or key in (
                    "ELESIM_SYSTEM_ID", "ROS_DOMAIN_ID", "CYCLONEDDS_URI"):
                os.environ.pop(key)
        os.environ.update(ROS_DOMAIN_ID="179", ELESIM_SYSTEM_ID=f"bench-{token}",
                          ELESIM_WALKING_METRICS="1")
        os.chdir(directory)
        yield
    finally:
        os.chdir(previous_cwd)
        os.environ.clear()
        os.environ.update(previous_env)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pilot-config", type=Path, required=True)
    parser.add_argument("--sim-config", type=Path, required=True)
    parser.add_argument("--suite", choices=("full", "resume", "ablation", "preview-sign"), default="full")
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--trial-start", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    from workbench.research.experiments import run_walking_baseline_batch as batch
    from workbench.research.experiments.config_overlay import write_config_overlay
    from workbench.research.analysis.analyze_walking_metrics import summarize_run
    from workbench.research.support.output import fresh_directory
    from elesim_pilot.config import load_runtime_role_config as pilot_role
    from elesim_sim.config import load_runtime_role_config as sim_role

    output = fresh_directory(args.output.resolve())
    pilot_config, sim_config = args.pilot_config.resolve(), args.sim_config.resolve()
    token = uuid.uuid4().hex[:16]
    runtime = runtime_configs(output, token)
    pilot_role(runtime["pilot"])
    sim_role(runtime["sim"])
    results = []
    began = time.monotonic()
    for case in suite_cases(args.suite):
        folder = output / case["name"]
        folder.mkdir()
        config = pilot_config
        if "b_pitch" in case:
            config = write_config_overlay(config, folder / "pilot.yaml", {
                "behaviors.gaze.preview.enable": True, "behaviors.gaze.preview.b_pitch": case["b_pitch"]})
        with trial_environment(folder, token):
            logs = folder / "logs/walking_baseline"
            result = batch.main(["--pilot-config", str(config), "--sim-config", str(sim_config),
                "--pilot-runtime", str(runtime["pilot"]), "--sim-runtime", str(runtime["sim"]),
                "--working-directory", str(folder), "--run-prefix", f"bench_{token}_{case['name']}",
                "--gaze", case["gaze"], "--duration", str(args.duration), "--trials", str(args.trials),
                "--trial-start", str(args.trial_start), "--log-dir", str(logs),
                "--batch-log-dir", str(folder / "process-logs"), "--result", str(folder / "result.json")])
            # Analysis failures propagate; no `|| true` and no fabricated summary.
            summaries = [summarize_run(r["run_id"], logs, write_merged=True, write_plots=False)
                         for r in result["trials"]]
        results.append({"condition": case, **result, "analysis": summaries})
    result = {"schema_version": 1, "complete": True,
              "accepted": bool(results and all(r["accepted"] for r in results)),
              "conditions": results, "wall_s": time.monotonic()-began, "clock": "wall",
              "system_id": f"bench-{token}", "scope": "simulation only; no viewer connected; paired gaze superiority not claimed"}
    (output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
