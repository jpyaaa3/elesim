"""Lifecycle/ownership checks for diagnostics; normal sessions are never targets."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

from workbench.tools.bench.catalog import Plan, Stage, build
from workbench.tools.bench.compose import ComposeExecutor
from workbench.tools.bench.runner import LocalExecutor, run


@pytest.mark.parametrize("failure", ["exit", "timeout"])
def test_failed_parallel_stage_cleans_only_its_own_process_group(tmp_path, failure):
    unrelated = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(30)"])

    class TestExecutor(LocalExecutor):
        def command(self, stage, plan, run, fingerprints):
            code = "import time;time.sleep(30)"
            if stage.name == "reject" and failure == "exit":
                code = "raise SystemExit(4)"
            return [sys.executable, "-c", code], os.environ.copy()
    executor = TestExecutor()
    plan = Plan("test", "multiview", {"hardware": False}, {}, {}, [
        Stage("producer", "unused", (), group="capture", timeout_s=.2 if failure == "timeout" else 10),
        Stage("reject", "unused", (), group="capture", timeout_s=10)], "missing.json")
    try:
        result = run(plan, tmp_path / "run", executor)
        assert not result["complete"] and not result["accepted"]
        assert result["error"].startswith("TimeoutError" if failure == "timeout" else "RuntimeError")
        assert all(process.poll() is not None for process in executor.processes)
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)


def test_cleanup_reaches_camera_child_after_worker_parent_has_exited(tmp_path):
    marker = tmp_path / "stopped"
    ready = tmp_path / "ready"
    child = ("import signal,time,pathlib;"
             f"signal.signal(signal.SIGTERM,lambda *_:(pathlib.Path({str(marker)!r}).write_text('stopped'),exit(0)));"
             f"pathlib.Path({str(ready)!r}).write_text('ready');time.sleep(30)")
    parent = f"import subprocess,sys;subprocess.Popen([sys.executable,'-c',{child!r}])"
    executor = LocalExecutor()
    process = executor.launch([sys.executable, "-c", parent], os.environ.copy(), tmp_path / "worker.log")
    process.wait(timeout=5)
    deadline = time.monotonic()+5
    while not ready.exists() and time.monotonic() < deadline:
        time.sleep(.01)
    assert ready.exists()
    try:
        executor.cleanup()
        deadline = time.monotonic()+2
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert marker.read_text() == "stopped"
    finally:
        executor.cleanup()


def test_compose_runtime_audit_rejects_wrong_gpu_or_extra_truth_mount(tmp_path):
    executor = ComposeExecutor(tmp_path, "test", gpu="GPU-selected")
    resource = {"gpu_uuid": "GPU-selected", "expected_mounts": [
        {"source": "/sensors", "target": "/input", "read_only": True}]}
    item = {"Mounts": [{"Type": "bind", "Source": "/sensors", "Destination": "/input", "RW": False}],
            "HostConfig": {"NetworkMode": "none", "ReadonlyRootfs": True, "Privileged": False,
                           "DeviceRequests": [{"DeviceIDs": ["GPU-selected"]}]}}
    executor._validate_container(item, resource)
    item["HostConfig"]["DeviceRequests"][0]["DeviceIDs"] = ["GPU-other"]
    with pytest.raises(ValueError, match="GPU differ"):
        executor._validate_container(item, resource)
    item["HostConfig"]["DeviceRequests"][0]["DeviceIDs"] = ["GPU-selected"]
    item["Mounts"].append({"Type": "bind", "Source": "/truth", "Destination": "/truth", "RW": False})
    with pytest.raises(ValueError, match="mounts/network/GPU"):
        executor._validate_container(item, resource)


def test_cleanup_refuses_container_from_another_installation(tmp_path):
    executor = ComposeExecutor(tmp_path, "test")
    executor.owner = {"install_uuid": "own", "docker": {"project": "project"}}
    executor.resources = [{"name": "owned-name", "id": "expected"}]
    executor._inspect = lambda name: {"Id": "expected", "Config": {"Labels": {
        "io.elesim.install_uuid": "other", "io.elesim.bench_id": executor.token,
        "com.docker.compose.project": "project"}}}
    calls = []
    executor._docker = lambda *args, **kwargs: calls.append(args)
    with pytest.raises(ValueError, match="foreign"):
        executor.cleanup()
    assert calls == []


def test_worker_rejects_hardware_config_before_entering_scene_code(tmp_path):
    import yaml
    from workbench.tools.bench.catalog import ROOT
    document = yaml.safe_load((ROOT / "payload/config/sim/config.yaml").read_text())
    document["simulation"]["runtime"]["use_hardware"] = True
    config = tmp_path / "sim.yaml"
    config.write_text(yaml.safe_dump(document))
    plan = build("visual-tilt", {"config": str(config), "data": str(ROOT / "payload/data")})
    executor = LocalExecutor()
    command, env = executor.command(plan.stages[0], plan, tmp_path, {})
    completed = subprocess.run(command, env=env, cwd=ROOT, text=True, capture_output=True, timeout=15)
    assert completed.returncode != 0
    assert "refuses use_hardware=true" in completed.stderr
    assert not (tmp_path / "work/scene").exists()


def test_config_extends_change_during_execution_cannot_pass(tmp_path):
    parent = tmp_path / "base.yaml"
    parent.write_text("schema_version: 1\nextends: null\nvalue: before\n")
    config = tmp_path / "config.yaml"
    config.write_text("schema_version: 1\nextends: base.yaml\n")
    class ChangedInputExecutor(LocalExecutor):
        def command(self, stage, plan, run, fingerprints):
            output = str(run / "work/result.json")
            code = (f"from pathlib import Path;Path({str(parent)!r}).write_text('changed');"
                    f"Path({output!r}).write_text('{{\"complete\":true,\"accepted\":true}}')")
            return [sys.executable, "-c", code], os.environ.copy()
    plan = Plan("test", "visual-calibration", {"hardware": False}, {"config": str(config)}, {},
                [Stage("compute", "unused", ())], "result.json")
    result = run(plan, tmp_path / "run", ChangedInputExecutor())
    assert result["complete"] is False and result["accepted"] is False
    assert "inputs changed" in result["error"]


@pytest.mark.skipif(shutil.which("docker") is None, reason="Docker Compose client not installed")
def test_real_compose_merge_drops_inherited_credentials_network_and_gpu_set(tmp_path):
    # `compose config` needs no daemon and never creates containers.
    prefix = tmp_path / "install"
    (prefix / "bin").mkdir(parents=True)
    (prefix / "containers").mkdir()
    wrapper = prefix / "bin/elesim-compose"
    wrapper.write_text('#!/bin/sh\nexec docker compose "$@"\n')
    wrapper.chmod(0o700)
    owner_uuid = "00000000-0000-4000-8000-000000000001"
    image = "sha256:" + "1"*64
    owner = {"prefix_realpath": str(prefix), "install_uuid": owner_uuid, "docker": {
        "context": "default", "engine_id": "pinned", "project": "bench-test"}}
    (prefix / "install-ownership.json").write_text(json.dumps(owner))
    services = {"tailscale": {"image": "alpine"}}
    for role in ("pilot", "sim"):
        services[f"svc-test-{role}"] = {"image": image, "network_mode": "service:tailscale",
            "depends_on": {"tailscale": {"condition": "service_started"}},
            "volumes": [f"{tmp_path}/private:/secrets:ro"], "ports": ["127.0.0.1:12345:12345"],
            "environment": {"CUDA_VISIBLE_DEVICES": "2", "SECRET": "test-only"},
            "deploy": {"resources": {"reservations": {"devices": [
                {"driver": "nvidia", "device_ids": ["0", "1", "2"], "capabilities": ["gpu"]}]}}},
            "labels": {"io.elesim.install_uuid": owner_uuid, "io.elesim.system_id": "test", "io.elesim.role": role}}
    (prefix / "containers/compose.yaml").write_text(json.dumps({"name": "bench-test", "services": services}))
    (prefix / "containers/compose.instances.yaml").write_text('{"services":{}}')
    destination = tmp_path / "run"
    (destination / "work/metadata").mkdir(parents=True)
    for directory in ("requests", "scans", "scene/sensors", "scene/truth", "evaluation"):
        (destination / "work" / directory).mkdir(parents=True)
    data = tmp_path / "data"
    data.mkdir()
    plan = build("multiview-scan", {"data": str(data)}, {"backend": "genesis"})
    executor = ComposeExecutor(prefix, "test", gpu="GPU-selected")
    def read_docker(*args, **kwargs):
        if args[0] == "info":
            return "pinned\n"
        if args[0] == "image":
            return json.dumps([{"Id": image, "Config": {"Labels": {"io.elesim.install_uuid": owner_uuid}}}])
        if args[0] == "ps":
            return ""
        raise AssertionError(f"unexpected Docker mutation/query: {args}")
    executor._docker = read_docker
    executor.preflight(plan, destination)
    for stage in plan.stages:
        command, env = executor.command(stage, plan, destination, {})
        assert "--no-deps" in command and "--rm" in command
        assert command[command.index("--pull")+1] == "never"
        mounts = executor.resources[-1]["expected_mounts"]
        assert not any(m["target"] == "/secrets" for m in mounts)
        if stage.name == "scan":
            assert not any("truth" in m["target"] or "arm_scan_scene.py" in m["target"] for m in mounts)
        assert "CUDA_VISIBLE_DEVICES" not in env
