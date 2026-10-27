"""Runner contracts: failed experiments are useful evidence, never passing gates."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import zipfile

import pytest

from workbench.tools.bench.catalog import CATALOG, FAMILIES, ROOT, build, source_closure
from workbench.tools.bench.evidence import digest, index, pack, unpack
from workbench.tools.bench.runner import LocalExecutor, assess, exit_code, run, verify


def comparison_inputs(tmp_path, *, degraded=False):
    baseline = {"episodes": [{"episode": i, "kind": "box", "views_attempted": 20, "accepted": False,
                 "fused": {"recall_5mm": .55, "surface_p95_mm": 3., "dimension_error_mm": [2., 3., 4.]}}
                for i in range(6)], "collision_audit": {"accepted": True}}
    candidate = copy.deepcopy(baseline)
    for row in candidate["episodes"]:
        row["views_attempted"] = 5
    if degraded:
        candidate["episodes"][0]["fused"]["dimension_error_mm"][2] += 4.763
    inputs = {}
    for name, value in (("baseline", baseline), ("candidate", candidate)):
        path = tmp_path / f"{name}.json"
        path.write_text(json.dumps(value))
        inputs[name] = str(path)
    return inputs


@pytest.mark.parametrize("degraded,code", [(False, 0), (True, 1)])
def test_run_uses_real_evaluator_and_distinguishes_completion_from_acceptance(tmp_path, degraded, code):
    plan = build("multiview-compare", comparison_inputs(tmp_path, degraded=degraded))
    destination = tmp_path / "run"
    manifest = run(plan, destination)
    assert manifest["complete"] is True
    assert manifest["accepted"] is (not degraded)
    assert exit_code(manifest) == code
    assert verify(destination)["decision"]["result"]["capture_reduction_fraction"] == .75
    for verb, expected in (("verify", code), ("report", 0)):
        result = subprocess.run([sys.executable, "-m", "workbench.tools.bench", verb, str(destination)],
                                cwd=ROOT, capture_output=True, text=True)
        assert result.returncode == expected, result.stderr
    assert manifest["timings"]["evaluate_s"] > 0
    assert manifest["source"]["code_sha256"]
    with pytest.raises(FileExistsError):
        run(plan, destination)


def test_failed_evidence_round_trip_is_verified_and_tampering_rejected(tmp_path):
    plan = build("multiview-compare", comparison_inputs(tmp_path, degraded=True))
    source = tmp_path / "source"
    run(plan, source)
    archive = tmp_path / "failed.zip"
    record = pack(source, archive)
    destination = tmp_path / "collected"
    result = subprocess.run([sys.executable, "-m", "workbench.tools.bench", "collect", str(archive),
                             "--sha256", record["sha256"], "--output", str(destination)],
                            cwd=ROOT, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert verify(destination)["accepted"] is False
    (destination / "work/evaluation/result.json").write_text('{"accepted":true}')
    with pytest.raises(ValueError, match="artifacts changed"):
        verify(destination)


def test_collection_rejects_path_traversal_before_creating_destination(tmp_path):
    archive = tmp_path / "bad.zip"
    with zipfile.ZipFile(archive, "w") as out:
        out.writestr("../outside", "bad")
    with pytest.raises(ValueError, match="unsafe"):
        unpack(archive, tmp_path / "result", digest(archive))
    assert not (tmp_path / "outside").exists()
    assert not (tmp_path / "result").exists()


def test_preflight_failure_is_incomplete_with_no_worker_started(tmp_path):
    inputs = comparison_inputs(tmp_path)
    inputs["baseline"] = str(tmp_path / "missing.json")
    executor = LocalExecutor()
    manifest = run(build("multiview-compare", inputs), tmp_path / "run", executor)
    assert exit_code(manifest) == 2
    assert not manifest["complete"] and not manifest["accepted"]
    assert executor.processes == []
    assert "missing input" in manifest["error"]
    assert verify(tmp_path / "run")["error"] == manifest["error"]


def test_performance_gate_uses_entire_loop_cost_not_the_short_inner_timer(tmp_path):
    plan = build("sim-step", {"config": "config", "data": "data"}, {"repeat": 1})
    (tmp_path / "probe").mkdir()
    path = tmp_path / "probe/walking.jsonl"
    row = {"kind": "summary", "completed": True, "step_ms_p50_p95": [7., 9.], "whole_loop_step_ms": 18.}
    path.write_text(json.dumps(row) + "\n")
    decision = assess(plan.document(), tmp_path)
    assert decision["complete"] and not decision["accepted"]
    row["whole_loop_step_ms"] = 9.
    path.write_text(json.dumps(row) + "\n")
    assert assess(plan.document(), tmp_path)["accepted"]


def test_five_family_catalog_rejects_unused_knobs_and_separates_truth_inputs():
    assert {r.family for r in CATALOG} == set(FAMILIES)
    for recipe in CATALOG:
        inputs = {key: f"/inputs/{key}" for key in recipe.inputs}
        plan = build(recipe.name, inputs)
        assert plan.stages and plan.scope["hardware"] is False
        for stage in plan.stages:
            if stage.phase != "evaluate" and stage.role != "sim":
                assert "truth" not in stage.inputs
                assert not any("truth" in p for p in stage.reads)
        with pytest.raises(ValueError, match="unused/unknown"):
            build(recipe.name, inputs, {"no_eye_video": True})
    for params in ({"duration": -1}, {"repeat": 1.5}, {"cameras": "false"}, {"command": [float("nan"), 0, 0]}):
        with pytest.raises(ValueError):
            build("sim-step", {"config": "a", "data": "b"}, params)
    with pytest.raises(ValueError, match="poses must be 4..128"):
        build("visual-tilt", {"config": "a", "data": "b"}, {"poses": 2})
    for params in ({"max_views": 1}, {"candidates": 99}):
        with pytest.raises(ValueError):
            build("multiview-scan", {"data": "b"}, params)


def test_legacy_index_does_not_claim_acceptance_or_remove_copied_source(tmp_path):
    for name in ("first", "second"):
        folder = tmp_path / name
        folder.mkdir()
        (folder / "run.py").write_text("same old script\n")
    result = index(tmp_path)
    assert result["code_files"] == 2 and result["distinct_code_contents"] == 1
    assert all(row["classification"] == "legacy-unverified" for row in result["runs"])
    assert (tmp_path / "first/run.py").read_text() == "same old script\n"


def test_catalog_worker_arguments_are_accepted_without_loading_gpu_stack(tmp_path):
    checked = set()
    executor = LocalExecutor()
    for recipe in CATALOG:
        plan = build(recipe.name, {key: str(tmp_path / key) for key in recipe.inputs})
        for stage in plan.stages:
            if stage.module in checked:
                continue
            checked.add(stage.module)
            command, env = executor.command(stage, plan, tmp_path, {"checkpoint": {".": {"sha256": "0"*64}}})
            command += ["--help"]
            completed = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True, timeout=20)
            assert completed.returncode == 0, (stage.module, completed.stderr)
    assert not list(tmp_path.glob("**/walking.csv"))


def test_consumer_source_closure_excludes_private_scene_catalog_and_evidence():
    sources = source_closure("workbench.research.experiments.arm_scan", "workbench.research.support.worker")
    names = {path.name for path in sources}
    assert {"arm_scan.py", "worker.py", "output.py"} <= names
    assert "arm_scan_scene.py" not in names and "vision_arm_scene.py" not in names
    assert all("evidence" not in path.relative_to(ROOT).parts for path in sources)
