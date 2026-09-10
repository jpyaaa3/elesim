from __future__ import annotations

import json
import multiprocessing
import os
from pathlib import Path

import pytest

from elesim_setup.instances import InstanceEndpoint, InstanceRegistry, InstanceState
from elesim_setup.state import ComputeSettings


KEY = "a" * 64


def instance(system: str, domain: int = 7, key: str = KEY) -> InstanceState:
    return InstanceState(system, key, (InstanceEndpoint("pilot", f"{system}-pilot"),), domain)


def test_round_trip_and_explicit_create_replace(tmp_path: Path):
    registry = InstanceRegistry(tmp_path / "prefix")
    value = instance("alpha")
    registry.save(value)
    assert registry.load("alpha") == value
    with pytest.raises(FileExistsError):
        registry.save(value)
    changed = instance("alpha", key="b" * 64)
    registry.save(changed, replace=True)
    assert registry.select() == changed


def test_instance_compute_policy_round_trips_and_legacy_records_inherit():
    value = InstanceState(
        "alpha",
        KEY,
        (InstanceEndpoint("pilot", "alpha-pilot"),),
        7,
        compute=ComputeSettings(gpu_mode="specific", gpu_device="GPU-abc123"),
    )
    restored = InstanceState.from_dict(value.to_dict())
    assert restored.compute == value.compute
    assert restored.compute_is_explicit is True

    legacy = value.to_dict()
    legacy["schema_version"] = 2
    legacy.pop("role_ids")
    legacy.pop("compute")
    migrated = InstanceState.from_dict(legacy)
    assert migrated.compute == ComputeSettings()
    assert migrated.compute_is_explicit is False
    assert migrated.pilot_id == "alpha-pilot"
    assert migrated.sim_id == "sim-default"
    assert migrated.ui_id == "ui-main"
    assert migrated.to_dict()["schema_version"] == 3
    assert "compute" not in migrated.to_dict()


def test_instance_compute_policy_rejects_non_exact_device_selector():
    with pytest.raises(ValueError, match="specific GPU policy"):
        InstanceState(
            "alpha",
            KEY,
            (InstanceEndpoint("pilot", "alpha-pilot"),),
            7,
            compute=ComputeSettings(gpu_mode="specific", gpu_device="gpu0"),
        )


def test_validation_is_strict():
    with pytest.raises(ValueError):
        InstanceState("alpha", "A" * 64, (), 0)
    with pytest.raises(ValueError):
        InstanceEndpoint("robot", "robot")
    with pytest.raises(ValueError):
        InstanceState.from_dict({"schema": 1, "system_id": "alpha"})
    with pytest.raises(ValueError):
        InstanceState(1, KEY, (InstanceEndpoint("pilot", "ep"),), 0)
    with pytest.raises(ValueError):
        InstanceEndpoint("pilot", "A" * 63)
    with pytest.raises(ValueError):
        InstanceState("alpha", None, (InstanceEndpoint("pilot", "ep"),), 0)
    with pytest.raises(ValueError):
        InstanceState.from_dict({"schema_version": True, "system_id": "alpha", "release_key": KEY, "endpoints": [{"role": "pilot", "endpoint_id": "ep"}], "domain_id": 0})


def test_rejected_collision_preserves_existing_state(tmp_path: Path):
    registry = InstanceRegistry(tmp_path / "prefix")
    registry.save(instance("alpha", 3))
    with pytest.raises(ValueError, match="collision"):
        registry.save(instance("beta", 3))
    assert registry.list() == (instance("alpha", 3),)


def test_selection_requires_one_unambiguous_instance(tmp_path: Path):
    registry = InstanceRegistry(tmp_path / "prefix")
    with pytest.raises(LookupError):
        registry.select()
    registry.save(instance("alpha"))
    registry.save(instance("beta", 8))
    with pytest.raises(LookupError, match="ambiguous"):
        registry.select()


def test_symlinked_managed_directory_is_rejected(tmp_path: Path):
    prefix = tmp_path / "prefix"
    prefix.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (prefix / "instances").symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        InstanceRegistry(prefix).save(instance("alpha"))
    assert not (outside / "instances").exists()


def test_directory_name_and_state_must_agree(tmp_path: Path):
    registry = InstanceRegistry(tmp_path / "prefix")
    registry.save(instance("alpha"))
    path = tmp_path / "prefix" / "instances" / "alpha" / "state.json"
    payload = json.loads(path.read_text())
    payload["system_id"] = "beta"
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="does not match"):
        registry.load("alpha")


def test_corrupt_replace_and_replace_failure_preserve_old_state(tmp_path: Path, monkeypatch):
    registry = InstanceRegistry(tmp_path / "prefix")
    old = instance("alpha")
    registry.save(old)
    path = tmp_path / "prefix" / "instances" / "alpha" / "state.json"
    path.write_text("not json")
    with pytest.raises(ValueError):
        registry.save(instance("alpha", key="b" * 64), replace=True)
    assert path.read_text() == "not json"
    path.write_text(json.dumps(old.to_dict()))
    original = path.read_bytes()
    monkeypatch.setattr("elesim_setup.instances.os.replace", lambda *_: (_ for _ in ()).throw(OSError("nope")))
    with pytest.raises(OSError):
        registry.save(instance("alpha", key="b" * 64), replace=True)
    assert path.read_bytes() == original
    assert not list(path.parent.glob(".state.*"))


def test_incomplete_other_entry_blocks_save(tmp_path: Path):
    registry = InstanceRegistry(tmp_path / "prefix")
    broken = tmp_path / "prefix" / "instances" / "broken"
    broken.mkdir(parents=True)
    with pytest.raises(ValueError, match="missing"):
        registry.save(instance("alpha"))


def test_removed_instance_snapshots_do_not_block_registry(tmp_path: Path):
    registry = InstanceRegistry(tmp_path / "prefix")
    registry.save(instance("alpha"))
    retained = tmp_path / "prefix" / "instances" / "alpha"
    (retained / "state.json").unlink()
    (retained / "security").mkdir()
    (retained / "security" / "generation").write_text("retained")
    (retained / "security" / ".lock").write_text("")
    assert registry.list() == ()
    registry.save(instance("beta"))
    assert registry.load("beta").system_id == "beta"


def test_first_publication_failure_does_not_poison_registry(tmp_path: Path, monkeypatch):
    registry = InstanceRegistry(tmp_path / "prefix")
    registry.save(instance("alpha"))
    with monkeypatch.context() as patch:
        patch.setattr("elesim_setup.instances.os.replace", lambda *_: (_ for _ in ()).throw(OSError("nope")))
        with pytest.raises(OSError, match="nope"):
            registry.save(instance("beta", 8))
    assert registry.list() == (instance("alpha"),)
    registry.save(instance("beta", 8))
    assert len(registry.list()) == 2


def test_non_regular_lock_is_rejected_without_blocking(tmp_path: Path):
    registry = InstanceRegistry(tmp_path)
    registry.lock_path.parent.mkdir(parents=True)
    os.mkfifo(registry.lock_path)
    with pytest.raises(ValueError, match="regular file"):
        registry.list()


def _save_worker(prefix: str, name: str) -> None:
    InstanceRegistry(prefix).save(instance(name, 20))


def test_concurrent_writers_keep_valid_complete_files(tmp_path: Path):
    prefix = str(tmp_path / "prefix")
    ctx = multiprocessing.get_context("fork")
    processes = [ctx.Process(target=_save_worker, args=(prefix, name)) for name in ("alpha", "beta")]
    for process in processes:
        process.start()
    for process in processes:
        process.join(10)
    assert sorted(process.exitcode == 0 for process in processes) == [False, True]
    registry = InstanceRegistry(prefix)
    states = registry.list()
    assert len(states) == 1
    for state in states:
        raw = json.loads((Path(prefix) / "instances" / state.system_id / "state.json").read_text())
        assert raw["schema_version"] == 3
