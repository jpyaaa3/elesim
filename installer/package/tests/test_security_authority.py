from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Sequence

import pytest

from elesim_setup import security_authority as authority_module
from elesim_setup.security_authority import (
    EnclaveIdentity,
    SecurityAuthorityError,
    Sros2Authority,
    verify_bundle,
)


def test_real_cli_runner_prefers_the_ros_distro_python_abi(monkeypatch) -> None:
    captured: dict[str, object] = {}

    def run(command, *, check, env, stdout, stderr, text, timeout):
        captured.update(
            command=command,
            check=check,
            env=env,
            stdout=stdout,
            stderr=stderr,
            text=text,
            timeout=timeout,
        )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(authority_module.subprocess, "run", run)
    authority_module.subprocess_command_runner(
        ("ros2", "security", "create_keystore", "/tmp/example")
    )

    assert captured["check"] is False
    assert captured["command"] == (
        "ros2",
        "security",
        "create_keystore",
        "/tmp/example",
    )
    environment = captured["env"]
    assert isinstance(environment, dict)
    assert environment["PYTHONPATH"].split(os.pathsep)[0] == (
        "/usr/lib/python3/dist-packages"
    )
    assert captured["stdout"] is authority_module.subprocess.PIPE
    assert captured["stderr"] is authority_module.subprocess.PIPE
    assert captured["text"] is True
    assert captured["timeout"] == 120


def test_real_cli_runner_bounds_failure_output(monkeypatch) -> None:
    def run(*_args, **_kwargs):
        return SimpleNamespace(returncode=7, stdout="", stderr="x" * 5000)

    monkeypatch.setattr(authority_module.subprocess, "run", run)

    with pytest.raises(SecurityAuthorityError) as failure:
        authority_module.subprocess_command_runner(("ros2", "security", "boom"))

    message = str(failure.value)
    assert "boom" in message
    assert len(message) < 4200


class FakeRos2SecurityRunner:
    """Create the ROS keystore shape without generating cryptographic keys."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, ...]] = []

    def __call__(self, command: Sequence[str]) -> None:
        call = tuple(command)
        self.calls.append(call)
        assert call[:2] == ("ros2", "security")
        operation = call[2]
        if operation == "create_keystore":
            keystore = Path(call[3])
            for name in ("public", "private", "enclaves"):
                (keystore / name).mkdir(parents=True)
            (keystore / "public/identity_ca.cert.pem").write_text(
                "identity-ca", encoding="utf-8"
            )
            (keystore / "public/permissions_ca.cert.pem").write_text(
                "permissions-ca", encoding="utf-8"
            )
            (keystore / "public/governance.p7s").write_text(
                "governance", encoding="utf-8"
            )
            (keystore / "private/identity_ca.key.pem").write_text(
                "authority-identity-private", encoding="utf-8"
            )
            (keystore / "private/permissions_ca.key.pem").write_text(
                "authority-permissions-private", encoding="utf-8"
            )
            return
        if operation == "create_enclave":
            keystore = Path(call[3])
            enclave = Path(*call[4].lstrip("/").split("/"))
            destination = keystore / "enclaves" / enclave
            destination.mkdir(parents=True)
            for name, content in {
                "cert.pem": "role-certificate",
                "key.pem": "role-private-key",
                "identity_ca.cert.pem": "identity-ca",
                "permissions_ca.cert.pem": "permissions-ca",
                "governance.p7s": "governance",
                "permissions.p7s": "default-permissions",
            }.items():
                (destination / name).write_text(content, encoding="utf-8")
            return
        if operation == "create_permission":
            keystore = Path(call[3])
            enclave = Path(*call[4].lstrip("/").split("/"))
            policy = Path(call[5])
            (keystore / "enclaves" / enclave / "permissions.p7s").write_text(
                f"signed:{policy.read_text(encoding='utf-8')}",
                encoding="utf-8",
            )
            return
        raise AssertionError(f"unexpected fake ROS security command: {call}")


class SymlinkRos2SecurityRunner(FakeRos2SecurityRunner):
    """Approximate the file-link layout emitted by Humble's SROS2 CLI."""

    def __call__(self, command: Sequence[str]) -> None:
        super().__call__(command)
        operation = command[2]
        keystore = Path(command[3])
        if operation == "create_keystore":
            public_ca = keystore / "public/ca.cert.pem"
            public_ca.write_text("identity-ca", encoding="utf-8")
            private_ca = keystore / "private/ca.key.pem"
            private_ca.write_text("authority-private", encoding="utf-8")
            for name in ("identity_ca.cert.pem", "permissions_ca.cert.pem"):
                path = keystore / "public" / name
                path.unlink()
                path.symlink_to("ca.cert.pem")
            for name in ("identity_ca.key.pem", "permissions_ca.key.pem"):
                path = keystore / "private" / name
                path.unlink()
                path.symlink_to("ca.key.pem")
            (keystore / "enclaves/governance.p7s").write_text(
                "governance", encoding="utf-8"
            )
        elif operation == "create_enclave":
            enclave = keystore / "enclaves" / Path(
                *command[4].lstrip("/").split("/")
            )
            targets = {
                "identity_ca.cert.pem": keystore / "public/identity_ca.cert.pem",
                "permissions_ca.cert.pem": (
                    keystore / "public/permissions_ca.cert.pem"
                ),
                "governance.p7s": keystore / "enclaves/governance.p7s",
            }
            for name, target in targets.items():
                path = enclave / name
                path.unlink()
                path.symlink_to(os.path.relpath(target, enclave))


@pytest.fixture
def fake_security_runner() -> FakeRos2SecurityRunner:
    return FakeRos2SecurityRunner()


def _issued_generation(
    tmp_path: Path,
    runner: FakeRos2SecurityRunner,
    generation: str = "g-0001",
):
    authority = Sros2Authority(tmp_path / "authority", runner=runner)
    transaction = authority.begin_generation("lab_a", generation=generation)
    pilot = transaction.create_enclave("pilot", "pilot-main")
    ui = transaction.create_enclave("ui", "ui-main")
    sim = transaction.create_enclave("sim", "sim-west")
    return authority, transaction, pilot, ui, sim


def test_identity_validation_and_canonical_enclave_path() -> None:
    identity = EnclaveIdentity("lab_a", "sim", "sim-west-2")

    assert identity.enclave == "/elesim/lab_a/sim/sim_west_2"
    assert identity.relative_enclave == Path("elesim/lab_a/sim/sim_west_2")

    with pytest.raises(ValueError, match="system_id"):
        EnclaveIdentity("../lab", "sim", "sim-west")
    with pytest.raises(ValueError, match="role"):
        EnclaveIdentity("lab_a", "router", "router-main")
    with pytest.raises(ValueError, match="endpoint_id"):
        EnclaveIdentity("lab_a", "ui", "../../ui")


def test_generation_uses_ros2_cli_and_role_policy(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    policy = tmp_path / "pilot.policy.xml"
    policy.write_text("<policy />", encoding="utf-8")
    authority = Sros2Authority(tmp_path / "authority", runner=fake_security_runner)

    generation = authority.begin_generation("lab_a", generation="g-0001")
    identity = generation.create_enclave(
        "pilot", "pilot-main", policy=policy
    )

    assert fake_security_runner.calls == [
        (
            "ros2",
            "security",
            "create_keystore",
            str(tmp_path / "authority/.staging/g-0001/keystore"),
        ),
        (
            "ros2",
            "security",
            "create_enclave",
            str(generation.keystore),
            identity.enclave,
        ),
        (
            "ros2",
            "security",
            "create_permission",
            str(generation.keystore),
            identity.enclave,
            str(policy),
        ),
    ]
    assert (
        generation.keystore
        / "enclaves"
        / identity.relative_enclave
        / "permissions.p7s"
    ).read_text(encoding="utf-8") == "signed:<policy />"


def test_real_sros2_file_links_are_materialized_before_distribution(
    tmp_path: Path,
) -> None:
    runner = SymlinkRos2SecurityRunner()
    authority = Sros2Authority(tmp_path / "authority", runner=runner)
    generation = authority.begin_generation("lab_a", generation="g-links")

    assert not any(path.is_symlink() for path in generation.keystore.rglob("*"))
    identity = generation.create_enclave("pilot", "pilot-main")
    assert not any(path.is_symlink() for path in generation.keystore.rglob("*"))

    bundle = generation.stage_host_bundle("operator-laptop", (identity,))
    assert not any(path.is_symlink() for path in bundle.path.rglob("*"))
    assert (
        bundle.path
        / "roles/pilot/keystore/enclaves"
        / identity.relative_enclave
        / "governance.p7s"
    ).read_text(encoding="utf-8") == "governance"


def test_host_bundle_contains_public_and_only_assigned_enclaves(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    authority, generation, pilot, ui, sim = _issued_generation(
        tmp_path, fake_security_runner
    )

    staged = generation.stage_host_bundle("operator-laptop", (pilot, ui))
    generation.stage_host_bundle("compute-server", (sim,))
    generation.publish()
    exported = generation.export_host_bundle(
        "operator-laptop", tmp_path / "exported/operator-laptop"
    )

    assert staged.manifest.generation == "g-0001"
    assert exported.manifest.enclaves == tuple(sorted((pilot, ui)))
    assert (exported.path / "keystore/public/identity_ca.cert.pem").is_file()
    assert (
        exported.path / "keystore/enclaves" / pilot.relative_enclave / "key.pem"
    ).is_file()
    assert (
        exported.path / "keystore/enclaves" / ui.relative_enclave / "key.pem"
    ).is_file()
    assert (
        exported.path
        / "roles/pilot/keystore/enclaves"
        / pilot.relative_enclave
        / "key.pem"
    ).is_file()
    assert (
        exported.path
        / "roles/ui/keystore/enclaves"
        / ui.relative_enclave
        / "key.pem"
    ).is_file()
    assert not (
        exported.path
        / "roles/pilot/keystore/enclaves"
        / ui.relative_enclave
    ).exists()
    assert (
        exported.path
        / "roles/pilot/keystore/public/identity_ca.cert.pem"
    ).is_file()
    assert not (
        exported.path
        / "keystore/enclaves"
        / sim.relative_enclave
    ).exists()
    assert not (exported.path / "keystore/private").exists()
    assert not (exported.path / "roles/pilot/keystore/private").exists()
    assert (authority.root / "generations/g-0001/keystore/private").is_dir()
    verify_bundle(exported.path)

    for path in (exported.path, *exported.path.rglob("*")):
        expected = 0o700 if path.is_dir() else 0o600
        assert path.stat().st_mode & 0o777 == expected


def test_manifest_detects_tampering_and_unlisted_files(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    _, generation, pilot, _, _ = _issued_generation(
        tmp_path, fake_security_runner
    )
    generation.stage_host_bundle("operator-laptop", (pilot,))
    generation.publish()
    bundle = generation.export_host_bundle(
        "operator-laptop", tmp_path / "exported/operator-laptop"
    ).path
    certificate = (
        bundle / "keystore/enclaves" / pilot.relative_enclave / "cert.pem"
    )
    certificate.write_text("tampered", encoding="utf-8")

    with pytest.raises(SecurityAuthorityError, match="digest mismatch"):
        verify_bundle(bundle)

    certificate.write_text("role-certificate", encoding="utf-8")
    extra = bundle / "keystore/enclaves/unassigned/key.pem"
    extra.parent.mkdir(parents=True, mode=0o700)
    extra.write_text("not-allowed", encoding="utf-8")
    extra.chmod(0o600)
    with pytest.raises(SecurityAuthorityError, match="file set mismatch"):
        verify_bundle(bundle)


def test_bundle_rejects_a_role_view_that_differs_from_its_aggregate_enclave(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    _, generation, pilot, _, _ = _issued_generation(
        tmp_path, fake_security_runner
    )
    generation.stage_host_bundle("operator-laptop", (pilot,))
    generation.publish()
    bundle = generation.export_host_bundle(
        "operator-laptop", tmp_path / "exported/operator-laptop"
    ).path
    role_key = (
        bundle
        / "roles/pilot/keystore/enclaves"
        / pilot.relative_enclave
        / "key.pem"
    )
    role_key.write_text("different-role-key", encoding="utf-8")
    role_key.chmod(0o600)
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    relative = role_key.relative_to(bundle).as_posix()
    manifest["files"][relative] = hashlib.sha256(role_key.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    manifest_path.chmod(0o600)

    with pytest.raises(SecurityAuthorityError, match="does not mirror"):
        verify_bundle(bundle)


def test_enclave_cannot_be_distributed_to_two_hosts(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    _, generation, pilot, _, _ = _issued_generation(
        tmp_path, fake_security_runner
    )
    generation.stage_host_bundle("operator-laptop", (pilot,))

    with pytest.raises(SecurityAuthorityError, match="already assigned"):
        generation.stage_host_bundle("backup-laptop", (pilot,))


def test_canonical_endpoint_collisions_are_rejected(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    authority = Sros2Authority(tmp_path / "authority", runner=fake_security_runner)
    generation = authority.begin_generation("lab_a", generation="g-0001")
    generation.create_enclave("ui", "ui-main")

    with pytest.raises(SecurityAuthorityError, match="collide"):
        generation.create_enclave("ui", "ui_main")


def test_generation_activation_and_rollback_are_atomic_metadata(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    authority, first, pilot, _, _ = _issued_generation(
        tmp_path, fake_security_runner, "g-0001"
    )
    first.stage_host_bundle("operator-laptop", (pilot,))
    first_activation = first.activate()
    assert first_activation.generation == "g-0001"
    assert first_activation.previous_generation is None

    second = authority.begin_generation("lab_a", generation="g-0002")
    next_pilot = second.create_enclave("pilot", "pilot-main")
    second.stage_host_bundle("operator-laptop", (next_pilot,))
    second_activation = second.activate()

    assert second_activation.previous_generation == "g-0001"
    assert authority.active() == second_activation
    rollback = second.rollback()
    assert rollback.action == "rollback"
    assert rollback.generation == "g-0001"
    assert rollback.rolled_back_from == "g-0002"
    assert authority.active() == rollback

    raw = json.loads((authority.root / "active.json").read_text(encoding="utf-8"))
    assert raw["generation"] == "g-0001"


def test_staging_transaction_aborts_without_publishing(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    authority = Sros2Authority(tmp_path / "authority", runner=fake_security_runner)

    with authority.begin_generation("lab_a", generation="g-abort") as generation:
        generation.create_enclave("robot", "robot-go2")
        staging = generation.path

    assert generation.state == "aborted"
    assert not staging.exists()
    assert not (authority.root / "generations/g-abort").exists()


def test_generation_rejects_symlinked_cli_output(tmp_path: Path) -> None:
    outside = tmp_path / "outside.pem"
    outside.write_text("outside", encoding="utf-8")

    def malicious_runner(command: Sequence[str]) -> None:
        keystore = Path(command[3])
        (keystore / "public").mkdir(parents=True)
        (keystore / "private").mkdir()
        (keystore / "enclaves").mkdir()
        (keystore / "public/identity_ca.cert.pem").symlink_to(outside)

    authority = Sros2Authority(tmp_path / "authority", runner=malicious_runner)

    with pytest.raises(SecurityAuthorityError, match="symlink"):
        authority.begin_generation("lab_a", generation="g-malicious")
    assert not (authority.root / ".staging/g-malicious").exists()


def test_symlinked_authority_root_and_policy_are_rejected(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    real_root = tmp_path / "real-authority"
    real_root.mkdir()
    linked_root = tmp_path / "linked-authority"
    linked_root.symlink_to(real_root, target_is_directory=True)

    with pytest.raises(SecurityAuthorityError, match="symlink"):
        Sros2Authority(linked_root, runner=fake_security_runner)

    authority = Sros2Authority(tmp_path / "authority", runner=fake_security_runner)
    generation = authority.begin_generation("lab_a", generation="g-0001")
    real_policy = tmp_path / "policy.xml"
    real_policy.write_text("<policy />", encoding="utf-8")
    linked_policy = tmp_path / "linked-policy.xml"
    linked_policy.symlink_to(real_policy)
    with pytest.raises(SecurityAuthorityError, match="symlink"):
        generation.create_enclave("ui", "ui-main", policy=linked_policy)


def test_bundle_manifest_path_escape_is_rejected(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    _, generation, pilot, _, _ = _issued_generation(
        tmp_path, fake_security_runner
    )
    generation.stage_host_bundle("operator-laptop", (pilot,))
    generation.publish()
    bundle = generation.export_host_bundle(
        "operator-laptop", tmp_path / "exported/operator-laptop"
    ).path
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["files"] = {"../authority/private/key.pem": "0" * 64}
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    manifest_path.chmod(0o600)

    with pytest.raises(SecurityAuthorityError, match="unsafe bundle manifest path"):
        verify_bundle(bundle)


def test_export_is_no_overwrite_and_authority_is_bound_to_one_system(
    tmp_path: Path,
    fake_security_runner: FakeRos2SecurityRunner,
) -> None:
    authority, generation, pilot, _, _ = _issued_generation(
        tmp_path, fake_security_runner
    )
    generation.stage_host_bundle("operator-laptop", (pilot,))
    generation.publish()
    destination = tmp_path / "exported/operator-laptop"
    generation.export_host_bundle("operator-laptop", destination)

    with pytest.raises(FileExistsError, match="already exists"):
        generation.export_host_bundle("operator-laptop", destination)
    with pytest.raises(SecurityAuthorityError, match="belongs to system"):
        authority.begin_generation("other_system", generation="g-other")
