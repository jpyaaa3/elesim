from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Sequence

import pytest

from elesim_setup.ownership import (
    DOCKER_INSTALL_UUID_LABEL,
    DockerOwnership,
    OwnershipError,
    SystemdUnitOwnership,
    install_host_uninstaller_bundle,
    ownership_install_uuid,
    prepare_ownership_refresh,
    sha256_file,
    write_ownership_manifest,
)
from elesim_setup.shell import managed_path_block
from elesim_setup.uninstall import (
    UninstallSafetyError,
    execute_uninstall,
    main,
    plan_uninstall,
)


def _write(path: Path, value: str = "owned\n", *, executable: bool = False) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")
    if executable:
        path.chmod(0o755)
    return path


def _manifest(
    tmp_path: Path,
    *,
    docker: DockerOwnership | None = None,
    systemd_units: tuple[SystemdUnitOwnership, ...] = (),
):
    prefix = tmp_path / "install"
    bin_dir = tmp_path / "bin"
    prefix.mkdir()
    bin_dir.mkdir()
    containers = prefix / "containers"
    security = prefix / "security"
    static = prefix / "static"
    logs = prefix / "logs"
    authority = prefix / "authority"
    external = security / "operator-external"
    _write(containers / "compose.yaml")
    _write(security / "roles/pilot/key.pem")
    _write(static / "generated.txt")
    _write(logs / "runs/one/pilot.log")
    _write(authority / "private/identity_ca.key.pem")
    _write(external / "do-not-delete.pem")
    _write(prefix / "install-state.json", "{}\n")
    wrappers = (
        _write(bin_dir / "elesim-up", "#!/bin/sh\n", executable=True),
        _write(bin_dir / "elesim-uninstall", "#!/bin/sh\n", executable=True),
    )
    bashrc = _write(
        tmp_path / ".bashrc",
        "export EDITOR=vim\n" + managed_path_block(bin_dir),
    )
    manifest = write_ownership_manifest(
        prefix=prefix,
        bin_dir=bin_dir,
        edition="general",
        inventory_roots=(containers, security, static, prefix / "install-state.json"),
        managed_roots=(containers, security),
        created_roots=(prefix, bin_dir),
        wrapper_paths=wrappers,
        log_roots=(logs,),
        authority_roots=(authority,),
        external_paths=(external,),
        shell_bashrc=bashrc,
        docker=docker,
        systemd_units=systemd_units,
        install_uuid=None if docker is None else docker.install_uuid,
    )
    return manifest, bashrc, external, logs, authority, static


def test_default_uninstall_preserves_logs_authority_external_and_foreign_files(
    tmp_path: Path,
) -> None:
    manifest, bashrc, external, logs, authority, static = _manifest(tmp_path)
    foreign = _write(static / "research-note.txt", "mine\n")
    dynamic_runtime_key = _write(
        Path(manifest.prefix) / "security/roles/ui/post-install.key"
    )

    plan = plan_uninstall(manifest.path)
    tombstone = execute_uninstall(plan, confirm_prefix=manifest.prefix)

    assert tombstone.is_file()
    assert not manifest.path.exists()
    assert external.joinpath("do-not-delete.pem").is_file()
    assert logs.joinpath("runs/one/pilot.log").is_file()
    assert authority.joinpath("private/identity_ca.key.pem").is_file()
    assert foreign.read_text(encoding="utf-8") == "mine\n"
    assert not dynamic_runtime_key.exists()
    assert not static.joinpath("generated.txt").exists()
    assert not (Path(manifest.bin_dir) / "elesim-up").exists()
    assert bashrc.read_text(encoding="utf-8") == "export EDITOR=vim\n"
    payload = json.loads(tombstone.read_text(encoding="utf-8"))
    assert payload["install_uuid"] == manifest.install_uuid
    assert payload["purged_logs"] is False
    assert payload["purged_authority"] is False


def test_explicit_purge_flags_remove_logs_and_authority(tmp_path: Path) -> None:
    manifest, _bashrc, _external, logs, authority, _static = _manifest(tmp_path)

    plan = plan_uninstall(
        manifest.path,
        purge_logs=True,
        purge_authority=True,
    )
    execute_uninstall(plan, confirm_prefix=manifest.prefix)

    assert not logs.exists()
    assert not authority.exists()


def test_changed_wrapper_aborts_before_any_mutation(tmp_path: Path) -> None:
    manifest, bashrc, _external, _logs, _authority, _static = _manifest(tmp_path)
    wrapper = Path(manifest.wrappers[0].path)
    wrapper.write_text("foreign\n", encoding="utf-8")

    with pytest.raises(UninstallSafetyError, match="wrapper가 설치 후 변경"):
        plan_uninstall(manifest.path)

    assert manifest.path.is_file()
    assert wrapper.read_text(encoding="utf-8") == "foreign\n"
    assert "Elesim managed PATH" in bashrc.read_text(encoding="utf-8")


def test_exact_prefix_confirmation_is_required(tmp_path: Path) -> None:
    manifest, *_ = _manifest(tmp_path)
    plan = plan_uninstall(manifest.path)

    with pytest.raises(UninstallSafetyError, match="--confirm-prefix"):
        execute_uninstall(plan, confirm_prefix=str(tmp_path / "other"))

    assert manifest.path.is_file()


def test_shell_change_after_plan_aborts_before_install_files_are_removed(
    tmp_path: Path,
) -> None:
    manifest, bashrc, *_ = _manifest(tmp_path)
    plan = plan_uninstall(manifest.path)
    bashrc.write_text(managed_path_block(Path("/newer/bin")), encoding="utf-8")

    with pytest.raises(UninstallSafetyError, match="상태가 변경"):
        execute_uninstall(plan, confirm_prefix=manifest.prefix)

    assert manifest.path.is_file()
    assert (Path(manifest.prefix) / "containers/compose.yaml").is_file()


def test_relative_prefix_confirmation_is_not_accepted(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest, *_ = _manifest(tmp_path)
    plan = plan_uninstall(manifest.path)
    monkeypatch.chdir(tmp_path)

    with pytest.raises(UninstallSafetyError, match="--confirm-prefix"):
        execute_uninstall(plan, confirm_prefix="install")

    assert manifest.path.is_file()


def test_managed_root_replaced_by_symlink_aborts(tmp_path: Path) -> None:
    manifest, *_ = _manifest(tmp_path)
    security = Path(manifest.prefix) / "security"
    outside = tmp_path / "outside"
    outside.mkdir()
    for path in sorted(security.rglob("*"), reverse=True):
        if path.is_dir():
            path.rmdir()
        else:
            path.unlink()
    security.rmdir()
    security.symlink_to(outside, target_is_directory=True)

    with pytest.raises(UninstallSafetyError, match="유형이 변경|안전한 directory"):
        plan_uninstall(manifest.path)

    assert outside.is_dir()


def test_installed_systemd_unit_fails_with_exact_remediation(tmp_path: Path) -> None:
    destination = tmp_path / "etc/systemd/system/elesim-robot.service"
    destination.parent.mkdir(parents=True)
    destination.write_text("[Unit]\n", encoding="utf-8")
    unit = SystemdUnitOwnership(
        name="elesim-robot.service",
        destination=str(destination),
        sha256=sha256_file(destination),
    )
    manifest, *_ = _manifest(tmp_path, systemd_units=(unit,))

    def runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(
            command,
            0,
            stdout=(
                "LoadState=loaded\n"
                "ActiveState=active\n"
                f"FragmentPath={destination}\n"
            ),
            stderr="",
        )

    with pytest.raises(UninstallSafetyError) as captured:
        plan_uninstall(manifest.path, runner=runner)

    message = str(captured.value)
    assert "sudo systemctl disable --now elesim-robot.service" in message
    assert f"sudo rm -- {destination}" in message
    assert "sudo systemctl daemon-reload" in message


def test_foreign_same_name_systemd_unit_never_suggests_rm(tmp_path: Path) -> None:
    destination = tmp_path / "etc/systemd/system/elesim-robot.service"
    destination.parent.mkdir(parents=True)
    destination.write_text("[Unit]\nDescription=expected\n", encoding="utf-8")
    unit = SystemdUnitOwnership(
        name="elesim-robot.service",
        destination=str(destination),
        sha256=sha256_file(destination),
    )
    manifest, *_ = _manifest(tmp_path, systemd_units=(unit,))
    destination.write_text("[Unit]\nDescription=foreign\n", encoding="utf-8")

    def runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(
            command,
            0,
            stdout=(
                "LoadState=loaded\n"
                "ActiveState=inactive\n"
                f"FragmentPath={destination}\n"
            ),
            stderr="",
        )

    with pytest.raises(UninstallSafetyError) as captured:
        plan_uninstall(manifest.path, runner=runner)

    assert "foreign/변경된" in str(captured.value)
    assert "sudo rm" not in str(captured.value)


def test_nested_bind_mount_aborts_before_mutation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest, *_ = _manifest(tmp_path)
    nested_mount = Path(manifest.prefix) / "security/roles"
    monkeypatch.setattr(
        "elesim_setup.uninstall._mount_points",
        lambda: (Path("/"), nested_mount),
    )

    with pytest.raises(UninstallSafetyError, match="mount/bind mount"):
        plan_uninstall(manifest.path)

    assert manifest.path.is_file()
    assert nested_mount.joinpath("pilot/key.pem").is_file()


class _DockerRunner:
    def __init__(
        self,
        docker: DockerOwnership,
        *,
        foreign: bool = False,
        unlisted: tuple[str, ...] = (),
        inspect_failure: bool = False,
    ) -> None:
        self.docker = docker
        self.foreign = foreign
        self.unlisted = unlisted
        self.inspect_failure = inspect_failure
        self.commands: list[tuple[str, ...]] = []

    def __call__(self, command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        values = tuple(command)
        self.commands.append(values)
        if values[:2] == ("docker", "info"):
            return subprocess.CompletedProcess(values, 0, stdout="26.0\n", stderr="")
        if values[:3] == ("docker", "container", "ls"):
            names = (*self.docker.containers, *self.unlisted)
            return subprocess.CompletedProcess(
                values,
                0,
                stdout="".join(f"{name}\n" for name in names),
                stderr="",
            )
        if values[:3] == ("docker", "container", "inspect"):
            if self.inspect_failure:
                return subprocess.CompletedProcess(
                    values, 1, stdout="", stderr="daemon race"
                )
            project = "foreign-project" if self.foreign else self.docker.project
            payload = [
                {
                    "Id": "sha256:container",
                    "Config": {
                        "Labels": {
                            "com.docker.compose.project": project,
                            "com.docker.compose.project.config_files": self.docker.compose_file,
                            DOCKER_INSTALL_UUID_LABEL: self.docker.install_uuid,
                        }
                    },
                }
            ]
            return subprocess.CompletedProcess(
                values, 0, stdout=json.dumps(payload), stderr=""
            )
        if values[:3] == ("docker", "image", "inspect"):
            payload = [
                {
                    "Id": "sha256:image",
                    "Config": {
                        "Labels": {
                            "com.docker.compose.project": self.docker.project,
                            DOCKER_INSTALL_UUID_LABEL: self.docker.install_uuid,
                        }
                    },
                }
            ]
            return subprocess.CompletedProcess(
                values, 0, stdout=json.dumps(payload), stderr=""
            )
        if values[:3] == ("docker", "image", "ls"):
            return subprocess.CompletedProcess(
                values,
                0,
                stdout="".join(f"{name}\n" for name in self.docker.local_images),
                stderr="",
            )
        if values[:3] in {
            ("docker", "container", "rm"),
            ("docker", "image", "rm"),
        }:
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        raise AssertionError(values)


def test_docker_deletes_only_exact_manifest_objects(tmp_path: Path) -> None:
    compose = tmp_path / "install/containers/compose.yaml"
    docker = DockerOwnership(
        install_uuid="11111111-1111-4111-8111-111111111111",
        compose_file=str(compose),
        project="elesim-runtime",
        containers=("elesim-sim",),
        local_images=("elesim/sim:local",),
    )
    manifest, *_ = _manifest(tmp_path, docker=docker)
    runner = _DockerRunner(docker)

    plan = plan_uninstall(manifest.path, runner=runner)
    execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert (
        "docker",
        "container",
        "rm",
        "--force",
        "sha256:container",
    ) in runner.commands
    assert ("docker", "image", "rm", "elesim/sim:local") in runner.commands
    assert not any("prune" in command for values in runner.commands for command in values)


def test_foreign_fixed_container_name_aborts_before_removal(tmp_path: Path) -> None:
    compose = tmp_path / "install/containers/compose.yaml"
    docker = DockerOwnership(
        install_uuid="22222222-2222-4222-8222-222222222222",
        compose_file=str(compose),
        project="elesim-runtime",
        containers=("elesim-sim",),
        local_images=(),
    )
    manifest, *_ = _manifest(tmp_path, docker=docker)
    runner = _DockerRunner(docker, foreign=True)

    with pytest.raises(UninstallSafetyError, match="다른 설치 소유"):
        plan_uninstall(manifest.path, runner=runner)

    assert not any(values[:3] == ("docker", "container", "rm") for values in runner.commands)


def test_unlisted_same_install_container_aborts_before_removal(tmp_path: Path) -> None:
    compose = tmp_path / "install/containers/compose.yaml"
    docker = DockerOwnership(
        install_uuid="33333333-3333-4333-8333-333333333333",
        compose_file=str(compose),
        project="elesim-runtime",
        containers=("elesim-sim",),
        local_images=(),
    )
    manifest, *_ = _manifest(tmp_path, docker=docker)
    runner = _DockerRunner(docker, unlisted=("elesim-runtime-tools-run-abcd",))

    with pytest.raises(UninstallSafetyError, match="manifest에 없는"):
        plan_uninstall(manifest.path, runner=runner)

    assert not any(values[:3] == ("docker", "container", "rm") for values in runner.commands)


def test_listed_container_inspect_error_is_not_treated_as_absent(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "install/containers/compose.yaml"
    docker = DockerOwnership(
        install_uuid="44444444-4444-4444-8444-444444444444",
        compose_file=str(compose),
        project="elesim-runtime",
        containers=("elesim-sim",),
        local_images=(),
    )
    manifest, *_ = _manifest(tmp_path, docker=docker)
    runner = _DockerRunner(docker, inspect_failure=True)

    with pytest.raises(UninstallSafetyError, match="inspect할 수 없습니다"):
        plan_uninstall(manifest.path, runner=runner)


def test_cli_plan_is_read_only(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    manifest, *_ = _manifest(tmp_path)

    assert main(("--manifest", str(manifest.path), "--plan")) == 0

    assert manifest.path.is_file()
    assert "아직 변경하지 않음" in capsys.readouterr().out


def test_manifest_refresh_keeps_uuid_after_full_preflight(tmp_path: Path) -> None:
    manifest, _bashrc, external, logs, authority, static = _manifest(tmp_path)
    token = prepare_ownership_refresh(
        prefix=Path(manifest.prefix),
        bin_dir=Path(manifest.bin_dir),
        edition="general",
    )
    assert token is not None
    assert ownership_install_uuid(token) == manifest.install_uuid
    wrapper_paths = tuple(Path(value.path) for value in manifest.wrappers)
    wrapper_paths[0].write_text("#!/bin/sh\n# refreshed\n", encoding="utf-8")

    refreshed = write_ownership_manifest(
        prefix=Path(manifest.prefix),
        bin_dir=Path(manifest.bin_dir),
        edition="general",
        inventory_roots=(
            Path(manifest.prefix) / "containers",
            Path(manifest.prefix) / "security",
            static,
            Path(manifest.prefix) / "install-state.json",
        ),
        managed_roots=(
            Path(manifest.prefix) / "containers",
            Path(manifest.prefix) / "security",
        ),
        created_roots=(Path(manifest.prefix), Path(manifest.bin_dir)),
        wrapper_paths=wrapper_paths,
        log_roots=(logs,),
        authority_roots=(authority,),
        external_paths=(external,),
        shell_bashrc=tmp_path / ".bashrc",
        refresh=token,
    )

    assert refreshed.install_uuid == manifest.install_uuid
    assert refreshed.created_at == manifest.created_at
    assert refreshed.created_roots == manifest.created_roots
    assert refreshed.wrappers[0].sha256 != manifest.wrappers[0].sha256


def test_refresh_retains_obsolete_but_still_existing_owned_resources(
    tmp_path: Path,
) -> None:
    install_uuid = "55555555-5555-4555-8555-555555555555"
    compose = tmp_path / "install/containers/compose.yaml"
    old_docker = DockerOwnership(
        install_uuid=install_uuid,
        compose_file=str(compose),
        project="elesim-runtime",
        containers=("elesim-sim",),
        local_images=("elesim/sim:local",),
    )
    manifest, _bashrc, external, logs, authority, static = _manifest(
        tmp_path,
        docker=old_docker,
    )
    token = prepare_ownership_refresh(
        prefix=Path(manifest.prefix),
        bin_dir=Path(manifest.bin_dir),
        edition="general",
    )
    assert token is not None
    new_docker = DockerOwnership(
        install_uuid=install_uuid,
        compose_file=str(compose),
        project="elesim-runtime",
        containers=("elesim-ui",),
        local_images=("elesim/ui:local",),
    )

    refreshed = write_ownership_manifest(
        prefix=Path(manifest.prefix),
        bin_dir=Path(manifest.bin_dir),
        edition="general",
        inventory_roots=(
            Path(manifest.prefix) / "containers",
            Path(manifest.prefix) / "security",
            static,
            Path(manifest.prefix) / "install-state.json",
        ),
        managed_roots=(),
        created_roots=(),
        # Simulate a newer role selection no longer enumerating elesim-up.
        wrapper_paths=(Path(manifest.bin_dir) / "elesim-uninstall",),
        log_roots=(logs,),
        authority_roots=(authority,),
        external_paths=(external,),
        docker=new_docker,
        install_uuid=install_uuid,
        refresh=token,
    )

    assert refreshed.docker is not None
    assert refreshed.docker.containers == ("elesim-sim", "elesim-ui")
    assert refreshed.docker.local_images == (
        "elesim/sim:local",
        "elesim/ui:local",
    )
    assert Path(manifest.bin_dir, "elesim-up") in {
        Path(value.path) for value in refreshed.wrappers
    }
    assert refreshed.created_roots == manifest.created_roots


def test_refresh_rejects_modified_previous_wrapper(tmp_path: Path) -> None:
    manifest, *_ = _manifest(tmp_path)
    Path(manifest.wrappers[0].path).write_text("foreign\n", encoding="utf-8")

    with pytest.raises(OwnershipError, match="기존 wrapper"):
        prepare_ownership_refresh(
            prefix=Path(manifest.prefix),
            bin_dir=Path(manifest.bin_dir),
            edition="general",
        )


def test_new_install_refuses_preexisting_claim_without_manifest(tmp_path: Path) -> None:
    prefix = tmp_path / "install"
    bin_dir = tmp_path / "bin"
    prefix.mkdir()
    bin_dir.mkdir()
    foreign = _write(prefix / "security/research.key", "foreign\n")

    with pytest.raises(OwnershipError, match="자동 인수하지"):
        prepare_ownership_refresh(
            prefix=prefix,
            bin_dir=bin_dir,
            edition="general",
            claimed_paths=(prefix / "security",),
        )

    assert foreign.read_text(encoding="utf-8") == "foreign\n"


def test_host_bundle_runs_plan_without_container_or_installed_package(
    tmp_path: Path,
) -> None:
    prefix = tmp_path / "host-install"
    bin_dir = tmp_path / "host-bin"
    prefix.mkdir()
    bin_dir.mkdir()
    generated = _write(prefix / "install-state.json", "{}\n")
    bundle = install_host_uninstaller_bundle(prefix=prefix, bin_dir=bin_dir)
    manifest = write_ownership_manifest(
        prefix=prefix,
        bin_dir=bin_dir,
        edition="general",
        inventory_roots=(generated, bundle.root),
        managed_roots=(),
        created_roots=(prefix, bin_dir),
        wrapper_paths=(bundle.wrapper,),
    )

    result = subprocess.run(
        (str(bundle.wrapper), "--plan"),
        check=False,
        capture_output=True,
        text=True,
        env={"PATH": "/usr/bin:/bin"},
    )

    assert result.returncode == 0, result.stderr
    assert manifest.install_uuid in result.stdout
    assert "아직 변경하지 않음" in result.stdout
    assert not bundle.root.joinpath("elesim_setup/__pycache__").exists()

    removed = subprocess.run(
        (str(bundle.wrapper), "--confirm-prefix", manifest.prefix),
        check=False,
        capture_output=True,
        text=True,
        env={"PATH": "/usr/bin:/bin"},
    )

    assert removed.returncode == 0, removed.stderr
    assert not bundle.root.exists()
    assert not bundle.wrapper.exists()


def test_host_bundle_accepts_safe_developer_generated_root(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    bin_dir = workspace / "bin"
    workspace.mkdir()
    bin_dir.mkdir()
    destination = workspace / ".elesim/development/maintenance"

    bundle = install_host_uninstaller_bundle(
        prefix=workspace,
        bin_dir=bin_dir,
        bundle_root=destination,
    )

    assert bundle.root == destination
    assert bundle.wrapper.is_file()


def test_developer_manifest_and_tombstone_stay_out_of_workspace_root(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace"
    bin_dir = workspace / "bin"
    generated = workspace / ".elesim/development"
    manifest_path = generated / "install-ownership.json"
    workspace.mkdir()
    bin_dir.mkdir()
    generated.mkdir(parents=True)
    bundle = install_host_uninstaller_bundle(
        prefix=workspace,
        bin_dir=bin_dir,
        manifest_path=manifest_path,
        bundle_root=generated / "maintenance",
    )
    manifest = write_ownership_manifest(
        prefix=workspace,
        bin_dir=bin_dir,
        edition="developer",
        inventory_roots=(bundle.root,),
        managed_roots=(generated,),
        created_roots=(),
        wrapper_paths=(bundle.wrapper,),
        manifest_path=manifest_path,
    )

    plan = plan_uninstall(manifest.path)
    tombstone = execute_uninstall(plan, confirm_prefix=manifest.prefix)

    assert tombstone.parent == generated
    assert tombstone.is_file()
    assert not workspace.joinpath("install-ownership.json").exists()
    assert not any(workspace.glob("uninstall-tombstone-*.json"))


def test_host_bundle_rejects_outside_or_symlinked_root(tmp_path: Path) -> None:
    prefix = tmp_path / "install"
    bin_dir = tmp_path / "bin"
    prefix.mkdir()
    bin_dir.mkdir()
    with pytest.raises(OwnershipError, match="prefix 하위"):
        install_host_uninstaller_bundle(
            prefix=prefix,
            bin_dir=bin_dir,
            bundle_root=tmp_path / "outside",
        )

    outside = tmp_path / "outside"
    outside.mkdir()
    link = prefix / "maintenance-link"
    link.symlink_to(outside, target_is_directory=True)
    with pytest.raises(OwnershipError, match="symlink|안전한 directory"):
        install_host_uninstaller_bundle(
            prefix=prefix,
            bin_dir=bin_dir,
            bundle_root=link,
        )
