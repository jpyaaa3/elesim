from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Callable, Sequence

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


@pytest.fixture(autouse=True)
def _isolated_uninstall_state(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))


@pytest.mark.parametrize(
    ("context", "engine_id"),
    (("default", ""), ("", "engine-id")),
)
def test_docker_ownership_pin_is_all_or_nothing(
    tmp_path: Path,
    context: str,
    engine_id: str,
) -> None:
    ownership = DockerOwnership(
        install_uuid="11111111-1111-4111-8111-111111111111",
        compose_file=str(tmp_path / "compose.yaml"),
        project="elesim-runtime",
        containers=(),
        local_images=(),
        context=context,
        engine_id=engine_id,
    )

    with pytest.raises(OwnershipError, match="함께"):
        ownership.validate()


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
    viewer_cleanup: bool = False,
    viewer_state: bool = False,
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
    wrapper_paths = [
        _write(bin_dir / "elesim-up", "#!/bin/sh\n", executable=True),
        _write(bin_dir / "elesim-uninstall", "#!/bin/sh\n", executable=True),
    ]
    cache = prefix / "cache"
    runtime_cache = prefix / ".runtime-cache"
    if viewer_cleanup or viewer_state:
        cache.mkdir()
        runtime_cache.mkdir()
    if viewer_cleanup:
        wrapper_paths.append(
            _write(
                bin_dir / "elesim-viewer-cleanup",
                "#!/bin/sh\nexit 0\n",
                executable=True,
            )
        )
    if viewer_state:
        _write(cache / "viewer-xhost", ":0\n\n")
    bashrc = _write(
        tmp_path / ".bashrc",
        "export EDITOR=vim\n" + managed_path_block(bin_dir),
    )
    manifest = write_ownership_manifest(
        prefix=prefix,
        bin_dir=bin_dir,
        edition="general",
        inventory_roots=(
            containers,
            security,
            static,
            prefix / "install-state.json",
            *((cache, runtime_cache) if viewer_cleanup or viewer_state else ()),
        ),
        managed_roots=(
            containers,
            security,
            *((cache, runtime_cache) if viewer_cleanup or viewer_state else ()),
        ),
        created_roots=(prefix, bin_dir),
        wrapper_paths=wrapper_paths,
        log_roots=(logs,),
        authority_roots=(authority,),
        external_paths=(external,),
        shell_bashrc=bashrc,
        docker=docker,
        systemd_units=systemd_units,
        install_uuid=None if docker is None else docker.install_uuid,
    )
    return manifest, bashrc, external, logs, authority, static


def test_default_uninstall_removes_logs_authority_but_preserves_external_and_foreign(
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
    assert not logs.exists()
    assert not authority.exists()
    assert foreign.read_text(encoding="utf-8") == "mine\n"
    assert not dynamic_runtime_key.exists()
    assert not static.joinpath("generated.txt").exists()
    assert not (Path(manifest.bin_dir) / "elesim-up").exists()
    assert bashrc.read_text(encoding="utf-8") == "export EDITOR=vim\n"
    payload = json.loads(tombstone.read_text(encoding="utf-8"))
    assert payload["install_uuid"] == manifest.install_uuid
    assert payload["purged_logs"] is True
    assert payload["purged_authority"] is True


def test_explicit_keep_flags_preserve_logs_and_authority(tmp_path: Path) -> None:
    manifest, _bashrc, _external, logs, authority, _static = _manifest(tmp_path)

    plan = plan_uninstall(
        manifest.path,
        purge_logs=False,
        purge_authority=False,
    )
    execute_uninstall(plan, confirm_prefix=manifest.prefix)

    assert logs.is_dir()
    assert authority.is_dir()


def test_changed_wrapper_aborts_before_any_mutation(tmp_path: Path) -> None:
    manifest, bashrc, _external, _logs, _authority, _static = _manifest(tmp_path)
    wrapper = Path(manifest.wrappers[0].path)
    wrapper.write_text("foreign\n", encoding="utf-8")

    with pytest.raises(UninstallSafetyError, match="wrapper가 설치 후 변경"):
        plan_uninstall(manifest.path)

    assert manifest.path.is_file()
    assert wrapper.read_text(encoding="utf-8") == "foreign\n"
    assert "Elesim managed PATH" in bashrc.read_text(encoding="utf-8")


def test_uninstall_runs_exact_owned_viewer_cleanup_before_any_other_mutation(
    tmp_path: Path,
) -> None:
    manifest, bashrc, *_ = _manifest(
        tmp_path,
        viewer_cleanup=True,
        viewer_state=True,
    )
    cleanup = Path(manifest.bin_dir) / "elesim-viewer-cleanup"
    state = Path(manifest.prefix) / "cache/viewer-xhost"
    commands: list[tuple[str, ...]] = []

    def runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        values = tuple(command)
        commands.append(values)
        assert values == (str(cleanup),)
        assert manifest.path.is_file()
        assert state.is_file()
        assert (Path(manifest.prefix) / "install-state.json").is_file()
        assert "Elesim managed PATH" in bashrc.read_text(encoding="utf-8")
        return subprocess.CompletedProcess(values, 0, stdout="", stderr="")

    plan = plan_uninstall(manifest.path, runner=runner)
    assert plan.viewer_cleanup == cleanup
    execute_uninstall(
        plan,
        confirm_prefix=manifest.prefix,
        runner=runner,
    )

    assert commands == [(str(cleanup),)]
    assert not manifest.path.exists()


def test_uninstall_stops_exact_sim_container_before_viewer_cleanup(
    tmp_path: Path,
) -> None:
    docker = DockerOwnership(
        install_uuid="77777777-7777-4777-8777-777777777777",
        compose_file=str(tmp_path / "install/containers/compose.yaml"),
        project="elesim-runtime",
        containers=("elesim-sim",),
        local_images=(),
    )
    manifest, *_ = _manifest(
        tmp_path,
        docker=docker,
        viewer_cleanup=True,
        viewer_state=True,
    )
    cleanup = Path(manifest.bin_dir) / "elesim-viewer-cleanup"
    docker_runner = _DockerRunner(docker)
    events: list[tuple[str, ...]] = []

    def runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        values = tuple(command)
        events.append(values)
        if values == (str(cleanup),):
            assert docker_runner.commands[-1][:3] == (
                "docker",
                "container",
                "stop",
            )
            assert (Path(manifest.prefix) / "cache/viewer-xhost").is_file()
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        return docker_runner(values)

    plan = plan_uninstall(manifest.path, runner=runner)
    execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    mutation_events = [
        values
        for values in events
        if values == (str(cleanup),)
        or values[:3]
        in {
            ("docker", "container", "stop"),
            ("docker", "container", "rm"),
        }
    ]
    assert mutation_events[0][:3] == ("docker", "container", "stop")
    assert mutation_events[1] == (str(cleanup),)
    assert mutation_events[2][:3] == ("docker", "container", "rm")


def test_viewer_cleanup_failure_aborts_before_uninstall_mutation(tmp_path: Path) -> None:
    manifest, bashrc, *_ = _manifest(
        tmp_path,
        viewer_cleanup=True,
        viewer_state=True,
    )
    cleanup = Path(manifest.bin_dir) / "elesim-viewer-cleanup"

    def runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        values = tuple(command)
        assert values == (str(cleanup),)
        return subprocess.CompletedProcess(
            values,
            74,
            stdout="",
            stderr="cannot revoke ACL",
        )

    plan = plan_uninstall(manifest.path, runner=runner)
    with pytest.raises(UninstallSafetyError, match="X11 Viewer ACL 회수 실패"):
        execute_uninstall(
            plan,
            confirm_prefix=manifest.prefix,
            runner=runner,
        )

    assert manifest.path.is_file()
    assert cleanup.is_file()
    assert (Path(manifest.prefix) / "cache/viewer-xhost").is_file()
    assert (Path(manifest.prefix) / "install-state.json").is_file()
    assert "Elesim managed PATH" in bashrc.read_text(encoding="utf-8")


def test_viewer_state_without_owned_exact_cleanup_fails_closed(tmp_path: Path) -> None:
    manifest, bashrc, *_ = _manifest(tmp_path, viewer_state=True)
    foreign = _write(
        Path(manifest.bin_dir) / "elesim-viewer-cleanup",
        "#!/bin/sh\nexit 0\n",
        executable=True,
    )

    with pytest.raises(UninstallSafetyError, match="ownership manifest에 없습니다"):
        plan_uninstall(manifest.path)

    assert manifest.path.is_file()
    assert foreign.is_file()
    assert "Elesim managed PATH" in bashrc.read_text(encoding="utf-8")


def test_missing_owned_viewer_cleanup_wrapper_fails_closed(tmp_path: Path) -> None:
    manifest, *_ = _manifest(tmp_path, viewer_cleanup=True)
    cleanup = Path(manifest.bin_dir) / "elesim-viewer-cleanup"
    cleanup.unlink()

    with pytest.raises(UninstallSafetyError, match="cleanup wrapper가 없습니다"):
        plan_uninstall(manifest.path)

    assert manifest.path.is_file()


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
            ("docker", "container", "stop"),
            ("docker", "container", "rm"),
            ("docker", "image", "rm"),
        }:
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        raise AssertionError(values)


class _TailscaleDockerRunner:
    image_digest = "sha256:" + "b" * 64
    image_id = "sha256:" + "c" * 64
    container_id = "d" * 64
    image_ref = "tailscale/tailscale:stable"

    def __init__(
        self,
        docker: DockerOwnership,
        state_path: Path,
        *,
        running: bool = True,
        present: bool = True,
        mount_source: Path | None = None,
        mounted_state_path: Path | None = None,
        helper_failure: bool = False,
        helper_failure_message: str = "refusing non-regular Tailscale state",
        after_helper: Callable[[], None] | None = None,
        remove_failure: bool = False,
        resume_failure: bool = False,
        start_failure: bool = False,
    ) -> None:
        self.docker = docker
        self.state_path = state_path
        self.running = running
        self.present = present
        self.mount_source = state_path if mount_source is None else mount_source
        self.mounted_state_path = (
            state_path if mounted_state_path is None else mounted_state_path
        )
        self.helper_failure = helper_failure
        self.helper_failure_message = helper_failure_message
        self.after_helper = after_helper
        self.remove_failure = remove_failure
        self.resume_failure = resume_failure
        self.start_failure = start_failure
        self.pid_stopped = False
        self.commands: list[tuple[str, ...]] = []

    def __call__(self, command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        values = tuple(command)
        self.commands.append(values)
        if values[:2] == ("docker", "info"):
            return subprocess.CompletedProcess(values, 0, stdout="26.0\n", stderr="")
        if values[:3] == ("docker", "container", "ls"):
            names = "elesim-tailscale\n" if self.present else ""
            return subprocess.CompletedProcess(values, 0, stdout=names, stderr="")
        if values[:3] == ("docker", "container", "inspect"):
            payload = [
                {
                    "Id": self.container_id,
                    "Image": self.image_id,
                    "State": {"Running": self.running},
                    "Mounts": [
                        {
                            "Type": "bind",
                            "Source": str(self.mount_source),
                            "Destination": "/var/lib/tailscale",
                            "RW": True,
                        }
                    ],
                    "Config": {
                        "Image": self.image_ref,
                        "User": "",
                        "Entrypoint": ["tailscaled"],
                        "Cmd": [
                            "--statedir=/var/lib/tailscale",
                            "--socket=/tmp/tailscaled.sock",
                            "--tun=tailscale0",
                        ],
                        "Labels": {
                            "com.docker.compose.project": self.docker.project,
                            "com.docker.compose.project.config_files": self.docker.compose_file,
                            "com.docker.compose.service": "tailscale",
                            DOCKER_INSTALL_UUID_LABEL: self.docker.install_uuid,
                        },
                    },
                }
            ]
            return subprocess.CompletedProcess(
                values, 0, stdout=json.dumps(payload), stderr=""
            )
        if values[:3] == ("docker", "image", "inspect"):
            payload = [
                {
                    "Id": self.image_id,
                    "RepoDigests": [f"tailscale/tailscale@{self.image_digest}"],
                }
            ]
            return subprocess.CompletedProcess(
                values, 0, stdout=json.dumps(payload), stderr=""
            )
        if values[:3] == ("docker", "image", "ls"):
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        if values[:3] == ("docker", "container", "exec"):
            sentinel_name, sentinel_value = values[-4:-2]
            sentinel = self.mounted_state_path / sentinel_name
            if (
                not sentinel.is_file()
                or sentinel.read_text(encoding="ascii") != sentinel_value
            ):
                return subprocess.CompletedProcess(
                    values,
                    70,
                    stdout="",
                    stderr="Tailscale state mount identity mismatch",
                )
            if self.helper_failure:
                return subprocess.CompletedProcess(
                    values,
                    70,
                    stdout="",
                    stderr=self.helper_failure_message,
                )
            self.pid_stopped = True
            for path in (
                self.mounted_state_path,
                *self.mounted_state_path.rglob("*"),
            ):
                if path.is_symlink():
                    continue
                path.chmod(0o700 if path.is_dir() else 0o600)
            if self.after_helper is not None:
                self.after_helper()
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        if values[:3] == ("docker", "container", "kill"):
            if self.resume_failure:
                return subprocess.CompletedProcess(
                    values, 1, stdout="", stderr="injected resume failure"
                )
            self.pid_stopped = False
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        if values[:3] == ("docker", "container", "start"):
            if self.start_failure:
                return subprocess.CompletedProcess(
                    values, 1, stdout="", stderr="injected start failure"
                )
            self.present = True
            self.running = True
            self.pid_stopped = False
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        if values[:3] == ("docker", "container", "rm"):
            if self.remove_failure:
                return subprocess.CompletedProcess(
                    values, 1, stdout="", stderr="injected remove failure"
                )
            self.present = False
            self.running = False
            self.pid_stopped = False
            return subprocess.CompletedProcess(values, 0, stdout="", stderr="")
        raise AssertionError(values)


def _tailscale_manifest(tmp_path: Path):
    prefix = tmp_path / "install"
    bin_dir = tmp_path / "bin"
    compose = prefix / "containers/compose.yaml"
    state_path = prefix / "secrets/tailscale"
    prefix.mkdir()
    bin_dir.mkdir()
    _write(compose)
    _write(state_path / "files/profile.json", "state\n")
    wrapper = _write(bin_dir / "elesim-uninstall", "#!/bin/sh\n", executable=True)
    bashrc = _write(tmp_path / ".bashrc", managed_path_block(bin_dir))
    docker = DockerOwnership(
        install_uuid="66666666-6666-4666-8666-666666666666",
        compose_file=str(compose),
        project="elesim-runtime",
        containers=("elesim-tailscale",),
        local_images=(),
    )
    manifest = write_ownership_manifest(
        prefix=prefix,
        bin_dir=bin_dir,
        edition="general",
        inventory_roots=(prefix / "containers",),
        managed_roots=(prefix / "containers", prefix / "secrets"),
        created_roots=(prefix, bin_dir),
        wrapper_paths=(wrapper,),
        shell_bashrc=bashrc,
        docker=docker,
        install_uuid=docker.install_uuid,
    )
    return manifest, docker, state_path


def test_uninstall_normalizes_exact_owned_tailscale_bind_before_removal(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    state_path.joinpath("files/profile.json").chmod(0)
    state_path.joinpath("files").chmod(0)
    runner = _TailscaleDockerRunner(docker, state_path, running=True)

    plan = plan_uninstall(manifest.path, runner=runner)
    execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    helpers = [
        values
        for values in runner.commands
        if values[:3] == ("docker", "container", "exec")
    ]
    assert len(helpers) == 1
    helper = helpers[0]
    assert helper[helper.index("--user") + 1] == "0:0"
    assert runner.container_id in helper
    script = next(
        value for value in helper if value.startswith("state=/var/lib/tailscale")
    )
    assert 'if ! test -d "$state" || test -L "$state"' in script
    assert "kill -STOP 1" in script
    assert "-type f -links +1" in script
    assert "chmod u+rwx" in script
    assert "chmod u+rw" in script
    assert not any(
        values[:3] == ("docker", "container", "stop")
        for values in runner.commands
    )
    assert not runner.present
    assert not state_path.parent.exists()


def test_uninstall_accepts_legacy_digest_pinned_tailscale_sidecar(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    runner = _TailscaleDockerRunner(docker, state_path, running=True)
    runner.image_ref = f"tailscale/tailscale:v1.98.9@{runner.image_digest}"

    plan = plan_uninstall(manifest.path, runner=runner)

    assert plan.tailscale_state_cleanup is not None


def test_uninstall_rejects_non_official_rolling_tailscale_image(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    runner = _TailscaleDockerRunner(docker, state_path, running=True)
    runner.image_ref = "example.invalid/tailscale:stable"

    with pytest.raises(UninstallSafetyError, match="official image"):
        plan_uninstall(manifest.path, runner=runner)


def test_stopped_tailscale_sidecar_needs_no_helper_for_host_removable_state(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    runner = _TailscaleDockerRunner(docker, state_path, running=False)

    plan = plan_uninstall(manifest.path, runner=runner)
    execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert not any(
        values[:3]
        in {
            ("docker", "container", "exec"),
            ("docker", "container", "stop"),
        }
        for values in runner.commands
    )
    assert not state_path.parent.exists()


def test_tailscale_cleanup_rejects_foreign_bind_before_mutation(tmp_path: Path) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    outside = tmp_path / "foreign-state"
    outside.mkdir()
    runner = _TailscaleDockerRunner(docker, state_path, mount_source=outside)

    with pytest.raises(UninstallSafetyError, match="exact 경계와 다릅니다"):
        plan_uninstall(manifest.path, runner=runner)

    assert manifest.path.is_file()
    assert not any(
        values[:3]
        in {
            ("docker", "container", "stop"),
            ("docker", "container", "exec"),
            ("docker", "container", "rm"),
        }
        for values in runner.commands
    )


def test_tailscale_cleanup_rejects_nested_symlink_and_restores_running_sidecar(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    outside = _write(tmp_path / "outside/keep.txt", "keep\n")
    state_path.joinpath("escape").symlink_to(outside)
    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        helper_failure=True,
    )
    plan = plan_uninstall(manifest.path, runner=runner)

    with pytest.raises(UninstallSafetyError, match="ownership 복구 실패"):
        execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert outside.read_text(encoding="utf-8") == "keep\n"
    assert manifest.path.is_file()
    assert any(
        values[:3] == ("docker", "container", "kill")
        for values in runner.commands
    )
    assert not any(
        values[:3] == ("docker", "container", "rm")
        for values in runner.commands
    )


def test_tailscale_cleanup_rejects_hardlink_and_restores_running_sidecar(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    outside = _write(tmp_path / "outside/keep.txt", "keep\n")
    hardlink = state_path / "outside-hardlink"
    hardlink.hardlink_to(outside)
    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        helper_failure=True,
        helper_failure_message="refusing hard-linked Tailscale state",
    )
    plan = plan_uninstall(manifest.path, runner=runner)

    with pytest.raises(UninstallSafetyError, match="ownership 복구 실패"):
        execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert outside.read_text(encoding="utf-8") == "keep\n"
    assert hardlink.is_file()
    assert any(
        values[:3] == ("docker", "container", "kill")
        for values in runner.commands
    )
    assert not any(
        values[:3] == ("docker", "container", "rm")
        for values in runner.commands
    )


def test_missing_owned_tailscale_state_needs_no_helper(tmp_path: Path) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    state_path.joinpath("files/profile.json").unlink()
    state_path.joinpath("files").rmdir()
    state_path.rmdir()
    runner = _TailscaleDockerRunner(docker, state_path, running=True)

    plan = plan_uninstall(manifest.path, runner=runner)
    execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert not any(
        values[:3] in {
            ("docker", "container", "stop"),
            ("docker", "container", "exec"),
        }
        for values in runner.commands
    )
    assert not state_path.parent.exists()


def test_inaccessible_tailscale_state_without_owned_sidecar_fails_closed(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    inaccessible = state_path / "files"
    inaccessible.chmod(0)
    runner = _TailscaleDockerRunner(docker, state_path, present=False)
    try:
        with pytest.raises(UninstallSafetyError, match="ownership repair"):
            plan_uninstall(manifest.path, runner=runner)
    finally:
        inaccessible.chmod(0o700)

    assert manifest.path.is_file()
    assert not any(
        values[:3] == ("docker", "container", "exec")
        for values in runner.commands
    )


def test_tailscale_cleanup_rejects_host_path_swap_after_existing_mount_helper(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    outside = _write(tmp_path / "outside/keep.txt", "keep\n")
    detached = state_path.with_name("tailscale-detached")

    def swap_path() -> None:
        state_path.rename(detached)
        state_path.symlink_to(outside.parent, target_is_directory=True)

    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        after_helper=swap_path,
    )
    plan = plan_uninstall(manifest.path, runner=runner)

    with pytest.raises(UninstallSafetyError, match="inode.*변경"):
        execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert outside.read_text(encoding="utf-8") == "keep\n"
    assert manifest.path.is_file()
    assert any(
        values[:3] == ("docker", "container", "kill")
        for values in runner.commands
    )


def test_tailscale_cleanup_rejects_stale_container_bind_inode(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    mounted = state_path.with_name("tailscale-mounted")
    state_path.rename(mounted)
    _write(state_path / "files/profile.json", "replacement\n")
    original_mode = mounted.joinpath("files/profile.json").stat().st_mode
    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        mounted_state_path=mounted,
    )
    plan = plan_uninstall(manifest.path, runner=runner)

    with pytest.raises(UninstallSafetyError, match="mount identity mismatch"):
        execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert mounted.joinpath("files/profile.json").stat().st_mode == original_mode
    assert manifest.path.is_file()
    assert not any(
        values[:3] == ("docker", "container", "rm")
        for values in runner.commands
    )


def test_tailscale_container_remove_failure_resumes_quiesced_sidecar(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        remove_failure=True,
    )
    plan = plan_uninstall(manifest.path, runner=runner)

    with pytest.raises(UninstallSafetyError, match="sidecar 제거 실패"):
        execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert runner.present
    assert runner.running
    assert not runner.pid_stopped
    assert manifest.path.is_file()
    assert any(
        values[:3] == ("docker", "container", "kill")
        for values in runner.commands
    )


def test_tailscale_container_remove_failure_starts_sidecar_when_resume_fails(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        remove_failure=True,
        resume_failure=True,
    )
    plan = plan_uninstall(manifest.path, runner=runner)

    with pytest.raises(UninstallSafetyError, match="sidecar 제거 실패"):
        execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert runner.present
    assert runner.running
    assert not runner.pid_stopped
    assert manifest.path.is_file()
    assert any(
        values[:3] == ("docker", "container", "start")
        for values in runner.commands
    )


def test_tailscale_container_remove_failure_reports_resume_and_start_failure(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        remove_failure=True,
        resume_failure=True,
        start_failure=True,
    )
    plan = plan_uninstall(manifest.path, runner=runner)

    with pytest.raises(
        UninstallSafetyError,
        match="sidecar resume/start도 실패: injected start failure",
    ):
        execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)

    assert runner.present
    assert runner.pid_stopped
    assert manifest.path.is_file()


def test_tailscale_cleanup_resumes_sidecar_when_host_postcheck_fails(
    tmp_path: Path,
) -> None:
    manifest, docker, state_path = _tailscale_manifest(tmp_path)
    child = state_path / "files"

    runner = _TailscaleDockerRunner(
        docker,
        state_path,
        running=True,
        after_helper=lambda: child.chmod(0),
    )
    plan = plan_uninstall(manifest.path, runner=runner)
    try:
        with pytest.raises(UninstallSafetyError, match="안전하게 제거"):
            execute_uninstall(plan, confirm_prefix=manifest.prefix, runner=runner)
    finally:
        child.chmod(0o700)

    assert manifest.path.is_file()
    assert any(
        values[:3] == ("docker", "container", "kill")
        for values in runner.commands
    )
    assert not any(
        values[:3] == ("docker", "container", "rm")
        for values in runner.commands
    )


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


def test_cli_uninstall_validates_then_executes(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    manifest, *_ = _manifest(tmp_path)

    assert main(("--manifest", str(manifest.path),)) == 0

    assert not manifest.path.exists()
    assert "EleSim 제거 완료" in capsys.readouterr().out


def test_cli_rejects_removed_plan_option(tmp_path: Path) -> None:
    manifest, *_ = _manifest(tmp_path)

    with pytest.raises(SystemExit):
        main(("--manifest", str(manifest.path), "--plan"))

    assert manifest.path.is_file()


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


def test_host_bundle_uninstalls_without_container_or_installed_package(
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

    removed = subprocess.run(
        (str(bundle.wrapper),),
        check=False,
        capture_output=True,
        text=True,
        env={
            "PATH": "/usr/bin:/bin",
            "XDG_STATE_HOME": str(tmp_path / "standalone-state"),
        },
    )

    assert removed.returncode == 0, removed.stderr
    assert "EleSim 제거 완료" in removed.stdout
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

    assert tombstone.parent == tmp_path / "state/elesim/uninstall"
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
