from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

from elesim_setup.manager_lifecycle import manager_lifecycle_fragment
from elesim_setup.container_installer import (
    SCOPED_MANAGER_PORT_BASE,
    SCOPED_MANAGER_PORT_SPAN,
    _manager_wrapper,
    _runtime_down_wrapper,
    _runtime_logs_wrapper,
)


def _scoped_manager_port_probe(
    wrapper: str, system: str, *args: str
) -> tuple[int, list[str]]:
    """Run the generated port-selection portion without starting Docker."""

    start = wrapper.index("manager_port_digest=")
    end = wrapper.index('manager_args+=(--host')
    probe = (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "manager_system=$1\n"
        "shift\n"
        "manager_args=(\"$@\")\n"
        + wrapper[start:end]
        + 'printf \'%s\\n\' "$manager_port"\n'
        + 'printf \'%s\\n\' "${manager_args[@]}"\n'
    )
    result = subprocess.run(
        ["bash", "-c", probe, "manager-port-probe", system, *args],
        text=True,
        capture_output=True,
        check=True,
    )
    lines = result.stdout.splitlines()
    return int(lines[0]), lines[1:]


def _run_fragment(tmp_path: Path, *, owner: str, present: bool, manager: str = "sys-manager", invocation: str = "expected-token", launch_failed: bool = False):
    docker = tmp_path / "docker"
    calls = tmp_path / "calls"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "set -u\n"
        "printf '%s\\n' \"$*\" >>\"$FAKE_CALLS\"\n"
        "if [[ ${1:-} == ps ]]; then\n"
        "  [[ ${FAKE_PRESENT:-0} == 1 ]] && printf 'manager-id\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ ${1:-} == inspect ]]; then\n"
        "  format=${3:-}\n"
        "  if [[ $format == *manager_invocation* ]]; then printf '%s|%s|%s|%s\\n' manager-id false \"$FAKE_OWNER\" \"${FAKE_INVOCATION:-foreign-token}\"; exit 0; fi\n"
        "  if [[ $format == *State.Running* ]]; then printf 'false\\n'; exit 0; fi\n"
        "  if [[ $format == *install_uuid* ]]; then printf '%s\\n' \"$FAKE_OWNER\"; exit 0; fi\n"
        "fi\n"
        "if [[ ${1:-} == rm ]]; then printf 'rm\\n' >>\"$FAKE_RM\"; exit 0; fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    script = tmp_path / "run.sh"
    script.write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\n"
        + manager_lifecycle_fragment("expected", container_name=manager)
        + "manager_invocation_token=expected-token\nmanager_started=1\n"
        + ("exit 125\n" if launch_failed else ""),
        encoding="utf-8",
    )
    script.chmod(0o755)
    env = dict(os.environ)
    env.update(
        PATH=f"{tmp_path}:{env.get('PATH', '')}",
        FAKE_CALLS=str(calls),
        FAKE_RM=str(tmp_path / "rm"),
        FAKE_OWNER=owner,
        FAKE_PRESENT="1" if present else "0",
        FAKE_INVOCATION=invocation,
    )
    result = subprocess.run([str(script)], env=env, text=True, capture_output=True)
    return result, calls, tmp_path / "rm"


def test_stopped_foreign_manager_is_preserved(tmp_path: Path) -> None:
    result, calls, rm = _run_fragment(tmp_path, owner="foreign", present=True)
    assert result.returncode == 73
    assert not rm.exists()
    assert "sys-manager" in calls.read_text(encoding="utf-8")


def test_owned_manager_cleanup_is_exact_and_forceful(tmp_path: Path) -> None:
    result, calls, rm = _run_fragment(tmp_path, owner="expected", present=False)
    assert result.returncode == 0
    assert rm.read_text(encoding="utf-8").splitlines() == ["rm"]
    assert "rm -f manager-id" in calls.read_text().splitlines()


def test_failed_concurrent_launch_preserves_same_install_other_invocation(tmp_path: Path) -> None:
    # The first invocation acquires the name after the second has checked ps.
    result, calls, rm = _run_fragment(
        tmp_path, owner="expected", present=False,
        invocation="first-invocation", launch_failed=True,
    )
    assert result.returncode == 125
    assert not rm.exists()
    assert not any(line.startswith("rm ") for line in calls.read_text().splitlines())


def test_cleanup_is_invocation_and_id_scoped() -> None:
    fragment = manager_lifecycle_fragment("expected", container_name="sys-manager")
    assert "{{.Id}}|{{.State.Running}}" in fragment
    assert "{{index .Config.Labels \"io.elesim.manager_invocation\"}}" in fragment
    assert 'docker rm -f "$manager_id"' in fragment
    assert "manager_invocation_token" in fragment


def test_complete_manager_wrapper_has_valid_shell_syntax(tmp_path: Path) -> None:
    wrapper = _manager_wrapper(
        compose=tmp_path / "compose.yaml",
        compose_wrapper=tmp_path / "elesim-compose",
        state_path=tmp_path / "topology.json",
        authority_root=tmp_path / "authority",
        local_install_root=tmp_path / "install",
        local_bin_dir=tmp_path / "bin",
        maintenance_root=tmp_path / "maintenance",
        install_uuid="01234567-89ab-cdef-0123-456789abcdef",
        guard="",
        container_network_mode="direct-host",
        gpu_mode="cpu",
        gpu_device="",
    )
    result = subprocess.run(
        ["bash", "-n"], input=wrapper, text=True, capture_output=True
    )
    assert result.returncode == 0, result.stderr


def test_scoped_manager_wrapper_requires_system_and_derives_private_workspace(
    tmp_path: Path,
) -> None:
    wrapper = _manager_wrapper(
        compose=tmp_path / "compose.yaml",
        compose_wrapper=tmp_path / "elesim-compose",
        state_path=tmp_path / "unused-topology.json",
        authority_root=tmp_path / "authority",
        local_install_root=tmp_path / "install",
        local_bin_dir=tmp_path / "bin",
        maintenance_root=tmp_path / "maintenance",
        install_uuid="01234567-89ab-cdef-0123-456789abcdef",
        guard="",
        container_network_mode="direct-host",
        gpu_mode="cpu",
        gpu_device="",
        project="elesim-runtime-0123456789abcdef0123456789abcdef",
        manager_container="unused-static-name",
        scoped_systems=True,
    )
    assert subprocess.run(
        ["bash", "-n"], input=wrapper, text=True, capture_output=True
    ).returncode == 0
    assert "--expected-system-id \"$manager_system\"" in wrapper
    assert "/connections/$manager_system/topology.json" in wrapper
    assert "manager_container=elesim-0123456789abcdef0123456789abcdef-manager-$manager_system" in wrapper
    assert "elesim_setup.manager_ownership" in wrapper
    assert "--manifest" in wrapper
    assert wrapper.index("elesim_setup.manager_ownership") < wrapper.index(
        "manager_started=1"
    )
    assert 'container_name_variable="manager_container"' not in wrapper


def test_scoped_manager_port_is_install_system_scoped_and_explicit_wins(
    tmp_path: Path,
) -> None:
    def render(install_uuid: str) -> str:
        return _manager_wrapper(
            compose=tmp_path / "compose.yaml",
            compose_wrapper=tmp_path / "elesim-compose",
            state_path=tmp_path / "topology.json",
            authority_root=tmp_path / "authority",
            local_install_root=tmp_path / "install",
            local_bin_dir=tmp_path / "bin",
            maintenance_root=tmp_path / "maintenance",
            install_uuid=install_uuid,
            guard="",
            container_network_mode="direct-host",
            gpu_mode="cpu",
            gpu_device="",
            project="elesim-runtime-scoped",
            scoped_systems=True,
        )

    first_uuid = "01234567-89ab-cdef-0123-456789abcdef"
    second_uuid = "fedcba98-7654-3210-fedc-ba9876543210"
    first = render(first_uuid)
    second = render(second_uuid)

    first_alpha, _ = _scoped_manager_port_probe(first, "alpha")
    first_beta, _ = _scoped_manager_port_probe(first, "beta")
    second_alpha, _ = _scoped_manager_port_probe(second, "alpha")
    assert SCOPED_MANAGER_PORT_BASE <= first_alpha < (
        SCOPED_MANAGER_PORT_BASE + SCOPED_MANAGER_PORT_SPAN
    )
    assert first_alpha != first_beta
    assert first_alpha != second_alpha
    assert '"127.0.0.1:${manager_port}:${manager_port}"' in first
    assert 'manager_args+=(--port "$manager_port")' in first

    explicit, forwarded = _scoped_manager_port_probe(
        first, "alpha", "--port", "12345", "--host", "0.0.0.0"
    )
    assert explicit == 12345
    assert forwarded == ["--host", "0.0.0.0", "--port", "12345"]

    legacy = _manager_wrapper(
        compose=tmp_path / "compose.yaml",
        compose_wrapper=tmp_path / "elesim-compose",
        state_path=tmp_path / "topology.json",
        authority_root=tmp_path / "authority",
        local_install_root=tmp_path / "install",
        local_bin_dir=tmp_path / "bin",
        maintenance_root=tmp_path / "maintenance",
        install_uuid=first_uuid,
        guard="",
        container_network_mode="direct-host",
        gpu_mode="cpu",
        gpu_device="",
    )
    assert "manager_port=8766\n" in legacy
    assert 'manager_args+=(--port "$manager_port")' in legacy


@pytest.mark.parametrize("name", ["", "bad name", "x; rm -rf /", "a" * 129])
def test_manager_container_name_rejects_shell_injection(name: str) -> None:
    with pytest.raises(ValueError):
        manager_lifecycle_fragment("expected", container_name=name)


def test_instance_scoped_down_uses_exact_services(tmp_path: Path) -> None:
    docker = tmp_path / "docker"
    calls = tmp_path / "calls"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >>\"$FAKE_CALLS\"\n"
        "if [[ ${1:-} == compose ]]; then printf 'role-id\\n'; fi\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    wrapper = tmp_path / "down"
    wrapper.write_text(
        _runtime_down_wrapper(
            compose=Path("/tmp/compose.yaml"),
            logs_root=tmp_path / "logs",
            services=("pilot",),
            archive_enabled=False,
            guard="",
            project="system-a",
            instance_scoped=True,
            manager_container="system-a-manager",
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}:{os.environ.get('PATH', '')}", FAKE_CALLS=str(calls))
    result = subprocess.run([str(wrapper), "--purge"], env=env, text=True, capture_output=True)
    assert result.returncode == 0
    output = calls.read_text(encoding="utf-8")
    assert "stop pilot" in output
    assert "rm -f -s pilot" in output
    assert "down" not in output
    assert "remove-orphans" not in output
    assert "system-a-manager" not in output


def test_logs_follow_names_explicit_services() -> None:
    wrapper = _runtime_logs_wrapper(
        compose=Path("/tmp/compose.yaml"),
        logs_root=Path("/tmp/logs"),
        services=("pilot", "sim"),
        archive_enabled=False,
        guard="",
        project="system-a",
    )
    assert "logs -f pilot sim" in wrapper


def test_scoped_down_still_stops_when_archive_fails(tmp_path: Path) -> None:
    docker = tmp_path / "docker"
    calls = tmp_path / "calls"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >>\"$FAKE_CALLS\"\n"
        "if [[ ${1:-} == compose ]]; then printf 'role-id\\n'; fi\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    real_logs = tmp_path / "real-logs"
    real_logs.mkdir()
    logs_link = tmp_path / "logs"
    logs_link.symlink_to(real_logs, target_is_directory=True)
    wrapper = tmp_path / "down"
    wrapper.write_text(
        _runtime_down_wrapper(
            compose=Path("/tmp/compose.yaml"),
            logs_root=logs_link,
            services=("pilot",),
            archive_enabled=True,
            guard="",
            project="system-a",
            instance_scoped=True,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}:{os.environ.get('PATH', '')}", FAKE_CALLS=str(calls))
    result = subprocess.run([str(wrapper)], env=env, text=True, capture_output=True)
    assert result.returncode != 0
    output = calls.read_text(encoding="utf-8")
    assert "stop pilot" in output
