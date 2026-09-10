from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
from pathlib import Path

import yaml

from elesim_setup.configuration import generate_role_configs
from elesim_setup.instance_identity import image_reference, service_key
from elesim_setup.instance_runtime import InstanceRuntime
from elesim_setup.instance_security import stage_instance_security
from elesim_setup.instances import InstanceEndpoint, InstanceState, turn_service_key
from elesim_setup.releases import ReleaseManifest, publish_release, release_key, runtime_data_digest
from elesim_setup.state import ContainerNetworkSettings, DdsSettings, TurnSettings


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def _release(state, tmp_path: Path) -> ReleaseManifest:
    source = tmp_path / "release-data"
    (source / "config").mkdir(parents=True)
    (source / "data").mkdir()
    (source / "data" / "model.bin").write_bytes(b"instance-teardown-scope")
    for role in state.roles:
        shutil.copytree(
            state.prefix_path / "apps" / role / "config",
            source / "config" / role,
        )
    fingerprints = {role: (chr(97 + index) * 64) for index, role in enumerate(state.roles)}
    manifest = ReleaseManifest(
        INSTALL,
        "git-" + "a" * 40,
        "linux/amd64",
        {role: image_reference(INSTALL, role, fingerprints[role]) for role in state.roles},
        {role: "sha256:" + fingerprints[role] for role in state.roles},
        fingerprints,
        runtime_data_digest(source),
    )
    publish_release(state.prefix_path, manifest, source)
    return manifest


def _instance(system: str, release: ReleaseManifest, domain: int, secret: Path) -> InstanceState:
    return InstanceState(
        system,
        release_key(release),
        tuple(InstanceEndpoint(role, f"{system}-{role}") for role in ("pilot", "sim", "ui")),
        domain,
        security_profile="sros2",
        security_generation="g1",
        turn=TurnSettings(
            mode="managed",
            realm=system,
            public_host=f"{system}.example.test",
            secret_file=str(secret),
            listen_port=51001 if system == "lab_a" else 51002,
            relay_min_port=42100 if system == "lab_a" else 42120,
            relay_max_port=42109 if system == "lab_a" else 42129,
        ),
    )


def _security_result(root: Path, instance: InstanceState):
    views = {}
    for endpoint in instance.endpoints:
        view = root / "source" / endpoint.role / "keystore"
        (view / "public").mkdir(parents=True)
        (view / "public" / "identity_ca.cert.pem").write_text("public", encoding="utf-8")
        enclave = view / "enclaves" / "elesim" / instance.system_id / endpoint.role / endpoint.endpoint_id.replace("-", "_")
        enclave.mkdir(parents=True)
        (enclave / "cert.pem").write_text("cert", encoding="utf-8")
        (enclave / "key.pem").write_text("key", encoding="utf-8")
        views[endpoint.role] = view
    return stage_instance_security(
        root / "published",
        INSTALL,
        instance,
        "g1",
        views,
    )


def _fake_docker(fake_bin: Path, call_log: Path) -> None:
    docker = fake_bin / "docker"
    docker.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
printf '%q ' "$@" >> "$ELESIM_FAKE_DOCKER_LOG"
printf '\\n' >> "$ELESIM_FAKE_DOCKER_LOG"
if [[ ${1:-} == info ]]; then
  printf 'engine\\n'
  exit 0
fi
if [[ ${1:-} == container && ${2:-} == inspect ]]; then
  # No containers are pre-existing, so the generated owner guard continues.
  exit 1
fi
if [[ ${1:-} == compose ]]; then
  has_ps=0
  has_status=0
  parsing_ps=0
  skip_status_value=0
  services=()
  for ((index = 2; index <= $#; index++)); do
    value=${!index}
    if (( skip_status_value )); then
      skip_status_value=0
      continue
    fi
    if [[ $value == ps ]]; then
      has_ps=1
      parsing_ps=1
      continue
    fi
    (( parsing_ps )) || continue
    [[ $value == --status ]] && { has_status=1; skip_status_value=1; continue; }
    [[ $value == -aq || $value == --services ]] && continue
    [[ $value == -* ]] && continue
    services+=("$value")
  done
  if (( has_ps )); then
    if (( has_status )); then
      printf '%s\\n' "${services[@]}"
    elif ((${#services[@]})); then
      printf '%s\\n' "${services[0]}"
    fi
  fi
  exit 0
fi
exit 0
""",
        encoding="utf-8",
    )
    docker.chmod(0o755)


def _run_locked(wrapper: Path, lock: Path, env: dict[str, str], *args: str) -> subprocess.CompletedProcess[str]:
    source_fd = os.open(lock, os.O_RDWR | os.O_CREAT)
    saved_fd9 = None
    if source_fd != 9:
        try:
            saved_fd9 = os.dup(9)
        except OSError:
            saved_fd9 = None
        os.dup2(source_fd, 9)
        os.close(source_fd)
    fd = 9
    try:
        return subprocess.run(
            [
                str(wrapper),
                "__elesim_operation_lock_held_v1",
                "9",
                *args,
            ],
            env=env,
            pass_fds=(fd,),
            capture_output=True,
            text=True,
            check=False,
        )
    finally:
        if saved_fd9 is not None:
            os.dup2(saved_fd9, 9)
            os.close(saved_fd9)
        else:
            os.close(fd)


def test_two_robot_free_instances_have_independent_teardown_scope(local_state, tmp_path: Path):
    state = local_state(
        roles=("pilot", "sim", "ui"),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
        container_network=ContainerNetworkSettings(
            mode="direct-host", docker_context="default", docker_engine_id="engine"
        ),
    )
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(
            state.source_path / "payload/config" / role,
            state.prefix_path / "apps" / role / "config",
        )
    generate_role_configs(state)
    release = _release(state, tmp_path)
    secrets = {}
    for system in ("lab_a", "lab_b"):
        secret = tmp_path / f"{system}.secret"
        secret.write_text(f"secret-{system}", encoding="utf-8")
        secrets[system] = secret

    runtime = InstanceRuntime(state, INSTALL)
    instances = {
        system: _instance(system, release, domain, secrets[system])
        for system, domain in (("lab_a", 11), ("lab_b", 12))
    }
    runtime.register(instances["lab_a"], release, security_result=_security_result(tmp_path / "security-a", instances["lab_a"]))
    runtime.register(instances["lab_b"], release, security_result=_security_result(tmp_path / "security-b", instances["lab_b"]))

    aggregate_path = state.prefix_path / "containers" / "compose.instances.yaml"
    aggregate = yaml.safe_load(aggregate_path.read_text(encoding="utf-8"))
    aggregate_services = aggregate["services"]
    services = {
        system: runtime._instance_services(instance)
        for system, instance in instances.items()
    }
    assert set(aggregate_services) == set(services["lab_a"]) | set(services["lab_b"])

    # Config, writable cache, and log roots all carry the system identity and
    # are never shared by the two service groups.
    for endpoint in instances["lab_a"].endpoints + instances["lab_b"].endpoints:
        system = endpoint.endpoint_id.removesuffix(f"-{endpoint.role}")
        config = state.prefix_path / "instances" / system / "endpoints" / endpoint.endpoint_id / "config"
        assert config.is_dir() and any(config.iterdir())
    config_a = state.prefix_path / "instances/lab_a/endpoints/lab_a-pilot/config"
    config_b = state.prefix_path / "instances/lab_b/endpoints/lab_b-pilot/config"
    cache_a = state.prefix_path / "instances/lab_a/endpoints/lab_a-sim/cache"
    cache_b = state.prefix_path / "instances/lab_b/endpoints/lab_b-sim/cache"
    logs_a = state.prefix_path / "instances/lab_a/logs"
    logs_b = state.prefix_path / "instances/lab_b/logs"
    assert config_a != config_b and cache_a != cache_b and logs_a != logs_b
    logs_b.mkdir()
    (logs_b / "keep.log").write_text("lab_b", encoding="utf-8")
    assert str(config_a) in json.dumps(aggregate_services[service_key("lab_a", "lab_a-pilot")])
    assert str(config_b) in json.dumps(aggregate_services[service_key("lab_b", "lab_b-pilot")])
    assert str(cache_a) in json.dumps(aggregate_services[service_key("lab_a", "lab_a-sim")])
    assert str(cache_b) in json.dumps(aggregate_services[service_key("lab_b", "lab_b-sim")])

    for system in ("lab_a", "lab_b"):
        other_keys = set(services["lab_b" if system == "lab_a" else "lab_a"])
        for command in ("up", "down", "logs", "status"):
            wrapper = state.prefix_path / "instances" / system / "bin" / command
            text = wrapper.read_text(encoding="utf-8")
            assert all(key in text for key in services[system])
            assert not any(key in text for key in other_keys)
        assert str(state.prefix_path / "instances" / system / "logs") in (
            state.prefix_path / "instances" / system / "bin" / "logs"
        ).read_text(encoding="utf-8")

    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    call_log = tmp_path / "docker.calls"
    _fake_docker(fake_bin, call_log)
    env = {"PATH": f"{fake_bin}:{os.environ['PATH']}", "ELESIM_FAKE_DOCKER_LOG": str(call_log)}
    lock = state.prefix_path / "instances" / ".locks" / "lab_a.lock"
    wrappers = state.prefix_path / "instances" / "lab_a" / "bin"
    for command, args in (("up", ()), ("logs", ()), ("logs", ("--save",)), ("status", ()), ("down", ())):
        if command in {"up", "down"}:
            result = _run_locked(wrappers / command, lock, env, *args)
        else:
            result = subprocess.run(
                [str(wrappers / command), *args],
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
        assert result.returncode == 0, result.stderr
    calls = [shlex.split(line) for line in call_log.read_text(encoding="utf-8").splitlines()]
    compose_calls = [call for call in calls if call[:1] == ["compose"]]
    assert compose_calls
    assert all(not any(key in " ".join(call) for key in services["lab_b"]) for call in compose_calls)
    assert any("stop" in call and services["lab_a"][-1] in call for call in compose_calls)
    assert any("logs" in call and turn_service_key("lab_a") in call for call in compose_calls)

    verified = []
    runtime.remove(
        "lab_a",
        verifier=lambda compose, project, target: (
            verified.append((compose, project, target))
            or {service: False for service in target}
        ),
    )
    assert verified == [(aggregate_path, runtime.project, services["lab_a"])]
    remaining = yaml.safe_load(aggregate_path.read_text(encoding="utf-8"))["services"]
    assert set(remaining) == set(services["lab_b"])
    assert all((state.prefix_path / "instances/lab_b" / part).exists() for part in ("state.json", "endpoints", "logs"))
