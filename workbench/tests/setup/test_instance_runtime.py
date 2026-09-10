from __future__ import annotations

import hashlib
import json
import shutil
import threading
import time
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from elesim_setup.configuration import generate_role_configs
from elesim_setup.container_installer import ContainerInstaller
from elesim_setup.instance_identity import container_name, image_reference, project_name, service_key
from elesim_setup.instance_runtime import InstanceRuntime
from elesim_setup.instance_security import SecurityAuthorityError, stage_instance_security
from elesim_setup.instances import InstanceEndpoint, InstanceState
from elesim_setup.releases import ReleaseManifest, publish_release, release_key, runtime_data_digest
from elesim_setup.state import ContainerNetworkSettings, DdsSettings


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def _state(factory, **kwargs):
    kwargs.setdefault(
        "container_network",
        ContainerNetworkSettings(
            mode="direct-host", docker_context="default", docker_engine_id="engine"
        ),
    )
    return factory(**kwargs)


def _release(
    state,
    tmp_path: Path,
    *,
    tokens_text: str = "abc",
    large_sentinel: bool = False,
) -> ReleaseManifest:
    source = tmp_path / "release-data"
    (source / "config").mkdir(parents=True)
    (source / "data").mkdir()
    (source / "data" / "model.bin").write_bytes(tokens_text.encode())
    if large_sentinel:
        (source / "data" / "large-sentinel.bin").write_bytes(b"sentinel" * 131072)
    for role in state.roles:
        shutil.copytree(state.prefix_path / "apps" / role / "config", source / "config" / role)
    roles = ("pilot", "sim", "ui")
    tokens = {role: value for role, value in zip(roles, tokens_text)}
    fingerprints = {role: tokens[role] * 64 for role in roles}
    manifest = ReleaseManifest(
        INSTALL,
        "git-" + "a" * 40,
        "linux/amd64",
        {role: image_reference(INSTALL, role, fingerprints[role]) for role in roles},
        {role: "sha256:" + tokens[role] * 64 for role in roles},
        fingerprints,
        runtime_data_digest(source),
    )
    publish_release(state.prefix_path, manifest, source)
    return manifest


def _instance(system: str, release: ReleaseManifest, domain: int) -> InstanceState:
    return InstanceState(
        system,
        release_key(release),
        tuple(InstanceEndpoint(role, f"{system}-{role}") for role in ("pilot", "sim", "ui")),
        domain,
    )


def _write_sidecar_base_compose(state) -> None:
    """Publish the exact install-level sidecar used by scoped instances."""

    installer = ContainerInstaller(state, state_path=state.state_path, dry_run=True)
    installer._install_uuid = INSTALL
    installer._special_container_names["tailscale"] = container_name(INSTALL, "tailscale")
    compose = state.prefix_path / "containers" / "compose.yaml"
    compose.parent.mkdir(parents=True, exist_ok=True)
    compose.write_text(
        yaml.safe_dump(
            {"name": project_name(INSTALL), "services": {"tailscale": installer._tailscale_service()}},
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def test_instances_coexist_and_removal_preserves_other_system(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot", "sim", "ui"))
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    release = _release(state, tmp_path)
    runtime = InstanceRuntime(state, INSTALL)
    runtime.register(_instance("alpha", release, 11))
    compose_bytes = (state.prefix_path / "containers/compose.instances.yaml").read_bytes()
    assert str(state.prefix_path).encode() in compose_bytes and b".instance-" not in compose_bytes
    services = tuple(
        service_key("alpha", f"alpha-{role}") for role in ("pilot", "sim", "ui")
    )
    for wrapper_name in ("up", "down", "logs", "status"):
        wrapper = (state.prefix_path / "instances/alpha/bin" / wrapper_name).read_text()
        assert "expected_docker_context=default" in wrapper
        assert "expected_docker_engine_id=engine" in wrapper
        assert "expected_project=elesim-runtime-0123456789abcdef" in wrapper
        assert "expected_compose=" in wrapper
        for service in services:
            assert service in wrapper
            assert container_name(INSTALL, service) in wrapper
        assert "down --remove-orphans" not in wrapper
    status_wrapper = (state.prefix_path / "instances/alpha/bin/status").read_text()
    for role, service in zip(("pilot", "sim", "ui"), services):
        assert f"{service}) printf '%s\\n' {role}" in status_wrapper
    assert "unexpected instance service" in status_wrapper
    runtime.register(_instance("beta", release, 12))
    beta_before = {
        path.relative_to(state.prefix_path / "instances/beta").as_posix(): path.read_bytes()
        for path in (state.prefix_path / "instances/beta").rglob("*")
        if path.is_file()
    }
    release_before = (state.prefix_path / "releases" / release_key(release) / "manifest.json").read_bytes()
    runtime.remove("alpha", verifier=lambda compose, project, services: {service: False for service in services})
    assert not (state.prefix_path / "instances/alpha").exists()
    assert beta_before == {
        path.relative_to(state.prefix_path / "instances/beta").as_posix(): path.read_bytes()
        for path in (state.prefix_path / "instances/beta").rglob("*")
        if path.is_file()
    }
    assert release_before == (state.prefix_path / "releases" / release_key(release) / "manifest.json").read_bytes()


def test_large_release_data_is_read_only_and_not_staged(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot",))
    state.prefix_path.mkdir(parents=True)
    shutil.copytree(state.source_path / "payload/config/pilot", state.prefix_path / "apps/pilot/config")
    generate_role_configs(state)
    release = _release(state, tmp_path, large_sentinel=True)
    runtime = InstanceRuntime(state, INSTALL)
    instance = InstanceState(
        "alpha", release_key(release), (InstanceEndpoint("pilot", "alpha-pilot"),), 11
    )
    observed: list[Path] = []

    def inject(step: str) -> None:
        if step == "before-commit":
            observed.extend(state.prefix_path.parent.glob(".instance-alpha-*"))
            raise RuntimeError(step)

    with pytest.raises(RuntimeError, match="before-commit"):
        runtime.register(instance, fail=inject)
    assert observed and all(not (stage / "releases").exists() for stage in observed)
    assert (state.prefix_path / "releases" / release_key(release) / "data" / "data" / "large-sentinel.bin").is_file()


def test_other_system_active_security_link_is_not_traversed(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot", "sim", "ui"))
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    release = _release(state, tmp_path)
    runtime = InstanceRuntime(state, INSTALL)
    runtime.register(_instance("beta", release, 12))
    outside = tmp_path / "authority"
    outside.mkdir()
    current = state.prefix_path / "instances/beta/security/current"
    current.parent.mkdir(parents=True)
    current.symlink_to(outside, target_is_directory=True)
    inode = current.lstat().st_ino
    runtime.register(_instance("alpha", release, 11))
    assert current.is_symlink() and current.lstat().st_ino == inode
    assert not any(outside.iterdir())


def test_same_release_replace_preserves_target_security_link(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot", "sim", "ui"))
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    release = _release(state, tmp_path)
    runtime = InstanceRuntime(state, INSTALL)
    original = _instance("alpha", release, 11)
    runtime.register(original)
    outside = tmp_path / "authority"
    outside.mkdir()
    current = state.prefix_path / "instances/alpha/security/current"
    current.parent.mkdir(parents=True)
    current.symlink_to(outside, target_is_directory=True)
    inode = current.lstat().st_ino
    runtime.replace(InstanceState("alpha", release_key(release), original.endpoints, 12))
    assert current.is_symlink() and current.lstat().st_ino == inode


def test_different_release_replace_refuses_active_security(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot", "sim", "ui"))
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    first = _release(state, tmp_path / "first")
    second = _release(state, tmp_path / "second", tokens_text="def")
    runtime = InstanceRuntime(state, INSTALL)
    original = _instance("alpha", first, 11)
    runtime.register(original)
    outside = tmp_path / "authority"
    outside.mkdir()
    current = state.prefix_path / "instances/alpha/security/current"
    current.parent.mkdir(parents=True)
    current.symlink_to(outside, target_is_directory=True)
    state_before = (state.prefix_path / "instances/alpha/state.json").read_bytes()
    changed = InstanceState("alpha", release_key(second), original.endpoints, 12)
    with pytest.raises(ValueError, match="security"):
        runtime.replace(changed)
    assert state_before == (state.prefix_path / "instances/alpha/state.json").read_bytes()
    assert current.is_symlink()


def test_sros2_register_publishes_external_security_result_atomically(local_state, tmp_path: Path):
    state = _state(local_state,
        roles=("pilot", "sim", "ui"),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
    )
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    release = _release(state, tmp_path)
    instance = replace(_instance("alpha", release, 11), security_profile="sros2", security_generation="g1")
    source = tmp_path / "security-source"
    views = {}
    for endpoint in instance.endpoints:
        view = source / endpoint.role / "keystore"
        (view / "public").mkdir(parents=True)
        (view / "public" / "identity_ca.cert.pem").write_text("public", encoding="utf-8")
        enclave = view / "enclaves" / "elesim" / "alpha" / endpoint.role / endpoint.endpoint_id.replace("-", "_")
        enclave.mkdir(parents=True)
        (enclave / "cert.pem").write_text("cert", encoding="utf-8")
        (enclave / "key.pem").write_text("key", encoding="utf-8")
        views[endpoint.role] = view
    result = stage_instance_security(tmp_path / "security-published", INSTALL, instance, "g1", views)
    runtime = InstanceRuntime(state, INSTALL)
    runtime.register(instance, security_result=result)
    current = state.prefix_path / "instances/alpha/security/current"
    assert current.is_symlink() and current.readlink() == Path("generations/g1")
    assert (state.prefix_path / "instances/alpha/endpoints/alpha-pilot/config/runtime.installed.yaml").is_file()


def test_sros2_register_revalidates_external_generation_contents(local_state, tmp_path: Path):
    state = _state(
        local_state,
        roles=("pilot", "sim", "ui"),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
    )
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(
            state.source_path / "payload/config" / role,
            state.prefix_path / "apps" / role / "config",
            dirs_exist_ok=True,
        )
    generate_role_configs(state)
    release = _release(state, tmp_path)
    instance = replace(
        _instance("alpha", release, 11),
        security_profile="sros2",
        security_generation="g1",
    )
    source = tmp_path / "security-source"
    views = {}
    for endpoint in instance.endpoints:
        view = source / endpoint.role / "keystore"
        (view / "public").mkdir(parents=True)
        (view / "public" / "identity_ca.cert.pem").write_text("public")
        enclave = (
            view
            / "enclaves"
            / "elesim"
            / "alpha"
            / endpoint.role
            / endpoint.endpoint_id.replace("-", "_")
        )
        enclave.mkdir(parents=True)
        (enclave / "cert.pem").write_text("cert")
        (enclave / "key.pem").write_text("key")
        views[endpoint.role] = view
    result = stage_instance_security(
        tmp_path / "security-published", INSTALL, instance, "g1", views
    )
    private = result.root / "apps/pilot/keystore/public/ca.key.pem"
    private.write_text("authority-private")
    payload = json.loads(result.manifest.read_text(encoding="utf-8"))
    relative = private.relative_to(result.root).as_posix()
    payload["files"][relative] = hashlib.sha256(private.read_bytes()).hexdigest()
    result.manifest.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(SecurityAuthorityError, match="private material"):
        InstanceRuntime(state, INSTALL).register(instance, security_result=result)
    assert not (state.prefix_path / "instances/alpha/state.json").exists()


def test_same_process_install_lock_serializes_threads(local_state):
    state = _state(local_state, roles=("pilot",))
    state.prefix_path.mkdir(parents=True)
    runtime = InstanceRuntime(state, INSTALL)
    entered = threading.Event()
    release = threading.Event()
    second_entered = threading.Event()

    def first():
        with runtime._lock_file(runtime.install_lock):
            entered.set()
            release.wait(2)

    def second():
        with runtime._lock_file(runtime.install_lock):
            second_entered.set()

    first_thread = threading.Thread(target=first)
    second_thread = threading.Thread(target=second)
    first_thread.start()
    assert entered.wait(1)
    second_thread.start()
    time.sleep(0.05)
    assert not second_entered.is_set()
    release.set()
    first_thread.join(2)
    second_thread.join(2)
    assert second_entered.is_set()


def test_tailscale_sidecar_aggregate_fails_closed(local_state, tmp_path: Path):
    state = _state(local_state,
        roles=("pilot",),
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="engine",
            tailscale_hostname="node",
                tailscale_state_dir=str(tmp_path / "install" / "secrets" / "tailscale"),
        ),
    )
    with pytest.raises(ValueError, match="B5|tailscale-sidecar|Tailscale sidecar"):
        InstanceRuntime(state, INSTALL)


def test_two_instances_share_one_sidecar_and_instance_stop_remove_is_scoped(
    local_state, tmp_path: Path
):
    state = _state(
        local_state,
        roles=("pilot",),
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="engine",
            tailscale_hostname="node",
            tailscale_state_dir=str(tmp_path / "install" / "secrets" / "tailscale"),
        ),
    )
    state.prefix_path.mkdir(parents=True)
    shutil.copytree(state.source_path / "payload/config/pilot", state.prefix_path / "apps/pilot/config")
    generate_role_configs(state)
    release = _release(state, tmp_path)
    _write_sidecar_base_compose(state)

    runtime = InstanceRuntime(state, INSTALL)
    alpha = InstanceState(
        "alpha", release_key(release), (InstanceEndpoint("pilot", "alpha-pilot"),), 11
    )
    beta = InstanceState(
        "beta", release_key(release), (InstanceEndpoint("pilot", "beta-pilot"),), 12
    )
    runtime.register(alpha, release)
    runtime.register(beta, release)
    payload = yaml.safe_load(
        (state.prefix_path / "containers/compose.instances.yaml").read_text(encoding="utf-8")
    )
    services = payload["services"]
    assert tuple(key for key in services if key == "tailscale") == ("tailscale",)
    assert services["tailscale"]["container_name"] == container_name(INSTALL, "tailscale")
    alpha_key = service_key("alpha", "alpha-pilot")
    beta_key = service_key("beta", "beta-pilot")
    assert services[alpha_key]["network_mode"] == "service:tailscale"
    assert services[beta_key]["network_mode"] == "service:tailscale"
    assert services[alpha_key]["depends_on"] == {"tailscale": {"condition": "service_healthy"}}
    assert services[beta_key]["depends_on"] == {"tailscale": {"condition": "service_healthy"}}

    down = (state.prefix_path / "instances/alpha/bin/down").read_text(encoding="utf-8")
    assert f"stop {alpha_key}" in down
    assert f"rm -f -s {alpha_key}" in down
    assert "stop tailscale" not in down
    assert "rm -f -s tailscale" not in down
    assert container_name(INSTALL, "tailscale") in down  # ownership guard only

    checked: list[tuple[str, ...]] = []
    runtime.remove(
        "alpha",
        verifier=lambda compose, project, target: (
            checked.append(target) or {service: False for service in target}
        ),
    )
    assert checked == [(alpha_key,)]
    remaining = yaml.safe_load(
        (state.prefix_path / "containers/compose.instances.yaml").read_text(encoding="utf-8")
    )["services"]
    assert "tailscale" in remaining
    assert beta_key in remaining
    assert alpha_key not in remaining


def test_unpinned_docker_backend_is_rejected(local_state):
    state = local_state(roles=("pilot",))
    with pytest.raises(ValueError, match="pinned Docker context"):
        InstanceRuntime(state, INSTALL)


def test_failed_replace_restores_target_and_aggregate(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot", "sim", "ui"))
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    release = _release(state, tmp_path)
    runtime = InstanceRuntime(state, INSTALL)
    original = _instance("alpha", release, 11)
    runtime.register(original)
    compose_before = (state.prefix_path / "containers/compose.instances.yaml").read_bytes()
    state_before = (state.prefix_path / "instances/alpha/state.json").read_bytes()
    changed = InstanceState("alpha", release_key(release), original.endpoints, 12)
    with pytest.raises(RuntimeError, match="after-compose"):
        runtime.replace(changed, fail=lambda step: (_ for _ in ()).throw(RuntimeError(step)) if step == "after-compose" else None)
    assert compose_before == (state.prefix_path / "containers/compose.instances.yaml").read_bytes()
    assert state_before == (state.prefix_path / "instances/alpha/state.json").read_bytes()


def test_failed_replace_preserves_race_winner_inside_wrapper_tree(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot",))
    state.prefix_path.mkdir(parents=True)
    shutil.copytree(state.source_path / "payload/config/pilot", state.prefix_path / "apps/pilot/config")
    generate_role_configs(state)
    release = _release(state, tmp_path)
    runtime = InstanceRuntime(state, INSTALL)
    original = InstanceState(
        "alpha", release_key(release), (InstanceEndpoint("pilot", "alpha-pilot"),), 11
    )
    runtime.register(original)
    original_up = (state.prefix_path / "instances/alpha/bin/up").read_bytes()

    def inject(step: str) -> None:
        if step == "after-instance":
            (state.prefix_path / "instances/alpha/bin/foreign").write_bytes(b"winner")
            raise RuntimeError(step)

    with pytest.raises(RuntimeError, match="after-instance"):
        runtime.replace(InstanceState("alpha", release_key(release), original.endpoints, 12), fail=inject)
    assert (state.prefix_path / "instances/alpha/bin/foreign").read_bytes() == b"winner"
    assert (state.prefix_path / "instances/alpha/bin/up").read_bytes() == original_up


def test_invalid_release_and_symlink_fail_before_registry_mutation(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot",))
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    missing = InstanceState("alpha", "f" * 64, (InstanceEndpoint("pilot", "alpha-p"),), 1)
    runtime = InstanceRuntime(state, INSTALL)
    with pytest.raises((ValueError, FileNotFoundError)):
        runtime.register(missing)
    instances = state.prefix_path / "instances"
    assert {path.name for path in instances.iterdir()} == {".locks"}
    assert not (instances / "alpha").exists()
    assert not (state.prefix_path / "containers/compose.instances.yaml").exists()
    for lock in (instances / ".locks").iterdir():
        lock.unlink()
    (instances / ".locks").rmdir()
    instances.rmdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (state.prefix_path / "instances").symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink|real directory"):
        runtime.register(missing)
    assert not any(outside.iterdir())


def test_remove_requires_exact_read_only_stop_verifier(local_state, tmp_path: Path):
    state = _state(local_state, roles=("pilot",))
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(state.source_path / "payload/config" / role, state.prefix_path / "apps" / role / "config", dirs_exist_ok=True)
    generate_role_configs(state)
    release = _release(state, tmp_path)
    runtime = InstanceRuntime(state, INSTALL)
    runtime.register(InstanceState("alpha", release_key(release), (InstanceEndpoint("pilot", "alpha-p"),), 1))
    rejected = (
        lambda compose, project, services: True,
        lambda services: {},
        lambda compose, project, services: {},
        lambda compose, project, services: {**{service: False for service in services}, "foreign": False},
        lambda compose, project, services: {service: "stopped" for service in services},
        lambda compose, project, services: {service: True for service in services},
    )
    for verifier in rejected:
        with pytest.raises(PermissionError):
            runtime.remove("alpha", verifier=verifier)
    checked = []
    runtime.remove(
        "alpha",
        verifier=lambda compose, project, services: (
            checked.append(tuple(services))
            or {service: False for service in services}
        ),
    )
    assert len(checked) == 1 and len(checked[0]) == 1 and checked[0][0].startswith("svc-alpha-")


def _instance_security_result(tmp_path: Path, instance: InstanceState, generation: str, marker: str):
    """Create a small validated external generation for rotation tests."""
    source = tmp_path / f"security-{instance.system_id}-{generation}"
    views = {}
    for endpoint in instance.endpoints:
        view = source / endpoint.role / "keystore"
        (view / "public").mkdir(parents=True)
        (view / "public" / "identity_ca.cert.pem").write_text("public")
        enclave = (
            view / "enclaves" / "elesim" / instance.system_id
            / endpoint.role / endpoint.endpoint_id.replace("-", "_")
        )
        enclave.mkdir(parents=True)
        (enclave / "cert.pem").write_text(f"{marker}-cert")
        (enclave / "key.pem").write_text(f"{marker}-key")
        views[endpoint.role] = view
    return stage_instance_security(
        tmp_path / f"published-{instance.system_id}-{generation}",
        INSTALL,
        instance,
        generation,
        views,
    )


def _managed_runtime(local_state, tmp_path: Path):
    state = _state(
        local_state,
        roles=("pilot", "sim", "ui"),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
    )
    state.prefix_path.mkdir(parents=True)
    for role in state.roles:
        shutil.copytree(
            state.source_path / f"payload/config/{role}",
            state.prefix_path / f"apps/{role}/config",
            dirs_exist_ok=True,
        )
    generate_role_configs(state)
    release = _release(state, tmp_path)
    runtime = InstanceRuntime(state, INSTALL)
    def make(system: str, domain: int, generation: str):
        base = replace(_instance(system, release, domain), security_profile="sros2", security_generation=generation)
        return base, _instance_security_result(tmp_path, base, generation, generation)
    return state, runtime, release, make


def test_security_rotation_isolated_per_system(local_state, tmp_path: Path):
    state, runtime, release, make = _managed_runtime(local_state, tmp_path)
    alpha, alpha_first = make("alpha", 11, "g1")
    beta, beta_first = make("beta", 12, "g1")
    runtime.register(alpha, release, security_result=alpha_first)
    runtime.register(beta, release, security_result=beta_first)
    beta_state = (state.prefix_path / "instances/beta/state.json").read_bytes()
    beta_link = (state.prefix_path / "instances/beta/security/current").readlink()

    alpha_next = replace(alpha, security_generation="g2")
    alpha_second = _instance_security_result(tmp_path, alpha_next, "g2", "rotated")
    runtime.rotate_security(alpha_next, alpha_second)

    assert json.loads((state.prefix_path / "instances/alpha/state.json").read_text())["security_generation"] == "g2"
    assert (state.prefix_path / "instances/alpha/security/current").readlink() == Path("generations/g2")
    assert (state.prefix_path / "instances/beta/state.json").read_bytes() == beta_state
    assert (state.prefix_path / "instances/beta/security/current").readlink() == beta_link
    assert (state.prefix_path / "instances/beta/security/generations/g1").is_dir()


def test_failed_security_rotation_restores_prior_state_and_generation(local_state, tmp_path: Path):
    state, runtime, release, make = _managed_runtime(local_state, tmp_path)
    alpha, alpha_first = make("alpha", 11, "g1")
    runtime.register(alpha, release, security_result=alpha_first)
    state_before = (state.prefix_path / "instances/alpha/state.json").read_bytes()
    link_before = (state.prefix_path / "instances/alpha/security/current").readlink()
    alpha_next = replace(alpha, security_generation="g2")
    alpha_second = _instance_security_result(tmp_path, alpha_next, "g2", "failed")

    def fail(step: str) -> None:
        if step == "after-state":
            raise RuntimeError(step)

    with pytest.raises(RuntimeError, match="after-state"):
        runtime.rotate_security(alpha_next, alpha_second, fail=fail)
    assert (state.prefix_path / "instances/alpha/state.json").read_bytes() == state_before
    assert (state.prefix_path / "instances/alpha/security/current").readlink() == link_before
    assert not (state.prefix_path / "instances/alpha/security/generations/g2").exists()
    assert (state.prefix_path / "instances/alpha/security/generations/g1").is_dir()


def test_sros2_replace_retains_previous_generation_for_atomic_restore(
    local_state, tmp_path: Path
) -> None:
    state, runtime, release, make = _managed_runtime(local_state, tmp_path)
    alpha, alpha_first = make("alpha", 11, "g1")
    runtime.register(alpha, release, security_result=alpha_first)

    alpha_next = replace(alpha, domain_id=12, security_generation="g2")
    alpha_second = _instance_security_result(
        tmp_path, alpha_next, "g2", "replacement"
    )
    runtime.replace(alpha_next, release, security_result=alpha_second)

    security = state.prefix_path / "instances/alpha/security"
    assert security.joinpath("current").readlink() == Path("generations/g2")
    assert security.joinpath("generations/g1").is_dir()
    assert security.joinpath("generations/g2").is_dir()
    assert json.loads(
        (state.prefix_path / "instances/alpha/state.json").read_text()
    )["domain_id"] == 12

    # A later-host failure can compensate with the exact prior state and its
    # already retained generation; no Authority-private material is needed.
    runtime.replace(
        alpha,
        release,
        security_result=security / "generations/g1",
    )
    assert security.joinpath("current").readlink() == Path("generations/g1")
    assert security.joinpath("generations/g1").is_dir()
    assert security.joinpath("generations/g2").is_dir()
    assert json.loads(
        (state.prefix_path / "instances/alpha/state.json").read_text()
    )["domain_id"] == 11


def test_failed_sros2_replace_restores_security_state_and_compose(
    local_state, tmp_path: Path
) -> None:
    state, runtime, release, make = _managed_runtime(local_state, tmp_path)
    alpha, alpha_first = make("alpha", 11, "g1")
    runtime.register(alpha, release, security_result=alpha_first)
    state_before = (state.prefix_path / "instances/alpha/state.json").read_bytes()
    compose_before = (state.prefix_path / "containers/compose.instances.yaml").read_bytes()

    alpha_next = replace(alpha, domain_id=12, security_generation="g2")
    alpha_second = _instance_security_result(tmp_path, alpha_next, "g2", "failed")

    def fail(step: str) -> None:
        if step == "after-compose":
            raise RuntimeError(step)

    with pytest.raises(RuntimeError, match="after-compose"):
        runtime.replace(
            alpha_next,
            release,
            security_result=alpha_second,
            fail=fail,
        )

    security = state.prefix_path / "instances/alpha/security"
    assert security.joinpath("current").readlink() == Path("generations/g1")
    assert security.joinpath("generations/g1").is_dir()
    assert not security.joinpath("generations/g2").exists()
    assert (state.prefix_path / "instances/alpha/state.json").read_bytes() == state_before
    assert (state.prefix_path / "containers/compose.instances.yaml").read_bytes() == compose_before
