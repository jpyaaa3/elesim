from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import shutil

import pytest
import yaml

from elesim_setup.configuration import generate_role_configs
from elesim_setup.instance_compose import aggregate_compose
from elesim_setup.instance_identity import image_reference
from elesim_setup.instance_preparation import prepare_instance_services
from elesim_setup.instance_security import activate_instance_security, stage_instance_security
from elesim_setup.instances import InstanceEndpoint, InstanceState
from elesim_setup.releases import (
    ReleaseManifest,
    publish_release,
    release_key,
    runtime_data_digest,
)
from elesim_setup.state import ComputeSettings, DdsSettings, InstallState, NetworkSettings


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def _state(tmp_path: Path) -> InstallState:
    state = InstallState(profile="custom", roles=("pilot", "sim", "ui"),
                         prefix=str(tmp_path / "prefix"), bin_dir=str(tmp_path / "bin"),
                         source_root=str(tmp_path), network=NetworkSettings(), dds=DdsSettings())
    for role, filename in (("pilot", "runtime.yaml"), ("sim", "runtime.yaml"), ("ui", "default.yaml")):
        config = state.prefix_path / "apps" / role / "config"
        config.mkdir(parents=True, exist_ok=True)
        (config / filename).write_text(yaml.safe_dump({"runtime": {"role": role}}))
        if role == "sim":
            (config / "config.yaml").write_text("simulation: {}\n")
    generate_role_configs(state)
    return state


def _release(state: InstallState, source: Path) -> ReleaseManifest:
    release_data = source.parent / "release-data"
    (release_data / "data").mkdir(parents=True)
    (release_data / "config").mkdir()
    for item in source.iterdir():
        shutil.copy2(item, release_data / "data" / item.name)
    for role in state.roles:
        shutil.copytree(
            state.prefix_path / "apps" / role / "config",
            release_data / "config" / role,
        )
    digest = runtime_data_digest(release_data)
    tokens = {"pilot": "b", "sim": "c", "ui": "d"}
    fingerprints = {role: (tokens[role] * 64) for role in tokens}
    images = {role: image_reference(INSTALL, role, fingerprints[role]) for role in fingerprints}
    value = ReleaseManifest(INSTALL, "git-" + "a" * 40, "linux/amd64", images,
                            {role: "sha256:" + tokens[role] * 64 for role in fingerprints},
                            fingerprints, digest)
    key = release_key(value)
    publish_release(state.prefix_path, value, release_data)
    return value


def _instance(system: str, release: ReleaseManifest) -> InstanceState:
    return InstanceState(system, release_key(release), (InstanceEndpoint("pilot", system + "-p"), InstanceEndpoint("sim", system + "-s"), InstanceEndpoint("ui", system + "-u")), 20 if system == "alpha" else 21)


def test_prepare_two_systems_isolated_and_pinned(tmp_path: Path):
    state = _state(tmp_path)
    legacy = (state.prefix_path / "apps/pilot/config/runtime.installed.yaml").read_bytes()
    source = tmp_path / "data"; source.mkdir(); (source / "model.bin").write_bytes(b"model")
    release = _release(state, source)
    legacy_compose = state.prefix_path / "containers/compose.yaml"
    legacy_compose.parent.mkdir(parents=True)
    legacy_compose.write_bytes(b"legacy-compose\n")
    groups = []
    for name in ("alpha", "beta"):
        groups.append(prepare_instance_services(state, INSTALL, _instance(name, release), release))
    assert (state.prefix_path / "apps/pilot/config/runtime.installed.yaml").read_bytes() == legacy
    assert legacy_compose.read_bytes() == b"legacy-compose\n"
    alpha_config = state.prefix_path / "instances/alpha/endpoints/alpha-p/config/runtime.installed.yaml"
    alpha_before = alpha_config.read_bytes()
    combined = aggregate_compose(INSTALL, {"alpha": groups[0], "beta": groups[1]}, {})
    assert len(combined["services"]) == 6
    for group in groups:
        for service in group.values():
            assert str(service["image"]).startswith("sha256:")
            assert any("/data:ro" in str(volume) for volume in service["volumes"])
            assert service["labels"]["io.elesim.system_id"] in {"alpha", "beta"}
    assert alpha_config.read_bytes() == alpha_before
    alpha_sim = next(
        service for service in groups[0].values()
        if service["labels"]["io.elesim.role"] == "sim"
    )
    beta_sim = next(
        service for service in groups[1].values()
        if service["labels"]["io.elesim.role"] == "sim"
    )
    alpha_writable = {
        str(v).split(":", 1)[0]
        for v in alpha_sim["volumes"]
        if str(v).endswith(":rw") and not str(v).startswith("/tmp/.X11-unix:")
    }
    beta_writable = {
        str(v).split(":", 1)[0]
        for v in beta_sim["volumes"]
        if str(v).endswith(":rw") and not str(v).startswith("/tmp/.X11-unix:")
    }
    assert alpha_writable.isdisjoint(beta_writable)


def test_prepare_applies_per_instance_compute_policy_to_pilot_and_sim(tmp_path: Path):
    state = _state(tmp_path)
    source = tmp_path / "data"; source.mkdir(); (source / "model.bin").write_bytes(b"model")
    release = _release(state, source)
    instance = replace(
        _instance("alpha", release),
        compute=ComputeSettings(gpu_mode="specific", gpu_device="GPU-abc123"),
    )
    services = prepare_instance_services(state, INSTALL, instance, release)
    by_role = {service["labels"]["io.elesim.role"]: service for service in services.values()}
    for role in ("pilot", "sim"):
        service = by_role[role]
        assert service["deploy"]["resources"]["reservations"]["devices"][0]["device_ids"] == ("GPU-abc123",)
        assert "CUDA_VISIBLE_DEVICES" not in service["environment"]

    cpu_instance = replace(
        instance,
        system_id="beta",
        domain_id=21,
        compute=ComputeSettings(gpu_mode="cpu"),
    )
    cpu_services = prepare_instance_services(state, INSTALL, cpu_instance, release)
    cpu_roles = {
        service["labels"]["io.elesim.role"]: service
        for service in cpu_services.values()
    }
    assert cpu_roles["pilot"]["environment"]["CUDA_VISIBLE_DEVICES"] == ""
    assert cpu_roles["sim"]["environment"]["CUDA_VISIBLE_DEVICES"] == ""


def test_bad_release_fails_before_instance_tree(tmp_path: Path):
    state = _state(tmp_path)
    source = tmp_path / "data"; source.mkdir(); (source / "x").write_text("x")
    release = _release(state, source)
    value = _instance("alpha", release)
    published = state.prefix_path / "releases" / release_key(release)
    (published / "manifest.json").unlink()
    with pytest.raises((FileNotFoundError, ValueError)):
        prepare_instance_services(state, INSTALL, value, release)
    assert not (state.prefix_path / "instances").exists()


def _security_views(tmp_path: Path, instance: InstanceState) -> dict[str, Path]:
    views: dict[str, Path] = {}
    for endpoint in instance.endpoints:
        view = tmp_path / "security-source" / endpoint.role / "keystore"
        (view / "public").mkdir(parents=True, exist_ok=True)
        (view / "public/identity_ca.cert.pem").write_text("public", encoding="utf-8")
        endpoint_key = endpoint.endpoint_id.replace("-", "_")[:63]
        enclave = view / "enclaves/elesim" / instance.system_id / endpoint.role / endpoint_key
        enclave.mkdir(parents=True, exist_ok=True)
        (enclave / "cert.pem").write_text(endpoint.role, encoding="utf-8")
        (enclave / "key.pem").write_text(endpoint.endpoint_id, encoding="utf-8")
        views[endpoint.role] = view
    return views


def test_instance_uses_pinned_release_data_and_managed_sros2_view(
    tmp_path: Path,
) -> None:
    state = _state(tmp_path)
    source = tmp_path / "data"
    source.mkdir()
    (source / "model.bin").write_bytes(b"model")
    release = _release(state, source)
    instance = replace(_instance("alpha", release), security_profile="sros2", security_generation="g1")
    managed = replace(
        state,
        dds=replace(
            state.dds,
            security_profile="sros2",
            security_provisioning="managed",
            security_generation="g1",
            security_bundle=str(
                state.prefix_path
                / "instances/alpha/security/generations/g1/apps/pilot/keystore"
            ),
            keystore=str(
                state.prefix_path
                / "instances/alpha/security/generations/g1/apps/pilot/keystore"
            ),
            enclave="/elesim/alpha/pilot/alpha_p",
        ),
    )
    views = _security_views(tmp_path, instance)
    stage_instance_security(state.prefix_path, INSTALL, instance, "g1", views)
    activate_instance_security(state.prefix_path, INSTALL, instance, "g1")
    services = prepare_instance_services(managed, INSTALL, instance, release)
    pilot = next(
        service
        for service in services.values()
        if service["environment"]["ELESIM_DDS_ENCLAVE"]
        == "/elesim/alpha/pilot/alpha_p"
    )
    environment = pilot["environment"]
    keystore = (
        state.prefix_path
        / "instances/alpha/security/generations/g1/apps/pilot/keystore"
    )
    assert environment["ROS_SECURITY_KEYSTORE"] == str(keystore)
    assert environment["ELESIM_DDS_ENCLAVE"] == "/elesim/alpha/pilot/alpha_p"
    assert f"{keystore}:{keystore}:ro" in pilot["volumes"]
    assert any(
        f"{state.prefix_path}/releases/{release_key(release)}/data/data:/opt/elesim/data:ro"
        == volume
        for volume in pilot["volumes"]
    )
    generated = (
        state.prefix_path / "instances/alpha/endpoints/alpha-p/config/runtime.installed.yaml"
    )
    before = generated.read_bytes()
    (state.prefix_path / "apps/pilot/config/runtime.yaml").write_text(
        "runtime:\n  role: changed\n", encoding="utf-8"
    )
    prepare_instance_services(managed, INSTALL, instance, release)
    assert generated.read_bytes() == before


def test_missing_active_sros2_generation_fails_before_instance_mutation(
    tmp_path: Path,
) -> None:
    state = _state(tmp_path)
    source = tmp_path / "data"
    source.mkdir()
    (source / "model.bin").write_bytes(b"model")
    release = _release(state, source)
    instance = replace(_instance("alpha", release), security_profile="sros2")
    managed = replace(
        state,
        dds=replace(
            state.dds,
            security_profile="sros2",
            security_provisioning="managed",
            security_generation="g1",
            security_bundle=str(state.prefix_path / "security/current"),
            keystore=str(state.prefix_path / "security/current"),
            enclave="/elesim/alpha/pilot/alpha_p",
        ),
    )
    with pytest.raises((FileNotFoundError, ValueError)):
        prepare_instance_services(managed, INSTALL, instance, release)
    assert not (state.prefix_path / "instances").exists()
