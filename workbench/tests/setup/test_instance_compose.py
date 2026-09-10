from __future__ import annotations

import copy

import pytest

from elesim_setup.instance_compose import aggregate_compose, render_instance_services
from elesim_setup.instance_identity import image_reference
from elesim_setup.instances import InstanceEndpoint, InstanceState
from elesim_setup.releases import ReleaseManifest, release_key


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"
FP = "b" * 64


def _release() -> ReleaseManifest:
    return ReleaseManifest(INSTALL, "git-" + "a" * 40, "linux/amd64", {"pilot": image_reference(INSTALL, "pilot", FP)}, {"pilot": "sha256:" + "c" * 64}, {"pilot": FP}, "d" * 64)


def _instance() -> InstanceState:
    release = _release()
    return InstanceState("lab_a", release_key(release), (InstanceEndpoint("pilot", "pilot_ep"),), 42)


def test_render_pins_image_and_scopes_service_without_mutating_input() -> None:
    source = {"pilot_ep": {"role": "pilot", "build": {"context": "."}, "environment": {"EXTRA": "yes"}, "volumes": ["/tmp/.X11-unix:/tmp/.X11-unix"]}}
    original = copy.deepcopy(source)
    result = render_instance_services(INSTALL, _instance(), _release(), source)
    key, service = next(iter(result.items()))
    assert key.startswith("svc-lab_a-pilot_ep-")
    assert service["image"] == "sha256:" + "c" * 64
    assert "build" not in service
    assert service["environment"]["ELESIM_SYSTEM_ID"] == "lab_a"
    assert source == original


def test_render_rejects_scope_endpoint_conflict_and_host_ipc() -> None:
    instance = _instance()
    with pytest.raises(ValueError, match="exactly match"):
        render_instance_services(INSTALL, instance, _release(), {})
    with pytest.raises(ValueError, match="conflicting"):
        render_instance_services(INSTALL, instance, _release(), {"pilot_ep": {"role": "pilot", "environment": {"ROS_DOMAIN_ID": "7"}}})
    with pytest.raises(ValueError, match="host IPC"):
        render_instance_services(INSTALL, instance, _release(), {"pilot_ep": {"role": "pilot", "ipc": "host"}})


def test_render_preserves_compose_null_environment_value() -> None:
    rendered = render_instance_services(
        INSTALL,
        _instance(),
        _release(),
        {"pilot_ep": {"role": "pilot", "environment": {"CUDA_VISIBLE_DEVICES": None}}},
    )
    service = next(iter(rendered.values()))
    assert service["environment"]["CUDA_VISIBLE_DEVICES"] is None


def test_aggregate_rejects_unscoped_infrastructure() -> None:
    rendered = render_instance_services(INSTALL, _instance(), _release(), {"pilot_ep": {"role": "pilot"}})
    with pytest.raises(ValueError, match="install-scoped"):
        aggregate_compose(INSTALL, {"one": rendered}, {"turn": {"container_name": "elesim-coturn", "labels": {"io.elesim.install_uuid": INSTALL}}})


def test_aggregate_checks_cross_system_writable_mount_overlap(tmp_path) -> None:
    first = render_instance_services(
        INSTALL, _instance(), _release(),
        {"pilot_ep": {"role": "pilot", "volumes": [f"{tmp_path}/cache:/cache"]}},
    )
    other = InstanceState("lab_b", release_key(_release()), (InstanceEndpoint("pilot", "ep_b"),), 42)
    second = render_instance_services(
        INSTALL, other, _release(),
        {"ep_b": {"role": "pilot", "volumes": [f"{tmp_path}/cache/sub:/cache"]}},
    )
    with pytest.raises(ValueError, match="overlap"):
        aggregate_compose(INSTALL, {"a": first, "b": second}, {})

    readonly = render_instance_services(
        INSTALL, other, _release(),
        {"ep_b": {"role": "pilot", "volumes": [f"{tmp_path}/cache:/cache:ro"]}},
    )
    with pytest.raises(ValueError, match="overlap"):
        aggregate_compose(INSTALL, {"a": first, "b": readonly}, {})
    first_readonly = render_instance_services(
        INSTALL, _instance(), _release(),
        {"pilot_ep": {"role": "pilot", "volumes": [f"{tmp_path}/cache:/cache:ro,z"], "ipc": "private"}},
    )
    assert len(aggregate_compose(INSTALL, {"a": first_readonly, "b": readonly}, {})["services"]) == 2


def test_aggregate_rejects_swapped_labels_foreign_project_and_ipc() -> None:
    rendered = render_instance_services(INSTALL, _instance(), _release(), {"pilot_ep": {"role": "pilot"}})
    key = next(iter(rendered))
    swapped = {key: {**rendered[key], "labels": {**rendered[key]["labels"], "io.elesim.endpoint_id": "other"}}}
    with pytest.raises(ValueError, match="identity"):
        aggregate_compose(INSTALL, {"one": swapped}, {})
    foreign = {key: {**rendered[key], "labels": {**rendered[key]["labels"], "com.docker.compose.project": "elesim-runtime-other"}}}
    with pytest.raises(ValueError, match="foreign"):
        aggregate_compose(INSTALL, {"one": foreign}, {})
    ipc = {key: {**rendered[key], "ipc": "service:foreign"}}
    with pytest.raises(ValueError, match="IPC"):
        aggregate_compose(INSTALL, {"one": ipc}, {})


def test_aggregate_rejects_undeclared_shared_network_and_dependencies() -> None:
    rendered = render_instance_services(
        INSTALL, _instance(), _release(), {"pilot_ep": {"role": "pilot"}}
    )
    key = next(iter(rendered))
    sidecar_without_service = {
        key: {**rendered[key], "network_mode": "service:tailscale"}
    }
    with pytest.raises(ValueError, match="shared tailscale"):
        aggregate_compose(INSTALL, {"one": sidecar_without_service}, {})

    undeclared_dependency = {
        key: {**rendered[key], "depends_on": ["tools"]}
    }
    with pytest.raises(ValueError, match="escapes"):
        aggregate_compose(INSTALL, {"one": undeclared_dependency}, {})


@pytest.mark.parametrize("name", ("tools", "dev", "manager", "coturn"))
def test_aggregate_rejects_install_global_infrastructure(name: str) -> None:
    rendered = render_instance_services(
        INSTALL, _instance(), _release(), {"pilot_ep": {"role": "pilot"}}
    )
    with pytest.raises(ValueError, match="only the exact tailscale"):
        aggregate_compose(
            INSTALL,
            {"one": rendered},
            {
                name: {
                    "container_name": f"elesim-{INSTALL.replace('-', '')}-{name}",
                    "labels": {"io.elesim.install_uuid": INSTALL},
                }
            },
        )
