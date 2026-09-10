"""Pure rendering of immutable, install-scoped instance Compose services."""

from __future__ import annotations

from copy import deepcopy
import os
from typing import Mapping

from .instance_identity import container_name, project_name, service_key
from .instances import InstanceState
from .releases import ReleaseManifest, release_key


_ROLES = frozenset(("pilot", "sim", "ui"))
_INSTALL_LABEL = "io.elesim.install_uuid"
_TAILSCALE_IMAGE = "tailscale/tailscale:stable"


def _env(service: dict[str, object]) -> dict[str, str | None]:
    raw = service.get("environment", {})
    if isinstance(raw, Mapping):
        result = dict(raw)
    elif isinstance(raw, list):
        result = {}
        for item in raw:
            if not isinstance(item, str) or "=" not in item:
                raise ValueError("service environment must be a mapping or KEY=value list")
            key, value = item.split("=", 1)
            result[key] = value
    else:
        raise ValueError("service environment must be a mapping or KEY=value list")
    if any(
        not isinstance(key, str)
        or (value is not None and not isinstance(value, str))
        for key, value in result.items()
    ):
        raise ValueError("service environment keys must be strings and values strings or null")
    return result


def _volume_source(value: object) -> str | None:
    if isinstance(value, str):
        source = value.split(":", 1)[0]
        return source if source.startswith("/") else None
    if isinstance(value, Mapping) and value.get("type") == "bind":
        source = value.get("source")
        return source if isinstance(source, str) else None
    return None


def _runtime_volume(value: object) -> tuple[str, bool] | None:
    if isinstance(value, str):
        parts = value.split(":")
        if len(parts) == 1:
            return None
        if not parts[0].startswith("/"):
            raise ValueError("runtime volumes must use absolute host paths")
        return _writable_volume(value)
    if isinstance(value, Mapping):
        if value.get("type") != "bind":
            raise ValueError("runtime named volumes are not isolated")
        source = value.get("source")
        if not isinstance(source, str) or not source.startswith("/"):
            raise ValueError("runtime bind sources must be absolute host paths")
        return _writable_volume(value)
    raise ValueError("runtime volume entry is invalid")


def _overlaps(first: str, second: str) -> bool:
    left, right = os.path.normpath(first), os.path.normpath(second)
    return left == right or left.startswith(right.rstrip("/") + "/") or right.startswith(left.rstrip("/") + "/")


def _validate_runtime_identity(install_uuid: str, key: str, service: Mapping[str, object], seen: set[tuple[str, str]]) -> None:
    labels = service.get("labels")
    if not isinstance(labels, Mapping):
        raise ValueError("runtime service labels are required")
    install, system, endpoint, role = (labels.get(name) for name in (_INSTALL_LABEL, "io.elesim.system_id", "io.elesim.endpoint_id", "io.elesim.role"))
    if not all(isinstance(value, str) for value in (install, system, endpoint, role)):
        raise ValueError("runtime service identity labels are incomplete")
    if install != install_uuid or role not in _ROLES or key != service_key(system, endpoint):
        raise ValueError("runtime service identity does not match its key")
    if endpoint == "coturn":
        if role != "sim" or labels.get("io.elesim.service_kind") != "coturn":
            raise ValueError("scoped Coturn service identity is incomplete")
    elif labels.get("io.elesim.service_kind") is not None:
        raise ValueError("role service has an unexpected service kind")
    if service.get("container_name") != container_name(install_uuid, key):
        raise ValueError("runtime service container name is not install-scoped")
    project = labels.get("com.docker.compose.project")
    if project is not None and project != project_name(install_uuid):
        raise ValueError("runtime service has a foreign Compose project")
    identity = (system, endpoint)
    if identity in seen:
        raise ValueError("duplicate runtime system endpoint")
    seen.add(identity)


def _writable_volume(value: object) -> tuple[str, bool] | None:
    source = _volume_source(value)
    if source is None:
        return None
    if isinstance(value, Mapping):
        read_only = value.get("read_only", False)
        if not isinstance(read_only, bool):
            raise ValueError("volume read_only must be boolean")
        return source, not read_only
    parts = value.split(":")
    return source, not (len(parts) == 3 and "ro" in parts[2].split(","))


def _validate_tailscale_infrastructure(
    install_uuid: str, service: Mapping[str, object]
) -> None:
    """Validate the immutable shape of the install-owned sidecar service."""

    if service.get("image") != _TAILSCALE_IMAGE:
        raise ValueError("tailscale infrastructure image is not the official stable image")
    if service.get("container_name") != container_name(install_uuid, "tailscale"):
        raise ValueError("tailscale infrastructure container name is not install-scoped")
    labels = service.get("labels")
    if not isinstance(labels, Mapping) or dict(labels) != {_INSTALL_LABEL: install_uuid}:
        raise ValueError("tailscale infrastructure labels are not exact")
    if service.get("restart") != "unless-stopped":
        raise ValueError("tailscale infrastructure restart policy is not exact")
    if service.get("entrypoint") not in (("tailscaled",), ["tailscaled"]):
        raise ValueError("tailscale infrastructure entrypoint is not exact")
    if service.get("command") not in (
        (
            "--statedir=/var/lib/tailscale",
            "--socket=/tmp/tailscaled.sock",
            "--tun=tailscale0",
        ),
        [
            "--statedir=/var/lib/tailscale",
            "--socket=/tmp/tailscaled.sock",
            "--tun=tailscale0",
        ],
    ):
        raise ValueError("tailscale infrastructure command is not exact")
    if service.get("devices") not in (("/dev/net/tun:/dev/net/tun",), ["/dev/net/tun:/dev/net/tun"]):
        raise ValueError("tailscale infrastructure device mapping is not exact")
    if service.get("cap_add") not in (("NET_ADMIN", "NET_RAW"), ["NET_ADMIN", "NET_RAW"]):
        raise ValueError("tailscale infrastructure capabilities are not exact")
    volumes = service.get("volumes")
    if not isinstance(volumes, (list, tuple)) or len(volumes) != 1:
        raise ValueError("tailscale infrastructure state mount is not exact")
    volume = volumes[0]
    if (
        not isinstance(volume, str)
        or len(volume.split(":")) != 3
        or not volume.split(":", 1)[0].startswith("/")
        or volume.split(":", 2)[1:] != ["/var/lib/tailscale", "rw"]
    ):
        raise ValueError("tailscale infrastructure state mount is not exact")
    healthcheck = service.get("healthcheck")
    expected_healthcheck = {
        "test": (
            "CMD-SHELL",
            "tailscale --socket=/tmp/tailscaled.sock status --json "
            "| grep -Eq '\"BackendState\"[[:space:]]*:[[:space:]]*\"Running\"'",
        ),
        "interval": "2s",
        "timeout": "2s",
        "retries": 60,
        "start_period": "2s",
    }
    if not isinstance(healthcheck, Mapping):
        raise ValueError("tailscale infrastructure healthcheck is missing")
    if {
        str(key): list(value) if isinstance(value, (list, tuple)) else value
        for key, value in healthcheck.items()
    } != {
        str(key): list(value) if isinstance(value, (list, tuple)) else value
        for key, value in expected_healthcheck.items()
    }:
        raise ValueError("tailscale infrastructure healthcheck is not exact")
    if "network_mode" in service or "depends_on" in service:
        raise ValueError("tailscale infrastructure cannot depend on or join another service")


def render_instance_services(
    install_uuid: str,
    instance: InstanceState,
    release: ReleaseManifest,
    endpoint_services: Mapping[str, Mapping[str, object]],
) -> dict[str, dict[str, object]]:
    """Pin generated endpoint services to one validated release."""

    if not isinstance(endpoint_services, Mapping):
        raise ValueError("endpoint_services must be an object")
    instance.validate()
    release.validate()
    if release.install_uuid != install_uuid:
        raise ValueError("release install_uuid does not match renderer install_uuid")
    if release_key(release) != instance.release_key:
        raise ValueError("instance release_key does not match release manifest")
    # project_name performs canonical install UUID validation.
    project_name(install_uuid)
    expected = {endpoint.endpoint_id: endpoint for endpoint in instance.endpoints}
    if len({endpoint.role for endpoint in instance.endpoints}) != len(instance.endpoints):
        raise ValueError("duplicate role execution within a system is not supported")
    if any(endpoint.endpoint_id == "coturn" for endpoint in instance.endpoints):
        raise ValueError("endpoint_id coturn is reserved for the Sim relay")
    if set(endpoint_services) != set(expected):
        raise ValueError("endpoint services must exactly match instance endpoints")
    rendered: dict[str, dict[str, object]] = {}
    sources: set[str] = set()
    for endpoint_id, endpoint in expected.items():
        original = endpoint_services[endpoint_id]
        if not isinstance(original, Mapping):
            raise ValueError("endpoint service must be an object")
        service = deepcopy(dict(original))
        role = service.pop("role", endpoint.role)
        if not isinstance(role, str) or role != endpoint.role or role not in _ROLES:
            raise ValueError("endpoint service role is unsupported or mismatched")
        if role not in release.role_images:
            raise ValueError(f"release has no image for role {role!r}")
        environment = _env(service)
        expected_env = {"ELESIM_SYSTEM_ID": instance.system_id, "ROS_DOMAIN_ID": str(instance.domain_id)}
        for key, value in expected_env.items():
            if key in environment and environment[key] != value:
                raise ValueError(f"service {endpoint_id!r} has conflicting {key}")
            environment[key] = value
        service["environment"] = environment
        service.pop("build", None)
        service["image"] = release.image_ids[role]
        key = service_key(instance.system_id, endpoint_id)
        if key in rendered:
            raise ValueError("rendered service keys collide")
        service["container_name"] = container_name(install_uuid, key)
        raw_labels = service.get("labels", {})
        if not isinstance(raw_labels, Mapping):
            raise ValueError("service labels must be an object")
        labels = dict(raw_labels)
        labels.update({_INSTALL_LABEL: install_uuid, "io.elesim.system_id": instance.system_id, "io.elesim.endpoint_id": endpoint_id, "io.elesim.role": role})
        service["labels"] = labels
        volumes = service.get("volumes", [])
        if not isinstance(volumes, (list, tuple)):
            raise ValueError("service volumes must be a list")
        for volume in volumes:
            mounted = _writable_volume(volume)
            if mounted is not None:
                source, writable = mounted
            else:
                continue
            if writable and source != "/tmp/.X11-unix":
                if source in sources:
                    raise ValueError(f"writable source path is shared: {source}")
                sources.add(source)
        if service.get("ipc") == "host" or service.get("ipc_mode") == "host":
            raise ValueError("host IPC is not permitted for instance Compose")
        rendered[key] = service
    return rendered


def aggregate_compose(
    install_uuid: str,
    instance_service_groups: Mapping[str, Mapping[str, Mapping[str, object]]],
    infrastructure: Mapping[str, Mapping[str, object]],
) -> dict[str, object]:
    """Merge rendered groups and install-scoped infrastructure without collisions."""

    project_name(install_uuid)
    services: dict[str, dict[str, object]] = {}
    if not isinstance(instance_service_groups, Mapping) or not isinstance(infrastructure, Mapping):
        raise ValueError("Compose service groups and infrastructure must be objects")
    seen: set[tuple[str, str]] = set()
    mounted_sources: list[tuple[str, bool]] = []
    for group in instance_service_groups.values():
        if not isinstance(group, Mapping):
            raise ValueError("Compose service group must be an object")
        for key, service in group.items():
            if key in services:
                raise ValueError(f"duplicate Compose service key: {key}")
            if not isinstance(key, str) or not isinstance(service, Mapping):
                raise ValueError("Compose services must have string names and object values")
            _validate_runtime_identity(install_uuid, key, service, seen)
            network_mode = service.get("network_mode", "host")
            if not isinstance(network_mode, str) or network_mode not in {
                "host",
                "service:tailscale",
            }:
                raise ValueError(
                    "runtime service network mode is not an allowed scoped mode"
                )
            if network_mode == "service:tailscale" and "tailscale" not in infrastructure:
                raise ValueError(
                    "runtime service requires the exact shared tailscale service"
                )
            ipc = service.get("ipc", service.get("ipc_mode"))
            if ipc not in (None, "private"):
                raise ValueError("foreign or host IPC is not permitted for runtime services")
            volumes = service.get("volumes", [])
            if not isinstance(volumes, (list, tuple)):
                raise ValueError("runtime service volumes must be a list")
            for volume in volumes:
                mounted = _runtime_volume(volume)
                if mounted is not None:
                    source, writable = mounted
                    if source != "/tmp/.X11-unix":
                        if any((writable or prior_writable) and _overlaps(source, previous) for previous, prior_writable in mounted_sources):
                            raise ValueError(f"writable source paths overlap: {source}")
                        mounted_sources.append((source, writable))
            services[key] = deepcopy(dict(service))
    scope = project_name(install_uuid).removeprefix("elesim-runtime-")
    for key, service in infrastructure.items():
        if key in services:
            raise ValueError(f"duplicate Compose service key: {key}")
        if not isinstance(key, str) or not isinstance(service, Mapping):
            raise ValueError("Compose infrastructure must have string names and object values")
        item = deepcopy(dict(service))
        container = item.get("container_name")
        if not isinstance(container, str) or not container.startswith(f"elesim-{scope}-"):
            raise ValueError("infrastructure container is not install-scoped")
        labels = item.get("labels")
        if not isinstance(labels, Mapping) or labels.get(_INSTALL_LABEL) != install_uuid:
            raise ValueError("infrastructure container lacks install ownership label")
        # The instance aggregate may share the installation's host-network
        # sidecar, but it must never become a second owner of arbitrary
        # infrastructure (in particular managed/external TURN).  The
        # caller validates the service against the immutable base Compose
        # entry; keep this final boundary narrow as well.
        if key != "tailscale":
            raise ValueError(
                "instance aggregate infrastructure may contain only the exact tailscale service"
            )
        _validate_tailscale_infrastructure(install_uuid, item)
        services[key] = item
    # Dependencies must resolve within this aggregate.  This catches a role
    # service that tries to join an install-global tools/dev/Coturn service
    # which is intentionally never copied into the scoped Compose project.
    for service in services.values():
        dependencies = service.get("depends_on", ())
        if isinstance(dependencies, Mapping):
            dependency_names = tuple(dependencies)
        elif isinstance(dependencies, (list, tuple)):
            dependency_names = tuple(dependencies)
        elif dependencies in (None, ""):
            dependency_names = ()
        else:
            raise ValueError("runtime service dependencies must be a mapping or list")
        if any(not isinstance(name, str) or name not in services for name in dependency_names):
            raise ValueError("runtime service dependency escapes this scoped Compose project")
    return {"name": project_name(install_uuid), "services": services}


__all__ = ["aggregate_compose", "render_instance_services"]
