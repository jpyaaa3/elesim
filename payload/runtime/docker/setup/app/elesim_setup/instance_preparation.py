"""Configuration preparation for endpoint-private instance services."""

from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import os
import stat
from pathlib import Path

from .configuration import generate_instance_configs
from .container_installer import ContainerInstaller
from .instances import (
    InstanceState,
    instance_turn_secret_path,
    scoped_turn_settings,
    turn_service_key,
)
from .credentials import validate_external_turn_credentials, _resolve_non_symlink_path
from .instance_identity import (
    CONTAINER_NAMING_HASH,
    validate_container_naming,
    service_key,
)
from .releases import ReleaseManifest, load_release, release_key, runtime_data_digest
from .instance_compose import render_instance_services
from .instance_security import active_instance_security_path
from .state import InstallState


def prepare_instance_services(
    state: InstallState,
    install_uuid: str,
    instance: InstanceState,
    release: ReleaseManifest,
    *,
    output_prefix: Path | None = None,
    security_views: dict[str, tuple[Path, str]] | None = None,
    robot_id: str | None = None,
    install_name: str = "",
    container_naming: str = CONTAINER_NAMING_HASH,
) -> dict[str, dict[str, object]]:
    """Validate a pinned release, render private configs, and return services.

    Endpoint-private configuration is written. Install state, legacy Compose,
    registry, and the Docker daemon are not modified. The returned services
    are suitable for an explicit aggregate Compose transaction.
    """
    state.validate()
    instance.validate()
    validate_container_naming(container_naming)
    if container_naming != CONTAINER_NAMING_HASH and not install_name:
        raise ValueError("system-v1 instance preparation requires install_name")
    effective_robot_id = state.network.robot_id
    if robot_id is not None:
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError("robot_id must be a non-empty endpoint identifier")
        effective_robot_id = robot_id
    replace(state.network, robot_id=effective_robot_id).validate()
    if "robot" in state.roles or not set(e.role for e in instance.endpoints).issubset(state.roles):
        raise ValueError("instance roles are not installed or include Robot")
    turn = scoped_turn_settings(instance, install_uuid)
    if turn.mode == "managed":
        expected_secret = instance_turn_secret_path(state.prefix_path, instance.system_id)
        actual_secret = Path(turn.secret_file).expanduser()
        if Path(os.path.abspath(os.fspath(actual_secret))) != expected_secret:
            raise ValueError(
                "managed instance TURN secret must be the instance-scoped path: "
                f"{expected_secret}"
            )
    if turn.mode == "managed" and not turn.public_host.strip() and not instance.turn_urls:
        raise ValueError("managed instance TURN requires a public host")
    if turn.mode == "managed":
        turn_urls = instance.turn_urls or (
            f"turn:{turn.public_host.strip()}:{turn.effective_listen_port}?transport=udp",
        )
    elif turn.mode == "external":
        turn_urls = instance.turn_urls
        if not turn_urls:
            raise ValueError("external instance TURN requires at least one URL")
        credentials = _resolve_non_symlink_path(
            Path(turn.credential_file), name="instance TURN credential path"
        )
        if not credentials.exists() or credentials.is_symlink() or not credentials.is_file():
            raise ValueError(
                f"instance TURN credential path is not a regular file: {credentials}"
            )
        if stat.S_IMODE(credentials.stat().st_mode) & 0o077:
            raise ValueError(
                "instance TURN credential file must be private (owner-only): "
                f"{credentials}"
            )
        validate_external_turn_credentials(credentials, urls=turn_urls)
    else:
        turn_urls = ()
    release.validate()
    if release_key(release) != instance.release_key or release.install_uuid != install_uuid:
        raise ValueError("release is not pinned to this installation or key")
    release_root = state.prefix_path / "releases" / instance.release_key
    manifest = load_release(release_root)
    if manifest != release:
        raise ValueError("published release does not match requested release")
    release_data_root = release_root / "data"
    if not release_data_root.is_dir() or release_data_root.is_symlink():
        raise FileNotFoundError(release_data_root)
    if runtime_data_digest(release_data_root) != manifest.runtime_data_digest:
        raise ValueError("release runtime data digest does not match manifest")
    template_root = release_data_root / "config"
    data_root = release_data_root / "data"
    if not template_root.is_dir() or template_root.is_symlink():
        raise FileNotFoundError(template_root)
    if not data_root.is_dir() or data_root.is_symlink():
        raise FileNotFoundError(data_root)
    roles = tuple(endpoint.role for endpoint in instance.endpoints)
    if any(role not in manifest.role_images or role not in manifest.build_fingerprints for role in roles):
        raise ValueError("release does not contain every instance role")
    endpoint_by_role = {e.role: e.endpoint_id for e in instance.endpoints}
    for role, endpoint in endpoint_by_role.items():
        for candidate in (
            state.prefix_path / "instances" / instance.system_id / "endpoints" / endpoint / "config",
            state.prefix_path / "instances" / instance.system_id / "endpoints" / endpoint / "cache",
        ):
            for parent in (candidate, *candidate.parents):
                if parent.is_symlink():
                    raise ValueError("instance path contains a symlink ancestor")
    if security_views is None:
        security_views = _instance_security_views(state, install_uuid, instance)
    elif instance.security_profile == "trusted-network" and security_views:
        raise ValueError("trusted-network instances cannot have security views")
    if instance.security_profile == "sros2" and not security_views:
        raise ValueError("sros2 instances require endpoint security views")
    if instance.security_profile == "sros2" and not instance.security_generation:
        raise ValueError("sros2 instances require an explicit security generation")
    instance_security_provisioning = (
        "managed" if instance.security_profile == "sros2" else "none"
    )
    if instance.security_profile == "sros2":
        # ``DdsSettings.validate`` requires a managed profile to carry a
        # complete bundle descriptor.  The actual per-endpoint paths are
        # supplied through ``security_views``; this representative view keeps
        # the temporary scoped state valid even when the install-level state
        # is still pending its first managed generation.
        representative_view = next(iter(security_views.values()))
        security_values = {
            "security_bundle": str(representative_view[0]),
            "keystore": str(representative_view[0]),
            "enclave": representative_view[1],
        }
    else:
        security_values = {
            "security_bundle": "",
            "keystore": "",
            "enclave": "",
        }
    # The transaction renders Compose and endpoint configuration below a
    # temporary staging prefix.  The service bind mounts are intentionally
    # rewritten to the published prefix at commit time, but the DDS settings
    # are also serialized into each role's YAML file and are not part of the
    # Compose document.  Keep those settings pointed at the eventual
    # published generation; otherwise a successful registration leaves the
    # containers referring to the deleted ``.instance-*`` staging tree.
    config_security_views = _published_security_views(
        security_views,
        output_prefix=output_prefix,
        published_prefix=state.prefix_path,
    )
    scoped = replace(
        state,
        # New instance records carry an immutable per-instance policy.  Older
        # schema-v2 records omitted it; retain their legacy install-wide GPU
        # selection instead of silently changing a fixed-device installation
        # to inherit mode during migration.
        compute=(instance.compute if instance.compute_is_explicit else state.compute),
        dds=replace(
            state.dds,
            system_id=instance.system_id,
            domain_id=instance.domain_id,
            rmw_implementation=instance.rmw_implementation,
            discovery_mode=instance.discovery_mode,
            static_peers=instance.static_peers,
            interface=instance.interface,
            security_profile=instance.security_profile,
            security_provisioning=instance_security_provisioning,
            security_generation=instance.security_generation,
            **security_values,
        ),
        turn=turn,
        network=replace(
            state.network,
            turn_urls=turn_urls,
            robot_id=effective_robot_id,
        ),
        assigned_roles=tuple(endpoint.role for endpoint in instance.endpoints),
    )
    scoped = replace(
        scoped,
        network=replace(scoped.network, **{
            f"{endpoint.role}_id": endpoint.endpoint_id
            for endpoint in instance.endpoints
        }),
    )
    configs = generate_instance_configs(
        scoped,
        instance,
        template_root=template_root,
        security_views=config_security_views or None,
        output_prefix=output_prefix,
        robot_id=effective_robot_id,
    )
    installer = ContainerInstaller(scoped, state_path=state.state_path, dry_run=True)
    installer._install_uuid = install_uuid
    installer._install_name = install_name
    installer._container_naming = container_naming
    installer._image_fingerprints = {
        f"elesim/{role}:local": manifest.build_fingerprints[role]
        for role in manifest.build_fingerprints
    }
    endpoint_services: dict[str, dict[str, object]] = {}
    for role in scoped.assigned_roles or ():
        endpoint = endpoint_by_role[role]
        endpoint_cache_root = (
            state.prefix_path if output_prefix is None else Path(output_prefix).expanduser()
        ) / "instances" / instance.system_id / "endpoints" / endpoint / "cache"
        # Compose creates a missing bind source as root.  Sim deliberately
        # runs as the installing operator, so leave the endpoint cache in the
        # transaction-owned tree before Compose ever sees it.
        for parent in (endpoint_cache_root, *endpoint_cache_root.parents):
            if parent.is_symlink():
                raise ValueError("instance cache path contains a symlink ancestor")
        endpoint_cache_root.mkdir(mode=0o700, parents=True, exist_ok=True)
        endpoint_cache_root.chmod(0o700)
        service = installer._role_service(
            role,
            config_root=configs[role].parent,
            cache_root=endpoint_cache_root,
            data_root=data_root,
            keystore_root=(security_views[role][0] if security_views else None),
            enclave=(security_views[role][1] if security_views else None),
            instance_scoped=True,
            compute=instance.role_compute.get(role, scoped.compute),
        )
        if role == "sim":
            service["environment"]["ELESIM_SIM_VIEWER"] = "1" if instance.viewer else "0"
            # The transaction may render under a temporary prefix which is
            # replaced with the installed prefix only after Compose is staged.
            # Derive the fallback identity from the final endpoint cache path
            # so repeated registrations do not leak a new /tmp cache.
            final_cache_root = (
                state.prefix_path
                / "instances"
                / instance.system_id
                / "endpoints"
                / endpoint
                / "cache"
            )
            service["environment"]["ELESIM_CACHE_NAMESPACE"] = hashlib.sha256(
                str(final_cache_root).encode("utf-8")
            ).hexdigest()[:16]
        endpoint_services[endpoint] = service
    rendered = render_instance_services(
        install_uuid,
        instance,
        release,
        endpoint_services,
        install_name=install_name,
        container_naming=container_naming,
    )
    if turn.mode == "managed":
        key = turn_service_key(instance.system_id)
        rendered[key] = installer._coturn_service(
            instance_scoped=True,
            service_key=key,
            sim_service_key=service_key(instance.system_id, endpoint_by_role["sim"]),
            turn=turn,
        )
    return rendered


def _published_security_views(
    security_views: dict[str, tuple[Path, str]] | None,
    *,
    output_prefix: Path | None,
    published_prefix: Path,
) -> dict[str, tuple[Path, str]] | None:
    """Point serialized DDS settings at the post-transaction prefix.

    ``security_views`` is also used for the host-side bind mount.  During an
    instance transaction those mounts must initially reference the staged
    tree, while the YAML consumed inside the container must reference the
    final tree after the staged directory is renamed into place.
    """

    if not security_views or output_prefix is None:
        return security_views
    staging = Path(output_prefix).expanduser()
    published = Path(published_prefix).expanduser()
    result: dict[str, tuple[Path, str]] = {}
    for role, (keystore, enclave) in security_views.items():
        candidate = Path(keystore)
        try:
            relative = candidate.relative_to(staging)
        except ValueError:
            # Callers may intentionally supply an already-published view.  Do
            # not rewrite paths outside this transaction's staging prefix.
            final_keystore = candidate
        else:
            final_keystore = published / relative
        result[role] = (final_keystore, enclave)
    return result


def _instance_security_views(
    state: InstallState,
    install_uuid: str,
    instance: InstanceState,
) -> dict[str, tuple[Path, str]]:
    if instance.security_profile == "trusted-network":
        return {}
    if state.dds.security_provisioning != "managed":
        raise ValueError("instance preparation requires managed SROS2 security")
    prefix = state.prefix_path
    current = prefix / "instances" / instance.system_id / "security" / "current"
    paths: dict[str, tuple[Path, str]] = {}
    for endpoint in instance.endpoints:
        keystore = active_instance_security_path(
            prefix,
            install_uuid,
            instance.system_id,
            endpoint.endpoint_id,
        )
        paths[endpoint.role] = (
            keystore.resolve(),
            f"/elesim/{instance.system_id}/{endpoint.role}/"
            f"{endpoint.endpoint_id.replace('-', '_')[:63]}",
        )
    manifest_path = current / "manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError("active instance security manifest is missing")
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("active instance security manifest is invalid") from exc
    if not isinstance(payload, dict):
        raise ValueError("active instance security manifest is invalid")
    if payload.get("instance_release_key") != instance.release_key:
        raise ValueError("active instance security release binding does not match release")
    active_generation = current.resolve().name
    if instance.security_generation and instance.security_generation != active_generation:
        raise ValueError("active instance security generation does not match state")
    expected = [
        {
            "role": endpoint.role,
            "endpoint_id": endpoint.endpoint_id,
            "enclave": f"enclaves/elesim/{instance.system_id}/{endpoint.role}/"
            f"{endpoint.endpoint_id.replace('-', '_')[:63]}",
        }
        for endpoint in instance.endpoints
    ]
    if payload.get("endpoints") != expected:
        raise ValueError("active instance security endpoints do not match instance")
    return paths


__all__ = ["prepare_instance_services"]
