"""Transport-neutral setup request shared by CLI automation and the web wizard."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .capabilities import HostCapabilities
from .profiles import normalize_roles
from .state import (
    ComputeSettings,
    ContainerNetworkSettings,
    DEFAULT_SOURCE_REF,
    DEFAULT_SOURCE_REPOSITORY,
    DdsSettings,
    DeveloperAttachmentSettings,
    InstallState,
    NetworkSettings,
    RuntimeTextLogSettings,
    TurnSettings,
)


@dataclass(frozen=True)
class SshCredentialSource:
    """Legacy-compatible SSH fields kept separate from DDS discovery settings."""

    host: str = ""
    port: int = 22
    user: str = ""
    remote_root: str = ""
    identity_file: str = ""
    accepted_fingerprint: str = ""

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any] | None) -> "SshCredentialSource":
        values = {} if raw is None else dict(raw)
        return cls(
            host=str(values.get("host", "")),
            port=int(values.get("port", 22)),
            user=str(values.get("user", "")),
            remote_root=str(values.get("remote_root", "")),
            identity_file=str(values.get("identity_file", "")),
            accepted_fingerprint=str(values.get("accepted_fingerprint", "")),
        )


@dataclass(frozen=True)
class SetupRequest:
    language: str
    roles: tuple[str, ...]
    prefix: Path
    bin_dir: Path
    source_root: Path
    compute: ComputeSettings
    network: NetworkSettings
    dds: DdsSettings
    turn: TurnSettings
    developer_attachment: DeveloperAttachmentSettings = field(
        default_factory=DeveloperAttachmentSettings
    )
    runtime_text_logs: RuntimeTextLogSettings = field(
        default_factory=RuntimeTextLogSettings
    )
    ssh: SshCredentialSource = field(default_factory=SshCredentialSource)
    register_path: bool = False
    repository: str = DEFAULT_SOURCE_REPOSITORY
    ref: str = DEFAULT_SOURCE_REF

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "SetupRequest":
        legacy_edition = str(raw.get("edition", "general"))
        if legacy_edition != "general":
            raise ValueError(
                "Developer is no longer a separate installation type. Remove the "
                "existing Developer installation, then select the developer "
                "attachment on a general installation"
            )
        turn_url = str(raw.get("turn_url", "")).strip()
        roles_raw = raw.get("roles", ())
        if not isinstance(roles_raw, (list, tuple)):
            raise ValueError("roles must be a list of program names")
        peers_raw = raw.get("dds_static_peers", ())
        if isinstance(peers_raw, str):
            peers = tuple(
                value.strip() for value in peers_raw.split(",") if value.strip()
            )
        elif isinstance(peers_raw, (list, tuple)):
            peers = tuple(str(value).strip() for value in peers_raw if str(value).strip())
        else:
            raise ValueError("dds_static_peers must be a list of hostnames/IPs")
        prefix = _required_path(raw, "prefix")
        developer_enabled = raw.get("developer_attachment", False)
        if type(developer_enabled) is not bool:
            raise ValueError("developer_attachment must be boolean")
        developer_wslg = raw.get("developer_wslg", False)
        if type(developer_wslg) is not bool:
            raise ValueError("developer_wslg must be boolean")
        developer_workspace = str(raw.get("developer_workspace", "")).strip()
        turn_mode = str(raw.get("turn_mode", "none"))
        secret_file = str(raw.get("turn_secret_file", "")).strip()
        credential_file = str(raw.get("turn_credential_file", "")).strip()
        security_profile = str(
            raw.get("dds_security_profile", "trusted-network")
        )
        security_bundle = str(raw.get("dds_security_bundle", "")).strip()
        security_provisioning = str(
            raw.get(
                "dds_security_provisioning",
                "external" if security_profile == "sros2" else "none",
            )
        )
        keystore = str(raw.get("dds_keystore", "")).strip()
        if security_provisioning == "managed" and security_bundle and not keystore:
            keystore = security_bundle
        if turn_mode == "managed" and not secret_file:
            secret_file = str(prefix / "secrets/turn.secret")
        return cls(
            language=str(raw.get("language", "ko")),
            roles=tuple(str(value) for value in roles_raw),
            prefix=prefix,
            bin_dir=_required_path(raw, "bin_dir"),
            source_root=_required_path(raw, "source_root"),
            compute=ComputeSettings(
                gpu_mode=str(raw.get("gpu_mode", "inherit")),
                gpu_device=str(raw.get("gpu_device", "")),
            ),
            network=NetworkSettings(
                turn_urls=(turn_url,) if turn_url else (),
                sim_id=str(raw.get("sim_id", "sim-default")),
                pilot_id=str(raw.get("pilot_id", "pilot-main")),
                ui_id=str(raw.get("ui_id", "ui-main")),
                robot_id=str(raw.get("robot_id", "robot-go2")),
            ),
            dds=DdsSettings(
                system_id=str(raw.get("dds_system_id", "elesim")),
                domain_id=int(raw.get("dds_domain_id", 0)),
                rmw_implementation=str(
                    raw.get("dds_rmw_implementation", "rmw_cyclonedds_cpp")
                ),
                discovery_mode=str(raw.get("dds_discovery_mode", "multicast")),
                static_peers=peers,
                interface=str(raw.get("dds_interface", "")),
                security_profile=security_profile,
                security_provisioning=security_provisioning,
                security_generation=str(raw.get("dds_security_generation", "")),
                security_bundle=security_bundle,
                keystore=keystore,
                enclave=str(raw.get("dds_enclave", "")),
            ),
            turn=TurnSettings(
                mode=turn_mode,
                realm=str(raw.get("turn_realm", "")),
                public_host=str(raw.get("turn_public_host", "")),
                secret_file=secret_file,
                credential_file=credential_file,
            ),
            developer_attachment=DeveloperAttachmentSettings(
                enabled=developer_enabled,
                workspace=developer_workspace if developer_enabled else "",
                wslg=developer_wslg if developer_enabled else False,
            ),
            runtime_text_logs=_runtime_text_log_settings(raw),
            ssh=SshCredentialSource.from_dict(
                raw.get("ssh") if isinstance(raw.get("ssh"), Mapping) else None
            ),
            register_path=bool(raw.get("register_path", False)),
            repository=str(raw.get("repository", DEFAULT_SOURCE_REPOSITORY)),
            ref=str(raw.get("ref", DEFAULT_SOURCE_REF)),
        )

    def validate(self, capabilities: HostCapabilities) -> "SetupRequest":
        if self.language not in {"ko", "en"}:
            raise ValueError(f"Unsupported language: {self.language!r}")
        if not str(self.prefix) or not str(self.bin_dir) or not str(self.source_root):
            raise ValueError(
                "Installation prefix, bin directory, and source root are required"
            )
        self.compute.validate()
        self.network.validate()
        self.dds.validate()
        self.turn.validate()
        self.runtime_text_logs.validate()
        self.developer_attachment.validate()
        if self.dds.discovery_mode == "static" and not self.dds.static_peers:
            raise ValueError("static DDS discovery requires at least one peer")
        if (
            self.dds.security_profile == "sros2"
            and self.dds.security_provisioning == "external"
            and (not self.dds.keystore.strip() or not self.dds.enclave.strip())
        ):
            raise ValueError("SROS2 profile requires a keystore and enclave")
        roles = normalize_roles(self.roles)
        if self.developer_attachment.enabled:
            if not capabilities.developer_installable:
                raise ValueError("developer attachment is supported only on Ubuntu/WSL amd64")
            if self.developer_attachment.wslg and not capabilities.wslg_available:
                raise ValueError("developer WSLg attachment requires a detected WSLg host")
            if roles == ("robot",):
                raise ValueError("developer attachment cannot be added to native Robot installation")
        if "robot" in roles:
            if roles != ("robot",):
                raise ValueError("native Robot installation must be standalone and separate from other roles")
            if not capabilities.robot_installable:
                raise ValueError("Robot installation requires a detected Jetson/JetPack host")
            if not self.dds.interface.strip():
                raise ValueError(
                    "Robot installation requires an explicitly specified inter-host "
                    "EleSim DDS interface"
                )
        if self.turn.managed:
            if "sim" not in self.roles:
                raise ValueError("managed Coturn requires a Sim installation host")
        if (
            self.turn.mode == "external"
            and "sim" in self.roles
            and self.turn.credential_path is None
        ):
            raise ValueError(
                "Sim external TURN requires a username/credential JSON file"
            )
        self._state(capabilities).validate()
        return self

    def to_install_state(
        self,
        capabilities: HostCapabilities | None = None,
    ) -> InstallState:
        return self._state(capabilities).require_installable_dds()

    def _state(
        self,
        capabilities: HostCapabilities | None = None,
    ) -> InstallState:
        roles = normalize_roles(self.roles)
        install_mode = "native" if roles == ("robot",) else "container"
        container_network = container_network_settings_for_host(
            capabilities=capabilities,
            install_mode=install_mode,
            prefix=self.prefix,
        )
        return InstallState(
            profile="custom",
            roles=roles,
            prefix=str(self.prefix),
            bin_dir=str(self.bin_dir),
            source_root=str(self.source_root),
            source_repository=self.repository,
            source_ref=self.ref,
            network=self.network,
            dds=self.dds,
            compute=self.compute,
            turn=self.turn,
            runtime_text_logs=self.runtime_text_logs,
            container_network=container_network,
            developer_attachment=self.developer_attachment,
            install_mode=install_mode,
        )


def container_network_settings_for_host(
    *,
    capabilities: HostCapabilities | None,
    install_mode: str,
    prefix: Path,
) -> ContainerNetworkSettings:
    if capabilities is None or install_mode != "container":
        return ContainerNetworkSettings()
    backend = capabilities.docker_backend.strip()
    context = capabilities.docker_context.strip()
    engine_id = capabilities.docker_engine_id.strip()
    endpoint = capabilities.docker_endpoint.strip()
    docker_host_override = capabilities.docker_host_override.strip()
    if docker_host_override:
        raise ValueError(
            "DOCKER_HOST overrides are unsupported. Unset DOCKER_HOST to use the "
            "installation-pinned local Docker context"
        )
    if not backend and not context and not engine_id and not endpoint:
        # Compatibility for direct API users that predate bootstrap-provided
        # Docker facts. The supported bootstrap always pins new installs.
        return ContainerNetworkSettings()
    if backend not in {"native", "docker-desktop"}:
        raise ValueError(f"Unsupported Docker backend: {backend!r}")
    if not context or not engine_id or not endpoint:
        raise ValueError(
            "Cannot determine Docker context, engine ID, and endpoint. Recheck the "
            "Docker daemon selected during bootstrap"
        )
    if endpoint.startswith(("ssh://", "tcp://")):
        raise ValueError(
            "remote Docker contexts cannot safely bind-mount local installation paths "
            f"and are unsupported: {endpoint}"
        )
    if not endpoint.startswith(("unix://", "npipe://")):
        raise ValueError(f"Unsupported Docker context endpoint: {endpoint!r}")
    if backend == "docker-desktop":
        stable_input = (
            engine_id + "\x00" + str(prefix.expanduser().resolve())
        ).encode("utf-8")
        hostname = "elesim-" + hashlib.sha256(stable_input).hexdigest()[:12]
        return ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context=context,
            docker_engine_id=engine_id,
            tailscale_hostname=hostname,
            tailscale_state_dir=str(prefix / "secrets/tailscale"),
        )
    return ContainerNetworkSettings(
        mode="direct-host",
        docker_context=context,
        docker_engine_id=engine_id,
    )


def _required_path(raw: Mapping[str, Any], name: str) -> Path:
    value = str(raw.get(name, "")).strip()
    if not value:
        raise ValueError(f"{name} path is required")
    return Path(value).expanduser().resolve()


def _runtime_text_log_settings(
    raw: Mapping[str, Any],
) -> RuntimeTextLogSettings:
    value = raw.get("runtime_text_logs")
    default_enabled = True
    if value is None:
        return RuntimeTextLogSettings(enabled=default_enabled)
    if not isinstance(value, Mapping):
        raise ValueError("runtime_text_logs must be an object")
    unexpected = set(value).difference({"enabled"})
    if unexpected:
        rendered = ", ".join(sorted(str(name) for name in unexpected))
        raise ValueError(f"unknown field in runtime_text_logs: {rendered}")
    return RuntimeTextLogSettings(
        enabled=value.get("enabled", default_enabled),
    ).validate()


__all__ = [
    "SetupRequest",
    "SshCredentialSource",
    "container_network_settings_for_host",
]
