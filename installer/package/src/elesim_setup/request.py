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
    InstallState,
    NetworkSettings,
    RuntimeTextLogSettings,
    TurnSettings,
)


EDITIONS = frozenset({"general", "developer"})


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
    edition: str
    roles: tuple[str, ...]
    prefix: Path
    bin_dir: Path
    source_root: Path
    compute: ComputeSettings
    network: NetworkSettings
    dds: DdsSettings
    turn: TurnSettings
    runtime_text_logs: RuntimeTextLogSettings = field(
        default_factory=RuntimeTextLogSettings
    )
    ssh: SshCredentialSource = field(default_factory=SshCredentialSource)
    register_path: bool = False
    jaeger: bool = False
    repository: str = DEFAULT_SOURCE_REPOSITORY
    ref: str = DEFAULT_SOURCE_REF

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "SetupRequest":
        edition = str(raw.get("edition", "general"))
        turn_url = str(raw.get("turn_url", "")).strip()
        roles_raw = raw.get("roles", ())
        if not isinstance(roles_raw, (list, tuple)):
            raise ValueError("roles는 프로그램 이름 목록이어야 합니다")
        peers_raw = raw.get("dds_static_peers", ())
        if isinstance(peers_raw, str):
            peers = tuple(
                value.strip() for value in peers_raw.split(",") if value.strip()
            )
        elif isinstance(peers_raw, (list, tuple)):
            peers = tuple(str(value).strip() for value in peers_raw if str(value).strip())
        else:
            raise ValueError("dds_static_peers는 hostname/IP 목록이어야 합니다")
        prefix = _required_path(raw, "prefix")
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
            edition=edition,
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
            runtime_text_logs=_runtime_text_log_settings(raw, edition=edition),
            ssh=SshCredentialSource.from_dict(
                raw.get("ssh") if isinstance(raw.get("ssh"), Mapping) else None
            ),
            register_path=bool(raw.get("register_path", False)),
            jaeger=bool(raw.get("jaeger", False)),
            repository=str(raw.get("repository", DEFAULT_SOURCE_REPOSITORY)),
            ref=str(raw.get("ref", DEFAULT_SOURCE_REF)),
        )

    def validate(self, capabilities: HostCapabilities) -> "SetupRequest":
        if self.language not in {"ko", "en"}:
            raise ValueError(f"지원하지 않는 언어: {self.language!r}")
        if self.edition not in EDITIONS:
            raise ValueError(f"지원하지 않는 설치 종류: {self.edition!r}")
        if not str(self.prefix) or not str(self.bin_dir) or not str(self.source_root):
            raise ValueError("설치, bin, source 경로가 필요합니다")
        self.compute.validate()
        self.network.validate()
        self.dds.validate()
        self.turn.validate()
        self.runtime_text_logs.validate()
        if self.dds.discovery_mode == "static" and not self.dds.static_peers:
            raise ValueError("static DDS discovery에는 peer가 하나 이상 필요합니다")
        if (
            self.dds.security_profile == "sros2"
            and self.dds.security_provisioning == "external"
            and (not self.dds.keystore.strip() or not self.dds.enclave.strip())
        ):
            raise ValueError("SROS2 profile에는 keystore와 enclave가 필요합니다")
        if self.edition == "developer":
            if not capabilities.developer_installable:
                raise ValueError("개발자 환경은 Ubuntu/WSL amd64에서만 지원합니다")
            if self.roles:
                raise ValueError("개발자 환경은 역할을 고르지 않고 전체 workspace를 설치합니다")
            if self.turn.mode != "none":
                raise ValueError("개발자 환경의 TURN은 runtime 설치에서 별도로 구성하십시오")
            if self.dds.security_provisioning == "managed":
                raise ValueError(
                    "개발자 환경은 connection-managed SROS2 provisioning을 지원하지 "
                    "않습니다. trusted-network 또는 external SROS2를 사용하십시오"
                )
            if self.runtime_text_logs.enabled:
                raise ValueError(
                    "개발자 환경은 기존 Compose log follow만 사용하며 runtime text "
                    "archive를 생성하지 않습니다"
                )
        else:
            roles = normalize_roles(self.roles)
            if "robot" in roles:
                if roles != ("robot",):
                    raise ValueError("Robot native 설치는 다른 역할과 분리한 단독 설치여야 합니다")
                if not capabilities.robot_installable:
                    raise ValueError("Robot 설치에는 감지된 Jetson/JetPack 호스트가 필요합니다")
                if not self.dds.interface.strip():
                    raise ValueError(
                        "Robot 설치는 inter-host EleSim DDS interface를 명시해야 합니다"
                    )
        if self.turn.managed:
            if self.edition != "general" or "sim" not in self.roles:
                raise ValueError("managed Coturn은 Sim 설치 호스트가 필요합니다")
        if (
            self.turn.mode == "external"
            and "sim" in self.roles
            and self.turn.credential_path is None
        ):
            raise ValueError(
                "Sim의 external TURN에는 username/credential JSON file이 "
                "필요합니다"
            )
        self._state(capabilities).validate()
        return self

    def to_install_state(
        self,
        capabilities: HostCapabilities | None = None,
    ) -> InstallState:
        if self.edition != "general":
            raise ValueError("developer request는 runtime InstallState로 변환할 수 없습니다")
        return self._state(capabilities).require_installable_dds()

    def _state(
        self,
        capabilities: HostCapabilities | None = None,
    ) -> InstallState:
        roles = (
            normalize_roles(self.roles)
            if self.roles
            else ("sim",) if self.edition == "developer" else ()
        )
        install_mode = "native" if roles == ("robot",) else "container"
        container_network = container_network_settings_for_host(
            capabilities=capabilities,
            edition=self.edition,
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
            install_mode=install_mode,
        )


def container_network_settings_for_host(
    *,
    capabilities: HostCapabilities | None,
    edition: str,
    install_mode: str,
    prefix: Path,
) -> ContainerNetworkSettings:
    if capabilities is None or edition != "general" or install_mode != "container":
        return ContainerNetworkSettings()
    backend = capabilities.docker_backend.strip()
    context = capabilities.docker_context.strip()
    engine_id = capabilities.docker_engine_id.strip()
    endpoint = capabilities.docker_endpoint.strip()
    docker_host_override = capabilities.docker_host_override.strip()
    if docker_host_override:
        raise ValueError(
            "DOCKER_HOST override는 지원하지 않습니다. 설치가 고정한 local Docker "
            "context를 재현할 수 있도록 DOCKER_HOST를 해제하십시오"
        )
    if not backend and not context and not engine_id and not endpoint:
        # Compatibility for direct API users that predate bootstrap-provided
        # Docker facts. The supported bootstrap always pins new installs.
        return ContainerNetworkSettings()
    if backend not in {"native", "docker-desktop"}:
        raise ValueError(f"지원하지 않는 Docker backend: {backend!r}")
    if not context or not engine_id or not endpoint:
        raise ValueError(
            "Docker context, engine ID와 endpoint를 확인할 수 없습니다. "
            "설치 bootstrap에서 선택한 Docker daemon을 다시 확인하십시오"
        )
    if endpoint.startswith(("ssh://", "tcp://")):
        raise ValueError(
            "remote Docker context는 local 설치 경로를 안전하게 bind mount할 수 "
            f"없어 지원하지 않습니다: {endpoint}"
        )
    if not endpoint.startswith(("unix://", "npipe://")):
        raise ValueError(f"지원하지 않는 Docker context endpoint: {endpoint!r}")
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
        raise ValueError(f"{name} 경로가 필요합니다")
    return Path(value).expanduser().resolve()


def _runtime_text_log_settings(
    raw: Mapping[str, Any],
    *,
    edition: str,
) -> RuntimeTextLogSettings:
    value = raw.get("runtime_text_logs")
    default_enabled = edition == "general"
    if value is None:
        return RuntimeTextLogSettings(enabled=default_enabled)
    if not isinstance(value, Mapping):
        raise ValueError("runtime_text_logs는 object여야 합니다")
    unexpected = set(value).difference({"enabled"})
    if unexpected:
        rendered = ", ".join(sorted(str(name) for name in unexpected))
        raise ValueError(f"runtime_text_logs에 알 수 없는 field가 있습니다: {rendered}")
    return RuntimeTextLogSettings(
        enabled=value.get("enabled", default_enabled),
    ).validate()


__all__ = [
    "EDITIONS",
    "SetupRequest",
    "SshCredentialSource",
    "container_network_settings_for_host",
]
