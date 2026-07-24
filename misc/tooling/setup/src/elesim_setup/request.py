"""Transport-neutral setup request shared by CLI automation and the web wizard."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .capabilities import HostCapabilities
from .profiles import normalize_roles
from .state import (
    ComputeSettings,
    DdsSettings,
    InstallState,
    NetworkSettings,
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
    ssh: SshCredentialSource = field(default_factory=SshCredentialSource)
    register_path: bool = False
    jaeger: bool = False
    repository: str = "jpyaaa3/elesim"
    ref: str = "main"

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "SetupRequest":
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
        if turn_mode == "managed" and not secret_file:
            secret_file = str(prefix / "secrets/turn.secret")
        return cls(
            language=str(raw.get("language", "ko")),
            edition=str(raw.get("edition", "general")),
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
                simulator_id=str(raw.get("simulator_id", "sim-default")),
                controller_id=str(raw.get("controller_id", "controller-main")),
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
                security_profile=str(
                    raw.get("dds_security_profile", "trusted-network")
                ),
                keystore=str(raw.get("dds_keystore", "")),
                enclave=str(raw.get("dds_enclave", "")),
            ),
            turn=TurnSettings(
                mode=turn_mode,
                realm=str(raw.get("turn_realm", "")),
                public_host=str(raw.get("turn_public_host", "")),
                secret_file=secret_file,
                credential_file=credential_file,
            ),
            ssh=SshCredentialSource.from_dict(
                raw.get("ssh") if isinstance(raw.get("ssh"), Mapping) else None
            ),
            register_path=bool(raw.get("register_path", False)),
            jaeger=bool(raw.get("jaeger", False)),
            repository=str(raw.get("repository", "jpyaaa3/elesim")),
            ref=str(raw.get("ref", "main")),
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
        if self.dds.discovery_mode == "static" and not self.dds.static_peers:
            raise ValueError("static DDS discovery에는 peer가 하나 이상 필요합니다")
        if self.dds.security_profile == "sros2" and (
            not self.dds.keystore.strip() or not self.dds.enclave.strip()
        ):
            raise ValueError("SROS2 profile에는 keystore와 enclave가 필요합니다")
        if self.edition == "developer":
            if not capabilities.developer_installable:
                raise ValueError("개발자 환경은 Ubuntu/WSL amd64에서만 지원합니다")
            if self.roles:
                raise ValueError("개발자 환경은 역할을 고르지 않고 전체 workspace를 설치합니다")
            if self.turn.mode != "none":
                raise ValueError("개발자 환경의 TURN은 runtime 설치에서 별도로 구성하십시오")
        else:
            roles = normalize_roles(self.roles)
            if "robot" in roles:
                if roles != ("robot",):
                    raise ValueError("Robot native 설치는 다른 역할과 분리한 단독 설치여야 합니다")
                if not capabilities.robot_installable:
                    raise ValueError("Robot 설치에는 감지된 Jetson/JetPack 호스트가 필요합니다")
        if self.turn.managed:
            if self.edition != "general" or "simulator" not in self.roles:
                raise ValueError("managed Coturn은 Simulator 설치 호스트가 필요합니다")
        if (
            self.turn.mode == "external"
            and "simulator" in self.roles
            and self.turn.credential_path is None
        ):
            raise ValueError(
                "Simulator의 external TURN에는 username/credential JSON file이 "
                "필요합니다"
            )
        self._state().validate()
        return self

    def to_install_state(self) -> InstallState:
        if self.edition != "general":
            raise ValueError("developer request는 runtime InstallState로 변환할 수 없습니다")
        return self._state().require_runnable_dds()

    def _state(self) -> InstallState:
        roles = (
            normalize_roles(self.roles)
            if self.roles
            else ("simulator",) if self.edition == "developer" else ()
        )
        install_mode = "native" if roles == ("robot",) else "container"
        return InstallState(
            profile="custom",
            roles=roles,
            prefix=str(self.prefix),
            bin_dir=str(self.bin_dir),
            source_root=str(self.source_root),
            network=self.network,
            dds=self.dds,
            compute=self.compute,
            turn=self.turn,
            install_mode=install_mode,
        )


def _required_path(raw: Mapping[str, Any], name: str) -> Path:
    value = str(raw.get(name, "")).strip()
    if not value:
        raise ValueError(f"{name} 경로가 필요합니다")
    return Path(value).expanduser().resolve()


__all__ = [
    "EDITIONS",
    "SetupRequest",
    "SshCredentialSource",
]
