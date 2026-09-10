"""Transactional lifecycle for install-scoped, robot-free instances.

This module is intentionally a separate boundary from the normal installer.
It only owns ``instances/<system>`` and ``containers/compose.instances.yaml``;
the ordinary install state, legacy Compose file, Docker daemon, and PATH are
never touched.
"""

from __future__ import annotations

import fcntl
import json
import os
import re
import secrets
import shlex
import shutil
import stat
import tempfile
import threading
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Iterator, Mapping

import yaml

from .container_installer import (
    ContainerInstaller,
    _docker_backend_guard,
    _runtime_down_wrapper,
    _runtime_logs_wrapper,
)
from .instance_compose import aggregate_compose
from .instance_identity import container_name, project_name, service_key
from .instance_preparation import prepare_instance_services
from .instance_security import (
    _rename_noreplace,
    active_instance_security_path,
    validate_instance_security_generation,
)
from .manager_lifecycle import compose_owner_guard
from .operation_lock import render_lock_preamble
from .instances import (
    InstanceState,
    instance_turn_secret_path,
    scoped_turn_settings,
    turn_service_key,
)
from .releases import ReleaseManifest, load_release, release_key
from .shell import write_executable
from .state import InstallState
from .ownership import append_instance_docker_ownership


_SYSTEM = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_SECURITY_GENERATION = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,95}$")
_HOST_LEASE_TOKEN = re.compile(r"^[0-9a-f]{64}$")
_THREAD_LOCKS: dict[str, threading.Lock] = {}
_THREAD_LOCKS_GUARD = threading.Lock()


class InstanceRuntime:
    """Manage several immutable-release instances in one new prefix.

    ``state`` is used as capability/configuration input only.  The methods do
    not call Docker and do not invoke the regular installer.  A release is
    selected by the content-addressed key in :class:`InstanceState` and is
    always loaded and verified before a registry mutation is staged.
    """

    def __init__(
        self,
        state: InstallState,
        install_uuid: str,
        *,
        ownership_manifest: Path | None = None,
    ) -> None:
        self.state = state.validate()
        if self.state.install_mode != "container":
            raise ValueError("instance runtime requires a container install")
        if "robot" in self.state.roles:
            raise ValueError("robot instances are not supported")
        if self.state.dds.security_profile not in {"trusted-network", "sros2"}:
            raise ValueError("instance runtime requires trusted-network or sros2 security")
        if not (
            self.state.container_network.docker_context.strip()
            and self.state.container_network.docker_engine_id.strip()
        ):
            raise ValueError(
                "instance runtime requires a pinned Docker context and engine ID"
            )
        # project_name performs strict canonical UUID validation.
        self.install_uuid = install_uuid
        self.project = project_name(install_uuid)
        self.prefix = self._lexical(Path(self.state.prefix))
        self.compose = self.prefix / "containers" / "compose.instances.yaml"
        self.base_compose = self.prefix / "containers" / "compose.yaml"
        self._shared_infrastructure = self._load_shared_infrastructure()
        self.ownership_manifest = (
            None if ownership_manifest is None else self._lexical(Path(ownership_manifest))
        )
        # Reuse the registry helper's install-level lock so direct
        # InstanceRegistry readers/writers cannot race this aggregate update.
        self.lock_root = self.prefix / "instances" / ".locks"
        self.install_lock = self.lock_root / "install.lock"

    @staticmethod
    def _normalize_compose_value(value: object) -> object:
        """Normalize YAML sequences for comparison with generated tuples."""

        if isinstance(value, Mapping):
            return {
                str(key): InstanceRuntime._normalize_compose_value(item)
                for key, item in value.items()
            }
        if isinstance(value, (list, tuple)):
            return [InstanceRuntime._normalize_compose_value(item) for item in value]
        return value

    def _load_shared_infrastructure(self) -> dict[str, Mapping[str, object]]:
        """Load the one install-owned sidecar from the immutable base Compose.

        A scoped instance aggregate is allowed to reference only the exact
        service emitted by the installation.  Reading and comparing the
        service here prevents a hand-edited base Compose file from becoming a
        privileged dependency of every instance.
        """

        if not self.state.container_network.uses_tailscale_sidecar:
            return {}
        try:
            self._check_tree(self.base_compose, allow_missing=False)
        except FileNotFoundError as exc:
            raise ValueError("tailscale-sidecar requires the install base Compose manifest") from exc
        if self.base_compose.is_symlink() or not self.base_compose.is_file():
            raise ValueError(f"base Compose manifest is not a regular file: {self.base_compose}")
        try:
            payload = yaml.safe_load(self.base_compose.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise ValueError(f"base Compose manifest could not be read: {self.base_compose}") from exc
        if not isinstance(payload, Mapping) or payload.get("name") != self.project:
            raise ValueError("base Compose manifest has the wrong install project")
        services = payload.get("services")
        if not isinstance(services, Mapping):
            raise ValueError("base Compose manifest has no services object")
        service = services.get("tailscale")
        if not isinstance(service, Mapping):
            raise ValueError("base Compose manifest has no tailscale service")

        # Reuse the installer's canonical sidecar renderer solely as a pure
        # expected-value source.  No files, images, or Docker resources are
        # touched by this dry-run helper.
        installer = ContainerInstaller(self.state, state_path=self.state.state_path, dry_run=True)
        installer._install_uuid = self.install_uuid
        installer._special_container_names["tailscale"] = container_name(
            self.install_uuid, "tailscale"
        )
        expected = installer._tailscale_service()
        if self._normalize_compose_value(service) != self._normalize_compose_value(expected):
            raise ValueError("base Compose tailscale service is not the exact install service")
        return {"tailscale": dict(service)}

    @staticmethod
    def _lexical(path: Path) -> Path:
        """Absolute path without resolving symlink components."""

        return Path(os.path.abspath(os.path.expanduser(os.fspath(path))))

    @staticmethod
    def _check_tree(path: Path, *, allow_missing: bool = True) -> None:
        path = Path(path)
        current = path
        while True:
            if current.is_symlink():
                raise ValueError(f"refusing symlink path: {current}")
            if current == current.parent:
                break
            current = current.parent
        if not allow_missing and not path.exists():
            raise FileNotFoundError(path)

    def _check_prefix(self) -> None:
        self._check_tree(self.prefix)
        if self.prefix.exists() and not self.prefix.is_dir():
            raise ValueError(f"install prefix is not a directory: {self.prefix}")
        for path in (self.prefix / "instances", self.prefix / "containers"):
            if path.is_symlink() or (path.exists() and not path.is_dir()):
                raise ValueError(f"runtime root is not a real directory: {path}")
        if self.compose.is_symlink() or (self.compose.exists() and not self.compose.is_file()):
            raise ValueError(f"Compose manifest is not a regular file: {self.compose}")

    @contextmanager
    def _lock_file(self, path: Path) -> Iterator[None]:
        self._check_tree(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.is_symlink():
            raise ValueError(f"refusing symlink lock: {path}")
        lock_key = str(path)
        with _THREAD_LOCKS_GUARD:
            thread_lock = _THREAD_LOCKS.setdefault(lock_key, threading.Lock())
        thread_lock.acquire()
        fd: int | None = None
        try:
            fd = os.open(
                path,
                os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
                0o600,
            )
        except OSError as exc:
            thread_lock.release()
            raise ValueError(f"unsafe lock path: {path}") from exc
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ValueError(f"lock must be a regular file: {path}")
            stream = os.fdopen(fd, "a+b")
            fd = None
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
                stream.close()
        except BaseException:
            if fd is not None:
                os.close(fd)
            raise
        finally:
            thread_lock.release()

    def _validate_host_lease(self, system_id: str, token: str) -> None:
        """Validate the lease held by the host-side Docker removal bridge.

        The bridge has to keep the install and system locks while Docker is
        touched, but the registry transaction runs in a tools container that
        cannot inherit those file descriptors.  A private, one-shot lease
        record is the handoff proof.  A missing, malformed, stale, or foreign
        record fails closed; the bridge removes it only after the transaction
        returns.
        """

        if _HOST_LEASE_TOKEN.fullmatch(token or "") is None:
            raise PermissionError("host instance removal lease is invalid")
        path = self.lock_root / f"{system_id}.lease"
        try:
            self._check_tree(path, allow_missing=False)
        except (FileNotFoundError, ValueError) as exc:
            raise PermissionError("host instance removal lease is unavailable") from exc
        if path.is_symlink() or not path.is_file():
            raise PermissionError("host instance removal lease is not a regular file")
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        except OSError as exc:
            raise PermissionError("host instance removal lease cannot be opened") from exc
        try:
            info = os.fstat(fd)
        except OSError as exc:
            os.close(fd)
            raise PermissionError("host instance removal lease metadata cannot be read") from exc
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_nlink != 1
            or stat.S_IMODE(info.st_mode) != 0o600
            or info.st_uid != os.getuid()
        ):
            os.close(fd)
            raise PermissionError("host instance removal lease has unsafe metadata")
        try:
            with os.fdopen(fd, "r", encoding="utf-8") as stream:
                fd = -1
                payload = json.load(stream)
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise PermissionError("host instance removal lease is malformed") from exc
        finally:
            if fd != -1:
                os.close(fd)
        if not isinstance(payload, Mapping):
            raise PermissionError("host instance removal lease is malformed")
        if (
            payload.get("version") != 1
            or payload.get("token") != token
            or payload.get("system_id") != system_id
            or payload.get("install_uuid") != self.install_uuid
        ):
            raise PermissionError("host instance removal lease does not match this install")

    @contextmanager
    def _locks(self, system_id: str, *, lease_token: str | None = None) -> Iterator[None]:
        self._check_prefix()
        if lease_token is not None:
            self._validate_host_lease(system_id, lease_token)
            yield
            return
        with self._lock_file(self.install_lock):
            # Keep this lock outside the replaceable system directory.  A
            # held lock therefore remains the lock for the same system across
            # a transaction that replaces or removes its directory.
            system_lock = self.lock_root / f"{system_id}.lock"
            with self._lock_file(system_lock):
                yield

    @staticmethod
    def _copy_tree(source: Path, destination: Path) -> None:
        """Copy a tree while rejecting every link/device entry."""

        source = Path(source)
        if source.is_symlink():
            raise ValueError(f"source tree contains a symlink: {source}")
        if not source.is_dir():
            raise FileNotFoundError(source)
        destination.mkdir(parents=True, exist_ok=True)
        for current, directories, files in os.walk(source, followlinks=False):
            current_path = Path(current)
            relative = current_path.relative_to(source)
            target = destination / relative
            target.mkdir(parents=True, exist_ok=True)
            for name in (*directories, *files):
                path = current_path / name
                mode = path.lstat().st_mode
                if stat.S_ISLNK(mode):
                    raise ValueError(f"source tree contains a symlink: {path}")
                if not (stat.S_ISDIR(mode) or stat.S_ISREG(mode)):
                    raise ValueError(f"source tree contains unsupported entry: {path}")
            for name in files:
                shutil.copy2(current_path / name, target / name)

    @staticmethod
    def _write_json(path: Path, value: Mapping[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False) as stream:
            json.dump(dict(value), stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
            temporary = Path(stream.name)
        temporary.chmod(0o600)
        os.replace(temporary, path)

    @staticmethod
    def _replace_prefix(value: Any, old: str, new: str) -> Any:
        """Point staged endpoint paths at their eventual installed prefix."""

        if isinstance(value, str):
            return value.replace(old, new)
        if isinstance(value, dict):
            return {key: InstanceRuntime._replace_prefix(item, old, new) for key, item in value.items()}
        if isinstance(value, list):
            return [InstanceRuntime._replace_prefix(item, old, new) for item in value]
        if isinstance(value, tuple):
            return tuple(InstanceRuntime._replace_prefix(item, old, new) for item in value)
        return value

    def _read_instances(self) -> dict[str, InstanceState]:
        root = self.prefix / "instances"
        self._check_tree(root)
        if not root.exists():
            return {}
        if not root.is_dir():
            raise ValueError(f"instances is not a directory: {root}")
        result: dict[str, InstanceState] = {}
        for child in sorted(root.iterdir(), key=lambda path: path.name):
            if child == self.lock_root:
                # The lock metadata is inside the managed instances subtree,
                # but is not an instance and must never be traversed.
                if child.is_symlink() or not child.is_dir():
                    raise ValueError(f"invalid instance lock metadata directory: {child}")
                continue
            if child.is_symlink() or not child.is_dir() or not _SYSTEM.fullmatch(child.name):
                raise ValueError(f"invalid instance directory: {child}")
            state_path = child / "state.json"
            if not os.path.lexists(state_path):
                # A removed instance may retain runtime-owned security/log
                # snapshots.  Those trees are deliberately not traversed or
                # deleted, and do not constitute a registered instance.
                entries = tuple(item.name for item in child.iterdir())
                if entries and set(entries).issubset({"security", "logs", "secrets"}):
                    continue
            self._check_tree(state_path, allow_missing=False)
            if state_path.is_symlink() or not state_path.is_file():
                raise ValueError(f"invalid instance state file: {state_path}")
            try:
                instance = InstanceState.from_dict(json.loads(state_path.read_text(encoding="utf-8")))
            except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                raise ValueError(f"malformed instance state: {state_path}") from exc
            if instance.system_id != child.name:
                raise ValueError("instance system_id does not match its directory")
            instance = self._normalize_instance(instance)
            if instance.system_id in result:
                raise ValueError("duplicate instance system_id")
            result[instance.system_id] = instance
        return result

    def _check_runtime_owned_trees(self, instances: Mapping[str, InstanceState]) -> None:
        """Reject path redirections before wrappers can reference them."""

        for system in instances:
            root = self.prefix / "instances" / system
            for name in ("logs", "secrets"):
                path = root / name
                if path.is_symlink() or (path.exists() and not path.is_dir()):
                    raise ValueError(f"instance runtime tree is not a real directory: {path}")
            security = root / "security"
            if security.is_symlink() or (security.exists() and not security.is_dir()):
                raise ValueError(f"instance security tree is not a real directory: {security}")

    def _load_release(self, instance: InstanceState) -> ReleaseManifest:
        instance.validate()
        path = self.prefix / "releases" / instance.release_key
        self._check_tree(path, allow_missing=False)
        release = load_release(path)
        if release_key(release) != instance.release_key or release.install_uuid != self.install_uuid:
            raise ValueError("release is not pinned to this installation")
        return release

    def _validate_release_set(self, instances: Mapping[str, InstanceState]) -> dict[str, ReleaseManifest]:
        releases: dict[str, ReleaseManifest] = {}
        domains: dict[int, str] = {}
        turn_ranges: list[tuple[str, int, int, str]] = []
        turn_credentials: dict[str, str] = {}
        # The install-level TURN settings describe the legacy aggregate, not
        # an active service in this instance namespace.  Reserving those
        # historical ports here caused a false collision for the first
        # scoped instance even though its own Compose services are isolated.
        # Only registered scoped instances share this port namespace and must
        # therefore be disjoint from one another.
        for system, instance in instances.items():
            if system != instance.system_id:
                raise ValueError("instance system_id mismatch")
            if instance.domain_id in domains and domains[instance.domain_id] != system:
                raise ValueError(f"domain_id collision with system {domains[instance.domain_id]!r}")
            domains[instance.domain_id] = system
            turn = scoped_turn_settings(instance, self.install_uuid)
            if turn.mode == "external":
                credential = str(Path(turn.credential_file).expanduser().absolute())
                if self.state.turn.mode == "external" and self.state.turn.credential_file:
                    install_credential = str(Path(self.state.turn.credential_file).expanduser().absolute())
                    if credential == install_credential:
                        raise ValueError(
                            f"external TURN credential for {system!r} must not reuse the install credential"
                        )
                owner = turn_credentials.get(credential)
                if owner is not None and owner != system:
                    raise ValueError(
                        f"external TURN credential path is shared by systems {owner!r} and {system!r}"
                    )
                turn_credentials[credential] = system
            if turn.mode == "managed":
                listen = turn.effective_listen_port
                relay_min = turn.effective_relay_min_port
                relay_max = turn.effective_relay_max_port
                # The listener must not fall in any relay range, and relay
                # ranges/listeners must not overlap across systems sharing the
                # host network namespace.  Refuse ambiguous allocations before
                # touching the registry or Compose file.
                ranges = (
                    (listen, listen, "listen"),
                    (relay_min, relay_max, "relay"),
                )
                for first, last, kind in ranges:
                    for other_system, other_first, other_last, other_kind in turn_ranges:
                        if first <= other_last and other_first <= last:
                            raise ValueError(
                                f"TURN {kind} port range for {system!r} collides with "
                                f"{other_kind} range for {other_system!r}"
                            )
                    turn_ranges.append((system, first, last, kind))
            releases[system] = self._load_release(instance)
        return releases

    def _normalize_instance(self, instance: InstanceState) -> InstanceState:
        """Bind managed TURN to this instance's private secret namespace."""

        if instance.turn.mode != "managed":
            return instance
        secret = instance_turn_secret_path(self.prefix, instance.system_id)
        return replace(instance, turn=replace(instance.turn, secret_file=str(secret)))

    @staticmethod
    def _validate_turn_secret(path: Path) -> None:
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"instance TURN secret is not a regular file: {path}")
        if stat.S_IMODE(path.stat().st_mode) != 0o600:
            raise ValueError(f"instance TURN secret must have mode 0600: {path}")
        payload = path.read_bytes()
        if not payload.strip() or len(payload) > 4096:
            raise ValueError("instance TURN secret must contain 1..4096 non-whitespace bytes")

    def _stage_instance_turn_secret(self, stage: Path, instance: InstanceState) -> None:
        """Create a new secret in staging; replacement never overwrites one."""

        if instance.turn.mode != "managed":
            return
        destination = stage / "instances" / instance.system_id / "secrets" / "turn.secret"
        destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=destination.parent,
            prefix=".turn.secret.", delete=False,
        ) as handle:
            handle.write(secrets.token_urlsafe(48) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        os.replace(temporary, destination)

    def _check_security_replacement(
        self,
        previous: InstanceState,
        replacement: InstanceState,
        *,
        has_security_result: bool,
    ) -> None:
        if previous.release_key == replacement.release_key:
            return
        if has_security_result and replacement.security_profile == "sros2":
            return
        current = self.prefix / "instances" / previous.system_id / "security" / "current"
        if os.path.lexists(current):
            raise ValueError(
                "cannot replace an active security-bound instance with a different release"
            )

    @staticmethod
    def _instance_services(instance: InstanceState) -> tuple[str, ...]:
        services = tuple(
            sorted(
                service_key(instance.system_id, endpoint.endpoint_id)
                for endpoint in instance.endpoints
            )
        )
        if instance.turn.mode == "managed":
            services += (turn_service_key(instance.system_id),)
        return services

    def _verify_stopped(
        self,
        instance: InstanceState,
        verifier: Callable[[Path, str, tuple[str, ...]], Mapping[str, bool]] | None,
    ) -> tuple[str, ...]:
        if not callable(verifier):
            raise PermissionError(
                "removal requires a read-only verifier for every target service"
            )
        services = self._instance_services(instance)
        try:
            result = verifier(self.compose, self.project, services)
        except Exception as exc:
            raise PermissionError("target service stop verification failed") from exc
        if (
            not isinstance(result, Mapping)
            or set(result) != set(services)
            or any(type(service) is not str for service in result)
            or any(type(status) is not bool for status in result.values())
            or any(status is not False for status in result.values())
        ):
            raise PermissionError(
                "stop verifier must return exact service keys with every running value false"
            )
        return services

    def _security_result_parts(self, result: object, instance: InstanceState) -> tuple[Path, str]:
        """Extract a separately staged, already-authorized security generation.

        ``stage_instance_security`` returns an ``InstanceSecurityResult`` whose
        ``root`` is the immutable generation directory.  Keeping this small
        duck-typed boundary lets the security publisher remain independent of
        instance registration while still making a new instance's first
        ``current`` link visible to preparation inside the transaction.
        """

        # ``pathlib.Path.root`` is the filesystem anchor (for example ``/``),
        # not an InstanceSecurityResult field.  Treat path-like inputs first
        # so compensating selection of an existing generation cannot be
        # mistaken for that attribute.
        if isinstance(result, (str, os.PathLike)):
            source = result
            generation = None
        else:
            source = getattr(result, "root", result)
            generation = getattr(result, "generation", None)
        if not isinstance(source, (str, os.PathLike)):
            raise ValueError("security result must identify a generation directory")
        source_path = InstanceRuntime._lexical(Path(source))
        if generation is None:
            generation = source_path.name
        if not isinstance(generation, str) or _SECURITY_GENERATION.fullmatch(generation) is None:
            raise ValueError("security generation is not a safe identifier")
        if source_path.name != generation:
            raise ValueError("security result generation does not match its directory")
        validated = validate_instance_security_generation(
            source_path,
            self.install_uuid,
            instance,
            generation,
        )
        return validated.root, validated.generation

    def _stage_security_result(
        self,
        stage: Path,
        instance: InstanceState,
        result: object | None,
        *,
        previous: InstanceState | None = None,
    ) -> bool:
        if result is None:
            return False
        if instance.security_profile != "sros2":
            raise ValueError("security result requires the sros2 profile")
        source, generation = self._security_result_parts(result, instance)
        destination = stage / "instances" / instance.system_id / "security"
        if destination.exists() or destination.is_symlink():
            raise ValueError("staged security destination already exists")
        generations = destination / "generations"
        generations.mkdir(mode=0o700, parents=True)

        # A replacement keeps the exact previously active generation beside
        # the candidate.  Besides retaining a local audit/rollback target,
        # this lets the multi-host manager compensate a later-host failure by
        # selecting the old generation in another atomic replace.  Unknown or
        # inactive history is deliberately not copied.
        existing = (
            self.prefix
            / "instances"
            / instance.system_id
            / "security"
        )
        if existing.exists() or existing.is_symlink():
            if previous is None:
                raise ValueError(
                    "existing security tree requires an enrolled previous instance"
                )
            if existing.is_symlink() or not existing.is_dir():
                raise ValueError("existing security target is not a real directory")
            current = existing / "current"
            if not current.is_symlink():
                raise ValueError("existing security current is not a symlink")
            current_target = current.resolve()
            existing_generations = (existing / "generations").resolve()
            if existing_generations not in current_target.parents:
                raise ValueError("existing security current escapes generations")
            if current_target.name != previous.security_generation:
                raise ValueError(
                    "existing security generation does not match previous state"
                )
            validate_instance_security_generation(
                current_target,
                self.install_uuid,
                previous,
                previous.security_generation,
            )
            self._copy_tree(
                current_target,
                generations / previous.security_generation,
            )

        published = generations / generation
        if not published.exists():
            self._copy_tree(source, published)
        # ``copytree``-style directory creation observes the process umask.
        # Security mounts instead retain a deterministic private boundary.
        destination.chmod(0o700)
        generations.chmod(0o700)
        for current, directories, files in os.walk(destination, followlinks=False):
            current_path = Path(current)
            current_path.chmod(0o700)
            for name in directories:
                (current_path / name).chmod(0o700)
            for name in files:
                (current_path / name).chmod(0o600)
        os.symlink(Path("generations") / generation, destination / "current")
        return True

    def _security_views(
        self,
        instance: InstanceState,
        prefix: Path,
    ) -> dict[str, tuple[Path, str]] | None:
        if instance.security_profile != "sros2":
            return None
        views: dict[str, tuple[Path, str]] = {}
        for endpoint in instance.endpoints:
            keystore = active_instance_security_path(
                prefix,
                self.install_uuid,
                instance.system_id,
                endpoint.endpoint_id,
            )
            views[endpoint.role] = (
                keystore.resolve(),
                f"/elesim/{instance.system_id}/{endpoint.role}/"
                f"{endpoint.endpoint_id.replace('-', '_')[:63]}",
            )
        return views

    def _render_status_wrapper(
        self,
        services: tuple[str, ...],
        instance: InstanceState,
        *,
        guard: str = "",
    ) -> str:
        # The general status helper intentionally shows the whole Compose
        # project.  An instance status wrapper must not reveal or inspect other
        # systems, so this deliberately uses an exact service list.
        command = f"docker compose -p {shlex.quote(self.project)} -f {shlex.quote(str(self.compose))}"
        names = " ".join(shlex.quote(service) for service in services)
        roles_by_service = {
            service_key(instance.system_id, endpoint.endpoint_id): endpoint.role
            for endpoint in instance.endpoints
        }
        if instance.turn.mode == "managed":
            roles_by_service[turn_service_key(instance.system_id)] = "coturn"
        cases = "\n".join(
            f"    {shlex.quote(service)}) printf '%s\\n' {shlex.quote(roles_by_service[service])} ;;"
            for service in services
        )
        return (
            "#!/usr/bin/env bash\nset -euo pipefail\n"
            + guard
            # Keep the output machine-readable for the connection manager:
            # one role per running endpoint, with no project-wide listing.
            + f"running_services=\"$({command} ps --status running --services {names})\"\n"
            + "while IFS= read -r service; do\n"
            + "  [[ -z $service ]] && continue\n"
            + "  case \"$service\" in\n"
            + cases
            + "\n    *) printf 'unexpected instance service: %s\\n' \"$service\" >&2; exit 65 ;;\n"
            + "  esac\n"
            + "done <<< \"$running_services\"\n"
        )

    def _write_wrappers(self, target: Path, instance: InstanceState, services: tuple[str, ...]) -> None:
        bin_dir = target / "bin"
        bin_dir.mkdir(parents=True, exist_ok=True)
        logs_root = self.prefix / "instances" / instance.system_id / "logs"
        backend_guard = _docker_backend_guard(self.state.container_network)
        owner_guard = compose_owner_guard(
            self.compose,
            project=self.project,
            containers=tuple(container_name(self.install_uuid, service) for service in services),
        )
        if self.state.container_network.uses_tailscale_sidecar:
            # The sidecar is created by the install-level Compose file and is
            # shared by every instance.  Guard its exact fixed identity, but
            # accept either Compose file because Docker records the file that
            # created the already-running infrastructure container.
            owner_guard += compose_owner_guard(
                self.compose,
                project=self.project,
                containers=(container_name(self.install_uuid, "tailscale"),),
                alternate_composes=(self.base_compose,),
            )
        guard = backend_guard + owner_guard
        down = _runtime_down_wrapper(
            compose=self.compose,
            logs_root=logs_root,
            services=services,
            archive_enabled=self.state.runtime_text_logs.enabled,
            guard=guard,
            project=self.project,
            instance_scoped=True,
        )
        logs = _runtime_logs_wrapper(
            compose=self.compose,
            logs_root=logs_root,
            services=services,
            archive_enabled=self.state.runtime_text_logs.enabled,
            guard=guard,
            project=self.project,
        )
        rendered = " ".join(shlex.quote(service) for service in services)
        up = (
            "#!/usr/bin/env bash\nset -euo pipefail\n"
            + guard
            + "if (( $# != 0 )) && [[ $1 != --no-build ]]; then\n"
            "  printf 'usage: elesim-instance-up [--no-build]\\n' >&2; exit 64\nfi\n"
            "if (( $# == 1 )); then shift; fi\n"
            f"exec docker compose -p {shlex.quote(self.project)} -f {shlex.quote(str(self.compose))} up -d --no-build {rendered}\n"
        )
        operation_lock = self.prefix / "instances" / ".locks" / f"{instance.system_id}.lock"
        maintenance_root = self.prefix / "maintenance"
        up = self._with_operation_lock(up, operation_lock, maintenance_root)
        down = self._with_operation_lock(down, operation_lock, maintenance_root)
        write_executable(bin_dir / "up", up)
        write_executable(bin_dir / "down", down)
        write_executable(bin_dir / "logs", logs)
        write_executable(bin_dir / "status", self._render_status_wrapper(services, instance, guard=guard))

    @staticmethod
    def _with_operation_lock(
        script: str,
        lock_path: Path,
        maintenance_root: Path,
    ) -> str:
        """Place the host lock handoff before an instance wrapper body."""

        lines = script.splitlines()
        if lines and lines[0].startswith("#!"):
            lines = lines[1:]
        return "\n".join(
            [
                "#!/usr/bin/env bash",
                "set -euo pipefail",
                *render_lock_preamble(
                    lock_path=lock_path,
                    maintenance_root=maintenance_root,
                ),
                *lines,
                "",
            ]
        )

    def _prepare(
        self,
        staged: Path,
        instances: Mapping[str, InstanceState],
        releases: Mapping[str, ReleaseManifest],
        *,
        target_system: str,
        remove: bool,
        security_result: object | None = None,
        previous: InstanceState | None = None,
    ) -> Path:
        staged_instances = staged / "instances"
        staged_instances.mkdir(mode=0o700, parents=True)
        # Only generated state/endpoints/bin are committed below.  Immutable
        # releases and existing security are read from their real prefix.
        # Logs and secrets are never traversed.
        if not remove and security_result is not None:
            self._stage_security_result(
                staged,
                instances[target_system],
                security_result,
                previous=previous,
            )
        if not remove:
            self._stage_instance_turn_secret(staged, instances[target_system])
        groups: dict[str, Mapping[str, Mapping[str, object]]] = {}
        for system, instance in sorted(instances.items()):
            views = (
                self._security_views(instance, staged)
                if security_result is not None and system == target_system
                else None
            )
            group = prepare_instance_services(
                self.state,
                self.install_uuid,
                instance,
                releases[system],
                output_prefix=staged,
                security_views=views,
            )
            groups[system] = group
            services = tuple(sorted(group))
            target = staged_instances / system
            self._write_json(target / "state.json", instance.to_dict())
            self._write_wrappers(target, instance, services)
        compose = aggregate_compose(
            self.install_uuid,
            groups,
            self._shared_infrastructure,
        )
        compose = self._replace_prefix(compose, str(staged), str(self.prefix))
        containers = staged / "containers"
        containers.mkdir(mode=0o700, exist_ok=True)
        output = containers / "compose.instances.yaml"
        output.write_text(yaml.safe_dump(compose, sort_keys=False, allow_unicode=True), encoding="utf-8")
        with output.open("rb") as stream:
            os.fsync(stream.fileno())
        return output

    def _commit(
        self,
        staged: Path,
        system: str,
        *,
        remove: bool,
        security_result: bool = False,
        fail: Callable[[str], None] | None = None,
    ) -> None:
        def injected(step: str) -> None:
            if fail is not None:
                fail(step)

        root = self.prefix / "instances"
        root.mkdir(mode=0o700, exist_ok=True)
        target = root / system
        staged_target = staged / "instances" / system
        compose = self.compose
        staged_compose = staged / "containers" / "compose.instances.yaml"
        compose_backup = compose.with_name(f".{compose.name}.rollback-{os.getpid()}")
        self._check_tree(target)
        self._check_tree(compose)
        if compose_backup.exists() or compose_backup.is_symlink():
            raise ValueError("transaction rollback path already exists")
        mutable = ("state.json", "endpoints", "bin")
        swaps: list[
            tuple[
                Path,
                Path,
                tuple[int, int] | None,
                tuple[int, int] | None,
                Mapping[Path, tuple[int, int]],
                Mapping[Path, tuple[int, int]],
            ]
        ] = []
        compose_moved = False
        compose_identity: tuple[int, int] | None = None
        compose_backup_identity: tuple[int, int] | None = None
        target_existed = target.exists()
        security_published_identity: tuple[int, int] | None = None
        security_owned: dict[Path, tuple[int, int]] = {}
        security_backup = target / f".security.rollback-{os.getpid()}"
        security_backup_identity: tuple[int, int] | None = None
        security_backup_owned: dict[Path, tuple[int, int]] = {}
        turn_secret_published_identity: tuple[int, int] | None = None
        turn_secret_owned: dict[Path, tuple[int, int]] = {}
        security_target = target / "security"
        try:
            injected("before-commit")
            if target.is_symlink() or (target.exists() and not target.is_dir()):
                raise ValueError(f"instance target is not a real directory: {target}")
            if not remove:
                target.mkdir(mode=0o700, exist_ok=True)
            elif not target.exists():
                raise FileNotFoundError(target)
            if not remove:
                staged_secret = staged_target / "secrets"
                secret_target = target / "secrets"
                if secret_target.is_symlink() or (
                    secret_target.exists() and not secret_target.is_dir()
                ):
                    raise ValueError(f"instance secrets path is not a real directory: {secret_target}")
                if secret_target.exists():
                    # A replace must retain the established secret byte-for-byte.
                    existing = secret_target / "turn.secret"
                    if not existing.exists() and not existing.is_symlink():
                        raise ValueError("existing managed TURN secret is missing")
                    if existing.is_symlink():
                        raise ValueError("existing managed TURN secret is a symlink")
                    if staged_secret.exists():
                        self._remove_owned_tree(staged_secret, self._owned_tree(staged_secret))
                elif staged_secret.exists():
                    os.replace(staged_secret, secret_target)
                    turn_secret_published_identity = self._identity(secret_target)
                    turn_secret_owned = self._owned_tree(secret_target)
            if security_result:
                staged_security = staged_target / "security"
                if security_backup.exists() or security_backup.is_symlink():
                    raise ValueError("security rollback path already exists")
                if staged_security.is_symlink() or not staged_security.is_dir():
                    raise ValueError("staged security result is not a directory")
                if security_target.exists() or security_target.is_symlink():
                    if security_target.is_symlink() or not security_target.is_dir():
                        raise ValueError("security target is not a real directory")
                    os.replace(security_target, security_backup)
                    security_backup_identity = self._identity(security_backup)
                    security_backup_owned = self._owned_tree(security_backup)
                os.replace(staged_security, security_target)
                security_published_identity = self._identity(security_target)
                security_owned = self._owned_tree(security_target)
            for name in mutable:
                target_child = target / name
                staged_child = staged_target / name
                backup = target / f".{name}.rollback-{os.getpid()}"
                if backup.exists() or backup.is_symlink():
                    raise ValueError("transaction rollback path already exists")
                if target_child.is_symlink() or (target_child.exists() and not (target_child.is_file() if name == "state.json" else target_child.is_dir())):
                    raise ValueError(f"instance mutable path is not a real entry: {target_child}")
                had_original = os.path.lexists(target_child)
                if had_original:
                    os.replace(target_child, backup)
                backup_identity = self._identity(backup) if had_original else None
                backup_owned = self._owned_tree(backup) if had_original else {}
                published_identity: tuple[int, int] | None = None
                published_owned: Mapping[Path, tuple[int, int]] = {}
                swaps.append(
                    (
                        target_child,
                        backup,
                        published_identity,
                        backup_identity,
                        published_owned,
                        backup_owned,
                    )
                )
                if not remove:
                    os.replace(staged_child, target_child)
                    published_identity = self._identity(target_child)
                    published_owned = self._owned_tree(target_child)
                    swaps[-1] = (
                        target_child,
                        backup,
                        published_identity,
                        backup_identity,
                        published_owned,
                        backup_owned,
                    )
            injected("after-instance")
            compose.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            if compose.exists() or compose.is_symlink():
                os.replace(compose, compose_backup)
                compose_moved = True
                compose_backup_identity = self._identity(compose_backup)
            os.replace(staged_compose, compose)
            compose_identity = self._identity(compose)
            injected("after-compose")
            directory_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
            for target_child, backup, _, backup_identity, _, backup_owned in swaps:
                if backup_identity is not None and self._identity(backup) == backup_identity:
                    if backup.is_dir():
                        self._remove_owned_tree(backup, backup_owned)
                    else:
                        backup.unlink()
            if compose_backup_identity is not None and self._identity(compose_backup) == compose_backup_identity:
                compose_backup.unlink()
            if (
                security_backup_identity is not None
                and self._identity(security_backup) == security_backup_identity
            ):
                self._remove_owned_tree(
                    security_backup,
                    security_backup_owned,
                )
            if remove and not tuple(target.iterdir()):
                target.rmdir()
        except BaseException:
            # Restore only the exact paths this transaction moved.  No broad
            # cleanup or Docker operation is performed.
            if compose_identity is not None and self._identity(compose) == compose_identity:
                compose.unlink()
            if compose_moved and compose_backup_identity is not None and self._identity(compose_backup) == compose_backup_identity and not os.path.lexists(compose):
                os.replace(compose_backup, compose)
            for (
                target_child,
                backup,
                published_identity,
                backup_identity,
                published_owned,
                backup_owned,
            ) in reversed(swaps):
                if published_identity is not None and self._identity(target_child) == published_identity:
                    if target_child.is_dir():
                        self._remove_owned_tree(target_child, published_owned)
                    else:
                        target_child.unlink()
                if backup_identity is not None and self._identity(backup) == backup_identity:
                    if not os.path.lexists(target_child):
                        os.replace(backup, target_child)
                    elif target_child.is_dir() and not target_child.is_symlink():
                        self._restore_tree_without_overwrite(backup, target_child)
            if (
                security_published_identity is not None
                and self._identity(security_target) == security_published_identity
            ):
                self._remove_owned_tree(security_target, security_owned)
            if (
                security_backup_identity is not None
                and self._identity(security_backup) == security_backup_identity
                and not os.path.lexists(security_target)
            ):
                os.replace(security_backup, security_target)
            if (
                turn_secret_published_identity is not None
                and self._identity(target / "secrets") == turn_secret_published_identity
            ):
                self._remove_owned_tree(target / "secrets", turn_secret_owned)
            if remove and target.exists() and target.is_dir() and not tuple(target.iterdir()):
                target.rmdir()
            elif not remove and not target_existed and target.exists() and not tuple(target.iterdir()):
                target.rmdir()
            raise

    @staticmethod
    def _identity(path: Path) -> tuple[int, int] | None:
        try:
            info = path.lstat()
        except FileNotFoundError:
            return None
        return (info.st_dev, info.st_ino)

    @classmethod
    def _owned_tree(cls, root: Path) -> dict[Path, tuple[int, int]]:
        """Snapshot transaction-owned entries without following links."""

        owned: dict[Path, tuple[int, int]] = {}
        if not root.exists() and not root.is_symlink():
            return owned
        for current, directories, files in os.walk(root, followlinks=False):
            current_path = Path(current)
            for name in (*directories, *files):
                path = current_path / name
                identity = cls._identity(path)
                if identity is not None:
                    owned[path] = identity
        return owned

    @classmethod
    def _remove_owned_tree(cls, root: Path, owned: Mapping[Path, tuple[int, int]]) -> None:
        """Rollback only entries whose inode was published by this transaction."""

        for path, identity in sorted(owned.items(), key=lambda item: len(item[0].parts), reverse=True):
            if cls._identity(path) != identity:
                continue
            if path.is_dir() and not path.is_symlink():
                try:
                    path.rmdir()
                except OSError:
                    pass
            else:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
        if cls._identity(root) is not None and not tuple(root.iterdir()):
            try:
                root.rmdir()
            except OSError:
                pass

    @classmethod
    def _restore_tree_without_overwrite(cls, source: Path, destination: Path) -> None:
        """Merge a moved backup back, leaving a concurrent winner untouched."""

        if not source.is_dir() or source.is_symlink():
            if not os.path.lexists(destination):
                os.replace(source, destination)
            return
        if not destination.exists():
            os.replace(source, destination)
            return
        if destination.is_symlink() or not destination.is_dir():
            return
        for entry in sorted(tuple(source.iterdir()), key=lambda path: path.name):
            target = destination / entry.name
            if not os.path.lexists(target):
                os.replace(entry, target)
            elif entry.is_dir() and not entry.is_symlink() and target.is_dir() and not target.is_symlink():
                cls._restore_tree_without_overwrite(entry, target)
        try:
            source.rmdir()
        except OSError:
            pass

    def register(
        self,
        instance: InstanceState,
        release: ReleaseManifest | None = None,
        *,
        replace_existing: bool = False,
        security_result: object | None = None,
        fail: Callable[[str], None] | None = None,
    ) -> InstanceState:
        """Register or replace one instance, publishing the full aggregate."""

        instance = self._normalize_instance(instance)
        instance.validate()
        if release is not None and release_key(release) != instance.release_key:
            raise ValueError("supplied release does not match instance release_key")
        with self._locks(instance.system_id):
            current = self._read_instances()
            if instance.system_id in current and not replace_existing:
                raise FileExistsError(instance.system_id)
            if instance.system_id in current and replace_existing:
                self._check_security_replacement(
                    current[instance.system_id],
                    instance,
                    has_security_result=security_result is not None,
                )
            candidate = dict(current)
            previous = current.get(instance.system_id)
            candidate[instance.system_id] = instance
            self._check_runtime_owned_trees(candidate)
            existing_secret = (
                self.prefix / "instances" / instance.system_id / "secrets" / "turn.secret"
            )
            if instance.turn.mode == "managed" and existing_secret.exists():
                self._validate_turn_secret(existing_secret)
            releases = self._validate_release_set(candidate)
            stage = Path(tempfile.mkdtemp(prefix=f".instance-{instance.system_id}-", dir=self.prefix.parent))
            try:
                self._prepare(
                    stage,
                    candidate,
                    releases,
                    target_system=instance.system_id,
                    remove=False,
                    security_result=security_result,
                    previous=previous,
                )
                if self.ownership_manifest is not None:
                    append_instance_docker_ownership(
                        manifest_path=self.ownership_manifest,
                        install_uuid=self.install_uuid,
                        project=self.project,
                        docker_context=self.state.container_network.docker_context,
                        docker_engine_id=self.state.container_network.docker_engine_id,
                        instance=instance,
                    )
                self._commit(
                    stage,
                    instance.system_id,
                    remove=False,
                    security_result=security_result is not None,
                    fail=fail,
                )
            finally:
                if stage.exists():
                    shutil.rmtree(stage)
        return instance

    def replace(
        self,
        instance: InstanceState,
        release: ReleaseManifest | None = None,
        *,
        security_result: object | None = None,
        fail: Callable[[str], None] | None = None,
    ) -> InstanceState:
        return self.register(
            instance,
            release,
            replace_existing=True,
            security_result=security_result,
            fail=fail,
        )

    def remove(
        self,
        system_id: str,
        *,
        verifier: Callable[[Path, str, tuple[str, ...]], Mapping[str, bool]] | None = None,
        host_lease: str | None = None,
        fail: Callable[[str], None] | None = None,
    ) -> None:
        """Remove one instance after exact verification or a host lease.

        Normal callers must prove every target service is absent.  The
        host-side bridge may instead supply its one-shot lease after it has
        removed the exact owned containers while holding the shared locks.
        """

        if not isinstance(system_id, str) or _SYSTEM.fullmatch(system_id) is None:
            raise ValueError("system_id must be a safe identifier")
        with self._locks(system_id, lease_token=host_lease):
            current = self._read_instances()
            if system_id not in current:
                raise FileNotFoundError(system_id)
            if host_lease is None:
                self._verify_stopped(current[system_id], verifier)
            candidate = dict(current)
            del candidate[system_id]
            self._check_runtime_owned_trees(current)
            releases = self._validate_release_set(candidate)
            stage = Path(tempfile.mkdtemp(prefix=f".instance-{system_id}-", dir=self.prefix.parent))
            try:
                self._prepare(
                    stage,
                    candidate,
                    releases,
                    target_system=system_id,
                    remove=True,
                )
                self._commit(stage, system_id, remove=True, fail=fail)
            finally:
                if stage.exists():
                    shutil.rmtree(stage)

    def rotate_security(
        self,
        instance: InstanceState,
        security_result: object,
        *,
        fail: Callable[[str], None] | None = None,
    ) -> InstanceState:
        """Atomically activate a new SROS2 generation for one instance.

        Rotation is deliberately local to ``instance.system_id``.  It copies
        a previously validated generation into that system's immutable
        generation store, switches only that system's ``current`` link, and
        records the active generation in its state file.  No Compose or
        Docker operation is performed here; callers own the stop/restart
        boundary.  A failure at any point restores both the old link and the
        old state and removes only the generation created by this operation.
        """

        instance.validate()
        if instance.security_profile != "sros2":
            raise ValueError("security rotation requires the sros2 profile")
        with self._locks(instance.system_id):
            current_instances = self._read_instances()
            previous = current_instances.get(instance.system_id)
            if previous is None:
                raise FileNotFoundError(instance.system_id)
            # A security rotation must not smuggle a release, endpoint, DDS,
            # or profile change into the transaction.  Those remain the
            # explicit replace operation and may require regenerated config.
            if (
                previous.release_key != instance.release_key
                or previous.endpoints != instance.endpoints
                or previous.domain_id != instance.domain_id
                or previous.rmw_implementation != instance.rmw_implementation
                or previous.discovery_mode != instance.discovery_mode
                or previous.static_peers != instance.static_peers
                or previous.interface != instance.interface
                or previous.security_profile != instance.security_profile
            ):
                raise ValueError("security rotation may change only security_generation")
            if not previous.security_generation:
                raise ValueError("instance has no active security generation")
            if not instance.security_generation:
                raise ValueError("security rotation requires a new generation")
            if previous.security_generation == instance.security_generation:
                raise ValueError("security rotation requires a different generation")

            source, generation = self._security_result_parts(security_result, instance)
            if generation != instance.security_generation:
                raise ValueError("security result generation does not match instance state")
            security_root = self.prefix / "instances" / instance.system_id / "security"
            current = security_root / "current"
            generations = security_root / "generations"
            if current.is_symlink():
                current_target = current.resolve()
                if generations.resolve() not in current_target.parents:
                    raise ValueError("instance security current escapes generations")
                if current_target.name != previous.security_generation:
                    raise ValueError("active security generation does not match instance state")
                # Validate the old generation before touching anything.  This
                # prevents a damaged or cross-instance current link from
                # becoming the rollback target.
                validate_instance_security_generation(
                    current_target,
                    self.install_uuid,
                    previous,
                    previous.security_generation,
                )
            else:
                raise FileNotFoundError(current)
            if not generations.is_dir() or generations.is_symlink():
                raise ValueError("instance security generations is not a directory")
            destination = generations / generation
            if destination.exists() or destination.is_symlink():
                raise FileExistsError(destination)
            staging_parent = security_root / ".staging"
            staging_parent.mkdir(mode=0o700, exist_ok=True)
            if staging_parent.is_symlink() or not staging_parent.is_dir():
                raise ValueError("instance security staging path is not a directory")
            staging = staging_parent / f"rotate-{generation}-{os.getpid()}-{threading.get_ident()}"
            if os.path.lexists(staging):
                raise ValueError("instance security staging path already exists")
            state_path = self.prefix / "instances" / instance.system_id / "state.json"
            if state_path.is_symlink() or not state_path.is_file():
                raise ValueError("instance state path is not a regular file")
            old_state = state_path.read_bytes()
            old_link = current.readlink()
            published_identity: tuple[int, int] | None = None
            published_owned: Mapping[Path, tuple[int, int]] = {}
            temporary_link: Path | None = None
            state_replaced = False

            def injected(step: str) -> None:
                if fail is not None:
                    fail(step)

            try:
                self._copy_tree(source, staging)
                staging.chmod(0o700)
                for root, directories, files in os.walk(staging, followlinks=False):
                    Path(root).chmod(0o700)
                    for name in directories:
                        (Path(root) / name).chmod(0o700)
                    for name in files:
                        (Path(root) / name).chmod(0o600)
                injected("before-generation")
                _rename_noreplace(staging, destination)
                published_identity = self._identity(destination)
                published_owned = self._owned_tree(destination)
                injected("after-generation")

                temporary_link = security_root / f".current-rotate-{os.getpid()}-{threading.get_ident()}"
                os.symlink(Path("generations") / generation, temporary_link)
                os.replace(temporary_link, current)
                temporary_link = None
                injected("after-activate")

                self._write_json(state_path, instance.to_dict())
                state_replaced = True
                injected("after-state")
            except BaseException:
                # Restore the link first so a reader can never observe a
                # failed generation as active.  ``os.replace`` is safe under
                # the system lock and the old target is known to be inside
                # this system's generations directory.
                if temporary_link is not None and os.path.lexists(temporary_link):
                    temporary_link.unlink()
                if current.is_symlink():
                    target = current.resolve()
                    if target.name == generation:
                        os.unlink(current)
                if not current.is_symlink() and not os.path.lexists(current):
                    os.symlink(old_link, current)
                elif current.is_symlink() and current.readlink() != old_link:
                    os.unlink(current)
                    os.symlink(old_link, current)
                if state_replaced:
                    temporary_state = state_path.with_name(
                        f".state-rotate-rollback-{os.getpid()}-{threading.get_ident()}"
                    )
                    temporary_state.write_bytes(old_state)
                    temporary_state.chmod(0o600)
                    os.replace(temporary_state, state_path)
                if published_identity is not None and self._identity(destination) == published_identity:
                    self._remove_owned_tree(destination, published_owned)
                if staging.exists() and not staging.is_symlink():
                    shutil.rmtree(staging)
                raise
            finally:
                if temporary_link is not None and os.path.lexists(temporary_link):
                    temporary_link.unlink()
            return instance

    # Descriptive aliases for callers that model registry and lifecycle as
    # separate boundaries.
    rotate_instance_security = rotate_security
    rotate_security_generation = rotate_security
    rotate = rotate_security

    # Explicit aliases keep the boundary readable to callers that distinguish
    # registry records from runtime operations.
    register_instance = register
    replace_instance = replace
    remove_instance = remove


__all__ = ["InstanceRuntime"]
