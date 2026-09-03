"""SROS2 authority generations and least-privilege host bundles.

Cryptographic material is created exclusively through the ROS 2 ``security``
CLI. This module owns generation transactions and delegates strict filesystem
and manifest handling to its private storage module.
"""

from __future__ import annotations

import os
import subprocess
import tempfile
import uuid
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ._security_storage import (
    BundleArtifact,
    BundleManifest,
    EnclaveIdentity,
    SROS2_ROLES,
    SecurityAuthorityError,
    atomic_json,
    copy_secure_tree,
    ensure_private_directory,
    file_sha256,
    fsync_directory,
    harden_tree,
    materialize_keystore_symlinks,
    new_generation_id,
    read_json_object,
    regular_files,
    remove_owned_tree,
    require_contained_directory,
    require_regular_file,
    secure_absolute,
    utc_now,
    validate_generation,
    validate_host_id,
    validate_keystore,
    validate_system_id,
    validate_tree,
    verify_bundle,
)


CommandRunner = Callable[[Sequence[str]], None]


@dataclass(frozen=True)
class ActivationMetadata:
    schema_version: int
    system_id: str
    generation: str
    previous_generation: str | None
    action: str
    changed_at: str
    rolled_back_from: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "system_id": self.system_id,
            "generation": self.generation,
            "previous_generation": self.previous_generation,
            "action": self.action,
            "changed_at": self.changed_at,
            "rolled_back_from": self.rolled_back_from,
        }


def subprocess_command_runner(command: Sequence[str]) -> None:
    """Run one ROS 2 CLI command with the distro Python ABI first.

    ROS 2 Humble's SROS2 CLI is tested with Ubuntu's patched cryptography
    package.  Application dependencies such as Paramiko may install a newer
    copy in ``/usr/local`` whose API is incompatible with Humble.  Prepending
    the distro site directory isolates the ROS CLI without downgrading the
    application environment.
    """

    environment = os.environ.copy()
    distro_site = Path("/usr/lib/python3/dist-packages")
    if distro_site.is_dir():
        inherited = environment.get("PYTHONPATH", "")
        environment["PYTHONPATH"] = os.pathsep.join(
            value for value in (str(distro_site), inherited) if value
        )
    try:
        completed = subprocess.run(
            tuple(command),
            check=False,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise SecurityAuthorityError(
            f"ROS 2 security command could not complete: {command[2]}"
        ) from exc
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout or "no diagnostic output")[-4096:]
        raise SecurityAuthorityError(
            f"ROS 2 security command failed ({command[2]}): {detail.strip()}"
        )


class Sros2Authority:
    """Own immutable SROS2 generations under one protected authority root."""

    def __init__(
        self,
        root: Path,
        *,
        runner: CommandRunner = subprocess_command_runner,
    ) -> None:
        self.root = secure_absolute(root)
        self.runner = runner
        ensure_private_directory(self.root)
        ensure_private_directory(self.root / ".staging")
        ensure_private_directory(self.root / "generations")

    def begin_generation(
        self,
        system_id: str,
        *,
        generation: str | None = None,
    ) -> "AuthorityGeneration":
        validate_system_id(system_id)
        generation_id = generation or new_generation_id()
        validate_generation(generation_id)
        self._bind_system(system_id)

        published = self.root / "generations" / generation_id
        if published.exists() or published.is_symlink():
            raise FileExistsError(f"authority generation already exists: {generation_id}")
        staging = self.root / ".staging" / generation_id
        if staging.exists() or staging.is_symlink():
            raise FileExistsError(f"authority generation is already staged: {generation_id}")
        staging.mkdir(mode=0o700)
        keystore = staging / "keystore"
        try:
            self.runner(("ros2", "security", "create_keystore", str(keystore)))
            materialize_keystore_symlinks(keystore)
            validate_keystore(keystore, require_private=True)
            harden_tree(staging)
        except Exception:
            remove_owned_tree(staging, owner=self.root / ".staging")
            raise
        return AuthorityGeneration(
            authority=self,
            system_id=system_id,
            generation=generation_id,
            location=staging,
            state="staging",
        )

    def active(self) -> ActivationMetadata | None:
        path = self.root / "active.json"
        if not path.exists():
            return None
        return _activation_from_dict(read_json_object(path))

    def activate_generation(self, generation: str) -> ActivationMetadata:
        validate_generation(generation)
        metadata = self.generation_metadata(generation)
        current = self.active()
        if current is not None and current.generation == generation:
            return current
        activation = ActivationMetadata(
            schema_version=1,
            system_id=str(metadata["system_id"]),
            generation=generation,
            previous_generation=None if current is None else current.generation,
            action="activate",
            changed_at=utc_now(),
        )
        atomic_json(self.root / "active.json", activation.to_dict())
        return activation

    def rollback_generation(self, expected_generation: str) -> ActivationMetadata:
        validate_generation(expected_generation)
        current = self.active()
        if current is None or current.generation != expected_generation:
            actual = None if current is None else current.generation
            raise SecurityAuthorityError(
                f"cannot rollback generation {expected_generation!r}; active={actual!r}"
            )
        target = current.previous_generation
        if target is None:
            raise SecurityAuthorityError("active generation has no rollback target")
        target_metadata = self.generation_metadata(target)
        rollback = ActivationMetadata(
            schema_version=1,
            system_id=str(target_metadata["system_id"]),
            generation=target,
            previous_generation=None,
            action="rollback",
            changed_at=utc_now(),
            rolled_back_from=expected_generation,
        )
        atomic_json(self.root / "active.json", rollback.to_dict())
        return rollback

    def generation_metadata(self, generation: str) -> Mapping[str, object]:
        validate_generation(generation)
        root = self.root / "generations" / generation
        require_contained_directory(root, self.root / "generations")
        metadata = read_json_object(root / "generation.json")
        if metadata.get("generation") != generation:
            raise SecurityAuthorityError("generation metadata does not match its path")
        return metadata

    def export_host_bundle(
        self,
        generation: str,
        host_id: str,
        destination: Path,
    ) -> BundleArtifact:
        validate_generation(generation)
        validate_host_id(host_id)
        source = self.root / "generations" / generation / "bundles" / host_id
        require_contained_directory(source, self.root / "generations")
        verify_bundle(source)

        target = secure_absolute(destination)
        if target.exists() or target.is_symlink():
            raise FileExistsError(f"bundle destination already exists: {target}")
        ensure_private_directory(target.parent)
        staging_parent = Path(
            tempfile.mkdtemp(prefix=f".{target.name}.staging-", dir=target.parent)
        )
        staging_parent.chmod(0o700)
        temporary = staging_parent / "bundle"
        published = False
        try:
            copy_secure_tree(source, temporary)
            artifact = verify_bundle(temporary)
            os.replace(temporary, target)
            published = True
            try:
                staging_parent.rmdir()
            except OSError:
                pass
            fsync_directory(target.parent)
            return BundleArtifact(target, artifact.manifest)
        except Exception:
            if (
                not published
                and staging_parent.exists()
                and not staging_parent.is_symlink()
            ):
                remove_owned_tree(staging_parent, owner=target.parent)
            raise

    def _bind_system(self, system_id: str) -> None:
        path = self.root / "authority.json"
        if path.exists():
            raw = read_json_object(path)
            if raw.get("system_id") != system_id:
                raise SecurityAuthorityError(
                    f"authority belongs to system {raw.get('system_id')!r}, "
                    f"not {system_id!r}"
                )
            return
        atomic_json(
            path,
            {
                "schema_version": 1,
                "system_id": system_id,
                "created_at": utc_now(),
            },
        )


class AuthorityGeneration:
    """A staging transaction that becomes immutable when published."""

    def __init__(
        self,
        *,
        authority: Sros2Authority,
        system_id: str,
        generation: str,
        location: Path,
        state: str,
    ) -> None:
        self.authority = authority
        self.system_id = system_id
        self.generation = generation
        self._location = location
        self._state = state
        self._identities: dict[str, EnclaveIdentity] = {}
        self._assigned_hosts: dict[str, str] = {}

    @property
    def state(self) -> str:
        return self._state

    @property
    def path(self) -> Path:
        return self._location

    @property
    def keystore(self) -> Path:
        return self._location / "keystore"

    def create_enclave(
        self,
        role: str,
        endpoint_id: str,
        *,
        policy: Path | None = None,
    ) -> EnclaveIdentity:
        self._require_staging()
        identity = EnclaveIdentity(self.system_id, role, endpoint_id)
        policy_path = None if policy is None else require_regular_file(policy)
        existing = self._identities.get(identity.enclave)
        if existing is not None:
            if existing == identity:
                raise FileExistsError(f"enclave already exists: {identity.enclave}")
            raise SecurityAuthorityError(
                f"endpoint IDs collide at enclave path {identity.enclave!r}"
            )

        self.authority.runner(
            (
                "ros2",
                "security",
                "create_enclave",
                str(self.keystore),
                identity.enclave,
            )
        )
        if policy_path is not None:
            self.authority.runner(
                (
                    "ros2",
                    "security",
                    "create_permission",
                    str(self.keystore),
                    identity.enclave,
                    str(policy_path),
                )
            )
        materialize_keystore_symlinks(self.keystore)
        enclave_root = self.keystore / "enclaves" / identity.relative_enclave
        require_contained_directory(enclave_root, self.keystore / "enclaves")
        validate_keystore(self.keystore, require_private=True)
        harden_tree(self._location)
        self._identities[identity.enclave] = identity
        return identity

    def stage_host_bundle(
        self,
        host_id: str,
        identities: Sequence[EnclaveIdentity],
    ) -> BundleArtifact:
        self._require_staging()
        validate_host_id(host_id)
        selected = tuple(sorted(set(identities)))
        if not selected:
            raise ValueError("a host bundle requires at least one enclave")
        if len({identity.role for identity in selected}) != len(selected):
            raise ValueError("a host bundle may contain only one enclave per role")
        self._validate_bundle_assignments(host_id, selected)

        bundles_root = self._location / "bundles"
        ensure_private_directory(bundles_root)
        destination = bundles_root / host_id
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(f"host bundle already staged: {host_id}")
        temporary = bundles_root / f".{host_id}.staging-{uuid.uuid4().hex}"
        temporary.mkdir(mode=0o700)
        try:
            manifest = self._write_bundle(temporary, host_id, selected)
            verify_bundle(temporary)
            os.replace(temporary, destination)
            fsync_directory(bundles_root)
        except Exception:
            if temporary.exists() and not temporary.is_symlink():
                remove_owned_tree(temporary, owner=bundles_root)
            raise
        for identity in selected:
            self._assigned_hosts[identity.enclave] = host_id
        return BundleArtifact(destination, manifest)

    def _validate_bundle_assignments(
        self,
        host_id: str,
        selected: Sequence[EnclaveIdentity],
    ) -> None:
        for identity in selected:
            if identity.system_id != self.system_id:
                raise ValueError("host bundle contains an enclave from another system")
            issued = self._identities.get(identity.enclave)
            if issued != identity:
                raise SecurityAuthorityError(
                    f"enclave was not issued by this generation: {identity.enclave}"
                )
            assigned = self._assigned_hosts.get(identity.enclave)
            if assigned is not None and assigned != host_id:
                raise SecurityAuthorityError(
                    f"enclave {identity.enclave} is already assigned to host {assigned!r}"
                )

    def _write_bundle(
        self,
        root: Path,
        host_id: str,
        selected: tuple[EnclaveIdentity, ...],
    ) -> BundleManifest:
        target_keystore = root / "keystore"
        target_keystore.mkdir(mode=0o700)
        copy_secure_tree(self.keystore / "public", target_keystore / "public")
        enclaves_root = target_keystore / "enclaves"
        enclaves_root.mkdir(mode=0o700)
        for identity in selected:
            source = self.keystore / "enclaves" / identity.relative_enclave
            copy_secure_tree(source, enclaves_root / identity.relative_enclave)
        app_views = root / "apps"
        app_views.mkdir(mode=0o700)
        for identity in selected:
            app_keystore = app_views / identity.role / "keystore"
            app_keystore.mkdir(parents=True, mode=0o700)
            copy_secure_tree(
                self.keystore / "public",
                app_keystore / "public",
            )
            copy_secure_tree(
                self.keystore / "enclaves" / identity.relative_enclave,
                app_keystore / "enclaves" / identity.relative_enclave,
            )
        harden_tree(root)
        files = {
            path.relative_to(root).as_posix(): file_sha256(path)
            for path in regular_files(root)
        }
        manifest = BundleManifest(
            schema_version=2,
            system_id=self.system_id,
            generation=self.generation,
            host_id=host_id,
            created_at=utc_now(),
            enclaves=selected,
            files=files,
        )
        atomic_json(root / "manifest.json", manifest.to_dict())
        harden_tree(root)
        return manifest

    def publish(self) -> Path:
        self._require_staging()
        if not self._identities:
            raise SecurityAuthorityError("cannot publish an empty authority generation")
        validate_keystore(self.keystore, require_private=True)
        validate_tree(self._location)
        metadata = {
            "schema_version": 1,
            "system_id": self.system_id,
            "generation": self.generation,
            "created_at": utc_now(),
            "enclaves": [
                identity.to_dict()
                for identity in sorted(self._identities.values())
            ],
            "hosts": dict(sorted(self._assigned_hosts.items())),
        }
        atomic_json(self._location / "generation.json", metadata)
        harden_tree(self._location)
        destination = self.authority.root / "generations" / self.generation
        if destination.exists() or destination.is_symlink():
            raise FileExistsError(f"authority generation already exists: {self.generation}")
        os.replace(self._location, destination)
        fsync_directory(destination.parent)
        self._location = destination
        self._state = "published"
        return destination

    def activate(self) -> ActivationMetadata:
        if self._state == "aborted":
            raise SecurityAuthorityError("generation transaction was aborted")
        if self._state == "staging":
            self.publish()
        return self.authority.activate_generation(self.generation)

    def rollback(self) -> ActivationMetadata:
        if self._state != "published":
            raise SecurityAuthorityError("only a published generation can be rolled back")
        return self.authority.rollback_generation(self.generation)

    def export_host_bundle(self, host_id: str, destination: Path) -> BundleArtifact:
        if self._state != "published":
            raise SecurityAuthorityError("publish the generation before exporting bundles")
        return self.authority.export_host_bundle(
            self.generation,
            host_id,
            destination,
        )

    def abort(self) -> None:
        if self._state != "staging":
            return
        remove_owned_tree(self._location, owner=self.authority.root / ".staging")
        self._state = "aborted"

    def _require_staging(self) -> None:
        if self._state != "staging":
            raise SecurityAuthorityError(
                f"generation is not mutable in state {self._state!r}"
            )

    def __enter__(self) -> "AuthorityGeneration":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        if self._state == "staging":
            self.abort()


def _activation_from_dict(raw: Mapping[str, Any]) -> ActivationMetadata:
    if raw.get("schema_version") != 1:
        raise SecurityAuthorityError("unsupported activation metadata schema")
    system_id = str(raw.get("system_id", ""))
    generation = str(raw.get("generation", ""))
    previous = raw.get("previous_generation")
    rolled_back = raw.get("rolled_back_from")
    validate_system_id(system_id)
    validate_generation(generation)
    if previous is not None:
        validate_generation(str(previous))
    if rolled_back is not None:
        validate_generation(str(rolled_back))
    action = str(raw.get("action", ""))
    if action not in {"activate", "rollback"}:
        raise SecurityAuthorityError(f"invalid activation action: {action!r}")
    return ActivationMetadata(
        schema_version=1,
        system_id=system_id,
        generation=generation,
        previous_generation=None if previous is None else str(previous),
        action=action,
        changed_at=str(raw.get("changed_at", "")),
        rolled_back_from=None if rolled_back is None else str(rolled_back),
    )


__all__ = [
    "ActivationMetadata",
    "AuthorityGeneration",
    "BundleArtifact",
    "BundleManifest",
    "CommandRunner",
    "EnclaveIdentity",
    "SROS2_ROLES",
    "SecurityAuthorityError",
    "Sros2Authority",
    "new_generation_id",
    "subprocess_command_runner",
    "verify_bundle",
]
