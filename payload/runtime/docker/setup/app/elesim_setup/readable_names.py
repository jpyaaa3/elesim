"""Persistent human-readable names; digests remain the content identities.

Names are never credentials or ownership proofs. Callers provide an exact
registry path inside an owned installation and perform Docker label checks
before assigning any tag. Reservations survive failed builds and cleanup so a
name cannot silently acquire a different meaning later.
"""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import stat
import tempfile
from collections.abc import Callable, Iterable, Mapping


# Keep the vocabulary small, familiar, ASCII-only and independent of installed
# third-party packages. The persisted mapping, not vocabulary order, is stable.
ADJECTIVES = tuple("""amber blue bright calm clear clever cool coral cozy crisp
curious dawn deep eager fair fast gentle glad golden grand green happy hazel
icy ivory jolly kind leafy light lively lucky mellow merry misty neat noble
olive orange pale patient peach pink plain proud purple quick quiet red rosy
royal sage sandy shy silent silver small smooth snowy soft steady sunny sweet
swift teal tidy tiny warm white wild wise young""".split())
ANIMALS = tuple("""alpaca ant badger bat bear beaver bee bison boar bobcat buffalo
camel canary cat cheetah chicken cobra cod cougar cow coyote crab crane cricket
crow deer dolphin donkey dove duck eagle eel elephant elk falcon ferret finch
fox frog gazelle gecko goat goose gull hamster hare hawk hedgehog heron horse
ibis jaguar jay jellyfish koala koi lamb lark lemur leopard lion llama lobster
lynx macaw magpie mink mole monkey moose moth mouse mule newt octopus orca
osprey otter owl ox oyster panda panther parrot peacock pelican penguin pigeon
pony puma quail rabbit raccoon ram raven robin salmon seal shark sheep shrimp
snail sparrow spider squid stork swan tiger toad trout tuna turkey turtle viper
walrus wasp weasel whale wolf wombat wren yak zebra""".split())
NAME_PATTERN = re.compile(r"[a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6})\Z")
_IMAGE_ROLE = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_IMAGE_FINGERPRINT = re.compile(r"[0-9a-f]{64}\Z")
_MAX_BYTES = 4 * 1024 * 1024
_IMAGE_SCOPE = "images"
_RELEASE_SCOPE = "releases"


def random_name() -> str:
    return secrets.choice(ANIMALS)


def random_install_name() -> str:
    return secrets.choice(ADJECTIVES)


def _unused_name(generate: Callable[[], str], used: set[str]) -> str:
    base = generate()
    if not isinstance(base, str) or not NAME_PATTERN.fullmatch(base):
        raise ValueError("name generator returned an invalid name")
    if base not in used:
        return base
    # Keep old injected two-word generators compatible with their registries.
    if "_" in base:
        for _ in range(4095):
            candidate = generate()
            if not isinstance(candidate, str) or not NAME_PATTERN.fullmatch(candidate):
                raise ValueError("name generator returned an invalid name")
            if candidate not in used:
                return candidate
        raise ValueError("no unused readable name available")
    stem = base.rstrip("0123456789")
    for number in range(2, 1000000):
        candidate = f"{stem}{number}"
        if candidate not in used:
            return candidate
    raise ValueError("no unused readable name available")


def lookup_name(path: Path, scope: str, identity: str) -> str:
    """Read a reservation without creating or repairing registry state."""
    path = Path(path)
    _safe_path(path)
    return _read(path).get(scope, {}).get(identity, "")


def release_reservation_identity(
    source_revision: str,
    roles: Iterable[str],
    build_fingerprints: Mapping[str, str],
    runtime_data_digest: str,
) -> str:
    """Return the legacy all-role identity for one release's build inputs.

    The readable suffix cannot be derived from the final release key because
    that key includes the suffix itself.  These values are all known before
    Docker assigns image IDs, so retries of the same update keep one suffix
    while a source/config/data change receives a new one.
    This shape is retained only to authenticate registries written by the
    intermediate shared-alias implementation; new reservations use
    :func:`role_release_reservation_identity`.
    """

    payload = {
        "source_revision": str(source_revision),
        "roles": sorted(str(role) for role in roles),
        "build_fingerprints": {
            str(role): str(fingerprint)
            for role, fingerprint in sorted(build_fingerprints.items())
        },
        "runtime_data_digest": str(runtime_data_digest),
    }
    digest = hashlib.sha256(
        (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode(
            "utf-8"
        )
    ).hexdigest()
    return f"release:{digest}"


def role_release_reservation_identity(
    source_revision: str,
    role: str,
    build_fingerprint: str,
    runtime_data_digest: str,
) -> str:
    """Return the stable pre-tag identity for one role's release inputs.

    A release suffix belongs to one application image, not to the complete
    set of roles that happened to be built in the same update.  Keeping the
    role in the reservation identity prevents Pilot and Sim (or two other
    roles) from receiving the same alias while still making retries of the
    same role/input reusable until successful publication.
    """

    payload = {
        "source_revision": str(source_revision),
        "role": str(role),
        "build_fingerprint": str(build_fingerprint),
        "runtime_data_digest": str(runtime_data_digest),
    }
    digest = hashlib.sha256(
        (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode(
            "utf-8"
        )
    ).hexdigest()
    return f"release-role:{digest}"


def reserve_role_release_name(
    path: Path,
    source_revision: str,
    role: str,
    build_fingerprint: str,
    runtime_data_digest: str,
    *,
    unavailable: Iterable[str] = (),
    generate: Callable[[], str] = random_name,
) -> str:
    """Reserve one installation-wide readable suffix for one role/input.

    ``all_scopes`` makes release aliases avoid installation, image, tools and
    historical reservations as well. Completed publications advance the
    generation; retries of an unpublished input keep their reservation.
    """

    identity = role_release_reservation_identity(
        source_revision, role, build_fingerprint, runtime_data_digest,
    )
    path = Path(path)
    _safe_path(path)
    return reserve_name(
        path,
        _RELEASE_SCOPE,
        identity,
        unavailable=unavailable,
        generate=generate,
        all_scopes=True,
        advance_completed=True,
    )


def role_release_names(path: Path, identity: str) -> tuple[str, ...]:
    """Read current and historical reservations for authenticated role inputs."""
    _safe_path(path)
    entries = _read(path).get(_RELEASE_SCOPE, {})
    return tuple(name for key, name in entries.items()
                 if key == identity or (key.startswith(identity + ":")
                                       and key[len(identity) + 1:].isdigit()))


def mark_release_names_published(path: Path, aliases: Iterable[str]) -> None:
    """Complete reservations only after publication and ownership succeed."""
    _safe_path(path)
    lock_path = path.with_name(path.name + ".lock")
    _safe_path(lock_path)
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
    with os.fdopen(fd, "a+b") as lock:
        _regular(lock.fileno())
        fcntl.flock(lock, fcntl.LOCK_EX)
        names = _read(path)
        selected = set(aliases)
        published = names.setdefault("published", {})
        for identity, name in names.get(_RELEASE_SCOPE, {}).items():
            if name in selected:
                published[identity] = name
        _write(path, names)




def _image_identity(role: str, fingerprint: str) -> str:
    if not isinstance(role, str) or _IMAGE_ROLE.fullmatch(role) is None:
        raise ValueError("image role has an invalid format")
    if not isinstance(fingerprint, str) or _IMAGE_FINGERPRINT.fullmatch(fingerprint) is None:
        raise ValueError("image fingerprint has an invalid format")
    return f"{role}:{fingerprint}"


def lookup_image_names(path: Path, role: str, fingerprint: str) -> tuple[str, ...]:
    """Return current and historical aliases for one role/build identity.

    Fresh reservations live in the installation-wide ``images`` scope.  The
    role-scoped fallback is retained so immutable tags emitted by older
    installers remain readable after the registry format is upgraded.  The
    first value is always the preferred alias for newly generated artifacts.
    """

    identity = _image_identity(role, fingerprint)
    path = Path(path)
    _safe_path(path)
    names = _read(path)
    values = []
    preferred = names.get(_IMAGE_SCOPE, {}).get(identity, "")
    legacy = names.get(role, {}).get(fingerprint, "")
    for name in (preferred, legacy):
        if name and name not in values:
            values.append(name)
    return tuple(values)


def lookup_image_name(path: Path, role: str, fingerprint: str) -> str:
    """Return the preferred alias for one role/build identity, if reserved."""

    names = lookup_image_names(path, role, fingerprint)
    return names[0] if names else ""


def _safe_path(path: Path) -> None:
    if not path.is_absolute():
        raise ValueError("name registry path must be absolute")
    for parent in (path, *path.parents):
        if parent.is_symlink():
            raise ValueError("name registry path must not contain symlinks")


def _regular(fd: int) -> None:
    info = os.fstat(fd)
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > _MAX_BYTES:
        raise ValueError("name registry must be a bounded singly-linked regular file")


def _read(path: Path) -> dict[str, dict[str, str]]:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except FileNotFoundError:
        return {}
    with os.fdopen(fd, "rb") as stream:
        _regular(stream.fileno())
        def pairs(items):
            result = {}
            for key, value in items:
                if key in result:
                    raise ValueError("duplicate name registry key")
                result[key] = value
            return result
        value = json.loads(stream.read(_MAX_BYTES + 1), object_pairs_hook=pairs)
    if not isinstance(value, dict) or set(value) != {"schema_version", "names"} or type(value["schema_version"]) is not int or value["schema_version"] != 1:
        raise ValueError("invalid name registry schema")
    names = value["names"]
    if not isinstance(names, dict):
        raise ValueError("invalid name registry mappings")
    for scope, entries in names.items():
        if not isinstance(scope, str) or not isinstance(entries, dict):
            raise ValueError("invalid name registry scope")
        for identity, name in entries.items():
            if not isinstance(identity, str) or not identity or not isinstance(name, str) or not NAME_PATTERN.fullmatch(name):
                raise ValueError("invalid name registry entry")
        if len(set(entries.values())) != len(entries):
            raise ValueError("duplicate name reservation")
    return names


def _write(path: Path, names: dict[str, dict[str, str]]) -> None:
    payload = (json.dumps({"schema_version": 1, "names": names}, sort_keys=True) + "\n").encode()
    if len(payload) > _MAX_BYTES:
        raise ValueError("name registry is full")
    descriptor, temporary = tempfile.mkstemp(prefix=".names-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def reserve_name(
    path: Path,
    scope: str,
    identity: str,
    *,
    unavailable: Iterable[str] = (),
    generate: Callable[[], str] = random_name,
    all_scopes: bool = False,
    advance_completed: bool = False,
) -> str:
    """Reserve once per identity, serializing collision checks and publication.

    ``unavailable`` contains names already used outside this registry (for
    example existing Docker projects). Existing bindings never get renamed.
    New single-word names receive a numeric suffix on collision.
    """
    if not isinstance(scope, str) or not scope or not isinstance(identity, str) or not identity:
        raise ValueError("name scope and identity must be non-empty strings")
    path = Path(path)
    _safe_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = path.with_name(path.name + ".lock")
    _safe_path(lock_path)
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
    with os.fdopen(fd, "a+b") as lock:
        _regular(lock.fileno())
        fcntl.flock(lock, fcntl.LOCK_EX)
        _safe_path(path)
        names = _read(path)
        entries = names.setdefault(scope, {})
        if advance_completed:
            generation = 0
            while f"{identity}:{generation}" in names.get("published", {}):
                generation += 1
            identity = f"{identity}:{generation}"
        if identity in entries:
            return entries[identity]
        used = (
            {
                name
                for scope_entries in names.values()
                for name in scope_entries.values()
            }
            if all_scopes
            else set(entries.values())
        ) | set(unavailable)
        name = _unused_name(generate, used)
        entries[identity] = name
        _write(path, names)
        return name


def reserve_image_name(
    path: Path,
    role: str,
    fingerprint: str,
    *,
    unavailable: Iterable[str] = (),
    generate: Callable[[], str] = random_name,
) -> str:
    """Reserve an installation-wide alias for one role/build identity.

    New aliases share one registry scope across every Docker role, tools and
    the optional development image.  Older registries used one scope per role;
    those bindings remain immutable history.  A colliding historical binding
    therefore receives a new shared-scope alias while the old tag stays valid.
    """

    identity = _image_identity(role, fingerprint)
    path = Path(path)
    _safe_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = path.with_name(path.name + ".lock")
    _safe_path(lock_path)
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
    with os.fdopen(fd, "a+b") as lock:
        _regular(lock.fileno())
        fcntl.flock(lock, fcntl.LOCK_EX)
        _safe_path(path)
        names = _read(path)
        entries = names.setdefault(_IMAGE_SCOPE, {})
        if identity in entries:
            return entries[identity]

        legacy = names.get(role, {}).get(fingerprint, "")
        if legacy:
            owners = sum(
                name == legacy
                for scope_entries in names.values()
                for name in scope_entries.values()
            )
            if owners == 1:
                # Promote a non-colliding historical binding into the shared
                # scope without changing its emitted Docker tag.
                entries[identity] = legacy
                _write(path, names)
                return legacy

        used = {
            name
            for scope_entries in names.values()
            for name in scope_entries.values()
        } | set(unavailable)
        name = _unused_name(generate, used)
        entries[identity] = name
        _write(path, names)
        return name
