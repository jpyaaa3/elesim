"""Persistent human-readable names; digests remain the content identities.

Names are never credentials or ownership proofs. Callers provide an exact
registry path inside an owned installation and perform Docker label checks
before assigning any tag. Reservations survive failed builds and cleanup so a
name cannot silently acquire a different meaning later.
"""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import re
import secrets
import stat
import tempfile
from collections.abc import Callable, Iterable


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
NAME_PATTERN = re.compile(r"[a-z]{2,16}_[a-z]{2,16}\Z")
_MAX_BYTES = 4 * 1024 * 1024


def random_name() -> str:
    return f"{secrets.choice(ADJECTIVES)}_{secrets.choice(ANIMALS)}"


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


def reserve_name(
    path: Path,
    scope: str,
    identity: str,
    *,
    unavailable: Iterable[str] = (),
    generate: Callable[[], str] = random_name,
) -> str:
    """Reserve once per identity, serializing collision checks and publication.

    ``unavailable`` contains names already used outside this registry (for
    example existing Docker projects). Existing bindings never get renamed.
    Exhaustion is an explicit error, never a numeric/hash suffix or overwrite.
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
        if identity in entries:
            return entries[identity]
        used = set(entries.values()) | set(unavailable)
        for _ in range(4096):
            name = generate()
            if not isinstance(name, str) or not NAME_PATTERN.fullmatch(name):
                raise ValueError("name generator returned an invalid name")
            if name not in used:
                break
        else:
            raise ValueError("no unused readable name available")
        entries[identity] = name
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
        return name
