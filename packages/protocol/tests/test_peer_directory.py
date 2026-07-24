from __future__ import annotations

import pytest

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    PeerAmbiguityError,
    PeerDescriptor,
    PeerDirectory,
    PeerDirectoryError,
    PeerError,
    PeerHeartbeat,
    PeerIdentity,
)


def descriptor(
    endpoint_id: str,
    boot_id: str,
    *,
    role: str = "robot",
    revision: int = 1,
) -> PeerDescriptor:
    return PeerDescriptor(
        identity=PeerIdentity(endpoint_id, boot_id),
        role=role,
        capabilities=(CAPABILITY_MOTION_ARM,),
        descriptor_revision=revision,
        service_prefix=f"/elesim/v5/{endpoint_id}/{boot_id}",
        topic_prefix=f"/elesim/v5/{endpoint_id}/{boot_id}",
        interface_hash="sha256:interface-v5",
    )


def make_live(
    directory: PeerDirectory,
    value: PeerDescriptor,
    *,
    now: float,
    sequence: int = 1,
) -> None:
    directory.announce(value, now=now)
    accepted = directory.heartbeat(
        PeerHeartbeat(
            value.identity,
            value.descriptor_revision,
            sequence,
        ),
        now=now,
    )
    assert accepted is True


def test_peer_identity_rejects_empty_or_whitespace_identifiers() -> None:
    with pytest.raises(PeerError):
        PeerIdentity("", "boot-a")
    with pytest.raises(PeerError):
        PeerIdentity("robot a", "boot-a")
    with pytest.raises(PeerError):
        PeerIdentity("robot-a", "boot a")


def test_descriptor_is_not_live_until_matching_heartbeat_arrives() -> None:
    directory = PeerDirectory(heartbeat_timeout_s=3.5)
    value = descriptor("robot-a", "boot-a")
    directory.announce(value, now=1.0)

    assert directory.resolve("robot-a", now=1.0) is None
    assert directory.heartbeat(
        PeerHeartbeat(value.identity, value.descriptor_revision + 1, 1),
        now=1.1,
    ) is False
    assert directory.heartbeat(
        PeerHeartbeat(value.identity, value.descriptor_revision, 1),
        now=1.2,
    ) is True
    assert directory.resolve("robot-a", now=1.2) == value


def test_stale_heartbeat_does_not_extend_peer_liveness() -> None:
    directory = PeerDirectory(heartbeat_timeout_s=2.0)
    value = descriptor("robot-a", "boot-a")
    make_live(directory, value, now=1.0, sequence=4)

    assert directory.heartbeat(
        PeerHeartbeat(value.identity, value.descriptor_revision, 4),
        now=2.5,
    ) is False
    assert directory.active(value.identity, now=3.01) is False


def test_new_descriptor_revision_requires_a_new_matching_heartbeat() -> None:
    directory = PeerDirectory()
    first = descriptor("robot-a", "boot-a", revision=1)
    second = descriptor("robot-a", "boot-a", revision=2)
    make_live(directory, first, now=1.0)

    directory.announce(second, now=1.1)

    assert directory.active(first.identity, now=1.1) is False
    assert directory.heartbeat(
        PeerHeartbeat(second.identity, 1, 2),
        now=1.2,
    ) is False
    assert directory.heartbeat(
        PeerHeartbeat(second.identity, 2, 1),
        now=1.3,
    ) is True
    assert directory.resolve("robot-a", now=1.3) == second


def test_same_revision_cannot_describe_two_different_peer_surfaces() -> None:
    directory = PeerDirectory()
    first = descriptor("robot-a", "boot-a")
    changed = PeerDescriptor(
        identity=first.identity,
        role=first.role,
        capabilities=first.capabilities,
        descriptor_revision=first.descriptor_revision,
        service_prefix="/different",
    )
    directory.announce(first, now=1.0)

    with pytest.raises(PeerDirectoryError, match="exactly one descriptor"):
        directory.announce(changed, now=1.1)


def test_duplicate_live_endpoint_id_is_ambiguous_and_fails_closed() -> None:
    directory = PeerDirectory()
    first = descriptor("robot-a", "boot-a")
    second = descriptor("robot-a", "boot-b")
    make_live(directory, first, now=1.0)
    make_live(directory, second, now=1.1)

    with pytest.raises(PeerAmbiguityError) as caught:
        directory.resolve("robot-a", now=1.2)

    assert caught.value.candidates == (first.identity, second.identity)


def test_discovery_filters_role_and_capability_without_selecting_a_winner() -> None:
    directory = PeerDirectory()
    robot = descriptor("robot-a", "boot-a")
    ui = PeerDescriptor(
        PeerIdentity("ui-a", "boot-ui"),
        "ui",
        capabilities=("operator_control",),
    )
    make_live(directory, robot, now=1.0)
    make_live(directory, ui, now=1.1)

    assert directory.discover(now=1.2, role="robot") == (robot,)
    assert directory.discover(
        now=1.2,
        capability=CAPABILITY_MOTION_ARM,
    ) == (robot,)


def test_expire_removes_stale_boot_generation() -> None:
    directory = PeerDirectory(heartbeat_timeout_s=1.0)
    old = descriptor("robot-a", "boot-old")
    current = descriptor("robot-a", "boot-current")
    make_live(directory, old, now=0.0)
    make_live(directory, current, now=0.8)

    assert directory.expire(now=1.1) == (old.identity,)
    assert directory.resolve("robot-a", now=1.1) == current
