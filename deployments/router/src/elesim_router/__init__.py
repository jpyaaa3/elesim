"""Elesim protocol-v3 registry, lease authority and router."""

from __future__ import annotations

from .core import RouterCore

__all__ = ["RouterCore", "RoutingServer"]


def __getattr__(name: str):
    if name != "RoutingServer":
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from .main import RoutingServer

    return RoutingServer
