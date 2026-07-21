"""Optional tracing hooks; robot operation never depends on telemetry packages."""

from __future__ import annotations

from contextlib import nullcontext
from functools import wraps
from typing import Any, Callable


def traced(_name: str, **_options: object):
    def decorate(function: Callable[..., Any]):
        @wraps(function)
        def wrapped(*args: object, **kwargs: object):
            return function(*args, **kwargs)

        return wrapped

    return decorate


def sampled_traced(_name: str, **options: object):
    return traced(_name, **options)


def span(_name: str, **_options: object):
    return nullcontext()


def configure_tracing(_service_name: str) -> None:
    return None


def shutdown_tracing() -> None:
    return None
