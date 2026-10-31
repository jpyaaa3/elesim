"""Unwind Pilot resources when Docker terminates its main process."""

from contextlib import contextmanager
import signal


def _exit_on_sigterm(signum, frame) -> None:
    raise SystemExit(0)


@contextmanager
def graceful_sigterm():
    previous = signal.signal(signal.SIGTERM, _exit_on_sigterm)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, previous)
