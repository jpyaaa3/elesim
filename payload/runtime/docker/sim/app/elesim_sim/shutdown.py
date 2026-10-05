"""Let Docker SIGTERM unwind Python/Genesis cleanup on the main thread."""

from contextlib import contextmanager
import signal


def _exit_on_sigterm(signum, frame) -> None:
    # SystemExit runs finally blocks and atexit callbacks. The default SIGTERM
    # handler skips both, losing Genesis' newly compiled disk cache.
    raise SystemExit(0)


@contextmanager
def graceful_sigterm():
    previous = signal.signal(signal.SIGTERM, _exit_on_sigterm)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, previous)
