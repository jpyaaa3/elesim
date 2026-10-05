"""Cooperative cancellation with an explicit per-install commit boundary."""

from __future__ import annotations

from typing import Callable


class InstallCancelled(RuntimeError):
    pass


class InstallProgress:
    def __init__(self, write: Callable[[str], None], cancelled: Callable[[], bool]) -> None:
        self.write = write
        self.cancelled = cancelled
        self.committed = False

    def begin_installation(self) -> None:
        self.committed = False
        self.check_cancelled()

    def commit_installation(self) -> None:
        self.committed = True

    def check_cancelled(self) -> None:
        if not self.committed and self.cancelled():
            raise InstallCancelled("installation cancelled by user")

    def __call__(self, message: str) -> None:
        self.write(message)
        self.check_cancelled()
