"""Decoder health values shared by presentation and stream recovery."""

from __future__ import annotations

import math
from typing import Any


def decoded_frame_age(receiver: Any) -> float | None:
    """Return a valid monotonic age, or None before the first decoded frame."""
    age = receiver.frame_age_s()
    if age is None:
        return None
    value = float(age)
    if not math.isfinite(value) or value < 0:
        raise ValueError("frame age must be finite and nonnegative")
    return value


def receiver_stats_suffix(receiver: Any) -> str:
    getter = getattr(receiver, "stats_snapshot", None)
    if not callable(getter):
        return ""
    try:
        stats = dict(getter())
        return " (" + ", ".join(f"{name}={value}" for name, value in stats.items()) + ")" if stats else ""
    except Exception:
        return ""
