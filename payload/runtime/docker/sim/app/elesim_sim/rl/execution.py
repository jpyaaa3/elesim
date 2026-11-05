"""Command interpolation and pre-reset capture, independent of the task."""
from dataclasses import dataclass
import math
from typing import Any, Protocol


class TrainingEnvironment(Protocol):
    num_envs: int
    num_actions: int
    device: Any
    cfg: Any

    def reset(self): ...
    def step(self, actions): ...
    def get_observations(self): ...


@dataclass(frozen=True)
class MotionClock:
    dt: float
    substeps: int
    move_fraction: float = .6

    def __post_init__(self):
        if (not math.isfinite(self.dt) or self.dt <= 0
                or type(self.substeps) is not int or self.substeps < 1
                or not 0 < self.move_fraction <= 1):
            raise ValueError('Invalid motion clock')

    @property
    def duration(self):
        return self.dt * self.substeps

    def targets(self, start, end):
        moving = max(1, round(self.substeps * self.move_fraction))
        for i in range(self.substeps):
            yield i, start + (end-start) * min(1., (i+1) / moving)


def snapshot(value):
    """Detach mutable terminal state before reset, retaining device batches."""
    if hasattr(value, 'detach'):
        return value.detach().clone()
    if isinstance(value, dict):
        return {key: snapshot(item) for key, item in value.items()}
    if isinstance(value, list):
        return [snapshot(item) for item in value]
    if isinstance(value, tuple):
        return tuple(snapshot(item) for item in value)
    if hasattr(value, 'copy'):
        return value.copy()
    return value
