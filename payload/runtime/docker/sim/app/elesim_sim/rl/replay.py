"""Replay timed commands through a caller-owned physics adapter.

The adapter owns reset, channel interpretation, contacts and success. Evaluation
never loads an optimizer or a policy. Commands are sampled at physics ticks.
"""
from typing import Protocol
import numpy as np
from .trajectory import Trajectory
from .execution import snapshot


class ReplayTarget(Protocol):
    def reset(self, metadata: dict): ...
    def command(self, channels: tuple[str, ...], values: np.ndarray): ...
    def step(self): ...
    def measure(self) -> dict: ...
    def evaluate(self, samples: list[dict]) -> dict: ...


def replay(trajectory: Trajectory, target: ReplayTarget, *, dt: float) -> dict:
    trajectory.validate()
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('Replay dt must be positive')
    ticks = int(round(trajectory.times[-1] / dt))
    if ticks < 1 or not np.isclose(ticks * dt, trajectory.times[-1], atol=1e-9, rtol=0):
        raise ValueError('Trajectory duration must be an integer number of physics ticks')
    times = np.arange(ticks + 1) * dt
    commands = trajectory.sample(times)
    target.reset(snapshot(trajectory.metadata))
    target.command(trajectory.channels, commands[0].copy())
    samples = [dict(time_s=0., state=snapshot(target.measure()))]
    for i in range(1, ticks + 1):
        target.command(trajectory.channels, commands[i].copy())
        target.step()
        samples.append(dict(time_s=float(times[i]), state=snapshot(target.measure())))
    return dict(schema='elesim-replay-result', schema_version=1,
                evaluation=target.evaluate(samples), samples=samples)
