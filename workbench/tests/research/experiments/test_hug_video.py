"""Recording cadence/lifecycle fixtures; these do not render or simulate."""
from types import SimpleNamespace

import numpy as np
import pytest

from workbench.research.experiments.hug_interpolation import video
from workbench.research.experiments.hug_interpolation.video import record_video


class Camera:
    def __init__(self):
        self.frames = 0
        self.saved = None

    def render(self):
        self.frames += 1
        return (np.zeros((2, 2, 3), dtype=np.uint8),)


@pytest.fixture(autouse=True)
def writer(monkeypatch):
    result = SimpleNamespace(frames=[], closed=False)
    def create(path, fps):
        result.path, result.fps = path, fps
        return result
    result.append_data = lambda frame: result.frames.append(frame)
    result.close = lambda: setattr(result, 'closed', True)
    monkeypatch.setattr(video, 'make_writer', create)
    return result


def environment():
    camera = Camera()
    env = SimpleNamespace(num_envs=1, s={'dt': .01}, tick_monitor=None,
                          scene=SimpleNamespace(cameras={'report': camera}))
    return env, camera


def test_report_video_samples_physics_ticks_including_hold(tmp_path, writer):
    env, camera = environment()
    with record_video(env, tmp_path/'report.mp4'):
        for _ in range(400):
            env.tick_monitor(env)
    assert camera.frames == 101  # initial pose plus all four seconds, including hold
    assert writer.path == tmp_path/'report.mp4' and writer.fps == 25.
    assert len(writer.frames) == 101 and writer.closed
    assert env.tick_monitor is None


def test_failed_rollout_saves_partial_video_and_restores_monitor(tmp_path, writer):
    env, camera = environment()
    seen = []
    previous = lambda current: seen.append(current)
    env.tick_monitor = previous
    with pytest.raises(RuntimeError, match='physics failure'):
        with record_video(env, tmp_path/'partial.mp4'):
            env.tick_monitor(env)
            raise RuntimeError('physics failure')
    assert writer.closed
    assert env.tick_monitor is previous
    assert seen == [env]


def test_video_refuses_overwrite_and_parallel_environment(tmp_path):
    env, _ = environment()
    path = tmp_path/'existing.mp4'; path.write_bytes(b'preserved')
    with pytest.raises(ValueError, match='new .mp4'):
        with record_video(env, path):
            pass
    assert path.read_bytes() == b'preserved'
    env.num_envs = 256
    with pytest.raises(ValueError, match='one environment'):
        with record_video(env, tmp_path/'parallel.mp4'):
            pass
