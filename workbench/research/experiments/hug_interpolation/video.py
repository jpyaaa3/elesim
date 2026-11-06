"""Optional, single-environment report video sampled at physics ticks."""
from contextlib import contextmanager
from pathlib import Path


def make_writer(path, fps):
    import imageio.v2 as imageio
    return imageio.get_writer(str(path), fps=fps, codec='libx264', macro_block_size=1)


@contextmanager
def record_video(env, path, *, every=4):
    if path is None:
        yield
        return
    path = Path(path)
    if path.suffix.lower() != '.mp4' or path.exists():
        raise ValueError('Choose a new .mp4 report video')
    if env.num_envs != 1 or every < 1:
        raise ValueError('Report video requires one environment and a positive stride')
    path.parent.mkdir(parents=True, exist_ok=True)
    camera = env.scene.cameras['report']
    writer = make_writer(path, 1/(env.s['dt']*every))
    previous = env.tick_monitor
    ticks = 0

    def capture(current):
        nonlocal ticks
        ticks += 1
        if ticks % every == 0:
            writer.append_data(camera.render()[0])
        if previous is not None:
            previous(current)

    env.tick_monitor = capture
    try:
        writer.append_data(camera.render()[0])
        yield
    finally:
        env.tick_monitor = previous
        writer.close()
