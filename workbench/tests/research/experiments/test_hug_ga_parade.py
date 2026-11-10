from types import SimpleNamespace
import numpy as np
import pytest
from workbench.research.experiments.hug_ga.parade import due,overview,record


def test_generation_numbers_are_one_based():
    assert [i for i in range(1,101) if due(i)]==list(range(10,101,10))
    assert not due(0)


def test_overview_contains_entire_population_in_vertical_fov():
    offsets=np.array([[i%12*3,i//12*3,0] for i in range(128)])
    view=overview(offsets);pos=np.array(view['pos']);center=np.array(view['lookat'])
    forward=(center-pos)/np.linalg.norm(center-pos)
    right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
    up=np.cross(right,forward)
    for offset in offsets:
        for delta in ([-1,-1,0],[1.2,1.2,1.5]):
            ray=offset+delta-pos
            depth=np.dot(ray,forward)
            assert depth>0
            assert abs(np.dot(ray,up))<depth*np.tan(np.deg2rad(25))
            assert abs(np.dot(ray,right))<depth*np.tan(np.deg2rad(25))*16/9


def test_record_cleans_up_on_failure_and_only_writes_raw_frames(tmp_path,monkeypatch):
    from workbench.research.experiments.hug_interpolation import video
    frames=[];closed=[];observed=[]
    monkeypatch.setattr(video,'make_writer',lambda p,fps:SimpleNamespace(append_data=frames.append,close=lambda:closed.append(True)))
    frame=np.zeros((2,3,3),np.uint8)
    previous=lambda env:observed.append(env)
    env=SimpleNamespace(s={'dt':.01},tick_monitor=previous,scene=SimpleNamespace(cameras={'parade':SimpleNamespace(render=lambda:(frame,))}))
    with pytest.raises(RuntimeError):
        with record(env,tmp_path/'parade.mp4'):
            for _ in range(12):env.tick_monitor(env)
            raise RuntimeError('stop')
    assert len(frames)==3 and all(f is frame for f in frames)
    assert len(observed)==12 and closed==[True] and env.tick_monitor is previous
