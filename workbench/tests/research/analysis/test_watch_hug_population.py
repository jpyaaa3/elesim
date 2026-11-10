"""Offline reporting must survive UI loss without controlling training."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from workbench.research.analysis import watch_hug_population as watch


def test_pid_identity_rejects_zombie_and_reused_process(tmp_path):
    proc=tmp_path/'123';proc.mkdir()
    experiment=tmp_path/'experiment.json'
    command=['python','-m','workbench.research.experiments.hug_c.experiment','train','--experiment',str(experiment)]
    (proc/'cmdline').write_bytes(b'\0'.join(x.encode() for x in command)+b'\0')
    (proc/'stat').write_text('123 (python worker) R 1 2 3')
    assert watch.training_alive(123,experiment,tmp_path)
    (proc/'stat').write_text('123 (python worker) Z 1 2 3')
    assert not watch.training_alive(123,experiment,tmp_path)
    (proc/'stat').write_text('123 (python worker) R 1 2 3')
    (proc/'cmdline').write_bytes(b'foreign-job\0')
    assert not watch.training_alive(123,experiment,tmp_path)


def test_milestones_wait_for_complete_generation_and_pick_progress(tmp_path):
    (tmp_path/'population.json').write_text(json.dumps({'num_envs':512}))
    d=tmp_path/'generation-0004';d.mkdir()
    assert watch.milestone_policy(tmp_path,500) is None
    (d/'summary.json').write_text(json.dumps({'training_env_steps':500*512*16,
        'reports':[{'safe':1},{'contact':1,'safe':1},{'grasp':1,'safe':1}]}))
    path,updates=watch.milestone_policy(tmp_path,500)
    assert updates==500 and path==d/'member-02/policy.npz'
    assert watch.milestone_policy(tmp_path,1000) is None


def fake_execute(command,**kwargs):
    from elesim_sim.rl.trajectory import Trajectory
    output=Path(command[command.index('--output')+1])
    if 'workbench.research.analysis.record_hug_video' in command:
        video=Path(command[command.index('--video')+1]);video.write_bytes(b'video fixture')
        Trajectory(np.array([0.,.01]),np.zeros((2,4)),('a','b','c','d'),{},
                   {'collection':{'success':False,'fault':False},
                    'report_camera':{'name':'arm-side-oblique-v1'}}).save(output)
        return SimpleNamespace(returncode=1)
    output.write_text('{}')
    return SimpleNamespace(returncode=2)


def test_failed_grasp_is_a_completed_report_and_gpu_is_pinned(tmp_path):
    source=tmp_path/'source.npz';source.write_bytes(b'policy fixture')
    calls=[]
    def execute(command,**kwargs):
        calls.append((command,kwargs));return fake_execute(command,**kwargs)
    report=watch.capture(tmp_path,tmp_path/'report',source,'GPU-test',execute=execute)
    assert report['status']=='complete' and not report['collection']['success']
    assert calls[0][1]['cwd']==watch.REPO
    assert calls[0][1]['env']['CUDA_VISIBLE_DEVICES']=='GPU-test'
    assert calls[0][0][3]=='workbench.research.analysis.record_hug_video'
    assert report['report_camera']['name']=='arm-side-oblique-v1'
    assert 'train' not in calls[0][0]


def test_replay_cli_error_is_not_a_completed_validation(tmp_path):
    source=tmp_path/'source.npz';source.write_bytes(b'policy fixture')
    report=watch.capture(tmp_path,tmp_path/'report',source,'GPU-test',execute=fake_execute,replay=True)
    assert report['status']=='error' and report['replay_returncode']==2


def test_once_without_milestone_does_not_touch_training(tmp_path,monkeypatch):
    (tmp_path/'launch.json').write_text(json.dumps({'train_pid':123,'gpu_uuid':'GPU-test'}))
    run=tmp_path/'runs/anchor';run.mkdir(parents=True)
    (run/'population.json').write_text(json.dumps({'num_envs':512}))
    monkeypatch.setattr(watch,'training_alive',lambda *args:True)
    monkeypatch.setattr(watch,'capture',lambda *args,**kwargs: (_ for _ in ()).throw(AssertionError('unexpected collect')))
    assert watch.main(['--root',str(tmp_path),'--case','anchor','--once'])==0
    status=json.loads((tmp_path/'server-reports/watch-status.json').read_text())
    assert status['phase']=='idle' and status['tasks']=={}
