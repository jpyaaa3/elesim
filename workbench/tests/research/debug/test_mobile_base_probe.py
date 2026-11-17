import numpy as np
import pytest
from elesim_pilot.mobile_manipulation.base_executor import BaseExecutor
from elesim_pilot.mobile_manipulation.contracts import Settings
from workbench.research.debug.profile_mobile_base import make_executor


@pytest.mark.parametrize('pose',[[0.,0.,0.],[.22,.01,.001],[.222,.01,0.],[.22,.01,.02],[0.,0.,3.14]])
def test_isolated_probe_uses_the_actual_walk_stop_level_executor(pose):
    settings=Settings();goal=np.array([.222,.01,0.]);reach=.5
    probe=make_executor(dict(settings=settings.__dict__,home_q=[0.,0.,0.,0.],target_reach_m=reach),goal,pose,lambda:0.)
    assert isinstance(probe,BaseExecutor)
    command,done=probe.step(pose,0.,0.,lambda *a:True,posture_ready=False)
    assert not done and np.isfinite(command).all()
    if np.linalg.norm(goal[:2]-pose[:2])<settings.base_position_tolerance_m and abs(pose[2])<settings.base_yaw_tolerance_rad:
        assert probe.stage=='leveling' and not command.any()
    else:assert probe.stage=='approach' and command.any()


@pytest.mark.parametrize('suffix',['.run.json','.summary.json','.state.jsonl','.profile.jsonl'])
def test_existing_probe_output_prevents_a_duplicate_physics_run(monkeypatch,tmp_path,capsys,suffix):
    from workbench.research.debug import profile_mobile_base
    result=tmp_path/('case'+suffix);result.write_text('{"already_finished":true}')
    monkeypatch.setattr('sys.argv',['probe','--case','unused.json','--config','unused.yaml',
        '--model-bundle','unused','--output',str(tmp_path/'case')])
    with pytest.raises(SystemExit) as error:profile_mobile_base.main()
    assert error.value.code==2 and 'output already exists' in capsys.readouterr().err
    assert result.read_text()=='{"already_finished":true}'
