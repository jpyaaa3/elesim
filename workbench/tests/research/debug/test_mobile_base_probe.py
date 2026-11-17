from types import SimpleNamespace
import numpy as np
import pytest
from elesim_pilot.mobile_manipulation.base_executor import BaseExecutor
from elesim_pilot.mobile_manipulation.contracts import Settings
from workbench.research.debug.profile_mobile_base import goal_command


@pytest.mark.parametrize('pose',[[0.,0.,0.],[.22,.01,.001],[.222,.01,0.],[.22,.01,.02],[0.,0.,3.14]])
def test_isolated_probe_matches_the_actual_base_executor_command(pose):
    settings=Settings();goal=np.array([.222,.01,0.]);reach=.5
    expected=BaseExecutor(settings,lambda:0.)
    expected.start(SimpleNamespace(path=np.array([pose,goal]),report=dict(kind='grasp',target_reach_m=reach)),pose,1)
    command,_=expected.step(pose,0.,0.,lambda *a:True)
    actual,close=goal_command(pose,goal,settings.__dict__,reach)
    np.testing.assert_allclose(actual,command)


@pytest.mark.parametrize('suffix',['.run.json','.summary.json','.state.jsonl','.profile.jsonl'])
def test_existing_probe_output_prevents_a_duplicate_physics_run(monkeypatch,tmp_path,capsys,suffix):
    from workbench.research.debug import profile_mobile_base
    result=tmp_path/('case'+suffix);result.write_text('{"already_finished":true}')
    monkeypatch.setattr('sys.argv',['probe','--case','unused.json','--config','unused.yaml',
        '--model-bundle','unused','--output',str(tmp_path/'case')])
    with pytest.raises(SystemExit) as error:profile_mobile_base.main()
    assert error.value.code==2 and 'output already exists' in capsys.readouterr().err
    assert result.read_text()=='{"already_finished":true}'
