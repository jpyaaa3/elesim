"""Keep diagnostic bridges bounded and policy handoff permanent."""
import numpy as np
from workbench.research.analysis.probe_reverse_bridge import bridge_actions


def test_handoff_does_not_pull_policy_back_to_bridge_target():
    path=np.zeros((491,4));mean=np.array([[.2,-.4,.6,-.8]])
    output,handed=bridge_actions(np.ones((1,4)),mean,path,[400],[400],['direct'],10,np.ones(4),already_handed={0:dict(step=3)})
    np.testing.assert_array_equal(output,mean)
    assert handed.tolist()==[True]


def test_proposal_obeys_existing_action_bounds_and_reaches_recorded_target():
    path=np.zeros((491,4));path[350]=[4,-4,2,-2]
    out,handed=bridge_actions(np.zeros((1,4)),np.zeros((1,4)),path,[400],[400],['direct'],0,np.ones(4))
    np.testing.assert_array_equal(out,[[1,-1,1,-1]])
    assert not handed[0]


def test_baseline_keeps_policy_output_and_backtrack_has_a_hold_before_handoff():
    path=np.zeros((491,4));mean=np.full((2,4),.3)
    out,handed=bridge_actions(np.zeros((2,4)),mean,path,[400,400],[400,400],['baseline','backtrack'],5,np.ones(4))
    np.testing.assert_array_equal(out[0],mean[0]);assert handed.tolist()==[True,False]
    _,handed=bridge_actions(np.zeros((2,4)),mean,path,[400,400],[400,400],['baseline','backtrack'],6,np.ones(4))
    assert handed.tolist()==[True,True]


def test_target_search_does_not_modify_reference_commands():
    path=np.zeros((491,4),np.float32);before=path.copy()
    out,_=bridge_actions(np.zeros((1,4)),np.zeros((1,4)),path,[400],[400],['direct'],0,
        np.array([.005,.1,.03,.03]),target_offsets=np.array([[.02,.1,0,0]]))
    np.testing.assert_array_equal(path,before)
    np.testing.assert_allclose(out,[[1,1,0,0]])
