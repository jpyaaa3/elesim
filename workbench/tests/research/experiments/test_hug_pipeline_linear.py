"""Linear budget and endpoint contracts without GPU or simulator dependencies."""
import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import numpy as np

from workbench.research.experiments.hug_pipeline.linear import (
    LinearBudget,LinearHoldTracker,at_target,check_trajectory,motor_position,validate_resets)
from workbench.research.experiments.hug_pipeline.policy import NumpyPolicy,OBSERVATIONS,ACTIONS


def meters(motor_deg):return -np.asarray(motor_deg,dtype=float)/1500


class LinearContractTests(unittest.TestCase):
    def test_hold_needs_continuous_five_seconds_at_zero_and_exempts_no_wrong_pose(self):
        from workbench.research.experiments.hug_c.configure import DEFAULTS
        q=np.array([[0.,0.,-.2,-.3]])
        owner=SimpleNamespace(num_envs=1,s=dict(DEFAULTS),linear_enforced=True,
                              mapper=SimpleNamespace(waypoint=q.copy()))
        h=LinearHoldTracker(owner)
        def tick():return h.update(np.array([.03]),np.array([False]),np.ones((1,2)),q,
                                   np.array([False]),np.zeros((1,3)))
        for _ in range(300):self.assertFalse(tick()[0])
        q[0,0]=meters(4);self.assertFalse(tick()[0]);self.assertEqual(h.count[0],0)
        q[0,0]=0
        for _ in range(499):self.assertFalse(tick()[0])
        self.assertTrue(tick()[0])
        owner.mapper.waypoint[0,0]=meters(4)
        self.assertFalse(tick()[0]);self.assertEqual(h.count[0],0)

    def test_budget_breach_cannot_earn_success_or_repeat_terminal_penalty(self):
        from workbench.research.experiments.hug_c.configure import DEFAULTS
        from workbench.research.experiments.hug_c.rewards import RewardLedger
        b=LinearBudget(1);b.reset([0],meters([0]));b.advance(meters([21]))
        ledger=RewardLedger(1,dict(DEFAULTS));q=np.array([[0.,0.,-.2,-.3]])
        ledger.reset([0],q,q[0])
        args=dict(confirmed_grasp=np.array([True]),lift_fraction=np.ones(1),hold_fraction=np.ones(1),
                  wrong_fraction=np.zeros(1),fault=b.exceeded,success=np.array([True]),
                  timeout=np.array([False]),drops=np.zeros(1))
        reward,done,terms=ledger.step(q,q[0],np.zeros((1,4)),**args)
        self.assertTrue(done[0]);self.assertLessEqual(reward[0],-30)
        self.assertEqual(terms['fault'][0],-30)
        for k in ['grasp','lift','hold','success']:self.assertEqual(terms[k][0],0)
        reward,_,terms=ledger.step(q,q[0],np.zeros((1,4)),**args)
        self.assertEqual(reward[0],0);self.assertEqual(terms['fault'][0],0)

    def test_home_and_curriculum_debits_include_only_minimum_home_distance(self):
        b=LinearBudget(5);b.reset(np.arange(5),meters([180,90,0,190,170]))
        np.testing.assert_allclose(b.spent,[0,90,180,10,10])
        np.testing.assert_allclose(b.remaining(),[200,110,20,190,190])
        np.testing.assert_allclose(motor_position(meters([0,180,260])),[0,180,260])

    def test_cumulative_roundtrip_cannot_cancel_or_recover_a_breach(self):
        b=LinearBudget(1);b.reset([0],meters([180]))
        for q in [0,10,0]:b.advance(meters([q]))
        self.assertAlmostEqual(b.spent[0],200);self.assertFalse(b.exceeded[0])
        b.advance(meters([.01]));self.assertTrue(b.exceeded[0])
        b.advance(meters([0]));self.assertTrue(b.exceeded[0]);self.assertEqual(b.remaining()[0],0)

    def test_midway_start_cannot_receive_a_fresh_200_degree_budget(self):
        b=LinearBudget(1);b.reset([0],meters([0]))
        b.advance(meters([11]));b.advance(meters([0]))
        self.assertAlmostEqual(b.spent[0],202);self.assertTrue(b.exceeded[0])

    def test_partial_reset_preserves_other_lanes_and_replaces_previous_command(self):
        b=LinearBudget(4);b.reset(np.arange(4),meters([180]*4));b.advance(meters([170]*4))
        b.reset([1,3],meters([0,90]));b.advance(meters([170,0,170,90]))
        np.testing.assert_allclose(b.spent,[10,180,10,90])

    def test_endpoint_requires_both_feedback_and_command(self):
        actual=meters([0,3,3.1,0,-4]);cmd=meters([0,0,0,4,0])
        np.testing.assert_array_equal(at_target(actual,cmd),[True,True,False,False,False])

    def test_infeasible_teacher_bins_are_rejected(self):
        validate_resets(meters([0,90,180,190]))
        with self.assertRaisesRegex(ValueError,'cannot reach'):validate_resets(meters([190.1]))

    def test_trajectory_gate_checks_total_travel_and_final_position(self):
        c=np.zeros((4,4));c[:,0]=meters([180,0,10,0])
        self.assertAlmostEqual(check_trajectory(c),200)
        c[:,0]=meters([180,0,11,0])
        with self.assertRaisesRegex(ValueError,'cumulative'):check_trajectory(c)
        c[:,0]=meters([180,120,60,4])
        with self.assertRaisesRegex(ValueError,'Final linear'):check_trajectory(c)

    def test_legacy_successful_paths_fail_new_linear_contract(self):
        # Observed old policies: Home 180 -> 0 -> 260; or 180 -> 260 -> 0.
        for positions in ([180,0,260],[180,260,0]):
            c=np.zeros((3,4));c[:,0]=meters(positions)
            with self.assertRaisesRegex(ValueError,'cumulative'):check_trajectory(c)

    def test_remaining_budget_changes_actor_output_and_old_schema_is_rejected(self):
        arrays=dict(schema=np.asarray('elesim-hug-pipeline-linear-policy'),schema_version=np.asarray(1),
            observation_names=np.asarray(OBSERVATIONS),action_names=np.asarray(ACTIONS),
            metadata_json=np.asarray(json.dumps({})),norm_mean=np.zeros(21),norm_std=np.ones(21),
            norm_eps=np.asarray(1e-8),n_layers=np.asarray(1),w0=np.zeros((4,21)),b0=np.zeros(4))
        arrays['w0'][0,-1]=1
        with TemporaryDirectory() as d:
            p=Path(d)/'policy.npz';np.savez(p,**arrays);policy=NumpyPolicy(p)
            obs=np.zeros((2,21));obs[1,-1]=1
            self.assertAlmostEqual(policy(obs)[1,0]-policy(obs)[0,0],1,places=6)
            with self.assertRaises(ValueError):policy(np.zeros((1,20)))
            arrays['schema']=np.asarray('elesim-c-hug-policy');arrays['schema_version']=np.asarray(4)
            np.savez(p,**arrays)
            with self.assertRaises(ValueError):NumpyPolicy(p)


if __name__=='__main__':unittest.main()
