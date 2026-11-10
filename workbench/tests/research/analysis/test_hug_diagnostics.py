"""Contact certificates, fixed-length equations and honest outcome semantics."""
from dataclasses import replace
import importlib.util
import math
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.optimize import least_squares

from workbench.research.analysis import hug_diagnostics as study


class ContactTests(unittest.TestCase):
    points = [[-1.,0.],[1.,0.]]
    normals = [[1.,0.],[-1.,0.]]

    def test_frictionless_two_point_contacts_do_not_constrain_planar_rotation(self):
        result = study.planar_force_closure(self.points,self.normals,0.)
        self.assertFalse(result['closure'])
        self.assertEqual(result['rank'],1)

    def test_opposed_frictional_point_contacts_have_planar_force_closure(self):
        result = study.planar_force_closure(self.points,self.normals,.5)
        self.assertTrue(result['closure'])
        self.assertEqual(result['rank'],3)
        self.assertGreater(result['positive_span_margin'],0.)

    def test_same_facing_normals_cannot_balance_a_squeeze(self):
        result = study.planar_force_closure(self.points,[[1,0],[1,0]],.5)
        self.assertFalse(result['closure'])

    def test_normal_force_unit_caps_bound_the_assumed_vertical_task_force(self):
        self.assertAlmostEqual(study.planar_support_capacity(self.points,self.normals,.5),1.)
        self.assertAlmostEqual(study.planar_support_capacity(self.points,self.normals,0.),0.)

    def test_force_closure_is_invariant_to_length_units(self):
        small = study.planar_force_closure(np.array(self.points)*.001,self.normals,.3)
        large = study.planar_force_closure(np.array(self.points)*1000,self.normals,.3)
        self.assertEqual(small['closure'],large['closure'])
        self.assertAlmostEqual(small['positive_span_margin'],large['positive_span_margin'])

    def test_invalid_contact_data_is_rejected(self):
        for normals,mu in (([[0,0],[1,0]],.3),(self.normals,-.1),(self.normals,float('nan'))):
            with self.subTest(normals=normals,mu=mu),self.assertRaises(ValueError):
                study.wrench_matrix(self.points,normals,mu)

    def test_timed_out_search_is_not_reported_as_physical_impossibility(self):
        self.assertEqual(study.outcome(False,True),'not_found_within_budget_budget_exhausted')
        self.assertEqual(study.outcome(True,True),'candidates_found_budget_exhausted')
        self.assertNotIn('impossible',study.outcome(False,False))

    def test_joint_headroom_exposes_a_saturated_second_section(self):
        result = study.angle_margins(np.radians([120,180]))
        self.assertAlmostEqual(result['minimum_deg'],0.)
        self.assertAlmostEqual(result['upper_deg'][1],0.)

    def test_legacy_forward_guard_rejects_deep_wrap_projection(self):
        self.assertFalse(study.legacy_forward(-.016,2e-6))
        self.assertTrue(study.legacy_forward(-1e-7,2e-6))

    def test_geometry_and_search_budgets_are_validated(self):
        for change in (dict(length_m=0),dict(budget_s=float('nan')),dict(pair_limit=0),
                       dict(turn_limit_rad=4),dict(max_solves=True)):
            with self.subTest(change=change),self.assertRaises(ValueError):
                replace(study.Settings(),**change).checked()

    def test_antipodal_chord_cannot_provide_a_finite_centroid_margin(self):
        def pair(first,second):
            return SimpleNamespace(first=SimpleNamespace(point=np.array(first)),
                second=SimpleNamespace(point=np.array(second)))
        antipodal = pair([-1.,0.],[1.,0.])
        offset = pair([-1.,.1],[1.,.1])
        self.assertEqual(study.finite_capture_pairs([antipodal,offset],.01),[offset])

    def test_study_keeps_the_existing_gripper_attached(self):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root)/'study'
            with self.assertRaisesRegex(ValueError,'attached gripper'):
                study.run_study(None,Path('unused'),output,['circle-160'],0.,study.Settings())
            self.assertFalse(output.exists())


def endpoint_witness(ref):
    return study.endpoint_witness(ref)


class ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if importlib.util.find_spec('shapely') is None:
            raise unittest.SkipTest('Full reference geometry requires Shapely; no host installation attempted')
        cls.ref = study.load_reference(study.DEFAULT_REFERENCE)

    def test_fixed_length_equations_detect_rescaling(self):
        ref = self.ref;settings = study.Settings()
        turns = np.radians([70,130]);a,second = .4,.6
        p1,p2,t1,t2 = ref._contact_geometry(*turns,a,second,'arc2',settings.length_m)
        first = SimpleNamespace(point=p1,tangent=t1)
        other = SimpleNamespace(point=p2,tangent=t2)
        delta = ref.wrap_line(t2-t1);winding = round((t2-t1-delta)/math.pi)
        values = np.r_[turns,second]
        residual = study.fixed_residual(ref,values,first,other,a,'arc2',0.,settings,winding)
        np.testing.assert_allclose(residual,0.,atol=1e-12)
        changed = study.fixed_residual(ref,values,first,other,a,'arc2',0.,
            replace(settings,length_m=.24),winding)
        self.assertGreater(np.linalg.norm(changed[:2]),.01)

    def test_endpoint_contact_does_not_require_smooth_side_tangency(self):
        target,pose,first,second = endpoint_witness(self.ref)
        residual = study.fixed_residual(self.ref,pose['turns'],first,second,pose['a'],
            'tip',0.,study.Settings(),0)
        np.testing.assert_allclose(residual,0.,atol=1e-12)
        self.assertGreater(pose['endpoint_tangency_residual'],.2)
        self.assertEqual(pose['length_m'],.25)

    def test_reflected_wrap_needs_the_opposite_first_tangent_orientation(self):
        _,pose,first,second = endpoint_witness(self.ref)
        for feature in (first,second):
            feature.point = feature.point*np.array([-1.,1.])
            feature.tangent = -feature.tangent
        turns = -np.array(pose['turns'])
        correct = study.fixed_residual(self.ref,turns,first,second,pose['a'],
            'tip',0.,study.Settings(),0,orientation=1)
        missed = study.fixed_residual(self.ref,turns,first,second,pose['a'],
            'tip',0.,study.Settings(),0,orientation=0)
        np.testing.assert_allclose(correct,0.,atol=1e-12)
        self.assertGreater(np.linalg.norm(missed),.1)

    def test_nonpenetrating_witness_has_capture_but_fails_forward_gate(self):
        target,pose,*_ = endpoint_witness(self.ref)
        result = study.assess_geometry(self.ref,target,pose,study.Settings())
        self.assertTrue(result['geometric_proxy_pass'],result)
        self.assertFalse(result['legacy_forward_pass'])
        self.assertGreater(result['body_target_clearance_m'],.018)
        self.assertLess(result['forward_projection_m'],-.016)
        self.assertIsNone(result['hold_verified'])
        self.assertIsNone(result['motor_load_margin'])
        self.assertIsNone(result['compression_margin'])

    def test_body_collision_remains_a_rejection(self):
        target,pose,*_ = endpoint_witness(self.ref)
        pose['translation'] = [0.,0.]
        result = study.assess_geometry(self.ref,target,pose,study.Settings())
        self.assertFalse(result['geometric_proxy_pass'])
        self.assertIn('body_target_collision',result['reasons'])

    def test_roundoff_at_antipodal_contact_chord_is_not_capture_margin(self):
        vertices = np.array([[-.06,-.08],[.06,-.08],[.06,.08],[-.06,.08]])
        target = self.ref.make_target(vertices,sdf_resolution=128)
        first = SimpleNamespace(point=np.array([.06,.08]),tangent=5*math.pi/6,
            normal=np.array([.5,math.sqrt(3)/2]),u=.5)
        second = SimpleNamespace(point=np.array([-.06,-.08]),tangent=-math.pi/6,
            normal=-first.normal,u=0.)
        pose = study.make_pose(self.ref,[.9097962329713023,2.655375587847905],
            first,second,.8,'tip',.155,study.Settings())
        result = study.assess_geometry(self.ref,target,pose,study.Settings())
        self.assertFalse(result['geometric_proxy_pass'])
        self.assertLess(result['centroid_margin_m'],1e-12)
        self.assertTrue(any('centroid' in reason for reason in result['reasons']))

    def test_force_proxy_never_becomes_a_measured_load_capacity(self):
        _,pose,*_ = endpoint_witness(self.ref)
        result = study.contact_proxies(pose)
        self.assertEqual([row['mu'] for row in result['by_friction']],[0.,.3,.6])
        self.assertFalse(result['by_friction'][0]['closure'])
        self.assertIn('No actuator coupling',result['assumptions'])

    def test_recovery_records_failed_searches_without_grasp_claims(self):
        target,pose,*_ = endpoint_witness(self.ref)
        result = study.local_recovery(self.ref,target,pose,study.Settings())
        self.assertEqual(result['trials'],6)
        self.assertIsNone(result['hold_verified'])
        self.assertIsNone(result['compression_margin'])
        for row in result['rows']:
            self.assertLessEqual(max(map(abs,row['corrections_deg'])),5.000001)


if __name__ == '__main__':
    unittest.main()
