import math
import unittest
import numpy as np
from workbench.research.analysis import hugmap_contact_geometry as g
from workbench.research.support import hugmap as h

class ContactGeometryTests(unittest.TestCase):
    def test_closed_form_chain_matches_joint_accumulation(self):
        rng=np.random.default_rng(74)
        for q in [np.zeros(2),np.array([np.pi,-np.pi]),np.array([1e-13,-1e-13]),*rng.uniform(-np.pi,np.pi,(30,2))]:
            hinges,angles=h.fk_frames(q)
            for part in range(len(h.PROFILES)):
                p,a=g.frame(q,part)
                np.testing.assert_allclose(p,hinges[min(part,10)],atol=3e-16)
                self.assertAlmostEqual(a,angles[min(part,10)],places=14)

    def test_vertex_vertex_centers_and_no_real_intersection(self):
        a=dict(kind='v',point=np.array([0.,0.]));b=dict(kind='v',point=np.array([1.,0.]))
        for center in g.centers(a,b,1):
            self.assertAlmostEqual(np.linalg.norm(center-a['point']),1)
            self.assertAlmostEqual(np.linalg.norm(center-b['point']),1)
        self.assertEqual(g.centers(a,b,.4),[])
        self.assertEqual(g.centers(a,a,1),[])

    def test_edge_vertex_and_edge_edge_centers(self):
        a=dict(kind='e',point=np.zeros(2),t=np.array([1.,0.]),n=np.array([0.,1.]),length=10.)
        b=dict(kind='v',point=np.array([1.,1.]))
        candidates=g.centers(a,b,1)
        self.assertEqual(len(candidates),2)
        for c in candidates:
            self.assertAlmostEqual(c[1],1)
            self.assertAlmostEqual(np.linalg.norm(c-b['point']),1)
        b=dict(kind='e',point=np.zeros(2),t=np.array([0.,-1.]),n=np.array([1.,0.]),length=10.)
        np.testing.assert_allclose(g.centers(a,b,2),[[2,2]])
        self.assertEqual(g.centers(a,a,2),[])

    def test_circle_friction_matches_normal_chord_calculation(self):
        for angle in [.3,1.,2.,2.9]:
            radius=.075;p1=np.array([radius,0]);p2=radius*np.array([math.cos(angle),math.sin(angle)])
            direct=h.friction_score([p1,p2],[-p1,-p2])['mu_critical']
            self.assertAlmostEqual(g.circle_mu(p1,p2,radius),direct,places=12)

    def test_physical_face_pair_formula_and_linear_diameter(self):
        q=np.radians([60.,-160.]);specs=[('node3','e',4),('gripper_claw_left','e',6)]
        fs=[g.feature(q,*s) for s in specs]
        c1=g.centers(*fs,.05)[0];c2=g.centers(*fs,.1)[0];mid=g.centers(*fs,.075)[0]
        np.testing.assert_allclose(mid,(c1+c2)/2,atol=1e-15)
        p1,p2=[g.contact_point(f,mid) for f in fs]
        self.assertAlmostEqual(g.circle_mu(p1,p2,.075),abs(1/math.tan((q[0]/5+q[1])/2)),places=12)

if __name__=='__main__':unittest.main()
