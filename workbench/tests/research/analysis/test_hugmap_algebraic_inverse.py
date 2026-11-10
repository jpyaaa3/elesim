"""Checks for finite-harmonic contact equations and unseeded inverse recovery."""
import unittest
import numpy as np
from numpy.polynomial import polynomial as poly
from workbench.research.analysis import hugmap_algebraic_inverse as inv
from workbench.research.analysis import hugmap_contact_geometry as geom
from workbench.research.support import hugmap as h

class AlgebraicInverseTests(unittest.TestCase):
    def test_finite_fourier_conversion_at_unseen_angles(self):
        rng=np.random.default_rng(12)
        for m in range(1,6):
            a=rng.normal(size=m+1);b=rng.normal(size=m+1)
            def f(t):return sum(a[k]*np.cos(k*t)+b[k]*np.sin(k*t) for k in range(m+1))
            coeff=inv.tan_polynomial(f,m)
            for t in rng.uniform(-.6,.6,15):
                u=np.tan(t/2)
                self.assertAlmostEqual(poly.polyval(u,coeff)/(1+u*u)**m,f(t),places=11)

    def test_both_roots_and_degenerate_equation(self):
        roots=inv.angle_roots(lambda a:np.cos(a)-np.cos(.2),1)
        np.testing.assert_allclose(roots,[-1.,1.],atol=1e-12)
        with self.assertRaises(ArithmeticError):inv.angle_roots(lambda a:0.,3)

    def test_cached_polynomials_match_physical_residual(self):
        center=np.array([-.1,.2]);radius=.075
        for spec in [('node1','e',1),('node2','v',2),('node3','e',4),('gripper_claw_left','e',6),('gripper_claw_left','v',0),('node9','v',4)]:
            part=h.PART_NAMES.index(spec[0]);m=part+1 if part<5 else min(part-4,5)
            constant,xterm,yterm,den=inv.feature_polynomials(spec)
            for angle in [-.5,.02,.43]:
                q=[5*angle,0] if part<5 else [0,5*angle]
                extra=center@center-radius*radius if spec[1]=='v' else -radius
                coeff=constant+center[0]*xterm+center[1]*yterm+extra*den
                u=np.tan(angle/2)
                self.assertAlmostEqual(poly.polyval(u,coeff)/(1+u*u)**m,inv.residual(q,center,radius,spec),places=12)

    def test_second_frame_transformation(self):
        center=np.array([-.1,.2]);q1=.9;q2=-2.8
        for spec in [('gripper_claw_left','e',6),('gripper_claw_left','v',0),('node9','e',4)]:
            self.assertAlmostEqual(inv.residual([q1,q2],center,.075,spec),
                inv.residual([0,q2],inv.second_center(q1,center),.075,spec),places=14)

    def test_printed_band2_equations_match_polygon_sides(self):
        center=np.array([-.12,.19]);radius=.075
        for q1,q2 in [(1.,-2.7),(.1,-3.),(2.,-2.4)]:
            a,b=q1/5,q2/5
            first=center[0]*np.cos(4*a)+center[1]*np.sin(4*a)-.05*sum(np.sin(k*a) for k in range(1,4))-.035-radius
            self.assertAlmostEqual(first,inv.residual([q1,q2],center,radius,('node3','e',4)),places=14)
            cprime=geom.rotate(center-geom.chain_sum(5,a),-q1)
            second=cprime[0]*np.cos(5*b)+cprime[1]*np.sin(5*b)-.05*sum(np.sin(k*b) for k in range(1,5))-.03546-radius
            self.assertAlmostEqual(second,inv.residual([q1,q2],center,radius,('gripper_claw_left','e',6)),places=14)

    def test_inverse_from_center_without_joint_seed(self):
        cases=[(-103.56528384756916,240.5360917836467,[-14,174],
                [('node1','e',1),('gripper_claw_left','v',0)]),
               (-1.6766084705128883,382.3886573739376,[56,-152],
                [('node3','e',4),('gripper_claw_left','e',6)])]
        catalog=[c[3] for c in cases]
        for x,y,expected,specs in cases:
            center=h.local_center(x,y)
            result=inv.solve_catalog(center,.075,catalog)
            self.assertFalse(result['degeneracies'])
            self.assertTrue(any(np.max(np.abs(np.array([r['seg1_deg'],r['seg2_deg']])-expected))<1e-5 for r in result['solutions']))
            for r in result['solutions']:
                self.assertIsNotNone(h.inspect_pose(np.radians([r['seg1_deg'],r['seg2_deg']]),center,.5,.075))

if __name__=='__main__':unittest.main()
