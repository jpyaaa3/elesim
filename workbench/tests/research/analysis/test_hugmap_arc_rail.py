import unittest
import math
import numpy as np
from workbench.research.analysis import hugmap_arc_rail as a
from workbench.research.support import hugmap as h

class ArcRailTests(unittest.TestCase):
    def test_endpoint_is_exact_discrete_chain(self):
        for q in [-math.pi,-.2,0,.2,math.pi]:
            hinges,_=h.fk_frames([q,0]);np.testing.assert_allclose(a.endpoint(q),hinges[5],atol=1e-14)

    def test_exact_side_edge_circle_relation(self):
        for q1 in [-2.,-.1,.1,2.]:
            hinges,angles=h.fk_frames([q1,0]);k=.025/math.tan(q1/10);K=np.array([-k,-.025]);R=.075
            for j in range(5):
                for t in [-.01,0,.01]:
                    c=hinges[j]+a.rotation(angles[j])@np.array([.035+R,.025+t])
                    self.assertAlmostEqual(float(np.sum((c-K)**2)),(k+.035+R)**2+t*t,places=12)

    def test_closed_form_inverse_and_friction(self):
        count=0
        for q1 in [-60,30,90,150]:
            for q2 in [-175,-150,-125]:
                for c in a.forward(q1,q2,150,90,False):
                    self.assertAlmostEqual(a.seg1_from_center(c['x'],c['y'],150,90),q1,places=7)
                    self.assertEqual(c['mu']<.5,c['normal_dot']<-.6)
                    count+=1
        self.assertGreater(count,10)

    def test_roll_is_only_coordinate_transform(self):
        left=a.forward(60,-150,150,-90,False);right=a.forward(60,-150,150,90,False)
        self.assertTrue(left)
        for l,r in zip(left,right):
            self.assertEqual(l['x'],-r['x']);self.assertEqual(l['y'],r['y']);self.assertEqual(l['mu'],r['mu'])

if __name__=='__main__':unittest.main()
