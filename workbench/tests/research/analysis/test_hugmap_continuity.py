import unittest
import numpy as np
from workbench.research.analysis import hugmap_continuity as c
from workbench.research.analysis import hugmap_band_functions as f

class ContinuityTests(unittest.TestCase):
    def test_alternative_path_avoids_greedy_switch(self):
        layers=[[dict(q_deg=[q,0],mu=m) for q,m in pairs] for pairs in
                [[(0,.1),(100,.2)],[(1,.2),(101,.1)],[(2,.2),(102,.1)]]]
        result=c.bottleneck_path(layers)
        self.assertEqual(result['max_step_deg'],1)
        self.assertEqual(c.bottleneck_path(layers,[0,0],[102,0])['max_step_deg'],101)
        self.assertIsNone(c.bottleneck_path([layers[0],[],layers[-1]]))

    def test_branches_match_existing_scalar_function(self):
        point=(-110,340,100);old=f.evaluate(*point)
        for roll,band in zip((-90,90),old['bands']):
            roots=c.branches(*point,roll)['candidates']
            self.assertAlmostEqual(roots[0]['mu'],band['mu_required'],places=9)
            self.assertTrue(all(r['q_deg'][1]<0 for r in roots))

    def test_refinement_of_regular_local_branch(self):
        result=c.trace((-110,340,100),(-110,340.1,100),-90,4)
        self.assertIsNotNone(result['feasible_path'])
        self.assertLess(result['feasible_path']['max_step_deg'],1)

    def test_missing_vertex_and_edge_transitions_are_recovered(self):
        for point in [(-94.5,165,105),(-93.40625,165,105)]:
            self.assertLess(c.required_mu(*point,-90),.5)
            self.assertFalse(c.branches(*point,-90,catalog=c.h.CATALOG)['candidates'])

if __name__=='__main__':unittest.main()
