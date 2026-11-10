"""Regressions for the Home-to-Extend-Arm exclusion envelope."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import unittest

MAPPER = Path(os.environ.get('HUGMAP_MAPPER_HTML', '/home/user/ws/hugmap/mapper.html'))


@unittest.skipUnless(MAPPER.exists() and shutil.which('node'), 'Mapper and Node required')
class MapperPreparationTests(unittest.TestCase):
    def probe(self, code):
        html = MAPPER.read_text()
        kernel = html[html.index('const rot='):html.index('let current=')]
        kernel += html[html.index('const CONTACT_OVERLAP_TOL_MM='):html.index('function muLimit()')]
        return json.loads(subprocess.check_output(['node'], input=kernel+code, text=True))

    def test_current_failed_anchor_is_rejected_before_rl(self):
        result = self.probe(r'''
const c={x:-85.18650136838987,y:222.71820844304784},p=collisionGeometry(-.5,-178.5);
const m=model(-.5,-178.5,100,-90);classifyCandidates(m,-.5,-178.5,100,-90);
const found=m.candidates.find(a=>Math.hypot(a.x-c.x,a.y-c.y)<1e-5);
console.log(JSON.stringify({final:finalObjectCollision(p,c,100,-90),combined:objectCollision(p,c,100,-90),prep:preparationReport(c,100),found}));
''')
        self.assertFalse(result['final'])
        self.assertTrue(result['combined'])
        self.assertTrue(result['prep']['blocked'])
        self.assertLess(result['prep']['minimum_gap_mm'], -8)
        self.assertTrue(result['found']['collision'])
        self.assertFalse(result['found']['finalCollision'])
        self.assertTrue(result['found']['preparation']['blocked'])

    def test_clear_placement_and_diameter_dependence(self):
        result = self.probe(r'''
console.log(JSON.stringify({clear:preparationReport({x:-300,y:350},100),small:preparationCollision({x:-110,y:350},100),large:preparationCollision({x:-110,y:350},150),
home:PREPARATION_COLLISION.home_command,extend:PREPARATION_COLLISION.extend_command,clearance:PREPARATION_COLLISION.clearance_mm,padding:PREPARATION_COLLISION.sampling_padding_mm}));
''')
        self.assertFalse(result['clear']['blocked'])
        self.assertFalse(result['small'])
        self.assertTrue(result['large'])
        self.assertEqual(result['home'][0], -.12)
        self.assertEqual(result['extend'], [-.01, 0., 0., 0.])
        self.assertEqual(result['clearance'], 1.)
        self.assertGreater(result['padding'], 0)
        self.assertLess(result['padding'], .2)

    def test_shared_unrotated_preparation_for_both_bands(self):
        result = self.probe(r'''
const report=[];
for(const roll of [-90,90]){
 const m=model(-.5,-178.5,100,roll);classifyCandidates(m,-.5,-178.5,100,roll);
 report.push(m.candidates.map(c=>({x:c.x,blocked:c.preparation.blocked})));
}
console.log(JSON.stringify(report));
''')
        self.assertTrue(result[0])
        self.assertEqual([r['blocked'] for r in result[0]], [r['blocked'] for r in result[1]])

    def test_no_contact_tolerance_in_preparation(self):
        result = self.probe(r'''
// Isolate a synthetic swept part, preserving the real preparation criterion.
PREPARATION_COLLISION.parts=[{name:'test',poly:[[0,0],[10,0],[10,10],[0,10]]}];
preparationBounds=null;
const threshold=PREPARATION_COLLISION.clearance_mm+PREPARATION_COLLISION.sampling_padding_mm;
console.log(JSON.stringify([-.5,0,.5,threshold-.0001,threshold+.0001].map(gap=>preparationCollision({x:15+gap,y:5},10))));
''')
        self.assertEqual(result, [True, True, True, True, False])

    def test_rl_forward_and_inverse_export_use_the_same_filter(self):
        from workbench.research.experiments.hug_c.source import read_kernel, evaluate, inverse
        kernel, _ = read_kernel(MAPPER)
        self.assertEqual(evaluate(kernel, 100, [[-.5, -178.5]]), [None])
        self.assertEqual(inverse(kernel, 100, [-85.18650136838987, 222.71820844304784],
                                 [dict(q_deg=[-.5, -178.5], feature='tip', root=1)]), [])
        self.assertIsNotNone(evaluate(kernel, 100, [[-89.5, -162.5]])[0])

    def test_baked_envelope_contains_independent_fk_between_samples(self):
        import numpy as np
        from scipy.spatial import ConvexHull
        from workbench.research.analysis.check_hug_approach import ObjectGeometry
        payload = self.probe('console.log(JSON.stringify(PREPARATION_COLLISION));')
        parts = {p['name']: p for p in payload['parts']}
        geometry = ObjectGeometry(dict(settings=dict(spawn_height_m=.32)),
                                  dict(object_xyz_m=[.42121820844304786, .08518650136838987, .55], diameter_mm=100))
        for t in (0., .0137, .1273, .3671, .7319, .9981, 1.):
            command = np.array(payload['home_command'])*(1-t)+np.array(payload['extend_command'])*t
            q = [command[0]*1000, np.degrees(command[1]), *np.degrees(command[2:])*5]
            for name, vertices in geometry.polygons(q):
                points = np.c_[-vertices[:, 1], vertices[:, 0]-.1985]*1000
                equations = ConvexHull(np.array(parts[name]['poly'])).equations
                self.assertLessEqual(float(np.max(points@equations[:, :2].T+equations[:, 2])),
                                     payload['sampling_padding_mm'], (name, t))

    def test_preset_constants_match_the_ui_buttons(self):
        import ast
        from workbench.research.experiments.hug_interpolation.configure import ROOT
        from workbench.research.experiments.wrap_grasp.app_config import UI_PRESETS
        tree = ast.parse((ROOT/'payload/runtime/docker/pilot/app/elesim_pilot/pick/actions.py').read_text())
        for name, preset in [('home_controls', 'home'), ('extend_arm_controls', 'extend_arm')]:
            method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
            call = next(n for n in ast.walk(method) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'apply_control_u')
            self.assertEqual({k.arg: ast.literal_eval(k.value) for k in call.keywords if k.arg.startswith('u_')},
                             UI_PRESETS[preset])

    def test_rebuild_is_idempotent(self):
        import re
        from workbench.research.analysis.rebuild_mapper_collision import rebuild
        html = MAPPER.read_text()
        payload = json.loads(re.search(r'const PREPARATION_COLLISION=(.*);', html).group(1))
        self.assertEqual(rebuild(html, payload), html)

    def test_visibility_checkbox_does_not_disable_collision_filter(self):
        import re
        from workbench.research.analysis.benchmark_mapper_gaps import STUB
        script = re.search(r'<script>(.*?)</script>', MAPPER.read_text(), re.S).group(1)
        code = r'''
const c={x:-85.18650136838987,y:222.71820844304784},p=collisionGeometry(-.5,-178.5);
$('preparation').checked=false;$('preparation').oninput();const off=objectCollision(p,c,100,-90);
$('preparation').checked=true;$('preparation').oninput();const on=objectCollision(p,c,100,-90);
$('plane').value='q';$('plane').onchange();const disabled=$('preparation').disabled;
console.log(JSON.stringify({off,on,disabled}));
'''
        result = json.loads(subprocess.check_output(['node'], input=STUB+script+code, text=True))
        self.assertEqual(result, dict(off=True, on=True, disabled=True))


if __name__ == '__main__':
    unittest.main()
