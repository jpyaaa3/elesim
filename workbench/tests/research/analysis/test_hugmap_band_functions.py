import math
from pathlib import Path
import json
import shutil
import subprocess
import unittest
import numpy as np
from workbench.research.analysis import hugmap_band_functions as f
from workbench.research.analysis import hugmap_contact_geometry as geom
from workbench.research.support import hugmap as h

class BandFunctionTests(unittest.TestCase):
    def test_independent_bands_and_friction_margin(self):
        p=(-70,315,150)
        result=f.evaluate(*p)
        self.assertIsNotNone(result['bands'][0])
        self.assertGreater(f.band1(*p),0)
        self.assertAlmostEqual(f.band1(*p,.6)-f.band1(*p,.5),.1,places=12)
        for band in result['bands']:
            if band is None:continue
            self.assertLess(band['seg2_deg'],0)
            center=h.local_center((band['roll_deg']/90)*p[0],p[1])
            self.assertIsNotNone(h.inspect_pose(np.radians([band['seg1_deg'],band['seg2_deg']]),center,math.inf,p[2]/2000))

    def test_c_and_s_both_obey_away_from_camera_policy(self):
        for point,kind in [((-115,220,150),'C'),((-70,315,150),'S')]:
            result=f.evaluate(*point)['bands'][0]
            self.assertEqual(result['roll_deg'],-90)
            self.assertLess(result['seg2_deg'],0)
            self.assertEqual('C' if result['seg1_deg']*result['seg2_deg']>0 else 'S',kind)
            self.assertGreater(result['score'],0)
        # Negating the legacy joint angles is not equivalent to resolving the
        # asymmetric gripper geometry in the other roll orientation.
        center=h.local_center(103.56528384756916,240.5360917836467)
        self.assertIsNone(h.inspect_pose(np.radians([14,-174]),center,.5,.075))

    def test_opposite_claw_corner_camera_condition(self):
        self.assertEqual(f.camera_boundary(150,7)['camera_clear_intervals_deg'],[[0.,90.]])
        old=f.camera_boundary(150,0)
        self.assertAlmostEqual(old['boundaries'][0]['phi_deg'],170.2383418628644,places=8)

    def test_known_friction_only_boundary_case(self):
        p=(-115,380,190)
        self.assertLess(f.band2(*p,.5),0)
        self.assertGreater(f.band2(*p,.51),0)
        self.assertAlmostEqual(f.evaluate(*p)['bands'][1]['mu_required'],.5033696520934544,places=10)

    def test_domain_and_protected_body(self):
        self.assertEqual(f.band1(1,200,150),-math.inf)
        self.assertEqual(f.evaluate(0,0,150)['status'],'body_excluded')
        with self.assertRaises(ValueError):f.evaluate(0,200,0)
        with self.assertRaises(ValueError):f.evaluate(float('nan'),200,150)

    def test_explicit_boundary_satisfies_critical_friction(self):
        checked=0
        for q1 in range(0,181,5):
            point=f.friction_boundary(q1,190)
            if point is None or not point['valid_subarc']:continue
            q=np.radians([point['q1'],point['q2']]);c=h.local_center(point['x'],point['y'])
            pair=[geom.feature(q,'node3','e',4),geom.feature(q,'gripper_claw_left','e',6)]
            points=[geom.contact_point(feature,c) for feature in pair]
            self.assertAlmostEqual(geom.circle_mu(*points,.095),.5,places=10)
            checked+=1
        self.assertGreater(checked,0)

    @unittest.skipUnless(shutil.which('node'),'Node unavailable')
    def test_preview_modes_keep_reference_threshold_explicit(self):
        path=Path(h.__file__).with_name('hugmap_band_functions_template.html')
        doc=dict(source_sha256='test',maps=[dict(D=d,observed=[[-100,250,1]],values=[[-100,250,[.3,-14,174],None],[-115,380,None,[.503,108,-148]]],boundary=[]) for d in [100,200]])
        js=r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');const html=fs.readFileSync(process.argv[1],'utf8'),payload=fs.readFileSync(0,'utf8'),script=html.match(/<script>\n([\s\S]*?)<\/script>/)[1],ctx=new Proxy({},{get:()=>()=>{}}),nodes={};
function node(id){return nodes[id]??={textContent:id==='data'?payload:'',value:id==='diameter'?'0':id==='band'?'2':id==='mu'?'0.5':'function',checked:true,clientWidth:600,clientHeight:500,getContext:()=>ctx,getBoundingClientRect:()=>({left:0,top:0})};}
const context={document:{getElementById:node},devicePixelRatio:1,ResizeObserver:class{observe(){}},console};vm.createContext(context);vm.runInContext(script,context);
assert(node('status').textContent.includes('Function region: 0'));node('mu').value='.51';vm.runInContext('draw()',context);assert(node('status').textContent.includes('Function region: 1'));
node('view').value='compare';vm.runInContext('draw()',context);assert.equal(node('mu').value,.5);assert(node('mu').disabled);node('band').value='1';vm.runInContext('draw()',context);assert(node('status').textContent.includes('Shared: 1'));
"""
        result=subprocess.run(['node','-e',js,str(path)],input=json.dumps(doc),text=True,capture_output=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
