import json
from pathlib import Path
import shutil
import subprocess
import unittest
import numpy as np
from workbench.research.analysis import explain_hugmap_bands as explain
from workbench.research.support import hugmap as h

class ExplanationTests(unittest.TestCase):
    def test_constraint_diagnostics_agree_with_validator(self):
        cases=[(-103.56528384756916,240.5360917836467,150,[-14,174],[]),
               (-65,345,100,[89.50700250311124,-180.3394845810082],['joint']),
               (-70,315,150,[-36.97605824043288,169.76608726878288],['object_collision']),
               (-115,380,190,[108.0518255757262,-148.17177235995848],['friction'])]
        for x,y,D,angles,expected in cases:
            q=np.radians(angles);center=h.local_center(x,y)
            result=explain.constraints(q,center,D/2000,.5)
            self.assertEqual(result['violations'],expected)
            self.assertEqual(h.inspect_pose(q,center,.5,D/2000) is not None,not expected)
            if expected==['friction']:
                self.assertIsNotNone(h.inspect_pose(q,center,.51,D/2000))
            if (x,y)==(-70,315):self.assertTrue(all(o.startswith('camera') for o in result['obstacles']))

    @unittest.skipUnless(shutil.which('node'),'Node unavailable')
    def test_explanation_viewer_modes_and_candidate_drawing(self):
        path=Path(h.__file__).with_name('hugmap_explanation_template.html')
        rows=[dict(x=-100,y=250,band=1,q1=-14,q2=174,mu=.3,features=[['node1','e',1],['gripper_claw_left','v',0]],joint_margin_deg=6)]
        candidate=dict(seg1_deg=-14,seg2_deg=174,mu_min=.3,features=rows[0]['features'],violations=['object_collision'],obstacles=['camera'])
        probe=dict(x=-105,y=250,label='object_collision',candidate_count=1,interior=False,best=candidate,reasons=[['object_collision']],recovered=[])
        doc=dict(source_sha256='test',mu_allow=.5,maps=[dict(D=d,found=rows,probes=[probe]) for d in [100,200]],summaries=[dict(D=d,bands={'1':{'count':1},'2':{'count':0}},probes={'object_collision':1}) for d in [100,200]],geometry=dict(profiles=h.REFERENCE['profiles'],body_center=h.BODY_CENTER.tolist()))
        js=r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');const payload=fs.readFileSync(0,'utf8');
const html=fs.readFileSync(process.argv[1],'utf8'),script=html.match(/<script>\n([\s\S]*?)<\/script>/)[1];
const ctx=new Proxy({},{get:()=>()=>{}}),nodes={};
function node(id){return nodes[id]??={textContent:id==='data'?payload:'',value:id==='diameter'?'0':id==='view'?'contacts':'both',clientWidth:600,clientHeight:400,getContext:()=>ctx,getBoundingClientRect:()=>({left:0,top:0})};}
const context={document:{getElementById:node,querySelectorAll:()=>[]},devicePixelRatio:1,ResizeObserver:class{observe(){}},console};vm.createContext(context);vm.runInContext(script,context);
for(const mode of ['contacts','audit','joint','mu']){node('view').value=mode;vm.runInContext('render()',context);}
node('view').value='audit';vm.runInContext("selected={type:'probe',r:report.maps[0].probes[0],D:100};render()",context);
assert(node('info').textContent.includes('NOT a valid grasp'));assert(node('info').textContent.includes('camera'));assert(node('band').disabled);
"""
        result=subprocess.run(['node','-e',js,str(path)],input=json.dumps(doc),capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
