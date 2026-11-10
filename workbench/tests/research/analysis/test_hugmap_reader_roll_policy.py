import json
from pathlib import Path
import shutil
import subprocess
import unittest
from workbench.research.support import hugmap as h

class ReaderRollPolicyTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'),'Node unavailable')
    def test_new_alternates_mirroring_and_legacy_loading(self):
        row=h.solve_placement(h.local_center(-110,340),.5,radius=.05)
        row.update(row.pop('best'));row.update(x_mm=-110,y_mm=340,theta_deg=0)
        document=dict(metadata=h.map_metadata(h.Config(10,10),1),results=[row],summary={})
        path=Path(h.__file__).with_name('hugmap_reader.html')
        js=r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');const html=fs.readFileSync(process.argv[1],'utf8'),doc=JSON.parse(fs.readFileSync(0,'utf8')),embedded=html.match(/<script id="bandFitModel" type="application\/json">([\s\S]*?)<\/script>/)[1],script=html.match(/<script>\n([\s\S]*?)<\/script>/)[1];
const ctx=new Proxy({measureText:t=>({width:t.length*6})},{get:(o,k)=>o[k]||(()=>{})}),nodes={};
function node(id){return nodes[id]??={textContent:id==='bandFitModel'?embedded:'',value:id==='diameter'?'0':id==='fitBand'?'both':id==='fitView'?'data':id==='field'?'mu_min':'2',checked:false,clientWidth:900,clientHeight:700,getContext:()=>ctx,addEventListener(){},getBoundingClientRect:()=>({left:0,top:0})};}
const c={document:{getElementById:node},devicePixelRatio:1,requestAnimationFrame(){},ResizeObserver:class{observe(){}},setTimeout,clearTimeout,console,doc};vm.createContext(c);vm.runInContext(script,c);
vm.runInContext('({maps,metadata,ranges}=parseDocument(doc))',c);
node('fitBand').value='0';assert.equal(vm.runInContext('solutionRows(maps[0])[0].roll_deg',c),-90);
node('fitBand').value='1';assert.equal(vm.runInContext('solutionRows(maps[0])[0].roll_deg',c),90);
assert.equal(vm.runInContext('solutionRows(maps[0])[0].seg2_deg<0',c),true);
vm.runInContext('selectMap();draw();canvas.onpointermove({clientX:hits[0].x,clientY:hits[0].y})',c);
assert(node('detail').textContent.includes('Saved roll: 90 deg'));assert(node('detail').textContent.includes('Displayed roll: -90 deg'));
assert(vm.runInContext('ranges.seg1_deg[1]',c)>100);
const bad=JSON.parse(JSON.stringify(doc));bad.results[0].roll_solutions[1].seg2_deg=20;c.bad=bad;assert.throws(()=>vm.runInContext('parseDocument(bad)',c));
const legacy={metadata:{schema:'hugmap',schema_version:1,object:{shape:'circle',diameter_mm:100},config:{dx_mm:10,dy_mm:10,dtheta_deg:90},constraints:{interest_region:'left_forward_quadrant'}},results:[{x_mm:-110,y_mm:340,theta_deg:0,status:'found',seg1_deg:20,seg2_deg:160,mu_min:.2}]};c.legacy=legacy;node('fitBand').value='both';vm.runInContext('({maps,metadata,ranges}=parseDocument(legacy));selectMap();draw()',c);assert(node('status').textContent.includes('Legacy fixed-roll'));
"""
        result=subprocess.run(['node','-e',js,str(path)],input=json.dumps(document),text=True,capture_output=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)

if __name__=='__main__':unittest.main()
