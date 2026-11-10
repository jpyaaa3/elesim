"""The offline viewer must keep bounded-search and unverified-hold semantics."""
import copy
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from workbench.research.analysis import hug_diagnostic_report as report


def fixture():
    return dict(schema_version=1,complete=True,settings=dict(length_m=.25,
        body_length_m=.0875,body_width_m=.07),reference=dict(sha256='a'*64),
        gripper=dict(fixed=True,present=True,ratio=.155),
        limitations=['No hardware load certificate.'],rows=[
            dict(case='example</script>',terminal_ratio=.155,section_length_m=.25,
                target_vertices=[[-.1,-.1],[.1,-.1],[0,.1]],legacy=dict(solutions=[]),
                fixed=dict(candidates=[],candidate_count=0,legacy_gate_pass_count=0,
                    forward_rejected_count=0,endpoint_non_tangent_count=0,solves=512,
                    elapsed_s=20.,status='not_found_within_budget_budget_exhausted'))])


class ViewerTests(unittest.TestCase):
    def test_rendered_section_uses_physical_length_and_turn(self):
        straight = report.arc([0.,0.],math.pi/2,0.,.25)
        self.assertAlmostEqual(straight[-1][0],0.)
        self.assertAlmostEqual(straight[-1][1],.25)
        quarter = report.arc([0.,0.],math.pi/2,math.pi/2,.25)
        self.assertAlmostEqual(quarter[-1][0],-.5/math.pi)
        self.assertAlmostEqual(quarter[-1][1],.5/math.pi)

    def test_partial_and_empty_reports_cannot_look_complete(self):
        for data in (dict(schema_version=1,complete=False,rows=[{}]),
                     dict(schema_version=1,complete=True,rows=[])):
            with self.subTest(data=data),self.assertRaises(ValueError):
                report.build_report(data,'a'*64)

    def test_length_sweep_cannot_be_labeled_as_a_fixed_gripper_study(self):
        data = fixture();another = copy.deepcopy(data['rows'][0]);another['terminal_ratio'] = .08
        data['rows'].append(another)
        with self.assertRaisesRegex(ValueError,'gripper must remain fixed'):
            report.build_report(data,'a'*64)

    def test_no_candidate_keeps_unverified_fields_and_preserves_input(self):
        data = fixture();before = copy.deepcopy(data)
        page,svg,summary = report.build_report(data,'b'*64)
        self.assertEqual(data,before)
        row = summary['summaries'][0]
        self.assertIsNone(row['best_headroom_deg'])
        for field in ('hold_verified','motor_load_margin','compression_margin'):
            self.assertIsNone(row[field])
        self.assertIn('Not found in bounded search',svg)
        self.assertIn('불가능하다는 증명이 아니다',page)
        encoded = re.search(r'type="application/json">(.*?)</script>',page,re.S).group(1)
        self.assertNotIn('</script>',encoded)
        self.assertEqual(json.loads(encoded)['rows'][0]['case'],'example</script>')

    @unittest.skipUnless(shutil.which('node'),'Node is optional; numerical report tests still run')
    def test_viewer_script_loads_and_handles_all_selections_without_network(self):
        page,_,_ = report.build_report(fixture(),'b'*64)
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'viewer.html';path.write_text(page)
            checked = subprocess.run(['node','-e',NODE_PROBE,str(path)],capture_output=True,text=True)
        self.assertEqual(checked.returncode,0,checked.stderr)
        self.assertIn('1 rows rendered',checked.stdout)


NODE_PROBE = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[1],'utf8');
const data=html.match(/type="application\/json">([\s\S]*?)<\/script>/)[1];
const script=html.match(/<\/script><script>([\s\S]*?)<\/script>/)[1];
const elements={};
function element(selector){
 if(!elements[selector]){
  const e={value:'',textContent:selector==='#data'?data:''};
  Object.defineProperty(e,'innerHTML',{get(){return this._html||''},set(v){
   this._html=v;
   if(selector==='#case'||selector==='#terminal'||selector==='#candidate'){
    const m=v.match(/<option(?: value="([^"]*)")?>([^<]*)/);
    this.value=m?(m[1]??m[2]).replace(/&(amp|lt|gt|quot|#39);/g,
      (_,e)=>({amp:'&',lt:'<',gt:'>',quot:'"','#39':"'"}[e])):'';
   }
  }});elements[selector]=e;
 }
 return elements[selector];
}
const context={document:{querySelector:element},console};
vm.createContext(context);vm.runInContext(script,context);
const measured=JSON.parse(data);let count=0;
for(const name of [...new Set(measured.rows.map(r=>r.case))]){
 element('#case').value=name;context.populate();
 const rows=measured.rows.filter(r=>r.case===name);
 for(let i=0;i<rows.length;i++){
  element('#terminal').value=String(i);context.populateCandidates();
  if(!rows[i].fixed.candidates.length)assert(element('#metrics').innerHTML.includes('불가능 판정이 아니다'));
  for(let j=0;j<rows[i].fixed.candidates.length;j++){
   element('#candidate').value=String(j);context.render();
   assert(element('#plot').innerHTML.includes('polyline'));
   assert(element('#metrics').innerHTML.includes('미검증'));
  }count++;
 }
}
console.log(count+' rows rendered');
"""


if __name__ == '__main__':
    unittest.main()
