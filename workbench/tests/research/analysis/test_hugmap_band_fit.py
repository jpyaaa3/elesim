"""Numerical and viewer-state checks for the empirical band functions."""
import json
from pathlib import Path
import re
import shutil
import subprocess
import unittest

import numpy as np
from workbench.research.analysis.fit_hugmap_bands import predict, metrics

ROOT=Path(__file__).resolve().parents[4]
HTML=ROOT/'workbench/research/support/hugmap_reader.html'
REPORT=ROOT/'workbench/evidence/curated/hugmap_gaga_band_fit.json'

class BandFitTests(unittest.TestCase):
    def test_embedded_coefficients_and_holdout_separation(self):
        report=json.loads(REPORT.read_text())
        embedded=json.loads(re.search(r'<script id="bandFitModel" type="application/json">(.*?)</script>',HTML.read_text(),re.S)[1])
        self.assertEqual(embedded,report)
        train=set(report['train_diameters']);val=set(report['validation_diameters']);test=set(report['test_diameters'])
        self.assertFalse(train&val or train&test or val&test)
        self.assertEqual(len(train|val|test),21)
        self.assertEqual(len(report['model']['weights']),len(report['model']['centers'])+1)

    @unittest.skipUnless(shutil.which('node'),'Node unavailable')
    def test_browser_functions_match_python_and_modes_render(self):
        report=json.loads(REPORT.read_text());model=report['model']
        rng=np.random.default_rng(719)
        x=np.column_stack((rng.uniform(-250,-30,80),rng.uniform(160,400,80),rng.uniform(100,200,80)))
        expected=predict(x,model)-model['threshold']
        js=r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[1],'utf8'),input=JSON.parse(fs.readFileSync(0,'utf8'));
const embedded=html.match(/<script id="bandFitModel" type="application\/json">([\s\S]*?)<\/script>/)[1];
const script=html.match(/<script>\n([\s\S]*?)<\/script>/)[1];
const ctx=new Proxy({measureText:t=>({width:t.length*6})},{get:(o,k)=>o[k]||(()=>{})});
const nodes={};function node(id){return nodes[id]??=( {value:id==='diameter'?'0':id==='field'?'mu_min':id==='fitBand'?'both':id==='fitView'?'data':'2',textContent:id==='bandFitModel'?embedded:'',checked:false,disabled:false,clientWidth:900,clientHeight:700,getContext:()=>ctx,addEventListener(){},getBoundingClientRect:()=>({left:0,top:0})} );}
const context={document:{getElementById:node},devicePixelRatio:1,requestAnimationFrame(){},ResizeObserver:class{observe(){}},setTimeout,clearTimeout,console};
vm.createContext(context);vm.runInContext(script,context);context.input=input;
const actual=vm.runInContext('input.map(p=>[band1(...p),band2(...p)])',context);
assert.equal(vm.runInContext('band1(1,200,150)',context),null);
assert.equal(vm.runInContext('band2(-100,200,201)',context),null);
vm.runInContext(`maps=[{diameter_mm:150,results:[{x_mm:-100,y_mm:250,theta_deg:0,status:'found',seg1_deg:20,seg2_deg:170,mu_min:.2},{x_mm:-120,y_mm:350,theta_deg:0,status:'not_found',seg1_deg:null,seg2_deg:null,mu_min:null}]}];metadata={object:{shape:'circle'},config:{dtheta_deg:90},constraints:{interest_region:'left_forward_quadrant'}};ranges={mu_min:[0,.5],seg1_deg:[-180,180],seg2_deg:[-180,180]};fitCompatible=true;`,context);
for(const mode of ['data','fit','compare'])for(const band of ['both','0','1']){node('fitView').value=mode;node('fitBand').value=band;vm.runInContext('selectMap();draw()',context);}
assert(node('legend').textContent.includes('Gray: match'));
process.stdout.write(JSON.stringify(actual));
"""
        result=subprocess.run(['node','-e',js,str(HTML)],input=json.dumps(x.tolist()),text=True,capture_output=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        np.testing.assert_allclose(json.loads(result.stdout),expected,atol=1e-10,rtol=1e-10)

    def test_metrics_count_false_positives(self):
        y=np.array([[True,False],[False,True]])
        score=metrics(y,np.ones((2,2),bool))
        self.assertEqual([s['iou'] for s in score],[.5,.5])

if __name__=='__main__':unittest.main()
