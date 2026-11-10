"""Raster/selection parity and real standalone UI event regression, without GPU."""
import json
from pathlib import Path
import re
import shutil
import subprocess
import numpy as np
import pytest
from workbench.research.experiments.hug_c.triangle import RasterArea,select
from workbench.research.analysis.mapper_triangle import build,study_result
from workbench.research.analysis.benchmark_mapper_gaps import STUB

ROOT=Path(__file__).resolve().parents[4]
JS=ROOT/'workbench/research/support/hugmap_triangle.js'
MAPPER=Path('/home/user/ws/hugmap/mapper.html')
STUDY=ROOT/'workbench/evidence/generated/hug-pipeline-gpu2-20261110-v5'
pytestmark=pytest.mark.skipif(not shutil.which('node'),reason='Node required')


def node(code,data):
    prefix=f"const T=require({json.dumps(str(JS))});const INPUT=JSON.parse(require('fs').readFileSync(0,'utf8'));\n"
    return json.loads(subprocess.check_output(['node','-e',prefix+code],input=json.dumps(data),text=True,timeout=90))


def test_scanline_raster_matches_python_at_edges_holes_and_fractional_cells():
    rng=np.random.default_rng(42);points=(rng.integers(0,60,(60,2))+.5)*3-180
    triangles=[points[rng.choice(len(points),3,replace=False)].tolist() for _ in range(30)]
    result=node('const f=T.raster(INPUT.points,3);console.log(JSON.stringify(INPUT.triangles.map(f)));',dict(points=points.tolist(),triangles=triangles))
    expected=[RasterArea(points,3).covered(np.array(t)) for t in triangles]
    np.testing.assert_allclose(result,expected,atol=1e-8)


def test_exhaustive_triangle_matches_python_with_clearance_and_mu_tie_breaks():
    rows=[dict(q_deg=[a,b],mu_min=.1+(i%3)*.1,preparation_passed=True,preparation_margin_mm=2+i%2)
          for i,(a,b) in enumerate([(-178.5,-178.5),(-175.5,-178.5),(-178.5,-175.5),(-175.5,-175.5),(-172.5,-175.5),(-178.5,-172.5)])]
    ids,report=select(rows,3);result=node('console.log(JSON.stringify(T.select(INPUT,3)));',rows)
    assert result['selection']['vertices_total_deg']==[rows[i]['q_deg'] for i in ids]
    assert result['selection']['covered_area_deg2']==report['covered_area_deg2']


@pytest.mark.skipif(not STUDY.exists(),reason='Actual study evidence unavailable')
def test_browser_reproduces_current_physics_screened_rl_triangle():
    study=study_result(STUDY);result=node('console.log(JSON.stringify(T.select(INPUT,3)));',study['result']['rows'])
    assert result['selection']['vertices_total_deg']==study['result']['selection']['vertices_total_deg']
    assert result['selection']['covered_area_deg2']==study['result']['selection']['covered_area_deg2']


@pytest.mark.skipif(not MAPPER.exists(),reason='Standalone mapper unavailable')
def test_browser_uses_same_function_and_cad_candidate_domain_as_python():
    from workbench.research.experiments.hug_c.source import read_kernel,evaluate
    kernel,_=read_kernel(MAPPER,require_preparation=True);expected=evaluate(kernel,100,step=3,mu_max=.4)
    result=node('console.log(JSON.stringify(T.scan(INPUT.kernel,100,3,.4)));',dict(kernel=kernel))
    assert len(result)==len(expected)>24
    for a,b in zip(result,expected):
        for key in ('q_deg','xy_mm','mu_min','feature','root','preparation_margin_mm'):assert a[key]==b[key]


@pytest.mark.skipif(not MAPPER.exists() or not STUDY.exists(),reason='Standalone mapper and actual study evidence required')
def test_overlay_switch_click_stale_worker_and_idempotent_build(tmp_path):
    from workbench.research.experiments.hug_c.source import read_kernel
    page=tmp_path/'mapper.html';build(MAPPER,page,STUDY);before=page.read_bytes();build(page,page,STUDY)
    assert page.read_bytes()==before
    assert read_kernel(page)[0]==read_kernel(MAPPER)[0]
    text=page.read_text();scripts=re.findall(r'<script>(.*?)</script>',text,re.S)
    selector=re.search(r'<script id="hugTriangleSource" type="text/plain">(.*?)</script>',text,re.S).group(1)
    stub=STUB+"\nglobal.workers=[];global.Worker=class{constructor(){workers.push(this)}terminate(){this.stopped=true}postMessage(data){this.input=data}};document.getElementById('hugTriangleSource').textContent="+json.dumps(selector)+';\n'
    probe=r'''
const initial=$('triangleStatus').textContent;
if(!initial.includes('physical admission screened')||!initial.includes('q3 -91.50'))throw Error(initial);
canvas.getBoundingClientRect=()=>({left:0,top:0,width:1100,height:780});
canvas.onclick({clientX:QX(-64.5),clientY:QY(-166.5)});
if(Number($('q3').value)!==-64.5||Number($('q4').value)!==-166.5)throw Error('Anchor click failed');
$('plane').value='xy';$('plane').onchange();
if(!$('triangleStatus').textContent.includes('XY dashed lines'))throw Error('XY overlay missing');
$('triangleSelect').onclick();const old=workers.at(-1);
$('mu').value='.2';$('mu').oninput();old.onmessage({data:{result:{anchors:[],rows:[],selection:{}}}});
if(!old.stopped||!$('triangleStatus').textContent.includes('cancelled')||$('triangleSelect').disabled)throw Error('Stale worker result restored');
$('triangleStudy').onclick();
if(Number($('mu').value)!==.4||$('plane').value!=='q'||!$('triangleStatus').textContent.includes('physical admission screened'))throw Error('RL restore failed');
console.log(JSON.stringify({ok:true,initial}));
'''
    result=json.loads(subprocess.check_output(['node'],input=stub+'\n'.join(scripts)+probe,text=True,timeout=60))
    assert result['ok']
