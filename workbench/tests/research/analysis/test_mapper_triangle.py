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


def test_xy_projection_samples_curved_q_edges_and_the_triangle_interior():
    result=node('''
const mesh=T.projectTriangle(INPUT,q=>[{xy_mm:[q[0],q[1]+q[0]**2],feature:'side',root:1}]);
console.log(JSON.stringify({triangles:Array.from(mesh.triangles_mm),boundary:Array.from(mesh.boundary_mm),count:mesh.valid_triangles}));
''',[[-2,-2],[0,-2],[-2,0]])
    assert result['count']==16
    boundary=np.array(result['boundary']).reshape(-1,2)
    assert np.min(np.linalg.norm(boundary-[-1,-1],axis=1))<1e-10
    # The curved image of an edge's midpoint differs from its XY chord.
    midpoint=boundary[np.argmin(np.linalg.norm(boundary-[-1,-1],axis=1))]
    assert np.linalg.norm(midpoint-(np.array([-2,2])+[0,-2])/2)>.9
    for x,y in boundary:
        q4=y-x*x
        assert min(abs(x+2),abs(q4+2),abs(x+q4+2))<1e-10
    for x,y in np.array(result['triangles']).reshape(-1,2):
        q4=y-x*x
        assert x>=-2-1e-10 and q4>=-2-1e-10 and x+q4<=-2+1e-10


def test_xy_projection_never_joins_distinct_contact_roots_or_invalid_q_gaps():
    result=node('''
const mesh=T.projectTriangle(INPUT,q=>{
  if(q[0]>-1.3&&q[0]<-.7)return [];
  return [0,100].map((offset,i)=>({xy_mm:[q[0]+offset,q[1]+offset],feature:'side',root:i?1:-1}));
});
console.log(JSON.stringify({triangles:Array.from(mesh.triangles_mm),boundary:Array.from(mesh.boundary_mm)}));
''',[[-2,-2],[0,-2],[-2,0]])
    triangles=np.array(result['triangles']).reshape(-1,3,2)
    assert len(triangles)>0
    for tri in triangles:
        assert np.ptp(tri[:,0])<=.5+1e-10 and np.ptp(tri[:,1])<=.5+1e-10
        xs=tri[:,0]-(100 if tri[0,0]>50 else 0)
        assert xs.max()<=-1.3 or xs.min()>=-.7


def test_xy_projection_reuses_friction_collision_and_preparation_filters():
    result=node('''
const kernel=`function model(a,b){return {candidates:[
  {x:a-10,y:b+10,mu:.1,kind:'side',branch:1},
  {x:a-100,y:b+10,mu:.9,kind:'side',branch:-1},
  {x:a-200,y:b+10,mu:.1,kind:'tip',branch:1},
  {x:a-300,y:b+10,mu:.1,kind:'heel',branch:1}]};}
function collisionGeometry(){return [];}function selfCollision(){return false;}
function objectCollision(p,c){return c.kind==='tip';}
function preparationReport(c){return {blocked:c.kind==='heel',minimum_gap_mm:2,required_clearance_mm:1};}`;
const mesh=T.project(kernel,100,.4,INPUT.map(q=>({q_deg:q})));
console.log(JSON.stringify({triangles:Array.from(mesh.triangles_mm),count:mesh.valid_triangles}));
''',[[-2,-2],[-.5,-2],[-2,-.5]])
    assert result['count']>0
    xy=np.array(result['triangles']).reshape(-1,2)
    assert np.all(xy[:,0]>=-12) and np.all(xy[:,0]<=-10.5)


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
    stub=STUB.replace('get:()=>noop','get:(target,key)=>target[key]||noop')+"\nglobal.workers=[];global.Worker=class{constructor(){workers.push(this)}terminate(){this.stopped=true}postMessage(data){this.input=data}};document.getElementById('hugTriangleSource').textContent="+json.dumps(selector)+';\n'
    probe=r'''
const labels=[];context.fillText=(text)=>labels.push(String(text));draw();
const initial=$('triangleStatus').textContent;
if(!initial.includes('physical admission screened')||!initial.includes('q3 -91.50'))throw Error(initial);
if(labels.some(s=>/^[ABC]$/.test(s))||/\n[ABC]:/.test(initial))throw Error('Vertex labels must be hidden');
canvas.getBoundingClientRect=()=>({left:0,top:0,width:1100,height:780});
canvas.onclick({clientX:QX(-64.5),clientY:QY(-166.5)});
if(Number($('q3').value)!==-64.5||Number($('q4').value)!==-166.5)throw Error('Anchor click failed');
$('plane').value='xy';$('plane').onchange();
if(!$('triangleStatus').textContent.includes('Mapping the q-triangle'))throw Error('XY projection worker missing');
const projection=workers.at(-1);if(!projection.input.anchors||projection.input.anchors.length!==3)throw Error('Missing q vertices');
projection.onmessage({data:{projection:{triangles_mm:new Float64Array([-100,160,-120,200,-110,180]),boundary_mm:new Float64Array([-100,160,-110,180]),valid_triangles:1}}});
if(!$('triangleStatus').textContent.includes('XY maps the q-triangle'))throw Error('Mapped XY overlay missing');
$('triangleSelect').onclick();const old=workers.at(-1);
$('mu').value='.2';$('mu').oninput();old.onmessage({data:{result:{anchors:[],rows:[],selection:{}}}});
if(!old.stopped||!$('triangleStatus').textContent.includes('cancelled')||$('triangleSelect').disabled)throw Error('Stale worker result restored');
$('triangleSelect').onclick();const cancelled=workers.at(-1);
if($('triangleSelect').disabled||$('stop').disabled)throw Error('Selection must be cancellable');
$('triangleSelect').onclick();
if(!cancelled.stopped||$('triangleSelect').textContent!=='Select C triangle')throw Error('Cancel button stuck');
$('triangleSelect').onclick();const stopped=workers.at(-1);$('stop').onclick();
if(!stopped.stopped||$('triangleSelect').disabled)throw Error('Stop did not cancel selection');
$('triangleSelect').onclick();const cleared=workers.at(-1);$('clear').onclick();
if(!cleared.stopped||$('triangleSelect').disabled)throw Error('Clear did not cancel selection');
const WorkingWorker=Worker;global.Worker=class{constructor(){throw Error('worker blocked')}};
$('triangleSelect').onclick();
if($('triangleSelect').disabled||$('triangleSelect').textContent!=='Select C triangle'||!$('triangleStatus').textContent.includes('worker blocked'))throw Error('Worker startup failure locked controls');
global.Worker=WorkingWorker;
$('triangleSelect').onclick();workers.at(-1).onerror({message:'test worker failure'});
if($('triangleSelect').disabled||$('triangleSelect').textContent!=='Select C triangle')throw Error('Worker runtime failure locked controls');
$('triangleStudy').onclick();
if(Number($('mu').value)!==.4||$('plane').value!=='q'||!$('triangleStatus').textContent.includes('physical admission screened'))throw Error('RL restore failed');
const originalModel=model;startConstruction();model=()=>{throw Error('test geometry failure')};animationTick(0);model=originalModel;
if(running||$('construct').disabled||!$('stop').disabled||!$('animationStatus').textContent.includes('test geometry failure'))throw Error('Construction failure locked controls');
console.log(JSON.stringify({ok:true,initial}));
'''
    result=json.loads(subprocess.check_output(['node'],input=stub+'\n'.join(scripts)+probe,text=True,timeout=60))
    assert result['ok']


@pytest.mark.skipif(not MAPPER.exists(),reason='Standalone mapper unavailable')
def test_presentation_refresh_preserves_study_and_increases_every_speed(tmp_path):
    from workbench.research.experiments.hug_c.source import read_kernel
    source=MAPPER.read_text()
    start=source.index('const config=',source.index('<script id="hugTriangleSource"'))+len('const config=')
    config,_=json.JSONDecoder().raw_decode(source[start:])
    page=tmp_path/'mapper.html';build(MAPPER,page)
    updated=page.read_text();start=updated.index('const config=',updated.index('<script id="hugTriangleSource"'))+len('const config=')
    after,_=json.JSONDecoder().raw_decode(updated[start:])
    assert after['study']==config['study']
    assert read_kernel(page)[0]==read_kernel(MAPPER)[0]
    before=page.read_bytes();build(page,page);assert page.read_bytes()==before
    script=re.search(r'<script>(.*?)</script>',updated,re.S).group(1)
    probe=r'''
let clock=0;global.performance={now:()=>++clock};model=()=>({candidates:[]});draw=()=>{};
const counts=[];
for(const speed of ['60000','270000','1000000']){
  $('speed').value=speed;startConstruction();animationTick(0);counts.push(index);$('stop').onclick();
  if(running||!$('stop').disabled||$('construct').disabled)throw Error('Stop must release construction controls');
}
console.log(JSON.stringify(counts));
'''
    fast=json.loads(subprocess.check_output(['node'],input=STUB+script+probe,text=True,timeout=30))
    old=re.sub(r'60000:\d+,270000:\d+,1000000:\d+','60000:4,270000:9,1000000:14',script)
    slow=json.loads(subprocess.check_output(['node'],input=STUB+old+probe,text=True,timeout=30))
    assert all(a>b for a,b in zip(fast,slow))
    assert fast==sorted(fast)
