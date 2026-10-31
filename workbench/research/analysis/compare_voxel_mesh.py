"""Remesh one saved observed/predicted cloud at fixed prediction support.

No camera capture, completion inference or Sim process runs. Truth is used only
after meshing for evaluation and the separate original-shape display.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path
import platform
import statistics
import subprocess
import time
import zipfile

import numpy as np

from elesim_pilot.reconstruction.artifacts import export_result
from elesim_pilot.reconstruction.contracts import CompletedObject, ObjectCloud, Task
from elesim_pilot.reconstruction.pipeline import Result
from elesim_pilot.reconstruction.providers import VoxelSurface
from elesim_pilot.reconstruction.contracts import SceneObject
from elesim_protocol.scene_artifact import SceneArtifact
from .evaluate_scan_cycle import surface_errors


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def component_count(cells):
    pending = set(map(tuple, cells))
    components = 0
    while pending:
        components += 1
        queue = [pending.pop()]
        while queue:
            cell = queue.pop()
            for axis in range(3):
                for sign in (-1, 1):
                    other = list(cell)
                    other[axis] += sign
                    other = tuple(other)
                    if other in pending:
                        pending.remove(other)
                        queue.append(other)
    return components


def write_views(output, displays):
    """Write a static geometric comparison and a dependency-free orbit viewer."""
    all_vertices = np.concatenate([d['vertices'] for d in displays])
    center = (all_vertices.min(axis=0)+all_vertices.max(axis=0))/2
    extent = float(np.ptp(all_vertices, axis=0).max())*1.3
    azimuth, elevation = np.deg2rad([-65., 25.])
    eye = np.array([np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
    right = np.cross([0.,0.,1.], eye); right /= np.linalg.norm(right)
    up = np.cross(eye, right)
    light = np.array([-.5,-.4,.8]); light /= np.linalg.norm(light)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="1120">',
        '<rect width="1400" height="1120" fill="#101827"/>',
        '<g fill="white" font-family="sans-serif"><text x="30" y="38" font-size="25">Jar: same point cloud / 20, 10, 5 mm mesh</text>',
        '<text x="30" y="66" font-size="17">Prediction support fixed at 10 mm, 8 neighbours. Observations and raw predictions unchanged.</text></g>']
    panels = []
    for index, item in enumerate(displays):
        x, y = 20+(index%2)*690, 90+(index//2)*490
        vertices, faces = item['vertices'], item['faces']
        vertices = vertices-center
        xy = np.column_stack((vertices@right, vertices@up))
        depth = vertices@eye
        color = item['color']
        svg.append(f'<rect x="{x}" y="{y}" width="670" height="475" rx="8" fill="#192536"/>')
        for face in faces[np.argsort(depth[faces].mean(axis=1))]:
            points = vertices[face]
            normal = np.cross(points[1]-points[0], points[2]-points[0])
            norm = np.linalg.norm(normal)
            brightness = .32+.68*max(0., float(normal@light/norm)) if norm else .32
            fill = '#'+''.join(f'{int(channel*brightness):02x}' for channel in color)
            coords = ' '.join(f'{x+335+px*380/extent:.2f},{y+260-py*380/extent:.2f}' for px,py in xy[face])
            svg.append(f'<polygon points="{coords}" fill="{fill}" stroke="{fill}" stroke-width="0.35"/>')
        svg.append(f'<g fill="white" font-family="sans-serif"><text x="{x+18}" y="{y+30}" font-size="22">{html.escape(item["label"])}</text><text x="{x+18}" y="{y+455}" font-size="16">{html.escape(item["note"])}</text></g>')
        panels.append(dict(label=item['label'], note=item['note'], file=item['file'], color=[c/255 for c in color],
            vertices=np.round(item['vertices']-center, 8).tolist(), faces=faces.tolist()))
    svg.append('<text x="30" y="1100" fill="white" font-family="sans-serif" font-size="16">Offline geometric render, not new camera footage. Vertex error does not establish complete surface coverage.</text></svg>')
    (output/'comparison.svg').write_text('\n'.join(svg)+'\n')
    document = '''<!doctype html><meta charset="utf-8"><title>Jar voxel comparison</title>
<style>body{background:#101827;color:#e5edf5;font:16px system-ui;margin:24px}h1{font-size:25px}.grid{display:grid;grid-template-columns:repeat(2,minmax(280px,1fr));gap:18px}section{background:#192536;padding:14px;border-radius:10px}h2{margin:0;font-size:20px}canvas{width:100%;height:360px;touch-action:none;cursor:grab}a{color:#9bd2ff}button{padding:8px 14px}small{color:#b5c3d4}@media(max-width:650px){.grid{grid-template-columns:1fr}}</style>
<h1>Jar: same point cloud, different mesh resolution</h1>
<p>Drag any view to rotate all four. Scroll to zoom. <button id="reset">Reset view</button></p>
<p>Support radius: 10 mm; minimum: 8 neighbours. Same observations and raw AdaPoinTr predictions.</p><div class="grid" id="grid"></div>
<p><small>Offline geometric views. Meshing does not reconstruct unsupported hidden surfaces or establish contact safety.</small></p>
<script>
const meshes=__MESHES__, extent=__EXTENT__,grid=document.getElementById('grid'),reset=document.getElementById('reset');let az=-65*Math.PI/180,el=25*Math.PI/180,zoom=1;
const views=[];
function shader(gl,type,src){const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
for(const mesh of meshes){const section=document.createElement('section'),h=document.createElement('h2'),note=document.createElement('p'),canvas=document.createElement('canvas'),link=document.createElement('a');h.textContent=mesh.label;note.textContent=mesh.note;link.href=mesh.file;link.textContent='Download OBJ';section.append(h,note,canvas,link);grid.append(section);
const gl=canvas.getContext('webgl',{antialias:true});if(!gl){note.textContent+=' WebGL unavailable; use the PNG comparison.';continue;}
const program=gl.createProgram();gl.attachShader(program,shader(gl,gl.VERTEX_SHADER,'attribute vec3 p,n;uniform vec3 right,up,eye;uniform vec2 scale;uniform float extent;varying vec3 normal;void main(){gl_Position=vec4(dot(p,right)*scale.x,dot(p,up)*scale.y,-dot(p,eye)/extent,1.);normal=n;}'));gl.attachShader(program,shader(gl,gl.FRAGMENT_SHADER,'precision mediump float;varying vec3 normal;uniform vec3 color;void main(){float b=.32+.68*max(0.,dot(normalize(normal),normalize(vec3(-.5,-.4,.8))));gl_FragColor=vec4(color*b,1.);}'));gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));gl.useProgram(program);
const positions=[],normals=[];for(const f of mesh.faces){const [a,b,c]=f.map(i=>mesh.vertices[i]),u=b.map((x,i)=>x-a[i]),v=c.map((x,i)=>x-a[i]);let n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];const norm=Math.hypot(...n)||1;n=n.map(x=>x/norm);for(const p of [a,b,c]){positions.push(...p);normals.push(...n);}}
for(const [name,data] of [['p',positions],['n',normals]]){const b=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,b);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(data),gl.STATIC_DRAW);const loc=gl.getAttribLocation(program,name);gl.enableVertexAttribArray(loc);gl.vertexAttribPointer(loc,3,gl.FLOAT,false,0,0);}gl.uniform3fv(gl.getUniformLocation(program,'color'),mesh.color);gl.uniform1f(gl.getUniformLocation(program,'extent'),extent);gl.enable(gl.DEPTH_TEST);views.push({gl,program,canvas,count:positions.length/3});
let last=null;canvas.onpointerdown=e=>{last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointerup=()=>last=null;canvas.onpointercancel=()=>last=null;canvas.onpointermove=e=>{if(!last)return;az+=(e.clientX-last[0])*.008;el=Math.max(-1.45,Math.min(1.45,el+(e.clientY-last[1])*.008));last=[e.clientX,e.clientY];render()};canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.5,Math.min(3,zoom*Math.exp(-e.deltaY*.001)));render()},{passive:false});}
function render(){const eye=[Math.cos(el)*Math.cos(az),Math.cos(el)*Math.sin(az),Math.sin(el)],right=[-Math.sin(az),Math.cos(az),0],up=[-Math.sin(el)*Math.cos(az),-Math.sin(el)*Math.sin(az),Math.cos(el)];for(const {gl,program,canvas,count} of views){const r=canvas.getBoundingClientRect(),d=Math.min(devicePixelRatio||1,2);canvas.width=r.width*d;canvas.height=r.height*d;gl.viewport(0,0,canvas.width,canvas.height);gl.clearColor(.098,.145,.212,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);for(const [name,value]of [['eye',eye],['right',right],['up',up]])gl.uniform3fv(gl.getUniformLocation(program,name),value);gl.uniform2fv(gl.getUniformLocation(program,'scale'),[1.8*zoom/extent*r.height/r.width,1.8*zoom/extent]);gl.drawArrays(gl.TRIANGLES,0,count);}}
reset.onclick=()=>{az=-65*Math.PI/180;el=25*Math.PI/180;zoom=1;render()};window.onresize=render;render();
</script>'''
    (output/'comparison.html').write_text(document.replace('__MESHES__',json.dumps(panels,separators=(',',':'))).replace('__EXTENT__',str(extent)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--truth',type=Path,required=True)
    parser.add_argument('--reference',type=Path,required=True,help='Original local-centred OBJ, display only')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output=args.output.absolute()
    if args.output.exists():raise FileExistsError('use a new comparison directory')
    with zipfile.ZipFile(args.input) as archive:
        if sum(i.file_size for i in archive.infolist())>16*1024*1024:raise ValueError('cloud exceeds comparison budget')
    with np.load(args.input,allow_pickle=False) as packet:
        shape=CompletedObject(ObjectCloud('target','jar','target','world',packet['observed']),packet['predicted'])
    geometry=json.loads(args.truth.read_text())['truth']
    options=dict(min_generated_neighbors=8,generated_neighbor_radius_m=.01,max_occupied_voxels=8000)
    args.output.mkdir(parents=True)
    timings={v:[] for v in (.02,.01,.005)}
    # Warm imports and each path once; five interleaved CPU timings exclude file I/O.
    for voxel in timings:
        VoxelSurface(options=dict(options,voxel_m=voxel),assets={}).mesh(shape,Task('warmup','target'))
    for _ in range(5):
        for voxel in timings:
            mesher=VoxelSurface(options=dict(options,voxel_m=voxel),assets={})
            started=time.perf_counter();mesh=mesher.mesh(shape,Task('timing','target'))
            timings[voxel].append((time.perf_counter()-started)*1000)
    rows=[];displays=[]
    vertices=np.array([list(map(float,line.split()[1:4])) for line in args.reference.read_text().splitlines() if line.startswith('v ')])+geometry['center_m']
    faces=np.array([[int(i)-1 for i in line.split()[1:4]] for line in args.reference.read_text().splitlines() if line.startswith('f ')])
    displays.append(dict(label='Original jar (120 x 220 mm)',note='Reference geometry; not used by mesher',vertices=vertices,faces=faces,color=(220,70,65),file='reference.obj'))
    # Write the centred reference for the viewer download without changing source bytes.
    (args.output/'reference.obj').write_bytes(args.reference.read_bytes())
    for voxel in timings:
        name=f'{round(voxel*1000)}mm';mesher=VoxelSurface(options=dict(options,voxel_m=voxel),assets={})
        mesh=mesher.mesh(shape,Task('jar-'+name,'target'));diagnostics=mesher.diagnostics()
        result=Result(Task('jar-'+name,'target'),(SceneObject(shape,mesh),),None,
                      dict(source_npz_sha256=sha(args.input),model_execution=dict(models=dict(mesher=diagnostics))))
        manifest=export_result(result,args.output/name)
        artifact=SceneArtifact.load(manifest);obj=artifact.objects[0].mesh_file
        copy=args.output/('voxel-'+name+'.obj');copy.write_bytes(obj.read_bytes())
        # Reconstruct occupied cell coordinates from the unchanged supported points.
        from scipy.spatial import cKDTree
        generated=shape.predicted[cKDTree(np.concatenate((shape.cloud.observed,shape.predicted))).query(shape.predicted,k=9)[0][:,-1]<=.01]
        cells=np.unique(np.floor(np.concatenate((shape.cloud.observed,generated))/voxel).astype(np.int64),axis=0)
        errors=surface_errors(mesh.vertices,geometry)
        row=dict(voxel_m=voxel,diagnostics=diagnostics,vertices=len(mesh.vertices),triangles=len(mesh.triangles),
            occupied_components_6=component_count(cells),obj_bytes=copy.stat().st_size,obj_file=copy.name,obj_sha256=sha(copy),
            scene_manifest=str(manifest.relative_to(args.output)),mesh_vertex_surface_error_p95_m=float(np.quantile(errors,.95)),
            mesh_vertex_surface_error_max_m=float(errors.max()),meshing_cpu_ms=timings[voxel],meshing_cpu_median_ms=statistics.median(timings[voxel]),
            fits_default_4000_voxel_budget=len(cells)<=4000)
        rows.append(row)
        displays.append(dict(label=name+' voxel',note=f'{len(mesh.vertices):,} vertices / {len(mesh.triangles):,} triangles / vertex p95 {row["mesh_vertex_surface_error_p95_m"]*1000:.1f} mm',vertices=mesh.vertices,faces=mesh.triangles,color=(215,220,225),file=copy.name))
    assert len({row['diagnostics']['meshed_generated_points'] for row in rows})==1
    report=dict(schema_version=1,source_npz=str(args.input),source_npz_sha256=sha(args.input),truth_sha256=sha(args.truth),reference_sha256=sha(args.reference),raw_output_root=str(args.output),
        reference_local_to_world_translation_m=geometry['center_m'],
        observed_points=len(shape.cloud.observed),raw_predicted_points=len(shape.predicted),support_policy=options,rows=rows,
        timing_scope='Host CPU remeshing only; one warmup then five interleaved runs, no file I/O, no GPU/Sim import timing',
        python=platform.python_version(),platform=platform.platform(),
        limitations=['Fixed input; no new scan or completion inference.','Vertex-to-surface distance does not establish complete surface coverage.','5 mm needs an explicit larger voxel budget; the runtime default remains 4000.'])
    (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    write_views(args.output,displays)
    subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-n','-i',str(args.output/'comparison.svg'),'-frames:v','1',str(args.output/'comparison.png')],check=True,timeout=120)
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
