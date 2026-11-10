"""Embed C-triangle selection and exact RL anchors into the standalone mapper."""
import argparse
import hashlib
import json
from pathlib import Path
import re

BEGIN='<!-- BEGIN HUG TRIANGLE -->'
END='<!-- END HUG TRIANGLE -->'


def study_result(root):
    from workbench.research.experiments.hug_c.triangle import select
    root=Path(root);plan=json.loads((root/'plan.json').read_text());chosen=json.loads((root/'selection.json').read_text())
    # Use the completed screen recorded by the study rather than guess a later attempt.
    manifest=json.loads((root/'manifest.json').read_text());entry=manifest['stages']['catalog-screen']
    if entry['status']!='complete':raise ValueError('Study catalog screening is incomplete')
    relative=Path(entry['output']).parts;index=relative.index('stages');catalog_path=root.joinpath(*relative[index:])/'report.json'
    catalog=json.loads(catalog_path.read_text())
    rows=[r for r in catalog['results'] if r['static_ready'] and
          not any(sum((a-b)**2 for a,b in zip(r['q_deg'],q))<(plan['step_deg']/2)**2 for q in manifest['excluded'])]
    ids,selection=select(rows,plan['step_deg'],seed=plan['seed']);anchors=[rows[i] for i in ids]
    actual=[c for c in chosen['cases'] if c['split']=='train'][:3]
    if {tuple(c['q_deg']) for c in actual}!={tuple(c['q_deg']) for c in anchors}:raise ValueError('Recomputed triangle differs from actual RL study')
    # Preserve A/B/C order used by the running study.
    return dict(diameter_mm=plan['diameter_mm'],mu_max=plan['mu_max'],step_deg=plan['step_deg'],
        study=root.name,kernel_sha256=catalog['source']['kernel_sha256'],result=dict(anchors=actual,rows=rows,selection=selection))


def build(source,output,study=None):
    from workbench.research.experiments.hug_c.source import read_kernel
    source=Path(source);text=source.read_text();kernel,_=read_kernel(source,require_preparation=True)
    text=re.sub(re.escape(BEGIN)+r'.*?'+re.escape(END)+r'\n?','',text,flags=re.S).rstrip()+'\n'
    support=Path(__file__).resolve().parents[1]/'support'
    selector=(support/'hugmap_triangle.js').read_text();ui=(support/'hugmap_triangle_ui.js').read_text()
    selected=study_result(study) if study else None
    if selected and selected['kernel_sha256']!=hashlib.sha256(kernel.encode()).hexdigest():raise ValueError('Study and mapper geometry differ')
    config=dict(kernel=kernel,study=selected)
    data=json.dumps(config,separators=(',',':'),ensure_ascii=True).replace('</','<\\/')
    script=ui.replace('__TRIANGLE_CONFIG__',data)
    controls='''<div id="triangleControls">
<button id="triangleSelect">Select C triangle</button> <button id="triangleStudy">Current RL anchors</button>
<label>Triangle grid (°) <input id="triangleStep" type="number" min="0.5" max="10" step="0.5" value="1" style="width:55px"></label>
<label><input id="triangleShow" type="checkbox" checked>Show triangle</label>
<pre id="triangleStatus" style="white-space:pre-wrap">Select a C triangle to maximize covered q3–q4 solution area. Tie-breaks: preparation clearance, worst μ, triangle conditioning.</pre>
</div>'''
    # Keep the original geometry script byte-for-byte: existing generated pages
    # and scientific kernel extraction continue to use the same first script.
    marker='<canvas id="plot"'
    if marker not in text:raise ValueError('Unsupported mapper canvas')
    text=text.replace(marker,BEGIN+controls+END+'\n'+marker,1)
    text+='\n'+BEGIN+'\n<script id="hugTriangleSource" type="text/plain">'+selector+'</script>\n<script>'+script+'</script>\n'+END+'\n'
    Path(output).write_text(text)
    after,_=read_kernel(output,require_preparation=True)
    if after!=kernel:raise AssertionError('Geometry kernel changed')
    return dict(output=str(Path(output).resolve()),bytes=len(text.encode()),kernel_sha256=hashlib.sha256(kernel.encode()).hexdigest(),study=config['study']['study'] if config['study'] else None)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mapper',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--study',type=Path);a=p.parse_args();print(json.dumps(build(a.mapper,a.output,a.study),indent=2))

if __name__=='__main__':main()
