"""Run the real mapper scan and pixel repair with a headless Canvas stub.

Records before/after pixel arrays and verifies every recovered pose. Timings
exclude actual browser rasterization; the JavaScript control flow is unchanged.
"""
import argparse
import base64
import json
from pathlib import Path
import re
import subprocess

STUB = r'''
const noop=()=>{},context=new Proxy({},{get:()=>noop}),elements=new Map();
global.document={getElementById:id=>{if(!elements.has(id))elements.set(id,{value:({band:'90',q3:'100',q4:'-150',mu:'.5',speed:'1000000'})[id]||'0',width:1100,height:780,getContext:()=>context});return elements.get(id)},createElement:()=>({width:1100,height:780,getContext:()=>context})};
global.requestAnimationFrame=()=>1;global.cancelAnimationFrame=noop;
'''

PROBE = r'''
$('diameter').value=ds.indexOf(TEST_DIAMETER);if(+$('diameter').value<0)throw Error('Diameter absent');$('band').value=TEST_ROLL;
let snapshot=null,scanMs=0,repairStart=0,targets=0;
const originalPrepare=prepareGapRepair;
prepareGapRepair=function(){scanMs=performance.now()-begin;snapshot=new Float64Array(minMu);repairStart=performance.now();originalPrepare();targets=gapQueue.length;};
const begin=performance.now();startConstruction();while(running)animationTick(performance.now());
const repairMs=performance.now()-repairStart;
let added=0,lowered=0,lost=0,invalid=0,maxResidual=0;
for(let p=0;p<minMu.length;p++){if(minMu[p]>snapshot[p])lost++;if(Number.isFinite(minMu[p])&&!Number.isFinite(snapshot[p]))added++;else if(minMu[p]<snapshot[p])lowered++;}
for(const [pixel,seed] of poseSeeds)if(seed.verified){
 const x=seed.x,y=seed.y,hit=inverseContactPose(x,y,TEST_DIAMETER,TEST_ROLL,seed);
 if(!hit||Math.round(X(x))!==pixel%canvas.width||Math.round(Y(y))!==Math.floor(pixel/canvas.width)){invalid++;continue;}maxResidual=Math.max(maxResidual,Math.hypot(hit.c.x-x,hit.c.y-y));
 const p=collisionGeometry(hit.q3,hit.q4);if(selfCollision(p)||objectCollision(p,hit.c,TEST_DIAMETER,TEST_ROLL))invalid++;
}
const thresholdCounts=[.01,.05,.1,.5,1,2].map(mu=>({mu,before:snapshot.filter(v=>v<=mu).length,after:minMu.filter(v=>v<=mu).length}));
const z=require('zlib'),encode=a=>z.deflateSync(Buffer.from(a.buffer)).toString('base64');
console.log(JSON.stringify({roll:TEST_ROLL,diameter:TEST_DIAMETER,scanMs,repairMs,targets,repaired,added,lowered,lost,invalid,maxResidual,thresholdCounts,beforeZ:encode(snapshot),afterZ:encode(minMu)}));
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('html', type=Path)
    parser.add_argument('--roll', type=int, choices=[-90, 90], default=-90)
    parser.add_argument('--diameter', type=float, default=150)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    script = re.search(r'<script>(.*?)</script>', args.html.read_text(), re.S).group(1)
    report = json.loads(subprocess.check_output(
        ['node'], input=f'const TEST_DIAMETER={args.diameter},TEST_ROLL={args.roll};\n' + STUB + script + PROBE,
        text=True))
    args.output.mkdir(parents=True, exist_ok=True)
    for phase in ('before', 'after'):
        (args.output / f'{phase}.f64.zlib').write_bytes(base64.b64decode(report.pop(f'{phase}Z')))
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)
    if report['invalid'] or report['lost'] or report['maxResidual'] > 1e-4:
        raise SystemExit('Gap verification failed')


if __name__ == '__main__':
    main()
