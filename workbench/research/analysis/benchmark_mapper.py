"""Compare standalone mapper kernels on the same deterministic angle samples.

This excludes browser rendering and frame scheduling. JSON pixel-map hashes
check that the optimization preserved the sampled display results.
"""
import argparse
import json
from pathlib import Path
import subprocess

PROBE = r'''
const minimum=new Float64Array(1100*780);minimum.fill(Infinity);
const collided=new Float64Array(minimum.length);collided.fill(Infinity);
let classified=0,steps=0;
const start=performance.now();
for(let ix=0;ix<3601*1800;ix+=STRIDE){
 const row=Math.floor(ix/1800),col=ix%1800,q1=row*.1-180,q2=row%2?-.1-col*.1:-180+col*.1;
 const m=model(q1,q2,150,ROLL);let polys=null,self=false;steps++;
 for(const c of m.candidates){
  if(!Number.isFinite(c.mu)||c.x>0||c.y<0||Math.hypot(c.x,c.y)>500)continue;
  const px=Math.round(120-c.x*.94),py=Math.round(635-c.y*.94),pixel=py*1100+px;
  if(px<0||px>=1100||py<0||py>=780||c.mu>=minimum[pixel])continue;
  if(!polys){polys=collisionGeometry(q1,q2);self=selfCollision(polys);classified++;}
  const hit=self||objectCollision(polys,c,150,ROLL),values=hit?collided:minimum;
  if(c.mu<values[pixel])values[pixel]=c.mu;
 }
}
const elapsedMs=performance.now()-start;
const crypto=require('crypto'),hash=crypto.createHash('sha256');
hash.update(Buffer.from(minimum.buffer));hash.update(Buffer.from(collided.buffer));
console.log(JSON.stringify({roll:ROLL,stride:STRIDE,steps,classified,elapsedMs,
 collisionFreePixels:minimum.filter(Number.isFinite).length,
 collisionPixels:collided.filter(Number.isFinite).length,pixelHash:hash.digest('hex')}));
'''


def benchmark(path, stride, roll):
    html = path.read_text()
    model = html[html.index('const rot='):html.index('let current=')]
    collision = html[html.index('const CONTACT_OVERLAP_TOL_MM='):html.index('function muLimit()')]
    result = subprocess.check_output(
        ['node'], input=f'const STRIDE={stride},ROLL={roll};\n' + model + collision + PROBE,
        text=True)
    return dict(path=str(path), **json.loads(result))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('html', nargs='+', type=Path)
    parser.add_argument('--stride', type=int, default=37)
    parser.add_argument('--roll', type=int, choices=[-90, 90], default=90)
    args = parser.parse_args()
    if args.stride <= 0:
        parser.error('stride must be positive')
    for path in args.html:
        print(json.dumps(benchmark(path, args.stride, args.roll)), flush=True)


if __name__ == '__main__':
    main()
