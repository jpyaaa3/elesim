"""Compare optimized mapper collision decisions with the original algorithms."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import unittest

MAPPER = Path(os.environ.get('HUGMAP_MAPPER_HTML', '/home/user/ws/hugmap/mapper.html'))

# Keep the straightforward implementation independent of optimized metadata.
REFERENCE = r'''
function referenceGap(a,b){
 for(const axis of [[1,0],[0,1]]){const aa=a.map(p=>dot(p,axis)),bb=b.map(p=>dot(p,axis));if(Math.max(Math.min(...bb)-Math.max(...aa),Math.min(...aa)-Math.max(...bb))>0)return 1;}
 let gap=-Infinity;
 for(const poly of [a,b])for(let i=0;i<poly.length;i++){const e=sub(poly[(i+1)%poly.length],poly[i]),len=Math.hypot(...e);if(len<1e-12)continue;const axis=[e[1]/len,-e[0]/len],aa=a.map(p=>dot(p,axis)),bb=b.map(p=>dot(p,axis));gap=Math.max(gap,Math.min(...bb)-Math.max(...aa),Math.min(...aa)-Math.max(...bb));if(gap>=-.05)return gap;}
 return gap;
}
function referenceDisk(poly,c,R){let inside=false,best=Infinity;for(let i=0,j=poly.length-1;i<poly.length;j=i++){
 const a=poly[j],b=poly[i],e=sub(b,a),t=Math.max(0,Math.min(1,dot(sub(c,a),e)/dot(e,e)));best=Math.min(best,Math.hypot(...sub(c,add(a,mul(e,t)))));
 if((a[1]>c[1])!==(b[1]>c[1])&&c[0]<(b[0]-a[0])*(c[1]-a[1])/(b[1]-a[1])+a[0])inside=!inside;
 }return (inside?-best:best)-R;
}
function referenceGeometry(q1d,q2d){let origin=[0,0],angles=[],hinges=[];const q1=q1d*Math.PI/180,q2=q2d*Math.PI/180;
 for(let i=0;i<10;i++){const a=i<5?(i+1)*q1/5:q1+(i-4)*q2/5;angles.push(a);hinges.push(origin);origin=add(origin,rot([0,50],a));}angles.push(angles[9]);hinges.push(origin);
 return collisionProfiles.map((poly,i)=>poly.map(p=>add(rot(p,angles[Math.min(i,10)]),hinges[Math.min(i,10)])));
}
function referenceSelf(polys){for(let i=0;i<polys.length;i++){for(let j=i+1;j<polys.length;j++){if(i>=9&&j>=10)continue;if(referenceGap(polys[i],polys[j])<-.05)return true;}for(const body of collisionBodies)if(referenceGap(polys[i],body)<-.05)return true;}return false;}
function referenceObject(polys,c,D,roll){const center=[c.x*roll/90,c.y-144.5],R=D/2;
 return polys.some((p,i)=>referenceDisk(p,center,R)<(i===13?1:-1))||collisionBodies.some(p=>referenceDisk(p,center,R)<=1);
}
'''

PROBE = r'''
let seed=123456789;function random(){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;}
const report={poses:0,pairs:0,objects:0,geometryError:0,gapMismatch:0,selfMismatch:0,objectMismatch:0,combinedMismatch:0};
for(let k=0;k<1800;k++){
 // Repeated q1 values exercise reuse across distinct distal poses.
 const q1=k%3===0?100:360*random()-180,q2=-180*random();
 const p=collisionGeometry(q1,q2),ref=referenceGeometry(q1,q2);report.poses++;
 for(let i=0;i<p.length;i++)for(let j=0;j<p[i].length;j++)report.geometryError=Math.max(report.geometryError,Math.hypot(...sub(p[i][j],ref[i][j])));
 report.selfMismatch+=selfCollision(p)!==referenceSelf(ref);
 for(let j=0;j<8;j++){
  const c={x:800*random()-400,y:650*random()-100},D=50+150*random(),roll=j%2?90:-90;
  report.objects++;report.objectMismatch+=finalObjectCollision(p,c,D,roll)!==referenceObject(ref,c,D,roll);
  report.combinedMismatch+=objectCollision(p,c,D,roll)!==(referenceObject(ref,c,D,roll)||preparationCollision(c,D));
 }
 const a=p[k%p.length],b=p[(k+5)%p.length];report.pairs++;report.gapMismatch+=(polygonGap(a,b)<-.05)!==(referenceGap(a,b)<-.05);
}
// Exercise the exact contact tolerances and both sides of each boundary.
for(const gap of [-1.000001,-1,-.999999,-.050001,-.05,-.049999,0,.999999,1,1.000001]){
 const a=[[0,0],[10,0],[10,10],[0,10]],b=[[10+gap,0],[20+gap,0],[20+gap,10],[10+gap,10]];
 report.pairs++;report.gapMismatch+=(polygonGap(a,b)<-.05)!==(referenceGap(a,b)<-.05);
 if(Math.abs(diskGap(a,[15+gap,5],5)-referenceDisk(a,[15+gap,5],5))>1e-10)throw Error('Disk boundary changed');
}
console.log(JSON.stringify(report));
'''


@unittest.skipUnless(MAPPER.exists() and shutil.which('node'), 'Standalone mapper and Node required')
class MapperOptimizationTests(unittest.TestCase):
    def test_collision_decisions_and_geometry_are_preserved(self):
        html = MAPPER.read_text()
        model = html[html.index('const rot='):html.index('let current=')]
        collision = html[html.index('const CONTACT_OVERLAP_TOL_MM='):html.index('function muLimit()')]
        report = json.loads(subprocess.check_output(
            ['node'], input=model + collision + REFERENCE + PROBE, text=True))
        self.assertEqual(report['gapMismatch'], 0, report)
        self.assertEqual(report['selfMismatch'], 0, report)
        self.assertEqual(report['objectMismatch'], 0, report)
        self.assertEqual(report['combinedMismatch'], 0, report)
        self.assertLess(report['geometryError'], 1e-10, report)


if __name__ == '__main__':
    unittest.main()
