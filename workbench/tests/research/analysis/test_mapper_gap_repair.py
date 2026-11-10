"""Gap repair must solve the model at XY, rather than paint a dilation."""
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import unittest

from workbench.research.analysis.benchmark_mapper_gaps import STUB

MAPPER = Path(os.environ.get('HUGMAP_MAPPER_HTML', '/home/user/ws/hugmap/mapper.html'))


@unittest.skipUnless(MAPPER.exists() and shutil.which('node'), 'Standalone mapper and Node required')
class MapperGapRepairTests(unittest.TestCase):
    def test_inverse_reproduces_forward_contacts(self):
        html = MAPPER.read_text()
        model = html[html.index('const rot='):html.index('let current=')]
        kernel = html[html.index('const CONTACT_OVERLAP_TOL_MM='):html.index('function muLimit()')]
        probe = r'''
const report={targets:0,recovered:0,validRecovered:0,valid:0,maxError:0,invalid:0};
for(const D of [85,100,150,200])for(const roll of [-90,90]){
 for(let q3=-170;q3<=170;q3+=5)for(let q4=-175;q4<0;q4+=5){
  const m=model(q3,q4,D,roll);for(const c of m.candidates){
   if(!Number.isFinite(c.mu)||c.mu>1||c.x>0||c.y<0||Math.hypot(c.x,c.y)>500)continue;
   const p=collisionGeometry(q3,q4),valid=!selfCollision(p)&&!objectCollision(p,c,D,roll);report.targets++;report.valid+=valid;
   const hit=inverseContactPose(c.x,c.y,D,roll,{q4:q4+.03,kind:c.kind,branch:c.branch});
   if(hit){report.recovered++;report.validRecovered+=valid;report.maxError=Math.max(report.maxError,Math.hypot(c.x-hit.c.x,c.y-hit.c.y));}
  }
 }
 for(const [x,y] of [[-1000,-1000],[1000,1000],[0,0]])for(const kind of ['side','tip','heel']){
  const hit=inverseContactPose(x,y,D,roll,{q4:-120,kind,branch:1});
  if(hit){const p=collisionGeometry(hit.q3,hit.q4);if(!selfCollision(p)&&!objectCollision(p,hit.c,D,roll))report.invalid++;}
 }
}
console.log(JSON.stringify(report));
'''
        report = json.loads(subprocess.check_output(['node'], input=model + kernel + probe, text=True))
        self.assertGreater(report['valid'], 500, report)
        self.assertGreater(report['validRecovered'] / report['valid'], .99, report)
        self.assertLess(report['maxError'], 1e-4, report)
        self.assertEqual(report['invalid'], 0, report)

    def test_repaired_pixels_have_real_collision_free_poses(self):
        html = MAPPER.read_text()
        script = re.search(r'<script>(.*?)</script>', html, re.S).group(1)
        probe = r'''
clearConstruction();const D=150,roll=-90;let initial=0;
for(let q3=-110;q3<=80;q3++)for(let q4=-179;q4<0;q4++)for(const c of model(q3,q4,D,roll).candidates){
 if(!Number.isFinite(c.mu)||c.mu>.1||c.x>0||c.y<0||Math.hypot(c.x,c.y)>500)continue;
 const p=collisionGeometry(q3,q4);if(selfCollision(p)||objectCollision(p,c,D,roll))continue;
 const px=Math.round(X(c.x)),py=Math.round(Y(c.y)),pixel=py*canvas.width+px;
 if(px<0||px>=canvas.width||py<0||py>=canvas.height||c.mu>=minMu[pixel])continue;
 minMu[pixel]=c.mu;seenPixel[pixel]=1;poseSeeds.set(pixel,{q3,q4,mu:c.mu,kind:c.kind,branch:c.branch});
}
const before=new Float64Array(minMu);initial=poseSeeds.size;prepareGapRepair();for(const entry of gapQueue)repairGap(entry,D,roll);
let verified=0,invalid=0,lost=0;
for(let i=0;i<minMu.length;i++)lost+=minMu[i]>before[i];
for(const [pixel,seed] of poseSeeds)if(seed.verified){verified++;
 const hit=inverseContactPose(seed.x,seed.y,D,roll,seed);if(!hit){invalid++;continue;}
 const p=collisionGeometry(hit.q3,hit.q4);
 if(Math.round(X(seed.x))!==pixel%canvas.width||Math.round(Y(seed.y))!==Math.floor(pixel/canvas.width)||selfCollision(p)||objectCollision(p,hit.c,D,roll))invalid++;
}
console.log(JSON.stringify({initial,verified,invalid,lost}));
'''
        report = json.loads(subprocess.check_output(['node'], input=STUB + script + probe, text=True))
        self.assertGreater(report['initial'], 10, report)
        self.assertGreater(report['verified'], 10, report)
        self.assertEqual(report['invalid'], 0, report)
        self.assertEqual(report['lost'], 0, report)


if __name__ == '__main__':
    unittest.main()
