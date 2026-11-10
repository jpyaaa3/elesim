"""Regression checks for the standalone shared mapper (stdlib + Node).

Set HUGMAP_MAPPER_HTML when testing a separately shared copy.
"""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import unittest

MAPPER = Path(os.environ.get('HUGMAP_MAPPER_HTML', '/home/user/ws/hugmap/mapper.html'))

@unittest.skipUnless(MAPPER.exists() and shutil.which('node'), 'Standalone mapper and Node required')
class MapperToleranceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        html = MAPPER.read_text()
        data = re.search(r'const DATA=(.*);', html).group(0)
        model = html[html.index('const rot='):html.index('let current=')]
        collision = html[html.index('const CONTACT_OVERLAP_TOL_MM='):html.index('function muLimit()')]
        probe = r'''
const report={reconstructed:0,rejected:0,protected:0,missedProtected:0,savedRejected:0,severe:0,severeRejected:0};
for(let i=0;i<DATA.rows.length;i++){
 const [D,roll,x,y,q1,q2]=DATA.rows[i],p=collisionGeometry(q1,q2),cs=model(q1,q2,D,roll).candidates;
 if(i%41===0&&(selfCollision(p)||finalObjectCollision(p,{x,y},D,roll)))report.savedRejected++;
 if(!cs.length)continue;
 const c=cs.reduce((a,b)=>Math.hypot(a.x-x,a.y-y)<Math.hypot(b.x-x,b.y-y)?a:b),center=[c.x*roll/90,c.y-144.5];
 const protectedHit=diskGap(p[13],center,D/2)<1||collisionBodies.some(poly=>diskGap(poly,center,D/2)<=1),hit=finalObjectCollision(p,c,D,roll);
 report.reconstructed++;report.rejected+=hit;report.protected+=protectedHit;report.missedProtected+=protectedHit&&!hit;
}
for(let q1=-180;q1<=-120;q1+=5)for(let q2=-180;q2<0;q2+=5){const p=collisionGeometry(q1,q2);for(const c of model(q1,q2,150,-90).candidates)if(c.mu<=.5&&c.x<=0&&c.y>=0&&Math.hypot(c.x,c.y)<=500){report.severe++;report.severeRejected+=selfCollision(p)||finalObjectCollision(p,c,150,-90);}}
console.log(JSON.stringify(report));
'''
        cls.report = json.loads(subprocess.check_output(['node'], input=data + model + collision + probe, text=True))

    def test_reconstruction_contact_error_is_tolerated(self):
        self.assertGreater(self.report['reconstructed'], 28000)
        self.assertEqual(self.report['rejected'], self.report['protected'], self.report)

    def test_protected_parts_remain_protected(self):
        self.assertGreater(self.report['protected'], 0)
        self.assertEqual(self.report['missedProtected'], 0)

    def test_saved_exact_poses_still_pass(self):
        self.assertEqual(self.report['savedRejected'], 0)

    def test_folded_impossible_branch_still_rejected(self):
        self.assertEqual(self.report['severe'], 26)
        self.assertEqual(self.report['severeRejected'], 26)

if __name__ == '__main__':
    unittest.main()
