"""Total-angle q3/q4 projection, interaction, and dual-view scan regression."""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import unittest

from workbench.research.analysis.benchmark_mapper_gaps import STUB

MAPPER = Path(os.environ.get('HUGMAP_MAPPER_HTML', '/home/user/ws/hugmap/mapper.html'))


@unittest.skipUnless(MAPPER.exists() and shutil.which('node'), 'Standalone mapper and Node required')
class MapperQPlaneTests(unittest.TestCase):
    def run_probe(self, probe):
        script = re.search(r'<script>(.*?)</script>', MAPPER.read_text(), re.S).group(1)
        return json.loads(subprocess.check_output(['node'], input=STUB + script + probe, text=True))

    def test_switching_preserves_both_maps_and_scan_position(self):
        result = self.run_probe(r'''
const qp=qPixelFor(-60,-120);recordQCandidate(qp,.2,false);recordQCandidate(qp,.1,true);
minMu[123456]=.123;index=12345;
$('plane').value='q';$('plane').onchange();
const qResult={mu:qMinMu[qp],collision:qCollisionMu[qp],pending:qDirty.length};
$('plane').value='xy';$('plane').onchange();const xy=minMu[123456],position=index;
$('mu').value='.1';$('mu').oninput();const filtered=qMinMu[qp];
clearConstruction();console.log(JSON.stringify({qResult,xy,position,filtered,cleared:qTouched.length===0&&!Number.isFinite(qMinMu[qp])&&!Number.isFinite(minMu[123456])}));
''')
        self.assertEqual(result['qResult'], {'mu': .2, 'collision': .1, 'pending': 0})
        self.assertEqual(result['xy'], .123)
        self.assertEqual(result['position'], 12345)
        self.assertEqual(result['filtered'], .2)
        self.assertTrue(result['cleared'])

    def test_click_and_hover_use_total_segment_degrees(self):
        result = self.run_probe(r'''
canvas.getBoundingClientRect=()=>({left:0,top:0,width:1100,height:780});
$('plane').value='q';$('plane').onchange();
canvas.onclick({clientX:QX(60),clientY:QY(-150)});
const clicked=[$('q3').value,$('q4').value],clickedInfo=$('info').textContent;
canvas.onpointermove({clientX:QX(-45),clientY:QY(-120)});
const hovered=$('info').textContent,unchanged=Number($('q3').value)===60&&Number($('q4').value)===-150;
canvas.onpointerleave();
console.log(JSON.stringify({clicked,clickedInfo,hovered,unchanged,restored:$('info').textContent===clickedInfo,left:QX(-180),right:QX(180),bottom:QY(-180),top:QY(0)}));
''')
        self.assertEqual(list(map(float, result['clicked'])), [60, -150])
        self.assertIn('q3=60.00°', result['clickedInfo'])
        self.assertIn('q4=-150.00°', result['clickedInfo'])
        self.assertIn('q3=-45.00°', result['hovered'])
        self.assertIn('q4=-120.00°', result['hovered'])
        self.assertTrue(result['unchanged'])
        self.assertTrue(result['restored'])
        self.assertEqual([result[k] for k in ('left', 'right', 'bottom', 'top')], [100, 1000, 650, 200])

    def test_real_scan_populates_q_and_xy_without_restarting(self):
        result = self.run_probe(r'''
$('diameter').value=ds.indexOf(150);$('band').value='-90';startConstruction();
index=1690*COLUMNS;for(let i=0;i<4;i++)animationTick(performance.now());stopConstruction();
const scanned=index,xy=touched.length,q=qTouched.length,valid=qTouched.filter(p=>Number.isFinite(qMinMu[p])).length;
$('plane').value='q';$('plane').onchange();$('plane').value='xy';$('plane').onchange();
console.log(JSON.stringify({scanned,xy,q,valid,unchanged:index===scanned,dirty:qDirty.length}));
''')
        self.assertGreater(result['xy'], 0, result)
        self.assertGreater(result['q'], 0, result)
        self.assertGreater(result['valid'], 0, result)
        self.assertTrue(result['unchanged'])
        self.assertEqual(result['dirty'], 0)


if __name__ == '__main__':
    unittest.main()
