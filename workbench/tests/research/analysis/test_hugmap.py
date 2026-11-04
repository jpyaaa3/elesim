"""Focused fixed-placement, export and standalone regressions for hugmap."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np
from workbench.research.support import hugmap as h


class HugmapTests(unittest.TestCase):
    def setUp(self):
        # Independently recorded newhug 100 mm geometry golden, no fixture dependency.
        self.q = np.radians([-60., -165.])
        self.center = h.rotate(-np.array([-.11289388160975815,.04518094209779758]),
                              -(math.radians(40.6937925642764)-math.pi/2))

    def test_grid_is_complete_quadrant_with_no_theta_duplicates(self):
        for config in (h.Config(),h.Config(73,41,7,.5),h.Config(250,250,90,.5)):
            points=list(h.grid_points(config))
            self.assertEqual(len(points),h.grid_size(config))
            self.assertEqual(len(points),len(set(points)))
            self.assertTrue(all(x<=0 and y>=0 and x*x+y*y<=250000+1e-8 and t==0 for x,y,t in points))
        self.assertEqual(list(h.grid_points(h.Config(250,250,1,.5))),
                         list(h.grid_points(h.Config(250,250,90,.5))))
        self.assertEqual(h.theta_values(h.Config(dtheta_deg=90),shape='polygon'),(-180,-90,0,90))
        expected={(x,y,0.) for x in range(-500,1,20) for y in range(0,501,20) if x*x+y*y<=250000}
        self.assertEqual(set(h.grid_points(h.Config())),expected)
        self.assertEqual(h.grid_size(h.Config()),516)
        meta=h.map_metadata(h.Config(),1)
        self.assertEqual(meta['symmetry']['computed_roll_deg'],90)
        self.assertEqual(meta['symmetry']['reflected_roll_deg'],-90)
        self.assertEqual(meta['constraints']['interest_region'],'left_forward_quadrant')

    def test_coordinates_are_object_relative_to_fixed_body(self):
        np.testing.assert_allclose(h.local_center(0,0,0),h.BODY_CENTER)
        p=(self.center-h.BODY_CENTER)*1000
        np.testing.assert_allclose(h.local_center(*p,0),self.center)
        self.assertIsNone(h.exclusion(self.center))
        self.assertEqual(h.exclusion(h.BODY_CENTER),'body_clearance')

    def test_known_geometry_and_strict_mu_cutoff(self):
        sol=h.inspect_pose(self.q,self.center,.5)
        self.assertAlmostEqual(sol['mu_min'],.07870170682461718,places=10)
        self.assertIsNone(h.inspect_pose(self.q,self.center,sol['mu_min']))
        self.assertIsNone(h.inspect_pose([4,0],self.center,.5))
        self.assertIsNone(h.inspect_pose(self.q,h.BODY_CENTER,.5))

    def test_solver_continues_to_neighboring_placements(self):
        for offset in (-.001,.001):
            center=self.center+[offset,0]
            result=h.solve_placement(center,.5)
            self.assertEqual(result['status'],'found')
            self.assertEqual(result['attempts'],64)
            best=result['best']
            self.assertLess(best['mu_min'],.5)
            checked=h.inspect_pose(np.radians([best['seg1_deg'],best['seg2_deg']]),center,.5)
            self.assertIsNotNone(checked)

    def test_batched_geometry_and_analytic_jacobian(self):
        rng=np.random.default_rng(13)
        q=rng.uniform(-3,3,(16,2))
        center=np.array([.12,.16])
        parts=np.arange(len(h.PROFILES))
        gaps,jacobian,points=h.batched_contacts(q,parts,center)
        for k,angles in enumerate(q):
            surfaces,_,_,_=h.robot_polygons(angles)
            np.testing.assert_allclose(gaps[k],
                [h.circle_gap(p,h.RADIUS,center) for p in surfaces],atol=1e-12)
            np.testing.assert_allclose(points[k],
                [h.nearest_boundary([center],p)[1][0] for p in surfaces],atol=1e-12)
        for axis in range(2):
            delta=np.zeros(2);delta[axis]=1e-6
            numerical=(h.batched_contacts(q+delta,parts,center)[0]-
                       h.batched_contacts(q-delta,parts,center)[0])/2e-6
            np.testing.assert_allclose(jacobian[:,:,axis],numerical,atol=1e-7)

    def test_candidate_limit_and_cancel(self):
        with patch.object(h,'MAX_CANDIDATES',1):
            result=h.solve_placement(self.center,.5)
        self.assertEqual(result['attempts'],h.MAX_ATTEMPTS)
        self.assertEqual(result['termination'],'candidate_limit')
        stop=threading.Event();stop.set()
        with self.assertRaises(h.Cancelled):
            h.solve_placement(self.center,.5,stop)

    def test_invalid_settings(self):
        for config in (h.Config(dx_mm=0),h.Config(mu_allow=float('nan')),
                       h.Config(dtheta_deg=361),h.Config(dy_mm=1e-300)):
            with self.assertRaises(ValueError):config.checked()

    def test_cancel_exports_valid_json_and_preserves_existing_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'map.json'
            stop=threading.Event();stop.set()
            result=h.run_map(h.Config(),path,stop=stop,workers=1)
            self.assertEqual(result['status'],'cancelled')
            self.assertEqual(json.loads(path.read_text())['results'],[])
            original=path.read_bytes()
            with self.assertRaises(FileExistsError):h.run_map(h.Config(),path,workers=1)
            self.assertEqual(path.read_bytes(),original)
            self.assertFalse(Path(str(path)+'.partial').exists())

    def test_standalone_parallel_matches_serial(self):
        # Copy only the one deliverable; isolated Python and unrelated cwd.
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp);script=tmp/'hugmap.py';shutil.copyfile(h.__file__,script)
            runs=[]
            for workers in (1,2):
                output=tmp/f'{workers}.json'
                result=subprocess.run([sys.executable,'-I',str(script),'--run','--output',str(output),
                    '--dx-mm','250','--dy-mm','250','--workers',str(workers)],cwd=tmp,
                    capture_output=True,text=True,timeout=120)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                data=json.loads(output.read_text());runs.append(data)
                self.assertEqual(data['summary']['status'],'complete')
                self.assertEqual(data['summary']['visited'],h.grid_size(h.Config(250,250)))
                self.assertEqual(data['summary']['visited'],len(data['results']))
                self.assertTrue(all(row['theta_deg']==0 for row in data['results']))
                print(f"standalone workers={workers}: {data['summary']['elapsed_s']:.2f}s",flush=True)
            key=lambda row:(row['x_mm'],row['y_mm'],row['theta_deg'])
            self.assertEqual(sorted(runs[0]['results'],key=key),sorted(runs[1]['results'],key=key))

    def test_active_workers_cancel_and_finalize(self):
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'cancelled.json'
            stop=threading.Event()
            timer=threading.Timer(1.5,stop.set)
            timer.start()
            try:
                result=h.run_map(h.Config(),output,stop=stop,workers=2)
            finally:
                timer.cancel()
            data=json.loads(output.read_text())
            self.assertEqual(result['status'],'cancelled')
            self.assertEqual(result['visited'],len(data['results']))
            self.assertLess(result['visited'],result['total'])
            self.assertFalse(Path(str(output)+'.partial').exists())


class SavedMapValidationTests(unittest.TestCase):
    def document(self):
        return dict(metadata=dict(schema='hugmap',schema_version=1,
            config=dict(dx_mm=20,dy_mm=20),constraints=dict(object_center_semicircle_radius_mm=500)),
            summary=dict(status='cancelled'),results=[
                dict(x_mm=-20,y_mm=200,theta_deg=0,status='found',seg1_deg=-60,seg2_deg=120,mu_min=.2),
                dict(x_mm=0,y_mm=200,theta_deg=0,status='not_found',seg1_deg=None,seg2_deg=None,mu_min=None),
                dict(x_mm=20,y_mm=200,theta_deg=0,status='excluded',seg1_deg=None,seg2_deg=None,mu_min=None),
                dict(x_mm=-20,y_mm=200,theta_deg=90,status='found',seg1_deg=80,seg2_deg=-150,mu_min=.4)])

    def test_empty_saved_map(self):
        doc=self.document();doc['results']=[]
        self.assertEqual(h.SavedMapData(doc).count,0)

    def test_invalid_documents(self):
        for field,value in [('schema_version',2),('schema','newhug')]:
            doc=self.document();doc['metadata'][field]=value
            with self.assertRaises(ValueError):h.SavedMapData(doc)
        for key,value in [('mu_min',float('nan')),('mu_min',-.1),('seg1_deg',None),
                          ('theta_deg',float('inf')),('x_mm',3),('status','bogus')]:
            doc=self.document();doc['results'][0][key]=value
            with self.assertRaises(ValueError):h.SavedMapData(doc)
        for key in ('metadata','summary','results'):
            doc=self.document();doc[key]=None
            with self.assertRaises(ValueError):h.SavedMapData(doc)
        doc=self.document();doc['results'].append(doc['results'][0].copy())
        with self.assertRaises(ValueError):h.SavedMapData(doc)

    def test_ascii_and_no_cli_reader_option(self):
        self.assertTrue(Path(h.__file__).read_text().isascii())
        result=subprocess.run([sys.executable,h.__file__,'--help'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertNotIn('--read',result.stdout)


class DiameterTableTests(unittest.TestCase):
    def test_standalone_table_cli_and_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp);script=tmp/'hugmap.py';shutil.copyfile(h.__file__,script)
            output=tmp/'table.json'
            commands=[['--table','--output',str(output),'--dx-mm','1000','--dy-mm','1000',
                       '--diameter-min-mm','50','--diameter-max-mm','60','--diameter-step-mm','5'],
                      ['--resume',str(output)]]
            for args in commands:
                result=subprocess.run([sys.executable,'-I',str(script),*args,'--workers','1'],
                    cwd=tmp,capture_output=True,text=True,timeout=30)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            doc=json.loads(output.read_text())
            self.assertEqual(doc['summary']['status'],'complete')
            self.assertEqual(doc['summary']['resumed_visited'],3)
            self.assertEqual(doc['summary']['computed_this_run'],0)

    def test_diameter_plan(self):
        self.assertEqual(h.diameter_values(),list(range(50,251,5)))
        for args in ((50,49,5),(50,60,0),(50,60,3),(float('nan'),60,5)):
            with self.assertRaises(ValueError):h.diameter_values(*args)

    def test_one_file_contains_41_maps(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'table.json'
            result=h.run_table(h.Config(1000,1000),path,workers=1)
            doc=json.loads(path.read_text())
            self.assertEqual(result['completed_maps'],41)
            self.assertEqual(len(doc['maps']),41)
            self.assertEqual(result['visited'],41)
            self.assertEqual(list(Path(tmp).glob('*.json')),[path])
            self.assertTrue(all(e['status']=='complete' for e in doc['maps']))
            self.assertEqual(h.validate_table(doc,for_resume=True),h.Config(1000,1000))

    def test_cancel_then_resume_uses_saved_settings_and_skips_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'table.json';stop=threading.Event()
            result=h.run_table(h.Config(1000,1000),path,50,60,5,workers=1,stop=stop,
                               progress=lambda _:stop.set())
            self.assertEqual(result['status'],'cancelled')
            first=json.loads(path.read_text())
            self.assertEqual(first['summary']['visited'],1)
            self.assertEqual(first['maps'][1]['status'],'pending')
            result=h.run_table(h.Config(),path,workers=1,resume=True)
            self.assertEqual(result['status'],'complete')
            self.assertEqual(result['resumed_visited'],1)
            final=json.loads(path.read_text())
            self.assertEqual(final['metadata']['config']['dx_mm'],1000)
            self.assertEqual(final['maps'][0]['results'],first['maps'][0]['results'])
            again=h.run_table(h.Config(),path,workers=1,resume=True)
            self.assertEqual(again['computed_this_run'],0)
            self.assertEqual(again['resumed_visited'],3)

    def test_radius_is_explicit_and_shared_by_worker_geometry(self):
        center=h.local_center(-100,340)
        old_radius=h.RADIUS
        solutions=[]
        for radius in (.05,.055):
            h.worker_init(threading.Event())
            result=h.solve_batch([((-100,340,0),[(-100,340,0)])],.5,radius)[0][1]
            self.assertEqual(result['status'],'found')
            q=np.radians([result['seg1_deg'],result['seg2_deg']])
            self.assertIsNotNone(h.inspect_pose(q,center,.5,radius))
            solutions.append(result)
        self.assertNotAlmostEqual(solutions[0]['seg1_deg'],solutions[1]['seg1_deg'],places=2)
        self.assertEqual(h.RADIUS,old_radius)

    def test_resume_rejects_incompatible_metadata_and_preserves_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'table.json';h.run_table(h.Config(1000,1000),path,50,50,5,workers=1)
            doc=json.loads(path.read_text());doc['metadata']['constraints']['clearance_mm']=0
            path.write_text(json.dumps(doc));original=path.read_bytes()
            with self.assertRaises(ValueError):h.run_table(h.Config(),path,workers=1,resume=True)
            self.assertEqual(path.read_bytes(),original)
            with self.assertRaises(FileExistsError):h.run_table(h.Config(),path,workers=1)

    def test_table_lock_and_atomic_snapshot(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'table.json'
            with h.TableCheckpoint(path,False) as writer:
                writer.save({'value':'first'})
                with self.assertRaises(ValueError):
                    with h.TableCheckpoint(path,True):pass
                with patch.object(h.os,'replace',side_effect=OSError('simulated failure')):
                    with self.assertRaises(OSError):writer.save({'value':'second'})
                self.assertEqual(json.loads(path.read_text()),{'value':'first'})
            with h.TableCheckpoint(path,True) as writer:writer.save({'value':'third'})
            self.assertEqual(json.loads(path.read_text()),{'value':'third'})

    def test_parallel_table_and_resume_match_uninterrupted_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'table.json';reference=Path(tmp)/'reference.json'
            cfg=h.Config(250,250);stop=threading.Event()
            timer=threading.Timer(1,stop.set);timer.start()
            try:h.run_table(cfg,path,100,110,10,stop=stop,workers=2)
            finally:timer.cancel()
            resumed=h.run_table(cfg,path,workers=2,resume=True)
            self.assertEqual(resumed['status'],'complete')
            h.run_table(cfg,reference,100,110,10,workers=1)
            a=json.loads(path.read_text());b=json.loads(reference.read_text())
            key=lambda r:(r['x_mm'],r['y_mm'],r['theta_deg'])
            for x,y in zip(a['maps'],b['maps']):
                self.assertEqual(x['diameter_mm'],y['diameter_mm'])
                self.assertEqual(sorted(x['results'],key=key),sorted(y['results'],key=key))
            # Resuming one partially saved batch must retain warm-start behavior.
            incomplete=json.loads(reference.read_text())
            incomplete['maps'][0]['results']=[r for r in incomplete['maps'][0]['results'] if not (r['x_mm']==250 and r['y_mm']==250)]
            incomplete['maps'][0]['status']='cancelled'
            path.write_text(json.dumps(incomplete))
            h.run_table(cfg,path,workers=2,resume=True)
            restored=json.loads(path.read_text())
            self.assertEqual(sorted(restored['maps'][0]['results'],key=key),sorted(b['maps'][0]['results'],key=key))


if __name__=='__main__':
    unittest.main()
