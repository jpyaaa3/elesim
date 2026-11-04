"""CAD regression: 130 mm housing is centered on the roll axis at height 155 mm."""
import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]

class HousingCenterTests(unittest.TestCase):
    def test_both_profiles_mesh_and_roll_axis(self):
        for profile in ('zed-mini','d435'):
            base=ROOT/'payload/data/models/assemblies'/profile
            vertices=np.array([[float(x) for x in line.split()[1:4]] for line in
                (base/'assets/housing/housing_mesh.obj').read_text().splitlines() if line.startswith('v ')])
            np.testing.assert_allclose([vertices[:,1].min(),vertices[:,1].max()],[-.065,.065],atol=1e-12)
            arc=vertices[vertices[:,2]>.15501,1:]
            np.testing.assert_allclose(np.linalg.norm(arc-[0,.155],axis=1),.065,atol=1e-6)
            physics=json.loads((base/'assets/housing/housing_physics.json').read_text())
            self.assertEqual(physics['com'][1],0)
            for name in ('arm.urdf','robot.urdf'):
                tree=ET.parse(base/name)
                self.assertEqual(float(tree.find(".//link[@name='housing']/inertial/origin").get('xyz').split()[1]),0)
                axis=tree.find(".//joint[@name='j_housing_wedge']/origin")
                np.testing.assert_allclose(np.fromstring(axis.get('xyz'),sep=' '),[0,0,.155],atol=1e-12)

    def test_planning_sources_and_sphere_coverage(self):
        base=ROOT/'payload/data/models/assemblies/zed-mini';plan=ROOT/'payload/data/models/planning/zed-mini'
        meta=json.loads((plan/'metadata.json').read_text())
        for name, digest in meta['artifacts'].items():
            self.assertEqual(digest,hashlib.sha256((plan/name).read_bytes()).hexdigest())
        for name in ('arm.urdf','assets/housing/housing_mesh.obj'):
            self.assertEqual(meta['sources'][name],hashlib.sha256((base/name).read_bytes()).hexdigest())
        tree=ET.parse(plan/'arm.urdf')
        self.assertEqual(float(tree.find(".//link[@name='housing']/inertial/origin").get('xyz').split()[1]),0)
        spheres=yaml.safe_load((plan/'robot.yml').read_text())['robot_cfg']['kinematics']['collision_spheres']['housing']
        centers=np.array([s['center'] for s in spheres]);radii=np.array([s['radius'] for s in spheres])
        vertices=np.array([[float(x) for x in l.split()[1:4]] for l in (base/'assets/housing/housing_mesh.obj').read_text().splitlines() if l.startswith('v ')])
        self.assertTrue(np.all(np.min(np.linalg.norm(vertices[:,None]-centers,axis=2)-radii,axis=1)<=1e-9))

    def test_research_geometry_and_old_checkpoint_rejection(self):
        from workbench.research.support import hugmap,newhug
        for module in (hugmap,newhug):
            np.testing.assert_allclose(np.array(module.REFERENCE['profiles']['housing'])[:,0],[-.065,.065,.065,-.065])
        self.assertEqual(hugmap.BODY_CENTER[0],0)
        meta=hugmap.map_metadata(hugmap.Config(),1)
        meta.update(schema='hugmap-table',schema_version=2,solver_revision='batch-gn-v1',diameters_mm=[100.])
        import copy
        meta=copy.deepcopy(meta);meta['geometry']['profiles']['housing'][0][0]=-.0665
        doc=dict(metadata=meta,maps=[dict(diameter_mm=100.,status='pending',results=[],summary={})],summary={})
        with self.assertRaisesRegex(ValueError,'geometry'):
            hugmap.validate_table(doc,for_resume=True)

if __name__=='__main__':unittest.main()
