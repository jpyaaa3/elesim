"""CPU audit of exported spheres against original triangles, independent of Warp SDF."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

from .v2_geometry import save, sha
from .v2_model import load_link_mesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    metadata = json.loads((args.model/'metadata.json').read_text())
    for name, digest in metadata['artifacts'].items():
        if sha(args.model/name) != digest:
            raise ValueError(f'Model artifact mismatch: {name}')
    for name, digest in metadata['source_meshes'].items():
        if sha(name) != digest:
            raise ValueError(f'Source mesh mismatch: {name}')
    root = ET.parse(args.model/'arm.urdf').getroot()
    spheres = yaml.safe_load((args.model/'robot.yml').read_text())['kinematics']['collision_spheres']
    rows = {}
    for name, values in spheres.items():
        mesh = load_link_mesh(root.find(f"link[@name='{name}']"))
        centers = np.array([sphere['center'] for sphere in values])
        radii = np.array([sphere['radius'] for sphere in values])
        inside = mesh.contains(centers)
        _, distance, _ = trimesh.proximity.closest_point(mesh, centers)
        margin = distance-radii
        rows[name] = dict(spheres=len(radii), source_watertight=bool(mesh.is_watertight),
                          outside_centers=int(np.count_nonzero(~inside)),
                          radius_excess_over_1um=int(np.count_nonzero(margin < -1e-6)),
                          minimum_inward_margin_mm=float(margin.min()*1000),
                          passed=bool(mesh.is_watertight and inside.all() and np.all(radii > 0)
                                      and np.all(margin >= -1e-6)))
    result = dict(model=str(args.model), model_metadata_sha256=sha(args.model/'metadata.json'),
                  method='CPU Trimesh containment and closest original triangle distance; no Warp SDF',
                  numeric_tolerance_m=1e-6, sphere_count=sum(row['spheres'] for row in rows.values()),
                  passed=all(row['passed'] for row in rows.values()), links=rows,
                  containment_certified=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    save(args.output, result)
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
