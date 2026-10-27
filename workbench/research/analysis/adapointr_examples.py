"""Export illustrative RGB/point-cloud examples from completed AP-PS55 trials."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


EXAMPLES = (
    ('good', 'view-0007-2-component-01', 'Good reconstruction'),
    ('partial', 'view-0003-2-component-01', 'Incomplete reconstruction'),
    ('failure', 'view-0007-2-component-02', 'Remaining failure'),
)


def export(root, sensors, truth_root, output):
    import cv2
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from workbench.research.experiments.adapointr_evaluate import sample_box

    root, sensors, truth_root, output = map(Path, (root, sensors, truth_root, output))
    output.mkdir(parents=True, exist_ok=True)
    env = json.loads((root / 'environment.json').read_text())
    evaluated = {label: json.loads((root / 'evaluation' / ('evaluation-' + label + '.json')).read_text())
                 for label in (env['baseline'], env['primary_variant'])}
    rows = {label: {v['id']: v for v in record['objects']} for label, record in evaluated.items()}
    evidence = {'schema_version': 1, 'source_revision': env['source_revision'],
                'selection': 'Three deliberately chosen examples spanning observed quality; not a random evaluation sample',
                'source': 'Recorded static Genesis RGB-D trial; no new inference or scene rendering',
                'panels': ['Recorded RGB with highlighted component', 'Observed points', 'PCN baseline', 'AP-PS55 primary', 'Evaluation truth'],
                'examples': []}
    for name, identity, title in EXAMPLES:
        before = rows[env['baseline']][identity]
        after = rows[env['primary_variant']][identity]
        paths = {label: root / ('predictions-' + label) / 'predictions' / record['output_file']
                 for label, record in ((env['baseline'], before), (env['primary_variant'], after))}
        with np.load(paths[env['baseline']], allow_pickle=False) as packet:
            observed, old = packet['observed'], packet['predicted']
        with np.load(paths[env['primary_variant']], allow_pickle=False) as packet:
            np.testing.assert_array_equal(observed, packet['observed'])
            new = packet['predicted']
        sensor_path = sensors / before['capture']
        with np.load(sensor_path, allow_pickle=False) as packet:
            color = packet['color_bgr']
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        red = (((hsv[:, :, 0] <= 12) | (hsv[:, :, 0] >= 168)) &
               (hsv[:, :, 1] >= 100) & (hsv[:, :, 2] >= 60)).astype(np.uint8)
        _, components = cv2.connectedComponents(red, connectivity=8)
        selected = components == before['component']
        assert np.any(selected)
        case = int(before['capture'].split('-')[1])
        truth_path = truth_root / f'case-{case:04d}.npz'
        with np.load(truth_path, allow_pickle=False) as packet:
            chosen = before['assigned_box']
            assert chosen == after['assigned_box']
            gt = sample_box(packet['object_centers'][chosen], packet['object_sizes'][chosen], packet['object_rotations'][chosen], count=4000)
        clouds = [observed, old, new, gt]
        all_points = np.concatenate(clouds)
        center = (all_points.min(0) + all_points.max(0)) / 2
        radius = max(np.ptp(all_points, axis=0).max() / 2, .01) * 1.08
        figure = plt.figure(figsize=(17, 4.5), layout='constrained')
        image_axis = figure.add_subplot(1, 5, 1)
        image_axis.imshow(color[:, :, ::-1])
        image_axis.contour(selected.astype(float), levels=[.5], colors=['#ffe54b'], linewidths=1.4)
        image_axis.set_title('Recorded RGB: yellow outline', fontsize=10)
        image_axis.set_axis_off()
        for column, (cloud, color_value, panel_title) in enumerate(zip(
                clouds, ['#176fba', '#7871b8', '#159b89', '#777777'],
                ['Observed depth points', 'Old: PCN', 'New: AP-PS55', 'Complete truth: evaluation only']), start=2):
            axis = figure.add_subplot(1, 5, column, projection='3d')
            shown = cloud[::max(1, len(cloud) // 3000)]
            axis.scatter(*shown.T, s=1.4, c=color_value, alpha=.75, rasterized=True)
            axis.set(xlim=(center[0]-radius, center[0]+radius),
                     ylim=(center[1]-radius, center[1]+radius), zlim=(center[2]-radius, center[2]+radius))
            axis.set_box_aspect((1, 1, 1))
            axis.view_init(elev=22, azim=-55)
            axis.set_xticks([])
            axis.set_yticks([])
            axis.set_zticks([])
            axis.set_title(panel_title, fontsize=10)
            if column in (3, 4):
                row = before if column == 3 else after
                axis.text2D(.02, -.01, f"Surface p95 {row['surface_p95_mm']:.1f} mm\nF1@10mm {row['f1_10mm']:.2f}", transform=axis.transAxes, fontsize=10)
        figure.suptitle(title + ' | ' + identity, fontsize=13)
        figure.supxlabel('Illustrative selection from the completed box trial. All point-cloud panels share world-frame bounds. Truth is never supplied to inference.', fontsize=9)
        path = output / ('adapointr_example_' + name + '.png')
        figure.savefig(path, dpi=150)
        plt.close(figure)
        evidence['examples'].append({'id': identity, 'selection_label': name, 'file': path.name,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'bytes': path.stat().st_size,
            'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (*paths.values(), sensor_path, truth_path)},
            'before': {key: before[key] for key in ('surface_p95_mm', 'f1_10mm', 'hidden_recall_10mm')},
            'after': {key: after[key] for key in ('surface_p95_mm', 'f1_10mm', 'hidden_recall_10mm')}})
    (output / 'examples.json').write_text(json.dumps(evidence, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('root', 'sensors', 'truth', 'output'):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    export(args.root, args.sensors, args.truth, args.output)


if __name__ == '__main__':
    main()
