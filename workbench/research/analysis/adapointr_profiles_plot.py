"""Plot fixed-replay AdaPoinTr checkpoint/frame comparisons."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def plot(root, truth_root, output):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from workbench.research.experiments.adapointr_evaluate import sample_box

    root, truth_root, output = map(Path, (root, truth_root, output))
    output.mkdir(parents=True, exist_ok=True)
    env = json.loads((root / 'environment.json').read_text())
    labels = [v['label'] for v in env['variants']]
    records = {v: json.loads((root / 'evaluation' / ('evaluation-' + v + '.json')).read_text()) for v in labels}
    colors = ['#7871b8' if v == env['baseline'] else '#159b89' if v == env['primary_variant'] else '#cf8e5a' for v in labels]
    figure, axes = plt.subplots(2, 3, figsize=(14, 9), layout='constrained')
    metrics = [('surface_p95_mm', 'Predicted surface p95 (mm)', 20., 'max'),
               ('observed_to_prediction_p95_mm', 'Observed-to-output p95 (mm)', 10., 'max'),
               ('precision_10mm', 'Precision at 10 mm', .8, 'min'),
               ('f1_10mm', 'F1 at 10 mm', None, 'min'),
               ('hidden_recall_10mm', 'Hidden-surface recall at 10 mm', .5, 'min'),
               ('forward_wall_ms_median', 'Warm forward wall time (ms)', None, 'max')]
    for axis, (key, title, threshold, direction) in zip(axes.flat, metrics):
        values = [records[v]['metrics'][key] for v in labels]
        bars = axis.bar(np.arange(len(labels)), values, color=colors)
        axis.bar_label(bars, fmt='%.2f', padding=3, fontsize=8)
        axis.set(xticks=np.arange(len(labels)), xticklabels=labels, title=title)
        axis.tick_params(axis='x', rotation=35, labelsize=8)
        limit = max(values) * 1.23
        if key == 'f1_10mm':
            axis.axhline(records[env['baseline']]['metrics']['partial_f1_10mm'], color='#176fba', linestyle='--', label='Observed-only F1')
        if threshold is not None:
            axis.axhline(threshold, color='#176fba', linestyle='--', label='Acceptance ' + ('<=' if direction == 'max' else '>=') + str(threshold))
            limit = max(limit, threshold * 1.2)
        if key in ('f1_10mm', 'hidden_recall_10mm', 'precision_10mm'):
            limit = 1.
        axis.set_ylim(0, max(limit, .1))
        if threshold is not None or key == 'f1_10mm':
            axis.legend(fontsize=8)
    figure.suptitle('Fixed RGB-D replay: PCN baseline and third-party AP-PS55 checkpoint\nMedian over 20 observations / 8 boxes; green is the predeclared primary candidate', fontsize=13)
    figure.supxlabel('Input IDs, sampling seed and acceptance criteria unchanged. PS55 is a different model bundle; no complete-shape alignment.', fontsize=9)
    figure.savefig(output / 'profiles_metrics.png', dpi=150)
    plt.close(figure)

    baseline = records[env['baseline']]
    primary = records[env['primary_variant']]
    by_id = {v['id']: v for v in primary['objects']}
    selected = baseline['objects'][:4]
    figure = plt.figure(figsize=(13, 3.1 * len(selected)), layout='constrained')
    for row, record in enumerate(selected):
        with np.load(root / ('predictions-' + env['baseline']) / 'predictions' / record['output_file'], allow_pickle=False) as packet:
            observed, before = packet['observed'], packet['predicted']
        candidate = by_id[record['id']]
        with np.load(root / ('predictions-' + env['primary_variant']) / 'predictions' / candidate['output_file'], allow_pickle=False) as packet:
            np.testing.assert_array_equal(observed, packet['observed'])
            after = packet['predicted']
        case = int(record['capture'].split('-')[1])
        with np.load(truth_root / f'case-{case:04d}.npz', allow_pickle=False) as packet:
            chosen = record['assigned_box']
            gt = sample_box(packet['object_centers'][chosen], packet['object_sizes'][chosen], packet['object_rotations'][chosen], count=3000)
        clouds = [observed, before, after, gt]
        points = np.concatenate(clouds)
        center = (points.min(0) + points.max(0)) / 2
        radius = max(np.ptp(points, axis=0).max() / 2, .01) * 1.08
        for column, (cloud, color) in enumerate(zip(clouds, ['#176fba', '#7871b8', '#159b89', '#aaaaaa'])):
            axis = figure.add_subplot(len(selected), 4, row * 4 + column + 1, projection='3d')
            chosen_points = cloud[::max(1, len(cloud) // 2000)]
            axis.scatter(*chosen_points.T, s=1.3, c=color, alpha=.7, rasterized=True)
            axis.set(xlim=(center[0]-radius, center[0]+radius), ylim=(center[1]-radius, center[1]+radius), zlim=(center[2]-radius, center[2]+radius))
            axis.set_box_aspect((1, 1, 1))
            axis.view_init(elev=22, azim=-55)
            axis.set_xticks([])
            axis.set_yticks([])
            axis.set_zticks([])
            if row == 0:
                axis.set_title(['Observed', 'PCN baseline', 'AP-PS55 center-only Z-up', 'Private truth: evaluation only'][column], fontsize=10)
            if column == 0:
                axis.text2D(.02, .02, f"Capture {case}; component {record['component']}", transform=axis.transAxes, fontsize=8)
            if column in (1, 2):
                item = record if column == 1 else candidate
                axis.text2D(.02, .02, f"p95 {item['surface_p95_mm']:.1f} mm; F1 {item['f1_10mm']:.2f}", transform=axis.transAxes, fontsize=8)
    figure.suptitle('First four observations in capture order; predeclared primary candidate\nRaw model outputs, shared world-coordinate bounds; observations stored separately', fontsize=13)
    figure.supxlabel('No truth pose/scale is supplied to inference. This is a box-fixture trial, not general-object or grasp validation.', fontsize=9)
    figure.savefig(output / 'profiles_clouds.png', dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('root', 'truth', 'output'):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    plot(args.root, args.truth, args.output)


if __name__ == '__main__':
    main()
