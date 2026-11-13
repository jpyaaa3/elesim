"""Compact model and unchanged-task comparison from preserved V2 evidence."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

from .v2_geometry import save, sha


def summarize_run(root):
    summary = root/'summary.json'
    batch = json.loads(summary.read_text()) if summary.exists() else {}
    process_times = {row['case']: row['elapsed_s'] for row in batch.get('cases', [])}
    rows = []
    for case in ('case_01', 'case_02', 'case_03'):
        path = root / case / 'completion.json'
        if not path.exists():
            rows.append(dict(case=case, status='incomplete'))
            continue
        result = json.loads(path.read_text())
        physics = result.get('physics_candidates', [])
        planner = result.get('planner_result', {})
        reasons = Counter(reason for candidate in physics for reason in candidate['report']['reasons'])
        calls = planner.get('candidates', [])
        winner = next((candidate['report'] for candidate in physics
                       if candidate['candidate'] == result.get('winner')), None)
        rows.append(dict(
            case=case, status=result['status'], elapsed_s=result['elapsed_s'],
            process_wall_s=process_times.get(case),
            planner_process_s=result.get('planner_process_s'),
            planner_setup_s=planner.get('setup_s'),
            planner_initialization_s=planner.get('planner_initialization_s'),
            planning_call_s=[call['planning_s'] for call in calls],
            graph_success_calls=sum(bool(call.get('graph_success')) for call in calls),
            endpoints=planner.get('endpoints'),
            start_offset_used=planner.get('start_offset_used'),
            authored_waypoints=planner.get('authored_waypoints'),
            planner_validation=[call.get('validation') for call in calls],
            physics_candidates=len(physics),
            physical_home_completions=sum(bool(c['report']['completed']) for c in physics),
            physical_successes=sum(bool(c['report']['success']) for c in physics),
            rejection_reason_counts=dict(reasons), winner=result.get('winner'),
            winner_metrics=({key: winner[key] for key in
                             ('duration_s', 'motion_group_motor_deg', 'peak_xy_m', 'peak_tilt_rad', 'home_error')}
                            if winner is not None else None),
            fresh_replays=result.get('replays'), error=result.get('error', planner.get('error')),
            completion_sha256=sha(path)))
    provenance_path = root/'case_01/provenance.json'
    provenance = json.loads(provenance_path.read_text()) if provenance_path.exists() else {}
    case_sources = [json.loads(path.read_text()).get('source_sha256')
                    for path in sorted(root.glob('case_*/provenance.json'))]
    return dict(root=str(root), cases=rows,
                model=provenance.get('model'), model_metadata_sha256=provenance.get('model_sha256'),
                gpu_uuid=provenance.get('gpu_uuid'), planner_mode=provenance.get('planner_mode'),
                native_profile=provenance.get('native_profile', 'default'),
                release_prefix=provenance.get('release_prefix', False),
                authored_waypoints_allowed=provenance.get('authored_waypoints_allowed'),
                automatic_start_offset_allowed=provenance.get('automatic_start_offset_allowed'),
                source_sha256=provenance.get('source_sha256'),
                case_source_consistent=bool(case_sources) and all(source == case_sources[0] for source in case_sources),
                batch_elapsed_s=batch.get('elapsed_s'),
                verified_cases=sum(r['status'] == 'reverse_path_verified' for r in rows),
                complete=all(r['status'] not in ('incomplete', 'running') for r in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, action='append', required=True)
    parser.add_argument('--extra-case', type=Path, action='append', default=[],
                        help='An additional single-case trial, reported separately from complete batches')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--ga-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu-sharing-note', help='Observed concurrent GPU work; timings cannot establish isolated speedup')
    args = parser.parse_args()
    metadata = json.loads((args.model/'metadata.json').read_text())
    ga = json.loads((args.ga_root/'summary.json').read_text())
    model = dict(root=str(args.model), curobo_version=metadata['curobo_version'],
                 curobo_commit=metadata['curobo_commit'], spheres=metadata['sphere_count'],
                 fitting_elapsed_s=metadata['elapsed_s'],
                 excluded_collision_links=metadata['excluded_collision_links'],
                 added_task_collision_links=metadata.get('added_task_collision_links', []),
                 sampling=metadata['sampling'], containment_certified=metadata['containment_certified'],
                 metadata_sha256=sha(args.model/'metadata.json'),
                 links={name: dict(spheres=link['final_spheres'],
                                  min_radius_mm=link['min_radius_mm'],
                                  max_radius_mm=link['max_radius_mm'],
                                  surface_gap_p95_mm=link['metrics']['surface_gap_p95']*1000,
                                  sampled_protrusion_fraction=link['metrics']['protrusion'])
                        for name, link in metadata['links'].items()})
    ga_rows = [dict(index=row['index'], status=row['status'], elapsed_s=row['elapsed_s'])
               for row in ga['cases']]
    extras = []
    for case in args.extra_case:
        trial = summarize_run(case.parent)
        row = next(row for row in trial['cases'] if row['case'] == case.name)
        extras.append(dict(root=str(case.parent), release_prefix=trial['release_prefix'],
                           planner_mode=trial['planner_mode'], native_profile=trial['native_profile'], **row))
    report = dict(schema='hug-curobo-v2-comparison', version=1, model=model,
                  gpu_exclusivity_verified=False, successful_task_speedup_claimed=False,
                  observed_gpu_sharing=args.gpu_sharing_note,
                  ga=dict(root=str(args.ga_root), cases=ga_rows,
                          case_wall_s_sum=sum(row['elapsed_s'] for row in ga_rows)),
                  runs=[summarize_run(root) for root in args.run],
                  extra_case_trials=extras,
                  measurement_notes=[
                      'GA wall times include its original postprocessing/video; V2 batch excludes video.',
                      'RRT mode uses eight parallel graph proposals. Full mode uses native cuRobo seed defaults. Both use one physical lane; GA used 128.',
                      'Initial setup, model fitting and per-call planning have separate timing scopes.',
                      'Compilation caches may be reused from earlier trials; no cold-start timing is claimed.',
                      'Failed runs do not establish a successful-task speedup.',
                      'GPU exclusivity was not monitored continuously; observed concurrent work is recorded separately.',
                      'Cylinder and floor are planned in V2; Go2 and the full contact task remain Genesis checks.',
                      'RRT polylines use task-rate timing without spline validation. Full mode requires native spline success, then uses only uniform slowdown.',
                      'Only three fresh successful admissions/replays establish reverse_path_verified.'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    save(args.output, report)
    print(json.dumps(dict(output=str(args.output), model_spheres=model['spheres'],
                          runs=[dict(root=r['root'], verified_cases=r['verified_cases'],
                                     complete=r['complete']) for r in report['runs']]), indent=2))
    complete = all(r['complete'] for r in report['runs']) and all(
        row['status'] not in ('incomplete', 'running') for row in extras)
    return 0 if complete else 2


if __name__ == '__main__':
    raise SystemExit(main())
