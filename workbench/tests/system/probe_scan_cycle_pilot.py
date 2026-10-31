"""Pilot-only diagnostics around real scan feedback/control methods."""
from __future__ import annotations

import argparse
from collections import deque
import faulthandler
import importlib.util
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--periodic-stacks',action='store_true')
    args, remaining = parser.parse_known_args()
    for sibling in ('elesim_sim','elesim_ui'):
        assert importlib.util.find_spec(sibling) is None, sibling
    args.evidence.mkdir(parents=True,exist_ok=True)
    crash = (args.evidence/'native-crash.log').open('w')
    faulthandler.enable(file=crash,all_threads=True)
    from elesim_pilot.main import main as run_pilot
    from elesim_pilot.reconstruction.live_scan import LiveScan
    args.evidence.mkdir(parents=True,exist_ok=True)
    events = deque(maxlen=200)
    evidence = dict(sibling_imports_absent=True,errors=[])
    original_accept, original_checked = LiveScan.accept_telemetry, LiveScan._checked_feedback
    original_start, original_tick = LiveScan.start, LiveScan.tick
    stack = (args.evidence/'stacks.log').open('w')
    tracing = [False]

    def save():
        evidence['events'] = list(events)
        try:
            (args.evidence/'pilot.json').write_text(json.dumps(evidence,indent=2,allow_nan=False)+'\n')
        except (OSError,ValueError) as exc:
            print('SCAN_DIAGNOSTIC_ERROR',str(exc),flush=True)

    def accept(scan,payload,**source):
        original_accept(scan,payload,**source)
        if tracing[0]:
            events.append(dict(at=time.time(),kind='telemetry',source_mono=payload.get('measurement_monotonic_s'),
                source_wall=payload.get('measurement_time_s'),step=payload.get('sim_step_count')))

    def checked(scan):
        try:
            return original_checked(scan)
        except RuntimeError as exc:
            if tracing[0]:
                feedback = scan._feedback
                evidence['errors'].append(dict(at=time.time(),error=str(exc),phase=scan._phase,
                    feedback_age_s=None if feedback is None else scan.clock()-feedback['received'],
                    source_wall=None if feedback is None else feedback['wall']))
                save()
            raise

    def start(scan,*values,**kwargs):
        events.clear()
        evidence['started'] = time.time()
        tracing[0] = True
        if args.periodic_stacks:
            faulthandler.dump_traceback_later(1.,repeat=True,file=stack)
        return original_start(scan,*values,**kwargs)

    def tick(scan):
        original_tick(scan)
        if tracing[0] and not scan.active:
            faulthandler.cancel_dump_traceback_later()
            tracing[0] = False
            evidence['finished'] = time.time()
            try:
                evidence['status'] = scan.status()
            except Exception as exc:
                evidence['diagnostic_error'] = str(exc)
            save()

    sys.argv = [sys.argv[0],*remaining]
    with patch.object(LiveScan,'accept_telemetry',accept),patch.object(LiveScan,'_checked_feedback',checked),\
            patch.object(LiveScan,'start',start),patch.object(LiveScan,'tick',tick):
        run_pilot()


if __name__ == '__main__':
    main()
