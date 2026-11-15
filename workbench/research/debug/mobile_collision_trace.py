"""Run Pilot with an observation-only dump of a rejected mobile arm return.

Use only in an explicitly labelled development diagnosis, with the ordinary
Pilot endpoint stopped. Certificates, sensor filtering and commands are unchanged.
The NPZ is data, never executable source. This is not installed acceptance.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    options,arguments=parser.parse_known_args()
    import numpy as np
    from elesim_pilot.mobile_manipulation.runtime import SimPorts,ArmPathBlocked
    original_prepare=SimPorts._arm_prepare
    original_return=SimPorts._return_prepare
    attempts=deque(maxlen=32)

    def prepare(self,waypoints):
        try:return original_prepare(self,waypoints)
        except ArmPathBlocked as exc:
            attempts.append(dict(waypoints=np.asarray(waypoints).tolist(),reason=str(exc)))
            raise

    def returning(self,goal):
        attempts.clear()
        try:return original_return(self,goal)
        except ArmPathBlocked:
            try:
                snapshot=self.feedback.snapshot(self.route()[:2])
                options.output.parent.mkdir(parents=True,exist_ok=True)
                np.savez_compressed(options.output,cloud=self.collision.cloud.points(),
                    base=self.collision.base,q=snapshot['joint']['q'],goal=goal,
                    history=np.asarray(self.history),attempts=json.dumps(list(attempts)),
                    body_pos=snapshot['base']['pos'],body_rpy=snapshot['base']['rpy'],
                    diagnostic_only=True)
                print('MOBILE_RETURN_TRACE '+str(options.output),flush=True)
            except Exception as exc:
                # Diagnostic I/O cannot replace or suppress the original fault.
                print('MOBILE_RETURN_TRACE_ERROR '+repr(exc),flush=True)
            raise

    SimPorts._arm_prepare=prepare
    SimPorts._return_prepare=returning
    from elesim_pilot.main import main as pilot_main
    sys.argv=['elesim-pilot',*arguments]
    return pilot_main()


if __name__=='__main__':raise SystemExit(main())
