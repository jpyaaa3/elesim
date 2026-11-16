"""Run Pilot with an observation-only dump of a rejected mobile arm path.

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
    original_map=SimPorts._map_search_prepare
    attempts=deque(maxlen=32)

    def prepare(self,waypoints):
        try:return original_prepare(self,waypoints)
        except ArmPathBlocked as exc:
            attempts.append(dict(waypoints=np.asarray(waypoints).tolist(),reason=str(exc)))
            raise

    def dump(self,goal,kind):
        try:
            snapshot=self.feedback.snapshot(self.route()[:2])
            options.output.parent.mkdir(parents=True,exist_ok=True)
            extra={}
            if kind=='floor':
                intr=self.mapping_intrinsics
                extra=dict(camera_q=self.camera_q,camera_poses=self.camera_cache,
                    footprint=self.footprint,free=np.array(sorted(self.grid.free)),
                    blocked=np.array(sorted(self.grid.blocked)),resolution=self.grid.resolution,
                    map_origin=self.grid.origin,
                    mapping_margin_m=self.mapping_margin_m,mapping_views=np.asarray(self.mapping_views),
                    intrinsics=np.array([intr.fx,intr.fy,intr.cx,intr.cy,intr.width,intr.height]))
            np.savez_compressed(options.output,cloud=self.collision.cloud.points(),
                    base=self.collision.base,q=snapshot['joint']['q'],goal=goal,
                    history=np.asarray(self.history),attempts=json.dumps(list(attempts)),
                    body_pos=snapshot['base']['pos'],body_rpy=snapshot['base']['rpy'],
                    diagnostic_only=True,kind=kind,**extra)
            print('MOBILE_RETURN_TRACE '+str(options.output),flush=True)
        except Exception as exc:
            # Diagnostic I/O cannot replace or suppress the original fault.
            print('MOBILE_RETURN_TRACE_ERROR '+repr(exc),flush=True)

    def returning(self,goal):
        attempts.clear()
        try:return original_return(self,goal)
        except ArmPathBlocked:
            dump(self,goal,'return')
            raise

    def mapping(self,index):
        attempts.clear()
        try:return original_map(self,index)
        except RuntimeError:
            dump(self,getattr(self,'mapping_goal_pose',np.zeros(3)),'floor')
            raise

    SimPorts._arm_prepare=prepare
    SimPorts._return_prepare=returning
    SimPorts._map_search_prepare=mapping
    from elesim_pilot.main import main as pilot_main
    sys.argv=['elesim-pilot',*arguments]
    return pilot_main()


if __name__=='__main__':raise SystemExit(main())
