"""Create an analytic placement fixture using the actual transport-arm envelope.

Floor and target below are declared fixture truth, not RGB-D acceptance evidence.
The generated JSON is consumed only by the independent Sim physics probe.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root',type=Path,default=Path('payload/data'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    from elesim_pilot.reconstruction.hug import HugModel
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    from elesim_pilot.reconstruction.scan_collision import ScanCollision
    from elesim_pilot.mobile_manipulation.local_map import transport_footprint,LocalMap
    from elesim_pilot.mobile_manipulation.base_planner import BasePlanner
    from elesim_pilot.mobile_manipulation.contracts import Settings,TargetEstimate
    from elesim_sim.simulation.model_frames import neutral_frame
    model=HugModel(a.data_root/'policies/hug-d100')
    arm=ArmSelfCollision(a.data_root/'models/planning/zed-mini',joint_uncertainty=[.001,.015,.015,.015])
    urdf=a.data_root/'models/assemblies/zed-mini/robot.physics.urdf'
    base=neutral_frame(urdf,'plate');node=neutral_frame(urdf,'node0')
    collision=ScanCollision(arm,base)
    foot=transport_footprint(collision._centers(model.meta['home']),collision._radii)
    grid=LocalMap()
    grid.observe(np.array([[x,y,0.] for x in np.arange(-1.2,1.8,.025) for y in np.arange(-1.2,1.2,.025)]))
    observed=np.array([[.7+.05*np.cos(t),.15+.05*np.sin(t),z]
        for t in np.linspace(0,2*np.pi,80) for z in (.1,.3,.5)])
    grid.observe(observed)
    target=TargetEstimate('target',np.array([.7,.15,.55]),np.array([0.,0.,1.]),.1,.003,0.,1,('fixture',),observed)
    goal=BasePlanner(model,grid,foot,node,.32,Settings()).grasp(target,np.zeros(3))
    result=dict(goal.report,scope='analytic placement fixture; no RGB-D or physics evidence',
        base_pose=goal.pose.tolist(),footprint_points=len(foot),
        source_manifest_sha256=hashlib.sha256((model.root/'manifest.json').read_bytes()).hexdigest())
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
