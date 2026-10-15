"""Fixed-body airborne GO2 foot tracking through the production torque path.

No DDS, gait scheduling or MPC solve. All four legs use the same torque path;
only the selected leg's reference moves. The assembled model retains its arm.
"""
from __future__ import annotations
import argparse
from dataclasses import fields
import json
from pathlib import Path
from types import SimpleNamespace


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('config')
    p.add_argument('model_bundle')
    p.add_argument('--dt', type=float, default=.02)
    p.add_argument('--leg', choices=('FL', 'FR', 'RL', 'RR'), default='RL')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--legacy-reference', action='store_true',
                   help='reproduce the former qpos0 bug in this isolated scene only')
    a = p.parse_args()
    import numpy as np
    import genesis as gs
    from elesim_sim.runtime import load_app_config, _make_urdf_morph
    from elesim_sim.robot.go2.initial_pose import validate_initial_stand_pose
    from elesim_sim.robot.go2.mpc.config import Go2MpcConfig
    from elesim_sim.robot.go2.pympc_controller import PyMpcGenesisController
    from elesim_sim.simulation.genesis.utils import to_numpy_1d
    if not np.isfinite(a.dt) or a.dt <= 0:
        p.error('positive finite dt required')
    bundle = Path(a.model_bundle).resolve()
    cfg = load_app_config(a.config, mode='remote')
    defaults = Go2MpcConfig()
    lc = cfg.go2_locomotion_config
    values = {f.name: getattr(lc, 'mpc_' + f.name, getattr(lc, f.name, getattr(defaults, f.name)))
              for f in fields(defaults)}
    values['payload_enable'] = False  # body is fixed; no centroidal solve
    gs.init(backend=gs.gpu, logging_level='warning')
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=a.dt, substeps=cfg.sim_param.substeps,
                                                     gravity=(0, 0, -9.81)),
                     rigid_options=gs.options.RigidOptions(constraint_solver=gs.constraint_solver.Newton,
                                                          iterations=50, noslip_iterations=0),
                     show_viewer=False)
    entity = scene.add_entity(_make_urdf_morph(str(bundle / 'robot.urdf'), (0, 0, .6),
                                             (0, 0, 0), fixed=True, merge_fixed_links=False))
    validate_initial_stand_pose(entity)
    if a.legacy_reference:
        from elesim_sim.robot.go2.locomotion.kinematics import GO2_STAND_Q
        for name, value in GO2_STAND_Q.items():
            desc = entity.get_joint(name).desc
            desc.init_qpos = np.array([value], dtype=desc.init_qpos.dtype)
    scene.build()
    print('model_mass_kg', float(sum(to_numpy_1d(entity.get_links_mass()))), flush=True)
    c = PyMpcGenesisController(entity, dt=a.dt, config=Go2MpcConfig(**values),
                              go2_urdf_path=bundle / 'assets/go2/go2.urdf',
                              solver_factory=lambda: SimpleNamespace())
    entity.set_dofs_position(c._kin.ready_q, dofs_idx_local=c._leg_dof_idxs)
    c._set_torque_actuation()
    print('STIFFNESS', to_numpy_1d(entity.get_dofs_stiffness(dofs_idx_local=c._leg_dof_idxs)).tolist(), flush=True)
    c._last_contacts.fill(0)
    c._contact_gait.swing_progress.fill(1.)
    c._walk_started_s = 0.
    c._sim_time = 2.
    leg = ('FL', 'FR', 'RL', 'RR').index(a.leg)
    records = []
    origin = None
    previous = None
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('w') as out:
        for step in range(round(3 / a.dt)):
            t = step * a.dt
            q, dq = c._bridge.read_pin_q_dq(dt=a.dt)
            pin = c._pin
            c._joint_gravity = np.asarray(
                pin.computeGeneralizedGravity(c._model, c._data, q)
            )[6:18].copy()
            pin.forwardKinematics(c._model, c._data, q, dq)
            pin.updateFramePlacements(c._model, c._data)
            pin.computeJointJacobians(c._model, c._data, q)
            feet = np.array([c._data.oMf[f].translation for f in c._frames])
            velocity = np.array([pin.getFrameVelocity(c._model, c._data, f, pin.LOCAL_WORLD_ALIGNED).linear
                                 for f in c._frames])
            jac = np.array([pin.getFrameJacobian(c._model, c._data, f, pin.LOCAL_WORLD_ALIGNED)[:3,6+3*i:9+3*i]
                            for i, f in enumerate(c._frames)])
            if origin is None:
                origin = feet.copy()
                c._swing_starts = feet.copy()
            target = origin.copy()
            # One second hold, then lower the position target smoothly by 3 cm.
            # Feedforward velocity stays zero: isolate position feedback.
            s = np.clip(t-1, 0, 1)
            target[leg, 2] -= .03 * s*s*(3-2*s)
            desired_v = -.18*s*(1-s) if 1 < t < 2 else 0.
            c._touchdowns = target.copy()
            sample = SimpleNamespace(contacts=np.zeros((4,12)), footholds_world=target)
            tau = c._torques(sample, feet, velocity, jac, dq[6:18])
            entity.control_dofs_force(tau, dofs_idx_local=c._leg_dof_idxs)
            actual = np.array([to_numpy_1d(entity.get_link(n+'_foot').get_pos()) for n in ('FL','FR','RL','RR')])
            row = dict(t=t, target=target[leg].tolist(), reference_velocity_z=0., target_path_velocity_z=desired_v,
                       foot=feet[leg].tolist(), genesis_foot=actual[leg].tolist(),
                       velocity=velocity[leg].tolist(),
                       finite_difference_velocity=None if previous is None else ((feet[leg]-previous)/a.dt).tolist(),
                       engine_force_terms={name: getattr(entity._solver.dyn_state.dofs, name).to_numpy()[c._leg_dof_idxs, 0][3*leg:3*leg+3].tolist() for name in ('qf_applied','qf_bias','qf_passive')},
                       internal_force=to_numpy_1d(entity.get_dofs_force(dofs_idx_local=c._leg_dof_idxs))[3*leg:3*leg+3].tolist(),
                       contact_forces=to_numpy_1d(entity.get_links_net_contact_force()).tolist(),
                       control_force=to_numpy_1d(entity.get_dofs_control_force(dofs_idx_local=c._leg_dof_idxs))[3*leg:3*leg+3].tolist(),
                       pin_gravity=c._joint_gravity[3*leg:3*leg+3].tolist(),
                       jacobian=jac[leg].tolist(),
                       engine_qpos=to_numpy_1d(entity.get_qpos())[entity.get_joint(a.leg+'_calf_joint').qs_idx_local].tolist(),
                       q=q[7+leg*3:10+leg*3].tolist(), dq=dq[6+leg*3:9+leg*3].tolist(),
                       tau_raw=c._tau_raw[3*leg:3*leg+3].tolist(), tau=tau[3*leg:3*leg+3].tolist())
            out.write(json.dumps(row)+'\n')
            records.append(row)
            previous = feet[leg].copy()
            scene.step()
        errors = [np.linalg.norm(np.array(r['target'])-r['foot']) for r in records if r['t']>.5]
        hold = [r['foot'][2] for r in records if .5<r['t']<1]
        summary = dict(dt=a.dt, leg=a.leg, legacy_reference=a.legacy_reference, max_error_m=max(errors), hold_z_span_m=np.ptp(hold),
                       max_speed_mps=max(np.linalg.norm(r['velocity']) for r in records))
        print(json.dumps(summary), flush=True)
    gs.destroy()

if __name__ == '__main__':
    main()
