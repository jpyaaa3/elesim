"""Independent PPO lineages with bounded mutation and a behavioural archive.

All policies step one physical batch together; buffers and optimizers never mix.
This is population-based search, not an additional task reward.
"""
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import numpy as np


VERSION = 4


def quality(report):
    """Physical milestones outrank posture/return; unsafe episodes cannot advance."""
    return tuple(float(report.get(k, 0.)) for k in ('success', 'hold', 'lift', 'grasp', 'contact', 'approach', 'efficiency', 'safe'))


def next_variation(state):
    """Advance by actual variation attempts, independent of stagnation cadence."""
    attempt = state.get('variation_attempts', 0)
    state['variation_attempts'] = attempt+1
    return ('exploration', 'weights', 'rewind')[attempt % 3]


def eligible(report):
    return report['safe'] >= 1.-1e-6 and (report['contact'] > 0 or report.get('approach', 0.) >= .05)


def cell(report):
    # Four normalized behaviour coordinates, five bins each (at most 625 niches).
    return tuple(int(np.clip(math.floor(float(x)*5), 0, 4)) for x in report['descriptor'])


def archive_key(report):
    return ','.join(map(str, cell(report)))


def archive_improves(report, archive):
    if not eligible(report):
        return False
    incumbent = archive.get(archive_key(report))
    return incumbent is None or quality(report) > tuple(incumbent['quality'])


def admit_child(candidate, parent, archive):
    # A new niche may be weaker than its parent. It must still pass the same
    # physical safety gate; novelty cannot excuse forbidden contacts.
    return (candidate['safe'] >= parent['safe'] and
            (quality(candidate) >= quality(parent) or archive_improves(candidate, archive)))


def select_replacement(reports, stale, patience):
    """Protect champion and a distinct useful strategy; replace at most one."""
    best = max(range(len(reports)), key=lambda i: quality(reports[i]))
    protected = {best}
    alternatives = [i for i, r in enumerate(reports)
                    if i != best and eligible(r) and cell(r) != cell(reports[best])]
    if alternatives:
        protected.add(max(alternatives, key=lambda i: quality(reports[i])))
    victims = [i for i in range(len(reports)) if i not in protected and stale[i] >= patience]
    return best, min(victims, key=lambda i: quality(reports[i])) if victims else None


def set_exploration(actor, floor, reset=False):
    """Pinned rsl_rl 5.4 GaussianDistribution; range is not in state_dict."""
    import torch
    distribution = actor.distribution
    if distribution is None or not hasattr(distribution, 'std_range'):
        raise ValueError('Population requires the pinned state-independent Gaussian policy')
    distribution.std_range = [floor, 1.0]
    distribution.log_std_range = [math.log(floor), 0.0]
    if reset:
        with torch.no_grad():
            if distribution.std_type == 'scalar':
                distribution.std_param.fill_(max(.5, floor))
            else:
                distribution.log_std_param.fill_(math.log(max(.5, floor)))


def mutate(actor, probes, seed, maximum=.12):
    """Accept only finite, small deterministic action changes on visited states."""
    import torch
    original = copy.deepcopy(actor.state_dict())
    generator = torch.Generator(device=next(actor.parameters()).device).manual_seed(seed)
    with torch.inference_mode():
        baseline = actor(probes).clone()
    parameters = list(actor.mlp.parameters())
    noise = [torch.randn(p.shape, generator=generator, device=p.device, dtype=p.dtype)
             / max(1, p.shape[-1])**.5 for p in parameters]
    scale = .01
    for _ in range(8):
        actor.load_state_dict(original)
        with torch.no_grad():
            for parameter, direction in zip(parameters, noise):
                parameter.add_(direction * scale)
        with torch.inference_mode():
            output = actor(probes)
            change = float((output - baseline).abs().max())
        if math.isfinite(change) and 1e-6 < change <= maximum:
            return {'accepted': True, 'max_action_change': change, 'scale': scale}
        scale *= .5
    actor.load_state_dict(original)
    return {'accepted': False, 'max_action_change': 0., 'scale': 0.}


def evaluate(env, actor, directory=None):
    """Evaluate all member policies in one physics batch; keep per-policy reports.

    Nominal replicas are not independent robustness trials. Completed lanes stop
    receiving credit, and each representative trajectory ends at its own done.
    A single actor still returns one report/probe bank for offspring admission.
    """
    import torch
    from tensordict import TensorDict
    from elesim_sim.rl.trajectory import Trajectory
    from .configure import CHANNELS
    from .population_batch import partitions
    multiple=isinstance(actor,(tuple,list))
    actors=list(actor) if multiple else [actor]
    groups=partitions(env.num_envs,len(actors))
    directories=list(directory) if multiple and directory is not None else [directory]*len(actors)
    if len(directories)!=len(actors):raise ValueError('One evaluation directory per policy is required')
    n=env.num_envs
    old_reset=env.auto_reset
    env.auto_reset=False
    for policy in actors:policy.eval()
    probes=[[] for _ in actors]
    active=torch.ones(n,dtype=torch.bool,device=env.device)
    bad=torch.zeros_like(active);contact=torch.zeros_like(active)
    grasp=torch.zeros_like(active);success=torch.zeros_like(active)
    lift=torch.zeros(n,device=env.device);hold=torch.zeros_like(lift)
    first=torch.zeros((n,4),device=env.device);closest_pose=torch.zeros_like(first)
    hi=torch.full_like(first,-float('inf'))
    first_ids=torch.tensor([group.start for group in groups],device=env.device)
    path_lengths=torch.ones(len(actors),device=env.device,dtype=torch.long)
    from .control import command_bounds,advance_command
    lower,upper=command_bounds(env.s)
    lower=torch.as_tensor(lower,device=env.device)
    width=torch.as_tensor(upper,device=env.device)-lower
    stalled=torch.zeros(n,device=env.device,dtype=torch.long)
    active_steps=torch.zeros_like(stalled)
    try:
        with torch.inference_mode():
            obs,_=env.reset()
            initial_distance=env.relative_object().norm(dim=-1).clamp_min(1e-6)
            closest_distance=initial_distance.clone()
            initial=env.start.cpu().numpy().copy()
            env.record_env_indices=first_ids
            env.record=[np.tile(initial,(len(actors),1))]
            for step in range(env.max_episode_length):
                if step%5==0:
                    for bank,group in zip(probes,groups):bank.append(obs['policy'][group][:min(8,group.stop-group.start)].clone())
                commands=torch.cat([policy(obs[group]) for policy,group in zip(actors,groups)],dim=0)
                commands[~active]=0
                start=env.mapper.waypoint
                requested=(commands.abs()>.1).any(-1)
                blocked=(advance_command(start,commands,env.s,torch)-start).abs().amax(-1)<1e-7
                stalled+=active&requested&blocked;active_steps+=active
                obs,_,done,_=env.step(commands)
                outcome=env.outcome()
                bad|=active&(env.fault|(env.macro_wrong>0))
                touched=(env.last_touch>0).any(-1)&active&~bad
                normalized=((env.actual()-lower)/width).float()
                distance=env.relative_object().norm(dim=-1)
                improved=active&~bad&(distance<closest_distance)
                closest_pose[improved]=normalized[improved]
                closest_distance=torch.where(improved,distance,closest_distance)
                first[touched&~contact]=normalized[touched&~contact]
                contact|=touched
                grasp|=active&outcome['grasp_confirmed']&~bad
                success|=active&outcome['success']&~bad
                lift=torch.maximum(lift,torch.where(active,env.ledger.best_lift,0))
                hold=torch.maximum(hold,torch.where(active,env.ledger.best_hold,0))
                hi=torch.where(active[:,None],torch.maximum(hi,normalized),hi)
                path_lengths=torch.where(active[first_ids],len(env.record),path_lengths)
                active&=~done
                if not bool(active.any()):break
            safe=~bad
            reference_pose=torch.where(contact[:,None],first,closest_pose)
            descriptor=torch.cat((hi[:,:1],reference_pose[:,1:]),dim=1)
            reports=[];banks=[]
            recorded=np.asarray(env.record)
            for index,(group,output) in enumerate(zip(groups,directories)):
                def mean(value):return float(value[group].float().mean())
                valid=contact[group]&safe[group]
                selected=descriptor[group][valid] if bool(valid.any()) else descriptor[group]
                report=dict(success=mean(success&safe),hold=mean(hold*safe),lift=mean(lift*safe),
                    grasp=mean(grasp&safe),contact=mean(contact&safe),safe=mean(safe),
                    approach=mean(((initial_distance-closest_distance)/initial_distance)*safe),
                    descriptor=selected.mean(0).cpu().tolist(),nominal_only=True,
                    evaluated_lanes=group.stop-group.start,evaluation_env_steps=(step+1)*(group.stop-group.start),
                    saturation_stall_fraction=float(stalled[group].sum()/active_steps[group].sum().clamp_min(1)))
                totals=getattr(env.ledger,'totals',{})
                if totals:
                    report['efficiency']=mean(sum(totals[name] for name in ('time','workspace','action')))
                    report['cost_components']={name:mean(totals[name]) for name in ('time','workspace','action')}
                if output is not None:
                    length=int(path_lengths[index]);path=recorded[:length,index]
                    metadata=dict(env.metadata(),duration_s=(length-1)*env.s['dt'])
                    if hasattr(env,'grasp_tick'):
                        tick=int(env.grasp_tick[group.start])
                        metadata['grasp_time_s']=None if tick<0 else tick*env.s['dt']
                    Trajectory(np.arange(length)*env.s['dt'],path,CHANNELS,metadata,
                        {'population_evaluation':report,'physics_validated':False}).save(Path(output)/'evaluation.npz')
                bank=torch.cat(probes[index],dim=0)
                reports.append(report);banks.append(TensorDict({'policy':bank},batch_size=[len(bank)]))
            return (reports,banks) if multiple else (reports[0],banks[0])
    finally:
        env.auto_reset=old_reset;env.record=None;env.record_env_indices=None


def run(args, experiment, case):
    """4 x 128 by default: same rollout budget as 512 x one policy per update.

    Members control disjoint lanes in one simultaneous physics batch. Resume
    starts at the last complete generation and refuses partial output rather
    than overwriting evidence.
    """
    import torch
    from .experiment import create_env, runner
    from .configure import write_json, sha
    from .policy import export_policy
    from .population_batch import PolicyEnvironments,partitions,learn_population,restore_learner,transfer_state
    from workbench.research.experiments.wrap_grasp.train import set_seed
    members = args.population_size
    if (members < 3 or args.num_envs % members or args.num_envs < members
            or args.iterations < 1 or args.generation_updates < 1 or args.stagnation_generations < 1):
        raise ValueError('Require >=3 policies, divisible positive environment budget and positive update/patience counts')
    directory = args.runs/case['case_id']
    config = dict(schema='c-hug-population', version=VERSION, experiment=experiment,
                  case_id=case['case_id'], num_envs=args.num_envs, members=members,
                  generation_updates=args.generation_updates,
                  patience=args.stagnation_generations, schedule='simultaneous-shared-physics',
                  reward_contract=experiment['simulation_context']['reward_contract'])
    if args.resume and getattr(args,'population_from',None):
        raise ValueError('Choose resume or an explicit population transfer')
    if args.resume:
        if json.loads((directory/'population.json').read_text()) != config:
            raise ValueError('Population resume provenance/configuration differs')
        state = json.loads((directory/'population-state.json').read_text())
    else:
        directory.mkdir(parents=True, exist_ok=False)
        write_json(directory/'population.json', config)
        # Standard exports/collect can read the selected champion checkpoint.
        write_json(directory/'run.json', dict(experiment=experiment, case_id=case['case_id'], num_envs=args.num_envs))
        state = dict(generation=0, updates=0, stale=[0]*members, best=[None]*members,
                     archive={}, checkpoints=[None]*members, events=[])
        if getattr(args,'population_from',None):
            state=transfer_state(args.population_from,directory,experiment,config)
    base_seed = experiment['settings']['seed']
    env = create_env(experiment, case, directory, args, args.num_envs, True)
    views=[PolicyEnvironments(env,group) for group in partitions(args.num_envs,members)]
    floors = state.setdefault('exploration_floors', np.linspace(.08, .35, members).tolist())
    policies = []
    for i in range(members):
        set_seed(base_seed+i*1009)
        policies.append(runner(views[i], None))
        if state['checkpoints'][i]:
            restore_learner(policies[-1],directory/state['checkpoints'][i])
        set_exploration(policies[-1].alg.get_policy(), floors[i], reset=not bool(state['checkpoints'][i]))
    if state.get('transfer') and not args.resume:
        champion_runner=runner(views[0],None)
        champion_runner.load(str(directory/state['champion_checkpoint']))
        export_policy(champion_runner.alg.get_policy(),
            (directory/state['champion_checkpoint']).with_name('policy.npz'),
            dict(env.metadata(),checkpoint_sha256=sha(directory/state['champion_checkpoint']),transfer=state['transfer']))
        temporary=directory/'population-state.pending.json'
        write_json(temporary,state);temporary.replace(directory/'population-state.json')
        del champion_runner
    target_updates = state['updates'] + args.iterations
    def progress(**fields):
        record = dict(schema='c-hug-population-progress', updated_utc=datetime.now(timezone.utc).isoformat(),
                      completed_generations=state['generation'], completed_updates=state['updates'],
                      target_updates=target_updates, **fields)
        temporary = directory/'population-progress.pending.json'
        write_json(temporary, record)
        temporary.replace(directory/'population-progress.json')
    while state['updates'] < target_updates:
        generation = state['generation']
        output = directory/f'generation-{generation:04d}'
        output.mkdir(exist_ok=False)
        count = min(args.generation_updates, target_updates-state['updates'])
        member_dirs=[output/f'member-{i:02d}' for i in range(members)]
        for member in member_dirs:member.mkdir()
        set_seed(base_seed+generation*100003)
        progress(phase='ppo',generation=generation,active_members=members,physical_num_envs=env.num_envs,
                 generation_updates_done=0,member_updates=count)
        def on_update(done,elapsed):
            progress(phase='ppo',generation=generation,active_members=members,physical_num_envs=env.num_envs,
                     generation_updates_done=done,member_updates=count,training_elapsed_s=elapsed)
        elapsed=learn_population(env,policies,count,on_update)
        for learner,member in zip(policies,member_dirs):
            learner.current_learning_iteration-=1
            learner.save(str(member/'evaluated.pt'))
            learner.current_learning_iteration+=1
        progress(phase='evaluation',generation=generation,active_members=members,training_elapsed_s=elapsed)
        reports,probes=evaluate(env,[p.alg.get_policy() for p in policies],member_dirs)
        for i,(learner,member,report) in enumerate(zip(policies,member_dirs,reports)):
            report.update(training_elapsed_s=elapsed,training_wall_time_shared=True,
                          training_env_steps=count*views[i].num_envs*16)
            write_json(member/'evaluation.json',report)
            export_policy(learner.alg.get_policy(),member/'policy.npz',
                          dict(env.metadata(),checkpoint_sha256=sha(member/'evaluated.pt')))
            q = quality(report)
            if state['best'][i] is None or q > tuple(state['best'][i]):
                state['best'][i] = list(q); state['stale'][i] = 0
            else:
                state['stale'][i] += 1
            if archive_improves(report, state['archive']):
                state['archive'][archive_key(report)] = dict(quality=list(q), report=report,
                    checkpoint=str((member/'evaluated.pt').relative_to(directory)))
        champion, victim = select_replacement(reports, state['stale'], args.stagnation_generations)
        # Published final is always an evaluated policy, never an untested child.
        import shutil
        if state.get('champion_quality') is None or quality(reports[champion]) > tuple(state['champion_quality']):
            shutil.copyfile(output/f'member-{champion:02d}'/'evaluated.pt', directory/'model_final.pt')
            state['champion_quality'] = list(quality(reports[champion]))
            state['champion_checkpoint'] = str((output/f'member-{champion:02d}'/'evaluated.pt').relative_to(directory))
        event = dict(generation=generation, champion=champion, victim=victim, mutation=None)
        if victim is not None and state['updates']+count < target_updates:
            progress(phase='evolution', generation=generation, member=victim)
            rng = np.random.default_rng(base_seed+generation)
            mode = next_variation(state)
            parent = output/f'member-{champion:02d}'/'evaluated.pt'
            # Prefer archived diverse successful approaches on alternating generations.
            entries = list(state['archive'].values())
            if entries and generation % 2:
                parent = directory/entries[int(rng.integers(len(entries)))]['checkpoint']
            if mode == 'rewind' and generation:
                previous=state.get('previous_evaluated')
                parent=(directory/previous[victim] if previous else
                        directory/f'generation-{generation-1:04d}'/f'member-{victim:02d}'/'evaluated.pt')
            child = runner(views[victim], None)
            child.load(str(parent), load_cfg={'actor': True, 'critic': True,
                                             'optimizer': False, 'iteration': False})
            child.current_learning_iteration = state['updates']+count
            actor = child.alg.get_policy()
            child_floor = float(np.clip(floors[victim]*rng.choice([.8, 1.2]), .05, .5))
            set_exploration(actor, child_floor, reset=True)
            # Physical admission compares against the parent, not the weak victim.
            parent_runner = runner(views[victim], None)
            parent_runner.load(str(parent))
            parent_report, parent_probes = evaluate(env, parent_runner.alg.get_policy())
            mutation = mutate(actor, parent_probes, base_seed+generation) if mode == 'weights' else None
            offspring_dir = output/'offspring'
            offspring_dir.mkdir()
            candidate, _ = evaluate(env, actor, offspring_dir)
            child.save(str(offspring_dir/'evaluated.pt'))
            write_json(offspring_dir/'evaluation.json', candidate)
            accepted = (admit_child(candidate, parent_report, state['archive']) and
                        (mutation is None or mutation['accepted']))
            if accepted:
                policies[victim] = child
                floors[victim] = child_floor
                if archive_improves(candidate, state['archive']):
                    state['archive'][archive_key(candidate)] = dict(quality=list(quality(candidate)),
                        report=candidate, checkpoint=str((offspring_dir/'evaluated.pt').relative_to(directory)))
                state['stale'][victim] = 0
                state['best'][victim] = list(quality(candidate))
            state['evaluation_env_steps'] = state.get('evaluation_env_steps', 0) + parent_report['evaluation_env_steps'] + candidate['evaluation_env_steps']
            event.update(parent=str(parent.relative_to(directory)), mode=mode,
                         mutation=mutation, accepted=accepted, candidate=candidate, exploration_floor=child_floor)
        # Keep evaluated checkpoints immutable; the next-generation state may be a child.
        checkpoints = []
        for i, learner in enumerate(policies):
            path = output/f'member-{i:02d}'/'continuation.pt'
            # load() callers increment last-completed index.
            learner.current_learning_iteration -= 1
            learner.save(str(path))
            learner.current_learning_iteration += 1
            checkpoints.append(str(path.relative_to(directory)))
        state.update(generation=generation+1, updates=state['updates']+count, checkpoints=checkpoints)
        state['previous_evaluated']=[str((member/'evaluated.pt').relative_to(directory)) for member in member_dirs]
        state['events'].append(event)
        state['evaluation_env_steps'] = state.get('evaluation_env_steps', 0) + sum(r['evaluation_env_steps'] for r in reports)
        state['training_env_steps'] = state['updates']*args.num_envs*16
        write_json(output/'summary.json', dict(reports=reports,event=event,training_wall_s=elapsed,
            physical_num_envs=env.num_envs,schedule=config['schedule'],training_env_steps=state['training_env_steps']))
        temporary = directory/'population-state.pending.json'
        write_json(temporary, state)
        temporary.replace(directory/'population-state.json')
        progress(phase='generation-complete', generation=generation,
                 training_env_steps=state['training_env_steps'], evaluation_env_steps=state['evaluation_env_steps'])
        print(json.dumps(dict(generation=generation, reports=reports, event=event)), flush=True)
    progress(phase='finished', training_env_steps=state['training_env_steps'], evaluation_env_steps=state['evaluation_env_steps'])
    return 0
