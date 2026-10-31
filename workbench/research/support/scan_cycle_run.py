"""Publish and run an isolated scan cycle with generated normal role images.

Run locally with ``upload``; the retained host driver then uses only generated
Compose and setup CLI primitives. No image collection or existing role restart.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import ipaddress
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile
import time


TASK = Path('/home/hckang/ws/elesim-scan-cycle-20261030')
PREFIX = TASK/'install'
DOCKER = ['docker','--context','default']


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def run_json(command):
    return json.loads(subprocess.check_output(command, text=True))


def compose(*extra):
    return [str(PREFIX/'bin/elesim-compose'), '--profile','tools',
        '-f',str(PREFIX/'containers/compose.yaml'),*extra]


def tool(args, label):
    override = TASK/'publisher.compose.yaml'
    prep = json.loads((TASK/'preparation.json').read_text())
    if not override.exists() or prep['image_ids']['tools'] not in override.read_text():
        mounts = [dict(type='bind',source=str(PREFIX),target=str(PREFIX)),
            dict(type='bind',source='/home/hckang/.local/share/elesim',
                target='/home/hckang/.local/share/elesim')]
        override.write_text('\n'.join(['services:','  tools:',
            '    image: '+json.dumps(prep['image_ids']['tools']),
            '    user: '+json.dumps(f'{os.getuid()}:{os.getgid()}'),
            '    working_dir: '+json.dumps(str(PREFIX)), '    read_only: true',
            '    network_mode: none', '    restart: "no"',
            '    environment: !override '+json.dumps({'PYTHONUNBUFFERED':'1',
                'ELESIM_OPERATOR_HOME':'/home/hckang','USER':'hckang','LOGNAME':'hckang'}),
            '    volumes: !override '+json.dumps(mounts), '    ports: !reset []',
            '    networks: !reset []', '    depends_on: !reset {}',
            "    tmpfs: !override ['/tmp:exec,mode=1777']"])+'\n')
    name = 'elesim-sandy-cycle-'+label+'-'+str(time.time_ns())
    command = compose('-f',str(override),'run','-T','--no-deps','--pull','never',
        '--name',name,'tools','elesim-setup','--state',str(PREFIX/'install-state.json'),*args)
    completed = subprocess.run(command, capture_output=True, text=True, timeout=300)
    (TASK/(name+'.log')).write_text(completed.stdout+'\n'+completed.stderr)
    if completed.returncode:
        raise RuntimeError(f'{label}: {completed.stdout[-2000:]} {completed.stderr[-2000:]}')
    return completed.stdout


def publish():
    prep = json.loads((TASK/'preparation.json').read_text())
    assert prep['complete'], prep
    evidence = dict(schema_version=1, install_uuid=prep['install_uuid'],
        project=prep['project'], platform='linux/amd64', roles={})
    for role in ('pilot','sim','ui'):
        info = run_json(DOCKER+['image','inspect',prep['images'][role]])[0]
        assert info['Id'] == prep['image_ids'][role]
        labels = info['Config']['Labels']
        evidence['roles'][role] = dict(image_reference=prep['images'][role], image_id=info['Id'],
            install_uuid=labels['io.elesim.install_uuid'],
            build_fingerprint=labels['io.elesim.build_fingerprint'],
            project=labels['com.docker.compose.project'])
    path = PREFIX/'maintenance/scan-cycle-build-evidence.json'
    write_json(path, evidence)
    revision = (TASK/'source-revision').read_text().strip()
    result = json.loads(tool(['release','publish','--source-revision','git-'+revision,
        '--snapshot',str(PREFIX/'containers/runtime-snapshot'),'--evidence',str(path)], 'publish'))
    write_json(TASK/'publication.json',result)
    print(json.dumps(result), flush=True)


def bind(source, target, *, read_only=False):
    return dict(type='bind',source=str(source),target=target,read_only=read_only)


def launch(attempt,math_threads=None,debug_allocator=False):
    assert 1 <= attempt <= 24
    root = TASK/f'cycle-{attempt:02d}'
    assert not root.exists(), 'use a new attempt; never recreate retained diagnostic containers'
    root.mkdir(mode=0o755)
    system = 'scan_cycle_20261030' if attempt == 1 else f'scan_cycle_20261030_{attempt:02d}'
    suffix = f'{attempt:02d}'
    endpoints = dict(sim='scan-sim-'+suffix,pilot='scan-pilot-'+suffix,ui='scan-ui-'+suffix)
    subnet = f'172.30.{230+attempt}.0/24'
    peers = [f'172.30.{230+attempt}.{value}' for value in (2,3,4)]
    network_ids = subprocess.check_output(DOCKER+['network','ls','-q'], text=True).split()
    for network in run_json(DOCKER+['network','inspect',*network_ids]):
        for config in network.get('IPAM',{}).get('Config',[]) or []:
            if config.get('Subnet'):
                assert not ipaddress.ip_network(subnet).overlaps(ipaddress.ip_network(config['Subnet'])), config
    publication = json.loads((TASK/'publication.json').read_text())
    arguments = ['instance','register','--system',system,'--release',publication['release_key'],
        '--domain-id','212','--discovery-mode','static','--interface','eth0',
        '--security-profile','trusted-network']
    for peer in peers:
        arguments.extend(('--static-peer',peer))
    for role, endpoint in endpoints.items():
        arguments.extend(('--endpoint',role+':'+endpoint))
    tool(arguments, 'register-'+suffix)
    instance_file = PREFIX/'containers/compose.instances.yaml'
    normalized = run_json(compose('-f',str(instance_file),'config','--format','json'))
    services = {value['labels']['io.elesim.role']:(key,value)
        for key,value in normalized['services'].items()
        if value.get('labels',{}).get('io.elesim.system_id') == system}
    assert set(services) == {'sim','pilot','ui'}, services.keys()
    exchange = root/'exchange'
    exchange.mkdir()
    lines = ['networks:','  scan_cycle:', '    internal: true',
        '    ipam:', '      config:', '        - subnet: '+subnet, 'services:']
    record = dict(system=system, endpoints=endpoints, peers=peers, subnet=subnet, services={},
        release=publication, source_revision=(TASK/'source-revision').read_text().strip(),
        started=time.time(), complete=False,probe_sha256={})
    record['pilot_math_threads'] = math_threads
    record['pilot_debug_allocator'] = debug_allocator
    record['pilot_periodic_stacks'] = False
    for index,role in enumerate(('sim','pilot','ui')):
        key,service = services[role]
        assert subprocess.run(DOCKER+['inspect',service['container_name']],capture_output=True).returncode != 0
        info = run_json(DOCKER+['image','inspect',service['image']])[0]
        record['services'][role] = dict(key=key,name=service['container_name'],
            image=service['image'],image_id=info['Id'])
        out = root/(role+'-output')
        out.mkdir()
        mounts = [v for v in service['volumes'] if v.get('target') != '/tmp/.X11-unix']
        mounts.append(bind(out,'/cycle-output'))
        lines.extend(('  '+key+':','    network_mode: !reset null',
            '    networks: !override '+json.dumps({'scan_cycle':{'ipv4_address':peers[index]}}),
            '    ports: !reset []','    restart: "no"','    depends_on: !reset {}'))
        environment = {}
        if role == 'pilot' and math_threads is not None:
            environment.update(OPENBLAS_NUM_THREADS=str(math_threads),OMP_NUM_THREADS=str(math_threads))
        if role == 'pilot' and debug_allocator:
            environment['PYTHONMALLOC'] = 'debug'
        if environment:
            lines.append('    environment: '+json.dumps(environment))
        if role in ('sim','pilot'):
            config = dict(schema_version=1, extends='/opt/elesim/config/'+
                ('app.installed.yaml' if role=='sim' else 'config.yaml'), mode='scan_cycle',
                profiles={'scan_cycle':dict(simulation={'runtime':dict(use_hardware=False,
                    use_go2=False,enable_viewer=False)}, robot={'arm':{'spawn':{
                        'xyz':[0.,0.,.8], 'euler_deg':[0.,0.,0.]}}})})
            profile = config['profiles']['scan_cycle']
            if role == 'sim':
                profile['simulation']['runtime'].update(scene_import_root='/exchange')
                profile['world'] = {'target':dict(enable=True, radius=.045,
                    color_rgba=[.85,.15,.15,1.], gravity=False), 'debug':{'markers':False}}
                # Seed only this new instance's private cache. The original
                # measurement cache remains untouched.
                cache = next(Path(v['source']) for v in mounts if v.get('target')=='/tmp/elesim-cache')
                seed = Path('/home/hckang/ws/elesim-genesis145-20261029/cache/objgpu2-small-obj')
                previous = TASK/f'cycle-{attempt-1:02d}'/'compose.resolved.json'
                if previous.exists():
                    old = json.loads(previous.read_text())
                    previous_system = json.loads((previous.parent/'run.json').read_text())['system']
                    old_sim = next(s for s in old['services'].values()
                        if s.get('labels',{}).get('io.elesim.system_id') == previous_system
                        and s.get('labels',{}).get('io.elesim.role') == 'sim')
                    seed = next(Path(v['source']) for v in old_sim['volumes'] if v['target']=='/tmp/elesim-cache')
                shutil.copytree(seed, cache, dirs_exist_ok=True)
                record['cache_seed'] = str(seed)
            else:
                profile['vision'] = {'perception':{'runtime':{'mode':'sim','provider':'local',
                    'run_local':True,'show_preview':False}}}
            config_directory = next(Path(v['source']) for v in mounts if v.get('target')=='/opt/elesim/config')
            config_path = config_directory/'scan-cycle.yaml'
            write_json(config_path,config)
            mounts.append(bind(exchange,'/exchange',read_only=role=='sim'))
        if role in ('sim','ui','pilot'):
            script = TASK/'probes'/('probe_scan_cycle_'+role+'.py')
            record['probe_sha256'][script.name] = hashlib.sha256(script.read_bytes()).hexdigest()
            mounts.append(bind(script,'/probe/run.py',read_only=True))
            command = ['python3','/probe/run.py','--evidence','/cycle-output']
            if role == 'sim':
                command.extend(('--config','/opt/elesim/config/scan-cycle.yaml','--mode','scan_cycle',
                    '--runtime-config','/opt/elesim/config/runtime.installed.yaml'))
            elif role == 'ui':
                mounts.append(bind(TASK/'probes/scan_cycle_recording.py','/probe/scan_cycle_recording.py',read_only=True))
                record['probe_sha256']['scan_cycle_recording.py'] = hashlib.sha256(
                    (TASK/'probes/scan_cycle_recording.py').read_bytes()).hexdigest()
                command.append('--record-video')
                command.extend(('--system',system,'--pilot',endpoints['pilot'],
                    '--sim',endpoints['sim'],'--ui',endpoints['ui']))
                for peer in peers:
                    command.extend(('--peer',peer))
                lines.append('    user: '+json.dumps(f'{os.getuid()}:{os.getgid()}'))
            else:
                command.extend(('--config','/opt/elesim/config/scan-cycle.yaml','--mode','scan_cycle',
                    '--runtime-config','/opt/elesim/config/runtime.installed.yaml',
                    '--target',endpoints['sim'],'--scan-output-root','/exchange'))
            entry = ['/bin/bash','-c',
                'source /opt/ros/humble/setup.bash && source /opt/elesim/ros/install/setup.bash && exec "$@"','scan-cycle']
            lines.append('    entrypoint: !override '+json.dumps(entry))
        else:
            command = ['--config','/opt/elesim/config/scan-cycle.yaml','--mode','scan_cycle',
                '--target',endpoints['sim'],'--scan-output-root','/exchange']
        lines.append('    command: !override '+json.dumps(command))
        lines.append('    volumes: !override '+json.dumps(mounts))
    override = root/'cycle.compose.yaml'
    override.write_text('\n'.join(lines)+'\n')
    invocation = compose('-f',str(instance_file),'-f',str(override))
    record['compose'] = invocation
    write_json(root/'run.json',record)
    merged = run_json(invocation+['config','--format','json'])
    write_json(root/'compose.resolved.json',merged)
    for role,row in record['services'].items():
        service = merged['services'][row['key']]
        assert 'network_mode' not in service and service['networks']['scan_cycle']['ipv4_address'] in peers
        if role == 'ui':
            assert not any(v['target'] in ('/exchange','/probe/sim.py') for v in service['volumes'])
    with (root/'launch.log').open('w') as log:
        subprocess.run(invocation+['up','-d','--no-build',*[r['key'] for r in record['services'].values()]],
            stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    print(json.dumps(record), flush=True)


def upload():
    names = [Path(__file__),Path('workbench/tests/system/probe_scan_cycle_sim.py'),
        Path('workbench/tests/system/probe_scan_cycle_ui.py'),
        Path('workbench/tests/system/probe_scan_cycle_pilot.py'),
        Path('workbench/tests/system/scan_cycle_recording.py'),
        Path('workbench/research/analysis/evaluate_scan_cycle.py'),
        Path('workbench/research/analysis/profile_scan_models.py')]
    names.append(Path('workbench/research/debug/scan_cycle_native.py'))
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as archive:
        for path in names:
            archive.add(path,arcname=path.name)
    remote = f'''
import io,sys,tarfile,json
from pathlib import Path
root=Path({str(TASK)!r})/'probes';root.mkdir(exist_ok=True)
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r:gz') as archive:
 for member in archive.getmembers():
  assert member.isfile() and Path(member.name).name==member.name
  (root/member.name).write_bytes(archive.extractfile(member).read())
print(json.dumps({{'uploaded':sorted(p.name for p in root.iterdir())}}))
'''
    command = ['ssh','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=10',
        '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',
        '-o','ProxyCommand=python3 -m workbench.research.support.ssh_mss_proxy %h %p',
        'hckang@100.74.222.24','python3 -c '+shlex.quote(remote)]
    subprocess.run(command,input=buffer.getvalue(),check=True,timeout=120)


def stop(attempt):
    root = TASK/f'cycle-{attempt:02d}'
    record = json.loads((root/'run.json').read_text())
    prep = json.loads((TASK/'preparation.json').read_text())
    names = []
    for role,row in record['services'].items():
        info = run_json(DOCKER+['inspect',row['name']])[0]
        labels = info['Config']['Labels']
        assert labels['io.elesim.install_uuid'] == prep['install_uuid']
        assert labels['io.elesim.system_id'] == record['system']
        assert info['Image'] == row['image_id']
        names.append(row['name'])
    subprocess.run(DOCKER+['stop','--time','30',*names],check=True,timeout=100)
    for role,row in record['services'].items():
        result = subprocess.run(DOCKER+['logs',row['name']],capture_output=True)
        (root/(role+'.stdout.log')).write_bytes(result.stdout)
        (root/(role+'.stderr.log')).write_bytes(result.stderr)
    before = json.loads((TASK/'containers-before.json').read_text())
    ids = subprocess.check_output(DOCKER+['ps','-aq','--no-trunc'],text=True).split()
    after = {row['Id']:{'name':row['Name'],'image':row['Image'],'status':row['State']['Status']}
        for row in run_json(DOCKER+['inspect',*ids])}
    preservation = dict(before=len(before),after=len(after),
        missing=sorted(set(before)-set(after)),
        changed=[key for key in before if key in after and before[key] != after[key]])
    write_json(root/'preservation.json',preservation)
    ui_path, evaluation_path = root/'ui-output/ui.json',root/'sim-output/evaluation.json'
    ui = json.loads(ui_path.read_text()) if ui_path.exists() else {}
    evaluation = json.loads(evaluation_path.read_text()) if evaluation_path.exists() else {}
    record.update(stopped=time.time(),complete=bool(ui.get('complete') and evaluation.get('complete')
        and ui.get('video',{}).get('complete') and not preservation['missing'] and not preservation['changed']))
    write_json(root/'run.json',record)


def profile_models(attempt):
    root = TASK/f'cycle-{attempt:02d}'
    record = json.loads((root/'run.json').read_text())
    script = (TASK/'probes/profile_scan_models.py').read_bytes()
    rows = []
    for extra in ([],['--single-thread']):
        result = subprocess.run(DOCKER+['exec','-i',record['services']['pilot']['name'],
            'python3','-',*extra],input=script,capture_output=True,timeout=120)
        rows.append(dict(options=extra,returncode=result.returncode,
            stdout=result.stdout.decode(),stderr=result.stderr.decode()))
        print(json.dumps(rows[-1]),flush=True)
    write_json(root/'model-profile.json',rows)


def retry_ui(attempt):
    root = TASK/f'cycle-{attempt:02d}'
    record = json.loads((root/'run.json').read_text())
    merged = json.loads((root/'compose.resolved.json').read_text())
    ui = record['services']['ui']
    assert run_json(DOCKER+['inspect',ui['name']])[0]['State']['Status'] == 'exited'
    sequence = len(record.get('ui_retries',[]))+1
    output = root/('ui-output-retry-'+str(sequence))
    output.mkdir()
    name = ui['name']+'-retry-'+str(sequence)
    service = merged['services'][ui['key']]
    volumes = [v for v in service['volumes'] if v['target']!='/cycle-output']
    volumes.append(bind(output,'/cycle-output'))
    override = root/('ui-retry-'+str(sequence)+'.compose.yaml')
    # The original exited diagnostic UI stays retained at .4. The retry
    # joins the same isolated bridge; .2/.3 remain its discovery seeds.
    address = record['peers'][0].rsplit('.',1)[0]+'.'+str(4+sequence)
    override.write_text('\n'.join(['services:','  '+ui['key']+':',
        '    networks: !override '+json.dumps({'scan_cycle':{'ipv4_address':address}}),
        '    volumes: !override '+json.dumps(volumes)])+'\n')
    command = record['compose']+['-f',str(override),'run','-d','--no-deps','--pull','never',
        '--name',name,ui['key']]
    with (root/('ui-retry-'+str(sequence)+'.launch.log')).open('w') as log:
        subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT,timeout=120)
    record.setdefault('ui_retries',[]).append(dict(name=name,output=str(output),started=time.time(),
        image_id=ui['image_id'],address=address))
    write_json(root/'run.json',record)
    print(json.dumps(record['ui_retries'][-1]),flush=True)


def collect(attempt):
    remote = f'''
import io,sys,tarfile
from pathlib import Path
root=Path({str(TASK)!r})/'cycle-{attempt:02d}'
selected=[]
for path in root.rglob('*'):
 if not path.is_file() or path.is_symlink():continue
 relative=path.relative_to(root)
 if path.name in ('run.json','preservation.json','retention-checks.json','model-profile.json') or (path.suffix in ('.json','.png','.mp4') and
     ('output' in relative.parts[0] or path.name in ('scan.json','scene.json'))):selected.append(path)
assert sum(p.stat().st_size for p in selected)<256*1024*1024
buffer=io.BytesIO()
with tarfile.open(fileobj=buffer,mode='w:gz') as archive:
 for path in selected:archive.add(path,arcname=path.relative_to(root).as_posix())
sys.stdout.buffer.write(buffer.getvalue())
'''
    command = ['ssh','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=10',
        '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',
        '-o','ProxyCommand=python3 -m workbench.research.support.ssh_mss_proxy %h %p',
        'hckang@100.74.222.24','python3 -c '+shlex.quote(remote)]
    completed = subprocess.run(command,capture_output=True,check=True,timeout=180)
    output = Path('workbench/evidence/generated/scan-cycle-20261030')/f'cycle-{attempt:02d}'
    output.mkdir(parents=True,exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(completed.stdout),mode='r:gz') as archive:
        assert sum(m.size for m in archive.getmembers()) < 256*1024*1024
        for member in archive.getmembers():
            path = Path(member.name)
            assert member.isfile() and not path.is_absolute() and '..' not in path.parts
            destination = output/path
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(archive.extractfile(member).read())
    print(output)


def evaluate(attempt):
    root = TASK/f'cycle-{attempt:02d}'
    record = json.loads((root/'run.json').read_text())
    script = (TASK/'probes/evaluate_scan_cycle.py').read_bytes()
    result = subprocess.run(DOCKER+['exec','-i',record['services']['sim']['name'],'python3','-'],
        input=script,capture_output=True,timeout=120)
    (root/'evaluation.stdout.log').write_bytes(result.stdout)
    (root/'evaluation.stderr.log').write_bytes(result.stderr)
    print(result.stdout.decode(),end='')
    print(result.stderr.decode(),end='')
    raise SystemExit(result.returncode)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('upload','publish','launch','stop','profile-models','retry-ui','collect','evaluate'))
    parser.add_argument('--attempt',type=int,default=1)
    parser.add_argument('--math-threads',type=int,choices=(1,2,4,8),default=None)
    parser.add_argument('--debug-allocator',action='store_true')
    args = parser.parse_args()
    if args.action == 'upload': upload()
    elif args.action == 'publish': publish()
    elif args.action == 'launch': launch(args.attempt,args.math_threads,args.debug_allocator)
    elif args.action == 'stop': stop(args.attempt)
    elif args.action == 'profile-models': profile_models(args.attempt)
    elif args.action == 'retry-ui': retry_ui(args.attempt)
    elif args.action == 'collect': collect(args.attempt)
    else: evaluate(args.attempt)


if __name__ == '__main__':
    main()
