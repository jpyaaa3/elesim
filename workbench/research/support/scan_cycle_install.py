"""Prepare a preserved, normal-role installation for the remote scan cycle.

The local command sends a committed source archive and this stdlib driver.
The host uses the current installation's tools service to generate a fresh
installation, then its generated Compose wrapper builds the role images.
No container/image collection, role restart or source-checkout mutation runs.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import time
import traceback


TASK = Path('/home/hckang/ws/elesim-scan-cycle-20261030')
CURRENT = Path('/home/hckang/ws/newsim')
GPU = 'GPU-a40782b2-881c-e285-2a8e-66cae9162a01'
ENGINE = '2ee9c1c8-0d97-43a9-82eb-dfef7cda0ae3'
DOCKER = ['docker', '--context', 'default']


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def inventory():
    ids = subprocess.check_output(DOCKER+['ps', '-aq', '--no-trunc'], text=True).split()
    return {row['Id']:{'name':row['Name'], 'image':row['Image'], 'status':row['State']['Status']}
        for row in json.loads(subprocess.check_output(DOCKER+['inspect', *ids], text=True))}


def container_setup(*, refresh=False):
    revision = (TASK/'source-revision').read_text().strip()
    source = TASK/('source-'+revision[:7])
    sys.path.insert(0, str(source/'payload/runtime/docker/setup/app'))
    from elesim_setup.container_installer import ContainerInstaller
    from elesim_setup.state import InstallState, ComputeSettings, ContainerNetworkSettings, DdsSettings

    state = InstallState(profile='custom', roles=('pilot', 'sim', 'ui'),
        prefix=str(TASK/'install'), bin_dir=str(TASK/'install/bin'), source_root=str(source),
        source_repository='jpyaaa3/elesim', source_ref=revision,
        compute=ComputeSettings(gpu_mode='specific', gpu_device=GPU),
        ui_compute=ComputeSettings(gpu_mode='cpu'),
        dds=DdsSettings(system_id='scan_cycle_20261030', domain_id=212,
            discovery_mode='static', static_peers=('172.30.231.2','172.30.231.3','172.30.231.4'),
            interface='eth0', security_profile='trusted-network'),
        container_network=ContainerNetworkSettings(mode='direct-host', docker_context='default',
            docker_engine_id=ENGINE))
    if refresh:
        from dataclasses import replace
        state = replace(InstallState.load(TASK/'install/install-state.json'),
            source_root=str(source),source_ref=revision)
    ContainerInstaller(state, log=lambda value:print(value, flush=True)).run()


def host_build(*, refresh=False):
    assert TASK.is_dir() and not TASK.is_symlink()
    prefix = TASK/'install'
    revision = (TASK/'source-revision').read_text().strip()
    source = TASK/('source-'+revision[:7])
    assert re.fullmatch('[0-9a-f]{40}', revision)
    assert subprocess.check_output(DOCKER+['info','--format','{{.ID}}'], text=True).strip() == ENGINE
    before = inventory()
    if not (TASK/'containers-before.json').exists():
        write_json(TASK/'containers-before.json', before)
    if refresh:
        shutil.copyfile(TASK/'preparation.json',TASK/('preparation-before-'+revision[:7]+'.json'))
    result = {'complete':False, 'stage':'setup', 'started':time.time(), 'source_revision':revision}

    def save(): write_json(TASK/'preparation.json', result)

    save()
    try:
        image = json.loads(subprocess.check_output(DOCKER+['image','inspect','elesim/tools:merry-crab'], text=True))[0]
        assert image['Config']['Labels'].get('io.elesim.install_uuid') == '3fc3487e-0788-4a76-a152-e485144b0d0b'
        mounts = [dict(type='bind', source=str(TASK), target=str(TASK), read_only=False),
            dict(type='bind', source=str(source), target=str(source), read_only=True),
            dict(type='bind', source='/home/hckang/.local/share/elesim',
                target='/home/hckang/.local/share/elesim', read_only=False)]
        env = {'PYTHONUNBUFFERED':'1', 'PYTHONDONTWRITEBYTECODE':'1',
            'ELESIM_SOURCE_REVISION':'git-'+revision, 'ELESIM_OPERATOR_HOME':'/home/hckang',
            'ELESIM_HOST_USER':'hckang', 'USER':'hckang', 'LOGNAME':'hckang'}
        lines = ['services:', '  tools:', '    image: '+json.dumps(image['Id']),
            '    user: '+json.dumps(f'{os.getuid()}:{os.getgid()}'),
            '    working_dir: '+json.dumps(str(TASK)), '    read_only: true',
            '    restart: "no"', '    network_mode: none',
            '    environment: !override '+json.dumps(env), '    volumes: !override '+json.dumps(mounts),
            '    ports: !reset []', '    networks: !reset []', '    depends_on: !reset {}',
            '    devices: !reset []', "    tmpfs: !override ['/tmp:exec,mode=1777']",
            "    labels: {io.elesim.scan_cycle_task: '20261030'}"]
        override = TASK/'setup.compose.yaml'
        override.write_text('\n'.join(lines)+'\n')
        old = [str(CURRENT/'bin/elesim-compose'), '--profile', 'tools',
            '-f', str(CURRENT/'containers/compose.yaml'), '-f', str(override)]
        config = json.loads(subprocess.check_output(old+['config','--format','json'], text=True))['services']['tools']
        assert config['image'] == image['Id'] and config['network_mode'] == 'none' and config['read_only']
        name = 'elesim-merry-scan-cycle-setup-'+str(time.time_ns())
        assert subprocess.run(DOCKER+['inspect',name], capture_output=True).returncode != 0
        command = old+['run','-T','--no-deps','--pull','never','--name',name,
            '--entrypoint','python3','tools',str(TASK/'prepare_install.py'),
            'container-refresh' if refresh else 'container-setup']
        with (TASK/'setup.log').open('w') as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=300)
        if completed.returncode:
            raise RuntimeError('setup failed; see setup.log')
        own = json.loads((prefix/'install-ownership.json').read_text())
        result.update(stage='build', install_uuid=own['install_uuid'], project=own['docker']['project'])
        compose = [str(prefix/'bin/elesim-compose'), '--profile', 'tools',
            '-f', str(prefix/'containers/compose.yaml')]
        config = json.loads(subprocess.check_output(compose+['config','--format','json'], text=True))
        result['images'] = {role:config['services'][role]['image'] for role in ('pilot','sim','ui','tools')}
        result['compose'] = compose
        save()
        if shutil.disk_usage(TASK).free < 80*1024**3:
            raise RuntimeError('less than 80 GiB free before full image build')
        with (TASK/'build.log').open('w') as log:
            completed = subprocess.run([compose[0],'--progress','plain',*compose[1:],
                'build','pilot','sim','ui','tools'], stdout=log, stderr=subprocess.STDOUT, timeout=7200)
        if completed.returncode:
            raise RuntimeError('build failed; see build.log')
        result['image_ids'] = {role:json.loads(subprocess.check_output(DOCKER+['image','inspect',name],
            text=True))[0]['Id'] for role,name in result['images'].items()}
        result.update(complete=True, stage='built', finished=time.time())
    except BaseException as error:
        result['error'] = str(error)
        traceback.print_exc()
    finally:
        after = inventory()
        result['preservation'] = {'before':len(before), 'after':len(after),
            'missing':sorted(set(before)-set(after)),
            'changed':[key for key in before if key in after and before[key] != after[key]]}
        save()
        print(json.dumps(result), flush=True)


def local_prepare(host, *, resume=False, refresh=False):
    if resume:
        remote = f'''
import subprocess,sys
from pathlib import Path
task=Path({str(TASK)!r})
assert task.is_dir() and not (task/'install').exists()
script=task/'prepare_install.py';script.write_bytes(sys.stdin.buffer.read())
log=(task/'prepare-resume.log').open('w')
job=subprocess.Popen(['python3',str(script),'host-build'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print(job.pid,flush=True)
'''
        data = Path(__file__).read_bytes()
    else:
        data, remote = prepare_payload(refresh=refresh)
    command = ['ssh','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=10',
        '-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',
        '-o','ProxyCommand=python3 -m workbench.research.support.ssh_mss_proxy %h %p',
        host,'python3 -c '+shlex.quote(remote)]
    completed = subprocess.run(command, input=data, capture_output=True, timeout=180)
    print(completed.stdout.decode(), end='')
    print(completed.stderr.decode(), end='')
    raise SystemExit(completed.returncode)


def prepare_payload(*, refresh=False):
    revision = subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip()
    source = gzip.compress(subprocess.check_output(['git','archive','--format=tar','HEAD']),
        compresslevel=3, mtime=0)
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as archive:
        for name,data in [('source.tar.gz',source), ('prepare_install.py',Path(__file__).read_bytes())]:
            member = tarfile.TarInfo(name)
            member.size, member.mode = len(data), 0o600
            archive.addfile(member, io.BytesIO(data))
    remote = f'''
import io,tarfile,hashlib,json,subprocess,sys
from pathlib import Path
task=Path({str(TASK)!r})
if {refresh!r}: assert task.is_dir() and (task/'install').is_dir()
else: task.mkdir(mode=0o700)
source_archive=task/('source-'+{revision[:7]!r}+'.tar.gz')
data=sys.stdin.buffer.read(128*1024*1024+1)
assert len(data)<=128*1024*1024
with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as archive:
 assert {{m.name for m in archive.getmembers()}}=={{'source.tar.gz','prepare_install.py'}}
 for member in archive.getmembers():
  assert member.isfile()
  target=source_archive if member.name=='source.tar.gz' else task/member.name
  target.write_bytes(archive.extractfile(member).read())
source=task/('source-'+{revision[:7]!r});source.mkdir()
with tarfile.open(fileobj=io.BytesIO(source_archive.read_bytes()),mode='r:gz') as archive:
 members=archive.getmembers()
 assert sum(m.size for m in members)<512*1024*1024 and len(members)<30000
 for member in members:
  path=Path(member.name)
  assert not path.is_absolute() and '..' not in path.parts and (member.isdir() or member.isfile())
 for member in members:
  target=source/member.name
  if member.isdir():target.mkdir(parents=True,exist_ok=True)
  else:
   target.parent.mkdir(parents=True,exist_ok=True)
   target.write_bytes(archive.extractfile(member).read());target.chmod(member.mode&0o777)
(task/'source-revision').write_text({revision!r}+'\\n')
(task/('source-archive-sha256-'+{revision[:7]!r})).write_text(hashlib.sha256(source_archive.read_bytes()).hexdigest()+'\\n')
log=(task/'prepare.log').open('w')
job=subprocess.Popen(['python3',str(task/'prepare_install.py'),{'refresh-build' if refresh else 'host-build'!r}],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print(json.dumps({{'pid':job.pid,'source':str(source),'archive_bytes':len(data)}}),flush=True)
'''
    return buffer.getvalue(), remote


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare','resume','refresh','host-build','refresh-build',
        'container-setup','container-refresh'))
    parser.add_argument('--host', default='hckang@100.74.222.24')
    args = parser.parse_args()
    if args.action in ('prepare','resume','refresh'):
        local_prepare(args.host,resume=args.action=='resume',refresh=args.action=='refresh')
    elif args.action in ('host-build','refresh-build'): host_build(refresh=args.action=='refresh-build')
    else: container_setup(refresh=args.action=='container-refresh')


if __name__ == '__main__':
    main()
