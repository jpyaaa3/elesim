"""Capture a native crash from this task's Pilot without touching other roles.

Uses the generated tools service, host GDB files mounted read-only, and the
diagnostic Pilot's private PID namespace. Retains the debugger container/core.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import time


TASK = Path('/home/hckang/ws/elesim-scan-cycle-20261030')
DOCKER = ['docker','--context','default']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--attempt',type=int)
    selection.add_argument('--run-dir',type=Path)
    args = parser.parse_args()
    root = args.run_dir.resolve() if args.run_dir is not None else TASK/f'cycle-{args.attempt:02d}'
    assert root.is_dir() and TASK in root.parents
    record = json.loads((root/'run.json').read_text())
    prep = json.loads((TASK/'preparation.json').read_text())
    pilot = record['services']['pilot']
    info = json.loads(subprocess.check_output(DOCKER+['inspect',pilot['name']],text=True))[0]
    assert info['State']['Running'] and info['Image'] == pilot['image_id']
    assert info['Config']['Labels']['io.elesim.install_uuid'] == prep['install_uuid']
    assert info['Config']['Labels']['io.elesim.system_id'] == record['system']
    paths = {'/usr/bin/gdb','/usr/share/gdb'}
    dependencies = subprocess.check_output(['ldd','/usr/bin/gdb'],text=True)
    paths.update(re.findall(r'(/[^\s()]+)',dependencies))
    assert all(Path(path).exists() for path in paths)
    mounts = [dict(type='bind',source=path,target=path,read_only=True) for path in sorted(paths)]
    mounts.append(dict(type='bind',source=str(root),target='/diagnostic'))
    name = f'elesim-sandy-native-{root.name}-{time.time_ns()}'
    commands = ['set debuginfod enabled off','set pagination off',
        'set sysroot /proc/1/root','attach 1','handle SIGPIPE nostop noprint pass',
        'continue','thread apply all bt 25','generate-core-file /diagnostic/pilot-native.core','detach']
    command = ['-nx','--batch']
    for value in commands:
        command.extend(('-ex',value))
    override = root/'native.compose.yaml'
    override.write_text('\n'.join(['services:','  tools:',
        '    image: '+json.dumps(prep['image_ids']['tools']),
        '    user: "0:0"','    read_only: true','    network_mode: none',
        '    pid: '+json.dumps('container:'+pilot['name']),
        '    cap_add: [SYS_PTRACE]',
        '    security_opt: [seccomp=unconfined, apparmor=unconfined]',
        '    restart: "no"','    entrypoint: !override ["/usr/bin/gdb"]',
        '    command: !override '+json.dumps(command),
        '    volumes: !override '+json.dumps(mounts),
        '    environment: !override {HOME: /tmp}',
        '    ports: !reset []','    networks: !reset []','    depends_on: !reset {}',
        "    tmpfs: !override ['/tmp:mode=1777']"])+'\n')
    invocation = [str(TASK/'install/bin/elesim-compose'),'--profile','tools',
        '-f',str(TASK/'install/containers/compose.yaml'),'-f',str(override),
        'run','-d','--no-deps','--pull','never','--name',name,'tools']
    subprocess.run(invocation,check=True,timeout=90)
    (root/'native-debugger.json').write_text(json.dumps(dict(name=name,target=pilot['name'],
        image=prep['image_ids']['tools'],host_dependencies=dependencies,command=command),indent=2)+'\n')
    print(name,flush=True)


if __name__ == '__main__':
    main()
