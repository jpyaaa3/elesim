"""Isolate Pilot serialization and stack diagnostics in its installed image.

The native watchdog mode deliberately reproduces the old diagnostic API.
It is opt-in and runs only in a new, retained, network/GPU-free container.
"""
from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time


TASK = Path('/home/hckang/ws/elesim-scan-cycle-20261030')
DOCKER = ['docker','--context','default']


def write(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def launch(args):
    assert re.fullmatch('[a-z0-9-]{1,48}',args.name)
    root = TASK/'native-139-20261031'/args.name
    root.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(__file__,root/'probe.py')
    shutil.copyfile(TASK/'probes/thread_stacks.py',root/'thread_stacks.py')
    base = json.loads((TASK/'cycle-14/run.json').read_text())
    pilot = base['services']['pilot']
    prep = json.loads((TASK/'preparation.json').read_text())
    image = json.loads(subprocess.check_output(DOCKER+['image','inspect',pilot['image_id']],text=True))[0]
    assert image['Config']['Labels']['io.elesim.install_uuid'] == prep['install_uuid']
    name = 'elesim-sandy-native-139-'+args.name
    command = ['python3','/probe.py','run','--mode',args.mode,'--seconds',str(args.seconds),
        '--interval',str(args.interval),'--workers',str(args.workers),'--output','/diagnostic']
    override = root/'probe.compose.yaml'
    override.write_text('\n'.join(['services:','  '+pilot['key']+':',
        '    image: '+json.dumps(pilot['image_id']),'    user: '+json.dumps(f'{os.getuid()}:{os.getgid()}'),
        '    read_only: true','    network_mode: none','    networks: !reset []',
        '    ports: !reset []','    depends_on: !reset {}','    restart: "no"',
        '    environment: !override '+json.dumps({'PYTHONUNBUFFERED':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}),
        '    volumes: !override '+json.dumps([
            dict(type='bind',source=str(root),target='/diagnostic'),
            dict(type='bind',source=str(root/'thread_stacks.py'),target='/thread_stacks.py',read_only=True),
            dict(type='bind',source=str(root/'probe.py'),target='/probe.py',read_only=True)]),
        '    deploy:', '      resources:', '        reservations:', '          devices: !reset []',
        '    entrypoint: !override '+json.dumps(command),'    command: !override []',
        "    tmpfs: !override ['/tmp:mode=1777']"])+'\n')
    invocation = base['compose']+['-f',str(override),'run','-d','--no-deps','--pull','never','--name',name,pilot['key']]
    subprocess.run(invocation,check=True,timeout=120)
    record = dict(system=base['system'],services={'pilot':dict(name=name,image_id=pilot['image_id'])},
        mode=args.mode,seconds=args.seconds,interval=args.interval,workers=args.workers,
        install_uuid=prep['install_uuid'],source_sha256=hashlib.sha256((root/'probe.py').read_bytes()).hexdigest())
    write(root/'run.json',record)
    print(json.dumps(dict(root=str(root),**record)),flush=True)


def start(args):
    root = TASK/'native-139-20261031'/args.name
    record = json.loads((root/'run.json').read_text())
    info = json.loads(subprocess.check_output(DOCKER+['inspect',record['services']['pilot']['name']],text=True))[0]
    assert info['State']['Running'] and info['Image'] == record['services']['pilot']['image_id']
    assert info['Config']['Labels']['io.elesim.install_uuid'] == record['install_uuid']
    deadline = time.monotonic()+30.
    while not (root/'ready.json').is_file():
        if time.monotonic() >= deadline:
            raise TimeoutError('diagnostic container did not become ready')
        time.sleep(.05)
    (root/'start').write_text(str(time.time())+'\n')


def run(args):
    from elesim_protocol.serde import encode_value
    from elesim_protocol import SimMappingConfig
    from elesim_pilot.remote_state import RemoteState

    output = args.output
    crash = (output/'native-crash.log').open('w')
    faulthandler.enable(file=crash,all_threads=True)
    payload = dict(schema_version=1,state=dict(linear=-.13,roll=.38,theta1=-.34,theta2=.56),
        service=dict(current_host_state=RemoteState(SimMappingConfig()).snapshot(tx_seq=0),
            control_mapping=SimMappingConfig(),object_scan=dict(phase='moving',accepted_views=1,
                coverage=dict(fraction=.4,unobserved_directions=[[.1,.2,.3] for _ in range(96)]))))
    stop = threading.Event()
    counts = [0]*args.workers
    errors = []
    def worker(index):
        count = 0
        try:
            while not stop.is_set():
                encode_value(encode_value(payload))
                count += 1
                counts[index] = count
        except BaseException as error:
            errors.append(repr(error))
            stop.set()
    result = dict(complete=False,python=sys.version,mode=args.mode,interval=args.interval,
        workers=args.workers,code_refcount_before=sys.getrefcount(encode_value.__code__))
    write(output/'ready.json',result)
    deadline = time.monotonic()+300.
    while not (output/'start').exists():
        if time.monotonic()>deadline:
            raise TimeoutError('diagnostic start was not requested')
        time.sleep(.05)
    threads = [threading.Thread(target=worker,args=(index,),name=f'serde-{index}') for index in range(args.workers)]
    sink = open(os.devnull,'w')
    sampler = None
    began = time.monotonic()
    try:
        for thread in threads: thread.start()
        if args.mode == 'native':
            faulthandler.dump_traceback_later(args.interval,repeat=True,file=sink)
        elif args.mode == 'safe':
            from thread_stacks import ThreadStackSampler
            sampler = ThreadStackSampler(sink,interval_s=args.interval).start()
        stop.wait(args.seconds)
    finally:
        if args.mode == 'native': faulthandler.cancel_dump_traceback_later()
        if sampler is not None: sampler.close()
        stop.set()
        for thread in threads: thread.join(10.)
        sink.close()
    result.update(complete=not errors and not any(thread.is_alive() for thread in threads),
        errors=errors,iterations=counts,elapsed_s=time.monotonic()-began,
        code_refcount_after=sys.getrefcount(encode_value.__code__))
    if sampler is not None:
        result.update(stack_samples=sampler.samples,stack_error=sampler.error)
        result['complete'] = result['complete'] and not sampler.error and sampler.samples>0
    write(output/'result.json',result)
    print(json.dumps(result),flush=True)
    if not result['complete']: raise RuntimeError('serialization diagnostic failed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('launch','start','run'))
    parser.add_argument('--name')
    parser.add_argument('--mode',choices=('none','native','safe'),default='none')
    parser.add_argument('--seconds',type=float,default=60.)
    parser.add_argument('--interval',type=float,default=.001)
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    if not 0 < args.seconds <= 600 or not 0 < args.interval <= 60 or not 1 <= args.workers <= 16:
        parser.error('diagnostic limits exceeded')
    if args.action == 'launch': launch(args)
    elif args.action == 'start': start(args)
    else: run(args)


if __name__ == '__main__':
    main()
