"""Prepare a private display in the owned diagnostic UI container, then record it."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time


DOCKER = ['docker','--context','default']
ENGINE = '2ee9c1c8-0d97-43a9-82eb-dfef7cda0ae3'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True)
    args = parser.parse_args()
    root = args.run_root.resolve(strict=True)
    record = json.loads((root/'run.json').read_text())
    assert record['native_window'], 'use only a fresh native-window scan-cycle'
    assert subprocess.check_output(DOCKER+['info','--format','{{.ID}}'],text=True).strip() == ENGINE
    prep = json.loads((root.parent/'preparation.json').read_text())
    ui = record['services']['ui']
    container = json.loads(subprocess.check_output(DOCKER+['inspect',ui['name']],text=True))[0]
    labels = container['Config']['Labels']
    assert labels['io.elesim.install_uuid'] == prep['install_uuid']
    assert labels['io.elesim.system_id'] == record['system']
    assert labels['io.elesim.role'] == 'ui' and container['Image'] == ui['image_id']
    assert container['State']['Running'] and not container['HostConfig']['ReadonlyRootfs']
    assert container['Args'][-2:] == ['sleep','infinity'], 'display setup requires an idle diagnostic UI'
    evidence = dict(container_id=container['Id'],image_id=container['Image'],
        scope='display tools in this new diagnostic UI container only; no host packages or image mutation',
        started=time.time(),complete=False)
    def run(command,log,timeout):
        with (root/log).open('w') as output:
            result = subprocess.run(command,stdout=output,stderr=subprocess.STDOUT,timeout=timeout)
        if result.returncode:
            raise RuntimeError(f'{log}: exit {result.returncode}')
    # The runtime bridge intentionally has no Internet route. UI is still a
    # sleep-only container: add egress for display packages, then remove it
    # before any DDS participant or desktop process starts.
    networks = set(container['NetworkSettings']['Networks'])
    assert 'bridge' not in networks
    subprocess.run(DOCKER+['network','connect','bridge',container['Id']],check=True,timeout=30)
    try:
        run(DOCKER+['exec','--user','0',ui['name'],'apt-get',
            '-o','APT::Update::Error-Mode=any','update'],'native-display-apt-update.log',300)
        run(DOCKER+['exec','--user','0','-e','DEBIAN_FRONTEND=noninteractive',ui['name'],
            'apt-get','install','-y','--no-install-recommends','xvfb','xdotool','ffmpeg','x11-utils'],
            'native-display-apt-install.log',600)
    finally:
        subprocess.run(DOCKER+['network','disconnect','bridge',container['Id']],check=True,timeout=30)
    after = json.loads(subprocess.check_output(DOCKER+['inspect',container['Id']],text=True))[0]
    assert set(after['NetworkSettings']['Networks']) == networks
    evidence['runtime_networks_restored'] = sorted(networks)
    run(DOCKER+['exec','-d',ui['name'],'Xvfb',':99','-screen','0','1920x1080x24',
        '-nolisten','tcp','-ac'],'native-display-start.log',30)
    time.sleep(1.)
    command = DOCKER+['exec','-e','DISPLAY=:99','-e','PYOPENGL_PLATFORM=glx',
        '-e','LIBGL_ALWAYS_SOFTWARE=1',ui['name'],'/bin/bash','-c',
        'source /opt/ros/humble/setup.bash && source /opt/elesim/ros/install/setup.bash && exec "$@"',
        'native-ui','python3','/probe/window.py','--evidence','/cycle-output',
        '--pilot',record['endpoints']['pilot'],'--sim',record['endpoints']['sim']]
    evidence['command'] = command
    try:
        run(command,'native-window.log',1200)
        evidence.update(complete=True,finished=time.time())
    finally:
        (root/'native-window-driver.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence),flush=True)


if __name__ == '__main__':
    main()
