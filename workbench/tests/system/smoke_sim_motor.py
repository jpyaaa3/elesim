#!/usr/bin/env python3
"""Real two-process DDS motion delivery through the installed SimEndpoint.

The default probe checks the command mailbox. GPU diagnostics can supply a
consumer that also checks the real SimRuntime/Genesis actuator path. The pipe
coordinates assertions only; every motion payload and lease uses DDS.
"""
from __future__ import annotations

import json
import multiprocessing as mp
import time
import traceback
import uuid

from elesim_protocol import DdsRuntimeSettings, EndpointDescriptor, PeerClient, SimMappingConfig
from elesim_sim.control_state import SimulationStateSource
from elesim_sim.endpoint import SimEndpoint


COMMANDS = (
    dict(command='target', q=[-.10,.15,.10,-.10], go2_vel=[.2,-.1,.15], claw_closed=True),
    dict(command='target', q=[-.12,-.10,-.08,.12], go2_vel=[-.2,.1,-.15], claw_closed=False),
    dict(command='torque_off'),
)


def _pilot(settings, pipe, timeout_s):
    peer = None
    try:
        peer = PeerClient(EndpointDescriptor('motor-pilot','pilot',()), settings=settings)
        deadline = time.monotonic()+timeout_s
        lease = ''
        phase = 0
        last_selection = last_motion = 0.
        sent = set()
        waiting = False
        while time.monotonic() < deadline:
            peer.heartbeat()
            now = time.monotonic()
            if not lease and now-last_selection >= .5:
                discovery = peer.send('discover',payload={'role':'sim'})
                if any(item['endpoint_id']=='motor-sim'
                       for item in (discovery.payload or {}).get('endpoints',())):
                    peer.send('select_target',payload={'target_id':'motor-sim'})
                    last_selection = now
            for message in peer.receive(timeout_ms=10):
                if message.message_type=='target_selected': lease=message.lease_id
                if (message.message_type=='ack' and not waiting
                        and (message.payload or {}).get('reply_to') in sent
                        and (message.payload or {}).get('ok') is True):
                    waiting=True
                    pipe.send(dict(phase=phase,accepted=True,lease=lease))
            if waiting and pipe.poll():
                assert pipe.recv()=='next'
                phase+=1
                if phase==len(COMMANDS):
                    pipe.send(dict(done=True))
                    return
                sent.clear();waiting=False;last_motion=0.
            if lease and not waiting and now-last_motion >= .1:
                message=peer.send('motion_command',target_id='motor-sim',
                                  lease_id=lease,payload=COMMANDS[phase])
                sent.add(message.message_id)
                last_motion=now
        raise TimeoutError('DDS Sim motor probe timed out')
    except BaseException:
        pipe.send(dict(error=traceback.format_exc()))
    finally:
        if peer is not None:peer.close()
        pipe.close()


def run_probe(consumer=None, *, mapping=None, timeout_s=25.):
    settings=DdsRuntimeSettings(system_id='motor_'+uuid.uuid4().hex[:16],domain_id=178,
        rmw_implementation='rmw_cyclonedds_cpp',security_profile='trusted-network',
        heartbeat_timeout_s=4.)
    state=SimulationStateSource(mapping or SimMappingConfig())
    endpoint=SimEndpoint(endpoint_id='motor-sim',state=state,streams={},settings=settings)
    context=mp.get_context('spawn')
    parent,child=context.Pipe()
    process=context.Process(target=_pilot,args=(settings,child,timeout_s),daemon=True)
    phases=[]
    try:
        endpoint.start()
        process.start();child.close()
        for phase,command in enumerate(COMMANDS):
            assert parent.poll(timeout_s), 'no DDS acknowledgement'
            message=parent.recv()
            assert 'error' not in message, message
            assert message['phase']==phase and message['accepted'],message
            expected=COMMANDS[min(phase,1)]
            q=state.estimate_q()
            assert [q.linear_m,q.roll_rad,q.theta1_rad,q.theta2_rad]==expected['q']
            assert state.go2_vel()==(tuple(expected['go2_vel']) if phase<2 else (0.,0.,0.))
            assert state.claw_closed()==expected['claw_closed']
            if consumer is not None:consumer(state,phase)
            phases.append(dict(phase=phase,command=command['command'],accepted=True,
                               runtime_consumer=consumer is not None))
            parent.send('next')
        assert parent.poll(timeout_s)
        assert parent.recv()==dict(done=True)
        process.join(timeout=3.)
        assert process.exitcode==0,process.exitcode
        return dict(passed=True,phases=phases,transport='real CycloneDDS, two processes',
                    system_id=settings.system_id,domain_id=settings.domain_id)
    finally:
        if process.pid is not None:
            process.join(timeout=1.)
            if process.is_alive():process.terminate();process.join(timeout=3.)
        endpoint.close()
        parent.close();child.close()


if __name__=='__main__':
    print(json.dumps(run_probe(),indent=2),flush=True)
