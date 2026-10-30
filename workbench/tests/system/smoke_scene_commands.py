"""Two OS processes exercise capability-gated scene commands over real DDS.

The Sim command consumer validates session ownership. Physics/camera readiness
is covered separately by the GPU runtime probe; this test opens no hardware.
"""
import multiprocessing as mp
import os
from pathlib import Path
import tempfile
import time
import traceback
import uuid

from elesim_protocol import (CAPABILITY_SIM_SCENE_REBUILD, DdsRuntimeSettings,
    EndpointDescriptor, OpenSimulationSessionRequest, PeerClient, SimulationCommandRequest,
    SimulationResultPayload)


def settings(system):
    return DdsRuntimeSettings(system_id=system, domain_id=186,
        rmw_implementation='rmw_cyclonedds_cpp', security_profile='trusted-network',
        discovery_mode='static', static_peers=('127.0.0.1',), network_interface='lo')


def sim(system, ready, stop, results):
    from elesim_protocol import SimMappingConfig
    from elesim_sim.control_state import SimulationStateSource
    from elesim_sim.endpoint import SimEndpoint
    endpoint = SimEndpoint(endpoint_id='scene-sim', state=SimulationStateSource(SimMappingConfig()),
        streams={}, scene_rebuild_enabled=True)
    endpoint.settings = settings(system)
    try:
        endpoint.start()
        ready.set()
        while not stop.is_set():
            for command in endpoint.operator_mailbox.drain():
                assert endpoint.operator_mailbox.owns(command)
                if command.command == 'load_scene':
                    assert command.arguments == {'manifest': 'new-object/scene.json', 'expected_revision': 'a'*64}
                else:
                    assert command.command == 'clear_scene'
                endpoint.operator_mailbox.complete(command, ok=True, reason='validated scene command')
            time.sleep(.01)
    except BaseException:
        results.put(('error', traceback.format_exc()))
    finally:
        endpoint.close()


def ui(system, ready, stop, results):
    peer = None
    try:
        assert ready.wait(10)
        peer = PeerClient(EndpointDescriptor('scene-ui', 'ui'), settings=settings(system))
        deadline = time.monotonic()+25
        opened, sent, acknowledged = '', False, set()
        retry = 0.
        while time.monotonic() < deadline and len(acknowledged) < 2:
            peer.heartbeat()
            live = peer.node.discover(role='sim', capability=CAPABILITY_SIM_SCENE_REBUILD)
            if live and not opened and time.monotonic() >= retry:
                peer.send('open_simulation_session', payload=OpenSimulationSessionRequest(
                    request_id='open-scene', sim_id='scene-sim', streams=('observer',)).to_payload())
                retry = time.monotonic()+.5
            for message in peer.receive(timeout_ms=20):
                if message.message_type == 'simulation_session_opened':
                    opened = message.lease_id
                elif message.message_type == 'simulation_result':
                    result = SimulationResultPayload.from_payload(message.payload)
                    assert result.ok and result.session_id == opened
                    acknowledged.add(result.command)
            if opened and not sent:
                for name, args in (('load_scene', {'manifest': 'new-object/scene.json', 'expected_revision': 'a'*64}),
                                   ('clear_scene', {})):
                    peer.send('simulation_command', target_id='scene-sim', lease_id=opened,
                        payload=SimulationCommandRequest(name, opened, name, args).to_payload())
                sent = True
            if opened:
                peer.send('renew_simulation_session', target_id='scene-sim', lease_id=opened, payload={})
        assert acknowledged == {'load_scene', 'clear_scene'}, acknowledged
        results.put(('passed', os.getpid()))
    except BaseException:
        results.put(('error', traceback.format_exc()))
    finally:
        if peer is not None: peer.close()
        stop.set()


def run():
    context = mp.get_context('spawn')
    ready, stop, results = context.Event(), context.Event(), context.Queue()
    system = 'scene_smoke_' + uuid.uuid4().hex[:12]
    workers = [context.Process(target=fn, args=(system, ready, stop, results)) for fn in (sim, ui)]
    for worker in workers: worker.start()
    deadline = time.monotonic()+40
    while time.monotonic() < deadline and any(p.is_alive() for p in workers):
        for worker in workers: worker.join(.1)
    stop.set()
    for worker in workers:
        worker.join(3)
        if worker.is_alive(): worker.terminate(); worker.join(3)
    records = []
    while not results.empty(): records.append(results.get())
    assert all(p.exitcode == 0 for p in workers), [p.exitcode for p in workers]
    assert any(row[0] == 'passed' for row in records), records
    assert not any(row[0] == 'error' for row in records), records
    print('PASS: load_scene/clear_scene, capability and session ownership, two real DDS processes', flush=True)


def main():
    with tempfile.TemporaryDirectory(prefix='elesim-scene-dds-') as directory:
        os.environ['ROS_LOG_DIR'] = str(Path(directory) / 'ros-logs')
        run()


if __name__ == '__main__':
    main()
