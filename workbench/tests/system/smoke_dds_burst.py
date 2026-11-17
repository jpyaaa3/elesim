#!/usr/bin/env python3
"""Protocol-only, two-process loopback test for a paused DDS receiver.

This needs ROSIDL/rclpy, but imports no deployment. It uses an isolated domain
and loopback interface; it never publishes motion or uses runtime credentials.
"""
import importlib.util
import hashlib
import inspect
import multiprocessing as mp
from pathlib import Path
import queue
import tempfile
import time
import uuid

from elesim_protocol import DdsRuntimeSettings, EndpointDescriptor, PeerClient


def _peer(role,system,vendor,matched,sent,done,results):
    peer=None
    try:
        settings=DdsRuntimeSettings(system_id=system,domain_id=176,
            security_profile='trusted-network',vendor_config=vendor)
        peer=PeerClient(EndpointDescriptor('burst-'+role,role),settings=settings)
        deadline=time.monotonic()+15.
        other='burst-'+('pilot' if role=='sim' else 'sim')
        while time.monotonic()<deadline:
            peer.heartbeat()
            discovered=next((d for d in peer.node.discover() if d.descriptor.endpoint_id==other),None)
            if discovered is not None:break
            list(peer.receive(timeout_ms=10))
        else:raise RuntimeError('burst peer discovery timed out')
        if role=='sim':
            # The control writer is volatile. Confirm reader/writer matching
            # before the deliberate receive pause, rather than assuming the
            # first sample can precede DDS association.
            while not matched.is_set() and time.monotonic()<deadline:
                peer.send('telemetry',target_id=other,payload={'actual_tip':[-1.,0.,0.]})
                peer.heartbeat();list(peer.receive(timeout_ms=10));time.sleep(.02)
            if not matched.is_set():raise RuntimeError('burst writer did not match')
            for i in range(16):
                payload={'actual_tip':[float(i),0.,0.],'sim_time_s':i*.05}
                if i%2:
                    payload.update(go2_base_pos=[0.,0.,.3],go2_base_rpy=[0.,0.,0.],
                        go2_base_timestamp_s=time.time())
                else:
                    payload.update(q=[-.1,0.,i*.001,0.],q_source='simulated',
                        measurement_time_s=time.time(),measurement_monotonic_s=time.monotonic())
                peer.send('telemetry',target_id=other,payload=payload)
            sent.set()
            while not done.is_set() and time.monotonic()<deadline:
                peer.heartbeat();list(peer.receive(timeout_ms=10))
            if not done.is_set():raise RuntimeError('burst receiver did not finish')
            results.put(('sim',True))
        else:
            while time.monotonic()<deadline:
                peer.heartbeat();batch=list(peer.receive(timeout_ms=10))
                if any(m.message_type=='telemetry' and (m.payload or {}).get('actual_tip')==[-1.,0.,0.] for m in batch):break
            else:raise RuntimeError('burst warm-up sample missing')
            matched.set()
            if not sent.wait(5.):raise RuntimeError('burst sender timed out')
            time.sleep(.2)
            # Exactly one pump must deliver the complete coherent burst.
            batch=list(peer.receive(timeout_ms=10))
            values=[int(m.payload['actual_tip'][0]) for m in batch
                if m.message_type=='telemetry' and m.source_id==other
                and m.source_boot_id==discovered.identity.boot_id
                and (m.payload or {}).get('actual_tip',[ -1. ])[0]>=0]
            if values!=list(range(16)):raise RuntimeError(f'incomplete one-pump burst: {values}')
            results.put(('pilot',True));done.set()
    except BaseException as exc:
        results.put((role,repr(exc)));done.set()
    finally:
        if peer is not None:peer.close()


def main():
    if importlib.util.find_spec('rclpy') is None or importlib.util.find_spec('elesim_interfaces') is None:
        print('ERROR: ROS 2 rclpy and generated elesim_interfaces are required')
        return 2
    from elesim_protocol import dds_transport
    print('DDS transport SHA256='+hashlib.sha256(Path(inspect.getfile(dds_transport)).read_bytes()).hexdigest(),flush=True)
    context=mp.get_context('spawn');results=context.Queue()
    matched,sent,done=(context.Event() for _ in range(3))
    system='elesim_burst_'+uuid.uuid4().hex[:12]
    # This is a test-process vendor configuration, not a host/network change.
    xml='''<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'''
    with tempfile.TemporaryDirectory(prefix='elesim-dds-burst-') as directory:
        vendor=Path(directory)/'cyclonedds.xml';vendor.write_text(xml)
        processes=[context.Process(target=_peer,args=(role,system,str(vendor),matched,sent,done,results),
            name='dds-burst-'+role) for role in ('pilot','sim')]
        observed={};deadline=time.monotonic()+20.
        try:
            for process in processes:process.start()
            while len(observed)<2 and time.monotonic()<deadline:
                try:role,result=results.get(timeout=.2);observed[role]=result
                except queue.Empty:pass
        finally:
            done.set()
            for process in processes:
                process.join(timeout=2.)
                if process.is_alive():process.terminate();process.join(timeout=2.)
        if observed!={'pilot':True,'sim':True} or any(p.exitcode!=0 for p in processes):
            raise RuntimeError(f'DDS receive burst failed: {observed}')
    print('Two-process loopback DDS burst passed: 16 arm/base samples in one pump')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
