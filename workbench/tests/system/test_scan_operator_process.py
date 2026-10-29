"""Separate UI/Pilot processes exercise additive operator API shapes over JSON.

This deliberately does not claim a DDS/RMW acceptance test.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

from elesim_protocol import encode_value
from elesim_ui.operator import RemoteControlService


ROOT = Path(__file__).resolve().parents[3]


def test_ui_scan_calls_reach_pilot_dispatcher_in_an_isolated_process(tmp_path):
    requests = []
    class Session:
        def submit(self, operation, name, *args, **kwargs):
            requests.append({'request_id': 'request-'+str(len(requests)), 'operation': operation,
                'name': name, 'args': [encode_value(value) for value in args], 'kwargs': {}})
    proxy = RemoteControlService(Session(), None)
    proxy.object_scan_async('start_object_scan', 'apple-1', 8, on_result=lambda value: None,
        on_error=lambda error: None)
    proxy.object_scan_async('cancel_object_scan', on_result=lambda value: None, on_error=lambda error: None)
    dependencies = [p for p in sys.path if p and Path(p).name in {'site-packages', 'dist-packages'}]
    source = '''
import importlib.util, json, sys
from types import SimpleNamespace
sys.path.extend(json.loads(sys.argv[1]))
assert importlib.util.find_spec("elesim_ui") is None
from elesim_pilot.operator import OperatorDispatcher
class Service:
    def start_object_scan(self, target_id, max_views):
        assert target_id == "apple-1" and max_views == 8
        return {"job_id": "job", "phase": "preparing", "geometry_complete_claimed": False}
    def cancel_object_scan(self):
        return {"job_id": "job", "phase": "cancelled"}
dispatcher = OperatorDispatcher(SimpleNamespace(), Service())
for line in sys.stdin:
    print(json.dumps(dispatcher.handle(json.loads(line))))
'''
    env = {**os.environ, 'PYTHONPATH': os.pathsep.join(map(str, [
        ROOT/'payload/runtime/common/protocol', ROOT/'payload/runtime/docker/pilot/app']))}
    result = subprocess.run([sys.executable, '-S', '-c', source, json.dumps(dependencies)],
        input='\n'.join(json.dumps(value) for value in requests)+'\n', env=env, cwd=tmp_path,
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    responses = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
    assert all(response['ok'] for response in responses), responses
    assert [r['result']['phase'] for r in responses] == ['preparing', 'cancelled']
