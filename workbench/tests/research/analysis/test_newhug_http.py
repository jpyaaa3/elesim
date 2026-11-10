"""Loopback-only standalone API checks; requires permission to bind localhost."""
import json
from pathlib import Path
import re
import select
import shutil
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest

from workbench.research.support import newhug


@pytest.fixture(scope='module')
def server():
    process = subprocess.Popen([sys.executable, newhug.__file__, '--no-browser', '--port', '0'],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert select.select([process.stdout], [], [], 10)[0], 'server startup timeout'
        line = process.stdout.readline()
        assert 'http://127.0.0.1:' in line, process.stderr.read() if process.poll() is not None else line
        base = line.split('http://', 1)[1].strip()
        base = 'http://' + base
        page = urllib.request.urlopen(base).read().decode()
        token = re.search(r'const TOKEN="([^"]+)"', page).group(1)
        yield base, token
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        process.stdout.close()
        process.stderr.close()


def request(server, path, data=None, *, authorized=True):
    base, token = server
    headers = {'X-Newhug-Token': token} if authorized else {}
    if data is not None:
        headers['Content-Type'] = 'application/json'
    req = urllib.request.Request(base.rstrip('/')+path,
                                 data=json.dumps(data).encode() if data is not None else None, headers=headers)
    with urllib.request.urlopen(req, timeout=15) as response:
        return json.load(response)


def complete(server, job):
    deadline = time.monotonic()+15
    while time.monotonic() < deadline:
        state = request(server, '/api/status')
        assert state['job'] == job
        if not state['running']:
            assert state['error'] is None
            return state['result']
        time.sleep(.02)
    pytest.fail('bounded small map did not finish')


def test_api_records_filters_and_regathers_without_losing_originals(server):
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, '/api/status', authorized=False)
    assert error.value.code == 403
    job = request(server, '/api/solve', dict(shape='circle', width_mm=100, settings=dict(body_clearance_m=.02),
                  needle_settings=dict(iterations=64)))['job']
    result = complete(server, job)
    needles = result['needle_map']['needles']
    assert needles  # A finite budget and a friction window do not promise a count.
    assert any(n['friction']['mu_critical'] < .2 for n in needles)
    assert 'mu_min' not in result['needle_map']['settings']
    assert 'minimum' not in result['needle_map']['filter']
    assert all(n['friction']['mu_critical'] < .5 for n in needles)
    assert all(n['geometry']['body_target_clearance_m'] > .02 for n in needles)
    assert set(result['arm_model']['protected_part_names']) == {'camera', 'housing', 'wedge'}
    band = request(server, '/api/gather', dict(job=job, support_fraction=.6, anchor_angle_deg=30))
    assert band['band_m'][0] < band['band_m'][1]
    assert band['consensus']['support_fraction'] == .6
    assert request(server, '/api/status')['result']['needle_map']['needles'] == needles
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, '/api/gather', dict(job='stale'))
    assert error.value.code == 400
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, '/api/solve', dict(settings=dict(body_clearance_m=-.001)))
    assert error.value.code == 400


def test_cancel_saves_partial_map_and_bad_iterations_fail_before_work(server):
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, '/api/solve', dict(needle_settings=dict(iterations=0)))
    assert error.value.code == 400
    job = request(server, '/api/solve', dict(shape='circle', width_mm=100,
                  needle_settings=dict(iterations=5000)))['job']
    request(server, '/api/cancel', {})
    result = complete(server, job)
    assert result['search']['status'] == 'cancelled'
    assert result['needle_map']['iterations_completed'] < 5000


def test_cutoff_updates_both_views_restores_known_solutions_and_fences_stale_updates(server):
    job = request(server, '/api/solve', dict(shape='circle', width_mm=100,
                  needle_settings=dict(iterations=16)))['job']
    initial = complete(server, job)
    assert initial['needle_map']['needles']
    r = request(server, '/api/cutoff', dict(job=job, operating_mu=0., revision=2))
    assert r['candidates'] == r['needle_map']['needles'] == r['needle_map']['gathered']['needles'] == []
    assert request(server, '/api/gather', dict(job=job))['needles'] == []
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, '/api/cutoff', dict(job=job, operating_mu=.8, revision=1))
    assert error.value.code == 400
    r = request(server, '/api/cutoff', dict(job=job, operating_mu=.5, revision=3))
    assert r['needle_map']['needles'] == initial['needle_map']['needles']
    assert 'normal_error_deg' not in r['settings']
    assert all('mu_robust' not in n['friction'] for n in r['needle_map']['needles'])
    widened = request(server, '/api/cutoff', dict(job=job, operating_mu=1., revision=4))
    assert widened['search']['additional_search_needed']
    assert widened['search']['search_mu_limit'] == .5
    with pytest.raises(urllib.error.HTTPError) as error:
        request(server, '/api/cutoff', dict(job='stale', operating_mu=.5))
    assert error.value.code == 400


def test_page_javascript_with_real_api(server):
    node = shutil.which('node')
    if not node:
        pytest.skip('Node is unavailable; API and geometry tests remain usable')
    source = Path(__file__).with_name('newhug_ui_smoke.cjs')
    result = subprocess.run([node, str(source), server[0]], text=True, capture_output=True, timeout=25)
    assert result.returncode == 0, result.stdout+result.stderr
