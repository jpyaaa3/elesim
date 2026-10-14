"""Opt-in acceptance of real fixed-body production torque traces."""
import json
import os
from pathlib import Path

import numpy as np
import pytest


def test_fixed_body_airborne_foot_tracks_without_persistent_offset():
    trace = os.environ.get('ELESIM_FIXED_FOOT_TRACE')
    if not trace:
        pytest.skip('set ELESIM_FIXED_FOOT_TRACE to an actual fixed-foot JSONL trace')
    rows = [json.loads(line) for line in Path(trace).read_text().splitlines()]
    assert len(rows) >= 100
    assert rows[-1]['t'] >= 2.9
    for row in rows:
        np.testing.assert_allclose(row['foot'], row['genesis_foot'], atol=1e-5)
    settled = [row for row in rows if .5 < row['t'] < 1 or row['t'] > 2.5]
    errors = [np.linalg.norm(np.asarray(row['target']) - row['foot']) for row in settled]
    assert max(errors) < .01, f'fixed-foot settled error {max(errors):.6f} m exceeds 10 mm'
    for window in ((.5, 1), (2.5, 3)):
        feet = np.array([row['foot'] for row in rows if window[0] < row['t'] < window[1]])
        assert np.max(np.ptp(feet, axis=0)) < .002, 'stationary foot oscillates more than 2 mm'
