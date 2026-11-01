"""A retained old study inventory must not override this run's resource audit."""
import pytest

from workbench.research.support.scan_cycle_run import preservation_ok


def test_old_absence_is_reported_separately_from_preserved_prelaunch_resources():
    historical = dict(missing=['absent-before-this-run'], changed=[])
    assert preservation_ok(dict(historical, prelaunch_before=202,
        prelaunch_missing=[], prelaunch_changed=[]))
    assert historical['missing'] == ['absent-before-this-run']


@pytest.mark.parametrize('field', ['prelaunch_missing', 'prelaunch_changed'])
def test_missing_or_changed_resource_from_this_run_fails(field):
    audit = dict(missing=[], changed=[], prelaunch_before=202,
        prelaunch_missing=[], prelaunch_changed=[])
    audit[field] = ['protected-container']
    assert not preservation_ok(audit)


@pytest.mark.parametrize('missing,changed,expected', [([],[],True),(['gone'],[],False),([],['changed'],False)])
def test_legacy_runs_without_start_inventory_keep_the_historical_gate(missing, changed, expected):
    assert preservation_ok(dict(missing=missing, changed=changed)) is expected
