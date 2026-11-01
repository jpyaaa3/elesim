"""Reject stale voxel profiles and keep retained cycle addresses reproducible."""
import ipaddress

import pytest

from workbench.research.support.scan_cycle_run import cycle_network
from workbench.tests.system.probe_scan_cycle_ui import check_poisson_surface


@pytest.mark.parametrize('attempt,subnet', [(1,'172.30.231.0/24'),(24,'172.30.254.0/24')])
def test_retained_cycle_addresses_are_unchanged(attempt, subnet):
    actual, peers = cycle_network(attempt)
    assert actual == subnet
    assert [str(ipaddress.ip_network(subnet).network_address+i) for i in (2,3,4)] == peers


def test_further_cycles_use_distinct_valid_networks_without_replacing_old_runs():
    networks = [cycle_network(attempt)[0] for attempt in range(1,255)]
    assert len(set(networks)) == 254
    assert all(ipaddress.ip_network(n).is_private for n in networks)
    assert all(not ipaddress.ip_network(a).overlaps(ipaddress.ip_network(b))
               for i,a in enumerate(networks) for b in networks[i+1:])


@pytest.mark.parametrize('attempt', [0,255,-1,True,1.5])
def test_out_of_range_cycle_never_creates_resources(attempt):
    with pytest.raises(ValueError):
        cycle_network(attempt)


def status(mesher):
    return dict(phase='completed',model_execution=dict(models=dict(mesher=mesher)))


def test_actual_poisson_diagnostics_are_accepted():
    diagnostics = dict(surface_method='screened_poisson',remesh_resolution_m=.004)
    assert check_poisson_surface(status(diagnostics)) == diagnostics


@pytest.mark.parametrize('mesher', [{}, {'voxel_m':.02},
    dict(surface_method='screened_poisson',remesh_resolution_m=0.)])
def test_completed_scan_with_stale_or_unregularized_mesh_cannot_pass_poisson_cycle(mesher):
    with pytest.raises(AssertionError):
        check_poisson_surface(status(mesher))
