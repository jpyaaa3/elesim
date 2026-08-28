"""Unit tests for the wrap-grasp reward terms.

Each test pins the *sign and magnitude* of one term for a hand-constructed
state, which is what makes a later regression legible: a reward that silently
flips sign trains a policy to do the opposite of the task.
"""

from __future__ import annotations

import math

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import pytest
import torch

from elesim_sim.rl.configs.loader import load_config
from elesim_sim.rl.envs.coverage import CoverageMeter, quat_to_axis
from elesim_sim.rl.envs.rewards import (
    TERM_NAMES,
    RewardBook,
    RewardInputs,
    approach_shaping,
    coverage_progress,
    enclosure_progress,
    object_disturbance,
)

_TWO_PI = 2.0 * math.pi

#: Coverage is computed in float32; a few micro-degrees of rounding is noise,
#: not the systematic bin-width over-credit these tests exist to catch.
_FLOAT_TOL_DEG = 1e-3


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture
def device():
    return torch.device("cpu")


def _book(cfg, device, n_envs=1):
    return RewardBook(
        cfg.reward,
        n_envs=n_envs,
        device=device,
        wrap_threshold_rad=cfg.success.coverage_target_rad,
    )


def _inputs(n=1, **kw):
    base = dict(
        phi=torch.zeros(n),
        enclosure=torch.zeros(n),
        surface_dist=torch.zeros(n),
        object_touch=torch.zeros(n, dtype=torch.bool),
        non_target_collision=torch.zeros(n, dtype=torch.bool),
        terminating_collision=torch.zeros(n, dtype=torch.bool),
        object_displacement=torch.zeros(n),
        object_tilt=torch.zeros(n),
        success=torch.zeros(n, dtype=torch.bool),
    )
    base.update(kw)
    return RewardInputs(**base)


# -- individual terms ------------------------------------------------------


def test_coverage_progress_is_signed_fraction_of_a_turn():
    quarter = torch.tensor([math.pi / 2])
    zero = torch.tensor([0.0])
    assert coverage_progress(quarter, zero).item() == pytest.approx(0.25)
    # Losing coverage must cost exactly what gaining it paid, otherwise
    # wrap/unwrap cycling is a reward farm.
    assert coverage_progress(zero, quarter).item() == pytest.approx(-0.25)


def test_approach_shaping_rewards_closing_and_stops_after_contact():
    far, near = torch.tensor([0.20]), torch.tensor([0.10])
    untouched = torch.zeros(1, dtype=torch.bool)
    closing = approach_shaping(near, far, d0=0.20, touched=untouched)
    assert closing.item() == pytest.approx(0.5)
    receding = approach_shaping(far, near, d0=0.20, touched=untouched)
    assert receding.item() == pytest.approx(-0.5)
    touched = torch.ones(1, dtype=torch.bool)
    assert approach_shaping(near, far, d0=0.20, touched=touched).item() == 0.0


def test_object_disturbance_has_a_deadband():
    assert object_disturbance(torch.tensor([0.004]), deadband_m=0.005).item() == 0.0
    assert object_disturbance(torch.tensor([0.015]), deadband_m=0.005).item() == (
        pytest.approx(0.010)
    )


# -- weighted totals and termination ---------------------------------------


def test_step_cost_is_the_only_term_on_an_idle_step(cfg, device):
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    out = book.step(_inputs())
    assert out.total.item() == pytest.approx(cfg.reward.weights.step_cost)
    assert not bool(out.terminate.item())


def test_non_target_collision_penalises_and_terminates(cfg, device):
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    out = book.step(
        _inputs(
            non_target_collision=torch.ones(1, dtype=torch.bool),
            terminating_collision=torch.ones(1, dtype=torch.bool),
        )
    )
    assert out.terms["non_target_collision"].item() == pytest.approx(
        cfg.reward.weights.non_target_collision
    )
    assert bool(out.terminate.item())
    assert bool(out.termination_reason["collision"].item())


def test_a_charged_collision_can_be_configured_not_to_terminate(cfg, device):
    """The penalty and the termination are separate inputs.

    Which contacts end the episode is `reward.self_contact.terminates`' call;
    the reward book only sees the two flags.
    """
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    out = book.step(_inputs(non_target_collision=torch.ones(1, dtype=torch.bool)))
    assert out.terms["non_target_collision"].item() == pytest.approx(
        cfg.reward.weights.non_target_collision
    )
    assert not bool(out.terminate.item())


def test_topple_terminates_and_costs_more_than_disturbance(cfg, device):
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    beyond = cfg.reward.disturbance.max_displacement_m + 0.01
    out = book.step(_inputs(object_displacement=torch.tensor([beyond])))
    assert bool(out.termination_reason["topple"].item())
    assert out.terms["object_topple"].item() == pytest.approx(
        cfg.reward.weights.object_topple
    )
    assert abs(cfg.reward.weights.object_topple) > abs(
        cfg.reward.weights.object_disturbance
    )


def test_success_pays_the_bonus_and_terminates(cfg, device):
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    out = book.step(_inputs(success=torch.ones(1, dtype=torch.bool)))
    assert out.terms["success"].item() == pytest.approx(cfg.reward.weights.success)
    assert bool(out.termination_reason["success"].item())


def test_disturbance_stops_being_charged_once_wrapped(cfg, device):
    """Moving a wrapped object is the goal, not a fault."""
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    wrapped = torch.tensor([cfg.success.coverage_target_rad])
    book.step(_inputs(phi=wrapped))
    out = book.step(_inputs(phi=wrapped, object_displacement=torch.tensor([0.03])))
    assert out.terms["object_disturbance"].item() == 0.0
    assert not bool(out.termination_reason["topple"].item())


def test_a_running_success_test_is_not_charged_for_moving_the_object(cfg, device):
    """The lift lays a standing pole down into the coil.

    That is 90 deg of tilt and far more displacement than `object_topple`
    allows, so without this the topple term terminates the episode in the middle
    of the manoeuvre the success test exists to perform -- and it fires before
    the wrap threshold has been reached, since the lift arms below it.
    """
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
               enclosure0=torch.zeros(1))
    laid_down = dict(
        object_displacement=torch.tensor([0.40]),
        object_tilt=torch.tensor([math.pi / 2]),
    )
    # Same state, once with the test running and once without.
    out = book.step(_inputs(**laid_down,
                            under_test=torch.ones(1, dtype=torch.bool)))
    assert not bool(out.termination_reason["topple"].item())
    assert out.terms["object_topple"].item() == 0.0
    assert out.terms["object_disturbance"].item() == 0.0

    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
               enclosure0=torch.zeros(1))
    out = book.step(_inputs(**laid_down))
    assert bool(out.termination_reason["topple"].item())


def test_episode_sums_accumulate_every_term(cfg, device):
    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    for _ in range(3):
        book.step(_inputs())
    sums = book.episode_sums()
    assert set(sums) == set(TERM_NAMES)
    assert sums["step_cost"].item() == pytest.approx(3 * cfg.reward.weights.step_cost)


# -- the "half wrapped" configuration the spec asks for --------------------


def test_half_wrapped_arm_scores_a_half_turn_of_coverage(cfg, device):
    """Twelve links on a semicircle at the surface -> phi = 180 deg, +1.0 reward."""
    radius = cfg.object.radius_m
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    angles = torch.linspace(0.0, math.pi, 12)
    links = torch.stack(
        (radius * torch.cos(angles), radius * torch.sin(angles), torch.zeros(12)),
        dim=-1,
    ).unsqueeze(0)
    res = meter.measure(
        links,
        torch.zeros(1, 3),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        radius_m=torch.tensor([radius]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    phi_deg = math.degrees(float(res.phi_rad[0]))
    # Coverage rounds down to a whole bin, so it may under-report but must not
    # exceed the true 180 deg beyond float32 noise.
    assert 175.0 <= phi_deg <= 180.0 + _FLOAT_TOL_DEG

    book = _book(cfg, device)
    book.reset(None, phi0=torch.zeros(1), dist0=torch.zeros(1),
           enclosure0=torch.zeros(1))
    out = book.step(_inputs(phi=res.phi_rad, object_touch=torch.ones(1, dtype=torch.bool)))
    expected = cfg.reward.weights.coverage_progress * (float(res.phi_rad[0]) / _TWO_PI)
    assert out.terms["coverage_progress"].item() == pytest.approx(expected)
    assert out.terms["coverage_progress"].item() > 0.9


def test_coverage_never_exceeds_the_true_arc(cfg, device):
    """Guard against the success gate being met by quantisation."""
    radius = cfg.object.radius_m
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    for true_deg in (30.0, 90.0, 172.0, 270.0):
        angles = torch.linspace(0.0, math.radians(true_deg), 24)
        links = torch.stack(
            (radius * torch.cos(angles), radius * torch.sin(angles), torch.zeros(24)),
            dim=-1,
        ).unsqueeze(0)
        res = meter.measure(
            links,
            torch.zeros(1, 3),
            torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
            radius_m=torch.tensor([radius]),
            height_m=torch.tensor([cfg.object.height_m]),
        )
        assert math.degrees(float(res.phi_rad[0])) <= true_deg + _FLOAT_TOL_DEG


# -- contact-based coverage ------------------------------------------------


def test_contact_span_bridges_two_distant_contacts(cfg, device):
    """Two contacts far apart along the chain span the links between them.

    A rigid cylinder inside a continuum coil touches at 2-3 points, because the
    coil is a spiral rather than a circle.  The strict rule scores that at zero
    whenever the touching links are not neighbours, which is most of the time;
    the span rule credits the run the arm is anchored on at both ends.
    """
    radius = cfg.object.radius_m
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    angles = torch.linspace(0.0, math.pi, 8)
    links = torch.stack(
        (radius * torch.cos(angles), radius * torch.sin(angles), torch.zeros(8)),
        dim=-1,
    ).unsqueeze(0)
    touching = torch.zeros(1, 8, dtype=torch.bool)
    touching[0, 1] = True
    touching[0, 6] = True
    common = dict(
        object_pos=torch.zeros(1, 3),
        object_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        radius_m=torch.tensor([radius]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    strict = meter.measure(links, contact_mask=touching, contact_rule="strict",
                           **{k: v for k, v in common.items()})
    span = meter.measure(links, contact_mask=touching, contact_rule="span",
                         **{k: v for k, v in common.items()})
    assert strict.phi_rad.item() == 0.0
    assert math.degrees(span.phi_rad.item()) >= 120.0


def test_contact_coverage_ignores_links_that_are_merely_close(cfg, device):
    """Proximity credits a hook; contact does not.

    A hook lying beside the cylinder has links near it in bearing but touching
    at one point.  Under the proximity rule those links bridge into an arc and
    the hook scores nearly as much as a real wrap, which is why a policy
    trained on it never pays the six macro steps of roll a coil costs.
    """
    radius = cfg.object.radius_m
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    angles = torch.linspace(0.0, math.pi, 8)
    links = torch.stack(
        (radius * torch.cos(angles), radius * torch.sin(angles), torch.zeros(8)),
        dim=-1,
    ).unsqueeze(0)
    common = dict(
        radius_m=torch.tensor([radius]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    proximity = meter.measure(
        links, torch.zeros(1, 3), torch.tensor([[1.0, 0.0, 0.0, 0.0]]), **common
    )
    # Only one link is actually touching.
    touching = torch.zeros(1, 8, dtype=torch.bool)
    touching[0, 3] = True
    contact = meter.measure(
        links,
        torch.zeros(1, 3),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        contact_mask=touching,
        contact_rule="strict",
        **common,
    )
    assert proximity.phi_rad.item() > 2.0        # ~180 deg from proximity alone
    assert contact.phi_rad.item() == 0.0         # a single contact spans nothing


def test_contact_coverage_rewards_a_spread_of_contacts(cfg, device):
    """Several contacts around the circumference do score an arc."""
    radius = cfg.object.radius_m
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    angles = torch.linspace(0.0, math.pi, 8)
    links = torch.stack(
        (radius * torch.cos(angles), radius * torch.sin(angles), torch.zeros(8)),
        dim=-1,
    ).unsqueeze(0)
    touching = torch.ones(1, 8, dtype=torch.bool)
    result = meter.measure(
        links,
        torch.zeros(1, 3),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        contact_mask=touching,
        radius_m=torch.tensor([radius]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    assert math.degrees(result.phi_rad.item()) >= 175.0


# -- enclosure -------------------------------------------------------------


def test_enclosure_progress_is_signed_fraction_of_a_turn(cfg, device):
    """Gaining bearing enclosure pays; giving it back costs the same.

    Unsigned, a policy could farm it by curling round the object and
    straightening out repeatedly.
    """
    gained = enclosure_progress(torch.tensor([math.pi]), torch.tensor([0.0]))
    given_back = enclosure_progress(torch.tensor([0.0]), torch.tensor([math.pi]))
    assert gained.item() == pytest.approx(0.5)
    assert given_back.item() == pytest.approx(-0.5)


def test_enclosure_needs_two_links_around_the_object(cfg, device):
    """One link has no bearing span, so it encloses nothing."""
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    single = torch.tensor([[[0.2, 0.0, 0.0]]])
    result = meter.measure(
        single,
        torch.zeros(1, 3),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        radius_m=torch.tensor([cfg.object.radius_m]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    assert result.enclosure_rad.item() == 0.0


def test_enclosure_ignores_distance(cfg, device):
    """A ring far outside the scoring band still encloses.

    This is the point of the term: it has to pay for getting the open coil
    around the object *before* anything is close enough to touch.  Every other
    signal is a distance, and a distance is minimised by poking.
    """
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    angles = torch.linspace(0.0, 2 * math.pi, 9)[:-1]
    common = dict(
        object_pos=torch.zeros(1, 3),
        object_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        radius_m=torch.tensor([cfg.object.radius_m]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    for ring in (0.08, 0.40):
        links = torch.stack(
            (ring * torch.cos(angles), ring * torch.sin(angles), torch.zeros(8)),
            dim=-1,
        ).unsqueeze(0)
        result = meter.measure(links, **common)
        assert math.degrees(result.enclosure_rad.item()) >= 300.0, ring


def test_an_arm_all_on_one_side_encloses_little(cfg, device):
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    angles = torch.linspace(0.0, 0.3, 8)
    links = torch.stack(
        (0.2 * torch.cos(angles), 0.2 * torch.sin(angles), torch.zeros(8)),
        dim=-1,
    ).unsqueeze(0)
    result = meter.measure(
        links,
        torch.zeros(1, 3),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        radius_m=torch.tensor([cfg.object.radius_m]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    assert math.degrees(result.enclosure_rad.item()) < 25.0


def test_plane_alignment_separates_a_wrap_from_a_curl_beside_it(cfg, device):
    """Bearing enclosure alone cannot tell those apart.

    Projecting to the object's cross-section throws away exactly the
    difference: measured in the scene, a coil curled beside the object in a
    vertical plane scores 98 deg of enclosure without going near it, more than
    the 34-80 deg a real approach passes through.  The alignment factor is what
    distinguishes them, so what is pinned here is that it does.
    """
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    angles = torch.linspace(0.0, 1.8 * math.pi, 8)
    ring = 0.09
    common = dict(
        object_pos=torch.zeros(1, 3),
        object_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]]),  # axis = +z
        radius_m=torch.tensor([cfg.object.radius_m]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    # A coil in the object's cross-section: normal parallel to the axis.
    flat = torch.stack(
        (ring * torch.cos(angles), ring * torch.sin(angles), torch.zeros(8)), dim=-1
    ).unsqueeze(0)
    # The same coil stood on edge: normal perpendicular to the axis.
    upright = torch.stack(
        (ring * torch.cos(angles), torch.zeros(8), ring * torch.sin(angles)), dim=-1
    ).unsqueeze(0)
    assert meter.measure(flat, **common).plane_alignment.item() > 0.95
    assert meter.measure(upright, **common).plane_alignment.item() < 0.05


def test_a_straight_arm_has_no_bend_plane_to_align(cfg, device):
    """Collinear links give no turning direction, so alignment stays low.

    It must not read as aligned by accident: the normal is a summed cross
    product, which vanishes for a straight chain, and the clamp keeps the
    normalisation from amplifying numerical noise into a full score.
    """
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    line = torch.stack(
        (torch.linspace(0.06, 0.30, 8), torch.zeros(8), torch.zeros(8)), dim=-1
    ).unsqueeze(0)
    result = meter.measure(
        line,
        torch.zeros(1, 3),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]]),
        radius_m=torch.tensor([cfg.object.radius_m]),
        height_m=torch.tensor([cfg.object.height_m]),
    )
    assert result.plane_alignment.item() < 0.05


def test_object_axis_carries_both_lean_direction_and_size(cfg, device):
    """The two object-orientation channels are the axis tilted into the plane.

    They used to be sin/cos of `atan2(axis.y, axis.x)`, which named itself yaw
    and was not: a cylinder is symmetric about its own axis, so rotating it
    changed nothing, while a lean of 1 deg and of 20 deg both read the same
    bearing and upright was `atan2(0, 0)`.  What is pinned here is that the
    replacement grows with the lean and stays put under a spin.
    """
    def axis(w, x, y, z):
        return quat_to_axis(torch.tensor([[w, x, y, z]]))[0, :2]

    # Spinning about the object's own axis moves nothing.
    for deg in (0.0, 90.0, 180.0):
        h = math.radians(deg) / 2
        assert axis(math.cos(h), 0.0, 0.0, math.sin(h)).abs().max().item() < 1e-6

    # Leaning grows as sin of the tilt, so magnitude is recoverable.
    for deg in (1.0, 5.0, 20.0, 40.0):
        h = math.radians(deg) / 2
        lean = axis(math.cos(h), math.sin(h), 0.0, 0.0)
        assert lean.norm().item() == pytest.approx(math.sin(math.radians(deg)), abs=1e-4)
