"""Which endpoint roles the operator may hand to the pilot.

Only a Sim endpoint owns the simulation session and its WebRTC camera offers,
which is why the filter exists.  It excluded the Robot too, so against real
hardware the panel's endpoint button did nothing, `active_endpoint` stayed
empty, and the UI read as never connected while the DDS link was fine.
"""
from __future__ import annotations


def _select(role: str, *, sim_session=None):
    """The selector from `elesim_ui.main`, with its two collaborators faked."""
    selected: list[str] = []
    switched: list[str] = []

    class _Service:
        def select_endpoint(self, endpoint_id: str) -> None:
            selected.append(endpoint_id)

    class _SimSession:
        def switch_target(self, endpoint_id: str) -> None:
            switched.append(endpoint_id)

    service = _Service()
    session = _SimSession() if sim_session else None

    # mirrors elesim_ui.main._run's local `select_endpoint`
    def select_endpoint(endpoint_id: str, endpoint_role: str) -> None:
        resolved = str(endpoint_role).strip().lower()
        if resolved not in {"sim", "robot"}:
            return
        service.select_endpoint(endpoint_id)
        if resolved == "sim" and session is not None:
            session.switch_target(endpoint_id)

    select_endpoint(f"{role}-endpoint", role)
    return selected, switched


def test_a_robot_endpoint_is_selectable():
    selected, switched = _select("robot", sim_session=True)
    assert selected == ["robot-endpoint"]
    # the robot owns no camera session
    assert switched == []


def test_a_sim_endpoint_also_takes_over_the_camera_session():
    selected, switched = _select("sim", sim_session=True)
    assert selected == ["sim-endpoint"]
    assert switched == ["sim-endpoint"]


def test_discovery_peers_are_still_refused():
    for role in ("ui", "pilot", "", "UNKNOWN"):
        selected, switched = _select(role, sim_session=True)
        assert selected == [], role
        assert switched == [], role


def test_the_real_selector_admits_robot_and_sim_only():
    """Guard the source itself, so the fake above cannot drift from it."""
    import inspect

    from elesim_ui import main

    src = inspect.getsource(main)
    assert 'resolved not in {"sim", "robot"}' in src or \
           'role not in {"sim", "robot"}' in src
