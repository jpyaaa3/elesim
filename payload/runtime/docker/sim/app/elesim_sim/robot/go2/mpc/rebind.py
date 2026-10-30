"""Replace Genesis handles without rebuilding the MPC model or JIT solver."""
from .genesis_pin_bridge import GenesisPinBridge
from .contact_diagnostics import GenesisContactDiagnostics
from ..locomotion.kinematics import Go2KinematicsModel
from elesim_sim.simulation.genesis.control_state import EntityControlStateReader


def rebind_controller(controller, entity, arm_entity):
    kin = Go2KinematicsModel.from_entity(entity)
    old_indices = list(controller._kin.all_leg_dof_idx)
    if list(kin.all_leg_dof_idx) != old_indices:
        raise ValueError("scene rebuild changed GO2 joint topology")
    controller._entity, controller._kin = entity, kin
    payload = getattr(controller, "_payload", None)
    if payload is not None:
        payload.rebind_entity(arm_entity)
    bridge = getattr(controller, "_bridge", None)
    if bridge is not None:
        controller._bridge = GenesisPinBridge(entity, old_indices,
            payload=getattr(bridge, "_payload", None))
    if hasattr(controller, "_state_reader"):
        controller._state_reader = EntityControlStateReader.for_entity(entity)
    diagnostic = getattr(controller, "_contact_diagnostics", None)
    if diagnostic is not None:
        controller._contact_diagnostics = GenesisContactDiagnostics(entity,
            cadence_steps=diagnostic.cadence_steps)
    # Physical parameters can live in scene arrays rather than descriptions.
    apply_physics = getattr(controller, "_apply_physics_params", None)
    if apply_physics is not None:
        apply_physics()
