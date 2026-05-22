# Elesim WIP

`elesim_wip` is a work-in-progress control and simulation workspace for a tendon/segment-style robotic arm with a gripper.  
The project combines:

- a **simulation runtime** built on Genesis,
- a **desktop control panel** built with ImGui + GLFW,
- a **host bridge** that mediates between UI, simulation, and real hardware,
- and a small **IK package** for position solve and pose refinement.

The codebase is aimed at fast iteration on kinematics, control ideas, gripper interaction, and hardware-assisted debugging.

## Main Components

- [sim.py](./sim.py)  
  Runs the Genesis scene, spawns the robot from generated assets/URDF, listens for commands, and publishes simulation feedback.

- [ctrl.py](./ctrl.py)  
  Runs the operator UI. This is where target position, target direction, hardware controls, and IK actions are exposed.

- [host.py](./host.py)  
  Acts as the broker between `ctrl.py`, `sim.py`, and optional Dynamixel hardware. It owns device connection, state broadcast, and command forwarding.

- [engine/ik](./engine/ik)  
  Internal IK package.
  - [kinematics.py](./engine/ik/kinematics.py): forward kinematics, grasp pose, Jacobians, common math
  - [solver.py](./engine/ik/solver.py): position-oriented IK
  - [tweaker.py](./engine/ik/tweaker.py): fine pose refinement / small-step correction
  - [pipeline.py](./engine/ik/pipeline.py): entrypoints used by the UI

- [addons](./addons)  
  Standalone tools for experiments and analysis.  
  Example: [addons/ik_solution_space_probe.py](./addons/ik_solution_space_probe.py) explores IK solution clouds for a target point.

## What The System Does

At a high level, the system lets you:

- command the robot in simulation,
- optionally connect to hardware,
- solve for a reachable arm pose from a target point,
- visualize the target point and target direction,
- inspect actual grasp position and direction,
- and iterate on IK and fine-tuning strategies without hard-coding everything into the main runtime.

The current robot model includes:

- one linear axis,
- one roll axis,
- two bend controls,
- and a gripper.

## Runtime Topology

The three main processes are intended to run together:

1. `host.py`
2. `sim.py`
3. `ctrl.py`

Communication is done over local ZeroMQ endpoints configured in [config.ini](./config.ini):

- control channel
- simulation publish channel
- simulation feedback channel

`host.py` is the hub.  
`ctrl.py` sends commands to `host.py`, `sim.py` subscribes to command/state updates, and simulation feedback flows back through the host.

## Installation

Python dependencies are listed in [requirements.txt](./requirements.txt).

Typical setup:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Notes:

- `genesis-world` must be installed in the active environment for `sim.py`.
- `pyzmq` is required for process communication.
- `glfw` and `imgui[glfw]` are required for the control UI.
- `dynamixel-sdk` and `pyserial` are required for hardware use.

## Running

Start the host:

```bash
python3 host.py
```

Start the simulator:

```bash
python3 sim.py
```

Start the control panel:

```bash
python3 ctrl.py
```

In simulation-only mode, keep `use_hardware = false` in [config.ini](./config.ini).  
For hardware-assisted runs, set `use_hardware = true` and ensure the target serial device is available.

## Configuration

Project-wide runtime settings live in [config.ini](./config.ini), including:

- GPU / hardware toggles
- ZeroMQ endpoints
- motor direction conventions
- joint limits and model settings
- spawn position and debug marker visibility

The robot assembly is generated into [craft](./craft) from JSON assets under [assets](./assets) and builder scripts under [builder](./builder).

## Operator Workflow

From the UI, a typical simulation workflow is:

1. choose a target position,
2. choose a target direction vector,
3. run `Solve IK`,
4. inspect the target marker versus the actual grasp marker,
5. iterate on refinement logic if the result is not satisfactory.

The UI also exposes hardware-related actions such as device selection, torque control, and gripper commands when hardware mode is enabled.

## Development Notes

This repository is intentionally structured for experimentation:

- robot geometry and assembly are data-driven,
- IK logic is separated from runtime orchestration,
- analysis tools live under `addons/`,
- and simulation/hardware can be exercised independently.

If you are extending the control logic, the best entrypoints are usually:

- [engine/ik/pipeline.py](./engine/ik/pipeline.py) for UI-facing IK flow,
- [engine/ik/solver.py](./engine/ik/solver.py) for position solve logic,
- [engine/ik/tweaker.py](./engine/ik/tweaker.py) for fine-tuning / clutch-like behavior,
- [engine/ik/kinematics.py](./engine/ik/kinematics.py) for shared kinematic computation.

## Status

This is an active WIP repository.  
Expect experimental behavior, ongoing refactors, and control logic that is still being tuned for both simulation and hardware use.
