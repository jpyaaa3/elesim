# Wrap geometry, measured in the built scene

Everything here comes from the Genesis scene, driven by ramping the joint
command (never `set_dofs_position`, which teleports into penetration and makes
the solver diverge).  Renders are in `render/`.

## Arm layout at the home pose

All links sit at `z = 0.555`, `y = 0`, extending along `+x`:

| link | x (m) |
|---|---:|
| `housing` | 0.350 (at z = 0.400) |
| `wedge` | 0.350 |
| `node0` .. `node4` (segment 1) | 0.378 .. 0.578 |
| `node5` .. `node9` (segment 2) | 0.628 .. 0.828 |
| `gripper_base` | 0.878 |

Node pitch is exactly 0.050 m, matching the 50 mm measured in
the historic `motion_planning` analytic wrap sweep.

## The bend plane has to be horizontal

An upright cylinder can only be surrounded by a coil lying in a horizontal
plane, and `roll` is what orients that plane:

| roll | link-cloud extent (dx, dy, dz) | coil plane |
|---:|---|---|
| 0 deg | 0.218, 0.101, **0.373** | vertical |
| -90 deg | 0.224, 0.231, 0.203 | **horizontal** |

## Coil centre and radius

Circle fit to the arm links at `roll = -90 deg`.  Residual scatter of 1-17 mm
says the coil really is circular.

| theta (deg/node) | centre (x, y) | radius (m) | z |
|---:|---|---:|---:|
| 20.1 | (0.355, 0.142) | 0.142 | 0.547 |
| 27.0 | (0.357, 0.116) | 0.114 | 0.548 |
| 31.5 | (0.361, 0.112) | 0.106 | 0.549 |
| 36.0 | (0.371, 0.112) | 0.097 | 0.548 |

The object therefore stands at **(0.357, 0.116, 0.548)** -- the hole the arm
closes around.  `support.height_m = 0.448` seats it there.

## Wrap angle against self-collision

The usable wrap is limited by the arm tip reaching its own base, not by the
joint limits.

| theta (deg/node) | Phi (deg) | caged | self-collision |
|---:|---:|:--|:--|
| 9.2 | 70 | no | no |
| 13.8 | 90 | no | no |
| 18.3 | 150 | no | no |
| 20.6 | 224 | no | no |
| **22.9** | **250** | no | **no** |
| 25.2 | 268 | yes | yes |
| 27.0 | 276 | yes | yes |

Beyond ~23 deg/node the gripper claws drive into the `housing` link: 31.5 mm
penetration at 21 N, a real collision rather than incidental touching.

**Collision-free maximum: Phi = 250 deg.**  The 172 deg success gate is
reachable with margin, at roughly 19-20 deg/node.

## Caveat on these Phi values

`coverage.link_radius_m` is set from the collision model's 56.22 mm, which is a
*bounding* radius (max perpendicular vertex distance), not a local contact
radius, and `radial_band_m` of 0.09 lets a link 9 cm clear of the surface count
towards the wrap angle.  Two consequences:

* the `caged` column is unreliable -- the inflated wrap radius makes the escape
  chord look wider than it is, so 250 deg reads as not caged when the geometry
  suggests it should be;
* poses in real contact can be rejected by the geometric interpenetration test.

Coverage should be redefined on the contact set the simulator reports rather
than on a guessed radius.  Until then, treat Phi as ordinal, not exact.
