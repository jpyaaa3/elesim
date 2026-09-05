# Wrap workspace probe

Peak wrap angle the 4-DoF arm reaches in the built Genesis scene, swept over the per-node bend limits with the best cylinder centre searched per pose.

| item | value |
|---|---|
| grid (linear x roll x theta1 x theta2) | 4 x 9 x 9 x 9 |
| poses evaluated | 2916 |
| per-node bend limit | 36.0 deg |
| cylinder radius | 45 mm |
| object centre height | 0.560 m (fixed) |
| coverage bins | 180 |
| success gate | 172.0 deg |
| poses at or above the gate | 8 / 2916 |
| poses that actually cage the object | 1 / 2916 |
| poses in real contact with the object | 207 / 2916 |

## Best poses

| linear (m) | roll (deg) | theta1 (deg/node) | theta2 (deg/node) | Phi (deg) | near links | caged | contacts | collides |
|---:|---:|---:|---:|---:|---:|:--|---:|:--|
| -0.153 | 135 | 36.0 | 36.0 | -57.3 | 0 | no | 7 | yes |
| -0.230 | 90 | 36.0 | 36.0 | -57.3 | 0 | no | 5 | yes |
| 0.000 | -90 | -36.0 | -36.0 | -57.3 | 0 | no | 5 | yes |
| -0.230 | -135 | -27.0 | -36.0 | -57.3 | 0 | no | 4 | no |
| -0.230 | -90 | -36.0 | -9.0 | -57.3 | 0 | no | 4 | no |
| -0.230 | 90 | 36.0 | 9.0 | -57.3 | 0 | no | 4 | no |
| -0.153 | -135 | -27.0 | -27.0 | -57.3 | 0 | no | 4 | no |
| -0.153 | -90 | -27.0 | -27.0 | -57.3 | 0 | no | 4 | no |
| -0.153 | 90 | 27.0 | 27.0 | -57.3 | 0 | no | 4 | no |
| -0.077 | -180 | -36.0 | -36.0 | -57.3 | 0 | no | 4 | no |
| -0.077 | -135 | -27.0 | -27.0 | -57.3 | 0 | no | 4 | no |
| -0.077 | -90 | -36.0 | -9.0 | -57.3 | 0 | no | 4 | no |
| -0.077 | -90 | -27.0 | -27.0 | -57.3 | 0 | no | 4 | no |
| -0.077 | 45 | 36.0 | 18.0 | -57.3 | 0 | no | 4 | yes |
| -0.077 | 90 | 27.0 | 27.0 | -57.3 | 0 | no | 4 | no |

Peak Phi = **-57.3 deg** at linear = -0.153 m, roll = 135 deg, theta1 = 36.0 deg/node, theta2 = 36.0 deg/node, with the cylinder centred at (0.000, 0.000, 0.000) m.

