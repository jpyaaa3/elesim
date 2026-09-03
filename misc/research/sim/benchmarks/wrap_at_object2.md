# Wrap workspace probe

Peak wrap angle the 4-DoF arm reaches in the built Genesis scene, swept over the per-node bend limits with the best cylinder centre searched per pose.

| item | value |
|---|---|
| grid (linear x roll x theta1 x theta2) | 4 x 9 x 9 x 9 |
| poses evaluated | 2916 |
| per-node bend limit | 36.0 deg |
| cylinder radius | 45 mm |
| object centre height | 0.559 m (fixed) |
| coverage bins | 180 |
| success gate | 172.0 deg |
| poses at or above the gate | 6 / 2916 |
| poses that actually cage the object | 2 / 2916 |
| poses in real contact with the object | 5 / 2916 |

## Best poses

| linear (m) | roll (deg) | theta1 (deg/node) | theta2 (deg/node) | Phi (deg) | near links | caged | contacts | collides |
|---:|---:|---:|---:|---:|---:|:--|---:|:--|
| -0.153 | -23 | 36.0 | 9.0 | -57.3 | 0 | no | 2 | no |
| -0.077 | -23 | 36.0 | 9.0 | -57.3 | 0 | no | 2 | no |
| 0.000 | -23 | 36.0 | 9.0 | -57.3 | 0 | no | 2 | no |
| -0.230 | -23 | 36.0 | 18.0 | -57.3 | 0 | no | 1 | no |
| 0.000 | -23 | 36.0 | 18.0 | -57.3 | 0 | no | 1 | no |
| -0.230 | 90 | 0.0 | -36.0 | 258.0 | 10 | yes | 0 | no |
| -0.230 | -90 | 0.0 | 36.0 | 252.0 | 10 | yes | 0 | no |
| -0.230 | 45 | 0.0 | -27.0 | 204.0 | 9 | no | 0 | no |
| -0.230 | -45 | 0.0 | 27.0 | 200.0 | 9 | no | 0 | no |
| -0.230 | -68 | 0.0 | 27.0 | 174.0 | 7 | no | 0 | no |
| -0.230 | 68 | 0.0 | -27.0 | 174.0 | 8 | no | 0 | no |
| -0.230 | -90 | 0.0 | 27.0 | 154.0 | 6 | no | 0 | no |
| -0.230 | 90 | 0.0 | -27.0 | 154.0 | 6 | no | 0 | no |
| -0.230 | -90 | 0.0 | 18.0 | 120.0 | 5 | no | 0 | no |
| -0.230 | 90 | 0.0 | -18.0 | 120.0 | 5 | no | 0 | no |

Peak Phi = **-57.3 deg** at linear = -0.153 m, roll = -23 deg, theta1 = 36.0 deg/node, theta2 = 9.0 deg/node, with the cylinder centred at (0.000, 0.000, 0.000) m.

