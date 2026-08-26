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
| poses at or above the gate | 228 / 2916 |
| poses that actually cage the object | 35 / 2916 |

## Best poses

| linear (m) | roll (deg) | theta1 (deg/node) | theta2 (deg/node) | Phi (deg) | best centre (m) | near links | caged |
|---:|---:|---:|---:|---:|---|---:|:--|
| 0.000 | 90 | 27.0 | 27.0 | 274.0 | (0.358, -0.099, 0.560) | 15 | yes |
| -0.230 | -90 | 27.0 | 27.0 | 272.0 | (0.130, 0.102, 0.560) | 15 | yes |
| -0.153 | -90 | 27.0 | 27.0 | 272.0 | (0.204, 0.102, 0.560) | 15 | yes |
| -0.077 | -90 | 27.0 | 27.0 | 272.0 | (0.250, 0.102, 0.560) | 15 | yes |
| 0.000 | -90 | 27.0 | 27.0 | 272.0 | (0.359, 0.102, 0.560) | 15 | yes |
| -0.230 | -90 | 18.0 | 27.0 | 266.0 | (0.172, 0.153, 0.560) | 15 | yes |
| -0.230 | 90 | 18.0 | 27.0 | 266.0 | (0.172, -0.153, 0.560) | 15 | yes |
| -0.153 | -90 | 18.0 | 27.0 | 266.0 | (0.247, 0.153, 0.560) | 15 | yes |
| -0.153 | 90 | 18.0 | 27.0 | 266.0 | (0.249, -0.153, 0.560) | 15 | yes |
| -0.077 | -90 | 18.0 | 27.0 | 266.0 | (0.293, 0.153, 0.560) | 15 | yes |
| -0.077 | 90 | 18.0 | 27.0 | 266.0 | (0.293, -0.153, 0.560) | 15 | yes |
| 0.000 | -90 | 18.0 | 27.0 | 266.0 | (0.402, 0.153, 0.560) | 15 | yes |
| 0.000 | 90 | 18.0 | 27.0 | 266.0 | (0.402, -0.153, 0.560) | 15 | yes |
| -0.230 | -90 | -27.0 | -18.0 | 264.0 | (0.114, -0.120, 0.560) | 15 | yes |
| -0.230 | 90 | -27.0 | -18.0 | 264.0 | (0.114, 0.120, 0.560) | 15 | yes |

Peak Phi = **274.0 deg** at linear = 0.000 m, roll = 90 deg, theta1 = 27.0 deg/node, theta2 = 27.0 deg/node, with the cylinder centred at (0.358, -0.099, 0.560) m.

