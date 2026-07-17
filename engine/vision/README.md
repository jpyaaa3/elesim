# Vision

`engine.vision` turns cameras into observations and geometry helpers.

- `perception/`: detector, tracker, depth, preview, and capture lifecycle.
- `perception_bridge/`: hand-eye transforms.
- `sim_camera/`: Genesis camera mount, publisher, relay, and recording.
- `visual_servoing/`: visual-servo and pose-planning mathematics.

Vision does not command robot motion; `engine.pick` consumes its observations.
