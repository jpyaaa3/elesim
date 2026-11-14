# Experimental reconstructed-cylinder hug

Three phase-aligned D100 C-hug command paths from
`hug-linear200-interpolation-v2-20261114`; this is not a neural-weight blend.
`manifest.json` pins the paths and the exact data-free mapper kernel. Node.js is
included in Pilot images to execute that kernel. Runtime never imports workbench.
Commands are linear metres, roll radians, and per-node bend radians. Displayed
q3/q4 are total bends (five times the per-node angle). The exported triangle is
specific to this robot and nominal Home. No extrapolation or diameter fallback.

In Visual Servoing choose **100 mm cylinder preset**, then **Place object**.
Scan/reconstruct the physical Sim cylinder. Do not apply the reconstruction as
an additional fixed collision mesh over the original cylinder. Return the arm
to Home, then choose **Prepare reconstructed hug**, review the fitted diameter,
solver XY and weights, and **Execute reconstructed hug**. Stop uses the existing
trajectory cancellation. Preparing never moves the arm or silently connects a
non-Home start. Changing the scene/base requires a fresh scan/plan.

The cylinder OBJ is 100 mm diameter and 1.1 m tall. Its Sim instance is dynamic,
with nominal density for 0.3 kg and friction 0.01. Other Sim/arm/floor/controller
settings are not automatically changed to match training. Integrated DDS timing,
contact behavior and physical success need live validation; completion of a
command trajectory does not prove loaded grasp success. The executor may pause
for feedback, so its wall-clock duration need not equal training simulation time.

The command path holds its last value up to the 20-second nominal episode budget.
This package remains explicitly `sim-experimental`: sampled study results are
not a certificate for the entire triangle or for reconstructed-geometry errors.
