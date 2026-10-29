# Wrap-grasp policy

`policy.pt` and `interface.json` are the byte-identical oct27 export restored
from commit `1a8913825c45112fe9c50a9bca63da8e7cdb2655`. File sizes, SHA256 and
source URLs are recorded in [provenance.json](provenance.json). Pilot consumes
these data files without importing Sim training/export code.

The manifest specifies 16 observations, five actions, four load channels and a
trained linear range of -0.230..0 m. It remains unchanged. Pilot intersects its
waypoint limits with the configured hardware mapping, now -260/1500..0 m
(1.5 degrees/mm, 260-degree cap), without rescaling actions or observations.
An out-of-range policy home is rejected. Narrowing the range does not establish
policy performance under physical load.

[wrap_grasp.yaml](../../../config/pilot/wrap_grasp.yaml) supplies the separate
Wrap tree geometry and roll sign. The runner uses real-time substep sleeps for
its lift script. A newly trained export must provide a compatible bounded
manifest; `policy.npz` remains an optional reference artifact.
