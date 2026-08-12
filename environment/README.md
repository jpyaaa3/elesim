# Environment inputs

`environment/` contains source inputs used to construct runtime environments.
It is not an additional EleSim application and is not imported by Pilot, Sim,
UI or Robot at runtime.

- `containers/`: role image Dockerfiles and pinned image-build assets
- `development/`: the persistent `elesim-dev` image and Compose context inputs
- `coturn/`: optional WebRTC TURN relay configuration

The setup package copies only the selected inputs into generated installation
contexts. Local credentials and generated scratch files must stay outside the
versioned source tree.
