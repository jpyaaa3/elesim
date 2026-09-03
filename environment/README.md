# Environment inputs

`environment/` contains auxiliary environment examples that are not deployed
as EleSim applications. Runtime and development image contexts live in
`payload/runtime/docker/`.

- `coturn/`: optional WebRTC TURN relay configuration
- `generated/`: local source-workspace scratch configuration

Local credentials and generated scratch files must stay outside the versioned
source tree.
