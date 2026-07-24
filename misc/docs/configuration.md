# Configuration

Configuration is owned by each deployment. There is no repository-global
runtime config loader.

| Role | Configuration |
| --- | --- |
| Controller | `controller/config/` |
| Robot | `robot/config/` |
| Simulator | `simulator/config/` |
| UI | `ui/config/` |

Each release context contains only its role's configuration. Paths inside a
config therefore resolve relative to that deployment rather than repository
root.

## YAML Profiles

Controller and simulator preserve the existing YAML inheritance behavior:

```yaml
schema_version: 1
extends: config.yaml
simulation:
  runtime:
    enable_viewer: false
```

`extends` is resolved relative to the file containing it. Mappings merge
recursively; a child scalar or list replaces its parent value. Unknown schema
versions and invalid types fail during startup.

## Runtime Identity And DDS

Every deployment owns a stable logical endpoint ID and creates a new boot ID
at startup. It advertises exact valid, boot-specific service/topic prefixes;
consumers do not reconstruct or silently sanitize them. A common DDS block
defines:

```yaml
dds:
  system_id: elesim
  domain_id: 42
  rmw_implementation: rmw_cyclonedds_cpp
  discovery_mode: multicast       # or static
  static_peers: []
  interface: wg0
  security_profile: trusted-network
  keystore: null
  enclave: null
```

All participants in one graph use the same system/domain ID, compatible RMW
implementation and QoS contract. `interface` is host-specific and must name
the LAN/VPN interface on which every required DDS peer is directly reachable.
Static peers are reachable IP addresses used to seed discovery when multicast
does not cross the network; they are not a relay or NAT traversal mechanism.

The supported profiles are:

- `trusted-network`: no DDS encryption; allowed only on an owned LAN or routed
  VPN restricted by the selected interface and firewall.
- `sros2`: DDS Security in enforce mode using the role's `keystore` and
  `enclave`, required for untrusted/shared networks.

`ROS_DOMAIN_ID` is not authentication or tenant isolation. There are no
ZMQ/CURVE endpoints or keys in runtime configuration.

CLI options may override DDS values for a temporary layout. Production
installations should keep stable graph values in deployed YAML and mount SROS2
private material separately.

The terminal installer writes equivalent host-specific configuration under
`~/.local/share/elesim/roles/<role>/config`. Files are named `installed.yaml`
or `runtime.installed.yaml`; Simulator also receives `app.installed.yaml` for
the selected GPU/CPU policy. It never modifies the checked-in defaults. The
non-secret source of truth is `install-state.json`, and `elesim-net configure`
regenerates every installed role from that state so Controller, UI, Simulator
and Robot cannot silently drift to incompatible system/domain IDs, RMW,
discovery, interface or security settings.

Simulator runtime configuration also owns TURN credentials. Managed mode points
to a Coturn REST HMAC secret; external mode points to a JSON file containing
`username`, `credential`, and optional `expires_at`. The two sources are
mutually exclusive. Generated container configuration mounts either source
under `/run/secrets`, and never copies the external file into Controller or UI.

GPU allocation remains outside deployment configuration by default. In
`inherit` mode the generated launchers preserve an existing
`CUDA_VISIBLE_DEVICES`, allowing Slurm or a laboratory launcher to own the
assignment. A pinned installation exports exactly one index/UUID; CPU mode
hides CUDA and sets Simulator `use_gpu` false.

## Model Inputs

Geometry and assembly settings are not runtime configuration. They are source
inputs under `misc/model/source`, compiled by `misc/tooling/model_builder`, and shipped
to the simulator as `model/bundles/default`.
