# Configuration

Configuration is owned by each deployment. There is no repository-global
runtime config loader.

| Role | Configuration |
| --- | --- |
| Controller | `controller/config/` |
| Router | `router/config/` |
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

## Runtime Identity

Router addresses, endpoint IDs and advertised streams live with the deployment
that uses them. For example, controller and simulator runtime identity files
are `controller/config/runtime.yaml` and
`simulator/config/runtime.yaml`. UI and robot use the equivalent
fields in their `default.yaml`.

CLI options may override addresses for temporary LAN layouts. Production
installations should keep stable addresses in the deployed YAML and mount
secrets separately. Runtime identity/security files use schema version 2;
simulation/model behavior profiles retain their independent schema version 1.

The checked-in defaults are loopback-only. Each role provides a
`public.example.yaml` or `runtime.public.example.yaml` showing Curve paths for
multi-host use. A direct media publisher has two different addresses:

- `rgbd_bind` is the local socket, normally `tcp://0.0.0.0:5568` on a remote
  Simulator.
- `rgbd_advertise` is the hostname/IP Controller can reach.

Public media also requires `media_server_secret_file` and
`media_client_public_keys_dir`. The latter is a ZAP allowlist containing only
Controller's public media key. Advertising a public address does not alter the
local bind automatically.

The terminal installer writes equivalent host-specific configuration under
`~/.local/share/elesim/roles/<role>/config`. Files are named `installed.yaml`
or `runtime.installed.yaml`; Simulator also receives `app.installed.yaml` for
the selected GPU/CPU policy. It never modifies the checked-in defaults. The
non-secret source of truth is `install-state.json`, and `elesim-net configure`
regenerates every installed role from that state so Router, Controller, UI,
Simulator and Robot cannot silently drift to different Router addresses.

GPU allocation remains outside deployment configuration by default. In
`inherit` mode the generated launchers preserve an existing
`CUDA_VISIBLE_DEVICES`, allowing Slurm or a laboratory launcher to own the
assignment. A pinned installation exports exactly one index/UUID; CPU mode
hides CUDA and sets Simulator `use_gpu` false.

## Model Inputs

Geometry and assembly settings are not runtime configuration. They are source
inputs under `misc/model/source`, compiled by `misc/tooling/model_builder`, and shipped
to the simulator as `model/bundles/default`.
