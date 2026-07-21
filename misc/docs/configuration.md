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
  viewer:
    enable: false
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
secrets separately when transport authentication is added.

## Model Inputs

Geometry and assembly settings are not runtime configuration. They are source
inputs under `misc/model/source`, compiled by `misc/tooling/model_builder`, and shipped
to the simulator as `model/bundles/default`.
