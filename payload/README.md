# EleSim deployable payload

`payload/` is the source-controlled installation meal kit. Anything that can
be copied without knowing the destination host is already arranged by its
installed responsibility. Developers edit these projects in place; the
installer, development environment, quality gates, and release builder consume
the same files.

```text
payload/
├── config/                 # immutable application defaults and examples
│   ├── pilot/
│   ├── sim/
│   ├── ui/
│   └── robot/
├── data/                    # durable, app-consumed runtime inputs
│   ├── models/
│   │   ├── arm/             # generated Pilot kinematic model
│   │   ├── objects/         # bounded mock-object catalog
│   │   ├── perception/      # detector weights
│   │   └── assemblies/      # zed-mini and d435 assembled model bundles
│   ├── policies/            # exported policy artifacts
│   └── calibration/
│       ├── arm/             # sag/calibration artifacts
│       └── cameras/         # hand-eye transforms
└── runtime/
    ├── common/              # ROSIDL interfaces and shared protocol package
    ├── docker/
    │   ├── pilot/           # Pilot application-image ingredients
    │   ├── sim/             # Sim application-image ingredients
    │   ├── ui/              # UI application-image ingredients
    │   ├── tools/           # setup/tools image ingredients
    │   ├── development/     # optional development attachment image
    │   └── shared/          # common application-image ingredients
    └── native/robot/        # ready-to-copy Robot app, service units and installer
```

Each app's `app/` directory is its Python project root. Package
namespaces such as `elesim_pilot/` remain explicit import boundaries, but there
is no intermediate `src/` directory. Tests and research artifacts deliberately
live outside `payload/`; they are not shipped as runtime ingredients.

The installer may assemble shared canonical ingredients into more than one
context, render mutable configuration, and create host-owned directories. It
must not reconstruct static application trees file by file. Generated host identity,
active DDS endpoints, SROS2 private material, TURN credentials, installation
state, caches, logs, and operator transactions never belong in `payload/`.

`dist/` has a different meaning: it is disposable release output produced from
this payload by `workbench/tools/release/build.py`.
