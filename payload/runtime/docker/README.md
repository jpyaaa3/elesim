# Docker payload

This directory contains source-controlled Docker meal-kit ingredients.

- `pilot`, `sim`, and `ui` contain each role's
  application, dependency lock, and fixed entrypoint.
- `tools` contains the complete setup application and tools image
  context.
- `development` contains the persistent Developer image context.
- `shared` contains canonical ingredients intentionally shared by more than
  one generated context. The installer copies these without rendering them.

Only host-specific Compose, DDS, GPU, security, credential, and ownership data
is generated during installation. Shared protocol and ROSIDL sources remain a
single canonical copy under `../common` and are assembled into isolated image
contexts by the installer.
