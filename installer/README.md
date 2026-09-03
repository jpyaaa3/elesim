# Installer and bootstrap

`installer/bootstrap/` is the stdlib-only public bootstrap entrypoint.
`payload/runtime/docker/tools/app/` is the `elesim-setup` package containing installation,
connection-management, deployment, security and uninstall operations.

The installer reads repository inputs and writes products under the selected
installation prefix. It does not mutate this source tree.
