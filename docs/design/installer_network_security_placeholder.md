# Installer Network and Security (Placeholder)

> Dummy record — keep this page until the installer/connection-manager
> boundary is revisited. It is not an operator guide.

The installer no longer presents a **Network and security** step. It creates
the selected role contexts with manager-owned defaults:

- managed SROS2 provisioning starts in its pending state;
- a Sim installation owns the managed Coturn service and creates its local
  secret, but does not guess a mutable relay address;
- DDS addresses, interfaces, discovery mode, security generation, SSH
  management endpoints, and the final TURN endpoint are selected and applied
  by `elesim-connections`.

The legacy DDS/security/SSH/TURN request fields remain readable at the setup
request boundary for existing automation and state migration. They are not
installer controls. A pending managed TURN endpoint is completed by the
connection manager from the current Sim host address before runtime launch.

Revisit this placeholder when the manager-owned configuration contract is
formally documented and the compatibility fields can be narrowed or removed.
