# Readable installation and release identities

Fresh container installations and subsequent updates use short, human-readable
names in the Compose project and immutable image tags. Full UUIDs, build
fingerprints, image IDs and release content digests remain the ownership and
provenance evidence; the readable names are presentation identifiers only.

## Agreed behavior

- Actual installation name: `quiet_otter`; fresh project: `elesim-quiet_otter`.
- Fresh runtime instance containers use the registered system alias and role:
  `elesim-<install_name>-<system_id>-pilot`,
  `elesim-<install_name>-<system_id>-sim`, or
  `elesim-<install_name>-<system_id>-ui`; managed instance Coturn uses
  `elesim-<install_name>-<system_id>-coturn`.
- Installation-wide containers use the installation alias instead, for example
  `elesim-quiet_otter-dev` and `elesim-quiet_otter-tailscale`. The development
  attachment is shared by the installation and deliberately has no
  `system_id`/DDS identity.
- Application image references use one suffix per role/input, for example
  `elesim/pilot:quiet_otter-silver_pigeon` and
  `elesim/sim:quiet_otter-golden_snail`. The suffix is the readable version /
  instance selector for that role; the full content-addressed release key
  remains internal.
- Release directories and instance CLI selectors continue to use the full
  immutable SHA-256 release key; a readable image suffix is never proof of
  ownership.
- Names are two familiar English words. Collision means retry, never overwrite.
- Store UUIDs, build fingerprints, image IDs and release content digests as
  internal ownership/provenance evidence; names are not credentials.
- Persist reservations, including failed builds and retired versions. Same
  identity reuses its current name; a different identity cannot inherit an old
  name. A colliding legacy binding keeps its old tag and receives one new
  current name.
- Do not rebuild simply to rename. Preserve old pinned releases, containers,
  ownership manifests and legacy projects. Existing projects must not change
  implicitly during update.

## Implementation

`readable_names.py` reserves identity-to-name mappings under an explicit registry
path with file locking and atomic publication. It rejects symlinks, malformed
registries and duplicate reservations. The installer reserves one name for its
UUID and one installation-wide image name per role/input, then emits tags such
as `elesim/sim:quiet_otter-calm_eagle` and a project such as
`elesim-quiet_otter`. The separate release scope reserves one suffix for each
application role/input.

The installation-wide image namespace covers Pilot, Sim, UI, tools and the
optional development image, so two roles in one installation cannot receive
the same new image alias. Release aliases are reserved in that same registry
but in a separate release scope; each role gets its own alias, and reservations
are collision-checked against every scope. This keeps independently selected
role images distinguishable even when they were built by one update. Registries
from the intermediate shared-release implementation are still accepted for
migration, while new publications never create shared aliases. Unchanged image
IDs and BuildKit layers may therefore be reused under a new role-specific
release tag.

`elesim-net identity` exposes the exact UUID/project enrollment pair and, on
updated installations, the readable `install_name` for display. A temporary
Compose `*-tools-run-*` container is only a setup/update helper and is never an
installation identity.

The ownership manifest records `container_naming: system-v1` for a fresh scoped
installation. Runtime names are therefore easy to read in Docker logs while
the UUID, Compose project and identity labels remain the authoritative checks.
Older scoped manifests omit this field and continue using their historical
`hash-v1` container names; refresh, instance removal and uninstall retain that
scheme rather than renaming live containers.

An update keeps the UUID, ownership manifest, release pins and an existing
Compose project unchanged. The same authenticated source revision, role build
fingerprints and runtime-data digest reuse the same per-role suffix, including
after successful publication. Repeating that unchanged update reuses the
existing immutable release instead of creating another tag or release.
Changed inputs receive new reservations; failed builds/publications retain
their reservations for retries. Reservations are marked complete only after
publication and image ownership recording succeed. Existing release tags and
pinned image IDs remain valid and are never retagged or removed merely to
shorten a name. A legacy UUID-scoped installation receives readable tags on
its next generated build without renaming its live project; this avoids
orphaning running instances.
Ownership, publication, instance lifecycle, connection-manager enrollment,
uninstall and image cleanup all validate the exact project/labels and accept
both the historical UUID tags and the reserved readable form.

## Operational constraints

1. Names are never credentials, release keys, or ownership proof. A display name
   is accepted only alongside the exact UUID, manifest, project, Docker labels,
   and build fingerprint checks.
2. Reservations survive failed builds and retired versions. The same identity
   reuses its name; a different identity cannot inherit an old name.
3. Do not rebuild simply to rename, and do not prune or wildcard-delete images.
   Cleanup remains reference-aware and preserves running/pinned resources.
4. The connection manager stores exact canonical installation/project identity;
   it does not ask operators to type a raw image tag as proof.

Live Docker and multi-host acceptance still require a disposable environment;
this change does not mutate an existing user's daemon during tests.
# Connection manager selection

The connection manager queries the explicitly configured installation/bin path
with **Find installation and releases**, then presents installation and release
choices. Remote queries use the pinned SSH endpoint; local queries are limited
to the installation mounted by the manager. This is not a machine-wide scan.
UUID/project values are stored automatically, and release selections retain the
full immutable key internally. Each role card shows only that role's alias;
aliases from other roles are not offered as choices. A newly queried release is
not activated or automatically selected. Existing saved selections remain
intact until edited.
