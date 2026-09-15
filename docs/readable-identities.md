# Readable installation and release identities

Fresh container installations and subsequent updates use short, human-readable
names in the Compose project and immutable image tags. Full UUIDs, build
fingerprints, image IDs and release content digests remain the ownership and
provenance evidence; the readable names are presentation identifiers only.

## Agreed behavior

- Actual installation name: `quiet_otter`; fresh project: `elesim-quiet_otter`.
- Actual image reference: `elesim/sim:quiet_otter-calm_eagle`.
- Actual release directory/CLI selector: `silver_pigeon`.
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
UUID and one installation-wide image name per role/fingerprint, then emits tags such as
`elesim/sim:quiet_otter-calm_eagle` and a project such as
`elesim-quiet_otter`.

The installation-wide image namespace covers Pilot, Sim, UI, tools and the
optional development image, so two roles in one installation cannot receive
the same new alias. Registries from older releases used role-specific scopes;
those historical bindings remain readable, including a collision if one was
already published, while any replacement binding is allocated from the shared
scope without rewriting the old immutable tag.

An update keeps the UUID, ownership manifest, release pins and an existing
Compose project unchanged. A legacy UUID-scoped installation receives a
readable tag on its next generated build without renaming its live project;
this avoids orphaning running instances. Existing release tags and pinned image
IDs remain valid and are never retagged or removed merely to shorten a name.
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
full immutable key internally. A newly queried release is not activated or
automatically selected. Existing saved selections remain intact until edited.
