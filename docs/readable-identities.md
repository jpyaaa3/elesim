# Readable installation and release identities — implementation checkpoint

Status: foundation only; **not enabled in generated installations**. Existing
Docker resources have not been renamed, rebuilt, retagged or removed.

## Agreed behavior

- Actual installation name: `quiet_otter`; fresh project: `elesim-quiet_otter`.
- Actual image reference: `elesim/sim:quiet_otter-calm_eagle`.
- Actual release directory/CLI selector: `silver_pigeon`.
- Names are two familiar English words. Collision means retry, never overwrite.
- Store UUIDs, build fingerprints, image IDs and release content digests as
  internal ownership/provenance evidence; names are not credentials.
- Persist reservations, including failed builds and retired versions. Same
  identity reuses its name; a different identity cannot inherit an old name.
- Do not rebuild simply to rename. Preserve old pinned releases, containers,
  ownership manifests and legacy projects. Existing projects must not change
  implicitly during update.

## Implemented foundation

`readable_names.py` reserves identity-to-name mappings under an explicit registry
path with file locking and atomic publication. It rejects symlinks, malformed
registries and duplicate reservations. `image_reference` can render an explicitly
reserved pair of names while still validating the internal UUID and fingerprint.
Its existing callers continue to generate the old hash tags.

## Remaining integration (required before claiming completion)

1. Add versioned persisted installation-name metadata and engine-scoped name
   reservation. Collision checks must include other prefixes and Docker owners,
   and must remain safe across concurrent installers and separate host accounts.
2. Wire installer image selection to persisted per-role fingerprint reservations.
   Verify exact owner/fingerprint labels before reusing or assigning a Docker tag.
3. Add versioned named release manifests with separate full content digests;
   preserve old manifest parsing, old paths and old instance pins. Resolve the
   readable CLI selector to exact verified release content, not an arbitrary tag.
4. Update all consumers together: publication evidence, instance registration and
   lifecycle, Compose project validation, remote identity enrollment, maintenance
   bundle, image cleanup, uninstall and update wrapper validation. In particular,
   do not relax ownership regexes without retaining full metadata checks.
5. Replace manager raw-ID inputs with discovered, confirmed installation/release
   choices. Store exact canonical identity, not merely a potentially colliding
   display string.
6. Test new installs, unchanged and changed updates, legacy fixtures, name
   collisions, concurrent reservations, corrupted mappings, foreign tags, pinned
   running/stopped instances and isolated maintenance packaging. A Docker smoke
   must use disposable resources, never the existing user's runtime containers.

No push or live installation mutation has been performed for this work.
