"""Execute the shipped lookup/probe functions with a minimal browser boundary."""

from pathlib import Path
import shutil
import subprocess

import pytest


def test_lookup_gate_and_stale_ssh_probe():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js is required for the frontend behavioral check")
    script = Path(__file__).resolve().parents[3] / (
        "payload/runtime/docker/setup/app/elesim_connections/"
        "connection_manager_web/app.js"
    )
    subprocess.run([node, "-e", r'''
const fs = require("node:fs");
const vm = require("node:vm");
const assert = require("node:assert/strict");
const source = fs.readFileSync(process.argv[1], "utf8");
const names = ["installationLookupReady", "updateInstallationLookup", "probeSsh"];
let local = "other", active = true, host = "old", port = 22, resolve;
const fields = {"ssh-fingerprint": {value: ""}, "ssh-tailscale": {checked: false}};
const button = {dataset: {}, setAttribute(k, v) {this[k] = v;}};
let hostCard = {querySelector: () => button};
let confirmations = 0;
const context = vm.createContext({
  document: {querySelector: () => ({value: local})},
  field: (_, name) => fields[name], card: () => hostCard,
  isActive: () => active, t: x => x, showNotice: () => {},
  sshHostFromForm: () => {if (!host) throw Error("empty"); return host;},
  sshPort: () => port,
  api: () => new Promise(r => {resolve = r;}),
  window: {confirm: () => {confirmations++; return true;}}
});
for (const name of names) {
  const match = source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n}`));
  assert.ok(match, name);
  vm.runInContext(match[0], context);
}
(async () => {
  context.updateInstallationLookup("com1");
  assert.equal(button.disabled, true);
  assert.equal(button.textContent, "install.lookup.blocked");
  local = "com1";
  context.updateInstallationLookup("com1");
  assert.equal(button.disabled, false);
  button.dataset.lookupBusy = "true";
  context.updateInstallationLookup("com1");
  assert.equal(button["aria-disabled"], "true");
  delete button.dataset.lookupBusy;
  local = "other";
  for (const change of [() => host = "new", () => port = 23,
      () => fields["ssh-tailscale"].checked = true,
      () => hostCard = {querySelector: () => button},
      () => local = "com1", () => active = false, () => host = ""]) {
    host = "old"; port = 22; local = "other"; active = true;
    fields["ssh-tailscale"].checked = false;
    const pending = context.probeSsh("com1");
    change(); resolve({fingerprint: "stale"}); await pending;
    assert.equal(fields["ssh-fingerprint"].value, "");
    assert.equal(confirmations, 0);
  }
  host = "old"; active = true;
  const pending = context.probeSsh("com1");
  resolve({fingerprint: "verified"}); await pending;
  assert.equal(fields["ssh-fingerprint"].value, "verified");
  assert.equal(confirmations, 1);
  assert.equal(button.disabled, false);
})().catch(error => {console.error(error); process.exitCode = 1;});
''', str(script)], check=True)
