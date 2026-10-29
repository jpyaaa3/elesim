"""Execute the shipped lookup/probe functions with a minimal browser boundary."""

from pathlib import Path
import shutil
import subprocess

import pytest


def test_runtime_status_during_jobs():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js is required for the frontend behavioral check")
    subprocess.run([
        node, "--test", str(Path(__file__).with_name("connection_runtime_status.test.cjs")),
    ], check=True)


def test_inherit_policy_arrival_restores_default_without_overwriting_user_choice():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js is required for the frontend behavioral check")
    script = Path(__file__).resolve().parents[3] / (
        "payload/runtime/docker/setup/app/elesim_connections/"
        "connection_manager_web/app.js"
    )
    subprocess.run([node, "-e", r'''
const fs = require("node:fs"), vm = require("node:vm");
const assert = require("node:assert/strict");
const source = fs.readFileSync(process.argv[1], "utf8");
const controls = {"pilot-gpu-inherit": {checked: false}, "sim-gpu-inherit": {checked: false}};
const context = vm.createContext({
  gpuPolicies: {pilot: {mode: "unknown"}, sim: {mode: "cpu"}},
  gpuDevices: {}, byId: id => controls[id], updateRuntimeOptions() {}, updateGpuDeviceOptions() {}
});
vm.runInContext(source.match(/function applyRuntimeGpuPolicies\([^]*?\n}/)[0], context);
const hosts = [{roles: ["pilot", "sim"], gpu_policy: {
  pilot: {mode: "inherit"}, sim: {mode: "inherit"}
}}];
context.applyRuntimeGpuPolicies(hosts);
assert.equal(controls["pilot-gpu-inherit"].checked, true);
assert.equal(controls["sim-gpu-inherit"].checked, true);
controls["pilot-gpu-inherit"].checked = false;
context.applyRuntimeGpuPolicies(hosts);
assert.equal(controls["pilot-gpu-inherit"].checked, false);
''', str(script)], check=True)


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
const names = ["installationLookupReady", "installationLookupReadyFor", "updateInstallationLookup",
  "updateInstallationLookupFor", "updateInstallationLookupButtons", "installationOptionLabel", "probeSsh",
  "bindHostCardEvents"];
let local = "other", active = true, host = "old", port = 22, resolve;
const fields = {"ssh-fingerprint": {value: ""}, "ssh-tailscale": {checked: false},
  "ssh-user": {value: "operator"}};
const button = {dataset: {}, setAttribute(k, v) {this[k] = v;}};
const robotButton = {dataset: {}, setAttribute(k, v) {this[k] = v;}};
let hostCard = {querySelector: selector => selector === ".robot-lookup-installation" ? robotButton : button};
let confirmations = 0;
const context = vm.createContext({
  slots: ["com1"], isRobotHost: () => true,
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
  context.updateInstallationLookupButtons();
  assert.equal(button.disabled, true);
  assert.equal(robotButton.disabled, true);
  assert.equal(button.textContent, "install.lookup.blocked");
  assert.equal(context.installationOptionLabel({install_mode: "native", name: "Robot (/opt/robot)"}),
    "install.native - Robot (/opt/robot)");
  local = "com1";
  context.updateInstallationLookupButtons();
  assert.equal(button.disabled, false);
  assert.equal(robotButton.disabled, false);
  button.dataset.lookupBusy = "true";
  context.updateInstallationLookup("com1");
  assert.equal(button["aria-disabled"], "true");
  delete button.dataset.lookupBusy;
  local = "other";
  fields["ssh-user"].value = "";
  await assert.rejects(context.probeSsh("com1"), /error.ssh.user.required/);
  assert.equal(confirmations, 0);
  fields["ssh-user"].value = "operator";
  for (const change of [() => host = "new", () => port = 23,
      () => fields["ssh-user"].value = "different",
      () => fields["ssh-tailscale"].checked = true,
      () => hostCard = {...hostCard},
      () => local = "com1", () => active = false, () => host = ""]) {
    host = "old"; port = 22; local = "other"; active = true;
    fields["ssh-tailscale"].checked = false;
    fields["ssh-user"].value = "operator";
    const pending = context.probeSsh("com1");
    change(); resolve({fingerprint: "stale"}); await pending;
    assert.equal(fields["ssh-fingerprint"].value, "");
    assert.equal(confirmations, 0);
  }
  host = "old"; active = true;
  context.updateInstallationLookupButtons();
  assert.equal(robotButton.disabled, true);
  const pending = context.probeSsh("com1");
  resolve({fingerprint: "verified"}); await pending;
  assert.equal(fields["ssh-fingerprint"].value, "verified");
  assert.equal(confirmations, 1);
  assert.equal(button.disabled, false);
  assert.equal(robotButton.disabled, false, "confirmed SSH must unlock the native Robot lookup");
  robotButton.dataset.lookupBusy = "true";
  context.updateInstallationLookupButtons();
  assert.equal(button.disabled, false);
  assert.equal(robotButton.disabled, true, "a Robot lookup keeps its own busy state");
  delete robotButton.dataset.lookupBusy;

  // Dispatch the shipped field handlers in their registered order: clearing
  // cached choices runs before host/port/mode edits invalidate the fingerprint.
  const listeners = new Map();
  const controls = new Map();
  function control(key) {
    if (!controls.has(key)) controls.set(key, {addEventListener(event, handler) {
      const handlers = listeners.get(`${key}:${event}`) || [];
      handlers.push(handler);
      listeners.set(`${key}:${event}`, handlers);
    }});
    return controls.get(key);
  }
  hostCard = {querySelector: control, querySelectorAll: () => []};
  context.field = (_, name) => Object.assign(control(name), fields[name]);
  context.clearAllInstallationLookups = () => context.updateInstallationLookupButtons();
  context.bindDropZone = () => {};
  context.updateSshVisibility = () => {};
  context.updateSshMode = () => {};
  context.bindHostCardEvents("com1");
  for (const [name, event] of [["ssh-host", "input"], ["ssh-port", "input"],
      ["ssh-tailscale", "change"]]) {
    fields["ssh-fingerprint"].value = "verified";
    hostCard.querySelector = selector => selector === ".robot-lookup-installation" ? robotButton : button;
    context.field = (_, name) => fields[name];
    context.updateInstallationLookupButtons();
    assert.equal(robotButton.disabled, false);
    for (const handler of listeners.get(`${name}:${event}`)) handler();
    assert.equal(fields["ssh-fingerprint"].value, "");
    assert.equal(button.disabled, true);
    assert.equal(robotButton.disabled, true, `${name} must relock the Robot lookup`);
  }
})().catch(error => {console.error(error); process.exitCode = 1;});
''', str(script)], check=True)
