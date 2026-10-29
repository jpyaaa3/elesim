// Run with: node --test workbench/tests/setup/connection_runtime_status.test.cjs
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.resolve(__dirname,
  '../../../payload/runtime/docker/setup/app/elesim_connections/connection_manager_web/app.js'), 'utf8');

test('busy jobs hide old state and reject a late pre-start status response', async () => {
  const controls = {'job-status': {dataset: {status: 'idle'}},
    'runtime-status': {textContent: 'com1: unregistered; com2: stopped'}};
  const replies = [];
  const context = vm.createContext({
    runtimeRevision: 0, runtimeReady: true, runtimePollInFlight: false,
    workflowApplied: true, byId: id => controls[id], t: key => key,
    api: () => new Promise(resolve => replies.push(resolve)), updateWorkflow() {},
    renderRuntimeStatus: value => {controls['runtime-status'].textContent = value.state;},
  });
  for (const name of ['renderRuntimeJobStatus', 'pollRuntimeStatus']) {
    vm.runInContext(source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n}`))[0], context);
  }
  const stale = context.pollRuntimeStatus();
  controls['job-status'].dataset.status = 'running';
  context.renderRuntimeJobStatus({status: 'running', action: 'start'});
  assert.match(controls['runtime-status'].textContent, /action.start.*runtime.updating/);
  assert.equal(context.runtimeReady, false);
  replies.shift()({state: 'unregistered'});
  await stale;
  assert.match(controls['runtime-status'].textContent, /runtime.updating/);
  await context.pollRuntimeStatus();
  assert.equal(replies.length, 0);
  controls['job-status'].dataset.status = 'cancelling';
  context.renderRuntimeJobStatus({status: 'cancelling', action: 'start'});
  assert.match(controls['runtime-status'].textContent, /job.cancelling/);
  controls['job-status'].dataset.status = 'completed';
  const fresh = context.pollRuntimeStatus();
  replies.shift()({state: 'running'});
  await fresh;
  assert.equal(controls['runtime-status'].textContent, 'running');
});

for (const status of ['completed', 'failed', 'cancelled']) {
  test(`${status} job refreshes status after the busy placeholder`, async () => {
    const controls = Object.fromEntries(['job-status', 'job-log', 'runtime-status', 'save'].map(
      id => [id, {dataset: {status: 'running'}, textContent: '', focus() {}}]));
    let refreshes = 0;
    const context = vm.createContext({
      byId: id => controls[id], t: key => key,
      api: async () => ({status, action: 'start', logs: []}),
      runtimeRevision: 1, workflowStarted: false, pollTimer: null,
      restoreRuntimeOptions() {}, workflowStepForAction: () => 'start',
      setWorkflowStepState() {}, markWorkflowDirty() {}, setJobRunning() {}, updateWorkflow() {},
      pollRuntimeStatus() {refreshes++;}, showError(error) {throw error;},
    });
    vm.runInContext(source.match(/async function pollJob\([^]*?\n}/)[0], context);
    await context.pollJob();
    assert.equal(controls['runtime-status'].textContent, 'runtime.refreshing');
    assert.equal(refreshes, 1);
    assert.equal(context.workflowStarted, status === 'completed');
  });
}

test('runtime stop completion refreshes status and enables another start', async () => {
  const controls = Object.fromEntries(['job-status', 'job-log', 'runtime-status', 'save'].map(
    id => [id, {dataset: {status: 'running'}, textContent: '', focus() {}}]));
  let refreshes = 0;
  const context = vm.createContext({
    byId: id => controls[id], t: key => key,
    api: async () => ({status: 'completed', action: 'stop', logs: []}),
    runtimeRevision: 1, workflowStarted: true, pollTimer: null,
    restoreRuntimeOptions() {}, workflowStepForAction: () => '',
    setWorkflowStepState() {}, markWorkflowDirty() {}, setJobRunning() {}, updateWorkflow() {},
    pollRuntimeStatus() {refreshes++;}, showError(error) {throw error;},
  });
  vm.runInContext(source.match(/async function pollJob\([^]*?\n}/)[0], context);
  await context.pollJob();
  assert.equal(context.workflowStarted, false);
  assert.equal(refreshes, 1);
});

test('a completed host is shown while another host is still pending', async () => {
  const pending = new Map(), renders = [];
  const context = vm.createContext({
    runtimeRevision: 0, runtimeReady: false, runtimePollInFlight: false,
    workflowApplied: true, byId: () => ({dataset: {status: 'completed'}}),
    api: url => url === '/api/runtime/hosts'
      ? Promise.resolve({hosts: [{host_id: 'fast', state: 'checking'}, {host_id: 'slow', state: 'checking'}]})
      : new Promise(resolve => pending.set(url, resolve)),
    renderRuntimeStatus: value => renders.push(JSON.parse(JSON.stringify(value))),
    updateWorkflow() {}, encodeURIComponent,
  });
  vm.runInContext(source.match(/async function pollRuntimeStatus\([^]*?\n}/)[0], context);
  const polling = context.pollRuntimeStatus();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(pending.size, 2);
  pending.get('/api/runtime?host=fast')({hosts: [{host_id: 'fast', state: 'running'}]});
  await new Promise(resolve => setImmediate(resolve));
  assert.deepEqual(renders.at(-1).hosts.map(host => host.state), ['running', 'checking']);
  pending.get('/api/runtime?host=slow')({hosts: [{host_id: 'slow', state: 'stopped'}]});
  await polling;
  assert.deepEqual(renders.at(-1).hosts.map(host => host.state), ['running', 'stopped']);
});

test('UI GPU selection survives status refreshes independently of Pilot compute', () => {
  const controls = Object.fromEntries(['pilot', 'sim', 'ui'].flatMap(role => [
    [`${role}-gpu-inherit`, {checked: false}], [`${role}-gpu-device`, {value: ''}],
  ]));
  const context = vm.createContext({
    gpuPolicies: {pilot: {mode: 'unknown'}, sim: {mode: 'unknown'}, ui: {mode: 'unknown'}},
    gpuDevices: {}, byId: id => controls[id], updateRuntimeOptions() {}, updateGpuDeviceOptions() {},
  });
  vm.runInContext(source.match(/function applyRuntimeGpuPolicies\([^]*?\n}/)[0], context);
  const host = {roles: ['pilot', 'ui'], gpu_policy: {
    pilot: {mode: 'cpu'}, ui: {mode: 'inherit', selected_mode: 'specific', selected_device: '0'},
  }, gpu_devices: [{index: '0', uuid: 'GPU-display'}]};
  context.applyRuntimeGpuPolicies([host]);
  assert.equal(controls['ui-gpu-inherit'].checked, true);
  assert.equal(controls['ui-gpu-device'].value, '0');
  assert.equal(controls['pilot-gpu-inherit'].checked, false);
  controls['ui-gpu-inherit'].checked = false;
  context.applyRuntimeGpuPolicies([host]);
  assert.equal(controls['ui-gpu-inherit'].checked, false);
});

test('pending host polls preserve draft GPU choices and device inventory', () => {
  const controls = Object.fromEntries(['pilot', 'sim', 'ui'].flatMap(role => [
    [`${role}-gpu-inherit`, {checked: false}], [`${role}-gpu-device`, {value: ''}],
  ]));
  const context = vm.createContext({
    gpuPolicies: {pilot: {mode: 'unknown'}, sim: {mode: 'unknown'}, ui: {mode: 'unknown'}},
    gpuDevices: {}, byId: id => controls[id], updateRuntimeOptions() {}, updateGpuDeviceOptions() {},
  });
  vm.runInContext(source.match(/function applyRuntimeGpuPolicies\([^]*?\n}/)[0], context);
  const host = {roles: ['pilot', 'ui'], gpu_policy: {
    pilot: {mode: 'inherit'}, ui: {mode: 'inherit', selected_mode: 'cpu'},
  }, gpu_devices: [{index: '0', uuid: 'GPU-compute'}, {index: '1', uuid: 'GPU-display'}]};
  context.applyRuntimeGpuPolicies([host]);
  controls['pilot-gpu-inherit'].checked = false;
  controls['ui-gpu-inherit'].checked = true;
  controls['ui-gpu-device'].value = '1';
  context.applyRuntimeGpuPolicies([{roles: host.roles, state: 'checking'}]);
  assert.equal(context.gpuPolicies.pilot.mode, 'inherit');
  assert.equal(context.gpuDevices.ui.length, 2);
  context.applyRuntimeGpuPolicies([host]);
  assert.equal(controls['pilot-gpu-inherit'].checked, false);
  assert.equal(controls['ui-gpu-inherit'].checked, true);
  assert.equal(controls['ui-gpu-device'].value, '1');
  context.applyRuntimeGpuPolicies([{roles: host.roles, state: 'unreachable'}]);
  assert.equal(context.gpuPolicies.pilot.mode, 'unknown');
  assert.equal(context.gpuDevices.ui.length, 0);
});
