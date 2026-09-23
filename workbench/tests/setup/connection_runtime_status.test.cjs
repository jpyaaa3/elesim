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
