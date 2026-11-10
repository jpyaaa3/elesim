// Exercise the real page JavaScript and HTTP API without adding a browser dependency.
// DOM strings are checked; this is not a screenshot/layout acceptance test.
'use strict';
const vm = require('node:vm');
const assert = require('node:assert/strict');
const base = process.argv[2];
class Element {
  constructor(id = '') { this.id = id; this.value = ''; this.innerHTML = ''; this.textContent = ''; this.disabled = false;
    this.style = {}; this.children = []; this.attributes = {}; this.classList = { contains: () => false }; }
  setAttribute(k, v) { this.attributes[k] = v; }
  appendChild(v) { this.children.push(v); }
}
(async () => {
  const html = await (await fetch(base)).text();
  assert.doesNotMatch(html, /recordMu|기록 μ 하한/);
  const elements = new Map();
  const get = id => { if (!elements.has(id)) elements.set(id, new Element(id)); return elements.get(id); };
  for (const input of html.matchAll(/<input\b[^>]*\bid="([^"]+)"[^>]*>/g)) {
    const val = input[0].match(/\bvalue="([^"]*)"/); get(input[1]).value = val ? val[1] : '';
  }
  get('shape').value = 'circle'; get('iterations').value = '16';
  get('width').value = '100';
  const context = vm.createContext({
    document: { getElementById: get, querySelectorAll: () => [], createElement: () => new Element() },
    console, setTimeout, clearTimeout, Blob, URL, Date, Math, JSON, Number,
    fetch: (path, options) => fetch(new URL(path, base), options),
  });
  vm.runInContext(html.split('<script>')[1].split('</script>')[0], context);
  await get('collectMap').onclick();
  const deadline = Date.now() + 15000;
  while (vm.runInContext('running', context) && Date.now() < deadline) await new Promise(r => setTimeout(r, 30));
  assert.equal(vm.runInContext('running', context), false, 'collection must finish');
  assert.notEqual(get('status').className, 'error', get('status').textContent);
  const originalCount = vm.runInContext('result.needle_map.needles.length', context);
  assert(originalCount > 0 && originalCount <= 16);
  assert(vm.runInContext('result.needle_map.needles.every(n => n.friction.mu_critical < .5)', context));
  assert.match(get('plot').innerHTML, /data-needle="0"/);
  assert.equal(get('mapView').disabled, false);
  get('gatheredView').onclick();
  assert.match(get('plot').innerHTML, /끌어모은 바늘/);
  assert.match(get('mapStatus').textContent, /band/);
  get('mu').value = '0';
  await get('mu').oninput();
  assert.equal(vm.runInContext('result.candidates.length + result.needle_map.needles.length + result.needle_map.gathered.needles.length', context), 0);
  assert.doesNotMatch(get('plot').innerHTML, /data-needle=/);
  get('mu').value = '.5';
  await get('mu').oninput();
  assert.equal(vm.runInContext('result.needle_map.needles.length', context), originalCount);
  // A late response to an older limit must not restore forbidden candidates.
  get('mu').value = '0'; const older = get('mu').oninput();
  get('mu').value = '.5'; const newer = get('mu').oninput();
  await Promise.all([older, newer]);
  assert.equal(vm.runInContext('result.settings.operating_mu', context), .5);
  assert.equal(vm.runInContext('cutoffPending', context), false);
  get('bandSupport').value = '60';
  await get('autoBand').onclick();
  assert.equal(vm.runInContext('result.needle_map.gathered.consensus.support_fraction', context), .6);
  get('bandInner').value = '';
  await get('gather').onclick();
  assert.equal(get('status').className, 'error', 'partial manual band input must fail visibly');
  get('armView').onclick();
  assert.match(get('plot').innerHTML, /ZED Mini/);
  console.log('UI JavaScript: collect, raw map, band gather, global cutoff, stale replies, invalid input, arm view passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
