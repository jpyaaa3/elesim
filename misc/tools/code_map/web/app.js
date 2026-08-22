"use strict";

const $ = id => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";
const NODE_WIDTH = 184, NODE_HEIGHT = 44, MODULE_WIDTH = 820;
const MODULE_GAP = 104, ROLE_GAP = 180, ROLE_PADDING = 76;
const ROLE_WIDTH = ROLE_PADDING * 2 + MODULE_WIDTH * 2 + MODULE_GAP;
const MIN_ZOOM = 0.06, MAX_ZOOM = 5;
let snapshot = null, observed = [], selected = null, dragging = null, pan = null, layoutBounds = null;
let transform = {x: 0, y: 0, k: 1};
const manualPositions = new Map();
const token = new URLSearchParams(location.search).get("token") || "";

async function api(path) {
  const response = await fetch(path, {headers: {"X-Code-Map-Token": token}});
  if (!response.ok) throw new Error((await response.json()).error || response.statusText);
  return response.json();
}
function option(select, value, text = value) {
  const element = document.createElement("option");
  element.value = value; element.textContent = text; select.append(element);
}
function initFilters() {
  [...new Set(snapshot.nodes.map(node => node.role))].sort().forEach(value => option($("role"), value));
  snapshot.workflows.forEach(workflow => option($("workflow"), workflow.id, `${workflow.title} (${Math.round(workflow.coverage * 100)}%)`));
}
function visible() {
  const role = $("role").value, change = $("change").value;
  const query = $("search").value.toLowerCase(), depth = $("depth").value;
  const workflow = snapshot.workflows.find(item => item.id === $("workflow").value);
  const allowed = workflow ? new Set(workflow.nodes) : null;
  const kinds = depth === "module" ? new Set(["role", "package", "module", "semantic", "entrypoint"])
    : depth === "class" ? new Set(["role", "package", "module", "class", "semantic", "entrypoint"]) : null;
  const rank = {role: 0, package: 1, semantic: 2, entrypoint: 3, module: 4, class: 5, function: 6, method: 6};
  let nodes = snapshot.nodes.filter(node => (!role || node.role === role) && (!change || node.change === change)
    && (!query || `${node.qualname} ${node.path}`.toLowerCase().includes(query)) && (!kinds || kinds.has(node.kind))
    && (!allowed || allowed.has(node.id)));
  nodes.sort((a, b) => (rank[a.kind] ?? 9) - (rank[b.kind] ?? 9)
    || (a.change === "unchanged") - (b.change === "unchanged") || a.id.localeCompare(b.id));
  nodes = nodes.slice(0, 300);
  return [nodes, new Set(nodes.map(node => node.id))];
}

function roleLayout(role, nodes) {
  const modules = new Map();
  nodes.forEach(node => {
    const key = node.path || "__overview__";
    if (!modules.has(key)) modules.set(key, []);
    modules.get(key).push(node);
  });
  const entries = [...modules.entries()].sort(([a], [b]) => a === "__overview__" ? -1 : b === "__overview__" ? 1 : a.localeCompare(b));
  const columnBottom = [116, 116], positions = new Map(), blocks = [];
  entries.forEach(([path, group], index) => {
    const columns = Math.min(3, Math.max(1, Math.ceil(Math.sqrt(group.length * 1.4))));
    const rows = Math.ceil(group.length / columns);
    const height = 104 + rows * NODE_HEIGHT + Math.max(0, rows - 1) * 42 + 44;
    const column = index === 0 ? 0 : columnBottom[0] <= columnBottom[1] ? 0 : 1;
    const x = ROLE_PADDING + column * (MODULE_WIDTH + MODULE_GAP), y = columnBottom[column];
    blocks.push({kind: "module", role, path, x, y, width: MODULE_WIDTH, height});
    const contentWidth = MODULE_WIDTH - 112;
    const stepX = columns === 1 ? 0 : (contentWidth - NODE_WIDTH) / (columns - 1);
    group.forEach((node, nodeIndex) => {
      const col = nodeIndex % columns, row = Math.floor(nodeIndex / columns);
      positions.set(node.id, {x: x + 56 + NODE_WIDTH / 2 + col * stepX,
        y: y + 86 + NODE_HEIGHT / 2 + row * (NODE_HEIGHT + 42)});
    });
    columnBottom[column] += height + MODULE_GAP;
  });
  return {role, width: ROLE_WIDTH, height: Math.max(...columnBottom) - MODULE_GAP + ROLE_PADDING, positions, blocks};
}

function layout(nodes) {
  const roleGroups = [...new Set(nodes.map(node => node.role))].sort()
    .map(role => roleLayout(role, nodes.filter(node => node.role === role)));
  const positions = new Map(), blocks = [], rowBottom = [];
  roleGroups.forEach((group, index) => {
    const column = index % 2, row = Math.floor(index / 2);
    const x = column * (ROLE_WIDTH + ROLE_GAP), y = row === 0 ? 0 : rowBottom[row - 1] + ROLE_GAP;
    rowBottom[row] = Math.max(rowBottom[row] || 0, y + group.height);
    blocks.push({kind: "role", role: group.role, x, y, width: group.width, height: group.height});
    group.blocks.forEach(block => blocks.push({...block, x: block.x + x, y: block.y + y}));
    group.positions.forEach((position, id) => positions.set(id, manualPositions.get(id) || {x: position.x + x, y: position.y + y}));
  });
  const width = roleGroups.length > 1 ? ROLE_WIDTH * 2 + ROLE_GAP : ROLE_WIDTH;
  return {positions, blocks, bounds: {x: 0, y: 0, width, height: rowBottom.at(-1) || 500}};
}

function svg(name, attrs = {}) {
  const element = document.createElementNS(NS, name);
  Object.entries(attrs).forEach(([key, value]) => element.setAttribute(key, value));
  return element;
}
function blockLabel(block) {
  if (block.kind === "role") return block.role.toUpperCase();
  return block.path === "__overview__" ? "Overview / contracts" : block.path;
}
function renderBlocks(blocks) {
  const fragment = document.createDocumentFragment();
  blocks.forEach(block => {
    const group = svg("g", {class: `map-block ${block.kind}-block`});
    group.append(svg("rect", {x: block.x, y: block.y, width: block.width, height: block.height, rx: block.kind === "role" ? 24 : 15}));
    const label = svg("text", {x: block.x + 28, y: block.y + (block.kind === "role" ? 48 : 38)});
    label.textContent = blockLabel(block); group.append(label); fragment.append(group);
  });
  $("groups").replaceChildren(fragment);
}

function render(fit = false) {
  if (!snapshot) return;
  const [nodes, ids] = visible(), result = layout(nodes);
  layoutBounds = result.bounds;
  const edges = [...snapshot.edges, ...observed].filter(edge => ids.has(edge.source) && ids.has(edge.target)
    && ((edge.evidence === "observed" && $("observedEdges").checked) || (edge.evidence !== "observed" && $("staticEdges").checked)));
  renderBlocks(result.blocks); $("nodes").replaceChildren(); $("edges").replaceChildren();
  edges.slice(0, 1200).forEach(edge => {
    const source = result.positions.get(edge.source), target = result.positions.get(edge.target);
    if (source && target) $("edges").append(svg("line", {x1: source.x, y1: source.y, x2: target.x, y2: target.y, class: `edge ${edge.evidence}`}));
  });
  nodes.forEach(node => {
    const position = result.positions.get(node.id);
    const group = svg("g", {class: `node ${node.kind} ${node.change} ${selected === node.id ? "selected" : ""}`,
      transform: `translate(${position.x} ${position.y})`});
    group.dataset.id = node.id;
    group.append(svg("rect", {x: -NODE_WIDTH / 2, y: -NODE_HEIGHT / 2, width: NODE_WIDTH, height: NODE_HEIGHT, rx: 8}));
    const name = svg("text", {class: "node-name", x: -NODE_WIDTH / 2 + 12, y: 1});
    name.textContent = node.name.length > 27 ? `${node.name.slice(0, 25)}…` : node.name; group.append(name);
    const kind = svg("text", {class: "node-kind", x: -NODE_WIDTH / 2 + 12, y: 15});
    kind.textContent = node.kind; group.append(kind);
    group.addEventListener("click", event => { event.stopPropagation(); show(node); });
    group.addEventListener("pointerdown", event => { event.stopPropagation(); dragging = {group, position, id: node.id, x: event.clientX, y: event.clientY}; group.setPointerCapture(event.pointerId); });
    group.addEventListener("pointermove", event => {
      if (dragging?.group !== group) return;
      position.x += (event.clientX - dragging.x) / transform.k; position.y += (event.clientY - dragging.y) / transform.k;
      dragging.x = event.clientX; dragging.y = event.clientY;
      manualPositions.set(node.id, {x: position.x, y: position.y}); group.setAttribute("transform", `translate(${position.x} ${position.y})`);
    });
    group.addEventListener("pointerup", () => { dragging = null; render(); });
    $("nodes").append(group);
  });
  $("summary").textContent = `${nodes.length}/${snapshot.stats.nodes} nodes · ${edges.length} edges`;
  if (fit) fitGraph(); else applyTransform();
}

function applyTransform() { $("scene").setAttribute("transform", `translate(${transform.x} ${transform.y}) scale(${transform.k})`); }
function fitGraph() {
  if (!layoutBounds) return;
  const viewport = $("viewport").getBoundingClientRect(), padding = 72;
  const scale = Math.max(MIN_ZOOM, Math.min(1.2, (viewport.width - padding * 2) / Math.max(1, layoutBounds.width),
    (viewport.height - padding * 2) / Math.max(1, layoutBounds.height)));
  transform = {k: scale, x: (viewport.width - layoutBounds.width * scale) / 2 - layoutBounds.x * scale,
    y: (viewport.height - layoutBounds.height * scale) / 2 - layoutBounds.y * scale};
  applyTransform();
}
function zoomAt(clientX, clientY, nextScale) {
  const bounds = $("graph").getBoundingClientRect();
  const screenX = clientX - bounds.left, screenY = clientY - bounds.top;
  const worldX = (screenX - transform.x) / transform.k, worldY = (screenY - transform.y) / transform.k;
  transform = {k: nextScale, x: screenX - worldX * nextScale, y: screenY - worldY * nextScale};
  applyTransform();
}

async function show(node) {
  selected = node.id; render(); $("detail").querySelector("h2").textContent = node.qualname;
  const incoming = snapshot.edges.filter(edge => edge.target === node.id).length;
  const outgoing = snapshot.edges.filter(edge => edge.source === node.id).length;
  $("meta").innerHTML = `<b>${node.kind}</b><br>${node.role}<br>${node.path}:${node.line}<br>${node.change}<br>callers ${incoming} / callees ${outgoing}<br>${escapeHtml(JSON.stringify(node.detail, null, 2))}`;
  if (!node.path) { $("source").textContent = ""; $("diff").textContent = ""; return; }
  try {
    const [source, diff] = await Promise.all([api(`/api/source?path=${encodeURIComponent(node.path)}&line=${node.line}`), api(`/api/diff?path=${encodeURIComponent(node.path)}`)]);
    $("source").textContent = `${source.start}-${source.end}\n${source.text}`; $("diff").textContent = diff.text || "HEAD 대비 변경 없음";
  } catch (error) { $("source").textContent = String(error); }
}
function escapeHtml(value) { return value.replace(/[&<>]/g, character => ({"&": "&amp;", "<": "&lt;", ">": "&gt;"}[character])); }
async function traces() {
  const data = await api("/api/traces"), byName = new Map(snapshot.nodes.map(node => [node.name.toLowerCase(), node.id]));
  observed = [];
  for (const span of data.spans || []) {
    if (span.error) continue;
    const ids = span.name.toLowerCase().split(/[^a-z0-9_]+/).map(word => byName.get(word)).filter(Boolean);
    for (let index = 1; index < ids.length; index++) observed.push({source: ids[index - 1], target: ids[index], kind: "trace", evidence: "observed", confidence: "exact"});
  }
  render(); $("status").textContent = `Jaeger ${data.spans?.length || 0} spans`;
}
async function load() {
  snapshot = await api("/api/snapshot");
  $("status").textContent = `${snapshot.git_head.slice(0, 8)} · ${new Date(snapshot.generated_at).toLocaleTimeString()}`;
  if (!$("role").dataset.ready) { initFilters(); $("role").dataset.ready = "1"; }
  render(true);
}

["search", "role", "depth", "change", "workflow", "staticEdges", "observedEdges"].forEach(id => $(id).addEventListener("input", () => render(true)));
$("traces").addEventListener("click", () => traces().catch(error => $("status").textContent = String(error)));
$("fit").addEventListener("click", fitGraph);
$("graph").addEventListener("wheel", event => {
  event.preventDefault();
  const delta = event.deltaMode === WheelEvent.DOM_DELTA_LINE ? event.deltaY * 16 : event.deltaY;
  zoomAt(event.clientX, event.clientY, Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, transform.k * Math.exp(-delta * .0012))));
}, {passive: false});
$("graph").addEventListener("pointerdown", event => {
  if (event.button !== 0 || event.target.closest?.(".node")) return;
  pan = {x: event.clientX, y: event.clientY, tx: transform.x, ty: transform.y}; $("graph").setPointerCapture(event.pointerId);
});
$("graph").addEventListener("pointermove", event => {
  if (!pan) return;
  transform.x = pan.tx + event.clientX - pan.x; transform.y = pan.ty + event.clientY - pan.y; applyTransform();
});
$("graph").addEventListener("pointerup", () => { pan = null; });
$("graph").addEventListener("pointercancel", () => { pan = null; dragging = null; });
load().catch(error => $("status").textContent = String(error));
const events = new EventSource(`/api/events?token=${encodeURIComponent(token)}`);
events.addEventListener("snapshot", event => { if (snapshot && event.data !== snapshot.digest) load(); });
