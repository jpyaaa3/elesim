"use strict";

const $ = id => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";
const NODE_WIDTH = 230, NODE_HEIGHT = 62;
const MIN_ZOOM = 0.06, MAX_ZOOM = 5, FLOW_BUDGET = 700;
let snapshot = null, graph = {nodes: [], edges: []}, observed = [], selected = null, uiMap = null;
let traceSpans = [], observedOrder = new Map(), observedErrors = new Set();
let dragging = null, pan = null, layoutBounds = null, layoutToken = 0, suppressClick = false;
let transform = {x: 0, y: 0, k: 1};
const manualPositions = new Map();
const expanded = new Set();
const token = new URLSearchParams(location.search).get("token") || "";

async function api(path) {
  const response = await fetch(path, {headers: {"X-Code-Map-Token": token}});
  if (!response.ok) throw new Error((await response.json()).error || response.statusText);
  return response.json();
}
function option(select, value, text = value) {
  const element = document.createElement("option"); element.value = value; element.textContent = text; select.append(element);
}
function initFilters() {
  [...new Set(snapshot.nodes.map(node => node.role))].sort().forEach(value => option($("role"), value));
  [...new Set((snapshot.flows || []).map(flow => flow.family))].sort().forEach(value => option($("flow-family"), value));
  refreshFlows();
}
function refreshFlows() {
  const family = $("flow-family").value, select = $("workflow"), old = select.value;
  select.replaceChildren(); option(select, "", "전체 플로우");
  (snapshot.flows || []).filter(flow => !family || flow.family === family).forEach(flow => {
    const gap = flow.gaps?.length ? " · gap" : "";
    option(select, flow.id, `${flow.title} [${flow.kind}${gap}]`);
  });
  if ([...select.options].some(item => item.value === old)) select.value = old;
}
function nodeKinds() {
  const depth = $("depth").value;
  return depth === "module" ? new Set(["role", "package", "module", "semantic", "entrypoint", "external"])
    : depth === "class" ? new Set(["role", "package", "module", "class", "semantic", "entrypoint", "external"]) : null;
}
function isTestNode(node) {
  const parts = String(node.path || "").toLowerCase().split("/");
  const filename = parts.at(-1) || "";
  return parts.includes("test") || parts.includes("tests") || filename.startsWith("test_") || filename.endsWith("_test.py");
}
function withoutTests(source) {
  if (!$("hideTests").checked) return source;
  const nodes = source.nodes.filter(node => !isTestNode(node));
  const ids = new Set(nodes.map(node => node.id));
  return {nodes, edges: source.edges.filter(edge => ids.has(edge.source) && ids.has(edge.target))};
}
function structuralGraph() {
  const role = $("role").value, change = $("change").value, query = $("search").value.toLowerCase(), kinds = nodeKinds(), hideTests = $("hideTests").checked;
  let nodes = snapshot.nodes.filter(node => (!role || node.role === role) && (!change || node.change === change)
    && (!query || `${node.qualname} ${node.path}`.toLowerCase().includes(query)) && (!kinds || kinds.has(node.kind))
    && (!hideTests || !isTestNode(node)));
  nodes.sort((a, b) => (a.role + a.kind + a.id).localeCompare(b.role + b.kind + b.id)); nodes = nodes.slice(0, 700);
  const ids = new Set(nodes.map(node => node.id));
  return {nodes, edges: snapshot.edges.filter(edge => ids.has(edge.source) && ids.has(edge.target)).slice(0, 3000)};
}
function svg(name, attrs = {}) {
  const element = document.createElementNS(NS, name); Object.entries(attrs).forEach(([key, value]) => element.setAttribute(key, value)); return element;
}
function nodeLabel(node) { const label = node.name || node.qualname || node.id; return label.length > 30 ? `${label.slice(0, 28)}…` : label; }
function flowLayoutInput(nodes, edges, directed = false) {
  const ordered = directed
    ? [...nodes].sort((a, b) => (Number(a.flow_depth ?? 0) - Number(b.flow_depth ?? 0))
      || (Number(a.flow_order ?? 0) - Number(b.flow_order ?? 0))
      || a.id.localeCompare(b.id))
    : nodes;
  const preferred = directed ? edges.filter(edge => edge.flow_edge) : edges;
  const layoutEdges = preferred.length ? preferred : edges;
  return {
    id: "root",
    children: ordered.map(node => ({
      id: node.id, width: NODE_WIDTH, height: NODE_HEIGHT,
      ports: [{id: `${node.id}:in`, width: 8, height: 8, layoutOptions: {"elk.port.side": "NORTH"}}, {id: `${node.id}:out`, width: 8, height: 8, layoutOptions: {"elk.port.side": "SOUTH"}}],
      layoutOptions: {
        "elk.portConstraints": "FIXED_SIDE",
        "elk.layered.priority": directed ? String(100000 - Number(node.flow_order ?? 0)) : "0",
      },
    })),
    edges: layoutEdges.map((edge, index) => ({
      id: `edge-${index}`,
      sources: [`${edge.source}:out`],
      targets: [`${edge.target}:in`],
      layoutOptions: {"elk.layered.priority": edge.flow_edge ? "100" : "1"},
    })),
    layoutOptions: {
      "elk.algorithm": "layered",
      "elk.direction": "DOWN",
      "elk.edgeRouting": "ORTHOGONAL",
      "elk.layered.layering.strategy": directed ? "LONGEST_PATH" : "NETWORK_SIMPLEX",
      "elk.layered.cycleBreaking.strategy": "GREEDY",
      "elk.layered.considerModelOrder.strategy": directed ? "NODES_AND_EDGES" : "NONE",
      "elk.layered.nodePlacement.strategy": "BRANDES_KOEPF",
      "elk.layered.spacing.nodeNodeBetweenLayers": directed ? "130" : "90",
      "elk.spacing.nodeNode": directed ? "72" : "42",
      "elk.layered.crossingMinimization.strategy": "LAYER_SWEEP",
      "elk.padding": directed ? "110" : "80",
    },
  };
}
function fallbackLayout(nodes, directed = false) {
  const positions = new Map();
  if (directed) {
    const groups = new Map();
    [...nodes].sort((a, b) => Number(a.flow_order ?? 0) - Number(b.flow_order ?? 0)).forEach(node => {
      const depth = Number(node.flow_depth ?? 0);
      if (!groups.has(depth)) groups.set(depth, []);
      groups.get(depth).push(node);
    });
    const depths = [...groups.keys()].sort((a, b) => a - b);
    const minDepth = depths[0] ?? 0;
    let maxWidth = NODE_WIDTH;
    depths.forEach(depth => {
      const items = groups.get(depth);
      items.forEach((node, row) => positions.set(node.id, {
        x: 180 + row * (NODE_WIDTH + 90),
        y: 130 + (depth - minDepth) * (NODE_HEIGHT + 130),
      }));
      maxWidth = Math.max(maxWidth, items.length * (NODE_WIDTH + 90));
    });
    return {
      positions,
      bounds: {x: 0, y: 0, width: Math.max(900, maxWidth + 160), height: Math.max(620, depths.length * (NODE_HEIGHT + 130) + 160)},
    };
  }
  const columns = Math.max(1, Math.ceil(Math.sqrt(nodes.length / 2)));
  nodes.forEach((node, index) => positions.set(node.id, {x: 180 + (index % columns) * (NODE_WIDTH + 90), y: 120 + Math.floor(index / columns) * (NODE_HEIGHT + 54)}));
  return {positions, bounds: {x: 0, y: 0, width: Math.max(900, columns * (NODE_WIDTH + 90)), height: Math.max(620, Math.ceil(nodes.length / columns) * (NODE_HEIGHT + 54))}};
}
// Kept as a compatibility hook for callers that used the old repository-map layout.
function roleLayout(nodes) { return fallbackLayout(nodes); }
async function layout(nodes, edges, directed = false) {
  if (!nodes.length || !window.ELK || nodes.length > FLOW_BUDGET) return fallbackLayout(nodes, directed);
  try {
    const result = await new ELK().layout(flowLayoutInput(nodes, edges, directed)), positions = new Map();
    (result.children || []).forEach(child => positions.set(child.id, manualPositions.get(child.id) || {x: (child.x || 0) + NODE_WIDTH / 2, y: (child.y || 0) + NODE_HEIGHT / 2}));
    return {positions, bounds: {x: 0, y: 0, width: result.width || 1200, height: result.height || 800}};
  } catch (_error) {
    return fallbackLayout(nodes, directed);
  }
}
function applyTransform() { $("scene").setAttribute("transform", `translate(${transform.x} ${transform.y}) scale(${transform.k})`); }
function fitGraph() {
  if (!layoutBounds) return; const viewport = $("viewport").getBoundingClientRect(), padding = 72;
  const scale = Math.max(MIN_ZOOM, Math.min(1.2, (viewport.width - padding * 2) / Math.max(1, layoutBounds.width), (viewport.height - padding * 2) / Math.max(1, layoutBounds.height)));
  transform = {k: scale, x: (viewport.width - layoutBounds.width * scale) / 2, y: (viewport.height - layoutBounds.height * scale) / 2}; applyTransform();
}
function zoomAt(clientX, clientY, nextScale) {
  const bounds = $("graph").getBoundingClientRect(), screenX = clientX - bounds.left, screenY = clientY - bounds.top;
  const worldX = (screenX - transform.x) / transform.k, worldY = (screenY - transform.y) / transform.k;
  transform = {k: nextScale, x: screenX - worldX * nextScale, y: screenY - worldY * nextScale}; applyTransform();
}
function roleLanes(nodes, positions, directed = false) {
  const lanes = new Map();
  nodes.forEach(node => { const position = positions.get(node.id); if (!position) return; const phase = directed ? (node.flow_phase || "process") : ""; const key = `${node.role}:${phase}`; const lane = lanes.get(key) || {role: node.role, phase, minX: position.x, maxX: position.x, minY: position.y, maxY: position.y}; lane.minX = Math.min(lane.minX, position.x); lane.maxX = Math.max(lane.maxX, position.x); lane.minY = Math.min(lane.minY, position.y); lane.maxY = Math.max(lane.maxY, position.y); lanes.set(key, lane); });
  return [...lanes.values()].map(lane => ({...lane, x: lane.minX - NODE_WIDTH, y: lane.minY - NODE_HEIGHT, width: lane.maxX - lane.minX + NODE_WIDTH * 2, height: lane.maxY - lane.minY + NODE_HEIGHT * 2}));
}
function renderLanes(nodes, positions, directed = false) {
  const fragment = document.createDocumentFragment(); roleLanes(nodes, positions, directed).forEach(lane => { const group = svg("g", {class: `lane phase-${lane.phase || "structure"}`}); group.append(svg("rect", {x: lane.x, y: lane.y, width: lane.width, height: lane.height, rx: 20})); const label = svg("text", {x: lane.x + 24, y: lane.y + 30}); label.textContent = `${lane.role.toUpperCase()}${lane.phase ? ` · ${lane.phase.toUpperCase()}` : ""}`; group.append(label); fragment.append(group); }); $("groups").replaceChildren(fragment);
}
function edgePath(source, target, edge, index = 0) {
  const startY = source.y + NODE_HEIGHT / 2, endY = target.y - NODE_HEIGHT / 2;
  if (!edge.flow_backedge && endY > startY) {
    const middleY = Math.round(((startY + endY) / 2) / 12) * 12;
    return `M ${source.x} ${startY} V ${middleY} H ${target.x} V ${endY}`;
  }
  const sideX = Math.max(source.x, target.x) + NODE_WIDTH / 2 + 56 + (index % 6) * 16;
  return `M ${source.x} ${startY} V ${startY + 24} H ${sideX} V ${endY - 24} H ${target.x} V ${endY}`;
}
function edgeClass(edge, directed = false) {
  return `edge ${edge.evidence || "static"} edge-${edge.kind || "call"} edge-${edge.confidence || "exact"}${directed && edge.flow_edge ? " flow-edge" : ""}${directed && edge.flow_branch ? " flow-branch" : ""}${directed && edge.flow_loop ? " flow-loop" : ""}${directed && edge.flow_merge ? " flow-merge" : ""}${directed && edge.flow_backedge ? " flow-backedge" : ""}`;
}
function renderEdges(nodes, edges, positions, directed = false) {
  const ids = new Set(nodes.map(node => node.id)), fragment = document.createDocumentFragment();
  const all = [...edges, ...observed].filter(edge => ids.has(edge.source) && ids.has(edge.target) && ((edge.evidence === "observed" && $("observedEdges").checked) || (edge.evidence !== "observed" && $("staticEdges").checked)));
  all.slice(0, 5000).forEach((edge, index) => { const source = positions.get(edge.source), target = positions.get(edge.target); if (!source || !target) return; const path = svg("path", {d: edgePath(source, target, edge, index), class: edgeClass(edge, directed), "marker-end": directed && edge.flow_edge ? "url(#flow-arrow)" : "url(#arrow)"}); const title = document.createElementNS(NS, "title"); title.textContent = `${edge.kind || "edge"} · ${edge.confidence || "exact"} · ${edge.path || "runtime"}:${edge.line || ""}`; path.append(title); fragment.append(path); }); $("edges").replaceChildren(fragment); return all.length;
}
function renderNodes(nodes, positions, edges) {
  const fragment = document.createDocumentFragment();
  nodes.forEach(node => {
    const position = positions.get(node.id);
    if (!position) return;
    const flowNode = node.flow_depth !== undefined, traceOrder = observedOrder.get(node.id);
    const group = svg("g", {
      class: `node ${node.kind} ${node.change} ${flowNode ? "flow-node" : ""} ${node.flow_entry ? "flow-entry" : ""} ${observedErrors.has(node.id) ? "observed-error" : ""} ${selected === node.id ? "selected" : ""}`,
      transform: `translate(${position.x} ${position.y})`,
    });
    group.dataset.id = node.id;
    group.append(svg("rect", {x: -NODE_WIDTH / 2, y: -NODE_HEIGHT / 2, width: NODE_WIDTH, height: NODE_HEIGHT, rx: 10}));
    const name = svg("text", {class: "node-name", x: -NODE_WIDTH / 2 + 14, y: -5});
    name.textContent = nodeLabel(node);
    group.append(name);
    const kind = svg("text", {class: "node-kind", x: -NODE_WIDTH / 2 + 14, y: 14});
    kind.textContent = `${node.kind} · ${node.role}${node.flow_phase ? ` · ${node.flow_phase}` : ""}`;
    group.append(kind);
    if (flowNode) {
      const step = svg("text", {class: "flow-step", x: NODE_WIDTH / 2 - 10, y: -NODE_HEIGHT / 2 + 17, "text-anchor": "end"});
      step.textContent = traceOrder !== undefined ? `T${traceOrder + 1}` : node.flow_entry ? "START" : `S${Number(node.flow_depth ?? 0)}`;
      group.append(step);
    }
    const detail = node.detail || {};
    const inputs = (detail.parameter_ports || []).map(port => port.name).join(", ");
    const incoming = edges.some(edge => edge.target === node.id), outgoing = edges.some(edge => edge.source === node.id);
    if (incoming) {
      const port = svg("circle", {class: "port input", cx: 0, cy: -NODE_HEIGHT / 2, r: 5});
      const title = document.createElementNS(NS, "title");
      title.textContent = `input: ${inputs || "message / dependency"}`;
      port.append(title);
      group.append(port);
    }
    if (outgoing) {
      const port = svg("circle", {class: "port output", cx: 0, cy: NODE_HEIGHT / 2, r: 5});
      const title = document.createElementNS(NS, "title");
      title.textContent = `output: ${detail.return_annotation || "return"}`;
      port.append(title);
      group.append(port);
    }
    group.addEventListener("click", event => {
      event.stopPropagation();
      if (suppressClick) { suppressClick = false; return; }
      if (node.kind === "collapsed") {
        expanded.add(node.detail.expand_id);
        selectFlow().catch(error => $("status").textContent = String(error));
      } else {
        show(node);
      }
    });
    group.addEventListener("pointerdown", event => {
      event.stopPropagation();
      suppressClick = false;
      dragging = {group, position, id: node.id, x: event.clientX, y: event.clientY, moved: false};
      group.setPointerCapture(event.pointerId);
    });
    group.addEventListener("pointermove", event => {
      if (dragging?.group !== group) return;
      const dx = event.clientX - dragging.x, dy = event.clientY - dragging.y;
      dragging.moved ||= Math.abs(dx) + Math.abs(dy) > 2;
      position.x += dx / transform.k;
      position.y += dy / transform.k;
      dragging.x = event.clientX;
      dragging.y = event.clientY;
      manualPositions.set(node.id, {x: position.x, y: position.y});
      group.setAttribute("transform", `translate(${position.x} ${position.y})`);
      renderEdges(nodes, edges, positions, Boolean($("workflow").value));
    });
    group.addEventListener("pointerup", () => {
      suppressClick = Boolean(dragging?.moved);
      dragging = null;
    });
    fragment.append(group);
  });
  $("nodes").replaceChildren(fragment);
}
async function render(fit = false) {
  if (!snapshot) return; const tokenAtStart = ++layoutToken, selectedFlow = $("workflow").value, current = selectedFlow ? withoutTests(graph) : structuralGraph(), directed = Boolean(selectedFlow);
  const result = await layout(current.nodes, current.edges, directed); if (tokenAtStart !== layoutToken) return; layoutBounds = result.bounds; renderLanes(current.nodes, result.positions, directed); const edgeCount = renderEdges(current.nodes, current.edges, result.positions, directed); renderNodes(current.nodes, result.positions, current.edges); $("summary").textContent = `${current.nodes.length}/${snapshot.stats.nodes} nodes · ${edgeCount} edges · ${(snapshot.flows || []).length} flows${directed ? " · directed" : ""}`; if (fit) fitGraph(); else applyTransform();
}
async function selectFlow() {
  const id = $("workflow").value; manualPositions.clear(); selected = null;
  if (!id) { expanded.clear(); $("flow-info").textContent = "플로우를 선택하면 START에서 상→하로 입력·출력·분기·루프를 펼칩니다. 주황색 선은 방향성 경로입니다."; graph = structuralGraph(); await render(true); return; }
  const expand = expanded.size ? `&expand=${encodeURIComponent([...expanded].join(","))}` : ""; const result = await api(`/api/flow?id=${encodeURIComponent(id)}&direction=${encodeURIComponent($("direction").value)}&view=${encodeURIComponent($("flow-view").value)}&budget=${FLOW_BUDGET}${expand}`); graph = {nodes: result.nodes, edges: result.edges}; const flow = result.flow; const direction = {both: "전체 경로", downstream: "후속 경로", upstream: "선행 경로"}[flow.direction] || "방향성 경로"; $("flow-info").textContent = `${flow.title} · ${direction} · ${flow.view} · ${flow.phases.join(" → ")} · ${flow.collapsed ? `${flow.collapsed}개 세부 호출 접힘` : "전체 표시"} · ${flow.gaps?.length ? `${flow.gaps.length} gaps` : "정적 경로 확정"}`; await render(true);
}
function escapeHtml(value) { return String(value).replace(/[&<>]/g, character => ({"&": "&amp;", "<": "&lt;", ">": "&gt;"}[character])); }
async function show(node) {
  selected = node.id; await render(); $("detail").querySelector("h2").textContent = node.qualname || node.name; const incoming = snapshot.edges.filter(edge => edge.target === node.id).length, outgoing = snapshot.edges.filter(edge => edge.source === node.id).length, detail = node.detail || {};
  $("meta").innerHTML = `<b>${escapeHtml(node.kind)}</b><br>${escapeHtml(node.role)}<br>${escapeHtml(node.path || "boundary")}:${node.line || ""}<br>${escapeHtml(node.change)}<br>callers ${incoming} / callees ${outgoing}<br>${escapeHtml(JSON.stringify(detail, null, 2))}`;
  if (!node.path) { $("source").textContent = ""; $("diff").textContent = ""; return; }
  try { const [source, diff] = await Promise.all([api(`/api/source?path=${encodeURIComponent(node.path)}&line=${node.line}`), api(`/api/diff?path=${encodeURIComponent(node.path)}`)]); $("source").textContent = `${source.start}-${source.end}\n${source.text}`; $("diff").textContent = diff.text || "HEAD 대비 변경 없음"; } catch (error) { $("source").textContent = String(error); }
}
function mockWidget(control) {
  const kind = control.kind, label = document.createElement("label");
  let widget;
  if (kind === "checkbox" || kind === "radio_button") {
    widget = document.createElement("input");
    widget.type = kind === "checkbox" ? "checkbox" : "radio";
    const text = document.createElement("span");
    text.textContent = control.label;
    label.append(widget, text);
    return label;
  }
  if (kind.startsWith("slider_") || kind.startsWith("drag_")) {
    const wrapper = document.createElement("div");
    const text = document.createElement("label");
    text.textContent = control.label;
    widget = document.createElement("input");
    widget.type = "range";
    widget.min = "0";
    widget.max = "100";
    widget.value = "50";
    wrapper.append(text, widget);
    return wrapper;
  }
  if (kind.startsWith("input_")) {
    widget = document.createElement("input");
    widget.type = "text";
    widget.placeholder = control.label;
    return widget;
  }
  if (kind === "combo" || kind === "begin_combo" || kind === "selectable") {
    widget = document.createElement("select");
    option(widget, "", control.label);
    return widget;
  }
  widget = document.createElement("button");
  widget.type = "button";
  widget.textContent = control.label;
  return widget;
}
function activateMockWorkflow(control) {
  $("flow-family").value = control.family;
  refreshFlows();
  const workflow = $("workflow");
  if (![...workflow.options].some(item => item.value === control.workflow_id)) {
    $("status").textContent = `연결된 workflow를 찾지 못했습니다: ${control.workflow_id}`;
    return;
  }
  workflow.value = control.workflow_id;
  expanded.clear();
  document.querySelectorAll(".mock-control").forEach(element => element.classList.toggle("active", element.dataset.workflow === control.workflow_id));
  selectFlow().catch(error => $("status").textContent = String(error));
}
function renderMockUi() {
  if (!uiMap) return;
  const content = document.createDocumentFragment();
  uiMap.surfaces.forEach((surface, surfaceIndex) => {
    const details = document.createElement("details");
    details.className = `mock-surface${surface.helper ? " helper" : ""}`;
    details.open = surfaceIndex < 2 && !surface.helper;
    const summary = document.createElement("summary");
    const controlCount = surface.sections.reduce((count, section) => count + section.controls.length, 0);
    summary.textContent = `${surface.title} · ${controlCount}`;
    details.append(summary);
    surface.sections.forEach(section => {
      const sectionElement = document.createElement("section");
      sectionElement.className = "mock-section";
      const title = document.createElement("h4");
      title.textContent = section.title;
      title.title = `${section.qualname}:${section.line}`;
      const controls = document.createElement("div");
      controls.className = "mock-controls";
      section.controls.forEach(control => {
        const wrapper = document.createElement("div");
        wrapper.className = `mock-control${$("workflow").value === control.workflow_id ? " active" : ""}`;
        wrapper.dataset.workflow = control.workflow_id;
        wrapper.title = `${control.path}:${control.line}\n${control.expression || control.label}`;
        wrapper.append(mockWidget(control));
        const source = document.createElement("small");
        source.textContent = `${control.kind} · L${control.line}`;
        wrapper.append(source);
        if (control.dynamic || control.conditional) {
          const badges = document.createElement("div");
          badges.className = "mock-badges";
          if (control.dynamic) {
            const badge = document.createElement("span");
            badge.className = "mock-badge dynamic";
            badge.textContent = "dynamic label";
            badges.append(badge);
          }
          if (control.conditional) {
            const badge = document.createElement("span");
            badge.className = "mock-badge conditional";
            badge.textContent = "conditional";
            badges.append(badge);
          }
          wrapper.append(badges);
        }
        wrapper.addEventListener("pointerdown", event => {
          event.stopPropagation();
          activateMockWorkflow(control);
        });
        controls.append(wrapper);
      });
      sectionElement.append(title, controls);
      details.append(sectionElement);
    });
    content.append(details);
  });
  $("mock-ui-content").replaceChildren(content);
  $("mock-ui-summary").textContent = `${uiMap.stats.surfaces} surfaces · ${uiMap.stats.controls} controls · ${uiMap.stats.dynamic} dynamic`;
}
async function setMockUiVisible(visible) {
  if (visible && !uiMap) {
    uiMap = await api("/api/ui-map");
    renderMockUi();
  }
  $("mock-ui").hidden = !visible;
  $("mock-ui-toggle").setAttribute("aria-expanded", String(visible));
  $("mock-ui-toggle").textContent = visible ? "Mock UI 닫기" : "Mock UI 열기";
}
function replaceOptions(id, values, emptyLabel, sort = true) {
  const select = $(id), old = select.value; select.replaceChildren(); option(select, "", emptyLabel);
  (sort ? [...values].sort() : values).forEach(value => option(select, value));
  if ([...select.options].some(item => item.value === old)) select.value = old;
}
function timestamp(value) { try { return BigInt(String(value || 0)); } catch (_error) { return 0n; } }
function compareTimestamp(left, right) { const a = timestamp(left), b = timestamp(right); return a < b ? -1 : a > b ? 1 : 0; }
function applyTrace() {
  const traceId = $("jaeger-trace").value, spans = traceId ? traceSpans.filter(span => span.trace_id === traceId) : [];
  const bySymbol = new Map(snapshot.nodes.map(node => [node.id, node.id])), byName = new Map(snapshot.nodes.map(node => [node.qualname, node.id])), spanNodes = new Map(); observed = []; observedOrder = new Map(); observedErrors = new Set();
  spans.forEach(span => { const attrs = span.attributes || {}, symbol = attrs["elesim.code.symbol_id"] || attrs["elesim.code.symbol"] || attrs["code.symbol"] || attrs["code.function.name"], id = bySymbol.get(symbol) || byName.get(symbol); if (id) spanNodes.set(span.span_id, id); });
  [...spans].sort((a, b) => compareTimestamp(a.start_ns, b.start_ns)).forEach(span => { const id = spanNodes.get(span.span_id); if (id && !observedOrder.has(id)) observedOrder.set(id, observedOrder.size); if (id && span.error) observedErrors.add(id); });
  spans.forEach(span => { const source = spanNodes.get(span.parent_span_id), target = spanNodes.get(span.span_id); if (source && target) observed.push({source, target, kind: span.error ? "exception" : "trace", evidence: "observed", confidence: "exact", detail: span}); });
}
async function traces() {
  const service = encodeURIComponent($("jaeger-service").value), operation = encodeURIComponent($("jaeger-operation").value);
  const query = `limit=100${service ? `&service=${service}` : ""}${operation ? `&operation=${operation}` : ""}`;
  try {
    const [services, operations] = await Promise.all([api("/api/jaeger/services"), api(`/api/jaeger/operations?${query}`)]);
    replaceOptions("jaeger-service", services.services || [], "전체 서비스");
    replaceOptions("jaeger-operation", operations.operations || [], "전체 operation");
  } catch (_error) { /* trace query below reports the useful connection error */ }
  const data = await api(`/api/traces?${query}`); traceSpans = data.spans || []; const latest = traceId => traceSpans.filter(span => span.trace_id === traceId).reduce((value, span) => timestamp(span.start_ns) > value ? timestamp(span.start_ns) : value, 0n); const traces = [...new Set(traceSpans.map(span => span.trace_id).filter(Boolean))].sort((a, b) => compareTimestamp(latest(b), latest(a))); replaceOptions("jaeger-trace", traces, "관측 순서 없음", false); if (traces.length) $("jaeger-trace").value = traces[0]; applyTrace(); $("status").textContent = data.spans?.[0]?.error ? `Jaeger 연결 실패: ${data.spans[0].error}` : `Jaeger ${traceSpans.length} spans · ${traces.length} traces`; await render();
}
async function load() {
  snapshot = await api("/api/snapshot"); graph = structuralGraph(); $("status").textContent = `${snapshot.git_head.slice(0, 8)} · ${new Date(snapshot.generated_at).toLocaleTimeString()} · schema ${snapshot.schema_version}`; if (!$("role").dataset.ready) { initFilters(); $("role").dataset.ready = "1"; } else refreshFlows(); if (!$('mock-ui').hidden) { uiMap = await api("/api/ui-map"); renderMockUi(); } else { uiMap = null; } if ($("workflow").value) { expanded.clear(); await selectFlow(); } else await render(true);
}
["search", "role", "depth", "change", "hideTests", "staticEdges", "observedEdges"].forEach(id => $(id).addEventListener("input", () => render(true)));
$("direction").addEventListener("change", () => { if ($("workflow").value) selectFlow().catch(error => $("status").textContent = String(error)); else render(true); });
$("flow-view").addEventListener("change", () => { expanded.clear(); if ($("workflow").value) selectFlow().catch(error => $("status").textContent = String(error)); });
$("flow-family").addEventListener("change", () => { expanded.clear(); refreshFlows(); selectFlow().catch(error => $("status").textContent = String(error)); }); $("workflow").addEventListener("change", () => { expanded.clear(); renderMockUi(); selectFlow().catch(error => $("status").textContent = String(error)); }); $("traces").addEventListener("click", () => traces().catch(error => $("status").textContent = String(error))); $("fit").addEventListener("click", fitGraph);
$("graph").addEventListener("wheel", event => { event.preventDefault(); const delta = event.deltaMode === WheelEvent.DOM_DELTA_LINE ? event.deltaY * 16 : event.deltaY; zoomAt(event.clientX, event.clientY, Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, transform.k * Math.exp(-delta * .0012)))); }, {passive: false});
$("jaeger-service").addEventListener("change", () => traces().catch(error => $("status").textContent = String(error)));
$("jaeger-operation").addEventListener("change", () => traces().catch(error => $("status").textContent = String(error)));
$("jaeger-trace").addEventListener("change", () => { applyTrace(); render().catch(error => $("status").textContent = String(error)); });
$("mock-ui-toggle").addEventListener("click", () => setMockUiVisible($("mock-ui").hidden).catch(error => $("status").textContent = String(error)));
$("mock-ui-close").addEventListener("click", () => setMockUiVisible(false));
$("collapse-all").addEventListener("click", () => { expanded.clear(); if ($("workflow").value) selectFlow().catch(error => $("status").textContent = String(error)); });
$("graph").addEventListener("pointerdown", event => { if (event.button !== 0 || event.target.closest?.(".node")) return; pan = {x: event.clientX, y: event.clientY, tx: transform.x, ty: transform.y}; $("graph").setPointerCapture(event.pointerId); }); $("graph").addEventListener("pointermove", event => { if (pan) { transform.x = pan.tx + event.clientX - pan.x; transform.y = pan.ty + event.clientY - pan.y; applyTransform(); } }); $("graph").addEventListener("pointerup", () => { pan = null; }); $("graph").addEventListener("pointercancel", () => { pan = null; dragging = null; });
load().catch(error => $("status").textContent = String(error)); const events = new EventSource(`/api/events?token=${encodeURIComponent(token)}`); events.addEventListener("snapshot", event => { if (snapshot && event.data !== snapshot.digest) load(); });
