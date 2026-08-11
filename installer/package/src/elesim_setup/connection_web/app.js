"use strict";

const query = new URLSearchParams(window.location.search);
if (query.get("token")) {
  sessionStorage.setItem("elesimConnectionToken", query.get("token"));
  history.replaceState({}, "", window.location.pathname);
}

const token = sessionStorage.getItem("elesimConnectionToken") || "";
const computerSlots = ["com1", "com2", "com3", "com4"];
const jetsonSlot = "com4";
const slots = [...computerSlots];
const applicationRoles = ["pilot", "sim", "ui", "robot"];
const movableRoles = applicationRoles.filter((role) => role !== "robot");
const dropBandRatio = 0.5;

let catalog = {};
let language = "ko";
let runtimePollInFlight = false;
let schemaVersion = 4;
let topologyMode = "full";
let roleLocations = {
  pilot: "com1",
  sim: "com2",
  ui: "com1",
  robot: "com4"
};
// Keep the user's visual order instead of re-sorting roles by application name.
// The initial value only supplies a sensible order for a new topology.
let roleOrder = [...applicationRoles];
// Legacy schema-v3 migration marker: old saved files used roleLocations.robot = "robot".
let endpointIds = {
  pilot: "pilot-main",
  sim: "sim-default",
  ui: "ui-main",
  robot: "robot-go2"
};
let dropPreviewKey = "";
let pollTimer = null;
let runtimePollTimer = null;
let runtimeRestartable = false;
let runtimeOptionsLocked = false;
let workflowSaved = false;
let workflowApplied = false;
// A browser session always revalidates the loaded form before it can proceed.
// The topology fields are still restored from disk, but a previous session's
// visual stage must never unlock Booting just because the local Authority has
// an active generation; that does not prove every host has its role bundle.
let workflowRequiresFreshSave = true;
const workflowStates = {save: "pending", apply: "pending", start: "pending"};

const byId = (id) => document.getElementById(id);
const card = (slot) => document.querySelector(`.host-card[data-slot="${slot}"]`);
const field = (slot, name) => card(slot).querySelector(`[data-field="${name}"]`);

function addField(slot, name, labelKey, value = "", {checkbox = false, wide = false, unitRole = ""} = {}) {
  const hostFields = card(slot).querySelector(".host-fields");
  if (field(slot, name)) return field(slot, name);
  const label = document.createElement("label");
  if (wide) label.classList.add("wide");
  if (unitRole) label.dataset.unitRole = unitRole;
  const input = document.createElement("input");
  input.dataset.field = name;
  input.type = checkbox ? "checkbox" : "text";
  if (checkbox) input.checked = Boolean(value);
  else input.value = value;
  input.spellcheck = false;
  const span = document.createElement("span");
  span.dataset.i18n = labelKey;
  label.append(input, span);
  hostFields.append(label);
  return input;
}

function normalizeHostCards() {
  // Schema-v3 shipped a special Robot card. Reuse that DOM slot as Jetson
  // while still accepting old saved topologies.
  const legacyRobot = document.querySelector('.host-card[data-slot="robot"]');
  if (legacyRobot) {
    legacyRobot.dataset.slot = "com4";
    legacyRobot.classList.remove("robot", "disabled");
    legacyRobot.querySelector(".slot-name").textContent = "Jetson";
    const fixed = legacyRobot.querySelector(".fixed");
    if (fixed) fixed.remove();
    const zone = legacyRobot.querySelector(".drop-zone");
    zone.dataset.dropSlot = "com4";
    zone.dataset.dropUnit = "runtime";
    zone.classList.remove("fixed-zone");
    if (!legacyRobot.querySelector(".unit-lanes")) {
      const lanes = document.createElement("div");
      lanes.className = "unit-lanes";
      const runtimeLane = document.createElement("section");
      runtimeLane.className = "unit-lane runtime-lane";
      const runtimeTitle = document.createElement("h3");
      runtimeTitle.dataset.i18n = "unit.runtime";
      runtimeLane.append(runtimeTitle, zone);
      const robotLane = document.createElement("section");
      robotLane.className = "unit-lane robot-lane";
      const robotTitle = document.createElement("h3");
      robotTitle.dataset.i18n = "unit.robot";
      const robotZone = document.createElement("div");
      robotZone.className = "drop-zone";
      robotZone.dataset.dropSlot = "com4";
      robotZone.dataset.dropUnit = "robot";
      robotLane.append(robotTitle, robotZone);
      lanes.append(runtimeLane, robotLane);
      legacyRobot.insertBefore(lanes, legacyRobot.querySelector(".host-fields"));
    }
    const local = legacyRobot.querySelector('input[name="local-host"]');
    local.disabled = false;
    local.value = "com4";
    legacyRobot.querySelector("header .unused")?.remove();
    const probe = legacyRobot.querySelector("[data-probe-slot]");
    if (probe) probe.dataset.probeSlot = "com4";
    field("com4", "install-root").value = "/opt/elesim";
    field("com4", "bin-dir").value = "/opt/elesim/bin";
    field("com4", "host-id").value = "com4";
  }
}

function t(key) {
  return catalog[language]?.[key] || key;
}

function setWorkflowStepState(step, state) {
  workflowStates[step] = state;
  document.querySelector(`.workflow-step[data-step="${step}"]`)?.setAttribute("data-state", state);
}

function setWorkflowStepEnabled(step, enabled) {
  const element = document.querySelector(`.workflow-step[data-step="${step}"]`);
  if (!element) return;
  element.dataset.enabled = String(enabled);
  const button = element.querySelector("button");
  if (button) button.disabled = !enabled;
}

function setWorkflowButtonsEnabled(step, buttons) {
  const element = document.querySelector(`.workflow-step[data-step="${step}"]`);
  if (!element) return;
  const enabled = Object.values(buttons).some(Boolean);
  element.dataset.enabled = String(enabled);
  Object.entries(buttons).forEach(([id, value]) => {
    const button = byId(id);
    if (button) button.disabled = !value;
  });
}

function workflowStepForAction(action) {
  if (["prepare", "provision", "deploy", "rotate"].includes(action)) return "apply";
  if (action === "restart") return "start";
  if (action === "start") return "start";
  return "";
}

async function api(path, options = {}) {
  const headers = new Headers(options.headers || {});
  headers.set("X-Elesim-Token", token);
  if (options.body !== undefined) headers.set("Content-Type", "application/json");
  const response = await fetch(path, {...options, headers});
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(payload.error || `${response.status} ${response.statusText}`);
  return payload;
}

function setBannerVisible(banner, visible) {
  banner.hidden = !visible;
  banner.classList.toggle("dismissed", !visible);
}

function showError(value) {
  const banner = byId("error-banner");
  banner.querySelector(".banner-message").textContent = value instanceof Error ? value.message : String(value);
  setBannerVisible(banner, true);
}

function showNotice(key) {
  const banner = byId("notice-banner");
  banner.querySelector(".banner-message").textContent = t(key);
  setBannerVisible(banner, true);
}

function applyLanguage(next) {
  language = next;
  document.documentElement.lang = language;
  document.querySelectorAll("[data-i18n]").forEach((element) => {
    element.textContent = t(element.dataset.i18n);
  });
  document.querySelectorAll("[data-i18n-aria-label]").forEach((element) => {
    element.setAttribute("aria-label", t(element.dataset.i18nAriaLabel));
  });
  document.querySelectorAll("[data-language]").forEach((button) => {
    button.classList.toggle("active", button.dataset.language === language);
  });
  renderRoleBlocks();
  updateWorkflow();
}

function isActive(slot) {
  return slot === jetsonSlot ? topologyMode === "full" : !field(slot, "unused").checked;
}

function visibleRoles() {
  const allowed = topologyMode === "simulation-only"
    ? applicationRoles.filter((role) => role !== "robot")
    : applicationRoles;
  return roleOrder.filter((role) => allowed.includes(role));
}

function activeSlots() {
  return slots.filter(isActive);
}

function applyTopologyMode(next) {
  topologyMode = next === "simulation-only" ? "simulation-only" : "full";
  const selector = byId("topology-mode");
  if (selector) selector.value = topologyMode;
  if (card(jetsonSlot)) card(jetsonSlot).hidden = topologyMode === "simulation-only";
  if (topologyMode === "simulation-only") {
    roleLocations.robot = "";
    if (card(jetsonSlot)) {
      setCardActive(jetsonSlot);
    }
  } else {
    roleLocations.robot = jetsonSlot;
    if (card(jetsonSlot)) {
      setCardActive(jetsonSlot);
    }
  }
  renderRoleBlocks();
  updateSshVisibility();
}

function firstActiveCom(except = "") {
  return slots.find((slot) => slot !== except && isActive(slot)) || "";
}

function firstActiveJetson(except = "") {
  const local = document.querySelector('input[name="local-host"]:checked')?.value;
  return jetsonSlot !== except && jetsonSlot !== local && isActive(jetsonSlot) ? jetsonSlot : "";
}

function firstActiveRuntime(except = "") {
  return slots.find((slot) => (
    slot !== except && isActive(slot) && slot !== jetsonSlot
  )) || "";
}

function canPlaceRole(role, target, {notify = true} = {}) {
  if (role === "robot" && target !== jetsonSlot) {
    if (notify) showError(t("error.robot.jetson"));
    return false;
  }
  const local = document.querySelector('input[name="local-host"]:checked')?.value;
  if (role === "robot" && local === target) {
    if (notify) showError(t("error.local.robot"));
    return false;
  }
  if (role === "sim" && target === jetsonSlot) {
    if (notify) showError(t("error.sim.jetson"));
    return false;
  }
  return true;
}

function renderRoleBlocks() {
  clearDropPreview();
  document.querySelectorAll(".drop-zone").forEach((zone) => zone.replaceChildren());
  visibleRoles().forEach((role) => {
    const slot = roleLocations[role];
    const unit = role === "robot" ? "robot" : "runtime";
    const zone = document.querySelector(`[data-drop-slot="${slot}"][data-drop-unit="${unit}"]`);
    if (!zone) return;
    const block = document.createElement("div");
    block.className = `role-block role-${role}`;
    block.dataset.role = role;
    block.draggable = role !== "robot";
    const title = document.createElement("strong");
    title.textContent = t(`role.${role}`);
    const endpoint = document.createElement("label");
    const label = document.createElement("span");
    label.textContent = t("role.endpoint");
    const input = document.createElement("input");
    input.type = "text";
    input.value = endpointIds[role];
    input.spellcheck = false;
    input.addEventListener("input", () => {
      endpointIds[role] = input.value;
      markWorkflowDirty();
    });
    endpoint.append(label, input);
    block.append(title, endpoint);
    if (role !== "robot") {
      block.addEventListener("dragstart", (event) => {
        clearDropPreview();
        event.dataTransfer.effectAllowed = "move";
        event.dataTransfer.setData("text/plain", role);
        block.classList.add("dragging");
      });
      block.addEventListener("dragend", () => {
        block.classList.remove("dragging");
        clearDropPreview();
      });
    }
    zone.append(block);
  });
}

function insertRoleInOrder(role, target, targetUnit, targetRole = "", insertBefore = false) {
  if (targetRole === role) return;
  roleOrder = roleOrder.filter((candidate) => candidate !== role);
  roleLocations[role] = target;

  const targetRoles = roleOrder.filter((candidate) => {
    if (roleLocations[candidate] !== target) return false;
    return targetUnit === "robot" ? candidate === "robot" : candidate !== "robot";
  });
  let insertionIndex = roleOrder.length;
  if (targetRole && roleLocations[targetRole] === target) {
    const targetIndex = roleOrder.indexOf(targetRole);
    if (targetIndex >= 0) insertionIndex = targetIndex + (insertBefore ? 0 : 1);
  } else if (targetRoles.length) {
    insertionIndex = roleOrder.indexOf(targetRoles[targetRoles.length - 1]) + 1;
  }
  roleOrder.splice(insertionIndex, 0, role);
}

function moveRole(
  role,
  target,
  targetUnit = "runtime",
  targetRole = "",
  insertBefore = false,
) {
  if (role === "robot") return;
  if (!movableRoles.includes(role) || !isActive(target)) return;
  if (targetUnit === "robot" && role !== "robot") {
    showError(t("error.unit.robot"));
    return;
  }
  if (targetUnit === "runtime" && role === "robot") {
    showError(t("error.unit.runtime"));
    return;
  }
  if (!canPlaceRole(role, target)) return;
  insertRoleInOrder(role, target, targetUnit, targetRole, insertBefore);
  markWorkflowDirty();
  renderRoleBlocks();
}

function dropPlacement(zone, pointerY, draggedRole = "") {
  const blocks = [...zone.querySelectorAll(".role-block")].filter(
    (block) => block.dataset.role !== draggedRole,
  );
  if (!blocks.length) return {targetRole: "", insertBefore: false};

  const zoneRect = zone.getBoundingClientRect();
  const rects = blocks.map((block) => ({
    block,
    rect: block.getBoundingClientRect(),
  }));
  const first = rects[0];
  if (
    pointerY >= zoneRect.top
    && pointerY <= first.rect.top + first.rect.height * dropBandRatio
  ) {
    return {targetRole: first.block.dataset.role, insertBefore: true};
  }

  for (let index = 1; index < rects.length; index += 1) {
    const previous = rects[index - 1].rect;
    const next = rects[index];
    const lowerBand = previous.bottom - previous.height * dropBandRatio;
    const upperBand = next.rect.top + next.rect.height * dropBandRatio;
    if (pointerY >= lowerBand && pointerY <= upperBand) {
      return {targetRole: next.block.dataset.role, insertBefore: true};
    }
  }

  const last = rects[rects.length - 1].rect;
  if (
    pointerY >= last.bottom - last.height * dropBandRatio
    && pointerY <= zoneRect.bottom
  ) {
    return {targetRole: "", insertBefore: false};
  }
  return null;
}

function dropChangesOrder(zone, draggedRole, placement) {
  const targetUnit = zone.dataset.dropUnit || "runtime";
  const targetRoles = roleOrder.filter((role) => {
    if (roleLocations[role] !== zone.dataset.dropSlot) return false;
    if (targetUnit === "robot") return role === "robot";
    return role !== "robot";
  });
  if (roleLocations[draggedRole] !== zone.dataset.dropSlot) return true;

  const withoutDragged = targetRoles.filter((role) => role !== draggedRole);
  const insertionIndex = placement.targetRole
    ? withoutDragged.indexOf(placement.targetRole)
    : withoutDragged.length;
  withoutDragged.splice(Math.max(0, insertionIndex), 0, draggedRole);
  return withoutDragged.some((role, index) => role !== targetRoles[index]);
}

function clearDropPreview() {
  document.querySelectorAll(".role-block.drop-shift").forEach((block) => {
    block.classList.remove("drop-shift");
  });
  dropPreviewKey = "";
}

function updateDropPreview(zone, placement, draggedRole) {
  if (!placement) {
    clearDropPreview();
    return;
  }
  const key = [
    zone.dataset.dropSlot,
    zone.dataset.dropUnit || "runtime",
    draggedRole,
    placement.targetRole,
    placement.insertBefore,
  ].join(":");
  if (key === dropPreviewKey) return;

  clearDropPreview();
  void zone.offsetHeight;
  const blocks = [...zone.querySelectorAll(".role-block")].filter(
    (block) => block.dataset.role !== draggedRole,
  );
  const insertionIndex = placement.targetRole
    ? blocks.findIndex((block) => block.dataset.role === placement.targetRole)
    : blocks.length;
  blocks.slice(Math.max(0, insertionIndex)).forEach((block) => {
    block.classList.add("drop-shift");
  });
  dropPreviewKey = key;
}

function setCardActive(slot) {
  const hostCard = card(slot);
  const active = isActive(slot);
  const unused = field(slot, "unused");
  hostCard.classList.toggle("disabled", !active);
  hostCard.querySelectorAll("input, select, button").forEach((control) => {
    if (control === unused) return;
    control.disabled = !active;
  });
  if (!active) {
    if (slot === jetsonSlot) {
      renderRoleBlocks();
      updateSshVisibility();
      return;
    }
    const destination = firstActiveCom(slot);
    if (!destination) {
      unused.checked = false;
      setCardActive(slot);
      showError(t("error.one.com"));
      return;
    }
    const movingRoles = movableRoles.filter((role) => roleLocations[role] === slot);
    const hasInvalidDestination = movingRoles.some((role) => {
      const destinationForRole = role === "robot"
        ? firstActiveJetson(slot)
        : role === "sim" ? firstActiveRuntime(slot) : destination;
      return !destinationForRole;
    });
    if (hasInvalidDestination) {
      unused.checked = false;
      setCardActive(slot);
      const message = movingRoles.includes("robot")
        ? t("error.robot.jetson") : t("error.sim.jetson");
      showError(message);
      return;
    }
    movingRoles.forEach((role) => {
      const roleDestination = role === "robot"
        ? firstActiveJetson(slot)
        : role === "sim" ? firstActiveRuntime(slot) : destination;
      if (roleDestination) roleLocations[role] = roleDestination;
    });
    const local = document.querySelector('input[name="local-host"]:checked');
    if (local?.value === slot) {
      document.querySelector(`input[name="local-host"][value="${destination}"]`).checked = true;
    }
  }
  renderRoleBlocks();
  updateSshVisibility();
}

function updateSshVisibility() {
  const local = document.querySelector('input[name="local-host"]:checked')?.value;
  slots.forEach((slot) => {
    const details = card(slot).querySelector(".ssh-fields");
    details.hidden = !isActive(slot) || local === slot;
    card(slot).classList.toggle("local", local === slot && isActive(slot));
    updateSshMode(slot);
  });
}

function updateSshMode(slot) {
  const tailscale = field(slot, "ssh-tailscale").checked;
  const active = isActive(slot);
  const port = field(slot, "ssh-port");
  const key = field(slot, "ssh-key");
  if (tailscale) {
    port.value = "22";
    key.value = "";
  }
  port.disabled = !active || tailscale;
  key.disabled = !active || tailscale;
  card(slot).classList.toggle("tailscale-ssh", tailscale);
}

function sshPort(slot) {
  return field(slot, "ssh-tailscale").checked
    ? 22
    : Number(field(slot, "ssh-port").value);
}

function topologyFromForm() {
  const localSlot = document.querySelector('input[name="local-host"]:checked')?.value || "";
  const active = activeSlots();
  const minimum = topologyMode === "simulation-only" ? 1 : 2;
  const maximum = topologyMode === "simulation-only" ? 3 : 4;
  if (active.length < minimum || active.length > maximum) throw new Error(t("error.host.count"));
  if (!active.includes(localSlot)) throw new Error(t("error.local"));
  if (topologyMode === "full" && (!active.includes(jetsonSlot) || roleLocations.robot !== jetsonSlot)) {
    throw new Error(t("error.jetson.robot"));
  }
  if (topologyMode === "full" && localSlot === jetsonSlot) {
    throw new Error(t("error.local.robot"));
  }
  const empty = active.find((slot) => !visibleRoles().some((role) => roleLocations[role] === slot));
  if (empty) throw new Error(`${t("error.empty.host")} (${empty})`);
  ensureRoutedDiscovery({notify: false});
  const hosts = active.map((slot) => {
    const local = slot === localSlot;
    const roles = visibleRoles().filter((role) => roleLocations[role] === slot);
    const assignments = roles.map((role) => ({role, endpoint_id: endpointIds[role].trim()}));
    const runtimeRoles = assignments.filter((item) => item.role !== "robot");
    const installRoot = field(slot, "install-root").value.trim();
    const binDir = field(slot, "bin-dir").value.trim();
    const units = [];
    if (runtimeRoles.length) {
      units.push({
        id: "runtime",
        assignments: runtimeRoles,
        install_mode: "container",
        install_root: installRoot,
        bin_dir: binDir,
        lifecycle: "compose"
      });
    }
    const robotAssignment = assignments.find((item) => item.role === "robot");
    if (robotAssignment) {
      units.push({
        id: "robot-native",
        assignments: [robotAssignment],
        install_mode: "native",
        install_root: installRoot,
        bin_dir: binDir,
        lifecycle: "systemd"
      });
    }
    const hostId = field(slot, "host-id").value.trim();
    const host = {
      id: hostId,
      local,
      dds: {
        address: field(slot, "dds-address").value.trim(),
        interface: field(slot, "dds-interface").value.trim(),
        ...(isTailscaleInterface(field(slot, "dds-interface").value)
          ? {address_source: "tailscale"} : {})
      },
      ssh: null,
      jetson: slot === jetsonSlot,
      units
    };
    if (!local) {
      host.ssh = {
        host: field(slot, "ssh-host").value.trim(),
        port: sshPort(slot),
        user: field(slot, "ssh-user").value.trim(),
        identity_file: field(slot, "ssh-tailscale").checked
          ? "" : field(slot, "ssh-key").value.trim(),
        pinned_fingerprint: field(slot, "ssh-fingerprint").value.trim(),
        auth_mode: field(slot, "ssh-tailscale").checked ? "tailscale" : "openssh"
      };
    }
    return host;
  });
  return {
    schema_version: schemaVersion,
    topology_mode: topologyMode,
    system_id: byId("system-id").value.trim(),
    security_profile: byId("security").value,
    dds_graph: {
      domain_id: Number(byId("domain-id").value),
      rmw_implementation: "rmw_cyclonedds_cpp",
      discovery_mode: byId("discovery").value
    },
    hosts
  };
}

function fillHost(slot, host) {
  field(slot, "host-id").value = host.id;
  field(slot, "dds-address").value = host.dds.address;
  field(slot, "dds-interface").value = host.dds.interface;
  const units = Array.isArray(host.units) ? host.units : [{
    id: host.install_mode === "native" ? "robot-native" : "runtime",
    assignments: host.assignments || [],
    install_mode: host.install_mode || "container",
    install_root: host.install_root || "/opt/elesim",
    bin_dir: host.bin_dir || "/opt/elesim/bin",
    lifecycle: host.lifecycle || "compose"
  }];
  const pathUnit = units.find((unit) => unit.install_mode === "container") || units[0];
  field(slot, "install-root").value = pathUnit?.install_root || "/opt/elesim";
  field(slot, "bin-dir").value = pathUnit?.bin_dir || "/opt/elesim/bin";
  document.querySelector(`input[name="local-host"][value="${slot}"]`).checked = host.local;
  if (host.ssh) {
    field(slot, "ssh-host").value = host.ssh.host;
    field(slot, "ssh-port").value = host.ssh.port;
    field(slot, "ssh-user").value = host.ssh.user;
    field(slot, "ssh-tailscale").checked = host.ssh.auth_mode === "tailscale";
    field(slot, "ssh-key").value = host.ssh.identity_file;
    field(slot, "ssh-fingerprint").value = host.ssh.pinned_fingerprint;
    updateSshMode(slot);
  } else {
    field(slot, "ssh-host").value = "";
    field(slot, "ssh-tailscale").checked = false;
    updateSshMode(slot);
  }
  units.flatMap((unit) => unit.assignments || []).forEach((assignment) => {
    roleLocations[assignment.role] = slot;
    endpointIds[assignment.role] = assignment.endpoint_id;
    if (!roleOrder.includes(assignment.role)) roleOrder.push(assignment.role);
  });
}

function applyLocalTailscaleHint(context) {
  if (context?.manager_transport?.container_network_mode === "tailscale-sidecar") {
    if (activeSlots().length > 1 && byId("discovery").value === "multicast") {
      byId("discovery").value = "static";
      showNotice("notice.tailscale.static");
    }
    return;
  }
  const hint = context?.tailscale;
  if (!hint?.available || !Array.isArray(hint.addresses) || !hint.addresses.length) return;
  const local = document.querySelector('input[name="local-host"]:checked')?.value || "com1";
  if (!isActive(local)) return;
  const address = field(local, "dds-address");
  const iface = field(local, "dds-interface");
  if (!address.value.trim()) address.value = String(hint.addresses[0]);
  if (!iface.value.trim()) iface.value = String(hint.interface || "tailscale0");
  ensureRoutedDiscovery();
  showNotice("notice.tailscale.prefill");
}

function ensureRoutedDiscovery({notify = true} = {}) {
  const active = activeSlots();
  const usesTailscale = active.some((slot) =>
    isTailscaleInterface(field(slot, "dds-interface").value) ||
    isTailscaleAddress(field(slot, "dds-address").value)
  );
  if (active.length > 1 && usesTailscale && byId("discovery").value === "multicast") {
    byId("discovery").value = "static";
    if (notify) showNotice("notice.tailscale.static");
    return true;
  }
  return false;
}

function isTailscaleInterface(value) {
  return /^tailscale[0-9]+$/i.test(String(value || "").trim());
}

function isTailscaleAddress(value) {
  const parts = String(value || "").trim().split(".");
  if (parts.length !== 4 || parts[0] !== "100") return false;
  const second = Number(parts[1]);
  return Number.isInteger(second) && second >= 64 && second <= 127;
}

function applyTopology(topology) {
  schemaVersion = topology.schema_version;
  applyTopologyMode(topology.topology_mode || "full");
  roleOrder = [];
  byId("system-id").value = topology.system_id;
  byId("domain-id").value = topology.dds_graph.domain_id;
  byId("discovery").value = topology.dds_graph.discovery_mode;
  byId("security").value = topology.security_profile;

  computerSlots.forEach((slot, index) => {
    const host = topology.hosts[index];
    if (field(slot, "unused")) field(slot, "unused").checked = !host;
    if (host) fillHost(slot, host);
    setCardActive(slot);
  });
  if (topologyMode === "full") {
    roleLocations.robot = jetsonSlot;
    setCardActive(jetsonSlot);
  }
  applicationRoles.forEach((role) => {
    if (!roleOrder.includes(role)) roleOrder.push(role);
  });
  updateSshVisibility();
  updateWorkflow();
  renderRoleBlocks();
}

function updateWorkflow(running = ["running", "cancelling"].includes(byId("job-status")?.dataset.status || "")) {
  const apply = byId("apply");
  apply.textContent = t("action.prepare");
  setWorkflowStepEnabled("save", !running && !workflowSaved);
  setWorkflowStepEnabled("apply", !running && workflowSaved && !workflowApplied);
  setWorkflowButtonsEnabled("start", {
    "runtime-start": !running && workflowSaved && workflowApplied && !runtimeRestartable,
    restart: !running && workflowSaved && workflowApplied && runtimeRestartable,
  });
}

function runtimeLaunchOptions() {
  const gpuInherit = Boolean(byId("gpu-inherit")?.checked);
  return {
    gpu_inherit: gpuInherit,
    gpu_device: gpuInherit ? String(byId("gpu-device")?.value || "") : "",
    viewer: Boolean(byId("use-viewer")?.checked),
  };
}

function updateRuntimeOptions() {
  const inherit = byId("gpu-inherit");
  const device = byId("gpu-device");
  if (!inherit || !device) return;
  inherit.disabled = runtimeOptionsLocked;
  device.disabled = runtimeOptionsLocked || !inherit.checked;
  const viewer = byId("use-viewer");
  if (viewer) viewer.disabled = runtimeOptionsLocked;
}

function setRuntimeOptionsLocked(locked) {
  runtimeOptionsLocked = Boolean(locked);
  updateRuntimeOptions();
}

async function saveTopology({quiet = false, invalidate = true} = {}) {
  let topology;
  let result;
  try {
    topology = topologyFromForm();
    result = await api("/api/save", {method: "POST", body: JSON.stringify(topology)});
  } catch (error) {
    setWorkflowStepState("save", "error");
    throw error;
  }
  workflowSaved = true;
  workflowRequiresFreshSave = false;
  setWorkflowStepState("save", "success");
  if (invalidate) {
    workflowApplied = false;
    runtimeRestartable = false;
  }
  if (invalidate) {
    setWorkflowStepState("apply", "pending");
    setWorkflowStepState("start", "pending");
  }
  updateWorkflow();
  if (!quiet) showNotice("notice.saved");
  return result;
}

async function probeSsh(slot) {
  if (document.querySelector('input[name="local-host"]:checked')?.value === slot) {
    throw new Error(t("error.local.probe"));
  }
  const host = field(slot, "ssh-host").value.trim();
  const port = sshPort(slot);
  const result = await api("/api/ssh/fingerprint", {
    method: "POST",
    body: JSON.stringify({
      host,
      port,
      auth_mode: field(slot, "ssh-tailscale").checked ? "tailscale" : "openssh"
    })
  });
  const prompt = `${t("ssh.trust")}\n${host}:${port}\n${result.fingerprint}`;
  if (window.confirm(prompt)) {
    field(slot, "ssh-fingerprint").value = result.fingerprint;
    showNotice("notice.fingerprint");
  }
}

async function startJob(action) {
  const locksRuntimeOptions = ["start", "restart"].includes(action);
  if (locksRuntimeOptions) setRuntimeOptionsLocked(true);
  let submitted = false;
  let step = "";
  try {
    if (locksRuntimeOptions && (!workflowSaved || !workflowApplied)) {
      throw new Error(t("error.workflow.incomplete"));
    }
    if (action === "restart") {
      await pollRuntimeStatus();
      if (!runtimeRestartable) {
        throw new Error(t("error.restart.unavailable"));
      }
    }
    await saveTopology({quiet: true, invalidate: false});
    if (["prepare", "provision", "deploy", "rotate"].includes(action)) {
      workflowApplied = false;
      runtimeRestartable = false;
      setWorkflowStepState("start", "pending");
    }
    step = workflowStepForAction(action);
    if (step) setWorkflowStepState(step, "running");
    const payload = ["start", "restart"].includes(action)
      ? runtimeLaunchOptions()
      : {};
    await api(`/api/job/${action}`, {method: "POST", body: JSON.stringify(payload)});
    submitted = true;
    setJobRunning(true);
    if (pollTimer) window.clearInterval(pollTimer);
    pollTimer = window.setInterval(pollJob, 500);
    await pollJob();
  } catch (error) {
    if (step) setWorkflowStepState(step, "error");
    if (locksRuntimeOptions && !submitted) setRuntimeOptionsLocked(false);
    throw error;
  }
}

async function runApplyJob() {
  const action = byId("security").value === "sros2" ? "prepare" : "deploy";
  await startJob(action);
}

function markWorkflowDirty() {
  workflowSaved = false;
  workflowRequiresFreshSave = true;
  workflowApplied = false;
  runtimeRestartable = false;
  setWorkflowStepState("save", "pending");
  setWorkflowStepState("apply", "pending");
  setWorkflowStepState("start", "pending");
  updateWorkflow();
}

function renderRuntimeStatus(result) {
  if (!result?.available) {
    runtimeRestartable = false;
    updateWorkflow();
    byId("runtime-status").textContent = result?.reason || t("runtime.unavailable");
    return;
  }
  const hosts = Array.isArray(result.hosts) ? result.hosts : [];
  runtimeRestartable = hosts.length > 0 && hosts.every(
    (host) => host.reachable !== false && host.containers_present === true
  );
  const rows = hosts.map((host) => {
    const roles = (host.roles || []).join(", ");
    const state = host.reachable ? (host.state || "unknown") : t("runtime.unreachable");
    const detail = host.detail ? ` — ${host.detail}` : "";
    return `${host.host_id}: ${state} [${roles}]${detail}`;
  });
  byId("runtime-status").textContent = rows.join("\n") || "—";
  updateWorkflow();
}

async function pollRuntimeStatus() {
  if (runtimePollInFlight || ["running", "cancelling"].includes(byId("job-status").dataset.status)) return;
  runtimePollInFlight = true;
  try {
    renderRuntimeStatus(await api("/api/runtime"));
  } catch (error) {
    runtimeRestartable = false;
    byId("runtime-status").textContent = error instanceof Error ? error.message : String(error);
    updateWorkflow();
  } finally {
    runtimePollInFlight = false;
  }
}

function setJobRunning(running) {
  ["save", "topology-mode", "runtime-start"].forEach((id) => { byId(id).disabled = running; });
  updateWorkflow(running);
  byId("cancel").disabled = !running;
  if (!running && runtimeOptionsLocked) setRuntimeOptionsLocked(false);
}

async function pollJob() {
  try {
    const job = await api("/api/job");
    const key = `job.${job.status}`;
    byId("job-status").dataset.status = job.status;
    byId("job-status").textContent = `${t(key)}${job.action ? ` · ${t(`action.${job.action}`)}` : ""}`;
    byId("job-log").textContent = [...job.logs, job.error].filter(Boolean).join("\n");
    const running = ["running", "cancelling"].includes(job.status);
    const step = workflowStepForAction(job.action);
    const topologyAppliedByThisJob =
      job.status === "completed" &&
      ["prepare", "provision", "deploy", "rotate"].includes(job.action);
    if (step && running) setWorkflowStepState(step, "running");
    if (step && !running && job.status === "completed") setWorkflowStepState(step, "success");
    if (step && !running && ["failed", "cancelled"].includes(job.status)) setWorkflowStepState(step, "error");
    if (topologyAppliedByThisJob && !workflowRequiresFreshSave) {
      workflowSaved = true;
      workflowApplied = true;
      setWorkflowStepState("save", "success");
      setWorkflowStepState("apply", "success");
    }
    if (!running && job.topology_updated) {
      const context = await api("/api/context");
      if (context.topology) applyTopology(context.topology);
      // A sidecar-discovered address is factual input, not an implicit save.
      // Keep the form populated but require the operator to validate/save it
      // before another security or runtime action can be enabled.
      workflowSaved = !workflowRequiresFreshSave;
      runtimeRestartable = false;
      setWorkflowStepState("save", workflowRequiresFreshSave ? "pending" : "success");
      if (!topologyAppliedByThisJob) {
        workflowApplied = false;
        setWorkflowStepState("apply", "pending");
        setWorkflowStepState("start", "pending");
      }
    }
    setJobRunning(running);
    updateWorkflow(running);
    if (
      !running
      && ["check", "prepare", "provision", "deploy", "rotate", "start", "restart"].includes(job.action)
    ) {
      pollRuntimeStatus();
    }
    if (!running && pollTimer) {
      window.clearInterval(pollTimer);
      pollTimer = null;
    }
  } catch (error) {
    if (pollTimer) window.clearInterval(pollTimer);
    pollTimer = null;
    setJobRunning(false);
    showError(error);
  }
}

function bindEvents() {
  document.querySelectorAll("[data-language]").forEach((button) => {
    button.addEventListener("click", () => applyLanguage(button.dataset.language));
  });
  document.querySelectorAll("[data-banner-close]").forEach((button) => {
    button.addEventListener("click", () => {
      const banner = button.closest(".banner");
      if (banner) setBannerVisible(banner, false);
    });
  });
  document.querySelectorAll(".drop-zone").forEach((zone) => {
    if (zone.dataset.dropUnit === "robot") return;
    zone.addEventListener("dragover", (event) => {
      const target = zone.dataset.dropSlot;
      const role = event.dataTransfer?.getData("text/plain")
        || document.querySelector(".role-block.dragging")?.dataset.role
        || "";
      const targetUnit = zone.dataset.dropUnit || "runtime";
      const unitAllowed = targetUnit === "robot" ? role === "robot" : role !== "robot";
      const placement = dropPlacement(zone, event.clientY, role);
      const allowed = movableRoles.includes(role)
        && isActive(target)
        && unitAllowed
        && canPlaceRole(role, target, {notify: false});
      const previewPlacement = allowed && placement && dropChangesOrder(zone, role, placement)
        ? placement
        : null;
      updateDropPreview(zone, previewPlacement, role);
      if (placement && allowed) {
        event.preventDefault();
        event.dataTransfer.dropEffect = "move";
      }
    });
    zone.addEventListener("drop", (event) => {
      const role = event.dataTransfer.getData("text/plain")
        || document.querySelector(".role-block.dragging")?.dataset.role
        || "";
      const placement = dropPlacement(zone, event.clientY, role);
      if (!placement) {
        clearDropPreview();
        return;
      }
      event.preventDefault();
      clearDropPreview();
      moveRole(
        role,
        zone.dataset.dropSlot,
        zone.dataset.dropUnit || "runtime",
        placement.targetRole,
        placement.insertBefore,
      );
    });
  });
  computerSlots.forEach((slot) => {
    field(slot, "unused")?.addEventListener("change", () => setCardActive(slot));
  });
  document.querySelectorAll('input[name="local-host"]').forEach((input) => {
    input.addEventListener("change", updateSshVisibility);
  });
  slots.forEach((slot) => {
    ["ssh-host", "ssh-port"].forEach((name) => {
      field(slot, name).addEventListener("input", () => { field(slot, "ssh-fingerprint").value = ""; });
    });
    field(slot, "ssh-tailscale").addEventListener("change", () => {
      updateSshMode(slot);
      field(slot, "ssh-fingerprint").value = "";
    });
    field(slot, "dds-interface").addEventListener("input", () => {
      ensureRoutedDiscovery();
    });
  });
  document.querySelectorAll("input, select").forEach((control) => {
    if (control.closest(".boot-options")) return;
    control.addEventListener("input", markWorkflowDirty);
    control.addEventListener("change", markWorkflowDirty);
  });
  byId("gpu-inherit").addEventListener("change", updateRuntimeOptions);
  document.querySelectorAll("[data-probe-slot]").forEach((button) => {
    button.addEventListener("click", () => probeSsh(button.dataset.probeSlot).catch(showError));
  });
  byId("topology-mode").addEventListener("change", (event) => {
    applyTopologyMode(event.target.value);
  });
  byId("security").addEventListener("change", updateWorkflow);
  byId("save").addEventListener("click", () => saveTopology().catch(showError));
  byId("apply").addEventListener("click", () => runApplyJob().catch(showError));
  byId("restart").addEventListener("click", () => startJob("restart").catch(showError));
  byId("runtime-start").addEventListener("click", () => startJob("start").catch(showError));
  byId("cancel").addEventListener("click", async () => {
    try { await api("/api/cancel", {method: "POST", body: JSON.stringify({})}); }
    catch (error) { showError(error); }
  });
}

async function initialize() {
  try {
    catalog = await fetch("/i18n.json", {cache: "no-store"}).then((response) => response.json());
    normalizeHostCards();
    bindEvents();
    updateRuntimeOptions();
    applyLanguage("ko");
    computerSlots.forEach(setCardActive);
    const context = await api("/api/context");
    schemaVersion = context.schema_version;
    applyTopologyMode(context.topology?.topology_mode || "full");
    if (!context.topology) {
      setCardActive(jetsonSlot);
    }
    if (context.topology) {
      // Restore all values, but deliberately restart the operator workflow at
      // validation/save.  A local active generation can outlive a failed or
      // partial remote rollout, so it is not sufficient to unlock Booting.
      workflowSaved = false;
      workflowApplied = false;
      runtimeRestartable = false;
      workflowRequiresFreshSave = true;
      setWorkflowStepState("save", "pending");
      setWorkflowStepState("apply", "pending");
      setWorkflowStepState("start", "pending");
      applyTopology(context.topology);
    } else if (context.local_defaults) {
      if (context.local_defaults.install_root) {
        field("com1", "install-root").value = context.local_defaults.install_root;
      }
      if (context.local_defaults.bin_dir) {
        field("com1", "bin-dir").value = context.local_defaults.bin_dir;
      }
    }
    applyLocalTailscaleHint(context);
    updateSshVisibility();
    updateWorkflow();
    renderRoleBlocks();
    await pollJob();
    await pollRuntimeStatus();
    if (runtimePollTimer) window.clearInterval(runtimePollTimer);
    runtimePollTimer = window.setInterval(pollRuntimeStatus, 10000);
  } catch (error) {
    showError(error);
  }
}

initialize();
