"use strict";

const query = new URLSearchParams(window.location.search);
if (query.get("token")) {
  sessionStorage.setItem("elesimConnectionToken", query.get("token"));
  history.replaceState({}, "", window.location.pathname);
}

const token = sessionStorage.getItem("elesimConnectionToken") || "";
const computerSlots = ["com1", "com2", "com3"];
const slots = [...computerSlots, "robot"];
const applicationRoles = ["pilot", "sim", "ui", "robot"];
const movableRoles = ["pilot", "sim", "ui"];

let catalog = {};
let language = "ko";
let schemaVersion = 3;
let topologyMode = "full";
let roleLocations = {
  pilot: "com1",
  sim: "com2",
  ui: "com1",
  robot: "robot"
};
let endpointIds = {
  pilot: "pilot-main",
  sim: "sim-default",
  ui: "ui-main",
  robot: "robot-go2"
};
let pollTimer = null;
let runtimePollTimer = null;
let workflowSaved = false;
let workflowApplied = false;

const byId = (id) => document.getElementById(id);
const card = (slot) => document.querySelector(`.host-card[data-slot="${slot}"]`);
const field = (slot, name) => card(slot).querySelector(`[data-field="${name}"]`);

function t(key) {
  return catalog[language]?.[key] || key;
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

function showError(value) {
  const banner = byId("error-banner");
  banner.textContent = value instanceof Error ? value.message : String(value);
  banner.hidden = false;
  window.setTimeout(() => { banner.hidden = true; }, 9000);
}

function showNotice(key) {
  const banner = byId("notice-banner");
  banner.textContent = t(key);
  banner.hidden = false;
  window.setTimeout(() => { banner.hidden = true; }, 4500);
}

function applyLanguage(next) {
  language = next;
  document.documentElement.lang = language;
  document.querySelectorAll("[data-i18n]").forEach((element) => {
    element.textContent = t(element.dataset.i18n);
  });
  document.querySelectorAll("[data-language]").forEach((button) => {
    button.classList.toggle("active", button.dataset.language === language);
  });
  renderRoleBlocks();
  renderDerivedPeers();
  updateSecurityWarning();
}

function isActive(slot) {
  return slot === "robot" ? topologyMode === "full" : !field(slot, "unused").checked;
}

function visibleRoles() {
  return topologyMode === "simulation-only" ? movableRoles : applicationRoles;
}

function activeSlots() {
  return slots.filter(isActive);
}

function applyTopologyMode(next) {
  topologyMode = next === "simulation-only" ? "simulation-only" : "full";
  const selector = byId("topology-mode");
  if (selector) selector.value = topologyMode;
  const robotCard = card("robot");
  if (robotCard) robotCard.hidden = topologyMode === "simulation-only";
  renderRoleBlocks();
  updateSshVisibility();
  renderDerivedPeers();
}

function firstActiveCom(except = "") {
  return slots.find((slot) => slot !== "robot" && slot !== except && isActive(slot)) || "";
}

function renderRoleBlocks() {
  document.querySelectorAll(".drop-zone").forEach((zone) => zone.replaceChildren());
  visibleRoles().forEach((role) => {
    const slot = role === "robot" ? "robot" : roleLocations[role];
    const zone = document.querySelector(`[data-drop-slot="${slot}"]`);
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
        event.dataTransfer.effectAllowed = "move";
        event.dataTransfer.setData("text/plain", role);
        block.classList.add("dragging");
      });
      block.addEventListener("dragend", () => block.classList.remove("dragging"));
    }
    zone.append(block);
  });
}

function moveRole(role, target) {
  if (!movableRoles.includes(role) || target === "robot" || !isActive(target)) return;
  roleLocations[role] = target;
  markWorkflowDirty();
  renderRoleBlocks();
}

function setCardActive(slot) {
  const hostCard = card(slot);
  const active = isActive(slot);
  hostCard.classList.toggle("disabled", !active);
  hostCard.querySelectorAll("input, select, button").forEach((control) => {
    if (control === field(slot, "unused")) return;
    control.disabled = !active;
  });
  if (!active) {
    const destination = firstActiveCom(slot);
    if (!destination) {
      field(slot, "unused").checked = false;
      setCardActive(slot);
      showError(t("error.one.com"));
      return;
    }
    movableRoles.filter((role) => roleLocations[role] === slot).forEach((role) => {
      roleLocations[role] = destination;
    });
    const local = document.querySelector('input[name="local-host"]:checked');
    if (local?.value === slot) {
      document.querySelector(`input[name="local-host"][value="${destination}"]`).checked = true;
    }
  }
  renderRoleBlocks();
  updateSshVisibility();
  renderDerivedPeers();
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
  const hosts = active.map((slot) => {
    const local = slot === localSlot;
    const roles = visibleRoles().filter((role) => {
      return role === "robot" ? slot === "robot" : roleLocations[role] === slot;
    });
    const host = {
      id: field(slot, "host-id").value.trim(),
      display_name: field(slot, "display-name").value.trim(),
      local,
      dds: {
        address: field(slot, "dds-address").value.trim(),
        interface: field(slot, "dds-interface").value.trim(),
        ...(field(slot, "dds-interface").value.trim() === "tailscale0"
          ? {address_source: "tailscale"} : {})
      },
      ssh: null,
      assignments: roles.map((role) => ({role, endpoint_id: endpointIds[role].trim()})),
      install_mode: topologyMode === "full" && slot === "robot" ? "native" : "container",
      jetson: topologyMode === "full" && slot === "robot",
      install_root: field(slot, "install-root").value.trim(),
      bin_dir: field(slot, "bin-dir").value.trim(),
      lifecycle: topologyMode === "full" && slot === "robot" ? "systemd" : "compose"
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
      rmw_implementation: byId("rmw").value,
      discovery_mode: byId("discovery").value
    },
    hosts
  };
}

function preflightFromForm() {
  const activeCom = slots.filter((slot) => slot !== "robot" && isActive(slot));
  if (activeCom.length !== 2) throw new Error(t("error.preflight.hosts"));
  const localSlot = document.querySelector('input[name="local-host"]:checked')?.value || "";
  if (!activeCom.includes(localSlot)) throw new Error(t("error.local"));
  return {
    schema_version: 1,
    discovery_mode: byId("discovery").value,
    hosts: activeCom.map((slot) => {
      const local = slot === localSlot;
      return {
        id: field(slot, "host-id").value.trim(),
        display_name: field(slot, "display-name").value.trim(),
        local,
        dds: {
          address: field(slot, "dds-address").value.trim(),
          interface: field(slot, "dds-interface").value.trim()
        },
        ssh: local ? null : {
          host: field(slot, "ssh-host").value.trim(),
          port: sshPort(slot),
          user: field(slot, "ssh-user").value.trim(),
          auth_mode: field(slot, "ssh-tailscale").checked ? "tailscale" : "openssh"
        }
      };
    }),
    probe_ssh: true
  };
}

function fillHost(slot, host) {
  field(slot, "host-id").value = host.id;
  field(slot, "display-name").value = host.display_name;
  field(slot, "dds-address").value = host.dds.address;
  field(slot, "dds-interface").value = host.dds.interface;
  field(slot, "install-root").value = host.install_root;
  field(slot, "bin-dir").value = host.bin_dir;
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
    field(slot, "ssh-tailscale").checked = false;
    updateSshMode(slot);
  }
  host.assignments.forEach((assignment) => {
    roleLocations[assignment.role] = slot;
    endpointIds[assignment.role] = assignment.endpoint_id;
  });
}

function applyLocalTailscaleHint(context) {
  const hint = context?.tailscale;
  if (!hint?.available || !Array.isArray(hint.addresses) || !hint.addresses.length) return;
  const local = document.querySelector('input[name="local-host"]:checked')?.value || "com1";
  if (local === "robot" || !isActive(local)) return;
  const address = field(local, "dds-address");
  const iface = field(local, "dds-interface");
  if (!address.value.trim()) address.value = String(hint.addresses[0]);
  if (!iface.value.trim()) iface.value = String(hint.interface || "tailscale0");
  showNotice("notice.tailscale.prefill");
}

function applyTopology(topology) {
  schemaVersion = topology.schema_version;
  applyTopologyMode(topology.topology_mode || "full");
  byId("system-id").value = topology.system_id;
  byId("domain-id").value = topology.dds_graph.domain_id;
  byId("rmw").value = topology.dds_graph.rmw_implementation;
  byId("discovery").value = topology.dds_graph.discovery_mode;
  byId("security").value = topology.security_profile;

  const robotHost = topology.topology_mode === "simulation-only"
    ? null
    : topology.hosts.find((host) => host.assignments.some((item) => item.role === "robot"));
  const computers = robotHost
    ? topology.hosts.filter((host) => host !== robotHost) : topology.hosts;
  computerSlots.forEach((slot, index) => {
    const host = computers[index];
    field(slot, "unused").checked = !host;
    if (host) fillHost(slot, host);
    setCardActive(slot);
  });
  if (robotHost) fillHost("robot", robotHost);
  roleLocations.robot = "robot";
  updateSshVisibility();
  updateSecurityWarning();
  renderRoleBlocks();
  renderDerivedPeers();
}

function renderDerivedPeers() {
  const active = activeSlots();
  const mode = byId("discovery")?.value || "multicast";
  const rows = active.map((slot) => {
    const name = field(slot, "host-id").value.trim() || slot;
    if (mode === "multicast") return `${name}: ${t("derived.multicast")}`;
    const peers = active.filter((other) => other !== slot)
      .map((other) => field(other, "dds-address").value.trim() || "?");
    return `${name}: ${peers.join(", ")}`;
  });
  byId("derived-peers").textContent = rows.join("\n") || "—";
}

function updateSecurityWarning() {
  const sros2 = byId("security").value === "sros2";
  byId("security-warning").textContent = sros2
    ? t("graph.sros2.help") : t("graph.trusted.warning");
  byId("security-warning").classList.toggle("safe", sros2);
  const running = ["running", "cancelling"].includes(byId("job-status").dataset.status || "");
  byId("apply").textContent = t(sros2 ? "action.provision" : "action.deploy");
  byId("apply").disabled = running;
  byId("rotate").disabled = running || !sros2;
  updateWorkflow(running);
}

function updateWorkflow(running = false) {
  const stage = byId("workflow-stage");
  if (!stage) return;
  const apply = byId("apply");
  const sros2 = byId("security").value === "sros2";
  apply.textContent = t(sros2 ? "action.provision" : "action.deploy");
  apply.disabled = running;
  byId("runtime-start").disabled = running || !workflowApplied;
  stage.textContent = running
    ? t("workflow.stage.running")
    : workflowApplied
      ? t("workflow.stage.ready")
      : workflowSaved
        ? t("workflow.stage.saved")
        : t("workflow.stage.unsaved");
}

async function saveTopology({quiet = false, invalidate = true} = {}) {
  const topology = topologyFromForm();
  const result = await api("/api/save", {method: "POST", body: JSON.stringify(topology)});
  workflowSaved = true;
  if (invalidate) workflowApplied = false;
  updateWorkflow();
  if (!quiet) showNotice("notice.saved");
  renderServerPeers(result.derived_static_peers);
  return result;
}

function renderServerPeers(peers) {
  if (byId("discovery").value !== "static") return;
  byId("derived-peers").textContent = Object.entries(peers)
    .map(([host, values]) => `${host}: ${values.join(", ")}`)
    .join("\n");
}

function renderPreflightResult(result) {
  const lines = [t("preflight.ok")];
  result.preflight.hosts.forEach((host) => {
    const peers = result.derived_static_peers[host.id] || [];
    lines.push(`DDS ${host.id}: ${host.dds.address} / ${host.dds.interface}`);
    if (result.preflight.discovery_mode === "static") {
      lines.push(`  peers: ${peers.join(", ") || "—"}`);
    }
    const ssh = result.ssh_checks[host.id];
    if (ssh) {
      const status = ssh.checked ? t("preflight.ssh.checked") : t("preflight.ssh.skipped");
      lines.push(`SSH ${host.id}: ${ssh.host}:${ssh.port} (${status})`);
    }
  });
  byId("preflight-result").textContent = lines.join("\n");
}

async function runPreflight() {
  const result = await api("/api/preflight", {
    method: "POST",
    body: JSON.stringify(preflightFromForm())
  });
  renderPreflightResult(result);
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
  if (action === "rotate" && !window.confirm(t("rotate.confirm"))) return;
  await saveTopology({quiet: true, invalidate: !["start", "stop", "restart", "check"].includes(action)});
  await api(`/api/job/${action}`, {method: "POST", body: JSON.stringify({})});
  setJobRunning(true);
  if (pollTimer) window.clearInterval(pollTimer);
  pollTimer = window.setInterval(pollJob, 500);
  await pollJob();
}

async function runApplyJob() {
  const action = byId("security").value === "sros2" ? "provision" : "deploy";
  await startJob(action);
}

function markWorkflowDirty() {
  workflowSaved = false;
  workflowApplied = false;
  updateWorkflow();
}

function renderRuntimeStatus(result) {
  if (!result?.available) {
    byId("runtime-status").textContent = result?.reason || t("runtime.unavailable");
    return;
  }
  const rows = (result.hosts || []).map((host) => {
    const roles = (host.roles || []).join(", ");
    const state = host.reachable ? (host.state || "unknown") : t("runtime.unreachable");
    const detail = host.detail ? ` — ${host.detail}` : "";
    return `${host.display_name || host.host_id}: ${state} [${roles}]${detail}`;
  });
  byId("runtime-status").textContent = rows.join("\n") || "—";
}

async function pollRuntimeStatus() {
  try {
    renderRuntimeStatus(await api("/api/runtime"));
  } catch (error) {
    byId("runtime-status").textContent = error instanceof Error ? error.message : String(error);
  }
}

function setJobRunning(running) {
  ["save", "preflight", "apply", "topology-mode", "runtime-check", "runtime-start", "runtime-stop", "runtime-restart"].forEach((id) => { byId(id).disabled = running; });
  updateSecurityWarning();
  byId("cancel").disabled = !running;
}

async function pollJob() {
  try {
    const job = await api("/api/job");
    const key = `job.${job.status}`;
    byId("job-status").dataset.status = job.status;
    byId("job-status").textContent = `${t(key)}${job.action ? ` · ${t(`action.${job.action}`)}` : ""}`;
    byId("job-log").textContent = [...job.logs, job.error].filter(Boolean).join("\n");
    const running = ["running", "cancelling"].includes(job.status);
    if (job.status === "completed" && ["provision", "deploy"].includes(job.action)) {
      workflowSaved = true;
      workflowApplied = true;
    }
    setJobRunning(running);
    updateWorkflow(running);
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
  document.querySelectorAll(".drop-zone").forEach((zone) => {
    zone.addEventListener("dragover", (event) => {
      const target = zone.dataset.dropSlot;
      if (target !== "robot" && isActive(target)) {
        event.preventDefault();
        event.dataTransfer.dropEffect = "move";
      }
    });
    zone.addEventListener("drop", (event) => {
      event.preventDefault();
      moveRole(event.dataTransfer.getData("text/plain"), zone.dataset.dropSlot);
    });
  });
  computerSlots.forEach((slot) => {
    field(slot, "unused").addEventListener("change", () => setCardActive(slot));
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
    ["host-id", "dds-address"].forEach((name) => {
      field(slot, name).addEventListener("input", renderDerivedPeers);
    });
  });
  document.querySelectorAll("input, select").forEach((control) => {
    control.addEventListener("input", markWorkflowDirty);
    control.addEventListener("change", markWorkflowDirty);
  });
  document.querySelectorAll("[data-probe-slot]").forEach((button) => {
    button.addEventListener("click", () => probeSsh(button.dataset.probeSlot).catch(showError));
  });
  byId("discovery").addEventListener("change", renderDerivedPeers);
  byId("topology-mode").addEventListener("change", (event) => {
    applyTopologyMode(event.target.value);
  });
  byId("security").addEventListener("change", updateSecurityWarning);
  byId("save").addEventListener("click", () => saveTopology().catch(showError));
  byId("preflight").addEventListener("click", () => runPreflight().catch(showError));
  byId("apply").addEventListener("click", () => runApplyJob().catch(showError));
  byId("rotate").addEventListener("click", () => startJob("rotate").catch(showError));
  byId("runtime-check").addEventListener("click", () => pollRuntimeStatus().catch(showError));
  ["start", "stop", "restart"].forEach((action) => {
    byId(`runtime-${action}`).addEventListener("click", () => startJob(action).catch(showError));
  });
  byId("cancel").addEventListener("click", async () => {
    try { await api("/api/cancel", {method: "POST", body: JSON.stringify({})}); }
    catch (error) { showError(error); }
  });
}

async function initialize() {
  try {
    catalog = await fetch("/i18n.json", {cache: "no-store"}).then((response) => response.json());
    bindEvents();
    applyLanguage("ko");
    computerSlots.forEach(setCardActive);
    const context = await api("/api/context");
    schemaVersion = context.schema_version;
    applyTopologyMode(context.topology?.topology_mode || "full");
    if (context.topology) {
      workflowSaved = true;
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
    updateSecurityWarning();
    updateWorkflow();
    renderRoleBlocks();
    renderDerivedPeers();
    await pollJob();
    await pollRuntimeStatus();
    if (runtimePollTimer) window.clearInterval(runtimePollTimer);
    runtimePollTimer = window.setInterval(pollRuntimeStatus, 10000);
  } catch (error) {
    showError(error);
  }
}

initialize();
