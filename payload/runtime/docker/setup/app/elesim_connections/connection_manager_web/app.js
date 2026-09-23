"use strict";

const query = new URLSearchParams(window.location.search);
if (query.get("token")) {
  sessionStorage.setItem("elesimConnectionToken", query.get("token"));
  history.replaceState({}, "", window.location.pathname);
}

const token = sessionStorage.getItem("elesimConnectionToken") || "";
const computerSlots = [];
const slots = computerSlots;
const hostKinds = {};
// Read-only installation lookup results are kept per COM card.  A release is
// selected by a role card, so this catalog is deliberately not global: moving
// a role to another computer must never reuse the source computer's choices.
const installationCatalogs = {};
// Native Robot installations are a separate deployment unit from the
// container installation on a Jetson.  Keep their lookup state separate too;
// an installation UUID/project from one unit must never populate the other.
const robotInstallationCatalogs = {};
const maximumHosts = 4;
const applicationRoles = ["pilot", "ui", "sim", "robot"];

let catalog = {};
let language = "ko";
let runtimePollInFlight = false;
let schemaVersion = 5;
let roleCards = [];
let nextCardId = 1;
let nextRoleNumbers = {pilot: 1, ui: 1, sim: 1, robot: 1};
let pendingRoleSlot = "";
let dropPreviewKey = "";
let dropPlaceholder = null;
let dragScrollFrame = null;
let dragScrollSpeed = 0;
let pollTimer = null;
let runtimePollTimer = null;
let runtimeOptionsLocked = false;
let gpuInheritAvailable = null;
let gpuPolicies = {pilot: null, sim: null};
let gpuDevices = {pilot: [], sim: []};
let jobSubmissionPending = false;
let workflowSaved = false;
let workflowApplied = false;
let runtimeReady = false;
let runtimeRevision = 0;
let workflowStarted = false;
// Do not let a click race the initial context/topology load.  The static page
// is rendered before the API context arrives, so the form is not a valid save
// source until initialization has completed.
let pageReady = false;
// A browser session always revalidates the loaded form before it can proceed.
// The topology fields are still restored from disk, but a previous session's
// visual stage must never unlock Booting just because the local Authority has
// an active generation; that does not prove every host has its role bundle.
let workflowRequiresFreshSave = true;
const workflowStates = {save: "pending", apply: "pending", start: "pending"};

const byId = (id) => document.getElementById(id);
const card = (slot) => document.querySelector(`.host-card[data-slot="${slot}"]`);
const field = (slot, name) => card(slot).querySelector(`[data-field="${name}"]`);

function nextComputerSlot() {
  for (let index = 1; index <= maximumHosts; index += 1) {
    const slot = `com${index}`;
    if (!computerSlots.includes(slot)) return slot;
  }
  return "";
}

function isRobotHost(slot) {
  return hostKinds[slot] === "robot";
}

function robotSlots() {
  return computerSlots.filter(isRobotHost);
}

function updateRobotInstallationVisibility(slot) {
  const section = card(slot)?.querySelector(".robot-install-fields");
  if (section) section.hidden = !isRobotHost(slot);
  const runtime = card(slot)?.querySelector(".install-fields");
  if (runtime) runtime.hidden = isRobotHost(slot)
    && !roleCards.some((entry) => entry.slot === slot && entry.role !== "robot");
}

function createHost({robot = false, slot = "", operational = false} = {}) {
  const selectedSlot = slot || nextComputerSlot();
  if (!selectedSlot || computerSlots.includes(selectedSlot)) return "";
  if (computerSlots.length >= maximumHosts) {
    showError(t("error.host.maximum"));
    return "";
  }
  if (robot && robotSlots().length) {
    showError(t("error.host.robot.exists"));
    return "";
  }
  const template = byId("host-template");
  const hostCard = template.content.firstElementChild.cloneNode(true);
  hostCard.dataset.slot = selectedSlot;
  hostCard.classList.toggle("robot-host", robot);
  hostKinds[selectedSlot] = robot ? "robot" : "computer";
  installationCatalogs[selectedSlot] = null;
  robotInstallationCatalogs[selectedSlot] = null;
  computerSlots.push(selectedSlot);
  hostCard.querySelector(".host-name").value = selectedSlot;
  hostCard.querySelector(".robot-badge").hidden = !robot;
  hostCard.querySelector(".robot-lane").hidden = !robot;
  hostCard.querySelectorAll(".drop-zone").forEach((zone) => {
    zone.dataset.dropSlot = selectedSlot;
    zone.setAttribute("aria-label", `${selectedSlot} ${zone.dataset.dropUnit} roles`);
  });
  const local = hostCard.querySelector('input[name="local-host"]');
  local.value = selectedSlot;
  local.checked = !robot && (operational || computerSlots.length === 1);
  hostCard.querySelector(".probe").dataset.probeSlot = selectedSlot;
  byId("host-grid").append(hostCard);
  updateRobotInstallationVisibility(selectedSlot);
  bindHostCardEvents(selectedSlot);
  updateHostLimit();
  updateHostOrderButtons();
  updateSshVisibility();
  return selectedSlot;
}

function fieldFromCard(hostCard, name) {
  return hostCard.querySelector(`[data-field="${name}"]`);
}

function updateHostOrderButtons() {
  computerSlots.forEach((slot, index) => {
    const hostCard = card(slot);
    if (!hostCard) return;
    hostCard.querySelector(".move-host-up").disabled = index === 0;
    hostCard.querySelector(".move-host-down").disabled = index === computerSlots.length - 1;
  });
}

function moveHost(slot, offset) {
  const index = computerSlots.indexOf(slot);
  const destination = index + offset;
  if (index < 0 || destination < 0 || destination >= computerSlots.length) return;
  [computerSlots[index], computerSlots[destination]] = [
    computerSlots[destination],
    computerSlots[index],
  ];
  const grid = byId("host-grid");
  computerSlots.forEach((orderedSlot) => grid.append(card(orderedSlot)));
  updateHostOrderButtons();
  markWorkflowDirty();
}

function beginHostRename(slot) {
  const input = card(slot).querySelector(".host-name");
  input.dataset.previous = input.value;
  input.readOnly = false;
  input.focus();
  input.select();
}

function finishHostRename(slot) {
  const input = card(slot).querySelector(".host-name");
  if (input.readOnly) return;
  const candidate = input.value.trim().toLowerCase();
  const duplicate = computerSlots.some((other) => (
    other !== slot
    && card(other).querySelector(".host-name").value.trim().toLowerCase() === candidate
  ));
  if (!/^[a-z][a-z0-9_-]{0,62}$/.test(candidate) || duplicate) {
    input.value = input.dataset.previous || slot;
    showError(t("error.host.id"));
  } else {
    input.value = candidate;
    markWorkflowDirty();
  }
  input.readOnly = true;
}

function removeHost(slot) {
  if (computerSlots.length === 1) {
    showError(t("error.one.com"));
    return;
  }
  const destination = firstActiveCom(slot);
  roleCards.forEach((roleCard) => {
    if (roleCard.slot !== slot) return;
    if (roleCard.role === "robot") {
      roleCard.slot = "";
      return;
    }
    const target = roleCard.role === "sim" ? firstActiveRuntime(slot) : destination;
    roleCard.slot = target || "";
    if (target) rebindRoleRelease(roleCard, target);
  });
  roleCards = roleCards.filter((roleCard) => roleCard.slot);
  const local = document.querySelector('input[name="local-host"]:checked');
  card(slot)?.remove();
  const index = computerSlots.indexOf(slot);
  if (index >= 0) computerSlots.splice(index, 1);
  delete hostKinds[slot];
  delete installationCatalogs[slot];
  delete robotInstallationCatalogs[slot];
  updateHostOrderButtons();
  if (local?.value === slot && computerSlots.length) {
    const replacement = firstActiveRuntime();
    if (replacement) {
      card(replacement).querySelector('input[name="local-host"]').checked = true;
    }
  }
  markWorkflowDirty();
  updateHostLimit();
  updateSshVisibility();
  renderRoleBlocks();
}

function updateHostLimit() {
  const button = byId("add-host");
  if (button) button.disabled = activeSlots().length >= maximumHosts;
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

function showNotice(key, detail = "") {
  const banner = byId("notice-banner");
  const message = t(key);
  banner.querySelector(".banner-message").textContent = detail
    ? `${message}\n${detail}`
    : message;
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
  slots.forEach(updateSshMode);
  renderRoleBlocks();
  updateRoleChoices();
  updateInstallationLookupButtons();
  updateWorkflow();
}

function isActive(slot) {
  return computerSlots.includes(slot);
}

function visibleRoles() {
  return roleCards.filter((roleCard) => applicationRoles.includes(roleCard.role) && roleCard.slot);
}

function activeSlots() {
  return slots.filter(isActive);
}

function firstActiveCom(except = "") {
  return slots.find((slot) => slot !== except && isActive(slot)) || "";
}

function firstActiveRuntime(except = "") {
  return slots.find((slot) => (
    slot !== except && isActive(slot) && !isRobotHost(slot)
  )) || "";
}

function canPlaceRole(role, target, {notify = true} = {}) {
  if (role === "robot" && !isRobotHost(target)) {
    if (notify) showError(t("error.robot.jetson"));
    return false;
  }
  const local = document.querySelector('input[name="local-host"]:checked')?.value;
  if (role === "robot" && local === target) {
    if (notify) showError(t("error.local.robot"));
    return false;
  }
  if (role === "sim" && isRobotHost(target)) {
    if (notify) showError(t("error.sim.jetson"));
    return false;
  }
  return true;
}

function defaultEndpointId(role, number) {
  return `${role}-${number}`;
}

function appendRoleCard(role, slot, endpointId = "", releaseKey = "") {
  const number = nextRoleNumbers[role];
  nextRoleNumbers[role] += 1;
  const roleCard = {
    id: `role-card-${nextCardId}`,
    role,
    slot,
    number,
    endpointId: endpointId || defaultEndpointId(role, number),
    releaseKey: role === "robot" ? "" : String(releaseKey || ""),
  };
  nextCardId += 1;
  roleCards.push(roleCard);
  return roleCard;
}

function installationFieldNames(kind = "runtime") {
  if (kind === "robot") {
    return {
      root: "robot-install-root",
      bin: "robot-bin-dir",
      uuid: "robot-install-uuid",
      project: "robot-install-project",
      name: "robot-install-name",
    };
  }
  return {
    root: "install-root",
    bin: "bin-dir",
    uuid: "install-uuid",
    project: "install-project",
    name: "install-name",
  };
}

function installationField(slot, kind, name) {
  return field(slot, installationFieldNames(kind)[name]);
}

function installationCatalog(slot, kind = "runtime") {
  return kind === "robot"
    ? robotInstallationCatalogs[slot]
    : installationCatalogs[slot];
}

function setInstallationCatalog(slot, kind, value) {
  if (kind === "robot") robotInstallationCatalogs[slot] = value;
  else installationCatalogs[slot] = value;
}

function selectedInstallation(slot, kind = "runtime") {
  const catalog = installationCatalog(slot, kind);
  if (!Array.isArray(catalog)) return null;
  const uuid = installationField(slot, kind, "uuid").value.trim();
  return catalog.find((item) => item?.install_uuid === uuid) || null;
}

function installationOptionLabel(item) {
  const base = item?.name || item?.project || item?.install_uuid || "";
  return item?.install_mode === "native"
    ? `${t("install.native")} - ${base}`
    : base;
}

function roleReleaseChoices(slot, role) {
  const installation = selectedInstallation(slot);
  if (!installation || !Array.isArray(installation.releases)) return [];
  return installation.releases.filter((release) => (
    release
    && typeof release.key === "string"
    && (!Array.isArray(release.roles) || release.roles.includes(role))
  ));
}

function releaseOptionLabel(release, role) {
  const roleLabel = release?.role_labels?.[role];
  if (typeof roleLabel === "string" && roleLabel.trim()) {
    const prefix = typeof release.label === "string"
      ? release.label.match(/^\d+\.\s*/)?.[0] || ""
      : "";
    return `${prefix}${roleLabel}`;
  }
  return typeof release.label === "string" ? release.label : release.key;
}

function refreshRoleReleaseSelect(roleCard, select) {
  const choices = roleReleaseChoices(roleCard.slot, roleCard.role);
  const catalogReady = Array.isArray(installationCatalogs[roleCard.slot]);
  const installation = selectedInstallation(roleCard.slot);
  const current = String(roleCard.releaseKey || "");
  const valid = choices.some((release) => release.key === current);
  if (catalogReady && (!installation || (current && !valid))) {
    roleCard.releaseKey = "";
  }
  const selected = String(roleCard.releaseKey || "");
  const placeholderKey = !catalogReady
    ? "install.queryFirst"
    : (choices.length ? "install.chooseRelease" : "install.noReleases");
  const options = [new Option(t(placeholderKey), "")];
  options.push(...choices.map((release) => new Option(
    releaseOptionLabel(release, roleCard.role),
    release.key,
  )));
  if (!catalogReady && selected && !choices.some((release) => release.key === selected)) {
    options.push(new Option(t("install.saved"), selected));
  }
  select.replaceChildren(...options);
  select.value = selected;
}

function refreshRoleReleaseOptions(slot) {
  roleCards
    .filter((roleCard) => roleCard.slot === slot && roleCard.role !== "robot")
    .forEach((roleCard) => {
      const block = document.querySelector(`[data-card-id="${roleCard.id}"]`);
      const select = block?.querySelector(".role-release");
      if (select) refreshRoleReleaseSelect(roleCard, select);
    });
}

function rebindRoleRelease(roleCard, target) {
  if (roleCard.role === "robot") return;
  const choices = roleReleaseChoices(target, roleCard.role);
  const catalogReady = Array.isArray(installationCatalogs[target]);
  if (!catalogReady || !selectedInstallation(target)) {
    roleCard.releaseKey = "";
  } else if (!choices.some((release) => release.key === roleCard.releaseKey)) {
    roleCard.releaseKey = "";
  }
}

function roleCardById(cardId) {
  return roleCards.find((roleCard) => roleCard.id === cardId);
}

function renderRoleBlocks() {
  computerSlots.forEach(updateRobotInstallationVisibility);
  clearDropPreview();
  document.querySelectorAll(".drop-zone").forEach((zone) => {
    zone.replaceChildren();
    zone.classList.add("empty");
    if (zone.dataset.dropUnit === "runtime") {
      const guidance = document.createElement("p");
      guidance.className = "drop-guidance";
      guidance.textContent = t("role.add.guidance");
      zone.append(guidance);
    }
  });
  visibleRoles().forEach((roleCard) => {
    const {role, slot} = roleCard;
    const unit = role === "robot" ? "robot" : "runtime";
    const zone = document.querySelector(`[data-drop-slot="${slot}"][data-drop-unit="${unit}"]`);
    if (!zone) return;
    zone.classList.remove("empty");
    const block = document.createElement("div");
    block.className = `role-block role-${role}`;
    block.dataset.cardId = roleCard.id;
    block.dataset.role = role;
    block.draggable = role !== "robot";
    const title = document.createElement("strong");
    title.textContent = `${t(`role.${role}`)} ${roleCard.number}`;
    const endpoint = document.createElement("label");
    const label = document.createElement("span");
    label.textContent = t("role.endpoint");
    const input = document.createElement("input");
    input.type = "text";
    input.value = roleCard.endpointId;
    input.spellcheck = false;
    input.addEventListener("input", () => {
      roleCard.endpointId = input.value;
      markWorkflowDirty();
    });
    endpoint.append(label, input);
    let release = null;
    if (role !== "robot") {
      release = document.createElement("label");
      const releaseLabel = document.createElement("span");
      releaseLabel.textContent = t("role.release");
      const releaseSelect = document.createElement("select");
      releaseSelect.className = "role-release";
      releaseSelect.setAttribute("aria-label", t("role.release"));
      refreshRoleReleaseSelect(roleCard, releaseSelect);
      releaseSelect.addEventListener("change", () => {
        roleCard.releaseKey = releaseSelect.value.trim();
        markWorkflowDirty();
      });
      release.append(releaseLabel, releaseSelect);
    }
    const remove = document.createElement("button");
    remove.type = "button";
    remove.className = "icon-button remove-role";
    remove.textContent = "×";
    remove.setAttribute("aria-label", t("action.remove.role"));
    remove.addEventListener("click", () => {
      roleCards = roleCards.filter((candidate) => candidate.id !== roleCard.id);
      markWorkflowDirty();
      renderRoleBlocks();
      updateRoleChoices();
    });
    block.append(title, endpoint);
    if (release) block.append(release);
    block.append(remove);
    if (role !== "robot") {
      block.addEventListener("dragstart", (event) => {
        clearDropPreview();
        event.dataTransfer.effectAllowed = "move";
        event.dataTransfer.setData("text/plain", roleCard.id);
        block.classList.add("dragging");
        window.setTimeout(() => block.classList.add("drag-source-hidden"), 0);
      });
      block.addEventListener("dragend", () => {
        block.classList.remove("dragging", "drag-source-hidden");
        clearDropPreview();
        stopDragAutoScroll();
      });
    }
    zone.append(block);
  });
}

function insertRoleInOrder(cardId, target, targetUnit, targetCardId = "", insertBefore = false) {
  if (targetCardId === cardId) return;
  const moving = roleCardById(cardId);
  if (!moving) return;
  roleCards = roleCards.filter((candidate) => candidate.id !== cardId);
  moving.slot = target;

  const targetCards = roleCards.filter((candidate) => {
    if (candidate.slot !== target) return false;
    return targetUnit === "robot" ? candidate.role === "robot" : candidate.role !== "robot";
  });
  let insertionIndex = roleCards.length;
  if (targetCardId && roleCardById(targetCardId)?.slot === target) {
    const targetIndex = roleCards.findIndex((candidate) => candidate.id === targetCardId);
    if (targetIndex >= 0) insertionIndex = targetIndex + (insertBefore ? 0 : 1);
  } else if (targetCards.length) {
    insertionIndex = roleCards.findIndex(
      (candidate) => candidate.id === targetCards[targetCards.length - 1].id,
    ) + 1;
  }
  roleCards.splice(insertionIndex, 0, moving);
}

function moveRole(
  cardId,
  target,
  targetUnit = "runtime",
  targetCardId = "",
  insertBefore = false,
) {
  const roleCard = roleCardById(cardId);
  if (!roleCard || roleCard.role === "robot" || !isActive(target)) return;
  const role = roleCard.role;
  const source = roleCard.slot;
  if (targetUnit === "robot" && role !== "robot") {
    showError(t("error.unit.robot"));
    return;
  }
  if (targetUnit === "runtime" && role === "robot") {
    showError(t("error.unit.runtime"));
    return;
  }
  if (!canPlaceRole(role, target)) return;
  insertRoleInOrder(cardId, target, targetUnit, targetCardId, insertBefore);
  if (source !== target) rebindRoleRelease(roleCard, target);
  markWorkflowDirty();
  renderRoleBlocks();
}

function dropPlacement(zone, pointerX, pointerY, draggedCardId = "") {
  const containsPointer = (rect) => (
    pointerX >= rect.left
    && pointerX <= rect.right
    && pointerY >= rect.top
    && pointerY <= rect.bottom
  );
  if (dropPlaceholder?.parentElement === zone && containsPointer(
    dropPlaceholder.getBoundingClientRect(),
  )) {
    return {
      targetCardId: dropPlaceholder.dataset.targetCardId || "",
      insertBefore: Boolean(dropPlaceholder.dataset.targetCardId),
    };
  }
  const blocks = [...zone.querySelectorAll(".role-block")].filter(
    (block) => block.dataset.cardId && block.dataset.cardId !== draggedCardId,
  );
  if (!blocks.length) return {targetCardId: "", insertBefore: false};

  for (const block of blocks) {
    const rect = block.getBoundingClientRect();
    if (containsPointer(rect)) {
      return {targetCardId: block.dataset.cardId, insertBefore: true};
    }
  }
  return {targetCardId: "", insertBefore: false};
}

function clearDropPreview() {
  dropPlaceholder?.remove();
  dropPlaceholder = null;
  dropPreviewKey = "";
}

function updateDropPreview(zone, placement, draggedCardId) {
  if (!placement) {
    clearDropPreview();
    return;
  }
  const key = [
    zone.dataset.dropSlot,
    zone.dataset.dropUnit || "runtime",
    draggedCardId,
    placement.targetCardId,
    placement.insertBefore,
  ].join(":");
  if (key === dropPreviewKey) return;

  const positions = new Map(
    [...document.querySelectorAll(".role-block[data-card-id]")].map((block) => [
      block.dataset.cardId,
      block.getBoundingClientRect(),
    ]),
  );
  document.querySelectorAll(".role-block[data-card-id]").forEach((block) => {
    block.getAnimations?.().forEach((animation) => animation.cancel());
  });
  clearDropPreview();
  dropPlaceholder = document.createElement("div");
  dropPlaceholder.className = "role-block drop-placeholder";
  dropPlaceholder.dataset.targetCardId = placement.targetCardId;
  dropPlaceholder.setAttribute("aria-hidden", "true");
  const target = placement.targetCardId
    ? zone.querySelector(`[data-card-id="${placement.targetCardId}"]`)
    : null;
  zone.insertBefore(dropPlaceholder, target);
  document.querySelectorAll(".role-block[data-card-id]").forEach((block) => {
    const previous = positions.get(block.dataset.cardId);
    if (!previous || !block.animate) return;
    const current = block.getBoundingClientRect();
    const offsetX = previous.left - current.left;
    const offsetY = previous.top - current.top;
    if (!offsetX && !offsetY) return;
    block.animate(
      [
        {transform: `translate(${offsetX}px, ${offsetY}px)`},
        {transform: "translate(0, 0)"},
      ],
      {duration: 150, easing: "ease-out"},
    );
  });
  dropPreviewKey = key;
}

function stopDragAutoScroll() {
  dragScrollSpeed = 0;
  if (dragScrollFrame !== null) window.cancelAnimationFrame(dragScrollFrame);
  dragScrollFrame = null;
}

function runDragAutoScroll() {
  if (!dragScrollSpeed) {
    dragScrollFrame = null;
    return;
  }
  window.scrollBy(0, dragScrollSpeed);
  dragScrollFrame = window.requestAnimationFrame(runDragAutoScroll);
}

function updateDragAutoScroll(pointerY) {
  const edge = Math.min(120, window.innerHeight * 0.18);
  if (pointerY < edge) {
    dragScrollSpeed = -Math.ceil(4 + 16 * (edge - pointerY) / edge);
  } else if (pointerY > window.innerHeight - edge) {
    dragScrollSpeed = Math.ceil(4 + 16 * (pointerY - window.innerHeight + edge) / edge);
  } else {
    stopDragAutoScroll();
    return;
  }
  if (dragScrollFrame === null) {
    dragScrollFrame = window.requestAnimationFrame(runDragAutoScroll);
  }
}

function updateSshVisibility() {
  const local = document.querySelector('input[name="local-host"]:checked')?.value;
  slots.forEach((slot) => {
    const hostCard = card(slot);
    if (!hostCard) return;
    const details = hostCard.querySelector(".ssh-fields");
    const warning = hostCard.querySelector(".local-security-warning");
    details.hidden = !isActive(slot) || local === slot;
    warning.hidden = !isActive(slot) || local !== slot;
    hostCard.classList.toggle("local", local === slot && isActive(slot));
    updateSshMode(slot);
  });
  updateInstallationLookupButtons();
}

function installationLookupReadyFor(slot, kind = "runtime") {
  if (kind === "runtime") return installationLookupReady(slot);
  if (!isActive(slot)) return false;
  if (kind === "robot" && !isRobotHost(slot)) return false;
  const local = document.querySelector('input[name="local-host"]:checked')?.value;
  // Local lookups use the manager's own, already validated installation path;
  // only remote cards need a pinned SSH host key before they can be queried.
  return local === slot || Boolean(field(slot, "ssh-fingerprint")?.value.trim());
}

function installationLookupReady(slot) {
  if (!isActive(slot)) return false;
  const local = document.querySelector('input[name="local-host"]:checked')?.value;
  // Local lookups use the manager's own, already validated installation path;
  // only remote cards need a pinned SSH host key before they can be queried.
  return local === slot || Boolean(field(slot, "ssh-fingerprint")?.value.trim());
}

function updateInstallationLookupFor(slot, kind = "runtime") {
  if (kind === "runtime") {
    updateInstallationLookup(slot);
    return;
  }
  const selector = kind === "robot"
    ? ".robot-lookup-installation"
    : ".lookup-installation:not(.robot-lookup-installation)";
  const button = card(slot)?.querySelector(selector);
  if (!button) return;
  const ready = installationLookupReadyFor(slot, kind);
  const busy = button.dataset.lookupBusy === "true";
  button.dataset.i18n = ready ? "install.lookup" : "install.lookup.blocked";
  button.textContent = t(button.dataset.i18n);
  button.disabled = busy || !ready;
  button.setAttribute("aria-disabled", String(button.disabled));
}

function updateInstallationLookup(slot) {
  const button = card(slot)?.querySelector(".lookup-installation");
  if (!button) return;
  const ready = installationLookupReady(slot);
  const busy = button.dataset.lookupBusy === "true";
  button.dataset.i18n = ready ? "install.lookup" : "install.lookup.blocked";
  button.textContent = t(button.dataset.i18n);
  button.disabled = busy || !ready;
  button.setAttribute("aria-disabled", String(button.disabled));
}

function updateInstallationLookupButtons() {
  slots.forEach((slot) => {
    updateInstallationLookup(slot);
    updateInstallationLookupFor(slot, "robot");
  });
}

function updateSshMode(slot) {
  const tailscale = field(slot, "ssh-tailscale").checked;
  const active = isActive(slot);
  const port = field(slot, "ssh-port");
  const key = field(slot, "ssh-key");
  if (tailscale) {
    port.value = "22";
    key.value = t("ssh.key.notRequired");
    key.dataset.tailscaleLabel = "true";
  } else if (key.dataset.tailscaleLabel === "true") {
    key.value = "";
    delete key.dataset.tailscaleLabel;
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

function sshHostFromForm(slot) {
  const host = field(slot, "ssh-host").value.trim();
  if (!host) throw new Error(`${t("error.ssh.host.required")} (${slot})`);
  return host;
}

function sshEndpointFromForm(slot) {
  const host = sshHostFromForm(slot);
  const port = sshPort(slot);
  if (!Number.isInteger(port) || port < 1 || port > 65535) {
    throw new Error(`${t("error.ssh.port.invalid")} (${slot})`);
  }
  const user = field(slot, "ssh-user").value.trim();
  if (!user) throw new Error(`${t("error.ssh.user.required")} (${slot})`);
  const pinnedFingerprint = field(slot, "ssh-fingerprint").value.trim();
  if (!pinnedFingerprint) {
    throw new Error(`${t("error.ssh.fingerprint.required")} (${slot})`);
  }
  const tailscale = field(slot, "ssh-tailscale").checked;
  return {
    host,
    port,
    user,
    identity_file: tailscale ? "" : field(slot, "ssh-key").value.trim(),
    pinned_fingerprint: pinnedFingerprint,
    auth_mode: tailscale ? "tailscale" : "openssh",
  };
}

function topologyFromForm() {
  const localSlot = document.querySelector('input[name="local-host"]:checked')?.value || "";
  const active = activeSlots();
  if (active.length < 1 || active.length > maximumHosts) throw new Error(t("error.host.count"));
  if (!active.includes(localSlot)) throw new Error(t("error.local"));
  const visible = visibleRoles();
  if (isRobotHost(localSlot)) {
    throw new Error(t("error.local.robot"));
  }
  const empty = active.find((slot) => !visible.some((roleCard) => roleCard.slot === slot));
  if (empty) throw new Error(`${t("error.empty.host")} (${empty})`);
  const hosts = active.map((slot) => {
    const local = slot === localSlot;
    const cardsForHost = visible.filter((roleCard) => roleCard.slot === slot);
    const assignments = cardsForHost.map((roleCard) => ({
      role: roleCard.role,
      endpoint_id: roleCard.endpointId.trim(),
      ...(roleCard.role !== "robot" && roleCard.releaseKey
        ? {release_key: roleCard.releaseKey.trim()}
        : {}),
    }));
    const runtimeRoles = assignments.filter((item) => item.role !== "robot");
    const installRoot = installationField(slot, "runtime", "root").value.trim();
    const binDir = installationField(slot, "runtime", "bin").value.trim();
    const installUuid = installationField(slot, "runtime", "uuid").value.trim();
    const installProject = installationField(slot, "runtime", "project").value.trim();
    const releaseKeys = [...new Set(
      runtimeRoles.map((assignment) => assignment.release_key).filter(Boolean),
    )];
    if (releaseKeys.length > 1) throw new Error(t("error.release.mixed"));
    const units = [];
    if (runtimeRoles.length) {
      units.push({
        id: "runtime",
        assignments: runtimeRoles,
        install_mode: "container",
        install_root: installRoot,
        bin_dir: binDir,
        lifecycle: "compose",
        ...(installUuid ? {install_uuid: installUuid} : {}),
        ...(installProject ? {project: installProject} : {}),
        ...(releaseKeys[0] ? {release_key: releaseKeys[0]} : {})
      });
    }
    const robotAssignments = assignments.filter((item) => item.role === "robot");
    if (robotAssignments.length) {
      const robotInstallRoot = installationField(slot, "robot", "root").value.trim();
      const robotBinDir = installationField(slot, "robot", "bin").value.trim();
      const robotInstallUuid = installationField(slot, "robot", "uuid").value.trim();
      if (!robotInstallUuid) throw new Error(`${t("error.robot.installation")} (${slot})`);
      units.push({
        id: "robot-native",
        assignments: robotAssignments,
        install_mode: "native",
        install_root: robotInstallRoot,
        bin_dir: robotBinDir,
        lifecycle: "systemd",
        install_uuid: robotInstallUuid,
        ...(installationField(slot, "robot", "project").value.trim()
          ? {project: installationField(slot, "robot", "project").value.trim()}
          : {})
      });
    }
    const hostId = card(slot).querySelector(".host-name").value.trim().toLowerCase();
    const ssh = local ? null : sshEndpointFromForm(slot);
    const host = {
      id: hostId,
      local,
      dds: {
        address: field(slot, "dds-address").value.trim(),
        interface: field(slot, "dds-interface").value.trim(),
        ...(isTailscaleInterface(field(slot, "dds-interface").value)
          ? {address_source: "tailscale"} : {})
      },
      ssh,
      jetson: isRobotHost(slot),
      units
    };
    return host;
  });
  return {
    schema_version: schemaVersion,
    system_id: byId("system-id").value.trim(),
    security_profile: byId("security").value,
    dds_graph: {
      domain_id: Number(byId("domain-id").value),
      rmw_implementation: "rmw_cyclonedds_cpp",
      discovery_mode: "static"
    },
    hosts
  };
}

function fillHost(slot, host) {
  card(slot).querySelector(".host-name").value = host.id;
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
  const runtimeUnit = units.find((unit) => unit.install_mode === "container");
  const robotUnit = units.find((unit) => unit.install_mode === "native" || (unit.assignments || []).some((assignment) => assignment.role === "robot"));
  const installationUnits = [
    ["runtime", runtimeUnit],
    ["robot", robotUnit],
  ];
  installationUnits.forEach(([kind, unit]) => {
    installationField(slot, kind, "root").value = unit?.install_root
      || (kind === "robot" ? "/opt/elesim-robot" : "/opt/elesim");
    installationField(slot, kind, "bin").value = unit?.bin_dir
      || (kind === "robot" ? "/opt/elesim-robot/bin" : "/opt/elesim/bin");
    installationField(slot, kind, "uuid").value = unit?.install_uuid || "";
    installationField(slot, kind, "project").value = unit?.project || "";
    installationField(slot, kind, "name").replaceChildren(
      new Option(t("install.saved"), unit?.install_uuid || ""),
    );
  });
  installationCatalogs[slot] = null;
  robotInstallationCatalogs[slot] = null;
  updateRobotInstallationVisibility(slot);
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
  units.forEach((unit) => {
    (unit.assignments || []).forEach((assignment) => {
      appendRoleCard(
        assignment.role,
        slot,
        assignment.endpoint_id,
        assignment.release_key || (assignment.role === "robot" ? "" : unit.release_key || ""),
      );
    });
  });
}

function applyLocalTailscaleHint(context) {
  if (context?.manager_transport?.container_network_mode === "tailscale-sidecar") {
    return;
  }
  const hint = context?.tailscale;
  if (!hint?.available || !Array.isArray(hint.addresses) || !hint.addresses.length) return;
  const local = document.querySelector('input[name="local-host"]:checked')?.value || "com1";
  if (!isActive(local)) return;
  const address = field(local, "dds-address");
  const iface = field(local, "dds-interface");
  let changed = false;
  if (!address.value.trim()) {
    address.value = String(hint.addresses[0]);
    changed = true;
  }
  if (!iface.value.trim()) {
    iface.value = String(hint.interface || "tailscale0");
    changed = true;
  }
  if (changed) {
    // A detected address is only a form suggestion.  It must still be
    // validated and explicitly saved by the operator.
    markWorkflowDirty();
    showNotice("notice.tailscale.prefill");
  }
}

function isTailscaleInterface(value) {
  return /^tailscale[0-9]+$/i.test(String(value || "").trim());
}

function applyTopology(topology) {
  schemaVersion = topology.schema_version;
  byId("host-grid").replaceChildren();
  computerSlots.splice(0);
  Object.keys(hostKinds).forEach((slot) => delete hostKinds[slot]);
  Object.keys(installationCatalogs).forEach((slot) => delete installationCatalogs[slot]);
  Object.keys(robotInstallationCatalogs).forEach((slot) => delete robotInstallationCatalogs[slot]);
  roleCards = [];
  nextCardId = 1;
  nextRoleNumbers = {pilot: 1, ui: 1, sim: 1, robot: 1};
  byId("system-id").value = topology.system_id;
  byId("domain-id").value = topology.dds_graph.domain_id;
  byId("security").value = topology.security_profile;

  topology.hosts.forEach((host, index) => {
    const slot = createHost({
      robot: host.jetson === true,
      slot: `com${index + 1}`,
      operational: host.local === true,
    });
    fillHost(slot, host);
  });
  updateSshVisibility();
  updateWorkflow();
  renderRoleBlocks();
  updateRoleChoices();
}

function updateWorkflow(running = ["running", "cancelling"].includes(byId("job-status")?.dataset.status || "")) {
  // Keep the controls locked across the save request that precedes a job
  // submission.  That request is asynchronous and otherwise re-enables
  // "start all" before /api/job/start has accepted the job.
  const busy = running || jobSubmissionPending;
  const initializerComplete = workflowStarted;
  const apply = byId("apply");
  apply.textContent = t("action.prepare");
  const start = byId("runtime-start");
  const waiting = !initializerComplete && (!workflowApplied || !runtimeReady);
  start.dataset.i18n = waiting ? "action.wait" : "action.start";
  start.textContent = t(start.dataset.i18n);
  setWorkflowStepEnabled(
    "save",
    pageReady && !busy && !initializerComplete && !workflowSaved,
  );
  setWorkflowStepEnabled(
    "apply",
    !busy && !initializerComplete && workflowSaved && !workflowApplied,
  );
  setWorkflowButtonsEnabled("start", {
    "runtime-start": !busy && !initializerComplete && workflowSaved && workflowApplied && runtimeReady,
  });
  updateRuntimeOptions();
}

function runtimeLaunchOptions() {
  const roleOptions = (role) => {
    const inherit = byId(`${role}-gpu-inherit`);
    const device = byId(`${role}-gpu-device`);
    const policy = gpuPolicies[role];
    if (policy?.mode === "specific") {
      return {
        [`${role}_gpu_inherit`]: true,
        // Compose owns a specific installation's device reservation.  Do not
        // reapply its UUID/index through CUDA_VISIBLE_DEVICES; the disabled
        // selector is display-only in this policy.
        [`${role}_gpu_device`]: "",
      };
    }
    if (policy?.mode === "cpu") {
      return {
        [`${role}_gpu_inherit`]: false,
        [`${role}_gpu_device`]: "",
      };
    }
    const enabled = policy?.mode === "inherit"
      || (policy == null && gpuInheritAvailable === true);
    const checked = enabled && Boolean(inherit?.checked);
    return {
      [`${role}_gpu_inherit`]: checked,
      [`${role}_gpu_device`]: checked ? String(device?.value || "") : "",
    };
  };
  return {
    ...roleOptions("pilot"),
    ...roleOptions("sim"),
    viewer: Boolean(byId("use-viewer")?.checked),
  };
}

function updateGpuDeviceOptions(role) {
  const device = byId(`${role}-gpu-device`);
  if (!device) return;
  const policy = gpuPolicies[role];
  const current = String(device.value || "");
  const values = [{value: "", label: t("boot.gpu.free")}];
  const seen = new Set([""]);
  (Array.isArray(gpuDevices[role]) ? gpuDevices[role] : []).forEach((entry) => {
    const index = String(entry?.index ?? "").trim();
    if (!/^\d+$/.test(index) || seen.has(index)) return;
    seen.add(index);
    const uuid = String(entry?.uuid ?? "").trim();
    values.push({value: index, label: index, title: uuid});
  });
  if (policy?.mode === "specific") {
    const fixed = String(policy.device || "");
    if (fixed && !seen.has(fixed)) values.push({value: fixed, label: fixed, title: fixed});
  }
  device.replaceChildren(...values.map((entry) => {
    const option = document.createElement("option");
    option.value = entry.value;
    option.textContent = entry.label;
    if (entry.title) option.title = entry.title;
    return option;
  }));
  const wanted = policy?.mode === "specific"
    ? String(policy.device || "")
    : (values.some((entry) => entry.value === current) ? current : "");
  device.value = values.some((entry) => entry.value === wanted) ? wanted : "";
}

function updateRuntimeOptions() {
  const viewer = byId("use-viewer");
  const workflowReady = workflowSaved && workflowApplied;
  const optionsLocked = runtimeOptionsLocked || !workflowReady || !runtimeReady;
  const bootOptions = document.querySelector(".boot-options");
  if (bootOptions) {
    bootOptions.classList.toggle("runtime-options-locked", optionsLocked);
    bootOptions.setAttribute("aria-disabled", String(optionsLocked));
  }
  ["pilot", "sim"].forEach((role) => {
    const inherit = byId(`${role}-gpu-inherit`);
    const device = byId(`${role}-gpu-device`);
    if (!inherit || !device) return;
    const policy = gpuPolicies[role];
    updateGpuDeviceOptions(role);
    const fixed = policy && policy.mode !== "inherit";
    if (fixed) {
      inherit.checked = policy.mode === "specific";
      device.value = policy.mode === "specific" ? String(policy.device || "") : "";
    } else if (!policy && gpuInheritAvailable === false) {
      inherit.checked = false;
      device.value = "";
    }
    const available = policy?.mode === "inherit"
      || (policy == null && gpuInheritAvailable === true);
    inherit.disabled = optionsLocked || fixed || !available;
    device.disabled = optionsLocked || fixed || !available || !inherit.checked;
  });
  if (viewer) viewer.disabled = optionsLocked;
}

function applyRuntimeCapabilities(context) {
  gpuInheritAvailable = context?.runtime_options?.gpu_inherit_available === true;
  // A saved topology may place Sim on another host.  Until that host's
  // read-only runtime status is fetched, keep the role fail-closed instead of
  // reusing the local install's GPU policy for it.
  gpuPolicies = context?.topology
    ? {pilot: {mode: "unknown", device: ""}, sim: {mode: "unknown", device: ""}}
    : {pilot: null, sim: null};
  gpuDevices = {pilot: [], sim: []};
  const policies = context?.runtime_options?.gpu_policies;
  if (policies && typeof policies === "object") {
    ["pilot", "sim"].forEach((role) => {
      const policy = policies[role];
      if (policy && ["inherit", "specific", "cpu"].includes(policy.mode)) {
        gpuPolicies[role] = {
          mode: policy.mode,
          device: policy.mode === "specific" ? String(policy.device || "") : "",
        };
      }
    });
  }
  updateRuntimeOptions();
}

function applyRuntimeGpuPolicies(hosts) {
  // A status response is authoritative for the current saved topology.  A
  // missing/unreachable host must not leave a previously valid checkbox
  // enabled with stale policy data.
  const next = {
    pilot: {mode: "unknown", device: ""},
    sim: {mode: "unknown", device: ""},
  };
  const nextDevices = {pilot: [], sim: []};
  (Array.isArray(hosts) ? hosts : []).forEach((host) => {
    const policies = host?.gpu_policy && typeof host.gpu_policy === "object"
      ? host.gpu_policy
      : {};
    ["pilot", "sim"].forEach((role) => {
      if (!(host?.roles || []).includes(role)) return;
      const policy = policies[role];
      if (!policy || !["inherit", "specific", "cpu"].includes(policy.mode)) {
        return;
      }
      next[role] = {
        mode: policy.mode,
        device: policy.mode === "specific" ? String(policy.device || "") : "",
      };
    });
    ["pilot", "sim"].forEach((role) => {
      if (!(host?.roles || []).includes(role)) return;
      if (Array.isArray(host?.gpu_devices)) nextDevices[role] = host.gpu_devices;
    });
  });
  ["pilot", "sim"].forEach((role) => {
    const previous = gpuPolicies[role];
    // Unknown/CPU policy disables and clears the checkbox. Restore the
    // installed inherit default when its policy arrives, but preserve the
    // operator's choice across subsequent status polls.
    if (next[role].mode === "inherit" && previous?.mode !== "inherit") {
      const inherit = byId(`${role}-gpu-inherit`);
      if (inherit) inherit.checked = true;
    }
    gpuPolicies[role] = next[role];
    gpuDevices[role] = nextDevices[role];
  });
  updateRuntimeOptions();
}

function setRuntimeOptionsLocked(locked) {
  runtimeOptionsLocked = Boolean(locked);
  updateRuntimeOptions();
}

function restoreRuntimeOptions(job) {
  if (!job || job.action !== "start" || !job.runtime_options) return;
  const options = job.runtime_options;
  const viewer = byId("use-viewer");
  ["pilot", "sim"].forEach((role) => {
    const inherit = byId(`${role}-gpu-inherit`);
    const device = byId(`${role}-gpu-device`);
    if (!inherit || !device) return;
    const inheritValue = Object.prototype.hasOwnProperty.call(options, `${role}_gpu_inherit`)
      ? options[`${role}_gpu_inherit`]
      : options.gpu_inherit;
    const deviceValue = Object.prototype.hasOwnProperty.call(options, `${role}_gpu_device`)
      ? options[`${role}_gpu_device`]
      : options.gpu_device;
    const policy = gpuPolicies[role];
    if (policy?.mode === "specific") {
      inherit.checked = true;
      device.value = policy.device;
    } else if (policy?.mode === "cpu") {
      inherit.checked = false;
      device.value = "";
    } else {
      inherit.checked = Boolean(inheritValue);
      device.value = String(deviceValue ?? "");
    }
  });
  if (viewer) viewer.checked = Boolean(options.viewer);
  setRuntimeOptionsLocked(true);
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
    workflowStarted = false;
    workflowApplied = false;
    runtimeReady = false;
    runtimeRevision += 1;
  }
  if (invalidate) {
    setWorkflowStepState("apply", "pending");
    setWorkflowStepState("start", "pending");
  }
  updateWorkflow();
  if (!quiet) {
    const savedPath = typeof result?.saved_path === "string"
      ? result.saved_path.trim()
      : "";
    showNotice("notice.saved", savedPath ? `${t("notice.saved.path")}: ${savedPath}` : "");
  }
  return result;
}

async function probeSsh(slot) {
  if (document.querySelector('input[name="local-host"]:checked')?.value === slot) {
    throw new Error(t("error.local.probe"));
  }
  const host = sshHostFromForm(slot);
  const port = sshPort(slot);
  const hostCard = card(slot);
  const user = field(slot, "ssh-user").value.trim();
  if (!user) throw new Error(`${t("error.ssh.user.required")} (${slot})`);
  const tailscale = field(slot, "ssh-tailscale").checked;
  const result = await api("/api/ssh/fingerprint", {
    method: "POST",
    body: JSON.stringify({
      host,
      port,
      auth_mode: tailscale ? "tailscale" : "openssh"
    })
  });
  // Never attach an in-flight result to a replaced card or changed endpoint.
  if (card(slot) !== hostCard || !isActive(slot)
      || document.querySelector('input[name="local-host"]:checked')?.value === slot
      || field(slot, "ssh-tailscale").checked !== tailscale) return;
  try {
    if (sshHostFromForm(slot) !== host || sshPort(slot) !== port
        || field(slot, "ssh-user").value.trim() !== user) return;
  } catch (_) { return; } // The endpoint may have been cleared while probing.
  const prompt = `${t("ssh.trust")}\n${host}:${port}\n${result.fingerprint}`;
  if (window.confirm(prompt)) {
    field(slot, "ssh-fingerprint").value = result.fingerprint;
    showNotice("notice.fingerprint");
    updateInstallationLookup(slot);
  }
}

async function startJob(action) {
  if (jobSubmissionPending) return;
  const locksRuntimeOptions = action === "start";
  jobSubmissionPending = true;
  // Disable every workflow action before the first asynchronous status/save
  // request.  Otherwise a second click can race the first accepted job and
  // attempt to save topology after the server has entered its running state.
  updateWorkflow(true);
  if (locksRuntimeOptions) setRuntimeOptionsLocked(true);
  let submitted = false;
  let step = "";
  try {
    if (locksRuntimeOptions && (!workflowSaved || !workflowApplied || !runtimeReady)) {
      throw new Error(t("error.workflow.incomplete"));
    }
    await saveTopology({quiet: true, invalidate: false});
    if (["prepare", "provision", "deploy", "rotate"].includes(action)) {
      workflowApplied = false;
      runtimeReady = false;
      runtimeRevision += 1;
      setWorkflowStepState("start", "pending");
      updateRuntimeOptions();
    }
    step = workflowStepForAction(action);
    if (step) setWorkflowStepState(step, "running");
    const payload = action === "start"
      ? runtimeLaunchOptions()
      : {};
    await api(`/api/job/${action}`, {method: "POST", body: JSON.stringify(payload)});
    submitted = true;
    byId("job-status").dataset.status = "running";
    renderRuntimeJobStatus({status: "running", action});
    setJobRunning(true);
    if (pollTimer) window.clearInterval(pollTimer);
    pollTimer = window.setInterval(pollJob, 500);
    await pollJob();
  } catch (error) {
    if (step) setWorkflowStepState(step, "error");
    if (locksRuntimeOptions && !submitted) setRuntimeOptionsLocked(false);
    throw error;
  } finally {
    jobSubmissionPending = false;
    // Recompute from the authoritative job status after the request/poll
    // sequence.  This re-enables the button after completion or a rejected
    // submission, while keeping it disabled if the job is still running.
    updateWorkflow();
  }
}

async function runApplyJob() {
  await startJob("prepare");
}

function markWorkflowDirty() {
  runtimeReady = false;
  runtimeRevision += 1;
  workflowStarted = false;
  workflowSaved = false;
  workflowRequiresFreshSave = true;
  workflowApplied = false;
  setWorkflowStepState("save", "pending");
  setWorkflowStepState("apply", "pending");
  setWorkflowStepState("start", "pending");
  updateWorkflow();
}

function renderRuntimeStatus(result) {
  runtimeReady = false;
  if (!result?.available) {
    updateWorkflow();
    byId("runtime-status").textContent = result?.reason || t("runtime.unavailable");
    return;
  }
  const hosts = Array.isArray(result.hosts) ? result.hosts : [];
  applyRuntimeGpuPolicies(hosts);
  runtimeReady = workflowApplied && hosts.length > 0 && hosts.every((host) =>
    (host.inventory_ready ?? host.reachable) && (host.roles || []).every((role) =>
      !["pilot", "sim"].includes(role)
      || ["inherit", "specific", "cpu"].includes(host.gpu_policy?.[role]?.mode)));
  const rows = hosts.map((host) => {
    const roles = (host.roles || []).join(", ");
    const state = !host.reachable
      ? t("runtime.unreachable")
      : host.state === "unregistered"
        ? t("runtime.unregistered")
        : (host.state || "unknown");
    const policies = host.gpu_policy && typeof host.gpu_policy === "object"
      ? ["pilot", "sim"].filter((role) => host.gpu_policy[role]).map((role) => {
        const policy = host.gpu_policy[role];
        const value = policy.mode === "specific" ? `:${policy.device}` : "";
        return `${role}-gpu=${policy.mode}${value}`;
      }).join(", ")
      : "";
    const policyDetail = policies ? `; ${policies}` : "";
    const policyError = host.gpu_policy_error ? `; gpu-policy=${host.gpu_policy_error}` : "";
    const detail = host.detail
      ? ` - ${host.detail}${policyDetail}${policyError}`
      : (policyDetail || policyError) ? ` -${policyDetail}${policyError}` : "";
    return `${host.host_id}: ${state} [${roles}]${detail}`;
  });
  byId("runtime-status").textContent = rows.join("\n") || "-";
  updateWorkflow();
}

function renderRuntimeJobStatus(job) {
  runtimeRevision += 1;
  runtimeReady = false;
  const action = job.action ? ` - ${t(`action.${job.action}`)}` : "";
  byId("runtime-status").textContent = `${t(`job.${job.status}`)}${action} - ${t("runtime.updating")}`;
}

async function pollRuntimeStatus() {
  if (runtimePollInFlight || ["running", "cancelling"].includes(byId("job-status").dataset.status)) return;
  runtimePollInFlight = true;
  const revision = runtimeRevision;
  const applied = workflowApplied;
  try {
    const result = await api("/api/runtime");
    if (revision === runtimeRevision && applied === workflowApplied) renderRuntimeStatus(result);
  } catch (error) {
    if (revision !== runtimeRevision || applied !== workflowApplied) return;
    runtimeReady = false;
    byId("runtime-status").textContent = error instanceof Error ? error.message : String(error);
    updateWorkflow();
  } finally {
    runtimePollInFlight = false;
    if (revision !== runtimeRevision && !["running", "cancelling"].includes(byId("job-status").dataset.status)) {
      pollRuntimeStatus();
    }
  }
}

function setJobRunning(running) {
  ["save", "runtime-start"].forEach((id) => { byId(id).disabled = running; });
  updateWorkflow(running);
  byId("cancel").disabled = !running;
}

async function pollJob() {
  try {
    const job = await api("/api/job");
    const wasRunning = ["running", "cancelling"].includes(byId("job-status").dataset.status);
    const key = `job.${job.status}`;
    byId("job-status").dataset.status = job.status;
    byId("job-status").textContent = `${t(key)}${job.action ? ` - ${t(`action.${job.action}`)}` : ""}`;
    byId("job-log").textContent = [...job.logs, job.error].filter(Boolean).join("\n");
    const running = ["running", "cancelling"].includes(job.status);
    if (running) renderRuntimeJobStatus(job);
    else if (wasRunning) {
      runtimeRevision += 1;
      byId("runtime-status").textContent = t("runtime.refreshing");
    }
    restoreRuntimeOptions(job);
    const step = workflowStepForAction(job.action);
    const topologyAppliedByThisJob =
      job.status === "completed" &&
      ["prepare", "provision", "deploy", "rotate"].includes(job.action);
    if (step && running) setWorkflowStepState(step, "running");
    if (step && !running && job.status === "completed") setWorkflowStepState(step, "success");
    if (step && !running && ["failed", "cancelled"].includes(job.status)) setWorkflowStepState(step, "error");
    if (!running && job.action === "start") {
      workflowStarted = job.status === "completed";
    }
    if (topologyAppliedByThisJob && !workflowRequiresFreshSave) {
      workflowSaved = true;
      workflowApplied = true;
      setWorkflowStepState("save", "success");
      setWorkflowStepState("apply", "success");
    }
    if (!running && job.topology_updated) {
      const context = await api("/api/context");
      applyRuntimeCapabilities(context);
      if (context.topology) applyTopology(context.topology);
      // A sidecar-discovered address is factual input, not an implicit save.
      // Keep the form populated but require the operator to validate/save it
      // before another security or runtime action can be enabled.
      workflowSaved = !workflowRequiresFreshSave;
      setWorkflowStepState("save", workflowRequiresFreshSave ? "pending" : "success");
      if (!topologyAppliedByThisJob) {
        workflowApplied = false;
        setWorkflowStepState("apply", "pending");
        setWorkflowStepState("start", "pending");
      }
    }
    if (job.status === "cancelled") {
      // Wait for backend cancellation/rollback to finish before reopening
      // the workflow. Preserve the form and diagnostic log for the retry.
      runtimeOptionsLocked = false;
      markWorkflowDirty();
    }
    setJobRunning(running);
    updateWorkflow(running);
    if (job.status === "cancelled") byId("save").focus();
    if (
      !running
      && (wasRunning || ["check", "prepare", "provision", "deploy", "rotate", "start"].includes(job.action))
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

function bindDropZone(zone) {
  if (zone.dataset.dropUnit === "robot") return;
  zone.addEventListener("dragover", (event) => {
    const target = zone.dataset.dropSlot;
    const cardId = event.dataTransfer?.getData("text/plain")
      || document.querySelector(".role-block.dragging")?.dataset.cardId
      || "";
    const roleCard = roleCardById(cardId);
    const placement = dropPlacement(zone, event.clientX, event.clientY, cardId);
    const allowed = Boolean(roleCard)
      && roleCard.role !== "robot"
      && isActive(target)
      && canPlaceRole(roleCard.role, target, {notify: false});
    const previewPlacement = allowed ? placement : null;
    updateDropPreview(zone, previewPlacement, cardId);
    if (placement && allowed) {
      event.preventDefault();
      event.dataTransfer.dropEffect = "move";
    }
  });
  zone.addEventListener("drop", (event) => {
    const cardId = event.dataTransfer.getData("text/plain")
      || document.querySelector(".role-block.dragging")?.dataset.cardId
      || "";
    const placement = dropPlacement(zone, event.clientX, event.clientY, cardId);
    if (!placement) {
      clearDropPreview();
      return;
    }
    event.preventDefault();
    clearDropPreview();
    moveRole(
      cardId,
      zone.dataset.dropSlot,
      zone.dataset.dropUnit || "runtime",
      placement.targetCardId,
      placement.insertBefore,
    );
  });
}

function installationQuery(slot, kind = "runtime") {
  const local = card(slot).querySelector('input[name="local-host"]').checked;
  return {local,
    install_root: installationField(slot, kind, "root").value.trim(),
    bin_dir: installationField(slot, kind, "bin").value.trim(),
    ssh: local ? null : sshEndpointFromForm(slot)};
}

async function lookupInstallation(slot, kind = "runtime") {
  const hostCard = card(slot);
  const button = kind === "robot"
    ? hostCard.querySelector(".robot-lookup-installation")
    : hostCard.querySelector(".lookup-installation");
  if (kind === "runtime") {
    if (!installationLookupReady(slot)) return;
  } else if (!installationLookupReadyFor(slot, kind)) return;
  const body = JSON.stringify(installationQuery(slot, kind));
  button.dataset.lookupBusy = "true";
  const previousInstall = installationField(slot, kind, "uuid").value.trim();
  updateInstallationLookupFor(slot, kind);
  try {
    const result = await api("/api/installations", {method: "POST", body});
    if (!hostCard.isConnected || body !== JSON.stringify(installationQuery(slot, kind))) return;
    let installations = Array.isArray(result.installations)
      ? result.installations.filter((item) => item && typeof item === "object")
      : [];
    installations = installations.filter((item) => kind === "robot"
      ? item.install_mode === "native"
      : item.install_mode !== "native");
    setInstallationCatalog(slot, kind, installations);
    const select = installationField(slot, kind, "name");
    const options = installations.length
      ? installations.map((item) => new Option(installationOptionLabel(item), item.install_uuid))
      : [new Option(t("install.queryFirst"), "")];
    select.replaceChildren(...options);
    if (installations.some((item) => item.install_uuid === previousInstall)) {
      select.value = previousInstall;
    }
    const choose = () => {
      const item = installations.find((entry) => entry.install_uuid === select.value);
      installationField(slot, kind, "uuid").value = item?.install_uuid || "";
      installationField(slot, kind, "project").value = item?.project || "";
      if (kind === "runtime") refreshRoleReleaseOptions(slot);
      markWorkflowDirty();
    };
    select.onchange = choose;
    choose();
  } finally {
    delete button.dataset.lookupBusy;
    updateInstallationLookupFor(slot, kind);
  }
}

function clearInstallationLookup(slot, kind = "runtime") {
  setInstallationCatalog(slot, kind, null);
  installationField(slot, kind, "uuid").value = "";
  installationField(slot, kind, "project").value = "";
  const select = installationField(slot, kind, "name");
  select.onchange = null;
  select.replaceChildren(new Option(t("install.queryFirst"), ""));
  if (kind === "runtime") {
    roleCards
      .filter((roleCard) => roleCard.slot === slot)
      .forEach((roleCard) => { roleCard.releaseKey = ""; });
    refreshRoleReleaseOptions(slot);
  }
  updateInstallationLookupFor(slot, kind);
}

function clearAllInstallationLookups(slot) {
  clearInstallationLookup(slot, "runtime");
  clearInstallationLookup(slot, "robot");
}

function bindHostCardEvents(slot) {
  const hostCard = card(slot);
  hostCard.querySelector(".lookup-installation").addEventListener("click", () => lookupInstallation(slot).catch(showError));
  hostCard.querySelector(".robot-lookup-installation").addEventListener("click", () => lookupInstallation(slot, "robot").catch(showError));
  const clearRuntimeInstallation = () => clearInstallationLookup(slot, "runtime");
  const clearRobotInstallation = () => clearInstallationLookup(slot, "robot");
  ["install-root", "bin-dir"].forEach((name) => {
    field(slot, name).addEventListener("input", clearRuntimeInstallation);
    field(slot, name).addEventListener("change", clearRuntimeInstallation);
  });
  ["robot-install-root", "robot-bin-dir"].forEach((name) => {
    field(slot, name).addEventListener("input", clearRobotInstallation);
    field(slot, name).addEventListener("change", clearRobotInstallation);
  });
  ["ssh-host", "ssh-port", "ssh-user", "ssh-key", "ssh-fingerprint", "ssh-tailscale"].forEach((name) => {
    field(slot, name).addEventListener("input", () => clearAllInstallationLookups(slot));
    field(slot, name).addEventListener("change", () => clearAllInstallationLookups(slot));
  });
  hostCard.querySelectorAll(".drop-zone").forEach(bindDropZone);
  hostCard.querySelector('input[name="local-host"]').addEventListener("change", updateSshVisibility);
  hostCard.querySelector('input[name="local-host"]').addEventListener("change", () => {
    slots.forEach((other) => clearAllInstallationLookups(other));
  });
  ["ssh-host", "ssh-port"].forEach((name) => {
    field(slot, name).addEventListener("input", () => {
      field(slot, "ssh-fingerprint").value = "";
      updateInstallationLookup(slot);
    });
  });
  field(slot, "ssh-tailscale").addEventListener("change", () => {
    updateSshMode(slot);
    field(slot, "ssh-fingerprint").value = "";
    updateInstallationLookup(slot);
  });
  hostCard.querySelectorAll("input, select").forEach((control) => {
    control.addEventListener("input", markWorkflowDirty);
    control.addEventListener("change", markWorkflowDirty);
  });
  hostCard.querySelector(".probe").addEventListener("click", () => probeSsh(slot).catch(showError));
  const hostName = hostCard.querySelector(".host-name");
  hostCard.querySelector(".rename-host").addEventListener("click", () => beginHostRename(slot));
  hostName.addEventListener("blur", () => finishHostRename(slot));
  hostName.addEventListener("keydown", (event) => {
    if (event.key === "Enter") hostName.blur();
    if (event.key === "Escape") {
      hostName.value = hostName.dataset.previous || slot;
      hostName.readOnly = true;
      hostName.blur();
    }
  });
  hostCard.querySelector(".move-host-up").addEventListener("click", () => moveHost(slot, -1));
  hostCard.querySelector(".move-host-down").addEventListener("click", () => moveHost(slot, 1));
  hostCard.querySelector(".remove-host").addEventListener("click", () => removeHost(slot));
  hostCard.querySelector(".add-role").addEventListener("click", () => {
    pendingRoleSlot = slot;
    updateRoleChoices();
    if (byId("new-role-kind").options.length) byId("add-role-dialog").showModal();
  });
}

function updateRoleChoices() {
  const select = byId("new-role-kind");
  if (!select) return;
  const allowed = applicationRoles;
  const available = allowed.filter(
    (role) => !pendingRoleSlot || canPlaceRole(role, pendingRoleSlot, {notify: false}),
  );
  select.replaceChildren(...available.map((role) => {
    const option = document.createElement("option");
    option.value = role;
    option.textContent = t(`role.${role}`);
    return option;
  }));
  document.querySelectorAll(".add-role").forEach((button) => {
    const slot = button.closest(".host-card")?.dataset.slot || "";
    button.disabled = !allowed.some(
      (role) => canPlaceRole(role, slot, {notify: false}),
    );
  });
}

function addSelectedRole() {
  const role = byId("new-role-kind").value;
  if (!applicationRoles.includes(role)) return;
  const target = pendingRoleSlot;
  if (!target) {
    showError(t(role === "robot" ? "error.robot.jetson" : "error.role.destination"));
    return;
  }
  if (!canPlaceRole(role, target)) return;
  appendRoleCard(role, target);
  markWorkflowDirty();
  renderRoleBlocks();
  updateRoleChoices();
}

function bindEvents() {
  document.addEventListener("dragover", (event) => {
    if (!document.querySelector(".role-block.dragging")) return;
    updateDragAutoScroll(event.clientY);
    if (!event.target.closest?.(".drop-zone")) clearDropPreview();
  });
  document.addEventListener("drop", stopDragAutoScroll);
  document.addEventListener("dragend", stopDragAutoScroll);
  document.querySelectorAll("[data-language]").forEach((button) => {
    button.addEventListener("click", () => applyLanguage(button.dataset.language));
  });
  document.querySelectorAll("[data-banner-close]").forEach((button) => {
    button.addEventListener("click", () => {
      const banner = button.closest(".banner");
      if (banner) setBannerVisible(banner, false);
    });
  });
  document.querySelectorAll("input, select").forEach((control) => {
    if (control.closest("template") || control.closest("dialog")) return;
    if (control.closest(".boot-options")) return;
    control.addEventListener("input", markWorkflowDirty);
    control.addEventListener("change", markWorkflowDirty);
  });
  ["pilot", "sim"].forEach((role) => {
    byId(`${role}-gpu-inherit`)?.addEventListener("change", updateRuntimeOptions);
    byId(`${role}-gpu-device`)?.addEventListener("change", updateRuntimeOptions);
  });
  byId("add-host").addEventListener("click", () => {
    const robotChoice = document.querySelector('input[name="new-host-kind"][value="robot"]');
    robotChoice.disabled = robotSlots().length > 0;
    if (robotChoice.disabled && robotChoice.checked) {
      document.querySelector('input[name="new-host-kind"][value="computer"]').checked = true;
    }
    byId("add-host-dialog").showModal();
  });
  byId("confirm-add-host").addEventListener("click", (event) => {
    event.preventDefault();
    const robot = document.querySelector('input[name="new-host-kind"]:checked')?.value === "robot";
    if (activeSlots().length >= maximumHosts) {
      showError(t("error.host.maximum"));
      return;
    }
    const slot = createHost({robot});
    if (slot && robot) appendRoleCard("robot", slot);
    if (slot) {
      byId("add-host-dialog").close();
      applyLanguage(language);
      markWorkflowDirty();
    }
  });
  byId("confirm-add-role").addEventListener("click", (event) => {
    event.preventDefault();
    addSelectedRole();
    byId("add-role-dialog").close();
  });
  byId("security").addEventListener("change", updateWorkflow);
  byId("save").addEventListener("click", async () => {
    if (jobSubmissionPending || !pageReady) return;
    jobSubmissionPending = true;
    updateWorkflow();
    try {
      await saveTopology();
    } catch (error) {
      showError(error);
    } finally {
      jobSubmissionPending = false;
      updateWorkflow();
    }
  });
  byId("apply").addEventListener("click", () => runApplyJob().catch(showError));
  byId("runtime-start").addEventListener("click", () => startJob("start").catch(showError));
  byId("cancel").addEventListener("click", async () => {
    try { await api("/api/cancel", {method: "POST", body: JSON.stringify({})}); }
    catch (error) { showError(error); }
  });
}

async function initialize() {
  try {
    catalog = await fetch("/i18n.json", {cache: "no-store"}).then((response) => response.json());
    bindEvents();
    updateRuntimeOptions();
    applyLanguage("ko");
    const context = await api("/api/context");
    applyRuntimeCapabilities(context);
    schemaVersion = context.schema_version;
    if (context.topology) {
      // Restore all values, but deliberately begin the operator workflow at
      // validation/save.  A local active generation can outlive a failed or
      // partial remote rollout, so it is not sufficient to unlock Booting.
      workflowSaved = false;
      workflowApplied = false;
      workflowStarted = false;
      workflowRequiresFreshSave = true;
      setWorkflowStepState("save", "pending");
      setWorkflowStepState("apply", "pending");
      setWorkflowStepState("start", "pending");
      applyTopology(context.topology);
    } else {
      const first = createHost({operational: true});
      roleCards = [];
      nextCardId = 1;
      nextRoleNumbers = {pilot: 1, ui: 1, sim: 1, robot: 1};
      appendRoleCard("pilot", first);
      appendRoleCard("ui", first);
      if (context.local_defaults) {
        if (context.local_defaults.system_id != null) {
          byId("system-id").value = context.local_defaults.system_id;
        }
        if (context.local_defaults.install_root) {
          field(first, "install-root").value = context.local_defaults.install_root;
        }
        if (context.local_defaults.bin_dir) {
          field(first, "bin-dir").value = context.local_defaults.bin_dir;
        }
      }
    }
    applyLanguage(language);
    applyLocalTailscaleHint(context);
    updateSshVisibility();
    updateWorkflow();
    renderRoleBlocks();
    updateRoleChoices();
    updateHostLimit();
    pageReady = true;
    updateWorkflow();
    await pollJob();
    await pollRuntimeStatus();
    if (runtimePollTimer) window.clearInterval(runtimePollTimer);
    runtimePollTimer = window.setInterval(pollRuntimeStatus, 10000);
  } catch (error) {
    pageReady = false;
    showError(error);
  }
}

initialize();
