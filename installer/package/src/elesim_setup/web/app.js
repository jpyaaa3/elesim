"use strict";

const query = new URLSearchParams(window.location.search);
if (query.get("token")) {
  sessionStorage.setItem("elesimSetupToken", query.get("token"));
  history.replaceState({}, "", window.location.pathname);
}

const token = sessionStorage.getItem("elesimSetupToken") || "";
const steps = ["mode", "roles", "paths", "compute", "network", "review", "install"];
const roleOrder = ["sim", "pilot", "ui", "robot"];
// The initial selection is only a convenience; every role remains an explicit
// checkbox and can be changed independently by the operator.
const defaultGeneralRoles = ["sim", "pilot", "ui"];

let catalog = {};
let language = "ko";
let context = null;
let currentStep = 0;
let pollTimer = null;
let acceptedFingerprint = "";
let browseTarget = "";
let browseMode = "directory";
let selectedFile = "";

const byId = (id) => document.getElementById(id);
const checkedValue = (name) => document.querySelector(`input[name="${name}"]:checked`)?.value || "";

function t(key) {
  return catalog[language]?.[key] || key;
}

async function api(path, options = {}) {
  const headers = new Headers(options.headers || {});
  headers.set("X-Elesim-Token", token);
  if (options.body && !headers.has("Content-Type")) {
    headers.set("Content-Type", "application/json");
  }
  const response = await fetch(path, {...options, headers});
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(payload.error || `${response.status} ${response.statusText}`);
  }
  return payload;
}

function setError(error) {
  const banner = byId("error-banner");
  if (!error) {
    banner.hidden = true;
    banner.textContent = "";
    return;
  }
  banner.textContent = error instanceof Error ? error.message : String(error);
  banner.hidden = false;
  window.setTimeout(() => {
    if (banner.textContent) banner.hidden = true;
  }, 9000);
}

function applyLanguage(nextLanguage) {
  language = nextLanguage;
  document.documentElement.lang = language;
  document.querySelectorAll("[data-i18n]").forEach((element) => {
    element.textContent = t(element.dataset.i18n);
  });
  document.querySelectorAll("[data-language]").forEach((button) => {
    button.classList.toggle("active", button.dataset.language === language);
  });
  renderRoles();
  updateStep();
}

function selectedRoles() {
  if (checkedValue("edition") === "developer") return [];
  return roleOrder.filter((role) => byId(`role-${role}`)?.checked);
}

function renderRoles() {
  const selected = new Set(
    roleOrder.filter((role) => byId(`role-${role}`)?.checked)
  );
  if (!selected.size && !byId("role-options").children.length) {
    defaultGeneralRoles.forEach((role) => selected.add(role));
  }
  const container = byId("role-options");
  container.replaceChildren();
  roleOrder.forEach((role) => {
    const unavailable = role === "robot" && !context.capabilities.robot_installable;
    const label = document.createElement("label");
    label.className = `role-option${unavailable ? " disabled" : ""}`;
    const input = document.createElement("input");
    input.type = "checkbox";
    input.id = `role-${role}`;
    input.value = role;
    input.checked = selected.has(role) && !unavailable;
    input.disabled = unavailable;
    input.addEventListener("change", () => {
      if (role === "robot" && input.checked) {
        roleOrder.filter((item) => item !== "robot").forEach((item) => {
          byId(`role-${item}`).checked = false;
        });
      } else if (input.checked && byId("role-robot")) {
        byId("role-robot").checked = false;
      }
      updateConditionalControls();
    });
    const text = document.createElement("span");
    const title = document.createElement("strong");
    title.textContent = t(`role.${role}`);
    const help = document.createElement("small");
    help.textContent = unavailable ? t("role.robot.unavailable") : t(`role.${role}.help`);
    text.append(title, help);
    label.append(input, text);
    container.append(label);
  });
}

function updateMode() {
  const developer = checkedValue("edition") === "developer";
  byId("privileged-confirm-row").hidden = !developer;
  byId("jaeger-row").hidden = !developer;
  byId("runtime-text-logs-row").hidden = developer;
  byId("runtime-text-logs").disabled = developer;
  byId("general-roles").hidden = developer;
  byId("developer-roles").hidden = !developer;
  if (developer) {
    byId("dds-system-id").value = context.defaults.dds_system_id;
    byId("dds-domain-id").value = context.defaults.dds_domain_id;
    byId("dds-rmw").value = context.defaults.dds_rmw_implementation;
    document.querySelector('input[name="dds-discovery-mode"][value="multicast"]').checked = true;
    document.querySelector(
      `input[name="dds-security-profile"][value="${context.defaults.dds_security_profile}"]`
    ).checked = true;
    byId("dds-static-peers").value = "";
    byId("dds-interface").value = "";
    byId("dds-keystore").value = "";
    byId("dds-enclave").value = "";
    document.querySelector('input[name="dds-security-provisioning"][value="external"]').checked = true;
    document.querySelector('input[name="turn-mode"][value="none"]').checked = true;
    acceptedFingerprint = "";
  }
  byId("prefix-help").textContent = t("paths.prefix.help");
  updateConditionalControls();
}

function hasSim() {
  return checkedValue("edition") === "developer" || selectedRoles().includes("sim");
}

function updateConditionalControls() {
  const gpuMode = checkedValue("gpu-mode");
  byId("gpu-device-row").hidden = gpuMode !== "specific";

  const staticDiscovery = checkedValue("dds-discovery-mode") === "static";
  byId("dds-static-fields").hidden = !staticDiscovery;
  const turnMode = checkedValue("turn-mode");
  if (turnMode === "managed") {
    document.querySelector(
      'input[name="dds-security-profile"][value="sros2"]'
    ).checked = true;
    document.querySelector(
      'input[name="dds-security-provisioning"][value="managed"]'
    ).checked = true;
  }
  const sros2 = checkedValue("dds-security-profile") === "sros2";
  byId("sros2-fields").hidden = !sros2;
  const developer = checkedValue("edition") === "developer";
  const managedProvisioning = document.querySelector(
    'input[name="dds-security-provisioning"][value="managed"]'
  );
  managedProvisioning.disabled = developer;
  if (developer && managedProvisioning.checked) {
    document.querySelector(
      'input[name="dds-security-provisioning"][value="external"]'
    ).checked = true;
  }
  const provisioning = checkedValue("dds-security-provisioning") || "managed";
  byId("sros2-external-fields").hidden = !sros2 || provisioning !== "external";

  const turnManaged = document.querySelector('input[name="turn-mode"][value="managed"]');
  const sim = hasSim();
  turnManaged.disabled = !sim;
  byId("turn-managed-option").hidden = !sim;
  byId("turn-section").hidden = checkedValue("edition") === "developer";
  if (!sim && checkedValue("turn-mode") === "managed") {
    document.querySelector('input[name="turn-mode"][value="none"]').checked = true;
  }
  byId("turn-fields").hidden = turnMode === "none";
  byId("turn-realm-row").hidden = turnMode !== "managed";
  byId("turn-public-row").hidden = turnMode !== "managed";
  byId("turn-secret-row").hidden = turnMode !== "managed";
  byId("turn-credential-row").hidden = turnMode !== "external" || !sim;
  if (turnMode === "managed") {
    if (!byId("turn-realm").value) byId("turn-realm").value = "elesim.local";
    if (!byId("turn-secret-file").value) {
      byId("turn-secret-file").value = `${byId("prefix").value.trim()}/secrets/turn.secret`;
    }
    if (!byId("turn-url").value) {
      byId("turn-url").value = `turn:${byId("turn-public-host").value}:3478?transport=udp`;
    }
  }
}

function payload() {
  const edition = checkedValue("edition") || "general";
  const securityProfile = checkedValue("dds-security-profile") || "trusted-network";
  const securityProvisioning = securityProfile === "sros2"
    ? (checkedValue("dds-security-provisioning") || "managed")
    : "none";
  const turnMode = edition === "general" ? (checkedValue("turn-mode") || "none") : "none";
  return {
    language,
    edition,
    roles: edition === "general" ? selectedRoles() : [],
    prefix: byId("prefix").value.trim(),
    bin_dir: byId("bin-dir").value.trim(),
    source_root: "",
    gpu_mode: checkedValue("gpu-mode") || "inherit",
    gpu_device: checkedValue("gpu-mode") === "specific" ? byId("gpu-device").value : "",
    dds_system_id: byId("dds-system-id").value.trim(),
    dds_domain_id: Number(byId("dds-domain-id").value),
    dds_rmw_implementation: byId("dds-rmw").value,
    dds_discovery_mode: checkedValue("dds-discovery-mode") || "multicast",
    dds_static_peers: checkedValue("dds-discovery-mode") === "static"
      ? byId("dds-static-peers").value.trim()
      : "",
    dds_interface: byId("dds-interface").value.trim(),
    dds_security_profile: securityProfile,
    dds_security_provisioning: securityProvisioning,
    dds_keystore: securityProfile === "sros2" && securityProvisioning === "external"
      ? byId("dds-keystore").value.trim() : "",
    dds_enclave: securityProfile === "sros2" && securityProvisioning === "external"
      ? byId("dds-enclave").value.trim() : "",
    ssh: {
      host: byId("ssh-host").value.trim(),
      port: Number(byId("ssh-port").value),
      user: byId("ssh-user").value.trim(),
      remote_root: byId("ssh-remote-root").value.trim(),
      identity_file: byId("ssh-key").value.trim(),
      accepted_fingerprint: acceptedFingerprint
    },
    turn_mode: turnMode,
    turn_url: turnMode === "none" ? "" : byId("turn-url").value.trim(),
    turn_realm: turnMode === "managed" ? byId("turn-realm").value.trim() : "",
    turn_public_host: turnMode === "managed" ? byId("turn-public-host").value.trim() : "",
    turn_secret_file: turnMode === "managed" ? byId("turn-secret-file").value.trim() : "",
    turn_credential_file: turnMode === "external" && selectedRoles().includes("sim")
      ? byId("turn-credential-file").value.trim()
      : "",
    register_path: byId("register-path").checked,
    runtime_text_logs: {
      enabled: edition === "general" && byId("runtime-text-logs").checked
    },
    jaeger: edition === "developer" && byId("jaeger").checked,
    repository: context.repository,
    ref: context.ref
  };
}

function validateCurrentStep() {
  const step = steps[currentStep];
  if (step === "mode" && checkedValue("edition") === "developer" && !byId("privileged-confirm").checked) {
    throw new Error(t("error.privileged"));
  }
  if (step === "roles" && checkedValue("edition") === "general" && !selectedRoles().length) {
    throw new Error(t("error.roles"));
  }
  if (step === "paths" && (!byId("prefix").value.trim() || !byId("bin-dir").value.trim())) {
    throw new Error(t("error.generic"));
  }
}

async function prepareReview() {
  const summary = await api("/api/validate", {
    method: "POST",
    body: JSON.stringify(payload())
  });
  const rows = [
    ["review.edition", summary.edition],
    ["review.roles", summary.roles.length ? summary.roles.join(", ") : "all development packages"],
    ["review.prefix", summary.prefix],
    ["review.bin", summary.bin_dir],
    ["review.gpu", summary.gpu_mode],
    ["review.security", summary.security_profile === "sros2"
      ? `${summary.security_profile} (${summary.security_provisioning})`
      : summary.security_profile],
    ["review.turn", summary.turn_mode],
    ["review.path", summary.register_path ? t("value.yes") : t("value.no")],
    ["review.logs", summary.runtime_text_logs ? t("value.yes") : t("value.no")],
    ["review.jaeger", summary.jaeger ? t("value.yes") : t("value.no")]
  ];
  const list = byId("review-list");
  list.replaceChildren();
  rows.forEach(([key, value]) => {
    const term = document.createElement("dt");
    term.textContent = t(key);
    const description = document.createElement("dd");
    description.textContent = value;
    list.append(term, description);
  });
  const warning = byId("review-warning");
  warning.hidden = summary.security_profile !== "trusted-network";
  warning.textContent = t("dds.security.trusted.help");
}

function updateStep() {
  document.querySelectorAll("[data-step]").forEach((page) => {
    page.classList.toggle("active", page.dataset.step === steps[currentStep]);
  });
  document.querySelectorAll("[data-step-link]").forEach((item, index) => {
    item.classList.toggle("active", index === currentStep);
    item.classList.toggle("completed", index < currentStep);
  });
  byId("back-button").disabled = currentStep === 0 || steps[currentStep] === "install";
  byId("next-button").hidden = steps[currentStep] === "install";
  byId("next-button").textContent = steps[currentStep] === "review" ? t("action.install") : t("action.next");
  byId("step-position").textContent = `${currentStep + 1} / ${steps.length}`;
}

async function nextStep() {
  try {
    setError("");
    validateCurrentStep();
    if (
      checkedValue("edition") === "developer"
      && steps[currentStep] === "compute"
    ) {
      await prepareReview();
      currentStep = steps.indexOf("review");
      updateStep();
      return;
    }
    if (steps[currentStep] === "network") {
      await prepareReview();
    }
    if (steps[currentStep] === "review") {
      currentStep = steps.indexOf("install");
      updateStep();
      await startInstall();
      return;
    }
    currentStep = Math.min(currentStep + 1, steps.length - 1);
    updateStep();
  } catch (error) {
    setError(error);
  }
}

function previousStep() {
  if (currentStep > 0) {
    if (
      checkedValue("edition") === "developer"
      && steps[currentStep] === "review"
    ) {
      currentStep = steps.indexOf("compute");
    } else {
      currentStep -= 1;
    }
    updateStep();
  }
}

async function startInstall() {
  byId("install-status").textContent = t("install.running");
  byId("cancel-install").disabled = false;
  byId("close-installer").disabled = true;
  byId("completion").hidden = true;
  byId("install-log").textContent = "";
  try {
    await api("/api/install", {method: "POST", body: JSON.stringify(payload())});
    pollTimer = window.setInterval(pollJob, 500);
    await pollJob();
  } catch (error) {
    setError(error);
    byId("cancel-install").disabled = true;
    byId("close-installer").disabled = false;
    byId("install-status").textContent = t("install.failed");
  }
}

async function pollJob() {
  try {
    const job = await api("/api/job");
    const log = byId("install-log");
    const nearBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 48;
    log.textContent = job.logs.join("\n");
    if (nearBottom) log.scrollTop = log.scrollHeight;
    if (job.status === "completed") {
      window.clearInterval(pollTimer);
      byId("cancel-install").disabled = true;
      byId("close-installer").disabled = false;
      byId("install-status").textContent = t("install.completed");
      byId("completion").hidden = false;
      const pendingManaged = checkedValue("edition") === "general"
        && checkedValue("dds-security-profile") === "sros2"
        && checkedValue("dds-security-provisioning") === "managed";
      byId("start-command").textContent = `${byId("bin-dir").value.trim()}/${
        pendingManaged ? "elesim-connections" : "elesim-up"
      }`;
      byId("source-command-row").hidden = !byId("register-path").checked;
    } else if (job.status === "failed") {
      window.clearInterval(pollTimer);
      byId("cancel-install").disabled = true;
      byId("close-installer").disabled = false;
      byId("install-status").textContent = t("install.failed");
      setError(job.error || t("error.generic"));
    } else if (job.status === "cancelling") {
      byId("cancel-install").disabled = true;
      byId("install-status").textContent = t("install.cancelling");
    } else if (job.status === "cancelled") {
      window.clearInterval(pollTimer);
      byId("cancel-install").disabled = true;
      byId("close-installer").disabled = false;
      byId("install-status").textContent = t("install.cancelled");
    }
  } catch (error) {
    window.clearInterval(pollTimer);
    byId("close-installer").disabled = false;
    setError(error);
  }
}

async function copyText(text) {
  await navigator.clipboard.writeText(text);
}

async function openBrowser(target) {
  browseTarget = target;
  browseMode = ["ssh-key", "turn-credential-file"].includes(target)
    ? "file"
    : "directory";
  selectedFile = "";
  const initial = byId(target).value.trim() || context.defaults.prefix;
  await loadDirectory(initial, true);
  byId("directory-dialog").showModal();
}

async function loadDirectory(path, allowParentFallback = false) {
  try {
    const suffix = browseMode === "file" ? "&files=1" : "";
    const listing = await api(`/api/directories?path=${encodeURIComponent(path)}${suffix}`);
    byId("browse-path").value = listing.path;
    byId("browse-parent").dataset.path = listing.parent;
    byId("browse-parent").disabled = !listing.parent;
    const list = byId("directory-list");
    list.replaceChildren();
    listing.directories.forEach((entry) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "directory-item";
      button.textContent = `${entry.kind === "file" ? "·" : "▸"} ${entry.name}`;
      button.addEventListener("click", () => {
        if (entry.kind === "file") {
          selectedFile = entry.path;
          list.querySelectorAll("button").forEach((item) => item.classList.remove("selected"));
          button.classList.add("selected");
        } else {
          loadDirectory(entry.path);
        }
      });
      list.append(button);
    });
  } catch (error) {
    if (allowParentFallback) {
      const slash = path.lastIndexOf("/");
      if (slash > 0) {
        await loadDirectory(path.slice(0, slash));
        return;
      }
    }
    setError(error);
  }
}

async function probeSsh() {
  acceptedFingerprint = "";
  try {
    const response = await api("/api/ssh/fingerprint", {
      method: "POST",
      body: JSON.stringify({
        host: byId("ssh-host").value.trim(),
        port: Number(byId("ssh-port").value)
      })
    });
    const prompt = language === "ko"
      ? `다음 SSH 호스트 fingerprint를 신뢰합니까?\n${response.fingerprint}`
      : `Trust this SSH host fingerprint?\n${response.fingerprint}`;
    if (window.confirm(prompt)) {
      acceptedFingerprint = response.fingerprint;
      byId("ssh-fingerprint").textContent = response.fingerprint;
    }
  } catch (error) {
    setError(error);
  }
}

function openUninstallGuide() {
  const prefix = byId("prefix").value.trim() || context.defaults.prefix;
  byId("uninstall-prefix").value = prefix;
  byId("uninstall-confirm-prefix").value = "";
  byId("uninstall-purge-logs").checked = false;
  byId("uninstall-purge-authority").checked = false;
  byId("uninstall-commands").hidden = true;
  byId("uninstall-dialog").showModal();
}

async function buildUninstallGuide() {
  try {
    setError("");
    const guide = await api("/api/uninstall/guide", {
      method: "POST",
      body: JSON.stringify({
        prefix: byId("uninstall-prefix").value.trim(),
        confirm_prefix: byId("uninstall-confirm-prefix").value,
        purge_logs: byId("uninstall-purge-logs").checked,
        purge_authority: byId("uninstall-purge-authority").checked
      })
    });
    byId("uninstall-plan-command").textContent = guide.plan_command;
    byId("uninstall-execute-command").textContent = guide.execute_command;
    byId("uninstall-commands").hidden = false;
  } catch (error) {
    setError(error);
  }
}

function initializeEvents() {
  document.querySelectorAll("[data-language]").forEach((button) => {
    button.addEventListener("click", () => applyLanguage(button.dataset.language));
  });
  document.querySelectorAll('input[name="edition"]').forEach((input) => input.addEventListener("change", updateMode));
  document.querySelectorAll('input[name="gpu-mode"], input[name="dds-discovery-mode"], input[name="dds-security-profile"], input[name="dds-security-provisioning"], input[name="turn-mode"]')
    .forEach((input) => input.addEventListener("change", updateConditionalControls));
  document.querySelectorAll("[data-browse]").forEach((button) => {
    button.addEventListener("click", () => openBrowser(button.dataset.browse));
  });
  byId("next-button").addEventListener("click", nextStep);
  byId("back-button").addEventListener("click", previousStep);
  byId("copy-log").addEventListener("click", () => copyText(byId("install-log").textContent));
  byId("cancel-install").addEventListener("click", async () => {
    try {
      byId("cancel-install").disabled = true;
      await api("/api/cancel", {method: "POST", body: "{}"});
      byId("install-status").textContent = t("install.cancelling");
    } catch (error) {
      byId("cancel-install").disabled = false;
      setError(error);
    }
  });
  document.querySelectorAll(".copy-command").forEach((button) => {
    button.addEventListener("click", () => copyText(byId(button.dataset.copyTarget).textContent));
  });
  byId("ssh-probe").addEventListener("click", probeSsh);
  byId("open-uninstall").addEventListener("click", openUninstallGuide);
  byId("uninstall-close").addEventListener("click", () => byId("uninstall-dialog").close());
  byId("build-uninstall-guide").addEventListener("click", buildUninstallGuide);
  byId("turn-public-host").addEventListener("input", () => {
    if (checkedValue("turn-mode") === "managed") {
      byId("turn-url").value = `turn:${byId("turn-public-host").value}:3478?transport=udp`;
    }
  });
  byId("browse-close").addEventListener("click", () => byId("directory-dialog").close());
  byId("browse-cancel").addEventListener("click", () => byId("directory-dialog").close());
  byId("browse-parent").addEventListener("click", () => loadDirectory(byId("browse-parent").dataset.path));
  byId("browse-go").addEventListener("click", () => loadDirectory(byId("browse-path").value));
  byId("browse-select").addEventListener("click", () => {
    byId(browseTarget).value = browseMode === "file" && selectedFile
      ? selectedFile
      : byId("browse-path").value;
    byId("directory-dialog").close();
  });
  byId("close-installer").addEventListener("click", async () => {
    await api("/api/shutdown", {method: "POST", body: "{}"});
    window.close();
  });
}

async function initialize() {
  try {
    [catalog, context] = await Promise.all([
      fetch("/i18n.json").then((response) => response.json()),
      api("/api/context")
    ]);
    language = navigator.language.toLowerCase().startsWith("ko") ? "ko" : "en";
    byId("prefix").value = context.defaults.prefix;
    byId("bin-dir").value = context.defaults.bin_dir;
    byId("dds-system-id").value = context.defaults.dds_system_id;
    byId("dds-domain-id").value = context.defaults.dds_domain_id;
    byId("dds-rmw").value = context.defaults.dds_rmw_implementation;
    byId("dds-interface").value = context.defaults.dds_interface;
    byId("dds-static-peers").value = context.defaults.dds_static_peers;
    const securityProfile = document.querySelector(
      `input[name="dds-security-profile"][value="${context.defaults.dds_security_profile}"]`
    );
    if (securityProfile) securityProfile.checked = true;
    document.querySelector(
      `input[name="dds-security-provisioning"][value="${context.defaults.dds_security_provisioning}"]`
    ).checked = true;
    byId("dds-keystore").value = context.defaults.dds_keystore;
    byId("dds-enclave").value = context.defaults.dds_enclave;
    byId("ssh-remote-root").value = context.defaults.prefix;
    byId("host-summary").textContent =
      `${context.capabilities.os_id || "Linux"} ${context.capabilities.os_version || ""} · ${context.capabilities.architecture}`;
    const gpu = byId("gpu-device");
    context.capabilities.gpu_devices.forEach((device) => {
      const option = document.createElement("option");
      option.value = device.uuid || device.index;
      option.textContent = `${device.index}: ${device.name} (${device.uuid})`;
      gpu.append(option);
    });
    if (!gpu.options.length) {
      const option = document.createElement("option");
      option.value = "0";
      option.textContent = "0";
      gpu.append(option);
    }
    initializeEvents();
    renderRoles();
    applyLanguage(language);
    updateMode();
  } catch (error) {
    setError(error);
  }
}

initialize();
