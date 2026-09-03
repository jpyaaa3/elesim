"use strict";

const query = new URLSearchParams(window.location.search);
if (query.get("token")) {
  sessionStorage.setItem("elesimSetupToken", query.get("token"));
  history.replaceState({}, "", window.location.pathname);
}

const token = sessionStorage.getItem("elesimSetupToken") || "";
const steps = ["mode", "roles", "paths", "compute", "review", "install"];
const roleOrder = ["sim", "pilot", "ui", "robot"];
// The initial selection is only a convenience; every role remains an explicit
// checkbox and can be changed independently by the operator.
const defaultRoles = ["sim", "pilot", "ui"];

let catalog = {};
let language = "ko";
let context = null;
let currentStep = 0;
let pollTimer = null;
let browseTarget = "";
let browseMode = "directory";
let selectedFile = "";

const byId = (id) => document.getElementById(id);
const checkedValue = (name) => document.querySelector(`input[name="${name}"]:checked`)?.value || "";
const shellQuote = (value) => `'${String(value).replace(/'/g, "'\\''")}'`;

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
  return roleOrder.filter((role) => byId(`role-${role}`)?.checked);
}

function renderRoles() {
  const selected = new Set(
    roleOrder.filter((role) => byId(`role-${role}`)?.checked)
  );
  if (!selected.size && !byId("role-options").children.length) {
    defaultRoles.forEach((role) => selected.add(role));
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
      updateTailscaleLoginCommand();
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
  const attachment = byId("developer-attachment").checked;
  byId("developer-workspace-row").hidden = !attachment;
  byId("developer-workspace").disabled = !attachment;
  updateTailscaleLoginCommand();
  updateConditionalControls();
}

function updateDeveloperAttachmentControls() {
  updateMode();
}

function updateTailscaleLoginCommand() {
  const row = byId("tailscale-login-command-row");
  if (!row) return;
  const hasContainerRole = selectedRoles().some((role) => role !== "robot");
  const usesDockerDesktop = context?.capabilities?.docker_backend === "docker-desktop";
  row.hidden = !(hasContainerRole && usesDockerDesktop);
}

function hasSim() {
  return selectedRoles().includes("sim");
}

function updateConditionalControls() {
  const gpuMode = checkedValue("gpu-mode");
  byId("gpu-device-row").hidden = gpuMode !== "specific";
  const robot = selectedRoles().includes("robot");
  byId("dds-interface-row").hidden = !robot;
}

function payload() {
  const defaults = context.defaults;
  // Endpoint, discovery and security choices belong to elesim-connections.
  // The installer emits only the manager-owned baseline and a pending managed
  // TURN record for a Sim host; no mutable host address is guessed here.
  const securityProfile = defaults.dds_security_profile || "sros2";
  const securityProvisioning = defaults.dds_security_provisioning || "managed";
  const turnMode = hasSim() && securityProfile === "sros2"
    ? "managed" : "none";
  const prefix = byId("prefix").value.trim();
  return {
    language,
    roles: selectedRoles(),
    developer_attachment: byId("developer-attachment").checked,
    developer_workspace: byId("developer-workspace").value.trim()
      || defaults.developer_workspace || "",
    developer_wslg: Boolean(context.capabilities.wslg_available),
    prefix,
    bin_dir: byId("bin-dir").value.trim(),
    source_root: "",
    gpu_mode: checkedValue("gpu-mode") || "inherit",
    gpu_device: checkedValue("gpu-mode") === "specific" ? byId("gpu-device").value : "",
    dds_system_id: defaults.dds_system_id || "elesim",
    dds_domain_id: Number(defaults.dds_domain_id || 0),
    dds_rmw_implementation: defaults.dds_rmw_implementation || "rmw_cyclonedds_cpp",
    dds_discovery_mode: defaults.dds_discovery_mode || "multicast",
    dds_static_peers: "",
    dds_interface: selectedRoles().includes("robot")
      ? byId("dds-interface").value.trim() : "",
    dds_security_profile: securityProfile,
    dds_security_provisioning: securityProvisioning,
    dds_keystore: "",
    dds_enclave: "",
    ssh: {},
    turn_mode: turnMode,
    turn_url: "",
    turn_realm: turnMode === "managed" ? "elesim.local" : "",
    turn_public_host: "",
    turn_secret_file: turnMode === "managed" ? `${prefix}/secrets/turn.secret` : "",
    turn_credential_file: "",
    register_path: byId("register-path").checked,
    runtime_text_logs: {
      enabled: byId("runtime-text-logs").checked
    },
    repository: context.repository,
    ref: context.ref
  };
}

function validateCurrentStep() {
  const step = steps[currentStep];
  if (step === "roles" && !selectedRoles().length) {
    throw new Error(t("error.roles"));
  }
  if (step === "paths" && (!byId("prefix").value.trim() || !byId("bin-dir").value.trim())) {
    throw new Error(t("error.generic"));
  }
  if (
    step === "paths" && selectedRoles().includes("robot") &&
    !byId("dds-interface").value.trim()
  ) {
    throw new Error(t("error.robot_dds_interface"));
  }
  if (
    step === "mode" && byId("developer-attachment").checked &&
    !byId("developer-workspace").value.trim()
  ) {
    throw new Error(t("error.developer_workspace"));
  }
}

async function prepareReview() {
  const summary = await api("/api/validate", {
    method: "POST",
    body: JSON.stringify(payload())
  });
  const rows = [
    ["review.attachment", summary.developer_attachment ? t("value.yes") : t("value.no")],
    ["review.workspace", summary.developer_workspace || "—"],
    ["review.roles", summary.roles.length ? summary.roles.join(", ") : "—"],
    ["review.prefix", summary.prefix],
    ["review.bin", summary.bin_dir],
    ["review.gpu", summary.gpu_mode],
    ["review.security", summary.security_profile === "sros2"
      ? `${summary.security_profile} (${summary.security_provisioning})`
      : summary.security_profile],
    ["review.turn", summary.turn_mode],
    ...(selectedRoles().includes("robot")
      ? [["review.dds_interface", summary.dds_interface]] : []),
    ["review.path", summary.register_path ? t("value.yes") : t("value.no")],
    ["review.logs", summary.runtime_text_logs ? t("value.yes") : t("value.no")]
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
  byId("review-warning").hidden = true;
}

function updateStep() {
  const atInstall = steps[currentStep] === "install";
  document.querySelectorAll("[data-step]").forEach((page) => {
    page.classList.toggle("active", page.dataset.step === steps[currentStep]);
  });
  document.querySelectorAll("[data-step-link]").forEach((item, index) => {
    item.classList.toggle("active", index === currentStep);
    item.classList.toggle("completed", index < currentStep);
  });
  byId("back-button").disabled = currentStep === 0 || atInstall;
  byId("next-button").hidden = atInstall;
  byId("close-installer").hidden = !atInstall;
  byId("next-button").textContent = steps[currentStep] === "review" ? t("action.install") : t("action.next");
  byId("step-position").textContent = `${currentStep + 1} / ${steps.length}`;
}

async function nextStep() {
  try {
    setError("");
    validateCurrentStep();
    if (steps[currentStep] === "compute") {
      await prepareReview();
      currentStep = steps.indexOf("review");
      updateStep();
      return;
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
    currentStep -= 1;
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
      const binDir = byId("bin-dir").value.trim();
      const managerCleanup =
        'if [ "$(docker inspect -f \'{{.State.Running}}\' elesim-manager 2>/dev/null)" = false ]; then docker rm elesim-manager; fi';
      byId("start-command").textContent =
        `cd ${shellQuote(binDir)} && source ~/.bashrc && ${managerCleanup}`;
      byId("post-install-command").textContent = "elesim-connections";
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
  browseMode = ["ssh-key"].includes(target)
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

function initializeEvents() {
  document.querySelectorAll("[data-language]").forEach((button) => {
    button.addEventListener("click", () => applyLanguage(button.dataset.language));
  });
  byId("developer-attachment").addEventListener("change", updateDeveloperAttachmentControls);
  document.querySelectorAll('input[name="gpu-mode"]')
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
    button.addEventListener("click", () => copyText(
      byId(button.dataset.copyTarget).textContent.trim()
    ));
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
    byId("developer-workspace").value = context.defaults.developer_workspace || context.defaults.prefix;
    byId("dds-interface").value = context.tailscale?.default_interface || "";
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
    updateTailscaleLoginCommand();
  } catch (error) {
    setError(error);
  }
}

initialize();
