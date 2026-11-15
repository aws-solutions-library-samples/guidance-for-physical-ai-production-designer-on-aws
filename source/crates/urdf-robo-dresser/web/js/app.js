// Orchestration: catalogue palette -> select robot+tool pair -> preview
// sources -> Dress -> load the dressed URDF -> orbit + joint sliders.
// Framework-free ES modules, mirroring the fitting-room's app.js style.

import "./theme.js"; // side effect only: applies Kiro/VS Code theme colors when embedded
import * as api from "./api.js";
import * as preload from "./preload.js";
import { Viewer } from "./viewer.js";
import { buildJointSliders } from "./joints.js";
import { installControls, refreshOrientLabels } from "./controls.js";

const state = {
  robotId: null,
  toolId: null,
  orient: [0, 0, 0],
  hull: "medium",
  dressedRel: null,
};

const viewer = new Viewer(document.getElementById("viewport"));
const statusEl = document.getElementById("status-line");
const diagEl = document.getElementById("diagnostics");
const jointsEl = document.getElementById("joint-sliders");
const hoverEl = document.getElementById("joint-hover");

const statusChip = document.getElementById("status-chip");

/**
 * Write the status readout. `kind` tints the chip: "info" (cyan, default),
 * "busy" (cyan + pulsing dot), "ok", "warn", "error".
 *
 * Callers that do not pass a kind get it inferred, so the ~20 existing call
 * sites keep working: a trailing "…" means work is in flight, and "failed"
 * anywhere means it is not.
 */
function setStatus(msg, kind) {
  statusEl.textContent = msg;
  if (!kind) {
    if (/fail|error/i.test(msg)) kind = "error";
    else if (msg.trimEnd().endsWith("…")) kind = "busy";
    else kind = "info";
  }
  statusChip.className = "status-chip" + (kind === "info" ? "" : " " + kind);
}

// Handle to the current joint sliders (rebuilt on every load) so viewport
// dragging can sync the panel and highlight the active row.
let jointSliders = { setValue() {}, highlight() {} };

function showJoints(robot) {
  jointSliders = buildJointSliders(jointsEl, robot, viewer);
}

// Re-color the WebGL scene (background/grid/tool/hulls) when the host theme
// changes (see js/theme.js) — CSS-var changes alone don't repaint canvas content.
window.addEventListener("robo-dresser:theme-applied", () => viewer.refreshTheme());

// Dragging a joint in the viewport updates its slider live (FR-059).
viewer.onJointChange = (name, value) => jointSliders.setValue(name, value);
// Hovering a joint shows its name in the viewport + highlights the panel row.
viewer.onJointHover = (name) => {
  jointSliders.highlight(name);
  if (name) {
    hoverEl.textContent = name;
    hoverEl.hidden = false;
  } else {
    hoverEl.hidden = true;
  }
};

// ---------- Palette ----------
async function refreshPalette() {
  try {
    const cat = await api.listCatalogue();
    paint("#palette-robots", cat.robots, "robot", "urdf-robot-collection");
    paint("#palette-tools", cat.tools, "tool", "urdf-tool-collection");
    setStatus(`Catalogue: ${cat.robots.length} robots, ${cat.tools.length} tools.`);
  } catch (e) {
    setStatus("Catalogue load failed: " + e.message);
  }
  await refreshDressed();
}

/** List already-dressed robots so the user can view prior results (FR-057). */
async function refreshDressed() {
  try {
    const dressed = await api.listDressed();
    const el = document.querySelector("#palette-dressed");
    el.innerHTML = "";
    if (dressed.length === 0) {
      el.innerHTML = '<li class="muted">None yet — dress a pair.</li>';
      return;
    }
    for (const entry of dressed) {
      const li = document.createElement("li");
      li.dataset.kind = "dressed";
      li.dataset.name = entry.name;
      const name = document.createElement("span");
      name.className = "entry-name";
      name.textContent = entry.name;
      name.title = entry.urdf_rel;
      li.append(name, cacheButton("urdf-dressed-robots", entry.name));
      li.addEventListener("click", () => viewDressed(entry, li));
      el.appendChild(li);
    }
  } catch (e) {
    setStatus("Dressed list failed: " + e.message);
  }
  // A new dress adds an entry the cache has never seen, so keep the note honest.
  showPreloadState();
}

function paint(selector, entries, kind, collection) {
  const el = document.querySelector(selector);
  el.innerHTML = "";
  for (const entry of entries) {
    const li = document.createElement("li");
    li.dataset.kind = kind;
    li.dataset.name = entry.name;

    // Asset VALIDITY (AssetStatusDto) — distinct from cache state, which the
    // trailing button shows. Keeping them separate means an incomplete asset
    // stays visible whether or not it happens to be cached.
    const dot = document.createElement("span");
    dot.className = "status-dot " + (entry.status?.kind || "valid");
    dot.title = statusTitle(entry.status);

    const name = document.createElement("span");
    name.className = "entry-name";
    name.textContent = entry.name;

    li.append(dot, name, cacheButton(collection, entry.name));
    li.addEventListener("click", () => selectEntry(kind, entry, li));
    el.appendChild(li);
  }
}

/**
 * Per-entry cache indicator that doubles as its own retry button: hollow while
 * the entry is not cached, green once it is. Clicking preloads just this entry,
 * so a single failed or stale item can be refetched without redoing the lot.
 */
function cacheButton(collection, name) {
  const btn = document.createElement("button");
  btn.className = "cache-dot";
  btn.dataset.collection = collection;
  btn.dataset.name = name;
  btn.type = "button";
  setCacheButton(btn, false);
  btn.addEventListener("click", async (e) => {
    // The row itself selects/previews the entry; caching must not also do that.
    e.stopPropagation();
    if (btn.disabled) return;
    btn.disabled = true;
    btn.classList.add("busy");
    try {
      const r = await preload.preloadOne(collection, name);
      setCacheButton(btn, true);
      setStatus(r.updated
        ? `Cached ${name} — ${mb(r.bytes)}.`
        : `${name} was already current (nothing transferred).`);
      showPreloadState();
    } catch (err) {
      setCacheButton(btn, false);
      setStatus(`Caching ${name} failed: ${err.message}`);
    } finally {
      btn.disabled = false;
      btn.classList.remove("busy");
    }
  });
  return btn;
}

function setCacheButton(btn, cached) {
  btn.classList.toggle("cached", cached);
  btn.title = cached
    ? "Cached locally — click to re-fetch"
    : "Not cached — click to fetch this entry";
}

/** Repaint every cache dot from the server's freshness index. */
async function refreshCacheDots() {
  let fresh;
  try {
    ({ fresh } = await preload.inspect());
  } catch {
    return;
  }
  const cached = new Set(fresh.map((e) => `${e.collection}/${e.name}`));
  document.querySelectorAll("button.cache-dot").forEach((btn) => {
    setCacheButton(btn, cached.has(`${btn.dataset.collection}/${btn.dataset.name}`));
  });
}

/** Load a previously-dressed robot straight into the viewer (FR-057). */
async function viewDressed(entry, li) {
  const list = li.parentElement;
  list.querySelectorAll("li.selected").forEach((n) => n.classList.remove("selected"));
  li.classList.add("selected");
  setStatus(`Loading dressed ${entry.name}…`);
  try {
    const robot = await viewer.loadUrdf(api.assetUrl(entry.urdf_rel));
    showJoints(robot);
    state.dressedRel = entry.urdf_rel;
    setOutputPath(entry.urdf_rel);
    setStatus(`Viewing dressed ${entry.name}. Drag to rotate.`);
  } catch (e) {
    setStatus(`Load failed (${entry.name}): ${e.message}`);
  }
}

let badgeTimer = null;

/**
 * Show where a dressed URDF lives, under urdf-dressed-robots/. When `fresh`
 * is true (a dress just ran), flag that a NEW URDF was written to disk; the
 * badge auto-hides after 10s.
 */
function setOutputPath(rel, fresh = false) {
  const el = document.getElementById("output-path");
  const badge = document.getElementById("output-badge");
  if (badgeTimer) { clearTimeout(badgeTimer); badgeTimer = null; }
  if (rel) {
    el.textContent = rel;
    el.title = rel;
    el.classList.remove("muted");
    badge.hidden = !fresh;
    if (fresh) {
      badgeTimer = setTimeout(() => { badge.hidden = true; badgeTimer = null; }, 10000);
    }
  } else {
    el.textContent = "—";
    el.classList.add("muted");
    badge.hidden = true;
  }
}

function statusTitle(status) {
  if (!status) return "";
  if (status.kind === "valid") return "valid";
  if (status.kind === "incomplete") return "incomplete: " + (status.issues || []).join(", ");
  if (status.kind === "error") return "error: " + (status.message || "");
  return status.kind;
}

// ---------- Selection + source preview ----------
async function selectEntry(kind, entry, li) {
  // Highlight the selected entry within its list.
  const list = li.parentElement;
  list.querySelectorAll("li.selected").forEach((n) => n.classList.remove("selected"));
  li.classList.add("selected");

  if (kind === "robot") {
    state.robotId = entry.name;
    document.getElementById("sel-robot").textContent = "Robot: " + entry.name;
    await preview(`urdf-robot-collection/${entry.name}/${entry.name}.urdf`, `robot ${entry.name}`);
  } else {
    state.toolId = entry.name;
    document.getElementById("sel-tool").textContent = "Tool: " + entry.name;
    await preview(`urdf-tool-collection/${entry.name}/${entry.name}.urdf`, `tool ${entry.name}`);
  }
  updateDressEnabled();
}

async function preview(rel, label) {
  setStatus(`Loading ${label}…`);
  try {
    const robot = await viewer.loadUrdf(api.assetUrl(rel));
    // Show joint names for the source too, not just after dressing.
    showJoints(robot);
    setStatus(`Previewing ${label}.`);
  } catch (e) {
    setStatus(`Preview failed (${label}): ${e.message}`);
  }
}

function updateDressEnabled() {
  document.getElementById("btn-dress").disabled = !(state.robotId && state.toolId);
}

// ---------- Dress + load result ----------
async function runDress() {
  if (!state.robotId || !state.toolId) {
    setStatus("Select a robot and a tool first.");
    return;
  }
  setStatus(`Dressing ${state.robotId} + ${state.toolId}…`);
  diagEl.innerHTML = "";
  try {
    const r = await api.dress(state);
    renderDiagnostics(r.diagnostics || []);
    state.dressedRel = r.dressed_urdf_rel;
    setOutputPath(r.dressed_urdf_rel, true); // just written to disk
    const robot = await viewer.loadUrdf(api.assetUrl(r.dressed_urdf_rel));
    showJoints(robot);
    await refreshDressed(); // the new combo now appears in the Dressed list
    setStatus(`Dressed ${state.robotId}__${state.toolId}. Drag to rotate.`);
  } catch (e) {
    // 422 = blocking validation; surface the message, keep last good model.
    renderDiagnostics([{ severity: "error", code: "dress_failed", message: e.message }]);
    setStatus("Dress failed: " + e.message);
  }
}

function renderDiagnostics(diags) {
  diagEl.innerHTML = "";
  if (diags.length === 0) {
    diagEl.innerHTML = '<li class="muted">No diagnostics.</li>';
    return;
  }
  for (const d of diags) {
    const li = document.createElement("li");
    li.className = d.severity === "error" ? "error" : "warning";
    const code = document.createElement("span");
    code.className = "code";
    code.textContent = `[${d.code}] `;
    li.append(code, document.createTextNode(d.message));
    diagEl.appendChild(li);
  }
}

// ---------- Boot ----------
installControls(state, runDress);
refreshOrientLabels(state);
updateDressEnabled();
setOutputPath(null);

// FR-058: collision-hull overlay toggle.
document.getElementById("toggle-collision").addEventListener("change", (e) => {
  viewer.setCollisionVisible(e.target.checked);
});

// FR-028: reach-envelope hull overlay toggle.
document.getElementById("toggle-reach").addEventListener("change", (e) => {
  // Derive the reach hull URL from the dressed URDF's combo path. The OBJ is
  // always at meshes/3dreach/reach_envelope.obj relative to the combo dir.
  let url = null;
  if (state.dressedRel) {
    const comboDir = state.dressedRel.replace(/\/[^/]+\.urdf$/, "");
    url = api.assetUrl(comboDir + "/meshes/3dreach/reach_envelope.obj");
  }
  viewer.setReachVisible(e.target.checked, url);
});

// ---------- Preload ----------
const preloadBtn = document.getElementById("btn-preload");
const preloadCancelBtn = document.getElementById("btn-preload-cancel");
const preloadNote = document.getElementById("preload-note");
let preloadAbort = null;

const MB = 1024 * 1024;
const mb = (n) => `${(n / MB).toFixed(1)} MB`;

/** Show what a preload would cost, without fetching anything. */
async function showPreloadState() {
  try {
    const { stale, fresh, staleBytes, totalBytes } = await preload.inspect();
    if (stale.length === 0) {
      preloadNote.textContent = fresh.length
        ? `Cache up-to-date — ${fresh.length} entr${fresh.length === 1 ? "y" : "ies"}, ${mb(totalBytes)}.`
        : "Catalogue is empty.";
    } else {
      preloadNote.textContent =
        `${stale.length} of ${stale.length + fresh.length} entries not cached (${mb(staleBytes)}).`;
    }
  } catch {
    preloadNote.textContent = "";
  }
  refreshCacheDots();
}

async function runPreload() {
  preloadAbort = new AbortController();
  preloadBtn.disabled = true;
  preloadCancelBtn.hidden = false;
  try {
    const result = await preload.preloadAll({
      signal: preloadAbort.signal,
      onProgress: (p) => {
        if (p.phase === "checking") setStatus("Checking cache…", "busy");
        else if (p.phase === "fetching") {
          setStatus(`Preloading ${p.label} — ${p.done + 1}/${p.total}, ${mb(p.bytes)} of ${mb(p.totalBytes)}…`, "busy");
        }
      },
    });

    const usage = await preload.cacheUsage();
    const held = usage === null ? "" : ` Cache holds ${mb(usage)}.`;
    const pruned = result.pruned ? ` Pruned ${result.pruned} removed entr${result.pruned === 1 ? "y" : "ies"}.` : "";

    if (result.upToDate) {
      // The whole point of the freshness check: say nothing was needed, rather
      // than implying work happened.
      setStatus(`Cache is up-to-date — ${result.skipped} entries, nothing to fetch.${pruned}${held}`, "ok");
    } else {
      setStatus(`Preloaded ${result.updated} entr${result.updated === 1 ? "y" : "ies"}, ${mb(result.bytes)} transferred.${pruned}${held}`, "ok");
    }
  } catch (e) {
    if (e.name === "AbortError") setStatus("Preload cancelled. Partial entries stay marked stale.", "warn");
    else setStatus("Preload failed: " + e.message, "error");
  } finally {
    preloadAbort = null;
    preloadBtn.disabled = false;
    preloadCancelBtn.hidden = true;
    showPreloadState();
  }
}

preloadBtn.addEventListener("click", runPreload);
preloadCancelBtn.addEventListener("click", () => preloadAbort?.abort());

// ---------- Sidebar collapse ----------
// Mirrors the fitting room's toggles. No synthetic resize is dispatched: the
// Viewer watches its canvas with a ResizeObserver (viewer.js), so it re-fits
// itself once the grid transition settles.
document.querySelectorAll(".aside-toggle").forEach((btn) => {
  btn.addEventListener("click", () => {
    const id = btn.dataset.aside;                       // "palette" | "inspector"
    const collapsed = document.getElementById(id).classList.toggle("collapsed");
    // The chevron points the way the panel will move: the catalogue collapses
    // leftward, the inspector rightward.
    btn.textContent = id === "palette"
      ? (collapsed ? "▸" : "◂")
      : (collapsed ? "◂" : "▸");
    const label = id === "palette" ? "catalogue" : "inspector";
    btn.title = `${collapsed ? "Expand" : "Collapse"} ${label}`;
  });
});

refreshPalette();
setStatus("Ready. Pick a robot and a tool, then Dress.");

// Refill THREE.Cache from the durable cache before anything is loaded, so a
// reopened panel serves meshes from local disk instead of back over the tunnel.
preload.rehydrate().then((n) => {
  if (n) setStatus(`Ready. ${n} cached file(s) restored — pick a robot and a tool, then Dress.`);
  showPreloadState();
});
