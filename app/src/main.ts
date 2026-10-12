import "leaflet/dist/leaflet.css";
import "./style.css";
import L from "leaflet";
import { listen } from "@tauri-apps/api/event";
import { getCurrentWebview } from "@tauri-apps/api/webview";
import { open, save } from "@tauri-apps/plugin-dialog";
import { mountPlanner } from "./planner";
import { mountViewer3D, Viewer3D } from "./viewer3d";
import {
  api,
  ExportFormat,
  JobFinished,
  Progress,
  ProjectView,
  RecentProject,
  Settings,
  SystemInfo,
} from "./api";

const root = document.getElementById("app")!;

const state: {
  sys: SystemInfo | null;
  project: ProjectView | null;
  recent: RecentProject[];
  busy: "process" | "export" | "3d" | null;
  tab: "map" | "3d";
  progress: Progress | null;
  notice: { kind: "ok" | "error" | "info"; text: string } | null;
} = { sys: null, project: null, recent: [], busy: null, tab: "map", progress: null, notice: null };

let map: L.Map | null = null;
let viewer: Viewer3D | null = null;

const esc = (s: string) =>
  s.replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]!);

const STAGES: Record<string, string> = {
  features: "Finding features",
  matching: "Matching photos",
  solve: "Placing photos",
  render: "Drawing the map",
  pyramid: "Building zoom levels",
  export: "Exporting",
  matching3d: "Matching photos in 3D",
  tracks: "Linking matches",
  bundle: "Refining cameras",
  colour: "Colouring points",
  "dense-prep": "Preparing photos for depth",
  depth: "Computing depth",
  fuse: "Building the point cloud",
  done: "Finished",
};

function fmtGsd(m: number, georef: boolean) {
  if (!georef) return `${m.toFixed(2)} units/px`;
  return m < 1 ? `${(m * 100).toFixed(1)} cm/px` : `${m.toFixed(2)} m/px`;
}

function fmtPixels(w: number, h: number) {
  const mp = (w * h) / 1e6;
  return `${w.toLocaleString()} × ${h.toLocaleString()} px (${mp >= 1000 ? (mp / 1000).toFixed(1) + " GP" : mp.toFixed(0) + " MP"})`;
}

function profileLabel(kind: string) {
  return { lite: "Lite (8 GB)", standard: "Standard (16 GB)", high: "High (32 GB+)" }[kind] ?? kind;
}

/** A notice that doesn't re-render the page — used by the flight planner. */
function toast(kind: "ok" | "error" | "info", text: string) {
  document.querySelectorAll(".notice.floating").forEach((n) => n.remove());
  const d = document.createElement("div");
  d.className = `notice floating ${kind}`;
  d.innerHTML = `${esc(text)}<button class="icon">×</button>`;
  d.querySelector("button")!.addEventListener("click", () => d.remove());
  document.body.appendChild(d);
  window.setTimeout(() => d.remove(), 8000);
}

function notify(kind: "ok" | "error" | "info", text: string) {
  state.notice = { kind, text };
  render();
}

// ───────────────────────────────── Home ─────────────────────────────────

function homeView() {
  const s = state.sys;
  const recent = state.recent
    .map(
      (r) => `<button class="recent" data-open="${esc(r.dir)}">
        <span class="recent-name">${esc(r.name)}</span>
        <span class="recent-meta">${r.photos} photos · ${esc(r.dir)}</span>
      </button>`,
    )
    .join("");
  return `
  <div class="home">
    <div class="brand">
      <div class="logo">◢◤</div>
      <h1>Aerial Stitch</h1>
      <p class="tag">Turn drone photos into one map — entirely on this computer.</p>
    </div>
    <div class="home-actions">
      <button class="primary big" data-action="new">New project</button>
      <button class="big" data-action="open">Open project…</button>
    </div>
    <div class="home-actions" style="margin-top:12px">
      <button class="link" data-action="planner">✈ Plan a mapping flight (Litchi)</button>
    </div>
    ${recent ? `<h3 class="section">Recent</h3><div class="recent-list">${recent}</div>` : ""}
    ${
      s
        ? `<div class="sys">${esc(s.os)} · ${esc(s.arch)} · ${s.ram_gb.toFixed(0)} GB RAM · ${s.cores} cores ·
           <b>${profileLabel(s.profile.kind)}</b> profile</div>`
        : ""
    }
  </div>`;
}

// ──────────────────────────────── Project ───────────────────────────────

function photosPanel(p: ProjectView) {
  const gps = p.photos.filter((x) => x.lat != null).length;
  const missing = p.photos.filter((x) => x.missing).length;
  const list = p.photos
    .slice(0, 300)
    .map(
      (x) => `<li class="${x.missing ? "missing" : ""}" title="${esc(x.path)}">
        <span>${esc(x.name)}</span>
        <span class="dim">${x.lat != null ? "GPS" : "no GPS"}</span>
        <button class="icon" data-remove="${x.id}" title="Remove from project">×</button>
      </li>`,
    )
    .join("");
  return `
  <section class="panel">
    <div class="panel-head">
      <h3>Photos <span class="count">${p.photos.length.toLocaleString()}</span></h3>
    </div>
    <div class="row">
      <button data-action="add-folder">Add folder…</button>
      <button data-action="add-files">Add photos…</button>
    </div>
    <p class="hint">Or drag photos or folders anywhere onto this window.</p>
    ${
      p.photos.length
        ? `<canvas id="track" width="300" height="180" title="Photo positions from GPS"></canvas>
           <p class="dim small">${gps.toLocaleString()} with GPS${missing ? ` · <span class="warn">${missing} missing on disk</span>` : ""}</p>
           <details><summary>List</summary><ul class="photo-list">${list}</ul>
           ${p.photos.length > 300 ? `<p class="dim small">…and ${(p.photos.length - 300).toLocaleString()} more</p>` : ""}</details>`
        : ""
    }
  </section>`;
}

function settingsPanel(p: ProjectView) {
  const s = p.settings;
  const sys = state.sys;
  const opt = (v: string, label: string, cur: string) =>
    `<option value="${v}" ${v === cur ? "selected" : ""}>${label}</option>`;
  return `
  <section class="panel">
    <h3>Settings</h3>
    <label>Output detail
      <select data-setting="output_scale">
        ${opt("1", "Full (native resolution)", String(s.output_scale))}
        ${opt("0.5", "Half — 4× smaller, faster", String(s.output_scale))}
        ${opt("0.25", "Quarter — quick preview", String(s.output_scale))}
      </select>
    </label>
    <label>Memory profile
      <select data-setting="profile">
        ${opt("", `Automatic${sys ? ` — ${profileLabel(sys.profile.kind)}` : ""}`, s.profile ?? "")}
        ${opt("lite", profileLabel("lite"), s.profile ?? "")}
        ${opt("standard", profileLabel("standard"), s.profile ?? "")}
        ${opt("high", profileLabel("high"), s.profile ?? "")}
      </select>
    </label>
    <label>Neighbours compared per photo
      <input type="number" min="4" max="40" value="${s.neighbours}" data-setting="neighbours" />
    </label>
  </section>`;
}

function runPanel(p: ProjectView) {
  const busy = state.busy;
  const pr = state.progress;
  const pct = pr && pr.total > 0 ? Math.round((pr.done / pr.total) * 100) : 0;
  return `
  <section class="panel">
    ${
      busy
        ? `<div class="progress">
             <div class="progress-label">${esc(STAGES[pr?.stage ?? ""] ?? "Working")}${
               pr && pr.total > 1 ? ` — ${pr.done.toLocaleString()} / ${pr.total.toLocaleString()}` : ""
             }</div>
             <div class="bar"><div style="width:${pct}%"></div></div>
             <div class="dim small">${esc(pr?.message ?? "")}</div>
             <button data-action="cancel">Cancel</button>
           </div>`
        : `<button class="primary wide" data-action="process" ${p.photos.length < 2 ? "disabled" : ""}>
             ${p.manifest ? "Re-process" : "Stitch photos"}</button>
           <button class="wide" data-action="build3d" ${p.photos.length < 3 ? "disabled" : ""}>
             ${p.model3d?.points ? "Rebuild 3D model" : "Build 3D model"}</button>
           ${
             p.manifest
               ? `<button class="wide" data-action="export">Export map…</button>`
               : `<p class="hint">Processing reuses earlier work, so adding photos later only processes what's new.</p>`
           }`
    }
  </section>`;
}

function projectView(p: ProjectView) {
  const m = p.manifest;
  return `
  <div class="layout">
    <aside>
      <div class="side-head">
        <button class="icon" data-action="home" title="All projects">‹</button>
        <div>
          <div class="pname">${esc(p.name)}</div>
          <div class="dim small" title="${esc(p.dir)}">${esc(p.dir)}</div>
        </div>
      </div>
      ${runPanel(p)}
      ${photosPanel(p)}
      ${settingsPanel(p)}
    </aside>
    <main>
      <div class="tabs">
        <button class="tab ${state.tab === "map" ? "on" : ""}" data-action="tab-map">2D map</button>
        <button class="tab ${state.tab === "3d" ? "on" : ""}" data-action="tab-3d">3D model</button>
      </div>
      <div class="tab-body">
      ${state.tab === "3d" ? view3dPane(p) : mapPane(p, m)}
      </div>
    </main>
  </div>`;
}

function mapPane(p: ProjectView, m: ProjectView["manifest"]) {
  return m
    ? `<div id="map"></div>
       <div class="map-info">
         ${fmtPixels(m.width, m.height)} · ${fmtGsd(m.gsd, m.epsg != null)} · ${m.photos} photos
         ${m.epsg ? ` · EPSG:${m.epsg}` : " · not georeferenced"}
         <span id="cursor"></span>
       </div>`
    : `<div class="empty">
         <div class="empty-icon">⧉</div>
         <p>${p.photos.length < 2 ? "Add your drone photos to begin." : "Ready to stitch."}</p>
       </div>`;
}

function view3dPane(p: ProjectView) {
  const m3 = p.model3d;
  if (!m3) {
    return `<div class="empty">
      <div class="empty-icon">◬</div>
      <p>Build a 3D point cloud of the site from your photos.</p>
      <p class="dim small" style="max-width:440px;text-align:center;line-height:1.5">
        Uses every photo — angled ones included, which give walls and tree sides their shape.
        It takes longer than the 2D map: roughly a few seconds per photo.</p>
      <button class="primary" data-action="build3d" ${p.photos.length < 3 || state.busy ? "disabled" : ""}>Build 3D model</button>
    </div>`;
  }
  return `<div id="view3d"></div>
    <div class="map-info">
      <span id="v3d-status">${(m3.points || m3.sparse_points).toLocaleString()} points</span>
      <span class="v3d-tools">
        <label>Point size <input type="range" min="1" max="6" step="0.5" value="2" id="v3d-size" /></label>
        <button class="small-btn" data-action="v3d-reset">Reset view</button>
        <button class="small-btn" data-action="export-cloud">Export point cloud…</button>
      </span>
    </div>`;
}

function exportDialog(p: ProjectView) {
  const georef = p.manifest?.epsg != null;
  const levels = p.levels
    .slice(0, 6)
    .map(
      (l, i) => `<option value="${l.zoom}" ${i === 0 ? "selected" : ""}>
        ${i === 0 ? "Full" : `1/${2 ** i}`} — ${fmtPixels(l.width, l.height)}, ${fmtGsd(l.gsd, georef)}</option>`,
    )
    .join("");
  return `
  <div class="modal-back" data-action="close-modal">
    <div class="modal">
      <h3>Export map</h3>
      <label>Format
        <select id="ex-format">
          <option value="geotiff">GeoTIFF — for GIS tools, any size, keeps location</option>
          <option value="png">PNG — lossless, transparent edges</option>
          <option value="jpeg">JPEG — smallest file, max 65,500 px a side</option>
        </select>
      </label>
      <label>Resolution
        <select id="ex-zoom">${levels}</select>
      </label>
      ${georef ? "" : `<p class="hint">No GPS was found, so the export won't carry a map location.</p>`}
      <div class="row end">
        <button data-action="close-modal">Cancel</button>
        <button class="primary" data-action="do-export">Save…</button>
      </div>
    </div>
  </div>`;
}

let modal: "export" | null = null;

function render() {
  const p = state.project;
  const notice = state.notice
    ? `<div class="notice ${state.notice.kind}">${esc(state.notice.text)}<button class="icon" data-action="dismiss">×</button></div>`
    : "";
  if (map) {
    map.remove();
    map = null;
  }
  if (viewer) {
    viewer.dispose();
    viewer = null;
  }
  root.innerHTML = (p ? projectView(p) : homeView()) + (p && modal === "export" ? exportDialog(p) : "") + notice;
  if (p) {
    drawTrack(p);
    if (state.tab === "map" && p.manifest) mountMap(p);
    if (state.tab === "3d" && p.model3d) mount3d(p);
  }
}

function mount3d(p: ProjectView) {
  const el = document.getElementById("view3d");
  const m3 = p.model3d!;
  if (!el) return;
  // The viewer loads a lighter copy; "Export point cloud" saves the full one.
  const file = m3.points ? "view.ply" : "sparse.ply";
  const status = document.getElementById("v3d-status");
  viewer = mountViewer3D(el, `${tileBase()}3d/${file}?v=${m3.version}`, (t) => {
    if (status) status.textContent = t;
  });
  document.getElementById("v3d-size")?.addEventListener("input", (e) => {
    viewer?.setPointSize(Number((e.target as HTMLInputElement).value));
  });
}

/** Re-renders only the run panel, so progress updates don't rebuild the map. */
function renderRun() {
  const p = state.project;
  const panel = root.querySelector("aside .panel");
  if (!p || !panel) return render();
  panel.outerHTML = runPanel(p);
}

// ────────────────────────────── Map viewer ──────────────────────────────

function tileBase() {
  // Custom protocols are served as http://<scheme>.localhost on Windows.
  return navigator.userAgent.includes("Windows") ? "http://tiles.localhost/" : "tiles://localhost/";
}

function mountMap(p: ProjectView) {
  const m = p.manifest!;
  const el = document.getElementById("map");
  if (!el) return;
  map = L.map(el, { crs: L.CRS.Simple, minZoom: 0, maxZoom: m.max_zoom + 2, zoomSnap: 0.5, attributionControl: false });
  const sw = map.unproject([0, m.height], m.max_zoom);
  const ne = map.unproject([m.width, 0], m.max_zoom);
  const bounds = L.latLngBounds(sw, ne);
  L.tileLayer(`${tileBase()}{z}/{x}/{y}.jpg?v=${m.version}`, {
    tileSize: m.tile_size,
    maxNativeZoom: m.max_zoom,
    maxZoom: m.max_zoom + 2,
    noWrap: true,
    bounds,
    keepBuffer: 4,
    errorTileUrl: "data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==",
  }).addTo(map);
  map.fitBounds(bounds);
  map.setMaxBounds(bounds.pad(0.5));

  // Scale bar in real units: one full-res pixel is m.gsd metres.
  const ScaleBar = L.Control.extend({
    onAdd() {
      const d = L.DomUtil.create("div", "scalebar");
      const update = () => {
        const z = map!.getZoom();
        const mPerPx = m.gsd * 2 ** (m.max_zoom - z);
        const target = mPerPx * 120;
        const nice = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000].find((v) => v >= target) ?? target;
        const px = nice / mPerPx;
        const label = m.epsg ? (nice >= 1000 ? `${nice / 1000} km` : nice >= 1 ? `${nice} m` : `${nice * 100} cm`) : `${nice} u`;
        d.innerHTML = `<div class="sb-bar" style="width:${px}px"></div><span>${label}</span>`;
      };
      map!.on("zoomend", update);
      update();
      return d;
    },
  });
  new ScaleBar({ position: "bottomleft" }).addTo(map);

  const cursor = document.getElementById("cursor");
  map.on("mousemove", (e: L.LeafletMouseEvent) => {
    if (!cursor || m.utm_left == null || m.utm_top == null) return;
    const pt = map!.project(e.latlng, m.max_zoom);
    const east = m.utm_left + pt.x * m.gsd;
    const north = m.utm_top - pt.y * m.gsd;
    cursor.textContent = ` · E ${east.toFixed(1)}  N ${north.toFixed(1)}`;
  });
}

/** Photo positions from GPS, as a quick sanity check of the survey shape. */
function drawTrack(p: ProjectView) {
  const c = document.getElementById("track") as HTMLCanvasElement | null;
  if (!c) return;
  const pts = p.photos.filter((x) => x.lat != null && x.lon != null);
  const g = c.getContext("2d")!;
  const dpr = window.devicePixelRatio || 1;
  const W = c.clientWidth || 300;
  const H = 180;
  c.width = W * dpr;
  c.height = H * dpr;
  g.scale(dpr, dpr);
  g.clearRect(0, 0, W, H);
  if (pts.length < 2) {
    g.fillStyle = "#7a8796";
    g.font = "12px system-ui";
    g.fillText("No GPS positions in these photos", 12, 24);
    return;
  }
  const lat0 = pts.reduce((a, x) => a + x.lat!, 0) / pts.length;
  const kx = Math.cos((lat0 * Math.PI) / 180);
  const xs = pts.map((x) => x.lon! * kx);
  const ys = pts.map((x) => x.lat!);
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  const span = Math.max(x1 - x0, y1 - y0) || 1e-9;
  const s = Math.min((W - 20) / span, (H - 20) / span);
  const ox = (W - (x1 - x0) * s) / 2;
  const oy = (H - (y1 - y0) * s) / 2;
  g.strokeStyle = "rgba(54,224,192,0.25)";
  g.beginPath();
  pts.forEach((_, i) => {
    const X = ox + (xs[i] - x0) * s;
    const Y = H - oy - (ys[i] - y0) * s;
    i ? g.lineTo(X, Y) : g.moveTo(X, Y);
  });
  g.stroke();
  g.fillStyle = "#36e0c0";
  pts.forEach((_, i) => {
    g.beginPath();
    g.arc(ox + (xs[i] - x0) * s, H - oy - (ys[i] - y0) * s, 2, 0, Math.PI * 2);
    g.fill();
  });
}

// ─────────────────────────────── Actions ────────────────────────────────

async function guard(fn: () => Promise<void>) {
  try {
    await fn();
  } catch (e) {
    notify("error", String(e));
  }
}

async function newProject() {
  const dir = await open({ directory: true, title: "Choose an empty folder for the new project" });
  if (typeof dir !== "string") return;
  const name = dir.split(/[\\/]/).filter(Boolean).pop() ?? "Project";
  state.project = await api.createProject(dir, name);
  state.notice = null;
  render();
}

async function openProject(dir?: string) {
  const d = dir ?? (await open({ directory: true, title: "Open a project folder" }));
  if (typeof d !== "string") return;
  state.project = await api.openProject(d);
  state.notice = null;
  render();
}

async function addPaths(paths: string[]) {
  const p = state.project;
  if (!p || !paths.length) return;
  notify("info", "Reading photos…");
  const r = await api.addPhotos(p.dir, paths);
  state.project = r.project;
  notify(
    r.failed.length ? "error" : "ok",
    `Added ${r.added.toLocaleString()} photos` + (r.failed.length ? ` · ${r.failed.length} couldn't be read` : ""),
  );
}

async function onAction(action: string) {
  const p = state.project;
  switch (action) {
    case "new":
      return newProject();
    case "open":
      return openProject();
    case "planner": {
      // Its own container, so the planner's listeners leave with it.
      const host = document.createElement("div");
      host.style.height = "100%";
      root.replaceChildren(host);
      return mountPlanner(
        host,
        () => {
          render();
        },
        toast,
      );
    }
    case "home":
      state.project = null;
      state.recent = await api.recentProjects();
      return render();
    case "dismiss":
      state.notice = null;
      return render();
    case "add-folder": {
      const d = await open({ directory: true, multiple: true, title: "Add folders of photos" });
      if (d) await addPaths(Array.isArray(d) ? d : [d]);
      return;
    }
    case "add-files": {
      const f = await open({
        multiple: true,
        title: "Add photos",
        filters: [{ name: "Photos", extensions: ["jpg", "jpeg", "JPG", "JPEG", "png", "tif", "tiff"] }],
      });
      if (f) await addPaths(Array.isArray(f) ? f : [f]);
      return;
    }
    case "process":
      if (!p) return;
      state.busy = "process";
      state.progress = { stage: "features", done: 0, total: 0, message: "Starting" };
      state.notice = null;
      render();
      return api.process(p.dir);
    case "cancel":
      return api.cancel();
    case "tab-map":
      state.tab = "map";
      return render();
    case "tab-3d":
      state.tab = "3d";
      return render();
    case "build3d":
      if (!p) return;
      state.busy = "3d";
      state.tab = "3d";
      state.progress = { stage: "features", done: 0, total: 0, message: "Starting" };
      state.notice = null;
      render();
      return api.build3d(p.dir);
    case "v3d-reset":
      return viewer?.resetView();
    case "export-cloud": {
      if (!p) return;
      const path = await save({
        title: "Save point cloud",
        defaultPath: `${p.name}.ply`,
        filters: [{ name: "Point cloud (PLY)", extensions: ["ply"] }],
      });
      if (!path) return;
      await api.exportCloud(p.dir, path);
      return notify("ok", "Point cloud saved — opens in CloudCompare, MeshLab or Blender");
    }
    case "export":
      modal = "export";
      return render();
    case "close-modal":
      modal = null;
      return render();
    case "do-export": {
      if (!p) return;
      const format = (document.getElementById("ex-format") as HTMLSelectElement).value as ExportFormat;
      const zoom = Number((document.getElementById("ex-zoom") as HTMLSelectElement).value);
      const ext = { geotiff: "tif", png: "png", jpeg: "jpg" }[format];
      const path = await save({
        title: "Save map",
        defaultPath: `${p.name}.${ext}`,
        filters: [{ name: format.toUpperCase(), extensions: [ext] }],
      });
      if (!path) return;
      modal = null;
      state.busy = "export";
      state.progress = { stage: "export", done: 0, total: 0, message: "Starting" };
      render();
      return api.exportMap(p.dir, format, zoom, path);
    }
  }
}

root.addEventListener("click", (e) => {
  const t = e.target as HTMLElement;
  const a = t.closest<HTMLElement>("[data-action]");
  if (a && (a === t || a.contains(t))) {
    if (a.classList.contains("modal-back") && t !== a) return;
    return void guard(() => onAction(a.dataset.action!));
  }
  const openBtn = t.closest<HTMLElement>("[data-open]");
  if (openBtn) return void guard(() => openProject(openBtn.dataset.open!));
  const rm = t.closest<HTMLElement>("[data-remove]");
  if (rm && state.project) {
    const id = Number(rm.dataset.remove);
    return void guard(async () => {
      state.project = await api.removePhotos(state.project!.dir, [id]);
      render();
    });
  }
});

root.addEventListener("change", (e) => {
  const t = e.target as HTMLInputElement | HTMLSelectElement;
  const key = t.dataset.setting as keyof Settings | undefined;
  const p = state.project;
  if (!key || !p) return;
  const s: Settings = { ...p.settings };
  if (key === "output_scale") s.output_scale = Number(t.value);
  if (key === "neighbours") s.neighbours = Math.max(4, Math.min(40, Number(t.value) || 10));
  if (key === "profile") s.profile = (t.value || null) as Settings["profile"];
  void guard(async () => {
    state.project = await api.saveSettings(p.dir, s);
    notify("info", "Settings saved — they apply the next time you process.");
  });
});

// ─────────────────────────────── Events ─────────────────────────────────

void listen<Progress>("progress", (e) => {
  state.progress = e.payload;
  renderRun();
});

void listen<JobFinished>("job-finished", async (e) => {
  const r = e.payload;
  state.busy = null;
  state.progress = null;
  if ((r.kind === "process" || r.kind === "3d") && state.project) {
    state.project = await api.openProject(state.project.dir);
  }
  notify(r.ok ? "ok" : "error", r.message);
});

void getCurrentWebview().onDragDropEvent((e) => {
  if (e.payload.type === "drop" && state.project && !state.busy) {
    void guard(() => addPaths(e.payload.type === "drop" ? e.payload.paths : []));
  }
});

window.addEventListener("resize", () => {
  if (state.project) drawTrack(state.project);
});

void (async () => {
  try {
    [state.sys, state.recent] = await Promise.all([api.systemInfo(), api.recentProjects()]);
  } catch (e) {
    state.notice = { kind: "error", text: String(e) };
  }
  render();
})();
