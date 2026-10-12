import "leaflet/dist/leaflet.css";
import "./style.css";
import L from "leaflet";
import { listen } from "@tauri-apps/api/event";
import { getCurrentWebview } from "@tauri-apps/api/webview";
import { open, save } from "@tauri-apps/plugin-dialog";
import { mountPlanner } from "./planner";
import { mountViewer3D, MeasureEvent, Tool, Viewer3D } from "./viewer3d";
import * as M from "./measure";
import {
  api,
  ExportFormat,
  MapLayer,
  JobFinished,
  Progress,
  ProjectView,
  RecentProject,
  Settings,
  SystemInfo,
  VolumeLevel,
} from "./api";

const root = document.getElementById("app")!;

const state: {
  sys: SystemInfo | null;
  project: ProjectView | null;
  recent: RecentProject[];
  busy: "process" | "export" | "3d" | null;
  tab: "map" | "3d";
  layer: MapLayer;
  progress: Progress | null;
  notice: { kind: "ok" | "error" | "info"; text: string } | null;
  /** What the 3D tab shows when both exist. */
  view3d: "surface" | "points";
  tool: Tool;
  volLevel: "lowest_edge" | "mean_edge" | "highest_edge" | "custom";
  /** For a custom level: metres above the lowest point of the outline. */
  volOffset: number;
} = {
  sys: null,
  project: null,
  recent: [],
  busy: null,
  tab: "map",
  layer: "ortho",
  progress: null,
  notice: null,
  view3d: "surface",
  tool: "none",
  volLevel: "lowest_edge",
  volOffset: 0,
};

let map: L.Map | null = null;
let viewer: Viewer3D | null = null;
let mapMeasure: MapMeasure | null = null;

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
  dsm: "Building the surface",
  visibility: "Working out what each photo sees",
  ortho: "Drawing the true orthophoto",
  mesh: "Texturing the 3D surface",
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
             p.manifest || p.ortho
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

/** The map currently shown: the true orthophoto when it exists and is
 *  selected, otherwise the 2D mosaic. */
function shownLayer(p: ProjectView): { layer: MapLayer; m: ProjectView["manifest"]; levels: ProjectView["levels"] } {
  if ((state.layer === "ortho" && p.ortho) || (!p.manifest && p.ortho)) {
    return { layer: "ortho", m: p.ortho, levels: p.ortho_levels };
  }
  return { layer: "mosaic", m: p.manifest, levels: p.levels };
}

function layerSwitch(p: ProjectView) {
  if (!p.ortho || !p.manifest) return "";
  const cur = shownLayer(p).layer;
  return `<div class="layer-switch">
    <button class="${cur === "mosaic" ? "on" : ""}" data-action="layer-mosaic" title="Stitched from the photos assuming flat ground">Mosaic</button>
    <button class="${cur === "ortho" ? "on" : ""}" data-action="layer-ortho" title="Projected onto the 3D surface: nothing leans">True ortho</button>
  </div>`;
}

function mapPane(p: ProjectView, _m: ProjectView["manifest"]) {
  const { layer, m } = shownLayer(p);
  return m
    ? `<div id="map"></div>
       ${layerSwitch(p)}
       ${measurePanel(false, layer === "ortho")}
       <div class="map-info">
         ${layer === "ortho" ? "<b>True orthophoto</b> · " : ""}
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
      <p>Build a 3D model of the site from your photos.</p>
      <p class="dim small" style="max-width:440px;text-align:center;line-height:1.5">
        Uses every photo — angled ones included, which give walls and tree sides their shape.
        It takes longer than the 2D map: roughly a few seconds per photo.</p>
      <button class="primary" data-action="build3d" ${p.photos.length < 3 || state.busy ? "disabled" : ""}>Build 3D model</button>
    </div>`;
  }
  const surface = showSurface(p);
  return `<div id="view3d"></div>
    ${
      m3.mesh
        ? `<div class="layer-switch">
             <button class="${surface ? "on" : ""}" data-action="v3d-surface" title="Solid surface textured from the photos">Surface</button>
             <button class="${surface ? "" : "on"}" data-action="v3d-points" title="The raw point cloud">Points</button>
           </div>`
        : ""
    }
    ${measurePanel(true, !!p.ortho)}
    <div class="map-info">
      <span id="v3d-status">${(m3.points || m3.sparse_points).toLocaleString()} points</span>
      <span class="v3d-tools">
        ${surface ? "" : `<label>Point size <input type="range" min="1" max="6" step="0.5" value="2" id="v3d-size" /></label>`}
        <button class="small-btn" data-action="v3d-reset">Reset view</button>
        ${m3.mesh ? `<button class="small-btn" data-action="export-model">Export 3D model…</button>` : ""}
        <button class="small-btn" data-action="export-cloud">Export point cloud…</button>
      </span>
    </div>`;
}

function showSurface(p: ProjectView) {
  return !!p.model3d?.mesh && state.view3d === "surface";
}

// ────────────────────────────── Measuring ───────────────────────────────

const TOOL_LABEL: Record<Exclude<Tool, "none">, string> = {
  distance: "Distance",
  height: "Height",
  area: "Area",
  volume: "Volume",
};

/** Floating measure toolbar + results. `is3d` adds height and true surface
 *  measures; `volume` is offered only where the surface model is available. */
function measurePanel(is3d: boolean, volume: boolean) {
  const tools = (["distance", "height", "area", "volume"] as const).filter(
    (t) => (t !== "height" || is3d) && (t !== "volume" || volume),
  );
  if (state.tool !== "none" && !(tools as readonly Tool[]).includes(state.tool)) state.tool = "none";
  return `<div class="measure" id="measure">
    <div class="mtools">
      <span class="dim small">Measure</span>
      ${tools
        .map(
          (t) =>
            `<button class="${state.tool === t ? "on" : ""}" data-tool="${t}">${TOOL_LABEL[t]}</button>`,
        )
        .join("")}
    </div>
    <div id="measure-out" class="mout" ${state.tool === "none" ? "hidden" : ""}>${measureHint(state.tool, is3d)}</div>
  </div>`;
}

function measureHint(tool: Tool, is3d: boolean) {
  switch (tool) {
    case "distance":
      return `<p class="hint">Click points along the line${is3d ? " on the model" : ""}. Double-click to finish.</p>`;
    case "height":
      return `<p class="hint">Click the foot of the wall (or the ground), then its top. Orbit round to the side first so both are in view.</p>`;
    case "area":
      return is3d
        ? `<p class="hint">Click the corners of the face to measure — a roof plane, a wall, a field. Double-click to close it. Sloping faces give their true area.</p>`
        : `<p class="hint">Click around the area. Double-click to close it.</p>`;
    case "volume":
      return `${volumeLevelControls()}<p class="hint">Click around the rim of the pond or hole (or the base of a stockpile), then double-click to close it.</p>`;
    default:
      return "";
  }
}

function volumeLevelControls() {
  const o = (v: string, label: string) => `<option value="${v}" ${state.volLevel === v ? "selected" : ""}>${label}</option>`;
  return `<label class="vol-level">Fill level
    <select id="vol-level">
      ${o("lowest_edge", "Up to the lowest point of the rim (where it would spill)")}
      ${o("mean_edge", "Average height of the rim")}
      ${o("highest_edge", "Up to the highest point of the rim")}
      ${o("custom", "Custom: metres above the lowest point of the rim")}
    </select>
  </label>
  <label class="vol-level" ${state.volLevel === "custom" ? "" : "hidden"} id="vol-offset-row">Metres above the lowest rim point
    <input type="number" step="0.1" id="vol-offset" value="${state.volOffset}" />
  </label>`;
}

/** The last finished outline, kept so changing the level re-measures it. */
let lastOutline: [number, number][] | null = null;
let volumeSeq = 0;

function setTool(t: Tool) {
  state.tool = state.tool === t ? "none" : t;
  lastOutline = null;
  document.querySelectorAll<HTMLElement>("#measure [data-tool]").forEach((b) => {
    b.classList.toggle("on", b.dataset.tool === state.tool);
  });
  const out = document.getElementById("measure-out");
  if (out) {
    out.hidden = state.tool === "none";
    out.innerHTML = measureHint(state.tool, state.tab === "3d");
  }
  viewer?.setTool(state.tool);
  mapMeasure?.setTool(state.tool);
}

function showMeasure(e: MeasureEvent, is3d: boolean) {
  const out = document.getElementById("measure-out");
  if (!out || e.tool === "none") return;
  const pts = e.points;
  let body = "";
  if (e.tool === "distance") {
    const d = M.distance(pts);
    body = d ? M.distanceHtml(d, is3d) : "";
  } else if (e.tool === "height") {
    const h = M.height(pts);
    body = h ? M.heightHtml(h) : "";
  } else if (e.tool === "area") {
    const a = M.area(pts);
    body = a ? M.areaHtml(a, is3d) : "";
  } else if (e.tool === "volume") {
    if (e.finished && pts.length >= 3) {
      lastOutline = pts.map((p) => [p[0], p[1]]);
      out.innerHTML = `${volumeLevelControls()}<div id="vol-result"><p class="hint">Measuring…</p></div>`;
      void runVolume();
      return;
    }
    lastOutline = null;
    const a = M.area(pts);
    body = a ? M.areaHtml(a, false) : "";
  }
  const done = e.finished
    ? `<p class="hint">Click again to start a new measurement.</p>`
    : pts.length
      ? `<div class="row end"><button class="small-btn" data-action="measure-clear">Clear</button>${
          e.tool !== "height" ? `<button class="small-btn primary" data-action="measure-finish">Finish</button>` : ""
        }</div>`
      : "";
  out.innerHTML = (pts.length ? body : measureHint(e.tool, is3d)) + done;
}

async function runVolume() {
  const p = state.project;
  const poly = lastOutline;
  const target = document.getElementById("vol-result");
  if (!p || !poly || !target) return;
  const seq = ++volumeSeq;
  try {
    let level: VolumeLevel = { kind: state.volLevel === "custom" ? "lowest_edge" : state.volLevel };
    let r = await api.measureVolume(p.dir, poly, level);
    if (state.volLevel === "custom") {
      level = { kind: "height", z: r.level_z + state.volOffset };
      r = await api.measureVolume(p.dir, poly, level);
    }
    if (seq !== volumeSeq) return;
    const el = document.getElementById("vol-result");
    if (el) el.innerHTML = M.volumeHtml(r) + `<p class="hint">Click again to start a new outline.</p>`;
  } catch (err) {
    if (seq !== volumeSeq) return;
    const el = document.getElementById("vol-result");
    if (el) el.innerHTML = `<p class="hint error">${esc(String(err))}</p>`;
  }
}

/** Measuring on the 2D map. Points are in the map's metric frame; on the true
 *  orthophoto that is the 3D model's frame, so volume can use the surface. */
class MapMeasure {
  private tool: Tool = "none";
  private pts: L.LatLng[] = [];
  private finished = false;
  private layer = L.layerGroup();
  constructor(
    private map: L.Map,
    private m: NonNullable<ProjectView["manifest"]>,
  ) {
    this.layer.addTo(map);
    map.on("click", (e: L.LeafletMouseEvent) => this.click(e.latlng));
    map.on("dblclick", () => this.finish());
  }
  /** Map point → metres (east, north; y up). */
  private metric(ll: L.LatLng): M.P3 {
    const pt = this.map.project(ll, this.m.max_zoom);
    return [this.m.left + pt.x * this.m.gsd, this.m.top - pt.y * this.m.gsd, 0];
  }
  setTool(t: Tool) {
    this.tool = t;
    this.clear();
    if (t === "none") this.map.doubleClickZoom.enable();
    else this.map.doubleClickZoom.disable();
    this.map.getContainer().style.cursor = t === "none" ? "" : "crosshair";
  }
  clear() {
    this.pts = [];
    this.finished = false;
    this.draw();
    this.emit();
  }
  finish() {
    if (this.tool === "none" || this.pts.length < 2) return;
    if ((this.tool === "area" || this.tool === "volume") && this.pts.length < 3) return;
    this.finished = true;
    this.draw();
    this.emit();
  }
  private click(ll: L.LatLng) {
    if (this.tool === "none") return;
    if (this.finished) {
      this.pts = [];
      this.finished = false;
    }
    // The two clicks of a double-click land on the same spot: keep one.
    const last = this.pts[this.pts.length - 1];
    if (last && this.map.latLngToContainerPoint(last).distanceTo(this.map.latLngToContainerPoint(ll)) < 4) return;
    this.pts.push(ll);
    this.draw();
    this.emit();
  }
  private draw() {
    this.layer.clearLayers();
    const style = { color: "#ffd23f", weight: 2, interactive: false };
    const closed = (this.tool === "area" || this.tool === "volume") && this.pts.length > 2;
    if (closed) L.polygon(this.pts, { ...style, fillOpacity: 0.15 }).addTo(this.layer);
    else if (this.pts.length > 1) L.polyline(this.pts, style).addTo(this.layer);
    for (const p of this.pts) {
      L.circleMarker(p, { radius: 4, color: "#ffd23f", fillColor: "#ffd23f", fillOpacity: 1, interactive: false }).addTo(
        this.layer,
      );
    }
  }
  private emit() {
    showMeasure({ tool: this.tool, points: this.pts.map((p) => this.metric(p)), finished: this.finished }, false);
  }
}

function exportDialog(p: ProjectView) {
  const shown = shownLayer(p);
  const georef = shown.m?.epsg != null;
  const levels = shown.levels
    .slice(0, 6)
    .map(
      (l, i) => `<option value="${l.zoom}" ${i === 0 ? "selected" : ""}>
        ${i === 0 ? "Full" : `1/${2 ** i}`} — ${fmtPixels(l.width, l.height)}, ${fmtGsd(l.gsd, georef)}</option>`,
    )
    .join("");
  return `
  <div class="modal-back" data-action="close-modal">
    <div class="modal">
      <h3>Export ${shown.layer === "ortho" ? "true orthophoto" : "map"}</h3>
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
  mapMeasure = null;
  root.innerHTML = (p ? projectView(p) : homeView()) + (p && modal === "export" ? exportDialog(p) : "") + notice;
  if (p) {
    drawTrack(p);
    if (state.tab === "map" && shownLayer(p).m) mountMap(p);
    if (state.tab === "3d" && p.model3d) mount3d(p);
  }
}

function mount3d(p: ProjectView) {
  const el = document.getElementById("view3d");
  const m3 = p.model3d!;
  if (!el) return;
  const status = document.getElementById("v3d-status");
  const onStatus = (t: string) => {
    if (status) status.textContent = t;
  };
  const onMeasure = (e: MeasureEvent) => showMeasure(e, true);
  if (showSurface(p)) {
    viewer = mountViewer3D(el, {
      url: `${tileBase()}3d/model_view.glb?v=${m3.version}`,
      kind: "glb",
      offset: m3.mesh_offset ?? [0, 0, 0],
      onStatus,
      onMeasure,
    });
  } else {
    // The viewer loads a lighter copy; "Export point cloud" saves the full one.
    const file = m3.points ? "view.ply" : "sparse.ply";
    viewer = mountViewer3D(el, { url: `${tileBase()}3d/${file}?v=${m3.version}`, kind: "ply", onStatus, onMeasure });
  }
  viewer.setTool(state.tool);
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
  const { layer, m: lm } = shownLayer(p);
  const m = lm!;
  const prefix = layer === "ortho" ? "ortho/" : "";
  const el = document.getElementById("map");
  if (!el) return;
  map = L.map(el, { crs: L.CRS.Simple, minZoom: 0, maxZoom: m.max_zoom + 2, zoomSnap: 0.5, attributionControl: false });
  const sw = map.unproject([0, m.height], m.max_zoom);
  const ne = map.unproject([m.width, 0], m.max_zoom);
  const bounds = L.latLngBounds(sw, ne);
  L.tileLayer(`${tileBase()}${prefix}{z}/{x}/{y}.jpg?v=${m.version}`, {
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

  mapMeasure = new MapMeasure(map, m);
  mapMeasure.setTool(state.tool);
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
  const dupes = r.failed.filter(([, why]) => why.startsWith("skipped")).length;
  const unreadable = r.failed.length - dupes;
  notify(
    unreadable ? "error" : "ok",
    `Added ${r.added.toLocaleString()} photos` +
      (dupes ? ` · ${dupes} already in the project, skipped` : "") +
      (unreadable ? ` · ${unreadable} couldn't be read` : ""),
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
    case "layer-mosaic":
      state.layer = "mosaic";
      return render();
    case "layer-ortho":
      state.layer = "ortho";
      return render();
    case "tab-map":
      state.tab = "map";
      state.tool = "none";
      return render();
    case "tab-3d":
      state.tab = "3d";
      state.tool = "none";
      return render();
    case "v3d-surface":
      state.view3d = "surface";
      return render();
    case "v3d-points":
      state.view3d = "points";
      return render();
    case "measure-clear":
      viewer?.clearMeasure();
      mapMeasure?.clear();
      return;
    case "measure-finish":
      viewer?.finishMeasure();
      mapMeasure?.finish();
      return;
    case "export-model": {
      if (!p) return;
      const path = await save({
        title: "Save 3D model",
        defaultPath: `${p.name}.glb`,
        filters: [{ name: "3D model (glTF binary)", extensions: ["glb"] }],
      });
      if (!path) return;
      await api.exportModel(p.dir, path);
      return toast("ok", "3D model saved — opens in Blender, Windows 3D Viewer and most web 3D tools");
    }
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
      return toast("ok", "Point cloud saved — opens in CloudCompare, MeshLab or Blender");
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
      return api.exportMap(p.dir, format, shownLayer(p).layer, zoom, path);
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
  const toolBtn = t.closest<HTMLElement>("[data-tool]");
  if (toolBtn) return setTool(toolBtn.dataset.tool as Tool);
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
  if (t.id === "vol-level" || t.id === "vol-offset") {
    if (t.id === "vol-level") state.volLevel = t.value as typeof state.volLevel;
    else state.volOffset = Number(t.value) || 0;
    const row = document.getElementById("vol-offset-row");
    if (row) row.hidden = state.volLevel !== "custom";
    if (lastOutline) {
      const r = document.getElementById("vol-result");
      if (r) r.innerHTML = `<p class="hint">Measuring…</p>`;
      void runVolume();
    }
    return;
  }
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
