// Flight planner screen: draw the survey area, tune height/overlap, preview
// the lawnmower grid, export Litchi CSV missions.
import L from "leaflet";
import { open, save } from "@tauri-apps/plugin-dialog";
import { Goal, LonLat, Plan, PlanInput, Shot, planApi } from "./api";
import { askConfirm } from "./dialogs";

const esc = (s: string) =>
  s.replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]!);

const SHOT_PRESETS: { id: string; label: string; shots: Shot[] }[] = [
  { id: "nadir", label: "Straight down — 2D map", shots: [{ pitch_deg: -90, yaw_offset_deg: 0 }] },
  {
    id: "nadir+fwd45",
    label: "Straight down + 45° forward",
    shots: [{ pitch_deg: -90, yaw_offset_deg: 0 }, { pitch_deg: -45, yaw_offset_deg: 0 }],
  },
  {
    id: "nadir+4x45",
    label: "Straight down + 45° on all 4 sides — 3D",
    shots: [
      { pitch_deg: -90, yaw_offset_deg: 0 },
      { pitch_deg: -45, yaw_offset_deg: 0 },
      { pitch_deg: -45, yaw_offset_deg: 90 },
      { pitch_deg: -45, yaw_offset_deg: 180 },
      { pitch_deg: -45, yaw_offset_deg: 270 },
    ],
  },
  {
    id: "4x45",
    label: "45° on all 4 sides only — 3D",
    shots: [0, 90, 180, 270].map((y) => ({ pitch_deg: -45, yaw_offset_deg: y })),
  },
];
const PITCHES = [-90, -80, -75, -60, -45, -30, -15, 0];
const DIRECTIONS: [number, string][] = [
  [0, "Forward"], [45, "Forward-right"], [90, "Right"], [135, "Back-right"],
  [180, "Back"], [225, "Back-left"], [270, "Left"], [315, "Forward-left"],
];
const pitchLabel = (p: number) => (p === -90 ? "Straight down" : p === 0 ? "Horizon (0°)" : `${-p}° down`);
const samePreset = (a: Shot[], b: Shot[]) =>
  a.length === b.length && a.every((s, i) => s.pitch_deg === b[i].pitch_deg && s.yaw_offset_deg === b[i].yaw_offset_deg);

const MISSION_COLORS = ["#36e0c0", "#ffb547", "#7aa2ff", "#ff7ab6", "#b6ff7a", "#c792ea"];

type Notify = (kind: "ok" | "error" | "info", text: string) => void;

/** Set when the planner is opened from a project: plans save into it. */
export interface PlannerContext {
  project: string;
  projectName: string;
  planId: string | null;
  input: PlanInput | null;
}

const RING_COLOR = "#ff9f43";
const OVERVIEW_COLOR = "#ffffff";

export async function mountPlanner(root: HTMLElement, onBack: () => void, notify: Notify, ctx: PlannerContext | null = null) {
  const [defaults, cameras] = await planApi.defaults();
  // A new plan starts from the automatic planner: the best detail two
  // batteries allow, with the edge ring and overview for a complete model.
  let input: PlanInput = ctx?.input
    ? { ...defaults, ...ctx.input }
    : { ...defaults, goal: { kind: "batteries", count: 2 }, ring: true, overview: true };
  let planId = ctx?.planId ?? null;
  let dirty = false;
  let plan: Plan | null = null;
  let planError = "";

  root.innerHTML = `
  <div class="layout">
    <aside id="plan-side"></aside>
    <main>
      <div id="pmap"></div>
      <div class="map-info" id="pmap-info">Click the map to mark the corners of the area of focus.</div>
    </main>
  </div>`;

  const map = L.map("pmap", { zoomControl: true, attributionControl: true }).setView([-32.5, 148.5], 6);
  const imagery = L.tileLayer(
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { maxZoom: 20, maxNativeZoom: 19, attribution: "Imagery © Esri, Maxar, Earthstar Geographics" },
  ).addTo(map);
  const streets = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 20,
    maxNativeZoom: 19,
    attribution: "© OpenStreetMap contributors",
  });
  L.control.layers({ Satellite: imagery, Streets: streets }, {}, { position: "topright" }).addTo(map);

  const areaLayer = L.layerGroup().addTo(map);
  const routeLayer = L.layerGroup().addTo(map);

  // ───────── Area drawing ─────────
  const vertexIcon = L.divIcon({ className: "vertex", iconSize: [14, 14] });
  function drawArea() {
    areaLayer.clearLayers();
    const latlngs = input.boundary.map(([lon, lat]) => L.latLng(lat, lon));
    if (latlngs.length >= 2) {
      L.polygon(latlngs, { color: "#ffe066", weight: 2, fillOpacity: 0.08, interactive: false }).addTo(areaLayer);
    }
    latlngs.forEach((ll, i) => {
      const m = L.marker(ll, { icon: vertexIcon, draggable: true, title: "Drag to move · right-click to delete" });
      m.on("drag", (e) => {
        const p = (e.target as L.Marker).getLatLng();
        input.boundary[i] = [p.lng, p.lat];
        schedule();
      });
      m.on("dragend", () => drawArea());
      m.on("contextmenu", () => {
        input.boundary.splice(i, 1);
        drawArea();
        schedule();
      });
      m.addTo(areaLayer);
    });
  }
  map.on("click", (e: L.LeafletMouseEvent) => {
    input.boundary.push([e.latlng.lng, e.latlng.lat]);
    drawArea();
    schedule();
  });

  function fitArea() {
    if (input.boundary.length >= 2) {
      map.fitBounds(L.latLngBounds(input.boundary.map(([lon, lat]) => L.latLng(lat, lon))).pad(0.3));
    }
  }

  // ───────── Planning ─────────
  let timer: number | undefined;
  function schedule() {
    dirty = true;
    window.clearTimeout(timer);
    timer = window.setTimeout(replan, 150);
  }

  async function replan() {
    routeLayer.clearLayers();
    plan = null;
    planError = "";
    if (input.boundary.length >= 3) {
      try {
        plan = await planApi.plan(input);
      } catch (e) {
        planError = String(e);
      }
    }
    if (plan) drawRoute(plan);
    renderSide();
  }

  function drawRoute(p: Plan) {
    const manyPoints = p.stats.photos > 2500;
    p.missions.forEach((m, i) => {
      const color = MISSION_COLORS[i % MISSION_COLORS.length];
      const pts = m.waypoints.map((w) => L.latLng(w.lat, w.lon));
      L.polyline(pts, { color, weight: 2, opacity: 0.85, interactive: false }).addTo(routeLayer);
      for (const w of m.waypoints) {
        if (w.leg === "ring") {
          // Each ring photo looks in at the area: draw where it's facing.
          const reach = Math.max(8, p.stats.ring_offset_m * 0.6);
          const h = (w.heading * Math.PI) / 180;
          const dLat = (reach * Math.cos(h)) / 111320;
          const dLon = (reach * Math.sin(h)) / (111320 * Math.cos((w.lat * Math.PI) / 180));
          L.polyline([[w.lat, w.lon], [w.lat + dLat, w.lon + dLon]], { color: RING_COLOR, weight: 1.5, interactive: false }).addTo(routeLayer);
          L.circleMarker([w.lat, w.lon], { radius: 3, color: RING_COLOR, weight: 1, fillOpacity: 1, interactive: false }).addTo(routeLayer);
        } else if (w.leg === "overview") {
          L.circleMarker([w.lat, w.lon], {
            radius: 6, color: OVERVIEW_COLOR, weight: 2, fillColor: "#000", fillOpacity: 0.4, interactive: false,
          }).addTo(routeLayer);
        } else if (w.photo && !manyPoints) {
          L.circleMarker([w.lat, w.lon], { radius: 2, color, weight: 1, fillOpacity: 1, interactive: false }).addTo(routeLayer);
        }
      }
      const start = m.waypoints[0];
      if (start) {
        L.marker([start.lat, start.lon], {
          icon: L.divIcon({ className: "mission-start", html: `<span style="background:${color}">${i + 1}</span>`, iconSize: [22, 22] }),
          interactive: false,
        }).addTo(routeLayer);
      }
    });
  }

  // ───────── Sidebar ─────────
  const fmtTime = (s: number) => {
    const m = Math.round(s / 60);
    return m >= 60 ? `${Math.floor(m / 60)} h ${m % 60} min` : `${m} min`;
  };
  const fmtArea = (m2: number) => (m2 >= 10000 ? `${(m2 / 10000).toFixed(2)} ha` : `${Math.round(m2)} m²`);

  function shotsSection() {
    const preset = SHOT_PRESETS.find((p) => samePreset(p.shots, input.shots));
    const presetOpts = SHOT_PRESETS.map(
      (p) => `<option value="${p.id}" ${preset?.id === p.id ? "selected" : ""}>${esc(p.label)}</option>`,
    ).join("");
    const rows = input.shots
      .map(
        (sh, i) => `<div class="shot-row">
          <span class="shot-n">${i + 1}</span>
          <select data-shot="${i}" data-k="pitch_deg">${PITCHES.map(
            (p) => `<option value="${p}" ${p === sh.pitch_deg ? "selected" : ""}>${pitchLabel(p)}</option>`,
          ).join("")}</select>
          <select data-shot="${i}" data-k="yaw_offset_deg" ${sh.pitch_deg === -90 ? "disabled title=\"Direction doesn't matter straight down\"" : ""}>${DIRECTIONS.map(
            ([d, l]) => `<option value="${d}" ${d === sh.yaw_offset_deg ? "selected" : ""}>${l}</option>`,
          ).join("")}</select>
          <button class="icon" data-p="shot-del" data-i="${i}" ${input.shots.length < 2 ? "disabled" : ""} title="Remove">×</button>
        </div>`,
      )
      .join("");
    return `
      <div class="shots">
        <label>Photos at each waypoint
          <select data-f="shot_preset">${presetOpts}<option value="custom" ${preset ? "" : "selected"}>Custom</option></select>
        </label>
        ${rows}
        <button class="small-btn" data-p="shot-add" ${input.shots.length >= 8 ? "disabled" : ""}>+ Add photo</button>
        <p class="hint">Direction is relative to the way the drone is flying. Angled photos are for 3D models —
          the 2D map uses only the straight-down ones.</p>
      </div>`;
  }

  function goalSection() {
    const g = input.goal;
    const s = plan?.stats;
    const opts = plan?.options ?? [];
    const table = g.kind !== "manual" && opts.length
      ? `<table class="tradeoff">
          <tr><th>Height</th><th>Detail</th><th>Photos</th><th>Time</th><th>Batt.</th></tr>
          ${opts
            .map((o) => `<tr class="${o.altitude_m === s?.altitude_m ? "chosen" : ""} ${o.fits ? "" : "nofit"}" data-alt="${o.altitude_m}"
                title="Fly at ${o.altitude_m} m instead">
              <td>${o.altitude_m} m</td><td>${o.gsd_cm.toFixed(1)} cm</td><td>${o.photos.toLocaleString()}</td>
              <td>${fmtTime(o.duration_s)}</td><td>${o.batteries}</td></tr>`)
            .join("")}
        </table>
        <p class="hint">Lower is sharper but needs more photos and time. Greyed rows don't fit your budget. Click a row to fly at that height yourself.</p>`
      : "";
    return `
      <section class="panel">
        <h3>Goal</h3>
        <label>Plan the route for
          <select data-f="goal">
            <option value="batteries" ${g.kind === "batteries" ? "selected" : ""}>The best detail my batteries allow</option>
            <option value="time" ${g.kind === "time" ? "selected" : ""}>The best detail in the time I have</option>
            <option value="manual" ${g.kind === "manual" ? "selected" : ""}>A height I choose</option>
          </select>
        </label>
        ${g.kind === "batteries"
          ? `<label>Batteries <input type="number" min="1" max="30" value="${g.count}" data-f="goal_count" /></label>`
          : g.kind === "time"
            ? `<label>Minutes of flying <input type="number" min="3" max="600" value="${g.minutes}" data-f="goal_minutes" /></label>`
            : ""}
        ${g.kind !== "manual"
          ? `<label>Lowest safe height — <b>${input.min_alt_m} m</b>
               <input type="range" min="15" max="100" step="5" value="${input.min_alt_m}" data-f="min_alt_m" /></label>
             <p class="hint">Clear of the tallest trees, poles and buildings in or around the area.</p>`
          : ""}
        <label>Highest allowed — <b>${input.max_alt_m} m</b>
          <input type="range" min="40" max="120" step="5" value="${input.max_alt_m}" data-f="max_alt_m" /></label>
        ${s && g.kind !== "manual"
          ? `<div class="chosen-height">Flies at <b>${s.altitude_m.toFixed(0)} m</b> · ${s.gsd_cm.toFixed(1)} cm/px ·
              ${s.batteries} batter${s.batteries === 1 ? "y" : "ies"} · ${fmtTime(s.duration_s)}</div>`
          : ""}
        ${table}
        <label class="check"><input type="checkbox" data-f="ring" ${input.ring ? "checked" : ""} />
          <span><b>Edge ring</b> — a loop just outside the area with photos angled 45° in at its border,
          so the sides of the 3D model are complete.</span></label>
        <label class="check"><input type="checkbox" data-f="overview" ${input.overview ? "checked" : ""} />
          <span><b>Overview</b> — the fewest straight-down photos from ${input.max_alt_m} m that cover the whole
          area, to help piece the model together.</span></label>
      </section>`;
  }

  function renderSide() {
    const side = document.getElementById("plan-side");
    if (!side) return;
    const s = plan?.stats;
    const camOpts = cameras
      .map((c, i) => `<option value="${i}" ${c.name === input.camera.name ? "selected" : ""}>${esc(c.name)}</option>`)
      .join("");
    const pct = (v: number) => Math.round(v * 100);
    side.innerHTML = `
      <div class="side-head">
        <button class="icon" data-p="back" title="${ctx ? "Back to the project" : "Back"}">‹</button>
        <div>${ctx ? `<div class="dim small">${esc(ctx.projectName)}</div>` : ""}
          <div class="pname">Flight planner</div><div class="dim small">Plan a mapping flight for Litchi</div></div>
      </div>

      <section class="panel">
        <label>Go to coordinates <input data-f="goto" placeholder="-33.8568, 151.2153" /></label>
        <label>Plan name <input data-f="name" value="${esc(input.name)}" /></label>
        <div class="row">
          <button data-p="import">Import KML…</button>
          <button data-p="clear" ${input.boundary.length ? "" : "disabled"}>Clear area</button>
        </div>
        <p class="hint">${input.boundary.length < 3
          ? `Click the map to mark the corners of the area of focus (${input.boundary.length} so far), or import a boundary drawn in Google Earth.`
          : `${input.boundary.length} corners. Drag to adjust, right-click a corner to delete it.`}</p>
      </section>

      ${goalSection()}

      <section class="panel">
        <h3>Drone &amp; camera</h3>
        <label>Drone <select data-f="camera">${camOpts}</select></label>
        ${input.goal.kind === "manual"
          ? `<label>Height above take-off — <b>${input.altitude_m} m</b>${s ? ` · ${s.gsd_cm.toFixed(1)} cm/px` : ""}
              <input type="range" min="20" max="120" step="5" value="${input.altitude_m}" data-f="altitude_m" />
            </label>`
          : ""}
        <label>Front overlap — <b>${pct(input.front_overlap)}%</b>
          <input type="range" min="50" max="90" step="5" value="${pct(input.front_overlap)}" data-f="front_overlap" />
        </label>
        <label>Side overlap — <b>${pct(input.side_overlap)}%</b>
          <input type="range" min="40" max="85" step="5" value="${pct(input.side_overlap)}" data-f="side_overlap" />
        </label>
        <label>Speed — <b>${input.speed_ms} m/s</b>
          <input type="range" min="3" max="12" step="1" value="${input.speed_ms}" data-f="speed_ms" />
        </label>
        <label>Line direction
          <select data-f="direction_mode">
            <option value="auto" ${input.direction_deg == null ? "selected" : ""}>Automatic (along the longest side)</option>
            <option value="custom" ${input.direction_deg != null ? "selected" : ""}>Custom</option>
          </select>
        </label>
        ${input.direction_deg != null
          ? `<label>Bearing — <b>${input.direction_deg}°</b>
               <input type="range" min="0" max="175" step="5" value="${input.direction_deg}" data-f="direction_deg" /></label>`
          : ""}
        <label>Photos
          <select data-f="capture">
            <option value="photo_at_waypoint" ${input.capture === "photo_at_waypoint" ? "selected" : ""}>Stop and shoot at each point (most reliable)</option>
            <option value="distance_interval" ${input.capture === "distance_interval" ? "selected" : ""}>Shoot while flying (experimental)</option>
          </select>
        </label>
        ${input.capture === "photo_at_waypoint" ? shotsSection() : ""}
        <label>Height reference
          <select data-f="altitude_mode">
            <option value="relative_to_takeoff" ${input.altitude_mode === "relative_to_takeoff" ? "selected" : ""}>Above take-off point (flat sites)</option>
            <option value="above_ground" ${input.altitude_mode === "above_ground" ? "selected" : ""}>Above ground (Litchi follows terrain)</option>
          </select>
        </label>
        <label>Usable minutes per battery — <b>${input.battery_minutes}</b>
          <input type="range" min="8" max="30" step="1" value="${input.battery_minutes}" data-f="battery_minutes" />
        </label>
      </section>

      ${s ? `
      <section class="panel">
        <h3>Plan</h3>
        <div class="stats">
          <div><span>Area</span><b>${fmtArea(s.area_m2)}</b></div>
          <div><span>Height</span><b>${s.altitude_m.toFixed(0)} m</b></div>
          <div><span>Detail</span><b>${s.gsd_cm.toFixed(1)} cm/px</b></div>
          <div><span>Photos</span><b>${s.photos.toLocaleString()}</b></div>
          <div><span>Flight time</span><b>${fmtTime(s.duration_s)}</b></div>
          <div><span>Missions</span><b>${s.missions}</b></div>
          <div><span>Batteries</span><b>≈ ${s.batteries}</b></div>
          <div><span>Lines</span><b>${s.lines}</b></div>
          <div><span>Line gap</span><b>${s.line_spacing_m.toFixed(0)} m</b></div>
          <div><span>Photo gap</span><b>${s.photo_spacing_m.toFixed(0)} m</b></div>
        </div>
        ${s.ring_photos || s.overview_photos
          ? `<p class="hint">${s.grid_photos.toLocaleString()} grid photos${
              s.ring_photos ? ` · <span style="color:${RING_COLOR}">${s.ring_photos} edge-ring photos</span> ${s.ring_offset_m.toFixed(0)} m outside the area` : ""
            }${s.overview_photos ? ` · ${s.overview_photos} overview photos from ${s.overview_alt_m.toFixed(0)} m` : ""}</p>`
          : ""}
        ${plan!.missions.length > 1 && input.capture === "photo_at_waypoint"
          ? `<p class="hint">Litchi allows 99 waypoints per mission, so this plan is split into ${s.missions}. Short ones can be flown back-to-back on one battery.</p>`
          : ""}
        ${plan!.missions.length > 1
          ? `<ol class="missions">${plan!.missions
              .map((m, i) => `<li><span class="swatch" style="background:${MISSION_COLORS[i % MISSION_COLORS.length]}"></span>
                  ${m.photos} photos · ${fmtTime(m.duration_s)} · ${m.waypoints.length} waypoints</li>`)
              .join("")}</ol>`
          : ""}
      </section>` : ""}

      ${plan?.warnings.length ? `<section class="panel warnbox">${plan.warnings.map((w) => `<p>⚠ ${esc(w)}</p>`).join("")}</section>` : ""}
      ${planError ? `<section class="panel warnbox"><p>${esc(planError)}</p></section>` : ""}

      <section class="panel">
        <button class="primary wide" data-p="export" ${plan ? "" : "disabled"}>Export for Litchi…</button>
        ${ctx
          ? `<button class="wide" data-p="save-project" ${input.boundary.length ? "" : "disabled"}>${planId ? "Save changes" : "Save to project"}</button>`
          : ""}
        <div class="row">
          <button data-p="save" ${input.boundary.length ? "" : "disabled"}>Save plan file…</button>
          <button data-p="load">Open plan file…</button>
        </div>
        <details class="howto">
          <summary>How to fly it</summary>
          <ol>
            <li>Open <b>Litchi Mission Hub</b> (flylitchi.com/hub) and import each CSV.</li>
            <li>In the mission settings choose <b>Heading: Custom</b>, <b>Path: Straight lines</b> and
                <b>Finish: Return to home</b>. Save, then open it in the Litchi app.</li>
            <li>Set the camera to <b>12 MP</b>, auto-exposure, and a fast shutter (1/500 s or quicker in daylight).</li>
            <li>Take off from inside or near the area. Keep the drone <b>in sight</b> and the controller connected —
                on the Mavic Air 2, Litchi steers from your phone and the mission stops if the signal drops.</li>
            <li><b>Don't rely on obstacle sensors</b> during the mission. Fly well above trees, towers and power lines.</li>
            <li>Each mission is sized to fit one battery. Swap batteries <b>between</b> missions, then fly the next
                numbered one.</li>
            <li>If a battery runs low part-way, let it return home, note the <b>last waypoint it reached</b>, swap
                the battery, reopen the same mission and choose <b>start from waypoint</b> — that number + 1.
                Litchi for the Mavic Air 2 doesn't resume automatically.</li>
            <li>Then make a new model in the project and drop all the photos into it.</li>
          </ol>
        </details>
      </section>`;
  }

  // ───────── Events ─────────
  root.addEventListener("input", (e) => {
    const t = e.target as HTMLInputElement;
    const f = t.dataset.f;
    if (!f || t.type !== "range") return;
    const v = Number(t.value);
    if (f === "front_overlap" || f === "side_overlap") input[f] = v / 100;
    else if (f === "altitude_m" || f === "speed_ms" || f === "battery_minutes" || f === "min_alt_m" || f === "max_alt_m") input[f] = v;
    else if (f === "direction_deg") input.direction_deg = v;
    // Update the label live without rebuilding the slider being dragged.
    const b = t.parentElement?.querySelector("b");
    if (b) b.textContent = f.endsWith("overlap") ? `${t.value}%` : f === "direction_deg" ? `${t.value}°` : f === "altitude_m" || f === "min_alt_m" || f === "max_alt_m" ? `${t.value} m` : f === "speed_ms" ? `${t.value} m/s` : t.value;
    window.clearTimeout(timer);
    dirty = true;
    timer = window.setTimeout(async () => {
      routeLayer.clearLayers();
      try {
        plan = input.boundary.length >= 3 ? await planApi.plan(input) : null;
        planError = "";
      } catch (err) {
        plan = null;
        planError = String(err);
      }
      if (plan) drawRoute(plan);
    }, 120);
  });
  root.addEventListener("change", (e) => {
    const t = e.target as HTMLInputElement | HTMLSelectElement;
    if (t.dataset.shot != null) {
      const i = Number(t.dataset.shot);
      const k = t.dataset.k as keyof Shot;
      input.shots = input.shots.map((sh, j) => (j === i ? { ...sh, [k]: Number(t.value) } : sh));
      if (k === "pitch_deg" && Number(t.value) === -90) input.shots[i].yaw_offset_deg = 0;
      return void replan();
    }
    const f = t.dataset.f;
    if (!f) return;
    if (f === "shot_preset") {
      const p = SHOT_PRESETS.find((x) => x.id === t.value);
      if (p) input.shots = p.shots.map((x) => ({ ...x }));
      return void replan();
    }
    if (f === "goto") {
      const m = t.value.match(/(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)/);
      if (m) map.setView([Number(m[1]), Number(m[2])], 17);
      else notify("error", "Enter coordinates as: latitude, longitude");
      return;
    }
    if (f === "goal") {
      const g: Goal =
        t.value === "batteries" ? { kind: "batteries", count: 2 }
        : t.value === "time" ? { kind: "time", minutes: 30 }
        : { kind: "manual" };
      // Keep the height the planner chose when switching to manual.
      if (g.kind === "manual" && plan) input.altitude_m = Math.round(plan.stats.altitude_m);
      input.goal = g;
    } else if (f === "goal_count") input.goal = { kind: "batteries", count: Math.max(1, Math.round(Number(t.value) || 1)) };
    else if (f === "goal_minutes") input.goal = { kind: "time", minutes: Math.max(3, Number(t.value) || 30) };
    else if (f === "ring" || f === "overview") input[f] = (t as HTMLInputElement).checked;
    else if (f === "name") input.name = t.value || "Survey";
    else if (f === "camera") input.camera = cameras[Number(t.value)];
    else if (f === "direction_mode") input.direction_deg = t.value === "custom" ? 0 : null;
    else if (f === "capture") input.capture = t.value as PlanInput["capture"];
    else if (f === "altitude_mode") input.altitude_mode = t.value as PlanInput["altitude_mode"];
    if (input.min_alt_m > input.max_alt_m) input.min_alt_m = input.max_alt_m;
    dirty = true;
    void replan(); // sliders also land here on release, refreshing the stats
  });

  root.addEventListener("click", async (e) => {
    const row = (e.target as HTMLElement).closest<HTMLElement>("[data-alt]");
    if (row) {
      input.goal = { kind: "manual" };
      input.altitude_m = Number(row.dataset.alt);
      dirty = true;
      return void replan();
    }
    const b = (e.target as HTMLElement).closest<HTMLElement>("[data-p]");
    if (!b) return;
    try {
      switch (b.dataset.p) {
        case "back":
          if (
            ctx && dirty && input.boundary.length &&
            !(await askConfirm("Leave without saving?", "Changes to this plan haven't been saved to the project.", "Leave"))
          ) return;
          map.remove();
          return onBack();
        case "save-project": {
          if (!ctx) return;
          planId = await planApi.saveToProject(ctx.project, planId, input);
          dirty = false;
          renderSide();
          return notify("ok", `Saved “${input.name}” to ${ctx.projectName}`);
        }
        case "shot-add": {
          const last = input.shots[input.shots.length - 1];
          const next = last && last.pitch_deg !== -90
            ? { pitch_deg: last.pitch_deg, yaw_offset_deg: (last.yaw_offset_deg + 90) % 360 }
            : { pitch_deg: -45, yaw_offset_deg: 0 };
          input.shots = [...input.shots, next];
          return void replan();
        }
        case "shot-del":
          input.shots = input.shots.filter((_, j) => j !== Number(b.dataset.i));
          return void replan();
        case "clear":
          input.boundary = [];
          drawArea();
          return void replan();
        case "import": {
          const f = await open({ title: "Import a boundary", filters: [{ name: "Google Earth", extensions: ["kml", "kmz"] }] });
          if (typeof f !== "string") return;
          input.boundary = (await planApi.importBoundary(f)) as LonLat[];
          drawArea();
          fitArea();
          await replan();
          return notify("ok", `Imported an area with ${input.boundary.length} corners`);
        }
        case "save": {
          const f = await save({ title: "Save plan", defaultPath: `${input.name}.aerialplan.json`, filters: [{ name: "Plan", extensions: ["json"] }] });
          if (!f) return;
          await planApi.save(f, input);
          return notify("ok", "Plan saved");
        }
        case "load": {
          const f = await open({ title: "Open plan", filters: [{ name: "Plan", extensions: ["json"] }] });
          if (typeof f !== "string") return;
          input = await planApi.load(f);
          if (!input.shots?.length) input.shots = [{ pitch_deg: -90, yaw_offset_deg: 0 }];
          drawArea();
          fitArea();
          return void replan();
        }
        case "export": {
          const dir = await open({ directory: true, title: "Choose a folder for the Litchi files" });
          if (typeof dir !== "string") return;
          const files = await planApi.exportLitchi(input, dir);
          const csvs = files.filter((x) => x.endsWith(".csv")).length;
          return notify("ok", `Saved ${csvs} Litchi mission${csvs > 1 ? "s" : ""} and a Google Earth preview (KML)`);
        }
      }
    } catch (err) {
      notify("error", String(err));
    }
  });

  renderSide();
  drawArea();
  if (input.boundary.length >= 3) {
    fitArea();
    void replan();
  }
  const Locate = L.Control.extend({
    onAdd() {
      const d = L.DomUtil.create("div", "leaflet-bar");
      d.innerHTML = `<a href="#" title="Go to my location" role="button">⌖</a>`;
      L.DomEvent.on(d, "click", (ev) => {
        L.DomEvent.preventDefault(ev);
        L.DomEvent.stopPropagation(ev);
        map.locate({ setView: true, maxZoom: 17 });
      });
      return d;
    },
  });
  new Locate({ position: "topleft" }).addTo(map);
  map.on("locationerror", () => notify("info", "Couldn't get your location — pan and zoom to the site instead."));
}

