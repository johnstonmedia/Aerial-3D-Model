// Measurement maths and formatting, shared by the 2D map and the 3D viewer.
// Points are [east, north, up] metres; 2D points simply have up = 0.
import type { VolumeResult } from "./api";

export type P3 = [number, number, number];

const sub = (a: P3, b: P3): P3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const len = (v: P3) => Math.hypot(v[0], v[1], v[2]);

export interface DistanceStats {
  /** Along the picked points, following slopes. */
  length: number;
  /** The same path seen from above. */
  horizontal: number;
  /** Last point minus first point. */
  rise: number;
  /** Straight line first → last, through the air. */
  direct: number;
  /** Average slope first → last, degrees (0 flat, 90 vertical). */
  slopeDeg: number;
}

export function distance(pts: P3[]): DistanceStats | null {
  if (pts.length < 2) return null;
  let length = 0;
  let horizontal = 0;
  for (let i = 1; i < pts.length; i++) {
    const d = sub(pts[i], pts[i - 1]);
    length += len(d);
    horizontal += Math.hypot(d[0], d[1]);
  }
  const d = sub(pts[pts.length - 1], pts[0]);
  const h = Math.hypot(d[0], d[1]);
  return {
    length,
    horizontal,
    rise: d[2],
    direct: len(d),
    slopeDeg: (Math.atan2(Math.abs(d[2]), h) * 180) / Math.PI,
  };
}

export interface HeightStats {
  /** Vertical difference — e.g. the height of a wall from its foot to its top. */
  vertical: number;
  /** Horizontal offset between the two points (a leaning or set-back top). */
  offset: number;
}

export function height(pts: P3[]): HeightStats | null {
  if (pts.length < 2) return null;
  const d = sub(pts[1], pts[0]);
  return { vertical: Math.abs(d[2]), offset: Math.hypot(d[0], d[1]) };
}

export interface AreaStats {
  /** The real area of the picked face — a sloping roof plane, a wall. */
  surface: number;
  /** Footprint seen from above (what a plan drawing shows). */
  plan: number;
  perimeter: number;
  /** Tilt of the best-fit plane from horizontal, degrees. */
  tiltDeg: number;
}

/** Newell's method: the vector area of a (possibly non-planar) polygon. Its
 *  length is the area of the best-fit plane; its z part is the plan area. */
export function area(pts: P3[]): AreaStats | null {
  if (pts.length < 3) return null;
  let nx = 0;
  let ny = 0;
  let nz = 0;
  let perimeter = 0;
  // Relative to the first point keeps precision with large coordinates.
  const o = pts[0];
  const q = pts.map((p) => sub(p, o));
  for (let i = 0; i < q.length; i++) {
    const a = q[i];
    const b = q[(i + 1) % q.length];
    nx += (a[1] - b[1]) * (a[2] + b[2]);
    ny += (a[2] - b[2]) * (a[0] + b[0]);
    nz += (a[0] - b[0]) * (a[1] + b[1]);
    perimeter += len(sub(b, a));
  }
  const surface = Math.hypot(nx, ny, nz) / 2;
  const plan = Math.abs(nz) / 2;
  const tiltDeg = surface > 0 ? (Math.acos(Math.min(1, plan / surface)) * 180) / Math.PI : 0;
  return { surface, plan, perimeter, tiltDeg };
}

export function fmtLen(m: number) {
  const a = Math.abs(m);
  if (a >= 1000) return `${(m / 1000).toFixed(3)} km`;
  if (a >= 10) return `${m.toFixed(1)} m`;
  if (a >= 1) return `${m.toFixed(2)} m`;
  return `${(m * 100).toFixed(1)} cm`;
}

export function fmtArea(m2: number) {
  if (m2 >= 10000) return `${m2.toLocaleString(undefined, { maximumFractionDigits: 0 })} m² (${(m2 / 10000).toFixed(2)} ha)`;
  return `${m2.toLocaleString(undefined, { maximumFractionDigits: m2 >= 100 ? 0 : 2 })} m²`;
}

export function fmtVolume(m3: number) {
  const litres = m3 * 1000;
  const l =
    litres >= 1e6
      ? `${(litres / 1e6).toLocaleString(undefined, { maximumFractionDigits: 2 })} ML`
      : `${litres.toLocaleString(undefined, { maximumFractionDigits: 0 })} L`;
  return `${m3.toLocaleString(undefined, { maximumFractionDigits: m3 >= 100 ? 0 : 2 })} m³ · ${l}`;
}

const row = (k: string, v: string, strong = false) =>
  `<div class="mrow${strong ? " strong" : ""}"><span>${k}</span><b>${v}</b></div>`;

export function distanceHtml(s: DistanceStats, has3d: boolean) {
  if (!has3d) return row("Distance", fmtLen(s.horizontal), true);
  return (
    row("Along the surface", fmtLen(s.length), true) +
    row("Horizontal", fmtLen(s.horizontal)) +
    row("Height change", `${s.rise >= 0 ? "+" : "−"}${fmtLen(Math.abs(s.rise))}`) +
    row("Slope", `${s.slopeDeg.toFixed(1)}°`)
  );
}

export function heightHtml(s: HeightStats) {
  return row("Height", fmtLen(s.vertical), true) + row("Horizontal offset", fmtLen(s.offset));
}

export function areaHtml(s: AreaStats, has3d: boolean) {
  if (!has3d) return row("Area", fmtArea(s.plan), true) + row("Perimeter", fmtLen(s.perimeter));
  return (
    row("Surface area", fmtArea(s.surface), true) +
    row("Plan (from above)", fmtArea(s.plan)) +
    row("Tilt", `${s.tiltDeg.toFixed(1)}°`) +
    row("Perimeter", fmtLen(s.perimeter))
  );
}

export function volumeHtml(r: VolumeResult) {
  const depth = r.level_z - r.min_z;
  let out =
    row("To fill (below the level)", fmtVolume(r.fill_m3), true) +
    row("Above the level", fmtVolume(r.cut_m3)) +
    row("Deepest point below level", fmtLen(Math.max(0, depth))) +
    row("Highest point above level", fmtLen(Math.max(0, r.max_z - r.level_z))) +
    row("Plan area", fmtArea(r.area_m2)) +
    row("Surface area", fmtArea(r.surface_area_m2));
  if (r.missing_fraction > 0.02) {
    out += `<p class="hint">${Math.round(r.missing_fraction * 100)}% of the outline had no surface data; those
      cells were assumed to sit at the average height, so treat the volume as approximate.</p>`;
  }
  return out;
}
