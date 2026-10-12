// Typed wrappers over the Rust commands in src-tauri/src/lib.rs.
import { invoke } from "@tauri-apps/api/core";

export type ProfileKind = "lite" | "standard" | "high";

export interface Profile {
  kind: ProfileKind;
  work_size: number;
  max_features: number;
  render_cache_bytes: number;
  threads: number;
}

export interface SystemInfo {
  os: string;
  arch: string;
  ram_gb: number;
  cores: number;
  profile: Profile;
}

export interface Settings {
  output_scale: number;
  profile: ProfileKind | null;
  neighbours: number;
}

export interface PhotoView {
  id: number;
  name: string;
  path: string;
  lat: number | null;
  lon: number | null;
  yaw: number | null;
  width: number;
  height: number;
  missing: boolean;
}

export interface Manifest {
  width: number;
  height: number;
  tile_size: number;
  max_zoom: number;
  gsd: number;
  left: number;
  top: number;
  epsg: number | null;
  utm_left: number | null;
  utm_top: number | null;
  photos: number;
  version: number;
}

export interface LevelInfo {
  zoom: number;
  width: number;
  height: number;
  gsd: number;
}

export interface ProjectView {
  dir: string;
  name: string;
  photos: PhotoView[];
  settings: Settings;
  manifest: Manifest | null;
  levels: LevelInfo[];
  /** True orthophoto (built with the 3D model). */
  ortho: Manifest | null;
  ortho_levels: LevelInfo[];
  model3d: { points: number; sparse_points: number; version: number } | null;
}

export interface RecentProject {
  dir: string;
  name: string;
  photos: number;
}

export interface Progress {
  stage: string;
  done: number;
  total: number;
  message: string;
}

export interface JobFinished {
  kind: "process" | "export" | "3d";
  ok: boolean;
  message: string;
  detail: any;
}

export type ExportFormat = "geotiff" | "png" | "jpeg";
export type MapLayer = "mosaic" | "ortho";

export const api = {
  systemInfo: () => invoke<SystemInfo>("system_info"),
  recentProjects: () => invoke<RecentProject[]>("recent_projects"),
  createProject: (dir: string, name: string) => invoke<ProjectView>("create_project", { dir, name }),
  openProject: (dir: string) => invoke<ProjectView>("open_project", { dir }),
  addPhotos: (dir: string, paths: string[]) =>
    invoke<{ added: number; failed: [string, string][]; project: ProjectView }>("add_photos", { dir, paths }),
  removePhotos: (dir: string, ids: number[]) => invoke<ProjectView>("remove_photos", { dir, ids }),
  saveSettings: (dir: string, settings: Settings) => invoke<ProjectView>("save_settings", { dir, settings }),
  process: (dir: string) => invoke<void>("process", { dir }),
  exportMap: (dir: string, format: ExportFormat, layer: MapLayer, zoom: number | null, path: string) =>
    invoke<void>("export_map", { dir, format, layer, zoom, path }),
  cancel: () => invoke<void>("cancel_job"),
  build3d: (dir: string) => invoke<void>("build_3d", { dir }),
  exportCloud: (dir: string, path: string) => invoke<string>("export_cloud", { dir, path }),
};

// ─────────────── Flight planner ───────────────

export interface Camera {
  name: string;
  sensor_w_mm: number;
  sensor_h_mm: number;
  focal_mm: number;
  image_w_px: number;
  image_h_px: number;
  min_photo_interval_s: number;
}

export type LonLat = [number, number];

export interface PlanInput {
  name: string;
  boundary: LonLat[];
  camera: Camera;
  altitude_m: number;
  front_overlap: number;
  side_overlap: number;
  speed_ms: number;
  direction_deg: number | null;
  capture: "photo_at_waypoint" | "distance_interval";
  altitude_mode: "relative_to_takeoff" | "above_ground";
  battery_minutes: number;
  overshoot: number;
  shots: Shot[];
}

/** One photo at each waypoint: tilt (-90 = straight down) and direction
 *  relative to the flight line (0 forward, 90 right, 180 back, 270 left). */
export interface Shot {
  pitch_deg: number;
  yaw_offset_deg: number;
}

export interface Waypoint {
  lon: number;
  lat: number;
  heading: number;
  photo: boolean;
  photo_dist_m: number;
  first: boolean;
}

export interface Mission {
  waypoints: Waypoint[];
  lines: number;
  photos: number;
  distance_m: number;
  duration_s: number;
}

export interface Plan {
  missions: Mission[];
  stats: {
    area_m2: number;
    gsd_cm: number;
    footprint_w_m: number;
    footprint_h_m: number;
    line_spacing_m: number;
    photo_spacing_m: number;
    speed_ms: number;
    direction_deg: number;
    lines: number;
    photos: number;
    distance_m: number;
    duration_s: number;
    missions: number;
    batteries: number;
  };
  warnings: string[];
}

export const planApi = {
  defaults: () => invoke<[PlanInput, Camera[]]>("plan_defaults"),
  plan: (input: PlanInput) => invoke<Plan>("plan_flight", { input }),
  exportLitchi: (input: PlanInput, dir: string) => invoke<string[]>("export_litchi", { input, dir }),
  importBoundary: (path: string) => invoke<LonLat[]>("import_boundary", { path }),
  save: (path: string, input: PlanInput) => invoke<void>("save_plan", { path, input }),
  load: (path: string) => invoke<PlanInput>("load_plan", { path }),
};
