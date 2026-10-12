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
  kind: "process" | "export";
  ok: boolean;
  message: string;
  detail: any;
}

export type ExportFormat = "geotiff" | "png" | "jpeg";

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
  exportMap: (dir: string, format: ExportFormat, zoom: number | null, path: string) =>
    invoke<void>("export_map", { dir, format, zoom, path }),
  cancel: () => invoke<void>("cancel_job"),
};
