// What differs between the desktop app and the web demo. The web build
// swaps this file for web/platform.ts (see vite.config.ts).
import L from "leaflet";

export const IS_WEB = false;

function tileBase() {
  // Custom protocols are served as http://<scheme>.localhost on Windows.
  return navigator.userAgent.includes("Windows") ? "http://tiles.localhost/" : "tiles://localhost/";
}

/** The map's tile layer. `prefix` is "" for the mosaic, "ortho/" for the true ortho. */
export function tileLayer(_dir: string, prefix: string, version: number, opts: L.TileLayerOptions): L.TileLayer {
  return L.tileLayer(`${tileBase()}${prefix}{z}/{x}/{y}.jpg?v=${version}`, opts);
}

/** URL of a 3D output (`3d/model_view.glb`, `3d/view.ply`, …). */
export async function assetUrl(_dir: string, rel: string, version: number): Promise<string> {
  return `${tileBase()}${rel}?v=${version}`;
}

export function homeExtras(): string {
  return "";
}

export function sampleLabel(): string | null {
  return null;
}

export async function samplePhotos(_onStatus: (t: string) => void): Promise<string[]> {
  return [];
}
