// Web demo side of src/platform.ts: tiles and 3D files come out of the
// in-browser file system instead of the desktop app's tiles:// protocol.
import L from "leaflet";
import { readFile, ingest } from "./backend";

export const IS_WEB = true;

const REPO = "https://github.com/johnstonmedia/Aerial-3D-Model";

/** Tiles are read out of the demo's file system as they come into view. */
const DemoTiles = L.TileLayer.extend({
  createTile(this: any, coords: L.Coords, done: L.DoneCallback) {
    const img = document.createElement("img");
    img.alt = "";
    const path = `${this._dir}/result/${this._prefix ? "ortho/tiles" : "tiles"}/${coords.z}/${coords.x}/${coords.y}.jpg`;
    readFile(path).then(
      (blob) => {
        if (!blob) return done(new Error("missing tile"), img);
        const url = URL.createObjectURL(blob);
        img.onload = () => {
          URL.revokeObjectURL(url);
          done(undefined, img);
        };
        img.onerror = () => done(new Error("bad tile"), img);
        img.src = url;
      },
      (e) => done(e, img),
    );
    return img;
  },
});

export function tileLayer(dir: string, prefix: string, _version: number, opts: L.TileLayerOptions): L.TileLayer {
  const layer = new (DemoTiles as any)("", opts);
  layer._dir = dir;
  layer._prefix = prefix;
  return layer;
}

const urls = new Map<string, string>();

export async function assetUrl(dir: string, rel: string, version: number): Promise<string> {
  const key = `${dir}/result/${rel}`;
  const cached = urls.get(key);
  if (cached?.startsWith(`${version}|`)) return cached.slice(cached.indexOf("|") + 1);
  const blob = await readFile(key);
  if (!blob) throw new Error(`${rel} is missing`);
  const url = URL.createObjectURL(blob);
  if (cached) URL.revokeObjectURL(cached.slice(cached.indexOf("|") + 1));
  urls.set(key, `${version}|${url}`);
  return url;
}

export function homeExtras(): string {
  return `<div class="demo-note">
    <b>Web demo.</b> The same engine as the desktop app, running entirely in this browser tab —
    your photos are read from your own disk and never uploaded. It uses one CPU core, so it is
    several times slower, and projects disappear when you close the tab. Best with up to ~40 photos.
    <a href="${REPO}#download" target="_blank" rel="noopener">Get the desktop app</a> for big surveys,
    saved projects and full speed.
  </div>`;
}

// A public test survey (OpenDroneMap's Aukerman Park set), fetched straight
// from where it is published — a compact block of 16 neighbouring photos.
const SAMPLE_BASE = "https://raw.githubusercontent.com/OpenDroneMap/odm_data_aukerman/master/images/";
const SAMPLE = [
  "DSC00251", "DSC00252", "DSC00259", "DSC00260", "DSC00263", "DSC00264", "DSC00265", "DSC00287",
  "DSC00295", "DSC00296", "DSC00303", "DSC00304", "DSC00305", "DSC00307", "DSC00308", "DSC00309",
];

export function sampleLabel(): string | null {
  return "Try sample photos (16 photos, ~125 MB)";
}

export async function samplePhotos(onStatus: (t: string) => void): Promise<string[]> {
  const files: { path: string; file: Blob }[] = [];
  let done = 0;
  onStatus(`Downloading sample photos… 0 / ${SAMPLE.length}`);
  // A few at a time: quick, without hammering the host.
  const queue = [...SAMPLE];
  const worker = async () => {
    for (let name = queue.shift(); name; name = queue.shift()) {
      const r = await fetch(`${SAMPLE_BASE}${name}.JPG`);
      if (!r.ok) throw new Error(`couldn't download sample photo ${name} (${r.status})`);
      files.push({ path: `/w/photos/sample/${name}.JPG`, file: await r.blob() });
      onStatus(`Downloading sample photos… ${++done} / ${SAMPLE.length}`);
    }
  };
  await Promise.all([worker(), worker(), worker(), worker()]);
  await ingest(files);
  onStatus("Sample photos ready — OpenDroneMap's Aukerman Park survey");
  return ["/w/photos/sample"];
}
