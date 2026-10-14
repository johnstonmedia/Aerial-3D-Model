# Aerial Stitch

Turn drone photos into one georeferenced map — **entirely on your own computer**.
No uploads, no accounts, no image limits.

- **Any number of photos.** The map is drawn tile by tile straight to disk, and
  decoded photos are held in a fixed memory budget, so RAM use stays flat
  whether you load 50 photos or 5,000.
- **Uses your drone's GPS.** Photos are only compared with their GPS
  neighbours (fast, even for big surveys), and the finished map is placed in
  real-world coordinates (UTM), so a GeoTIFF export drops straight into QGIS,
  ArcGIS or Google Earth.
- **Projects hold models.** A project is a folder for one site. Inside it
  you keep any number of **models** (a set of photos and the map / 3D model
  made from them — one per flight or visit, say) and the site's **flight
  plans**. Models in a project share their photo analysis, so the same photo
  in two models is only analysed once. Earlier work is cached, so adding
  photos later only processes what's new.
- **Exports:** GeoTIFF (any size), PNG, JPEG, each with a world file.
- **Your photos stay where they are.** A project only *references* photo
  files — nothing is copied. Adding the same photo twice (same file under
  another path, or a copy in another folder) is detected and skipped.
- **3D model.** Builds a coloured 3D point cloud of the site from the same
  photos — angled shots included — then a solid **textured surface**, with a
  built-in 3D viewer and PLY / GLB export (CloudCompare, MeshLab, Blender,
  Windows 3D Viewer).
- **True orthophoto.** The 2D map re-projected onto the 3D surface, so roofs
  sit over their footprints instead of leaning.
- **Measuring.** On the 2D map: distance and area. On the 3D model: distance
  along the surface (with height change and slope), the **height of a wall**,
  and the **true area of a sloping roof or wall face**. **Volume** on either:
  outline a pond and it tells you how much water fills it (m³ and litres) to
  its rim or any level you choose — or how much material is in a stockpile.
- **Flight planner.** Draw the area of focus (or import a KML/KMZ from Google
  Earth) and say what you have — **a number of batteries, or minutes of
  flying** — and it finds the **lowest height (sharpest photos) whose whole
  flight fits**, showing the trade-off at other heights. Optional extra legs
  for a complete 3D model:
  - an **edge ring** just outside the area, with photos angled 45° looking
    in at its border, set back exactly far enough that each photo is centred
    on the border;
  - an **overview**: the fewest straight-down photos from 120 m that cover
    the whole area (at least three), to tie the detailed photos together.

  Exports **Litchi CSV** missions (split at Litchi's 99-waypoint limit and
  per battery) plus a KML to check in Google Earth. Plans save into the
  project. Mavic Air 2 and Air 2S presets built in.

## Try it in your browser

**[johnstonmedia.github.io/Aerial-3D-Model](https://johnstonmedia.github.io/Aerial-3D-Model/)**
— the web demo. It is the same engine and the same app, compiled to
WebAssembly, running entirely in the browser tab: your photos are read from
your own disk and never uploaded. Use your own photos or the 16-photo sample
survey.

Compared with the desktop app it runs on one CPU core (several times slower),
keeps projects only until the tab is closed, and suits up to ~40 photos. The
3D model works too, but slowly.

## Versions

| Build | Machines | Status |
|---|---|---|
| Windows ARM64 | Snapdragon / Copilot+ PCs | ✅ built by CI |
| Windows x64 | Intel / AMD | one line in `build.yml` |
| macOS Apple Silicon | M-series (16 GB+) | planned |
| macOS Lite | MacBook Neo (A18 Pro, 8 GB) | planned |
| Web demo | any modern browser | ✅ published by CI (GitHub Pages) |

All share one codebase. The app picks a **memory profile** from installed RAM
(Lite < 12 GB, Standard 12–28 GB, High ≥ 28 GB); it can be overridden per
project in Settings.

## Download

Every push to `main` builds the installer: **Actions → Build → latest run →
Artifacts → `AerialStitch-windows-ARM64`**. Tagging a commit `v*` also attaches
it to a GitHub Release.

The installer is not code-signed yet, so Windows SmartScreen will warn on first
run: choose **More info → Run anyway**.

## How it works

1. **Features** — each photo is shrunk to a working size and reduced to a few
   thousand ORB-style keypoints (cached per photo).
2. **Pairs** — each photo is compared with its *k* nearest GPS neighbours.
3. **Matches** — RANSAC similarity fit per pair (cached per pair).
4. **Placement** — one sparse least-squares solve places every photo; the
   block is then fitted onto the GPS positions (UTM metres, north up).
5. **Render** — 512 px JPEG tiles plus a zoom pyramid; the viewer and
   exports read those tiles.

### Disk space

Measured on a 77-photo, 18 MP survey (518 MB of photos):

| What | Size | Notes |
|---|---|---|
| 2D map tiles | ~50 MB | |
| 3D results | ~125 MB | point cloud 63 MB, surface model 39 MB (+25 MB lighter viewer copy) |
| True orthophoto | ~20 MB | |
| Caches | ~230 MB | features, matches, depth maps — deletable; they only make re-runs fast |

Photos are never copied. The 3D stage makes shrunk working copies of each
photo while it runs and deletes them when it finishes.

This produces a flat 2D mosaic, not a survey-grade orthophoto: there's no
terrain model, so tall objects lean and hilly ground may show small seams.

## Building from source

Needs Rust (stable) and Node 22.

```sh
cd app
npm install
npx tauri dev                                              # run locally
npx tauri build --target aarch64-pc-windows-msvc --bundles nsis   # Windows ARM64 installer
```

Command line (handy for batch runs and testing):

```sh
cargo run --release -p stitch-cli -- process ./my-project ./photos
cargo run --release -p stitch-cli -- export  ./my-project ./map.tif
cargo run --release -p stitch-cli -- sparse ./my-project    # 3D: cameras + tie points
cargo run --release -p stitch-cli -- dense  ./my-project    # 3D: dense point cloud
cargo run --release -p stitch-cli -- plan ./out < plan.json  # flight plan → Litchi CSVs
```

## Layout

```
crates/stitch-core   the engine (pure Rust — no native libraries)
crates/stitch-cli    command-line front end
app/                 desktop app (Tauri 2 + TypeScript + Leaflet)
```
