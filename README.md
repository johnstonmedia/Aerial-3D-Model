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
- **Local projects.** A project is a folder. Close it, reopen it, add more
  photos — earlier work is cached, so only the new photos are processed.
- **Exports:** GeoTIFF (any size), PNG, JPEG, each with a world file.

## Versions

| Build | Machines | Status |
|---|---|---|
| Windows ARM64 | Snapdragon / Copilot+ PCs | ✅ built by CI |
| Windows x64 | Intel / AMD | one line in `build.yml` |
| macOS Apple Silicon | M-series (16 GB+) | planned |
| macOS Lite | MacBook Neo (A18 Pro, 8 GB) | planned |

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
```

## Layout

```
crates/stitch-core   the engine (pure Rust — no native libraries)
crates/stitch-cli    command-line front end
app/                 desktop app (Tauri 2 + TypeScript + Leaflet)
```
