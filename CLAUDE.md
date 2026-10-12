# Aerial Stitch — project notes

Local-first drone-photo stitcher. Read README.md first.

## Hard constraints
- **The engine stays pure Rust.** No OpenCV or other native libraries: that is
  what lets one codebase cross-compile to Windows ARM64 (Snapdragon), x64 and
  Apple Silicon. Adding a C dependency breaks the ARM64 cross-build on CI.
- **Memory must stay bounded.** Nothing may hold all photos, all features or
  the whole mosaic in memory. Sizes come from `Profile` (profile.rs). The
  MacBook Neo target has 8 GB, soldered.
- Targets, in order: Windows ARM64 (first, the owner's machine — Snapdragon,
  16 GB) → Windows x64 → macOS M-series (16 GB) → macOS Lite (MacBook Neo, A18
  Pro, 8 GB). No Intel Macs.

## Engine notes (crates/stitch-core)
- Coordinates: features are in working-image pixels (y down). The solver uses
  *centred, y-up* working coordinates so every transform is a proper
  similarity into a right-handed frame (UTM east/north). `Placed::full_to_frame`
  / `frame_to_full` are the only conversions — use them.
- ⚠️ The global solve **pins one photo per connected component** and only then
  fits the block to GPS. Putting GPS in as soft priors alone has a degenerate
  optimum (every photo shrinks to a point, zeroing all match residuals); the
  first real run did exactly that.
- Blending: weight `(dx·dy)^16` from each photo's centre → near-hard seams with
  a short soft transition. Low powers (4) visibly ghosted on real data.
- No-data is near-black: real pixels are lifted to max-channel ≥ 12, and
  max-channel < 6 is empty (`is_nodata`). Keep that gap — JPEG rings at edges.
- Caches are keyed by a file fingerprint (size + mtime + name) and the
  profile's work size, so changing profile recomputes features.
- Verified on OpenDroneMap's `odm_data_aukerman` (77 × 18 MP): all placed,
  2.7 cm GSD, EPSG:32617. UTM maths checked against PROJ to the millimetre.

## App notes (app/)
- Tiles reach the webview through a custom `tiles://` protocol (served as
  `http://tiles.localhost/` on Windows) reading the open project's folder — no
  filesystem scope is granted to the webview.
- Long jobs run on a thread and emit `progress` / `job-finished` events.

## CI
`.github/workflows/build.yml` cross-compiles the Windows ARM64 NSIS installer on
an x64 runner. Uncomment the x64 matrix entry to ship Intel/AMD too.
