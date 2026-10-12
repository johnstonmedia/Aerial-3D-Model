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
- ⚠️ **Render memory** (measured on the 77 × 18 MP Aukerman set): Lite peaks at
  ~1.1 GB, Standard ~2–3 GB. Three things hold that, all learned the hard way
  (the first Lite run peaked at 5.4 GB):
  1. `contributors()` loads only photos that visibly contribute to a tile
     (~3 of the ~15 covering it in a 75%-overlap survey).
  2. On eviction the cache drops BOTH its strong ref and the decode cell (the
     `OnceLock` holds its own `Arc` — dropping only one freed nothing).
  3. Tiles are drawn in 4×4 blocks, not whole rows, so the working set fits.
  Re-measure with a per-stage RSS sampler if you touch render.rs.
- Caches are keyed by a file fingerprint (size + mtime + name) and the
  profile's work size, so changing profile recomputes features.
- Verified on OpenDroneMap's `odm_data_aukerman` (77 × 18 MP): all placed,
  2.7 cm GSD, EPSG:32617. UTM maths checked against PROJ to the millimetre.

## Flight planner (crates/stitch-core/src/plan.rs, app/src/planner.ts)
- Owner flies a **DJI Mavic Air 2** with **Litchi**. The Air 2 has no
  onboard waypoint engine: Litchi flies it by "virtual sticks" from the phone,
  so the mission stops if the RC link drops. DJI Fly has no waypoints for the
  Air 2, so a DJI WPML/KMZ export would be useless to this owner.
- Litchi CSV: 8 fixed columns, 15 action type/param pairs, 8 tail columns.
  Action codes used: 0 stay (ms), 1 take photo, 5 tilt camera. These are well
  attested. The `photo_distinterval` column (DistanceInterval mode) is NOT
  officially documented — that mode is labelled experimental in the UI.
- **Several photos per waypoint** (`shots`: tilt + direction relative to the
  line). `waypoint_actions()` emits rotate (4) / tilt (5) only when they change
  and settles (0) only before the first shot, so "nadir + 45° × 4 sides" fits
  Litchi's **15 actions per waypoint** (13 on a mission's first waypoint).
  ⚠️ Rotate-aircraft params are absolute headings in −180..180 (DJI SDK
  convention); not verified against Litchi docs — confirm on a test flight.
- The stitcher **skips oblique photos** (DJI XMP gimbal pitch > −75°,
  `OBLIQUE_PITCH_DEG`): they're for 3D and would smear the 2D map. Photos with
  no pitch metadata are kept.
- **Battery swaps:** Litchi for DJI (Mini 2 and older, incl. the Air 2) has no
  automatic resume — the pilot restarts a mission with "start from waypoint".
  So missions are split to fit ONE battery each, at line boundaries, and the
  checklist tells the pilot how to restart mid-mission.
- 99 waypoints per Litchi mission → plans split; `batteries` packs short
  missions onto one battery in order. Don't conflate missions with batteries.
- Obstacle avoidance during Litchi missions is unconfirmed (forum reports
  conflict). The app tells the pilot not to rely on it.
- Geometry: camera long side across the line, heading along it. Footprint =
  altitude × sensor / focal. Verified: Air 2 @ 80 m → 113.8 × 85.3 m, 2.84 cm/px.
- Basemaps (Esri World Imagery, OSM) need internet and are in the CSP;
  everything else in the planner works offline (KML import, plan, export).

## 3D reconstruction (crates/stitch-core/src/recon/)
- Pipeline: `sparse.rs` (GPS-aided SfM) → `dense.rs` (PatchMatch depth maps
  + consistency fusion) → `result/3d/{cameras.json,sparse.ply,dense.ply}`.
  Uses ALL photos, obliques included (the 2D map skips them).
- World frame: local ENU metres; origin UTM E/N in cameras.json and as a PLY
  comment. Heights are relative to take-off when every photo has DJI
  RelativeAltitude, else EXIF GPS altitude.
- Initial poses: GPS + DJI gimbal yaw/pitch; nadir photos with no gimbal data
  take yaw from the 2D solve (`yaw = −angle of the 2D similarity`).
- ⚠️ **The first BA round must be loose (200 working px).** Metadata misses the
  few degrees of in-flight tilt (~300 px at 100 m). A tight first threshold
  threw away most genuine matches and left a "floppy" block with the ground
  split into layers. Measured on Aukerman: 2.1 views/point and 935 stray
  points before; 3.6 views/point and 5 stray points after.
- ⚠️ After round 2, `pair_fits` drops photo pairs whose matches don't fit the
  camera geometry — repetitive grass fools F-RANSAC between photos that don't
  overlap, leaving 2-view points far below ground.
- 3D uses its own feature set (`profile_3d`: more keypoints, larger working
  size) — the 2D map's features were too sparse for good tracks.
- Nadir-only surveys can't separate focal length from flying height (both
  scale the image); obliques fix it. The BA test allows 2.5% for that reason.
- Dense: working size 640/960/1400 px (Lite/Standard/High); depth maps run one
  photo at a time (each uses every core) and are cached; fusion keeps depths
  confirmed by ≥2 neighbouring depth maps (≥1 if fewer than 3 neighbours) and
  voxelises to fit the profile's point budget.
- Measured on Aukerman (77 × 18 MP, Standard, 4-core sandbox): 4.27 M points,
  0.22 m voxel, peak RSS 783 MB, 17.5 min (almost all PatchMatch).
- The viewer loads `view.ply` (≤ 1.5 M points, strided from dense.ply);
  "Export point cloud" copies the full `dense. py`. 4 M points was too slow to
  load in a software-GL test browser — integrated GPUs deserve the same care.

## App notes (app/)
- Tiles reach the webview through a custom `tiles://` protocol (served as
  `http://tiles.localhost/` on Windows) reading the open project's folder — no
  filesystem scope is granted to the webview.
- Long jobs run on a thread and emit `progress` / `job-finished` events.

## CI
`.github/workflows/build.yml` cross-compiles the Windows ARM64 NSIS installer on
an x64 runner. Uncomment the x64 matrix entry to ship Intel/AMD too.
