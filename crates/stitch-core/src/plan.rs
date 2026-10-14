//! Survey flight planning: an area boundary + camera + height in, a
//! lawnmower grid of waypoints out, split into Litchi-sized missions.
//!
//! Geometry is done in UTM metres (same maths as the mosaic), then converted
//! back to lat/lon for export. The camera's long side runs across the flight
//! line and the drone faces along it, which is how mapping apps conventionally
//! fly: fewer, longer lines.

use crate::geo::{from_utm, to_utm, UtmZone};
use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};

/// Litchi refuses missions with more than 99 waypoints.
pub const LITCHI_MAX_WAYPOINTS: usize = 99;
/// Australian (CASA) and most other standard-category limits: 120 m AGL.
pub const LEGAL_MAX_ALT_M: f64 = 120.0;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Camera {
    pub name: String,
    pub sensor_w_mm: f64,
    pub sensor_h_mm: f64,
    pub focal_mm: f64,
    pub image_w_px: u32,
    pub image_h_px: u32,
    /// Shortest reliable time between photos, seconds.
    pub min_photo_interval_s: f64,
}

pub fn camera_presets() -> Vec<Camera> {
    vec![
        // 1/2" CMOS, 24 mm-equivalent lens. 12 MP is the recommended mode:
        // photos save in about 2 s, and the 48 MP mode is interpolated anyway.
        Camera {
            name: "DJI Mavic Air 2 — 12 MP (recommended)".into(),
            sensor_w_mm: 6.4,
            sensor_h_mm: 4.8,
            focal_mm: 4.5,
            image_w_px: 4000,
            image_h_px: 3000,
            min_photo_interval_s: 2.0,
        },
        Camera {
            name: "DJI Mavic Air 2 — 48 MP".into(),
            sensor_w_mm: 6.4,
            sensor_h_mm: 4.8,
            focal_mm: 4.5,
            image_w_px: 8000,
            image_h_px: 6000,
            min_photo_interval_s: 5.0,
        },
        Camera {
            name: "DJI Air 2S — 20 MP".into(),
            sensor_w_mm: 13.2,
            sensor_h_mm: 8.8,
            focal_mm: 8.38,
            image_w_px: 5472,
            image_h_px: 3648,
            min_photo_interval_s: 2.0,
        },
    ]
}

/// One photo taken at each waypoint. Several shots per waypoint give the
/// angled (oblique) views a 3D model needs; the 2D map uses only the
/// straight-down ones.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Shot {
    /// Camera tilt: -90 straight down, 0 at the horizon.
    pub pitch_deg: f64,
    /// Direction relative to the flight line, clockwise: 0 forward,
    /// 90 right, 180 back, 270 left.
    pub yaw_offset_deg: f64,
}

pub fn nadir_only() -> Vec<Shot> {
    vec![Shot {
        pitch_deg: -90.0,
        yaw_offset_deg: 0.0,
    }]
}

/// Litchi allows 15 actions per waypoint.
pub const LITCHI_MAX_ACTIONS: usize = 15;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureMode {
    /// A waypoint per photo: stop, settle, shoot. Uses only Litchi's
    /// well-established "stay" and "take photo" actions — the reliable choice.
    PhotoAtWaypoint,
    /// Waypoints only at line ends, photos by distance while flying. Far fewer
    /// waypoints, but relies on Litchi's distance-interval column.
    DistanceInterval,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AltitudeMode {
    /// Height above the take-off point. Right for flat sites — take off from
    /// the site itself.
    RelativeToTakeoff,
    /// Litchi keeps this height above ground using its own elevation data.
    AboveGround,
}

/// What the planner optimises for.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Goal {
    /// Fly at `altitude_m`, whatever it costs.
    Manual,
    /// The sharpest photos (lowest height) that fit in this many batteries.
    Batteries { count: u32 },
    /// The sharpest photos that fit in this many minutes of flying.
    Time { minutes: f64 },
}

fn manual() -> Goal {
    Goal::Manual
}
fn default_min_alt() -> f64 {
    30.0
}
fn default_max_alt() -> f64 {
    LEGAL_MAX_ALT_M
}
fn default_ring_pitch() -> f64 {
    -45.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanInput {
    pub name: String,
    /// Boundary of the area of focus as (lon, lat) pairs, not closed.
    pub boundary: Vec<(f64, f64)>,
    pub camera: Camera,
    /// Height of the main grid (and the edge ring). With a budget goal this
    /// is chosen by the planner and only the result's `altitude_m` counts.
    pub altitude_m: f64,
    /// Overlap between consecutive photos along a line, 0–0.95.
    pub front_overlap: f64,
    /// Overlap between neighbouring lines, 0–0.95.
    pub side_overlap: f64,
    /// Cruise speed, m/s.
    pub speed_ms: f64,
    /// Flight-line direction in degrees clockwise from north; None = along
    /// the boundary's longest edge.
    pub direction_deg: Option<f64>,
    pub capture: CaptureMode,
    pub altitude_mode: AltitudeMode,
    /// Usable flying minutes per battery, keeping a reserve.
    pub battery_minutes: f64,
    /// Extend each line past the boundary by this many photo spacings so the
    /// edges get full overlap.
    pub overshoot: f64,
    /// Photos at each grid waypoint (PhotoAtWaypoint mode only).
    #[serde(default = "nadir_only")]
    pub shots: Vec<Shot>,
    #[serde(default = "manual")]
    pub goal: Goal,
    /// Lowest height the planner may choose: clear of trees, poles, buildings.
    #[serde(default = "default_min_alt")]
    pub min_alt_m: f64,
    /// Highest height the planner may choose (and the overview's height).
    #[serde(default = "default_max_alt")]
    pub max_alt_m: f64,
    /// A loop just outside the area with angled photos looking IN at its
    /// border — fills in the sides the grid only sees from above.
    #[serde(default)]
    pub ring: bool,
    /// Camera tilt for the ring photos (-45 = 45° down).
    #[serde(default = "default_ring_pitch")]
    pub ring_pitch_deg: f64,
    /// A few straight-down photos from `max_alt_m` covering the whole area —
    /// the fewest that do — to tie the detailed photos together.
    #[serde(default)]
    pub overview: bool,
}

impl Default for PlanInput {
    fn default() -> Self {
        Self {
            name: "Survey".into(),
            boundary: Vec::new(),
            camera: camera_presets().remove(0),
            altitude_m: 80.0,
            front_overlap: 0.75,
            side_overlap: 0.65,
            speed_ms: 8.0,
            direction_deg: None,
            capture: CaptureMode::PhotoAtWaypoint,
            altitude_mode: AltitudeMode::RelativeToTakeoff,
            battery_minutes: 18.0,
            overshoot: 1.0,
            shots: nadir_only(),
            goal: Goal::Manual,
            min_alt_m: default_min_alt(),
            max_alt_m: default_max_alt(),
            ring: false,
            ring_pitch_deg: default_ring_pitch(),
            overview: false,
        }
    }
}

/// Which part of the flight a waypoint belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Leg {
    /// The lawnmower grid over the area.
    Grid,
    /// The loop outside the area, camera angled in at its border.
    Ring,
    /// High straight-down photos of the whole area.
    Overview,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub lon: f64,
    pub lat: f64,
    /// Height of this waypoint (the altitude column in Litchi).
    pub alt_m: f64,
    /// Degrees clockwise from north, 0–360.
    pub heading: f64,
    /// Camera tilt held at this waypoint.
    pub pitch_deg: f64,
    pub leg: Leg,
    /// Take a photo at this waypoint (PhotoAtWaypoint mode).
    pub photo: bool,
    /// Photo every N metres on the leg starting here (DistanceInterval mode).
    pub photo_dist_m: f64,
    /// First waypoint of a mission: tilt the camera down and settle.
    pub first: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Mission {
    pub waypoints: Vec<Waypoint>,
    pub lines: usize,
    pub photos: usize,
    pub distance_m: f64,
    pub duration_s: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanStats {
    pub area_m2: f64,
    /// Height of the grid and ring.
    pub altitude_m: f64,
    pub gsd_cm: f64,
    pub footprint_w_m: f64,
    pub footprint_h_m: f64,
    pub line_spacing_m: f64,
    pub photo_spacing_m: f64,
    pub speed_ms: f64,
    pub direction_deg: f64,
    pub lines: usize,
    pub photos: usize,
    pub grid_photos: usize,
    pub ring_photos: usize,
    /// How far outside the area the ring flies.
    pub ring_offset_m: f64,
    pub overview_photos: usize,
    pub overview_alt_m: f64,
    pub distance_m: f64,
    pub duration_s: f64,
    pub missions: usize,
    /// Batteries needed when short missions share a battery (flown in order;
    /// a mission never spans two batteries).
    pub batteries: usize,
}

/// One height the planner tried, for showing the trade-off.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeightOption {
    pub altitude_m: f64,
    pub gsd_cm: f64,
    pub photos: usize,
    pub duration_s: f64,
    pub batteries: usize,
    pub fits: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Plan {
    pub missions: Vec<Mission>,
    pub stats: PlanStats,
    pub warnings: Vec<String>,
    /// Other heights and what they would cost (budget goals only).
    #[serde(default)]
    pub options: Vec<HeightOption>,
}

/// Seconds lost per photo stop: decelerate, settle, shoot, accelerate.
const STOP_OVERHEAD_S: f64 = 3.0;
/// Extra seconds for each additional shot at a waypoint (turn/tilt + shoot).
const EXTRA_SHOT_S: f64 = 3.5;
/// Seconds lost per turn at a line end.
const TURN_OVERHEAD_S: f64 = 4.0;
/// A ring stop also turns the drone to face the area.
const RING_STOP_S: f64 = 5.0;
/// Climb / descent rate used for height changes between legs, m/s.
const VERTICAL_MS: f64 = 3.0;
/// Overlap between the high overview photos: the least that still lets
/// neighbours match, so the overview stays the fewest photos possible.
const OVERVIEW_OVERLAP: f64 = 0.4;
/// The overview must fly at least this much higher than the grid to add
/// anything; closer than that it just repeats the grid.
const OVERVIEW_MIN_RATIO: f64 = 1.3;

fn polygon_area(p: &[(f64, f64)]) -> f64 {
    let n = p.len();
    let mut a = 0.0;
    for i in 0..n {
        let (x0, y0) = p[i];
        let (x1, y1) = p[(i + 1) % n];
        a += x0 * y1 - x1 * y0;
    }
    a.abs() / 2.0
}

/// Area of a (lon, lat) boundary in square metres.
pub fn boundary_area_m2(boundary: &[(f64, f64)]) -> f64 {
    if boundary.len() < 3 {
        return 0.0;
    }
    let (clon, clat) = (
        boundary.iter().map(|p| p.0).sum::<f64>() / boundary.len() as f64,
        boundary.iter().map(|p| p.1).sum::<f64>() / boundary.len() as f64,
    );
    let zone = UtmZone::for_lonlat(clon, clat);
    let pts: Vec<(f64, f64)> = boundary
        .iter()
        .map(|&(lon, lat)| to_utm(lon, lat, zone))
        .collect();
    polygon_area(&pts)
}

/// Bearing (deg clockwise from north) of a vector in east/north metres.
fn bearing(dx: f64, dy: f64) -> f64 {
    dx.atan2(dy).to_degrees().rem_euclid(360.0)
}

/// Convex hull, counter-clockwise (Andrew's monotone chain).
fn convex_hull(pts: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut p = pts.to_vec();
    p.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    p.dedup();
    if p.len() < 3 {
        return p;
    }
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };
    let mut lower: Vec<(f64, f64)> = Vec::new();
    for &q in &p {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], q) <= 0.0 {
            lower.pop();
        }
        lower.push(q);
    }
    let mut upper: Vec<(f64, f64)> = Vec::new();
    for &q in p.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], q) <= 0.0 {
            upper.pop();
        }
        upper.push(q);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Nearest point on a closed polygon's outline.
fn nearest_on_outline(poly: &[(f64, f64)], p: (f64, f64)) -> (f64, f64) {
    let mut best = (f64::MAX, poly[0]);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 {
            (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = (a.0 + t * dx, a.1 + t * dy);
        let d = (q.0 - p.0).hypot(q.1 - p.1);
        if d < best.0 {
            best = (d, q);
        }
    }
    best.1
}

/// The path `offset` metres outside a convex polygon (counter-clockwise),
/// with rounded corners, as a dense list of points.
fn offset_ring(hull: &[(f64, f64)], offset: f64) -> Vec<(f64, f64)> {
    let n = hull.len();
    let mut out = Vec::new();
    for i in 0..n {
        let (a, b) = (hull[i], hull[(i + 1) % n]);
        let len = (b.0 - a.0).hypot(b.1 - a.1).max(1e-9);
        // Outward normal of a counter-clockwise edge is to its right.
        let (nx, ny) = ((b.1 - a.1) / len, -(b.0 - a.0) / len);
        out.push((a.0 + nx * offset, a.1 + ny * offset));
        out.push((b.0 + nx * offset, b.1 + ny * offset));
        // Arc around corner b to the next edge's normal.
        let c = hull[(i + 2) % n];
        let len2 = (c.0 - b.0).hypot(c.1 - b.1).max(1e-9);
        let (mx, my) = ((c.1 - b.1) / len2, -(c.0 - b.0) / len2);
        let a0 = ny.atan2(nx);
        let mut a1 = my.atan2(mx);
        // Counter-clockwise hull: the normal turns counter-clockwise.
        while a1 < a0 {
            a1 += std::f64::consts::TAU;
        }
        let steps = ((a1 - a0) / 0.2).ceil() as usize;
        for k in 1..steps {
            let t = a0 + (a1 - a0) * k as f64 / steps as f64;
            out.push((b.0 + t.cos() * offset, b.1 + t.sin() * offset));
        }
    }
    out
}

/// Evenly spaced points along a closed path.
fn resample_closed(path: &[(f64, f64)], spacing: f64) -> Vec<(f64, f64)> {
    let n = path.len();
    let seg_len = |i: usize| {
        let (a, b) = (path[i], path[(i + 1) % n]);
        (b.0 - a.0).hypot(b.1 - a.1)
    };
    let total: f64 = (0..n).map(seg_len).sum();
    let count = ((total / spacing).ceil() as usize).max(3);
    let step = total / count as f64;
    let mut out = Vec::with_capacity(count);
    // Walk the path once; `start` is the distance at which segment `i` begins.
    let (mut i, mut start) = (0usize, 0.0f64);
    for k in 0..count {
        let target = k as f64 * step;
        while i < n - 1 && start + seg_len(i) < target {
            start += seg_len(i);
            i += 1;
        }
        let (a, b) = (path[i], path[(i + 1) % n]);
        let t = ((target - start) / seg_len(i).max(1e-9)).clamp(0.0, 1.0);
        out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
    }
    out
}

/// A run of waypoints flown together, for packing into missions.
struct Segment {
    wps: Vec<Waypoint>,
    /// Local x/y of each waypoint, for distances between segments.
    xy: Vec<(f64, f64)>,
    len: f64,
    photos: usize,
    /// Seconds spent stopping and shooting (not travelling).
    stops_s: f64,
}

pub fn plan(input: &PlanInput) -> Result<Plan> {
    match input.goal {
        Goal::Manual => plan_at(input, input.altitude_m),
        Goal::Batteries { count } => {
            optimise(input, |p| p.stats.batteries <= count.max(1) as usize)
        }
        Goal::Time { minutes } => {
            optimise(input, |p| p.stats.duration_s <= minutes.max(1.0) * 60.0)
        }
    }
}

/// The lowest height (most detail) between min and max whose whole flight
/// fits the budget. Cost falls as height rises, so a bisection finds it.
fn optimise(input: &PlanInput, fits: impl Fn(&Plan) -> bool) -> Result<Plan> {
    let lo = input.min_alt_m.clamp(10.0, 500.0).round() as i64;
    let hi = input.max_alt_m.clamp(lo as f64, 500.0).round() as i64;
    let ok = |alt: i64| plan_at(input, alt as f64).ok().filter(|p| fits(p));
    let mut chosen = None;
    if let Some(p) = ok(lo) {
        chosen = Some(p);
    } else if ok(hi).is_some() {
        let (mut bad, mut good) = (lo, hi);
        while good - bad > 1 {
            let mid = (bad + good) / 2;
            if ok(mid).is_some() {
                good = mid;
            } else {
                bad = mid;
            }
        }
        chosen = ok(good);
    }
    let mut plan = match chosen {
        Some(p) => p,
        None => {
            let mut p = plan_at(input, hi as f64)?;
            p.warnings.insert(
                0,
                format!(
                    "Even at {hi} m this needs {} batteries ({}). Add batteries or time, turn off the extra legs, or make the area smaller.",
                    p.stats.batteries,
                    fmt_minutes(p.stats.duration_s)
                ),
            );
            p
        }
    };
    // The trade-off at a handful of heights, for the user to weigh up.
    let chosen_alt = plan.stats.altitude_m.round() as i64;
    let mut heights: Vec<i64> = (0..6).map(|k| lo + (hi - lo) * k / 5).collect();
    heights.push(chosen_alt);
    heights.sort();
    heights.dedup();
    plan.options = heights
        .into_iter()
        .filter_map(|alt| {
            let p = plan_at(input, alt as f64).ok()?;
            Some(HeightOption {
                altitude_m: alt as f64,
                gsd_cm: p.stats.gsd_cm,
                photos: p.stats.photos,
                duration_s: p.stats.duration_s,
                batteries: p.stats.batteries,
                fits: fits(&p),
            })
        })
        .collect();
    Ok(plan)
}

fn fmt_minutes(s: f64) -> String {
    let m = (s / 60.0).round() as i64;
    if m >= 60 {
        format!("{} h {} min", m / 60, m % 60)
    } else {
        format!("{m} min")
    }
}

/// The full plan with the grid (and ring) at `alt`.
pub fn plan_at(input: &PlanInput, alt: f64) -> Result<Plan> {
    let b = &input.boundary;
    if b.len() < 3 {
        bail!("draw at least three corners of the area");
    }
    let cam = &input.camera;
    if cam.focal_mm <= 0.0 || cam.sensor_w_mm <= 0.0 || cam.sensor_h_mm <= 0.0 {
        bail!("camera settings are incomplete");
    }
    if !(5.0..=500.0).contains(&alt) {
        bail!("height must be between 5 and 500 m");
    }
    if input.capture == CaptureMode::PhotoAtWaypoint {
        if input.shots.is_empty() {
            bail!("add at least one photo per waypoint");
        }
        if input
            .shots
            .iter()
            .any(|s| !(-90.0..=0.0).contains(&s.pitch_deg))
        {
            bail!("camera tilt must be between -90° (straight down) and 0° (horizon)");
        }
        let worst = waypoint_actions(input, true, 0.0).len();
        if worst > LITCHI_MAX_ACTIONS {
            bail!(
                "that many photos per waypoint needs {worst} Litchi actions; the limit is {LITCHI_MAX_ACTIONS}. \
                 Remove a photo or group photos that share a direction."
            );
        }
    }
    let front = input.front_overlap.clamp(0.0, 0.95);
    let side = input.side_overlap.clamp(0.0, 0.95);
    let mut warnings = Vec::new();

    let (clon, clat) = (
        b.iter().map(|p| p.0).sum::<f64>() / b.len() as f64,
        b.iter().map(|p| p.1).sum::<f64>() / b.len() as f64,
    );
    let zone = UtmZone::for_lonlat(clon, clat);
    let (e0, n0) = to_utm(clon, clat, zone);
    let pts: Vec<(f64, f64)> = b
        .iter()
        .map(|&(lon, lat)| {
            let (e, n) = to_utm(lon, lat, zone);
            (e - e0, n - n0)
        })
        .collect();
    let area = polygon_area(&pts);
    let unproject_en = |e: f64, n: f64| from_utm(e + e0, n + n0, zone);

    // Ground footprint of one photo; long side across the line.
    let fw = alt * cam.sensor_w_mm / cam.focal_mm;
    let fh = alt * cam.sensor_h_mm / cam.focal_mm;
    let gsd_cm = fw / cam.image_w_px as f64 * 100.0;
    let line_spacing = fw * (1.0 - side);
    let photo_spacing = fh * (1.0 - front);

    let direction = match input.direction_deg {
        Some(d) => d.rem_euclid(180.0),
        None => {
            let mut best = (0.0, 0.0);
            for i in 0..pts.len() {
                let (a, c) = (pts[i], pts[(i + 1) % pts.len()]);
                let len = (c.0 - a.0).hypot(c.1 - a.1);
                if len > best.0 {
                    best = (len, bearing(c.0 - a.0, c.1 - a.1).rem_euclid(180.0));
                }
            }
            best.1
        }
    };

    // Rotate so flight lines run along +x. A bearing θ is the unit vector
    // (sin θ, cos θ); rotating by φ = θ − 90° maps it onto (1, 0).
    let th = direction.to_radians();
    let (ux, uy) = (th.sin(), th.cos()); // along-line
    let (vx, vy) = (-uy, ux); // across-line (90° counter-clockwise)
    let to_local = |p: (f64, f64)| (p.0 * ux + p.1 * uy, p.0 * vx + p.1 * vy);
    let to_world = |x: f64, y: f64| (x * ux + y * vx, x * uy + y * vy);
    let local: Vec<(f64, f64)> = pts.iter().map(|&p| to_local(p)).collect();
    let xmin = local.iter().map(|p| p.0).fold(f64::MAX, f64::min);
    let xmax = local.iter().map(|p| p.0).fold(f64::MIN, f64::max);
    let ymin = local.iter().map(|p| p.1).fold(f64::MAX, f64::min);
    let ymax = local.iter().map(|p| p.1).fold(f64::MIN, f64::max);

    let n_lines = (((ymax - ymin) / line_spacing).ceil() as usize + 1).max(1);
    if n_lines > 2000 {
        bail!("that area needs over 2,000 flight lines — fly higher or split the area");
    }
    let span = (n_lines - 1) as f64 * line_spacing;
    let y_start = ymin + ((ymax - ymin) - span) / 2.0;
    let overshoot = input.overshoot.max(0.0) * photo_spacing;

    // Each line: (x_from, x_to, y) in local metres, serpentine.
    let mut lines: Vec<(f64, f64, f64)> = Vec::new();
    for k in 0..n_lines {
        let y = y_start + k as f64 * line_spacing;
        let mut xs = Vec::new();
        for i in 0..local.len() {
            let (a, c) = (local[i], local[(i + 1) % local.len()]);
            if (a.1 <= y && c.1 > y) || (c.1 <= y && a.1 > y) {
                xs.push(a.0 + (y - a.1) / (c.1 - a.1) * (c.0 - a.0));
            }
        }
        // A line that clips only a vertex still needs covering: use the
        // nearest edge span so the boundary's extremes aren't skipped.
        if xs.is_empty() {
            let near: Vec<f64> = local
                .iter()
                .filter(|p| (p.1 - y).abs() <= line_spacing / 2.0)
                .map(|p| p.0)
                .collect();
            if near.is_empty() {
                continue;
            }
            xs = near;
        }
        let x0 = xs.iter().copied().fold(f64::MAX, f64::min) - overshoot;
        let x1 = xs.iter().copied().fold(f64::MIN, f64::max) + overshoot;
        if lines.len() % 2 == 0 {
            lines.push((x0, x1, y));
        } else {
            lines.push((x1, x0, y));
        }
    }

    // Speed: in interval mode the drone must not outrun the shutter.
    let mut speed = input.speed_ms.clamp(1.0, 15.0);
    if input.capture == CaptureMode::DistanceInterval {
        let max = photo_spacing / cam.min_photo_interval_s;
        if speed > max {
            warnings.push(format!(
                "Speed lowered to {max:.1} m/s so the camera can keep up (one photo every {:.0} m).",
                photo_spacing
            ));
            speed = max;
        }
    }

    // Local (along/across) → world east/north.
    let local_to_en = |x: f64, y: f64| to_world(x, y);
    let wp = |e: f64,
              n: f64,
              alt_m: f64,
              heading: f64,
              pitch_deg: f64,
              leg: Leg,
              photo: bool,
              dist: f64| {
        let (lon, lat) = unproject_en(e, n);
        Waypoint {
            lon,
            lat,
            alt_m,
            heading,
            pitch_deg,
            leg,
            photo,
            photo_dist_m: dist,
            first: false,
        }
    };

    // ── Overview: the fewest high photos covering the whole area ──
    let mut segments: Vec<Segment> = Vec::new();
    let mut overview_alt = 0.0;
    let mut overview_photos = 0;
    let overview_h = input.max_alt_m.clamp(alt, 500.0);
    if input.overview && overview_h < alt * OVERVIEW_MIN_RATIO {
        warnings.push(format!(
            "No overview photos: the grid already flies at {alt:.0} m, close to the {overview_h:.0} m overview height, \
             so they would only repeat it."
        ));
    } else if input.overview {
        let h = overview_h;
        overview_alt = h;
        let ow = h * cam.sensor_w_mm / cam.focal_mm; // across the lines
        let oh = h * cam.sensor_h_mm / cam.focal_mm; // along the lines
        let count = |extent: f64, size: f64| {
            if extent <= size {
                1
            } else {
                ((extent - size) / (size * (1.0 - OVERVIEW_OVERLAP))).ceil() as usize + 1
            }
        };
        let (nx, ny) = (count(xmax - xmin, oh), count(ymax - ymin, ow));
        let (cx, cy) = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0);
        let mut spots: Vec<(f64, f64)> = Vec::new();
        if nx * ny == 1 {
            // One photo would cover it, but a model needs views to compare:
            // three overlapping shots along the long axis.
            let d = oh * 0.25;
            spots.extend([(cx - d, cy), (cx, cy), (cx + d, cy)]);
        } else {
            let step = |n: usize, size: f64| {
                if n > 1 {
                    size * (1.0 - OVERVIEW_OVERLAP)
                } else {
                    0.0
                }
            };
            let (sx, sy) = (step(nx, oh), step(ny, ow));
            for j in 0..ny {
                let y = cy + (j as f64 - (ny - 1) as f64 / 2.0) * sy;
                for i in 0..nx {
                    let i = if j % 2 == 0 { i } else { nx - 1 - i };
                    spots.push((cx + (i as f64 - (nx - 1) as f64 / 2.0) * sx, y));
                }
            }
        }
        let heading = bearing(ux, uy);
        let xy = spots.clone();
        let wps: Vec<Waypoint> = spots
            .iter()
            .map(|&(x, y)| {
                let (e, n) = local_to_en(x, y);
                wp(e, n, h, heading, -90.0, Leg::Overview, true, 0.0)
            })
            .collect();
        let len: f64 = xy
            .windows(2)
            .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
            .sum();
        overview_photos = wps.len();
        segments.push(Segment {
            stops_s: wps.len() as f64 * STOP_OVERHEAD_S,
            photos: wps.len(),
            wps,
            xy,
            len,
        });
    }

    // ── Grid ──
    let mut grid_lines = 0;
    for &(x0, x1, y) in &lines {
        let len = (x1 - x0).abs();
        let dir = (x1 - x0).signum();
        let heading = bearing(ux * dir, uy * dir);
        let mut wps = Vec::new();
        let mut xy = Vec::new();
        let photos;
        let stops_s;
        match input.capture {
            CaptureMode::PhotoAtWaypoint => {
                let n = (len / photo_spacing).ceil() as usize + 1;
                let step = if n > 1 { len / (n - 1) as f64 } else { 0.0 };
                for i in 0..n {
                    let x = x0 + dir * step * i as f64;
                    let (e, n_) = local_to_en(x, y);
                    wps.push(wp(e, n_, alt, heading, -90.0, Leg::Grid, true, 0.0));
                    xy.push((x, y));
                }
                photos = n * input.shots.len().max(1);
                let extra = (input.shots.len().max(1) - 1) as f64 * EXTRA_SHOT_S;
                stops_s = n as f64 * (STOP_OVERHEAD_S + extra) + TURN_OVERHEAD_S;
            }
            CaptureMode::DistanceInterval => {
                for (x, dist) in [(x0, photo_spacing), (x1, 0.0)] {
                    let (e, n_) = local_to_en(x, y);
                    wps.push(wp(e, n_, alt, heading, -90.0, Leg::Grid, false, dist));
                    xy.push((x, y));
                }
                photos = (len / photo_spacing).floor() as usize + 1;
                stops_s = TURN_OVERHEAD_S;
            }
        }
        if wps.len() > LITCHI_MAX_WAYPOINTS {
            bail!(
                "one flight line needs {} waypoints, over Litchi's limit of {LITCHI_MAX_WAYPOINTS} — \
                 fly higher, use the distance-interval mode, or draw a smaller area",
                wps.len()
            );
        }
        grid_lines += 1;
        segments.push(Segment {
            wps,
            xy,
            len,
            photos,
            stops_s,
        });
    }
    if grid_lines == 0 {
        bail!("the area is too small for this height — fly lower or draw a larger area");
    }

    // ── Ring: outside the area, looking in at its border ──
    let mut ring_photos = 0;
    let mut ring_offset = 0.0;
    if input.ring {
        let tilt = (-input.ring_pitch_deg).clamp(10.0, 80.0).to_radians();
        // Far enough out that the middle of each photo lands on the border.
        ring_offset = alt / tilt.tan();
        let slant = alt / tilt.sin();
        let width = slant * cam.sensor_w_mm / cam.focal_mm;
        let spacing = (width * (1.0 - front)).max(2.0);
        let hull = convex_hull(&local);
        if hull.len() >= 3 {
            let path = offset_ring(&hull, ring_offset);
            let spots = resample_closed(&path, spacing);
            let mut wps = Vec::new();
            let mut xy = Vec::new();
            for &(x, y) in &spots {
                let target = nearest_on_outline(&hull, (x, y));
                let (dxl, dyl) = (target.0 - x, target.1 - y);
                let (de, dn) = local_to_en(dxl, dyl);
                let (e, n_) = local_to_en(x, y);
                wps.push(wp(
                    e,
                    n_,
                    alt,
                    bearing(de, dn),
                    input.ring_pitch_deg,
                    Leg::Ring,
                    true,
                    0.0,
                ));
                xy.push((x, y));
            }
            // Close the loop back to the start, then split into Litchi-sized runs.
            ring_photos = wps.len();
            let mut i = 0;
            while i < wps.len() {
                let j = (i + LITCHI_MAX_WAYPOINTS).min(wps.len());
                let part_xy = xy[i..j].to_vec();
                let len: f64 = part_xy
                    .windows(2)
                    .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
                    .sum();
                segments.push(Segment {
                    wps: wps[i..j].to_vec(),
                    photos: j - i,
                    stops_s: (j - i) as f64 * RING_STOP_S,
                    xy: part_xy,
                    len,
                });
                i = j;
            }
        }
    }

    // ── Pack segments into missions (Litchi's waypoint limit, battery time) ──
    let battery_s = input.battery_minutes.max(3.0) * 60.0;
    let seg_time = |s: &Segment| s.len / speed + s.stops_s;
    let transit = |a: &Segment, b: &Segment| {
        let (p, q) = (a.xy[a.xy.len() - 1], b.xy[0]);
        let dz = (a.wps[a.wps.len() - 1].alt_m - b.wps[0].alt_m).abs();
        ((q.0 - p.0).hypot(q.1 - p.1), dz)
    };
    let empty = || Mission {
        waypoints: Vec::new(),
        lines: 0,
        photos: 0,
        distance_m: 0.0,
        duration_s: 0.0,
    };
    let mut missions: Vec<Mission> = Vec::new();
    let mut cur = empty();
    let mut prev: Option<usize> = None;
    for (k, s) in segments.iter().enumerate() {
        let (d, dz) = match prev {
            Some(p) if cur.lines > 0 => transit(&segments[p], s),
            _ => (0.0, 0.0),
        };
        let add_t = seg_time(s) + d / speed + dz / VERTICAL_MS;
        let over_wp = cur.waypoints.len() + s.wps.len() > LITCHI_MAX_WAYPOINTS;
        let over_t = cur.duration_s + add_t > battery_s;
        let (d, dz) = if cur.lines > 0 && (over_wp || over_t) {
            missions.push(std::mem::replace(&mut cur, empty()));
            (0.0, 0.0)
        } else {
            (d, dz)
        };
        cur.waypoints.extend(s.wps.iter().cloned());
        cur.lines += 1;
        cur.photos += s.photos;
        cur.distance_m += s.len + d;
        cur.duration_s += seg_time(s) + d / speed + dz / VERTICAL_MS;
        if cur.duration_s > battery_s && cur.lines == 1 {
            warnings.push(format!(
                "Part {} of the flight alone takes longer than one battery — consider flying higher or faster.",
                k + 1
            ));
        }
        prev = Some(k);
    }
    if cur.lines > 0 {
        missions.push(cur);
    }
    for m in &mut missions {
        if let Some(w) = m.waypoints.first_mut() {
            w.first = true;
        }
    }

    if alt > LEGAL_MAX_ALT_M || (input.overview && overview_alt > LEGAL_MAX_ALT_M) {
        warnings.push(format!(
            "{:.0} m is above the usual {LEGAL_MAX_ALT_M:.0} m limit (CASA in Australia). Check you're allowed to fly this high.",
            alt.max(overview_alt)
        ));
    }
    if input.capture == CaptureMode::DistanceInterval && input.shots.len() > 1 {
        warnings.push("Shooting while flying takes one straight-down photo per spot; extra angles need stop-and-shoot.".into());
    }
    if input.capture == CaptureMode::DistanceInterval {
        warnings.push(
            "Distance-interval photos depend on Litchi's interval setting, which isn't officially documented for the \
             Mavic Air 2. Do a short test mission first, or use photo-at-each-waypoint."
                .into(),
        );
    }
    if input.capture == CaptureMode::PhotoAtWaypoint
        && input.shots.iter().any(|s| s.pitch_deg > -80.0)
    {
        warnings.push(
            "Angled photos are for 3D models. The 2D map uses only the straight-down ones and skips the rest.".into(),
        );
        if !input.shots.iter().any(|s| s.pitch_deg <= -80.0) {
            warnings.push(
                "No straight-down photo is included, so these photos can't make a 2D map.".into(),
            );
        }
    }
    if input.ring {
        warnings.push(format!(
            "The edge ring flies {:.0} m outside the area. Check that strip is clear of trees, buildings and wires, \
             and that you may fly over it.",
            ring_offset
        ));
    }
    if front < 0.6 || side < 0.5 {
        warnings.push(
            "Overlap below 60% front / 50% side often leaves gaps in the stitched map.".into(),
        );
    }

    // Missions split by the waypoint limit are often short; pack them into
    // batteries in flying order, allowing a minute for each restart.
    let mut batteries = 0;
    let mut left = 0.0;
    for m in &missions {
        let need = m.duration_s + if left < battery_s { 60.0 } else { 0.0 };
        if batteries == 0 || need > left {
            batteries += 1;
            left = battery_s - m.duration_s;
        } else {
            left -= need;
        }
    }

    let photos: usize = missions.iter().map(|m| m.photos).sum();
    let stats = PlanStats {
        area_m2: area,
        altitude_m: alt,
        gsd_cm,
        footprint_w_m: fw,
        footprint_h_m: fh,
        line_spacing_m: line_spacing,
        photo_spacing_m: photo_spacing,
        speed_ms: speed,
        direction_deg: direction,
        lines: grid_lines,
        photos,
        grid_photos: photos - ring_photos - overview_photos,
        ring_photos,
        ring_offset_m: ring_offset,
        overview_photos,
        overview_alt_m: overview_alt,
        distance_m: missions.iter().map(|m| m.distance_m).sum(),
        duration_s: missions.iter().map(|m| m.duration_s).sum(),
        missions: missions.len(),
        batteries,
    };
    Ok(Plan {
        missions,
        stats,
        warnings,
        options: Vec::new(),
    })
}

// ───────────────────────────── Litchi CSV ─────────────────────────────

const LITCHI_HEADER_FIXED: &str = "latitude,longitude,altitude(m),heading(deg),curvesize(m),rotationdir,gimbalmode,gimbalpitchangle";
const LITCHI_HEADER_TAIL: &str =
    "altitudemode,speed(m/s),poi_latitude,poi_longitude,poi_altitude(m),poi_altitudemode,photo_timeinterval,photo_distinterval";

/// Litchi actions for a photo waypoint. Codes: 0 stay (ms), 1 take photo,
/// 4 rotate aircraft (absolute heading, -180..180), 5 tilt camera (deg).
///
/// Turns and tilts are emitted only when they change, and only the first shot
/// waits to settle: that keeps "straight down + 45° on all four sides" within
/// Litchi's 15 actions per waypoint.
pub fn waypoint_actions(input: &PlanInput, first: bool, line_heading: f64) -> Vec<(i32, f64)> {
    let mut a: Vec<(i32, f64)> = Vec::new();
    if first {
        a.push((5, -90.0)); // camera straight down before the first leg
        a.push((0, 2000.0));
    }
    let mut yaw = 0.0; // relative to the line; the drone arrives facing along it
    let mut pitch = -90.0; // the gimbal column holds -90 between waypoints
    for (i, s) in input.shots.iter().enumerate() {
        if (s.yaw_offset_deg - yaw).rem_euclid(360.0) > 0.5 {
            let abs = (line_heading + s.yaw_offset_deg).rem_euclid(360.0);
            a.push((4, if abs > 180.0 { abs - 360.0 } else { abs }));
            yaw = s.yaw_offset_deg;
        }
        if (s.pitch_deg - pitch).abs() > 0.5 {
            a.push((5, s.pitch_deg));
            pitch = s.pitch_deg;
        }
        if i == 0 {
            a.push((0, 1000.0)); // settle on arrival
        }
        a.push((1, 0.0));
    }
    a
}

/// Litchi actions for one waypoint, whichever leg it belongs to.
fn actions_for(input: &PlanInput, w: &Waypoint) -> Vec<(i32, f64)> {
    match w.leg {
        Leg::Grid if w.photo => waypoint_actions(input, w.first, w.heading),
        Leg::Grid => {
            if w.first {
                vec![(5, -90.0), (0, 2000.0)]
            } else {
                Vec::new()
            }
        }
        // The drone already faces the waypoint's heading (Heading: Custom);
        // tilt, settle, shoot.
        Leg::Ring | Leg::Overview => {
            let settle = if w.first { 2000.0 } else { 1000.0 };
            vec![(5, w.pitch_deg), (0, settle), (1, 0.0)]
        }
    }
}

/// Action codes (Litchi CSV): -1 none, 0 stay (ms), 1 take photo,
/// 4 rotate aircraft, 5 tilt camera (deg).
pub fn litchi_csv(input: &PlanInput, mission: &Mission, speed_ms: f64) -> String {
    let mut out = String::new();
    out.push_str(LITCHI_HEADER_FIXED);
    for i in 1..=15 {
        out.push_str(&format!(",actiontype{i},actionparam{i}"));
    }
    out.push(',');
    out.push_str(LITCHI_HEADER_TAIL);
    out.push('\n');
    let alt_mode = match input.altitude_mode {
        AltitudeMode::RelativeToTakeoff => 0,
        AltitudeMode::AboveGround => 1,
    };
    for w in &mission.waypoints {
        let actions = actions_for(input, w);
        // gimbalmode 2 = interpolate between waypoints' pitches.
        out.push_str(&format!(
            "{:.8},{:.8},{:.1},{:.1},0,0,2,{:.0}",
            w.lat, w.lon, w.alt_m, w.heading, w.pitch_deg
        ));
        for i in 0..15 {
            match actions.get(i) {
                Some((t, p)) => out.push_str(&format!(",{t},{p}")),
                None => out.push_str(",-1,0"),
            }
        }
        out.push_str(&format!(
            ",{alt_mode},{speed_ms:.1},0,0,0,0,-1,{}\n",
            if w.photo_dist_m > 0.0 {
                format!("{:.1}", w.photo_dist_m)
            } else {
                "-1".into()
            }
        ));
    }
    out
}

/// The route and boundary as KML, for checking in Google Earth before flying.
pub fn plan_kml(input: &PlanInput, plan: &Plan) -> String {
    let esc = |s: &str| {
        s.replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;")
    };
    let mut k = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>\n",
    );
    k.push_str(&format!("<name>{}</name>\n", esc(&input.name)));
    k.push_str("<Style id=\"area\"><LineStyle><color>ff00ffff</color><width>2</width></LineStyle><PolyStyle><color>2000ffff</color></PolyStyle></Style>\n");
    k.push_str("<Style id=\"route\"><LineStyle><color>ffc0e036</color><width>2</width></LineStyle></Style>\n");
    let ring: Vec<String> = input
        .boundary
        .iter()
        .chain(input.boundary.first())
        .map(|(lon, lat)| format!("{lon:.8},{lat:.8},0"))
        .collect();
    k.push_str(&format!(
        "<Placemark><name>Area</name><styleUrl>#area</styleUrl><Polygon><outerBoundaryIs><LinearRing><coordinates>{}</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>\n",
        ring.join(" ")
    ));
    let alt_mode = match input.altitude_mode {
        AltitudeMode::RelativeToTakeoff => "relativeToGround",
        AltitudeMode::AboveGround => "relativeToGround",
    };
    for (i, m) in plan.missions.iter().enumerate() {
        let coords: Vec<String> = m
            .waypoints
            .iter()
            .map(|w| format!("{:.8},{:.8},{:.1}", w.lon, w.lat, w.alt_m))
            .collect();
        k.push_str(&format!(
            "<Placemark><name>Mission {} ({} photos, {:.0} min)</name><styleUrl>#route</styleUrl><LineString><altitudeMode>{alt_mode}</altitudeMode><coordinates>{}</coordinates></LineString></Placemark>\n",
            i + 1,
            m.photos,
            m.duration_s / 60.0,
            coords.join(" ")
        ));
    }
    k.push_str("</Document></kml>\n");
    k
}

/// Reads the first polygon (or line) from a KML/KMZ-extracted KML string as
/// (lon, lat) pairs. Good enough for boundaries drawn in Google Earth.
pub fn parse_kml_boundary(kml: &str) -> Result<Vec<(f64, f64)>> {
    let lower = kml.to_ascii_lowercase();
    let start_tag = ["<polygon", "<linestring", "<linearring"]
        .iter()
        .filter_map(|t| lower.find(t))
        .min()
        .unwrap_or(0);
    let Some(rel) = lower[start_tag..].find("<coordinates") else {
        bail!("no shape found in that KML file")
    };
    let open = start_tag + rel;
    let body_start = open + lower[open..].find('>').map(|i| i + 1).unwrap_or(0);
    let Some(len) = lower[body_start..].find("</coordinates") else {
        bail!("the KML file is incomplete")
    };
    let mut pts: Vec<(f64, f64)> = kml[body_start..body_start + len]
        .split_whitespace()
        .filter_map(|t| {
            let mut it = t.split(',');
            Some((it.next()?.parse().ok()?, it.next()?.parse().ok()?))
        })
        .collect();
    if pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    if pts.len() < 3 {
        bail!("the KML shape needs at least three points");
    }
    Ok(pts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(size_m: f64) -> Vec<(f64, f64)> {
        // ~size_m square near Sydney.
        let (lon0, lat0): (f64, f64) = (151.0, -33.9);
        let dlat = size_m / 111_320.0;
        let dlon = size_m / (111_320.0 * lat0.to_radians().cos());
        vec![
            (lon0, lat0),
            (lon0 + dlon, lat0),
            (lon0 + dlon, lat0 + dlat),
            (lon0, lat0 + dlat),
        ]
    }

    #[test]
    fn square_survey_has_expected_geometry() {
        let input = PlanInput {
            boundary: square(300.0),
            direction_deg: Some(90.0),
            ..Default::default()
        };
        let p = plan(&input).unwrap();
        let s = &p.stats;
        // Mavic Air 2 at 80 m: footprint ≈ 113.8 × 85.3 m, GSD ≈ 2.84 cm.
        assert!(
            (s.footprint_w_m - 113.78).abs() < 0.1,
            "{}",
            s.footprint_w_m
        );
        assert!((s.gsd_cm - 2.84).abs() < 0.01, "{}", s.gsd_cm);
        assert!((s.area_m2 - 90_000.0).abs() < 900.0, "{}", s.area_m2);
        // 300 m across at 39.8 m spacing → 9 lines.
        assert_eq!(s.lines, 9);
        assert!(s.batteries <= s.missions && s.batteries >= 1);
        for m in &p.missions {
            assert!(m.waypoints.len() <= LITCHI_MAX_WAYPOINTS);
            assert!(m.duration_s <= input.battery_minutes * 60.0 + 1.0);
        }
        // East-west lines alternate heading 90 / 270.
        let h: Vec<f64> = p.missions[0]
            .waypoints
            .iter()
            .map(|w| w.heading.round())
            .collect();
        assert!(h.contains(&90.0) && h.contains(&270.0));
    }

    #[test]
    fn csv_has_one_row_per_waypoint_and_full_header() {
        let input = PlanInput {
            boundary: square(150.0),
            ..Default::default()
        };
        let p = plan(&input).unwrap();
        let csv = litchi_csv(&input, &p.missions[0], p.stats.speed_ms);
        let rows: Vec<&str> = csv.lines().collect();
        assert_eq!(rows.len(), p.missions[0].waypoints.len() + 1);
        let cols = rows[0].split(',').count();
        assert_eq!(cols, 8 + 30 + 8);
        assert!(rows.iter().all(|r| r.split(',').count() == cols));
        assert!(rows[1].contains(",5,-90,0,2000,0,1000,1,0,"), "{}", rows[1]);
    }

    #[test]
    fn four_way_obliques_fit_litchi_action_limit() {
        let mut shots = nadir_only();
        for yaw in [0.0, 90.0, 180.0, 270.0] {
            shots.push(Shot {
                pitch_deg: -45.0,
                yaw_offset_deg: yaw,
            });
        }
        let input = PlanInput {
            boundary: square(200.0),
            shots,
            ..Default::default()
        };
        let p = plan(&input).unwrap();
        let a = waypoint_actions(&input, true, 90.0);
        assert!(a.len() <= LITCHI_MAX_ACTIONS, "{a:?}");
        // Five photos, four of them after a tilt to -45.
        assert_eq!(a.iter().filter(|x| x.0 == 1).count(), 5);
        assert_eq!(a.iter().filter(|x| x.0 == 5 && x.1 == -45.0).count(), 1);
        // Rotations: right of an east-bound line is south (180), back is west (-90).
        assert!(a.contains(&(4, 180.0)) && a.contains(&(4, -90.0)), "{a:?}");
        let wps: usize = p.missions.iter().map(|m| m.waypoints.len()).sum();
        assert_eq!(p.stats.photos, wps * 5);
        assert!(p.warnings.iter().any(|w| w.contains("3D")));
        // Every CSV row still has the same column count.
        let csv = litchi_csv(&input, &p.missions[0], p.stats.speed_ms);
        let n = csv.lines().next().unwrap().split(',').count();
        assert!(csv.lines().all(|r| r.split(',').count() == n));
    }

    #[test]
    fn too_many_shots_is_refused() {
        let shots = (0..8)
            .map(|i| Shot {
                pitch_deg: -30.0 - i as f64,
                yaw_offset_deg: i as f64 * 45.0,
            })
            .collect();
        let input = PlanInput {
            boundary: square(200.0),
            shots,
            ..Default::default()
        };
        assert!(plan(&input).unwrap_err().to_string().contains("15"));
    }

    #[test]
    fn interval_mode_uses_two_waypoints_per_line() {
        let input = PlanInput {
            boundary: square(300.0),
            capture: CaptureMode::DistanceInterval,
            ..Default::default()
        };
        let p = plan(&input).unwrap();
        let wps: usize = p.missions.iter().map(|m| m.waypoints.len()).sum();
        assert_eq!(wps, p.stats.lines * 2);
    }

    /// Local east/north metres of a waypoint relative to the square's centre.
    fn en(w: &Waypoint, size: f64) -> (f64, f64) {
        let (lon0, lat0): (f64, f64) = (151.0, -33.9);
        let x = (w.lon - lon0) * 111_320.0 * lat0.to_radians().cos() - size / 2.0;
        let y = (w.lat - lat0) * 111_320.0 - size / 2.0;
        (x, y)
    }

    #[test]
    fn ring_flies_outside_and_looks_in_at_the_border() {
        let input = PlanInput {
            boundary: square(200.0),
            altitude_m: 60.0,
            ring: true,
            ..Default::default()
        };
        let p = plan(&input).unwrap();
        let ring: Vec<&Waypoint> = p
            .missions
            .iter()
            .flat_map(|m| &m.waypoints)
            .filter(|w| w.leg == Leg::Ring)
            .collect();
        assert_eq!(ring.len(), p.stats.ring_photos);
        assert!(ring.len() >= 12, "{}", ring.len());
        // 45° down from 60 m: the ring sits 60 m outside the area.
        assert!((p.stats.ring_offset_m - 60.0).abs() < 0.5);
        for w in &ring {
            let (x, y) = en(w, 200.0);
            // Distance outside the 200 m square.
            let dx = (x.abs() - 100.0).max(0.0);
            let dy = (y.abs() - 100.0).max(0.0);
            let out = dx.hypot(dy);
            assert!((out - 60.0).abs() < 2.0, "{out} at ({x:.0},{y:.0})");
            // Facing inward: the heading points back towards the centre half-plane.
            let h = w.heading.to_radians();
            assert!(
                h.sin() * -x + h.cos() * -y > 0.0,
                "heading {} at ({x:.0},{y:.0})",
                w.heading
            );
            assert_eq!(w.pitch_deg, -45.0);
            assert_eq!(w.alt_m, 60.0);
        }
        // Ring rows tilt to 45° and shoot.
        let csv = litchi_csv(&input, p.missions.last().unwrap(), p.stats.speed_ms);
        assert!(
            csv.lines()
                .last()
                .unwrap()
                .contains(",2,-45,5,-45,0,1000,1,0,"),
            "{csv}"
        );
    }

    #[test]
    fn overview_takes_the_fewest_high_photos() {
        // 150 m square fits in one 120 m-high photo (170 × 128 m): three shots.
        let small = PlanInput {
            boundary: square(120.0),
            overview: true,
            ..Default::default()
        };
        let p = plan(&small).unwrap();
        assert_eq!(p.stats.overview_photos, 3);
        assert_eq!(p.stats.overview_alt_m, 120.0);
        let ov: Vec<&Waypoint> = p
            .missions
            .iter()
            .flat_map(|m| &m.waypoints)
            .filter(|w| w.leg == Leg::Overview)
            .collect();
        assert!(ov.iter().all(|w| w.alt_m == 120.0 && w.pitch_deg == -90.0));
        // A 500 m square needs a small grid of them, far fewer than the main grid.
        let big = PlanInput {
            boundary: square(500.0),
            overview: true,
            ..Default::default()
        };
        let p = plan(&big).unwrap();
        assert!(
            (4..=40).contains(&p.stats.overview_photos),
            "{}",
            p.stats.overview_photos
        );
        assert!(p.stats.overview_photos * 5 < p.stats.grid_photos);
        assert_eq!(
            p.stats.photos,
            p.stats.grid_photos + p.stats.overview_photos
        );
    }

    #[test]
    fn budget_goal_picks_the_lowest_height_that_fits() {
        let base = PlanInput {
            boundary: square(400.0),
            ring: true,
            overview: true,
            ..Default::default()
        };
        let two = plan(&PlanInput {
            goal: Goal::Batteries { count: 2 },
            ..base.clone()
        })
        .unwrap();
        let five = plan(&PlanInput {
            goal: Goal::Batteries { count: 5 },
            ..base.clone()
        })
        .unwrap();
        assert!(two.stats.batteries <= 2, "{}", two.stats.batteries);
        assert!(five.stats.batteries <= 5);
        // More batteries buys a lower flight and finer detail.
        assert!(
            five.stats.altitude_m < two.stats.altitude_m,
            "{} vs {}",
            five.stats.altitude_m,
            two.stats.altitude_m
        );
        assert!(five.stats.gsd_cm < two.stats.gsd_cm);
        // One metre lower would no longer fit (unless already at the floor).
        if two.stats.altitude_m > base.min_alt_m {
            let lower = plan_at(&base, two.stats.altitude_m - 1.0).unwrap();
            assert!(lower.stats.batteries > 2);
        }
        assert!(
            !two.options.is_empty()
                && two
                    .options
                    .iter()
                    .any(|o| o.altitude_m == two.stats.altitude_m)
        );
        // Time goal behaves the same way.
        let t = plan(&PlanInput {
            goal: Goal::Time { minutes: 45.0 },
            ..base.clone()
        })
        .unwrap();
        assert!(t.stats.duration_s <= 45.0 * 60.0, "{}", t.stats.duration_s);
        // At the overview's own height the overview is dropped, not repeated.
        let top = plan_at(&base, 120.0).unwrap();
        assert_eq!(top.stats.overview_photos, 0);
        assert!(top.warnings.iter().any(|w| w.contains("No overview")));
        // An impossible budget still returns a plan, with a warning.
        let tiny = PlanInput {
            boundary: square(3000.0),
            goal: Goal::Batteries { count: 1 },
            ..Default::default()
        };
        let p = plan(&tiny).unwrap();
        assert!(p.warnings[0].contains("Even at"), "{:?}", p.warnings);
    }

    #[test]
    fn reads_google_earth_polygon() {
        let kml = r#"<kml><Placemark><Polygon><outerBoundaryIs><LinearRing><coordinates>
            151.0,-33.9,0 151.01,-33.9,0 151.01,-33.89,0 151.0,-33.9,0
        </coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark></kml>"#;
        let b = parse_kml_boundary(kml).unwrap();
        assert_eq!(b.len(), 3);
        assert_eq!(b[1], (151.01, -33.9));
    }
}
