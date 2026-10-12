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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanInput {
    pub name: String,
    /// Boundary as (lon, lat) pairs, not closed.
    pub boundary: Vec<(f64, f64)>,
    pub camera: Camera,
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
    /// Photos at each waypoint (PhotoAtWaypoint mode only).
    #[serde(default = "nadir_only")]
    pub shots: Vec<Shot>,
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
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub lon: f64,
    pub lat: f64,
    /// Degrees clockwise from north, 0–360.
    pub heading: f64,
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
    pub gsd_cm: f64,
    pub footprint_w_m: f64,
    pub footprint_h_m: f64,
    pub line_spacing_m: f64,
    pub photo_spacing_m: f64,
    pub speed_ms: f64,
    pub direction_deg: f64,
    pub lines: usize,
    pub photos: usize,
    pub distance_m: f64,
    pub duration_s: f64,
    pub missions: usize,
    /// Batteries needed when short missions share a battery (flown in order;
    /// a mission never spans two batteries).
    pub batteries: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Plan {
    pub missions: Vec<Mission>,
    pub stats: PlanStats,
    pub warnings: Vec<String>,
}

/// Seconds lost per photo stop: decelerate, settle, shoot, accelerate.
const STOP_OVERHEAD_S: f64 = 3.0;
/// Extra seconds for each additional shot at a waypoint (turn/tilt + shoot).
const EXTRA_SHOT_S: f64 = 3.5;
/// Seconds lost per turn at a line end.
const TURN_OVERHEAD_S: f64 = 4.0;

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

/// Bearing (deg clockwise from north) of a vector in east/north metres.
fn bearing(dx: f64, dy: f64) -> f64 {
    dx.atan2(dy).to_degrees().rem_euclid(360.0)
}

pub fn plan(input: &PlanInput) -> Result<Plan> {
    let b = &input.boundary;
    if b.len() < 3 {
        bail!("draw at least three corners of the area");
    }
    let cam = &input.camera;
    if cam.focal_mm <= 0.0 || cam.sensor_w_mm <= 0.0 || cam.sensor_h_mm <= 0.0 {
        bail!("camera settings are incomplete");
    }
    if !(5.0..=500.0).contains(&input.altitude_m) {
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

    // Ground footprint of one photo; long side across the line.
    let fw = input.altitude_m * cam.sensor_w_mm / cam.focal_mm;
    let fh = input.altitude_m * cam.sensor_h_mm / cam.focal_mm;
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

    let unproject = |x: f64, y: f64| {
        let (e, n) = to_world(x, y);
        from_utm(e + e0, n + n0, zone)
    };

    // Build per-line waypoint lists.
    struct LineWps {
        wps: Vec<Waypoint>,
        len: f64,
        photos: usize,
    }
    let mut built: Vec<LineWps> = Vec::new();
    for &(x0, x1, y) in &lines {
        let len = (x1 - x0).abs();
        let dir = (x1 - x0).signum();
        let heading = bearing(ux * dir, uy * dir);
        let mut wps = Vec::new();
        let photos;
        match input.capture {
            CaptureMode::PhotoAtWaypoint => {
                let n = (len / photo_spacing).ceil() as usize + 1;
                let step = if n > 1 { len / (n - 1) as f64 } else { 0.0 };
                for i in 0..n {
                    let (lon, lat) = unproject(x0 + dir * step * i as f64, y);
                    wps.push(Waypoint {
                        lon,
                        lat,
                        heading,
                        photo: true,
                        photo_dist_m: 0.0,
                        first: false,
                    });
                }
                photos = n * input.shots.len().max(1);
            }
            CaptureMode::DistanceInterval => {
                let (lon, lat) = unproject(x0, y);
                wps.push(Waypoint {
                    lon,
                    lat,
                    heading,
                    photo: false,
                    photo_dist_m: photo_spacing,
                    first: false,
                });
                let (lon, lat) = unproject(x1, y);
                wps.push(Waypoint {
                    lon,
                    lat,
                    heading,
                    photo: false,
                    photo_dist_m: 0.0,
                    first: false,
                });
                photos = (len / photo_spacing).floor() as usize + 1;
            }
        }
        built.push(LineWps { wps, len, photos });
    }
    if built.is_empty() {
        bail!("the area is too small for this height — fly lower or draw a larger area");
    }

    // Split into missions by Litchi's waypoint limit and battery time.
    let battery_s = input.battery_minutes.max(3.0) * 60.0;
    let line_time = |l: &LineWps| {
        let extra = (input.shots.len().max(1) - 1) as f64 * EXTRA_SHOT_S;
        let stops = if input.capture == CaptureMode::PhotoAtWaypoint {
            l.wps.len() as f64 * (STOP_OVERHEAD_S + extra)
        } else {
            0.0
        };
        l.len / speed + stops + TURN_OVERHEAD_S
    };
    let mut missions: Vec<Mission> = Vec::new();
    let mut cur = Mission {
        waypoints: Vec::new(),
        lines: 0,
        photos: 0,
        distance_m: 0.0,
        duration_s: 0.0,
    };
    for (k, l) in built.iter().enumerate() {
        let transit = if cur.lines > 0 { line_spacing } else { 0.0 };
        let add_t = line_time(l) + transit / speed;
        if l.wps.len() > LITCHI_MAX_WAYPOINTS {
            bail!(
                "one flight line needs {} waypoints, over Litchi's limit of {LITCHI_MAX_WAYPOINTS} — \
                 fly higher, use the distance-interval mode, or draw a smaller area",
                l.wps.len()
            );
        }
        let over_wp = cur.waypoints.len() + l.wps.len() > LITCHI_MAX_WAYPOINTS;
        let over_t = cur.duration_s + add_t > battery_s;
        if cur.lines > 0 && (over_wp || over_t) {
            missions.push(std::mem::replace(
                &mut cur,
                Mission {
                    waypoints: Vec::new(),
                    lines: 0,
                    photos: 0,
                    distance_m: 0.0,
                    duration_s: 0.0,
                },
            ));
        }
        let transit = if cur.lines > 0 { line_spacing } else { 0.0 };
        cur.waypoints.extend(l.wps.iter().cloned());
        cur.lines += 1;
        cur.photos += l.photos;
        cur.distance_m += l.len + transit;
        cur.duration_s += line_time(l) + transit / speed;
        if cur.duration_s > battery_s && cur.lines == 1 {
            warnings.push(format!(
                "Line {} alone takes longer than one battery — consider flying higher or faster.",
                k + 1
            ));
        }
    }
    if cur.lines > 0 {
        missions.push(cur);
    }
    for m in &mut missions {
        if let Some(w) = m.waypoints.first_mut() {
            w.first = true;
        }
    }

    if input.altitude_m > LEGAL_MAX_ALT_M {
        warnings.push(format!(
            "{:.0} m is above the usual {LEGAL_MAX_ALT_M:.0} m limit (CASA in Australia). Check you're allowed to fly this high.",
            input.altitude_m
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
            "Angled photos are for 3D models. The 2D map uses only the straight-down photos and skips the rest."
                .into(),
        );
        if !input.shots.iter().any(|s| s.pitch_deg <= -80.0) {
            warnings.push(
                "No straight-down photo is included, so these photos can't make a 2D map.".into(),
            );
        }
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

    let stats = PlanStats {
        area_m2: area,
        gsd_cm,
        footprint_w_m: fw,
        footprint_h_m: fh,
        line_spacing_m: line_spacing,
        photo_spacing_m: photo_spacing,
        speed_ms: speed,
        direction_deg: direction,
        lines: built.len(),
        photos: missions.iter().map(|m| m.photos).sum(),
        distance_m: missions.iter().map(|m| m.distance_m).sum(),
        duration_s: missions.iter().map(|m| m.duration_s).sum(),
        missions: missions.len(),
        batteries,
    };
    Ok(Plan {
        missions,
        stats,
        warnings,
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
        let actions = if w.photo {
            waypoint_actions(input, w.first, w.heading)
        } else if w.first {
            vec![(5, -90.0), (0, 2000.0)]
        } else {
            Vec::new()
        };
        // gimbalmode 2 = interpolate: hold the pitch below between waypoints.
        out.push_str(&format!(
            "{:.8},{:.8},{:.1},{:.1},0,0,2,-90",
            w.lat, w.lon, input.altitude_m, w.heading
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
            .map(|w| format!("{:.8},{:.8},{:.1}", w.lon, w.lat, input.altitude_m))
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
