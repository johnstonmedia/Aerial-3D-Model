//! Sparse reconstruction ("structure from motion", GPS-aided).
//!
//! Drone photos come with a good guess of their own pose: GPS gives the
//! position, the DJI gimbal record gives yaw/pitch, and for nadir photos
//! without gimbal data the 2D mosaic solve gives the yaw. So instead of
//! growing a model photo by photo from scratch, every camera starts where its
//! metadata puts it, and rounds of triangulate → bundle-adjust with tightening
//! outlier thresholds pull everything into agreement.

use super::ba::{bundle_adjust, Obs, Problem};
use super::geometry::{max_ray_angle, ransac_fundamental, triangulate, Intrinsics, Pose, V3};
use crate::geo::{to_utm, UtmZone};
use crate::imgops::{load_rgb, shrink_to};
use crate::matching::putative;
use crate::pipeline::{self, effective_profile, load_features, OBLIQUE_PITCH_DEG};
use crate::project::{PhotoEntry, Project};
use crate::Ctx;
use anyhow::{bail, Result};
use nalgebra::Vector2;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Camera {
    pub id: u32,
    pub path: PathBuf,
    pub key: String,
    pub group: usize,
    pub pose: Pose,
    pub oblique: bool,
    /// Other cameras that see the most of the same points (best first).
    pub neighbours: Vec<usize>,
    /// Depth range of this camera's sparse points (5th–95th percentile).
    pub depth_range: (f64, f64),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Sparse {
    pub zone: UtmZone,
    /// World origin: UTM easting/northing and the height reference.
    pub origin_e: f64,
    pub origin_n: f64,
    /// "takeoff" (heights relative to take-off) or "gps" (EXIF altitude).
    pub height_ref: String,
    pub intrinsics: Vec<Intrinsics>,
    pub cameras: Vec<Camera>,
    pub points: Vec<[f64; 3]>,
    pub colors: Vec<[u8; 3]>,
    pub rms_px: f64,
    pub skipped: Vec<u32>,
}

/// Sensor widths (mm) for cameras that don't record a 35 mm-equivalent focal
/// length. Bundle adjustment refines the focal length anyway; this only has
/// to be close.
fn sensor_width_mm(model: &str) -> Option<f64> {
    let m = model.to_ascii_uppercase();
    [
        ("DSC-WX220", 6.17),
        ("FC3170", 6.4),   // Mavic Air 2
        ("FC3411", 13.2),  // Air 2S
        ("FC220", 6.17),   // Mavic Pro
        ("FC6310", 13.2),  // Phantom 4 Pro
        ("FC7303", 6.3),   // Mini 2
        ("L1D-20C", 13.2), // Mavic 2 Pro
    ]
    .iter()
    .find(|(k, _)| m.contains(k))
    .map(|(_, w)| *w)
}

fn initial_focal_px(p: &PhotoEntry) -> f64 {
    let (w, h) = (p.meta.width as f64, p.meta.height as f64);
    if let Some(f35) = p.meta.focal_35mm {
        // 35 mm equivalence is defined on the diagonal (43.27 mm).
        return f35 / 43.2666 * (w * w + h * h).sqrt();
    }
    if let Some(fmm) = p.meta.focal_mm {
        let sw = p
            .meta
            .model
            .as_deref()
            .and_then(sensor_width_mm)
            .unwrap_or(6.17);
        return fmm / sw * w;
    }
    0.8 * w
}

fn group_key(p: &PhotoEntry) -> String {
    format!(
        "{}|{}|{}x{}|{:.2}",
        p.meta.make.as_deref().unwrap_or(""),
        p.meta.model.as_deref().unwrap_or(""),
        p.meta.width,
        p.meta.height,
        p.meta.focal_mm.unwrap_or(0.0)
    )
}

fn is_oblique(p: &PhotoEntry) -> bool {
    p.meta.pitch.is_some_and(|pt| pt > OBLIQUE_PITCH_DEG)
}

pub fn reconstruct_sparse(project: &Project, ctx: &Ctx) -> Result<Sparse> {
    let profile = effective_profile(project);
    let p3 = profile_3d(&profile);
    let mut photos: Vec<&PhotoEntry> = project
        .file
        .photos
        .iter()
        .filter(|p| p.enabled && p.path.exists())
        .collect();
    // Older projects predate the focal-length fields: re-read on demand.
    let refreshed: Vec<PhotoEntry> = photos
        .iter()
        .map(|p| {
            let mut q = (*p).clone();
            if q.meta.focal_mm.is_none() && q.meta.focal_35mm.is_none() {
                if let Ok(m) = crate::meta::read_meta(&q.path) {
                    q.meta = m;
                }
            }
            q
        })
        .collect();
    photos = refreshed.iter().collect();
    if photos.iter().filter(|p| p.meta.has_gps()).count() < 3 {
        bail!("3D needs GPS-tagged photos (at least three)");
    }

    // ── World frame ──
    let mut lons: Vec<f64> = photos.iter().filter_map(|p| p.meta.lon).collect();
    let mut lats: Vec<f64> = photos.iter().filter_map(|p| p.meta.lat).collect();
    lons.sort_by(f64::total_cmp);
    lats.sort_by(f64::total_cmp);
    let zone = UtmZone::for_lonlat(lons[lons.len() / 2], lats[lats.len() / 2]);
    let utm: Vec<Option<(f64, f64)>> = photos
        .iter()
        .map(|p| Some(to_utm(p.meta.lon?, p.meta.lat?, zone)))
        .collect();
    let n_gps = utm.iter().flatten().count() as f64;
    let e0 = (utm.iter().flatten().map(|v| v.0).sum::<f64>() / n_gps).round();
    let n0 = (utm.iter().flatten().map(|v| v.1).sum::<f64>() / n_gps).round();
    let use_rel = photos.iter().all(|p| p.meta.rel_alt.is_some());
    let height = |p: &PhotoEntry| if use_rel { p.meta.rel_alt } else { p.meta.alt };

    // ── 2D solve, for the yaw of nadir photos without gimbal data ──
    let needs_2d = photos
        .iter()
        .any(|p| !is_oblique(p) && p.meta.yaw.is_none());
    let mut yaw_2d: HashMap<u32, f64> = HashMap::new();
    let mut gsd_2d: Option<f64> = None;
    if needs_2d {
        ctx.report(
            "features",
            0,
            0,
            "Estimating camera directions from the 2D layout",
        );
        let nadir: Vec<&PhotoEntry> = photos.iter().copied().filter(|p| !is_oblique(p)).collect();
        pipeline::compute_features(project, &nadir, &profile, ctx)?;
        let pairs = pipeline::candidate_pairs(&nadir, project.file.settings.neighbours);
        let matches = pipeline::compute_matches(project, &nadir, &pairs, &profile, ctx)?;
        let al = pipeline::solve_alignment(project, &nadir, &pairs, &matches, &profile)?;
        let mut g: Vec<f64> = al.placed.iter().map(|pl| pl.gsd()).collect();
        g.sort_by(f64::total_cmp);
        gsd_2d = g.get(g.len() / 2).copied();
        for pl in &al.placed {
            // The 2D similarity rotates image-up by φ (counter-clockwise);
            // heading is clockwise from north, so yaw = −φ.
            yaw_2d.insert(pl.id, -pl.sim.angle().to_degrees());
        }
    }

    // ── Lens groups and initial poses ──
    let mut group_ids: HashMap<String, usize> = HashMap::new();
    let mut intrinsics: Vec<Intrinsics> = Vec::new();
    let mut cams: Vec<(usize, Pose, usize)> = Vec::new(); // (photo index, pose, group)
    let mut skipped = Vec::new();
    for (i, p) in photos.iter().enumerate() {
        let (Some((e, n)), Some(z)) = (utm[i], height(p)) else {
            skipped.push(p.id);
            continue;
        };
        let yaw = p.meta.yaw.or_else(|| yaw_2d.get(&p.id).copied());
        let Some(yaw) = yaw else {
            skipped.push(p.id);
            continue;
        };
        let pitch = p.meta.pitch.unwrap_or(-90.0);
        let key = group_key(p);
        let g = *group_ids.entry(key).or_insert_with(|| {
            intrinsics.push(Intrinsics {
                f: initial_focal_px(p),
                cx: p.meta.width as f64 / 2.0,
                cy: p.meta.height as f64 / 2.0,
                k1: 0.0,
                k2: 0.0,
                width: p.meta.width,
                height: p.meta.height,
            });
            intrinsics.len() - 1
        });
        let pose = Pose::from_angles(V3::new(e - e0, n - n0, z), yaw, pitch, 0.0);
        cams.push((i, pose, g));
    }
    if cams.len() < 3 {
        bail!("couldn't set up enough cameras for 3D (need GPS and a known direction)");
    }

    // ── Ground level, for pairing photos by what they look at ──
    let mut zs: Vec<f64> = cams.iter().map(|c| c.1.c.z).collect();
    zs.sort_by(f64::total_cmp);
    let agl = if use_rel {
        zs[zs.len() / 2]
    } else if let Some(g) = gsd_2d {
        g * intrinsics[0].f
    } else {
        60.0
    };
    let ground_z = zs[zs.len() / 2] - agl;
    let footprint: Vec<V3> = cams
        .iter()
        .map(|(_, pose, _)| {
            let a = pose.axis();
            if a.z < -0.2 {
                pose.c + a * ((ground_z - pose.c.z) / a.z).min(agl * 4.0)
            } else {
                pose.c + V3::new(a.x, a.y, 0.0).normalize() * agl * 2.0
            }
        })
        .collect();

    // ── Features for every photo, oblique ones included ──
    let all: Vec<&PhotoEntry> = cams.iter().map(|c| photos[c.0]).collect();
    pipeline::compute_features(project, &all, &p3, ctx)?;

    // ── Candidate pairs: nearest by footprint, similar enough view direction ──
    let k = project.file.settings.neighbours.max(8) + 4;
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    for i in 0..cams.len() {
        let mut d: Vec<(f64, usize)> = (0..cams.len())
            .filter(|&j| j != i)
            .filter(|&j| cams[i].1.axis().dot(&cams[j].1.axis()) > (75f64).to_radians().cos())
            .map(|j| ((footprint[i] - footprint[j]).norm_squared(), j))
            .collect();
        d.sort_by(|a, b| a.0.total_cmp(&b.0));
        for &(_, j) in d.iter().take(k) {
            pairs.push((i.min(j), i.max(j)));
        }
    }
    pairs.sort();
    pairs.dedup();

    // ── Verified matches (epipolar geometry, so oblique pairs work) ──
    let cache = project.cache_dir("matches3d");
    std::fs::create_dir_all(&cache)?;
    let done = AtomicUsize::new(0);
    let total = pairs.len();
    let matches: Vec<((usize, usize), Vec<(u32, u32)>)> = pairs
        .par_iter()
        .map(|&(i, j)| -> Result<((usize, usize), Vec<(u32, u32)>)> {
            ctx.check()?;
            let (a, b) = (all[i], all[j]);
            let path = cache.join(format!(
                "{}_{}_{}_{}.json",
                a.key, b.key, p3.work_size, p3.max_features
            ));
            let m: Vec<(u32, u32)> = match std::fs::read(&path)
                .ok()
                .and_then(|x| serde_json::from_slice(&x).ok())
            {
                Some(m) => m,
                None => {
                    let fa = load_features(project, a, &p3)?;
                    let fb = load_features(project, b, &p3)?;
                    let put = putative(&fa, &fb);
                    let pa: Vec<(f64, f64)> = put
                        .iter()
                        .map(|&(x, _)| (fa.kps[x].x as f64, fa.kps[x].y as f64))
                        .collect();
                    let pb: Vec<(f64, f64)> = put
                        .iter()
                        .map(|&(_, y)| (fb.kps[y].x as f64, fb.kps[y].y as f64))
                        .collect();
                    let inl = ransac_fundamental(&pa, &pb, 1.5, 600, (i * 7919 + j) as u64);
                    let m: Vec<(u32, u32)> = if inl.len() >= 20 {
                        inl.iter()
                            .map(|&t| (put[t].0 as u32, put[t].1 as u32))
                            .collect()
                    } else {
                        Vec::new()
                    };
                    std::fs::write(&path, serde_json::to_vec(&m)?)?;
                    m
                }
            };
            let n = done.fetch_add(1, Ordering::Relaxed) + 1;
            if n % 8 == 0 || n == total {
                ctx.report("matching3d", n, total, "Matching photos in 3D");
            }
            Ok(((i, j), m))
        })
        .collect::<Result<_>>()?;

    // ── Tracks ──
    ctx.report("tracks", 0, 1, "Linking matches into tracks");
    // Keep only keypoint positions (full-resolution pixels): descriptors are
    // done with, and positions are ~20 KB a photo even for big surveys.
    let kps: Vec<Vec<Vector2<f64>>> = all
        .iter()
        .map(|p| {
            let f = load_features(project, p, &p3)?;
            Ok(f.kps
                .iter()
                .map(|k| Vector2::new(k.x as f64 / f.scale, k.y as f64 / f.scale))
                .collect())
        })
        .collect::<Result<_>>()?;
    let mut matches = matches;
    let mut tracks = build_tracks(&kps, &matches);
    if tracks.len() < 50 {
        bail!(
            "too few matching points between photos to build 3D ({}) — are the photos overlapping?",
            tracks.len()
        );
    }

    // ── Rounds of triangulate → bundle adjust ──
    let mut poses: Vec<Pose> = cams.iter().map(|c| c.1).collect();
    let group: Vec<usize> = cams.iter().map(|c| c.2).collect();
    let prior: Vec<Option<(V3, V3)>> = cams
        .iter()
        .map(|c| Some((c.1.c, V3::new(3.0, 3.0, if use_rel { 2.0 } else { 6.0 }))))
        .collect();
    let work_to_full = 1.0 / profile_scale(&all, &p3);
    // The first round is deliberately loose. Metadata gives yaw well but not
    // the few degrees of tilt a drone has in flight, which at 100 m moves the
    // view by ~300 px; a tight first threshold would discard exactly the
    // matches needed to correct it. The Cauchy loss keeps wrong matches from
    // dominating meanwhile.
    let rounds = [(200.0, 20), (40.0, 15), (10.0, 15), (4.0, 20), (2.5, 25)];
    let verify_round = 2;
    let mut rms = 0.0;
    let mut kept: Vec<(V3, Vec<(usize, Vector2<f64>)>)> = Vec::new();
    for (ri, &(thr_work, iters)) in rounds.iter().enumerate() {
        ctx.check()?;
        let thr = thr_work * work_to_full;
        if ri == verify_round {
            // Cameras are roughly placed now: drop photo pairs whose matches
            // don't fit the camera geometry. Repetitive texture (grass, crops,
            // roofs) can fool the epipolar test between photos that don't even
            // overlap; those pairs otherwise leave stray points under the ground.
            matches.retain(|(_, m)| !m.is_empty());
            let before = matches.len();
            matches.retain(|((i, j), m)| {
                pair_fits(
                    &poses[*i],
                    &poses[*j],
                    &intrinsics[group[*i]],
                    &intrinsics[group[*j]],
                    &kps[*i],
                    &kps[*j],
                    m,
                ) < 6.0 * work_to_full
            });
            tracks = build_tracks(&kps, &matches);
            ctx.report(
                "bundle",
                ri,
                rounds.len(),
                format!(
                    "Dropped {} of {} photo pairs as inconsistent",
                    before - matches.len(),
                    before
                ),
            );
        }
        ctx.report(
            "bundle",
            ri,
            rounds.len(),
            format!("Refining cameras (round {} of {})", ri + 1, rounds.len()),
        );
        kept = tracks
            .par_iter()
            .filter_map(|t| {
                let obs: Vec<(Pose, Vector2<f64>)> = t
                    .iter()
                    .map(|(c, u)| (poses[*c], intrinsics[group[*c]].undistort(*u)))
                    .collect();
                let x = triangulate(&obs)?;
                let mut good: Vec<(usize, Vector2<f64>)> = Vec::new();
                for (c, u) in t {
                    if let Some(pu) = poses[*c].project(&intrinsics[group[*c]], &x) {
                        if (pu - u).norm() < thr {
                            good.push((*c, *u));
                        }
                    }
                }
                if good.len() < 2 {
                    return None;
                }
                // Depth from a narrow baseline is poorly determined: a point
                // seen by only two photos must be seen from clearly different
                // places. Without this, repeated texture (grass) matched along
                // the flight line scatters points far below the ground.
                let centres: Vec<V3> = good.iter().map(|(c, _)| poses[*c].c).collect();
                let angle = max_ray_angle(&x, &centres);
                let ok = if good.len() >= 3 {
                    angle > 2.0
                } else {
                    angle > 6.0
                };
                ok.then_some((x, good))
            })
            .collect();
        if kept.len() < 30 {
            bail!(
                "the photos didn't agree on enough 3D points ({} after round {})",
                kept.len(),
                ri + 1
            );
        }
        let mut obs = Vec::new();
        let mut points = Vec::with_capacity(kept.len());
        for (pi, (x, o)) in kept.iter().enumerate() {
            points.push(*x);
            for (c, u) in o {
                obs.push(Obs {
                    cam: *c as u32,
                    pt: pi as u32,
                    u: *u,
                });
            }
        }
        let mut prob = Problem {
            intr: intrinsics.clone(),
            poses: poses.clone(),
            group: group.clone(),
            prior: prior.clone(),
            points,
            obs,
            refine_intrinsics: ri >= 1,
        };
        let st = bundle_adjust(&mut prob, iters, thr / 2.0);
        ctx.report(
            "bundle",
            ri + 1,
            rounds.len(),
            format!(
                "{} points, error {:.1} → {:.1} px",
                kept.len(),
                st.initial_rms,
                st.final_rms
            ),
        );
        poses = prob.poses;
        intrinsics = prob.intr;
        for (k, x) in kept.iter_mut().zip(prob.points) {
            k.0 = x;
        }
        rms = st.final_rms;
    }

    // ── Neighbours and depth ranges for the dense stage ──
    let nc = cams.len();
    let mut shared: Vec<HashMap<usize, usize>> = vec![HashMap::new(); nc];
    let mut depths: Vec<Vec<f64>> = vec![Vec::new(); nc];
    for (x, o) in &kept {
        for (a, _) in o {
            depths[*a].push(poses[*a].to_cam(x).z);
            for (b, _) in o {
                if a != b {
                    *shared[*a].entry(*b).or_default() += 1;
                }
            }
        }
    }

    // ── Point colours from a downsized copy of one photo per point ──
    ctx.report("colour", 0, 1, "Colouring points");
    let mut by_cam: Vec<Vec<(usize, Vector2<f64>)>> = vec![Vec::new(); nc];
    for (pi, (_, o)) in kept.iter().enumerate() {
        by_cam[o[0].0].push((pi, o[0].1));
    }
    let mut colors = vec![[128u8; 3]; kept.len()];
    let painted: Vec<Vec<(usize, [u8; 3])>> = by_cam
        .par_iter()
        .enumerate()
        .map(|(c, list)| {
            if list.is_empty() {
                return Vec::new();
            }
            let Ok(img) = load_rgb(&all[c].path) else {
                return Vec::new();
            };
            let (small, s) = shrink_to(img, 1200);
            list.iter()
                .filter_map(|(pi, u)| {
                    let px = small.sample(u.x * s, u.y * s)?;
                    Some((*pi, [px[0] as u8, px[1] as u8, px[2] as u8]))
                })
                .collect()
        })
        .collect();
    for v in painted.into_iter().flatten() {
        colors[v.0] = v.1;
    }

    let cameras = cams
        .iter()
        .enumerate()
        .map(|(ci, (pi, _, g))| {
            let p = photos[*pi];
            let mut nb: Vec<(usize, usize)> = shared[ci].iter().map(|(k, v)| (*k, *v)).collect();
            nb.sort_by(|a, b| b.1.cmp(&a.1));
            let mut d = depths[ci].clone();
            d.sort_by(f64::total_cmp);
            let range = if d.len() >= 10 {
                (d[d.len() / 20] * 0.8, d[d.len() * 19 / 20] * 1.25)
            } else {
                (agl * 0.3, agl * 3.0)
            };
            Camera {
                id: p.id,
                path: p.path.clone(),
                key: p.key.clone(),
                group: *g,
                pose: poses[ci],
                oblique: is_oblique(p),
                neighbours: nb.into_iter().take(8).map(|x| x.0).collect(),
                depth_range: range,
            }
        })
        .collect();
    Ok(Sparse {
        zone,
        origin_e: e0,
        origin_n: n0,
        height_ref: if use_rel {
            "takeoff".into()
        } else {
            "gps".into()
        },
        intrinsics,
        cameras,
        points: kept.iter().map(|(x, _)| [x.x, x.y, x.z]).collect(),
        colors,
        rms_px: rms,
        skipped,
    })
}

/// Links pairwise matches into multi-photo tracks (union–find), keeping only
/// tracks seen at most once per photo.
fn build_tracks(
    kps: &[Vec<Vector2<f64>>],
    matches: &[((usize, usize), Vec<(u32, u32)>)],
) -> Vec<Vec<(usize, Vector2<f64>)>> {
    let mut offset = vec![0usize; kps.len() + 1];
    for i in 0..kps.len() {
        offset[i + 1] = offset[i] + kps[i].len();
    }
    let mut parent: Vec<u32> = (0..offset[kps.len()] as u32).collect();
    fn find(p: &mut [u32], x: u32) -> u32 {
        let mut r = x;
        while p[r as usize] != r {
            r = p[r as usize];
        }
        let mut c = x;
        while p[c as usize] != r {
            let n = p[c as usize];
            p[c as usize] = r;
            c = n;
        }
        r
    }
    let mut used = vec![false; parent.len()];
    for ((i, j), m) in matches {
        for &(a, b) in m {
            let na = (offset[*i] + a as usize) as u32;
            let nb = (offset[*j] + b as usize) as u32;
            used[na as usize] = true;
            used[nb as usize] = true;
            let (ra, rb) = (find(&mut parent, na), find(&mut parent, nb));
            if ra != rb {
                parent[ra as usize] = rb;
            }
        }
    }
    let mut groups: HashMap<u32, Vec<(usize, usize)>> = HashMap::new();
    for img in 0..kps.len() {
        for kp in 0..kps[img].len() {
            let node = (offset[img] + kp) as u32;
            if used[node as usize] {
                let r = find(&mut parent, node);
                groups.entry(r).or_default().push((img, kp));
            }
        }
    }
    groups
        .into_values()
        .filter(|t| t.len() >= 2)
        .filter(|t| {
            let mut imgs: Vec<usize> = t.iter().map(|x| x.0).collect();
            imgs.sort();
            imgs.windows(2).all(|w| w[0] != w[1])
        })
        .map(|t| t.into_iter().map(|(img, kp)| (img, kps[img][kp])).collect())
        .collect()
}

/// Median epipolar (Sampson) error, in pixels, of a pair's matches under the
/// current camera poses.
fn pair_fits(
    a: &Pose,
    b: &Pose,
    ka: &Intrinsics,
    kb: &Intrinsics,
    pa: &[Vector2<f64>],
    pb: &[Vector2<f64>],
    m: &[(u32, u32)],
) -> f64 {
    let r = b.r * a.r.transpose();
    let t = b.r * (a.c - b.c);
    let e = super::geometry::skew(&t) * r;
    let mut errs: Vec<f64> = m
        .iter()
        .map(|&(i, j)| {
            let x1 = ka.undistort(pa[i as usize]);
            let x2 = kb.undistort(pb[j as usize]);
            super::geometry::sampson(&e, (x1.x, x1.y), (x2.x, x2.y)).sqrt() * ka.f
        })
        .collect();
    if errs.is_empty() {
        return f64::MAX;
    }
    errs.sort_by(f64::total_cmp);
    errs[errs.len() / 2]
}

/// 3D needs more and finer tie points than the 2D map: depth comes from
/// small differences between photos, and points seen by only two photos leave
/// the model loose. Same memory class as the 2D profile (one decode per
/// thread), just more keypoints at a higher working size.
fn profile_3d(p: &crate::Profile) -> crate::Profile {
    let mut q = p.clone();
    let (ws, mf) = match p.kind {
        crate::ProfileKind::Lite => (1600, 4000),
        crate::ProfileKind::Standard => (2400, 7000),
        crate::ProfileKind::High => (3200, 10000),
    };
    q.work_size = ws;
    q.max_features = mf;
    q
}

/// Typical working/full scale across the photos (features are stored in
/// working pixels; thresholds are set in those units).
fn profile_scale(photos: &[&PhotoEntry], profile: &crate::Profile) -> f64 {
    let mut s: Vec<f64> = photos
        .iter()
        .map(|p| (profile.work_size as f64 / p.meta.width.max(p.meta.height) as f64).min(1.0))
        .collect();
    s.sort_by(f64::total_cmp);
    s[s.len() / 2]
}
