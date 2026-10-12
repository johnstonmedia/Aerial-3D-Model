//! The full processing run: features → pairs → matches → placement → tiles.
//! Every stage reads its cache first, so re-running after adding photos only
//! does the new work (plus a re-solve and re-render, which are cheap by
//! comparison).

use crate::features::{detect, Features};
use crate::geo::{to_utm, UtmZone};
use crate::imgops::{load_rgb, shrink_to};
use crate::matching::{match_pair, MatchParams, PairMatch};
use crate::profile::Profile;
use crate::project::{PhotoEntry, Project};
use crate::sim::Sim2;
use crate::solve::LinearSystem;
use crate::Ctx;
use anyhow::{bail, Context, Result};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Frame {
    /// UTM metres relative to (origin_e, origin_n); y is north.
    Utm {
        zone: crate::geo::UtmZone,
        origin_e: f64,
        origin_n: f64,
    },
    /// No GPS: units are one working pixel of the anchor photo.
    Local,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Placed {
    pub id: u32,
    pub key: String,
    pub path: PathBuf,
    /// Full-resolution size.
    pub width: u32,
    pub height: u32,
    /// working / full scale used when the features were computed.
    pub work_scale: f64,
    /// Centred, y-up working coordinates → frame.
    pub sim: Sim2,
}

impl Placed {
    /// Full-resolution pixel → frame.
    pub fn full_to_frame(&self, x: f64, y: f64) -> (f64, f64) {
        let u = (x - self.width as f64 / 2.0) * self.work_scale;
        let v = -(y - self.height as f64 / 2.0) * self.work_scale;
        self.sim.apply(u, v)
    }

    /// Frame → full-resolution pixel.
    pub fn frame_to_full(&self, inv: &Sim2, fx: f64, fy: f64) -> (f64, f64) {
        let (u, v) = inv.apply(fx, fy);
        (
            u / self.work_scale + self.width as f64 / 2.0,
            -v / self.work_scale + self.height as f64 / 2.0,
        )
    }

    /// Frame units per full-resolution pixel.
    pub fn gsd(&self) -> f64 {
        self.sim.scale() * self.work_scale
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Alignment {
    pub frame: Frame,
    pub placed: Vec<Placed>,
    pub unplaced: Vec<u32>,
    pub pairs_tried: usize,
    pub pairs_matched: usize,
    /// RMS reprojection error over all matches, in working pixels.
    pub rms_px: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct Summary {
    pub photos: usize,
    pub placed: usize,
    pub unplaced: usize,
    pub pairs_tried: usize,
    pub pairs_matched: usize,
    pub rms_px: f64,
    pub width: u64,
    pub height: u64,
    pub gsd: f64,
    pub georeferenced: bool,
    /// Angled photos left out of the 2D map.
    pub oblique_skipped: usize,
}

/// Photos tilted less steeply than this are treated as oblique and left out
/// of the 2D map. -90 is straight down.
pub const OBLIQUE_PITCH_DEG: f64 = -75.0;

pub fn effective_profile(project: &Project) -> Profile {
    match project.file.settings.profile {
        Some(k) => Profile::for_kind(k).fit_platform(),
        None => Profile::detect(),
    }
}

pub fn process(project: &Project, ctx: &Ctx) -> Result<Summary> {
    let profile = effective_profile(project);
    crate::with_pool(profile.threads, || {
        let usable: Vec<&PhotoEntry> = project
            .file
            .photos
            .iter()
            .filter(|p| p.enabled && p.path.exists())
            .collect();
        // Angled (oblique) shots are for 3D; they'd smear a flat map. DJI
        // records the gimbal pitch, so they can be told apart reliably.
        let (photos, oblique): (Vec<&PhotoEntry>, Vec<&PhotoEntry>) =
            usable.into_iter().partition(|p| {
                p.meta
                    .pitch
                    .map_or(true, |pitch| pitch <= OBLIQUE_PITCH_DEG)
            });
        if !oblique.is_empty() {
            ctx.report(
                "features",
                0,
                0,
                format!("Skipping {} angled photos (kept for 3D)", oblique.len()),
            );
        }
        if photos.len() < 2 {
            bail!(if oblique.is_empty() {
                "add at least two photos"
            } else {
                "the map needs straight-down photos; these are all angled"
            });
        }
        compute_features(project, &photos, &profile, ctx)?;
        let pairs = candidate_pairs(&photos, project.file.settings.neighbours);
        let matches = compute_matches(project, &photos, &pairs, &profile, ctx)?;
        ctx.report("solve", 0, 1, "Placing photos");
        let alignment = solve_alignment(project, &photos, &pairs, &matches, &profile)?;
        std::fs::create_dir_all(project.result_dir())?;
        std::fs::write(
            project.result_dir().join("alignment.json"),
            serde_json::to_vec(&alignment)?,
        )?;
        ctx.report(
            "solve",
            1,
            1,
            format!(
                "Placed {} of {} photos",
                alignment.placed.len(),
                photos.len()
            ),
        );
        let manifest = crate::render::render(project, &alignment, &profile, ctx)?;
        Ok(Summary {
            photos: photos.len(),
            placed: alignment.placed.len(),
            unplaced: alignment.unplaced.len(),
            pairs_tried: alignment.pairs_tried,
            pairs_matched: alignment.pairs_matched,
            rms_px: alignment.rms_px,
            width: manifest.width,
            height: manifest.height,
            gsd: manifest.gsd,
            georeferenced: manifest.epsg.is_some(),
            oblique_skipped: oblique.len(),
        })
    })
}

pub(crate) fn feature_path(project: &Project, key: &str, profile: &Profile) -> PathBuf {
    project.cache_dir("features").join(format!(
        "{key}_{}_{}.af",
        profile.work_size, profile.max_features
    ))
}

pub(crate) fn compute_features(
    project: &Project,
    photos: &[&PhotoEntry],
    profile: &Profile,
    ctx: &Ctx,
) -> Result<()> {
    std::fs::create_dir_all(project.cache_dir("features"))?;
    let todo: Vec<&&PhotoEntry> = photos
        .iter()
        .filter(|p| !feature_path(project, &p.key, profile).exists())
        .collect();
    let total = photos.len();
    let done = AtomicUsize::new(total - todo.len());
    ctx.report(
        "features",
        done.load(Ordering::Relaxed),
        total,
        "Finding features",
    );
    todo.par_iter().try_for_each(|p| -> Result<()> {
        ctx.check()?;
        let rgb = load_rgb(&p.path)?;
        let (small, scale) = shrink_to(rgb, profile.work_size as usize);
        let gray = small.to_gray();
        drop(small);
        let f = detect(&gray, profile.max_features, scale);
        let path = feature_path(project, &p.key, profile);
        let tmp = path.with_extension("tmp");
        let mut buf = Vec::new();
        f.write(&mut buf)?;
        std::fs::write(&tmp, buf)?;
        std::fs::rename(&tmp, &path)?;
        let n = done.fetch_add(1, Ordering::Relaxed) + 1;
        ctx.report(
            "features",
            n,
            total,
            format!("{} keypoints in {}", f.kps.len(), file_name(&p.path)),
        );
        Ok(())
    })
}

pub(crate) fn load_features(
    project: &Project,
    p: &PhotoEntry,
    profile: &Profile,
) -> Result<Features> {
    let path = feature_path(project, &p.key, profile);
    Features::read(
        std::fs::File::open(&path)
            .with_context(|| format!("missing features for {}", file_name(&p.path)))?,
    )
}

pub(crate) fn file_name(p: &std::path::Path) -> String {
    p.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default()
}

/// The UTM zone of the survey and each photo's position in it, relative to
/// the survey's mean position.
pub(crate) fn gps_positions(
    photos: &[&PhotoEntry],
) -> Option<(UtmZone, f64, f64, Vec<Option<(f64, f64)>>)> {
    let with: Vec<(f64, f64)> = photos
        .iter()
        .filter_map(|p| Some((p.meta.lon?, p.meta.lat?)))
        .collect();
    if with.len() < 2 {
        return None;
    }
    let mut lons: Vec<f64> = with.iter().map(|v| v.0).collect();
    let mut lats: Vec<f64> = with.iter().map(|v| v.1).collect();
    lons.sort_by(f64::total_cmp);
    lats.sort_by(f64::total_cmp);
    let zone = UtmZone::for_lonlat(lons[lons.len() / 2], lats[lats.len() / 2]);
    let abs: Vec<Option<(f64, f64)>> = photos
        .iter()
        .map(|p| Some(to_utm(p.meta.lon?, p.meta.lat?, zone)))
        .collect();
    let n = abs.iter().flatten().count() as f64;
    let e0 = (abs.iter().flatten().map(|v| v.0).sum::<f64>() / n).round();
    let n0 = (abs.iter().flatten().map(|v| v.1).sum::<f64>() / n).round();
    let rel = abs
        .into_iter()
        .map(|v| v.map(|(e, n)| (e - e0, n - n0)))
        .collect();
    Some((zone, e0, n0, rel))
}

/// Candidate pairs, as indices into `photos` with i < j.
pub(crate) fn candidate_pairs(photos: &[&PhotoEntry], k: usize) -> Vec<(usize, usize)> {
    let mut set = HashSet::new();
    let n = photos.len();
    if let Some((_, _, _, pos)) = gps_positions(photos) {
        // GPS neighbours: the k nearest. On a lawnmower survey that reaches
        // the adjacent flight lines as well as the photos either side.
        let pts: Vec<(usize, (f64, f64))> = pos
            .iter()
            .enumerate()
            .filter_map(|(i, p)| p.map(|p| (i, p)))
            .collect();
        for &(i, pi) in &pts {
            let mut d: Vec<(f64, usize)> = pts
                .iter()
                .filter(|(j, _)| *j != i)
                .map(|&(j, pj)| ((pi.0 - pj.0).powi(2) + (pi.1 - pj.1).powi(2), j))
                .collect();
            let kk = k.min(d.len());
            if kk == 0 {
                continue;
            }
            d.select_nth_unstable_by(kk - 1, |a, b| a.0.total_cmp(&b.0));
            d.truncate(kk);
            for (_, j) in d {
                set.insert((i.min(j), i.max(j)));
            }
        }
        // Photos without GPS fall back to capture-order neighbours.
        for i in 0..n {
            if pos[i].is_none() {
                for j in i.saturating_sub(3)..(i + 4).min(n) {
                    if j != i {
                        set.insert((i.min(j), i.max(j)));
                    }
                }
            }
        }
    } else {
        for i in 0..n {
            for j in i + 1..(i + 1 + k.max(3)).min(n) {
                set.insert((i, j));
            }
        }
    }
    let mut v: Vec<_> = set.into_iter().collect();
    v.sort();
    v
}

fn match_path(project: &Project, a: &PhotoEntry, b: &PhotoEntry, profile: &Profile) -> PathBuf {
    project
        .cache_dir("matches")
        .join(format!("{}_{}_{}.json", a.key, b.key, profile.work_size))
}

pub(crate) fn compute_matches(
    project: &Project,
    photos: &[&PhotoEntry],
    pairs: &[(usize, usize)],
    profile: &Profile,
    ctx: &Ctx,
) -> Result<HashMap<(usize, usize), PairMatch>> {
    std::fs::create_dir_all(project.cache_dir("matches"))?;
    let total = pairs.len();
    let done = AtomicUsize::new(0);
    let params = MatchParams::default();
    ctx.report("matching", 0, total, "Matching neighbouring photos");
    let results: Vec<((usize, usize), Option<PairMatch>)> = pairs
        .par_iter()
        .map(|&(i, j)| -> Result<((usize, usize), Option<PairMatch>)> {
            ctx.check()?;
            let path = match_path(project, photos[i], photos[j], profile);
            let m = match std::fs::read(&path)
                .ok()
                .and_then(|b| serde_json::from_slice(&b).ok())
            {
                Some(cached) => cached,
                None => {
                    let fa = load_features(project, photos[i], profile)?;
                    let fb = load_features(project, photos[j], profile)?;
                    let seed =
                        (i as u64).wrapping_mul(0x9E37_79B9) ^ (j as u64).wrapping_mul(0x85EB_CA6B);
                    let m = match_pair(&fa, &fb, seed, &params);
                    std::fs::write(&path, serde_json::to_vec(&m)?)?;
                    m
                }
            };
            let n = done.fetch_add(1, Ordering::Relaxed) + 1;
            if n % 8 == 0 || n == total {
                ctx.report("matching", n, total, "Matching neighbouring photos");
            }
            Ok(((i, j), m))
        })
        .collect::<Result<_>>()?;
    Ok(results
        .into_iter()
        .filter_map(|(k, m)| m.map(|m| (k, m)))
        .collect())
}

struct Dsu(Vec<usize>);
impl Dsu {
    fn find(&mut self, x: usize) -> usize {
        let mut r = x;
        while self.0[r] != r {
            r = self.0[r];
        }
        let mut c = x;
        while self.0[c] != r {
            let n = self.0[c];
            self.0[c] = r;
            c = n;
        }
        r
    }
    fn union(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a] = b;
        }
    }
}

pub(crate) fn solve_alignment(
    project: &Project,
    photos: &[&PhotoEntry],
    pairs: &[(usize, usize)],
    matches: &HashMap<(usize, usize), PairMatch>,
    profile: &Profile,
) -> Result<Alignment> {
    let n = photos.len();
    // Working sizes, read from the feature headers.
    let mut work: Vec<(f64, f64, f64)> = Vec::with_capacity(n); // (w, h, scale)
    for p in photos {
        let f = load_features(project, p, profile)?;
        work.push((f.w as f64, f.h as f64, f.scale));
    }
    let centred =
        |i: usize, x: f32, y: f32| (x as f64 - work[i].0 / 2.0, -(y as f64 - work[i].1 / 2.0));

    // Pairwise similarities in centred y-up coordinates, plus sampled points.
    struct Edge {
        i: usize,
        j: usize,
        sim: Sim2, // i → j
        pts: Vec<((f64, f64), (f64, f64))>,
    }
    let mut edges = Vec::new();
    for (&(i, j), m) in matches {
        let pts: Vec<_> = m
            .points
            .iter()
            .map(|p| (centred(i, p[0], p[1]), centred(j, p[2], p[3])))
            .collect();
        let src: Vec<_> = pts.iter().map(|p| p.0).collect();
        let dst: Vec<_> = pts.iter().map(|p| p.1).collect();
        let Some(sim) = Sim2::fit(&src, &dst) else {
            continue;
        };
        let step = (pts.len() / 40).max(1);
        let pts = pts.into_iter().step_by(step).collect();
        edges.push(Edge { i, j, sim, pts });
    }
    edges.sort_by_key(|e| (e.i, e.j));

    // Connected components and a spanning-tree initialisation within each.
    let mut dsu = Dsu((0..n).collect());
    for e in &edges {
        dsu.union(e.i, e.j);
    }
    let mut adj: Vec<Vec<(usize, Sim2)>> = vec![Vec::new(); n];
    for e in &edges {
        adj[e.i].push((e.j, e.sim));
        adj[e.j].push((e.i, e.sim.inverse()));
    }
    let mut init: Vec<Option<Sim2>> = vec![None; n];
    let mut comp_of = vec![usize::MAX; n];
    let mut comps: Vec<Vec<usize>> = Vec::new();
    for root in 0..n {
        if init[root].is_some() {
            continue;
        }
        let cid = comps.len();
        let mut members = vec![root];
        init[root] = Some(Sim2::IDENTITY);
        comp_of[root] = cid;
        let mut q = std::collections::VecDeque::from([root]);
        while let Some(i) = q.pop_front() {
            let ti = init[i].unwrap();
            for &(j, s_ij) in &adj[i] {
                if init[j].is_none() {
                    // p_i = s_ij⁻¹(p_j)  ⇒  T_j = T_i ∘ s_ij⁻¹
                    init[j] = Some(ti.compose(&s_ij.inverse()));
                    comp_of[j] = cid;
                    members.push(j);
                    q.push_back(j);
                }
            }
        }
        comps.push(members);
    }
    let mut init: Vec<Sim2> = init.into_iter().map(|s| s.unwrap()).collect();

    let gps = gps_positions(photos);
    let mut placed_mask = vec![false; n];
    let frame;
    match &gps {
        Some((zone, e0, n0, pos)) => {
            frame = Frame::Utm {
                zone: *zone,
                origin_e: *e0,
                origin_n: *n0,
            };
            // Bring each component into UTM using its photos' GPS.
            let mut solo_scale = Vec::new();
            for members in &comps {
                let src: Vec<_> = members
                    .iter()
                    .filter(|&&i| pos[i].is_some())
                    .map(|&i| (init[i].tx, init[i].ty))
                    .collect();
                let dst: Vec<_> = members.iter().filter_map(|&i| pos[i]).collect();
                let spread = dst
                    .iter()
                    .map(|d| (d.0 - dst[0].0).hypot(d.1 - dst[0].1))
                    .fold(0.0, f64::max);
                if members.len() >= 2 && src.len() >= 2 && spread > 5.0 {
                    if let Some(g) = Sim2::fit(&src, &dst) {
                        for &i in members {
                            init[i] = g.compose(&init[i]);
                            placed_mask[i] = true;
                        }
                        solo_scale.push(init[members[0]].scale());
                    }
                }
            }
            // Isolated photos (or tiny components) with GPS: place by GPS
            // with the survey's typical scale and the gimbal yaw if known.
            solo_scale.sort_by(f64::total_cmp);
            let typical = solo_scale.get(solo_scale.len() / 2).copied();
            if let Some(scale) = typical {
                for members in &comps {
                    if placed_mask[members[0]] {
                        continue;
                    }
                    for &i in members {
                        if let Some((e, nn)) = pos[i] {
                            // Image "up" points along the gimbal yaw (clockwise from north).
                            let rot = photos[i].meta.yaw.map(|y| -y.to_radians()).unwrap_or(0.0);
                            init[i] = Sim2 {
                                a: scale * rot.cos(),
                                b: scale * rot.sin(),
                                tx: e,
                                ty: nn,
                            };
                            placed_mask[i] = true;
                        }
                    }
                }
            }
        }
        None => {
            frame = Frame::Local;
            let biggest = comps.iter().max_by_key(|c| c.len()).unwrap();
            for &i in biggest {
                placed_mask[i] = true;
            }
        }
    }

    // Global refinement over everything placed.
    let index: Vec<Option<usize>> = {
        let mut next = 0;
        placed_mask
            .iter()
            .map(|&p| {
                p.then(|| {
                    next += 1;
                    next - 1
                })
            })
            .collect()
    };
    let m = placed_mask.iter().filter(|&&p| p).count();
    if m < 2 {
        bail!("couldn't place the photos: they don't overlap enough and have no GPS");
    }
    let mut scales: Vec<f64> = (0..n)
        .filter(|&i| placed_mask[i])
        .map(|i| init[i].scale())
        .collect();
    scales.sort_by(f64::total_cmp);
    let s_med = scales[scales.len() / 2];
    let mut sys = LinearSystem::new(m);
    let w_match = 1.0 / s_med; // 1 working pixel of error ≈ 1 unit of cost
    for e in &edges {
        let (Some(a), Some(b)) = (index[e.i], index[e.j]) else {
            continue;
        };
        for (pi, pj) in &e.pts {
            sys.add_match(a, *pi, b, *pj, w_match);
        }
    }
    let mut x0 = vec![Sim2::IDENTITY; m];
    for i in 0..n {
        if let Some(k) = index[i] {
            x0[k] = init[i];
            // Very weak: only decisive for photos nothing else constrains.
            sys.add_rotation_prior(k, init[i].a, init[i].b, 1e-3 / s_med);
        }
    }
    // Fix each component's gauge by pinning its best-connected photo where the
    // initialisation put it. Without this the problem has a degenerate
    // optimum — every photo shrunk to a point — which GPS priors alone can't
    // prevent, because shrinking makes every match residual vanish.
    let mut degree = vec![0usize; n];
    for e in &edges {
        degree[e.i] += 1;
        degree[e.j] += 1;
    }
    for members in &comps {
        let Some(&anchor) = members
            .iter()
            .filter(|&&i| placed_mask[i])
            .max_by_key(|&&i| degree[i])
        else {
            continue;
        };
        let k = index[anchor].unwrap();
        sys.add_position(k, init[anchor].tx, init[anchor].ty, 1e3);
        sys.add_rotation_prior(k, init[anchor].a, init[anchor].b, 1e3 / s_med);
    }
    let mut solved = sys.solve(&x0, 4000);

    // With GPS, re-fit each component onto its photos' GPS positions so any
    // error is spread evenly rather than all charged to the pinned photo.
    if let Some((_, _, _, pos)) = &gps {
        for members in &comps {
            let ks: Vec<(usize, (f64, f64))> = members
                .iter()
                .filter_map(|&i| Some((index[i]?, pos[i]?)))
                .collect();
            let spread = ks
                .iter()
                .map(|(_, d)| (d.0 - ks[0].1 .0).hypot(d.1 - ks[0].1 .1))
                .fold(0.0, f64::max);
            if ks.len() < 3 || spread < 5.0 {
                continue;
            }
            let src: Vec<_> = ks
                .iter()
                .map(|&(k, _)| (solved[k].tx, solved[k].ty))
                .collect();
            let dst: Vec<_> = ks.iter().map(|&(_, d)| d).collect();
            if let Some(g) = Sim2::fit(&src, &dst) {
                for &i in members {
                    if let Some(k) = index[i] {
                        solved[k] = g.compose(&solved[k]);
                    }
                }
            }
        }
    }

    // RMS over matches, in working pixels of photo i.
    let (mut se, mut cnt) = (0.0, 0usize);
    for e in &edges {
        let (Some(a), Some(b)) = (index[e.i], index[e.j]) else {
            continue;
        };
        for (pi, pj) in &e.pts {
            let p = solved[a].apply(pi.0, pi.1);
            let q = solved[b].apply(pj.0, pj.1);
            let d = ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2)) / solved[a].scale().powi(2);
            se += d;
            cnt += 1;
        }
    }

    let mut placed = Vec::with_capacity(m);
    let mut unplaced = Vec::new();
    for (i, p) in photos.iter().enumerate() {
        match index[i] {
            Some(k) => placed.push(Placed {
                id: p.id,
                key: p.key.clone(),
                path: p.path.clone(),
                width: p.meta.width,
                height: p.meta.height,
                work_scale: work[i].2,
                sim: solved[k],
            }),
            None => unplaced.push(p.id),
        }
    }
    let _ = comp_of;
    Ok(Alignment {
        frame,
        placed,
        unplaced,
        pairs_tried: pairs.len(),
        pairs_matched: edges.len(),
        rms_px: if cnt > 0 {
            (se / cnt as f64).sqrt()
        } else {
            0.0
        },
    })
}
