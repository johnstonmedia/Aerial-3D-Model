//! Dense reconstruction: a depth for (nearly) every pixel of every photo, by
//! PatchMatch stereo, then fused into one coloured point cloud.
//!
//! Memory stays bounded: photos are reduced once to a small working copy on
//! disk, each depth map is computed from its photo and a few neighbours and
//! written to disk, and fusion streams depth maps back one photo at a time
//! into a voxel grid whose cell size is chosen so the cloud fits the profile.

use super::geometry::{Intrinsics, Pose, V3};
use super::sparse::Sparse;
use crate::imgops::{load_rgb, shrink_to};
use crate::profile::{Profile, ProfileKind};
use crate::project::Project;
use crate::Ctx;
use anyhow::{bail, Result};
use nalgebra::Vector2;
use rayon::prelude::*;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

pub struct DenseParams {
    /// Long side of the working images, pixels.
    pub size: usize,
    pub iterations: usize,
    pub neighbours: usize,
    /// Upper bound on points in the fused cloud.
    pub max_points: usize,
}

impl DenseParams {
    pub fn for_profile(p: &Profile) -> Self {
        match p.kind {
            ProfileKind::Lite => Self { size: 640, iterations: 3, neighbours: 3, max_points: 2_000_000 },
            ProfileKind::Standard => Self { size: 960, iterations: 4, neighbours: 4, max_points: 5_000_000 },
            ProfileKind::High => Self { size: 1400, iterations: 5, neighbours: 5, max_points: 12_000_000 },
        }
    }
}

/// A photo reduced for dense matching.
struct View {
    w: usize,
    h: usize,
    gray: Vec<f32>,
    rgb: Vec<u8>,
    k: Intrinsics,
    pose: Pose,
}

impl View {
    #[inline]
    fn gray_at(&self, x: f32, y: f32) -> Option<f32> {
        let fx = x - 0.5;
        let fy = y - 0.5;
        if fx < 0.0 || fy < 0.0 {
            return None;
        }
        let (x0, y0) = (fx as usize, fy as usize);
        if x0 + 1 >= self.w || y0 + 1 >= self.h {
            return None;
        }
        let (ax, ay) = (fx - x0 as f32, fy - y0 as f32);
        let i = y0 * self.w + x0;
        let g = &self.gray;
        let top = g[i] * (1.0 - ax) + g[i + 1] * ax;
        let bot = g[i + self.w] * (1.0 - ax) + g[i + self.w + 1] * ax;
        Some(top * (1.0 - ay) + bot * ay)
    }
}

fn view_path(dir: &PathBuf, key: &str, size: usize) -> PathBuf {
    dir.join(format!("{key}_{size}.view"))
}

/// Builds (or reuses) the reduced copy of each photo: grey f32 + RGB u8.
fn prepare_views(project: &Project, sp: &Sparse, prm: &DenseParams, ctx: &Ctx) -> Result<PathBuf> {
    let dir = project.cache_dir("dense");
    std::fs::create_dir_all(&dir)?;
    let total = sp.cameras.len();
    let done = AtomicUsize::new(0);
    sp.cameras.par_iter().try_for_each(|c| -> Result<()> {
        ctx.check()?;
        let path = view_path(&dir, &c.key, prm.size);
        if !path.exists() {
            let img = load_rgb(&c.path)?;
            let (small, _) = shrink_to(img, prm.size);
            let mut buf = Vec::with_capacity(8 + small.w * small.h * 7);
            buf.extend_from_slice(&(small.w as u32).to_le_bytes());
            buf.extend_from_slice(&(small.h as u32).to_le_bytes());
            for p in small.data.chunks_exact(3) {
                let g = (0.299 * p[0] as f32 + 0.587 * p[1] as f32 + 0.114 * p[2] as f32) / 255.0;
                buf.extend_from_slice(&g.to_le_bytes());
            }
            buf.extend_from_slice(&small.data);
            let tmp = path.with_extension("tmp");
            std::fs::write(&tmp, buf)?;
            std::fs::rename(&tmp, &path)?;
        }
        let n = done.fetch_add(1, Ordering::Relaxed) + 1;
        ctx.report("dense-prep", n, total, "Preparing photos for depth");
        Ok(())
    })?;
    Ok(dir)
}

fn load_view(dir: &PathBuf, sp: &Sparse, ci: usize, size: usize) -> Result<View> {
    let c = &sp.cameras[ci];
    let mut buf = Vec::new();
    std::fs::File::open(view_path(dir, &c.key, size))?.read_to_end(&mut buf)?;
    let w = u32::from_le_bytes(buf[0..4].try_into()?) as usize;
    let h = u32::from_le_bytes(buf[4..8].try_into()?) as usize;
    let n = w * h;
    if buf.len() != 8 + n * 7 {
        bail!("damaged cache file for {}", c.path.display());
    }
    let gray = buf[8..8 + n * 4].chunks_exact(4).map(|b| f32::from_le_bytes(b.try_into().unwrap())).collect();
    let rgb = buf[8 + n * 4..].to_vec();
    let full = sp.intrinsics[c.group];
    let s = w as f64 / full.width as f64;
    let k = Intrinsics { f: full.f * s, cx: full.cx * s, cy: full.cy * s, ..full };
    Ok(View { w, h, gray, rgb, k, pose: c.pose })
}

#[derive(Clone, Copy)]
struct Hyp {
    d: f32,
    n: [f32; 3],
    cost: f32,
}

/// Relative pose ref → source, and the source view.
struct Src<'a> {
    v: &'a View,
    r: [[f32; 3]; 3],
    t: [f32; 3],
}

const WIN: [i32; 5] = [-4, -2, 0, 2, 4];
/// Minimum grey-level spread (0–1 scale, ≈2 levels of 255) for a window to
/// carry matchable texture. Below it NCC is dominated by JPEG noise.
const TEXTURE_MIN: f32 = 0.008;

struct Ctx2<'a> {
    rf: &'a View,
    rays: &'a [[f32; 2]],
    srcs: Vec<Src<'a>>,
}

impl Ctx2<'_> {
    /// 1 − NCC averaged over the best two sources; 2.0 when undefined.
    fn cost(&self, x: usize, y: usize, h: &Hyp) -> f32 {
        let w = self.rf.w;
        let ray = self.rays[y * w + x];
        let n = h.n;
        let xp = [ray[0] * h.d, ray[1] * h.d, h.d];
        let c = n[0] * xp[0] + n[1] * xp[1] + n[2] * xp[2];
        let mut refv = [0f32; 25];
        let mut pts = [[0f32; 3]; 25];
        let mut k = 0;
        for dy in WIN {
            for dx in WIN {
                let (qx, qy) = (x as i32 + dx, y as i32 + dy);
                if qx < 0 || qy < 0 || qx >= w as i32 || qy >= self.rf.h as i32 {
                    return 2.0;
                }
                let qi = qy as usize * w + qx as usize;
                refv[k] = self.rf.gray[qi];
                let r = self.rays[qi];
                let den = n[0] * r[0] + n[1] * r[1] + n[2];
                if den.abs() < 1e-6 {
                    return 2.0;
                }
                let dq = c / den;
                if dq <= 0.0 {
                    return 2.0;
                }
                pts[k] = [r[0] * dq, r[1] * dq, dq];
                k += 1;
            }
        }
        let (mr, sr) = mean_sd(&refv);
        if sr < TEXTURE_MIN {
            return 2.0; // textureless: no information
        }
        let mut costs = [2f32; 8];
        for (si, s) in self.srcs.iter().enumerate() {
            let mut sv = [0f32; 25];
            let mut ok = true;
            for (j, p) in pts.iter().enumerate() {
                let xs = [
                    s.r[0][0] * p[0] + s.r[0][1] * p[1] + s.r[0][2] * p[2] + s.t[0],
                    s.r[1][0] * p[0] + s.r[1][1] * p[1] + s.r[1][2] * p[2] + s.t[1],
                    s.r[2][0] * p[0] + s.r[2][1] * p[1] + s.r[2][2] * p[2] + s.t[2],
                ];
                if xs[2] <= 0.0 {
                    ok = false;
                    break;
                }
                let u = s.v.k.distort(Vector2::new((xs[0] / xs[2]) as f64, (xs[1] / xs[2]) as f64));
                match s.v.gray_at(u.x as f32, u.y as f32) {
                    Some(g) => sv[j] = g,
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if !ok {
                continue;
            }
            let (ms, ss) = mean_sd(&sv);
            if ss < 1e-3 {
                continue;
            }
            let mut cov = 0.0;
            for j in 0..25 {
                cov += (refv[j] - mr) * (sv[j] - ms);
            }
            costs[si] = 1.0 - cov / (25.0 * sr * ss);
        }
        let m = self.srcs.len().min(8);
        let cs = &mut costs[..m];
        cs.sort_by(f32::total_cmp);
        if m >= 2 { (cs[0] + cs[1]) * 0.5 } else { cs[0] }
    }
}

#[inline]
fn mean_sd(v: &[f32; 25]) -> (f32, f32) {
    let m = v.iter().sum::<f32>() / 25.0;
    let var = v.iter().map(|x| (x - m) * (x - m)).sum::<f32>() / 25.0;
    (m, var.sqrt())
}

#[inline]
fn rnd(s: &mut u64) -> f32 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    ((*s >> 40) as f32) / ((1u64 << 24) as f32)
}

fn random_normal(s: &mut u64, ray: [f32; 2], around: Option<[f32; 3]>, spread: f32) -> [f32; 3] {
    let base = around.unwrap_or_else(|| {
        let l = (ray[0] * ray[0] + ray[1] * ray[1] + 1.0).sqrt();
        [-ray[0] / l, -ray[1] / l, -1.0 / l]
    });
    let mut n = [
        base[0] + (rnd(s) - 0.5) * 2.0 * spread,
        base[1] + (rnd(s) - 0.5) * 2.0 * spread,
        base[2] + (rnd(s) - 0.5) * 2.0 * spread,
    ];
    // Must face the camera.
    if n[0] * ray[0] + n[1] * ray[1] + n[2] > 0.0 {
        n = [-n[0], -n[1], -n[2]];
    }
    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-9);
    [n[0] / l, n[1] / l, n[2] / l]
}

fn relative(r: &Pose, s: &Pose) -> ([[f32; 3]; 3], [f32; 3]) {
    let rr = s.r * r.r.transpose();
    let t = s.r * (r.c - s.c);
    let mut m = [[0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = rr[(i, j)] as f32;
        }
    }
    (m, [t.x as f32, t.y as f32, t.z as f32])
}

fn depth_path(dir: &PathBuf, key: &str, size: usize) -> PathBuf {
    dir.join(format!("{key}_{size}.depth"))
}

/// PatchMatch for one reference photo. Returns (depth, cost) per pixel.
fn depth_map(dir: &PathBuf, sp: &Sparse, ci: usize, prm: &DenseParams) -> Result<Option<(usize, usize, Vec<f32>)>> {
    let cam = &sp.cameras[ci];
    let rf = load_view(dir, sp, ci, prm.size)?;
    let nbrs: Vec<usize> = cam
        .neighbours
        .iter()
        .copied()
        .filter(|&j| {
            // Skip near-duplicate viewpoints: no depth information there.
            let base = (sp.cameras[j].pose.c - cam.pose.c).norm();
            base > 0.02 * cam.depth_range.0.max(1.0)
        })
        .take(prm.neighbours)
        .collect();
    if nbrs.is_empty() {
        return Ok(None);
    }
    let views: Vec<View> = nbrs.iter().map(|&j| load_view(dir, sp, j, prm.size)).collect::<Result<_>>()?;
    let (w, h) = (rf.w, rf.h);
    let rays: Vec<[f32; 2]> = (0..w * h)
        .map(|i| {
            let p = rf.k.undistort(Vector2::new((i % w) as f64 + 0.5, (i / w) as f64 + 0.5));
            [p.x as f32, p.y as f32]
        })
        .collect();
    let srcs: Vec<Src> = views
        .iter()
        .map(|v| {
            let (r, t) = relative(&rf.pose, &v.pose);
            Src { v, r, t }
        })
        .collect();
    let ctx2 = Ctx2 { rf: &rf, rays: &rays, srcs };
    let (dmin, dmax) = (cam.depth_range.0.max(0.5) as f32, cam.depth_range.1 as f32);
    let (imin, imax) = (1.0 / dmax, 1.0 / dmin);

    // Random initialisation (uniform in inverse depth), normals near "facing
    // up", which is the right prior for ground seen from the air.
    let up_cam = {
        let u = cam.pose.r * V3::new(0.0, 0.0, 1.0);
        [u.x as f32, u.y as f32, u.z as f32]
    };
    let mut st: Vec<Hyp> = (0..w * h)
        .into_par_iter()
        .map(|i| {
            let mut s = (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
            let d = 1.0 / (imin + rnd(&mut s) * (imax - imin));
            let n = random_normal(&mut s, rays[i], Some(up_cam), 0.3);
            let mut hy = Hyp { d, n, cost: 2.0 };
            hy.cost = ctx2.cost(i % w, i / w, &hy);
            hy
        })
        .collect();

    // Pixels whose window has no texture can't be matched at any depth;
    // leave them out of every iteration instead of failing them each time.
    let textured: Vec<bool> = (0..w * h)
        .into_par_iter()
        .map(|i| {
            let (x, y) = ((i % w) as i32, (i / w) as i32);
            if x < 4 || y < 4 || x >= w as i32 - 4 || y >= h as i32 - 4 {
                return false;
            }
            let mut v = [0f32; 25];
            let mut k = 0;
            for dy in WIN {
                for dx in WIN {
                    v[k] = rf.gray[(y + dy) as usize * w + (x + dx) as usize];
                    k += 1;
                }
            }
            mean_sd(&v).1 >= TEXTURE_MIN
        })
        .collect();
    const CAND: [(i32, i32); 8] = [(-1, 0), (1, 0), (0, -1), (0, 1), (-5, 0), (5, 0), (0, -5), (0, 5)];
    for it in 0..prm.iterations {
        let range = 0.5f32.powi(it as i32);
        for parity in 0..2usize {
            let snap = st.clone();
            st.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
                for x in 0..w {
                    if (x + y) % 2 != parity {
                        continue;
                    }
                    let i = y * w + x;
                    if !textured[i] {
                        continue;
                    }
                    let mut best = row[x];
                    // Propagation from the other colour.
                    for (dx, dy) in CAND {
                        let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                        if nx < 0 || ny < 0 || nx >= w as i32 || ny >= h as i32 {
                            continue;
                        }
                        let cand = snap[ny as usize * w + nx as usize];
                        // Converged neighbourhoods propose the same plane over
                        // and over; testing it again can't improve anything.
                        if cand.d == best.d && cand.n == best.n {
                            continue;
                        }
                        let mut hy = Hyp { cost: 2.0, ..cand };
                        hy.cost = ctx2.cost(x, y, &hy);
                        if hy.cost < best.cost {
                            best = hy;
                        }
                    }
                    // Random refinement, narrowing each iteration.
                    let mut s = ((i as u64) << 8 ^ (it as u64 * 2 + parity as u64)).wrapping_mul(0x2545_F491_4F6C_DD1D) | 1;
                    for _ in 0..2 {
                        let id = 1.0 / best.d + (rnd(&mut s) - 0.5) * range * 0.25 * (imax - imin);
                        let d = if id > imin * 0.5 && id < imax * 1.5 { 1.0 / id } else { best.d };
                        let n = random_normal(&mut s, rays[i], Some(best.n), 0.25 * range);
                        let mut hy = Hyp { d, n, cost: 2.0 };
                        hy.cost = ctx2.cost(x, y, &hy);
                        if hy.cost < best.cost {
                            best = hy;
                        }
                    }
                    row[x] = best;
                }
            });
        }
    }
    let depth: Vec<f32> = st
        .iter()
        .map(|hy| if hy.cost < 0.45 && hy.d >= dmin * 0.7 && hy.d <= dmax * 1.4 { hy.d } else { 0.0 })
        .collect();
    Ok(Some((w, h, depth)))
}

fn save_depth(path: &PathBuf, w: usize, h: usize, d: &[f32]) -> Result<()> {
    let mut f = std::io::BufWriter::new(std::fs::File::create(path.with_extension("tmp"))?);
    f.write_all(&(w as u32).to_le_bytes())?;
    f.write_all(&(h as u32).to_le_bytes())?;
    for v in d {
        f.write_all(&v.to_le_bytes())?;
    }
    f.flush()?;
    drop(f);
    std::fs::rename(path.with_extension("tmp"), path)?;
    Ok(())
}

fn load_depth(path: &PathBuf) -> Option<(usize, usize, Vec<f32>)> {
    let b = std::fs::read(path).ok()?;
    let w = u32::from_le_bytes(b.get(0..4)?.try_into().ok()?) as usize;
    let h = u32::from_le_bytes(b.get(4..8)?.try_into().ok()?) as usize;
    (b.len() == 8 + w * h * 4)
        .then(|| (w, h, b[8..].chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect()))
}

pub struct DenseCloud {
    pub points: Vec<[f32; 3]>,
    pub colors: Vec<[u8; 3]>,
    pub voxel_m: f64,
}

pub fn reconstruct_dense(project: &Project, sp: &Sparse, profile: &Profile, ctx: &Ctx) -> Result<DenseCloud> {
    let prm = DenseParams::for_profile(profile);
    let dir = prepare_views(project, sp, &prm, ctx)?;
    let n = sp.cameras.len();

    // ── Depth maps (cached per photo) ──
    let done = AtomicUsize::new(0);
    // Each depth map already runs on every core, so maps go one at a time:
    // that keeps only one photo and its few neighbours in memory.
    for ci in 0..n {
        ctx.check()?;
        let path = depth_path(&dir, &sp.cameras[ci].key, prm.size);
        if !path.exists() {
            if let Some((w, h, d)) = depth_map(&dir, sp, ci, &prm)? {
                save_depth(&path, w, h, &d)?;
            }
        }
        let k = done.fetch_add(1, Ordering::Relaxed) + 1;
        ctx.report("depth", k, n, format!("Depth for photo {k} of {n}"));
    }

    // ── Fusion: keep depths that neighbouring photos agree with ──
    // Voxel size: fine enough for the photos' detail, coarse enough that the
    // cloud fits the profile's budget.
    let mut xs: Vec<f64> = sp.points.iter().map(|p| p[0]).collect();
    let mut ys: Vec<f64> = sp.points.iter().map(|p| p[1]).collect();
    xs.sort_by(f64::total_cmp);
    ys.sort_by(f64::total_cmp);
    let pct = |v: &[f64], q: f64| v[((v.len() - 1) as f64 * q) as usize];
    let area = ((pct(&xs, 0.99) - pct(&xs, 0.01)) * (pct(&ys, 0.99) - pct(&ys, 0.01))).max(1.0);
    let mut depths: Vec<f64> = sp.cameras.iter().map(|c| (c.depth_range.0 + c.depth_range.1) / 2.0).collect();
    depths.sort_by(f64::total_cmp);
    let f0 = sp.intrinsics[0].f * prm.size as f64 / sp.intrinsics[0].width.max(sp.intrinsics[0].height) as f64;
    let gsd_dense = depths[depths.len() / 2] / f0;
    // Surfaces seen from above and the side: allow ~2.5 points per m² of plan
    // area per unit of the finest spacing.
    let voxel = (gsd_dense * 1.5).max((area * 2.5 / prm.max_points as f64).sqrt());
    let inv = 1.0 / voxel;

    let grid: std::sync::Mutex<HashMap<(i32, i32, i32), [f32; 7]>> = std::sync::Mutex::new(HashMap::new());
    let fused = AtomicUsize::new(0);
    (0..n).into_par_iter().try_for_each(|ci| -> Result<()> {
        ctx.check()?;
        let cam = &sp.cameras[ci];
        let Some((w, h, d)) = load_depth(&depth_path(&dir, &cam.key, prm.size)) else { return Ok(()) };
        let rf = load_view(&dir, sp, ci, prm.size)?;
        let nb: Vec<(usize, View, (usize, usize, Vec<f32>))> = cam
            .neighbours
            .iter()
            .take(prm.neighbours + 1)
            .filter_map(|&j| {
                let dm = load_depth(&depth_path(&dir, &sp.cameras[j].key, prm.size))?;
                let v = load_view(&dir, sp, j, prm.size).ok()?;
                Some((j, v, dm))
            })
            .collect();
        let need = if nb.len() >= 3 { 2 } else { 1 };
        let rwc = cam.pose.r.transpose();
        let mut local: HashMap<(i32, i32, i32), [f32; 7]> = HashMap::new();
        for y in 0..h {
            for x in 0..w {
                let di = d[y * w + x];
                if di <= 0.0 {
                    continue;
                }
                let p = rf.k.undistort(Vector2::new(x as f64 + 0.5, y as f64 + 0.5));
                let xc = V3::new(p.x * di as f64, p.y * di as f64, di as f64);
                let xw = rwc * xc + cam.pose.c;
                let mut agree = 0;
                for (_, v, (nw, nh, nd)) in &nb {
                    let xs = v.pose.to_cam(&xw);
                    if xs.z <= 0.0 {
                        continue;
                    }
                    let u = v.k.distort(Vector2::new(xs.x / xs.z, xs.y / xs.z));
                    let (ux, uy) = (u.x.floor() as i64, u.y.floor() as i64);
                    if ux < 0 || uy < 0 || ux >= *nw as i64 || uy >= *nh as i64 {
                        continue;
                    }
                    let dn = nd[uy as usize * nw + ux as usize] as f64;
                    if dn > 0.0 && ((dn - xs.z) / xs.z).abs() < 0.01 {
                        agree += 1;
                    }
                }
                if agree < need {
                    continue;
                }
                let key = ((xw.x * inv).floor() as i32, (xw.y * inv).floor() as i32, (xw.z * inv).floor() as i32);
                let c = &rf.rgb[(y * w + x) * 3..(y * w + x) * 3 + 3];
                let e = local.entry(key).or_insert([0.0; 7]);
                e[0] += xw.x as f32;
                e[1] += xw.y as f32;
                e[2] += xw.z as f32;
                e[3] += c[0] as f32;
                e[4] += c[1] as f32;
                e[5] += c[2] as f32;
                e[6] += 1.0;
            }
        }
        let mut g = grid.lock().unwrap();
        for (k, v) in local {
            let e = g.entry(k).or_insert([0.0; 7]);
            for i in 0..7 {
                e[i] += v[i];
            }
        }
        drop(g);
        let k = fused.fetch_add(1, Ordering::Relaxed) + 1;
        ctx.report("fuse", k, n, "Merging depth maps");
        Ok(())
    })?;
    let grid = grid.into_inner().unwrap();
    let mut points = Vec::with_capacity(grid.len());
    let mut colors = Vec::with_capacity(grid.len());
    for v in grid.values() {
        let c = v[6];
        points.push([v[0] / c, v[1] / c, v[2] / c]);
        colors.push([(v[3] / c) as u8, (v[4] / c) as u8, (v[5] / c) as u8]);
    }
    Ok(DenseCloud { points, colors, voxel_m: voxel })
}
