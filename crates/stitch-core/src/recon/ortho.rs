//! True orthophoto: every map pixel is coloured from a photo that can actually
//! see that spot, projected from its real height on the surface model. Roofs
//! sit exactly over their footprints and nothing leans — unlike the 2D
//! mosaic, which assumes flat ground.
//!
//! Written in the same tile/pyramid/manifest format as the 2D map, under
//! `result/ortho/`, so the map viewer and every export work unchanged.

use super::dsm::Dsm;
use super::geometry::{Pose, V3};
use super::sparse::Sparse;
use crate::imgops::{box_down, load_rgb, Rgb};
use crate::profile::Profile;
use crate::project::Project;
use crate::render::{build_pyramid, max_zoom_for, tile_path, write_jpeg, Manifest, RasterCache, DATA_FLOOR, TILE};
use crate::Ctx;
use anyhow::{bail, Result};
use nalgebra::Vector2;
use rayon::prelude::*;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

pub fn ortho_dir(project: &Project) -> PathBuf {
    project.result_dir().join("ortho")
}

/// Up to three cameras that see a surface cell, with their weights.
pub(crate) type Seen = [(u32, f32); 3];
const NONE: (u32, f32) = (u32::MAX, 0.0);

/// Is the straight line from `p` to the camera centre `c` clear of the surface?
pub(crate) fn visible(dsm: &Dsm, p: &V3, c: &V3) -> bool {
    let d = c - p;
    let len = d.norm();
    if len < 1e-6 {
        return true;
    }
    let dir = d / len;
    let step = dsm.cell * 0.8;
    let mut t = step;
    // Start above the surface and allow some slack: the surface model is
    // noisy on plain roofs, and a few centimetres of bump must not count as a
    // wall. Real occluders (trees, buildings) stand metres proud.
    let start = p + V3::new(0.0, 0.0, 0.5);
    for _ in 0..4000 {
        let q = start + dir * t;
        if q.z > dsm.zmax as f64 + 1.0 || t >= len {
            return true;
        }
        if let Some(h) = dsm.height(q.x, q.y) {
            if h as f64 > q.z + 0.5 {
                return false;
            }
        }
        t += step;
    }
    true
}

/// How centrally (0 at the edge, 1 at the middle) a pixel sits in a photo.
#[inline]
pub fn centredness(u: &Vector2<f64>, w: f64, h: f64) -> f64 {
    let dx = (u.x.min(w - u.x) / (w / 2.0)).max(0.0);
    let dy = (u.y.min(h - u.y) / (h / 2.0)).max(0.0);
    dx * dy
}

/// Buckets of cameras by the ground they can see, for fast candidate lookup.
pub struct CamIndex {
    x0: f64,
    y0: f64,
    size: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

impl CamIndex {
    pub fn new(sp: &Sparse, dsm: &Dsm) -> Self {
        let size = 20.0f64.max(dsm.cell * 8.0);
        let (x0, y0) = (dsm.x0, dsm.y0);
        let nx = ((dsm.nx as f64 * dsm.cell) / size).ceil() as usize + 1;
        let ny = ((dsm.ny as f64 * dsm.cell) / size).ceil() as usize + 1;
        let mut zs: Vec<f32> = dsm.h.iter().copied().filter(|v| v.is_finite()).collect();
        zs.sort_by(f32::total_cmp);
        let ground = zs.get(zs.len() / 2).copied().unwrap_or(0.0) as f64;
        let mut cells = vec![Vec::new(); nx * ny];
        for (ci, cam) in sp.cameras.iter().enumerate() {
            let k = &sp.intrinsics[cam.group];
            let rwc = cam.pose.r.transpose();
            let agl = (cam.pose.c.z - ground).max(5.0);
            let (mut xa, mut xb, mut ya, mut yb) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for (u, v) in [(0.0, 0.0), (k.width as f64, 0.0), (0.0, k.height as f64), (k.width as f64, k.height as f64), (k.cx, k.cy)] {
                let p = k.undistort(Vector2::new(u, v));
                let ray = rwc * V3::new(p.x, p.y, 1.0);
                // Rays near the horizon would reach forever: cap at 5× height.
                let t = if ray.z < -0.05 { ((ground - cam.pose.c.z) / ray.z).min(agl * 5.0) } else { agl * 5.0 };
                let g = cam.pose.c + ray * t;
                xa = xa.min(g.x);
                xb = xb.max(g.x);
                ya = ya.min(g.y);
                yb = yb.max(g.y);
            }
            let i0 = (((xa - x0) / size).floor().max(0.0) as usize).min(nx - 1);
            let i1 = (((xb - x0) / size).floor().max(0.0) as usize).min(nx - 1);
            let j0 = (((ya - y0) / size).floor().max(0.0) as usize).min(ny - 1);
            let j1 = (((yb - y0) / size).floor().max(0.0) as usize).min(ny - 1);
            for j in j0..=j1 {
                for i in i0..=i1 {
                    cells[j * nx + i].push(ci as u32);
                }
            }
        }
        Self { x0, y0, size, nx, ny, cells }
    }

    pub fn near(&self, x: f64, y: f64) -> &[u32] {
        let i = ((x - self.x0) / self.size).floor();
        let j = ((y - self.y0) / self.size).floor();
        if i < 0.0 || j < 0.0 || i >= self.nx as f64 || j >= self.ny as f64 {
            return &[];
        }
        &self.cells[j as usize * self.nx + i as usize]
    }
}

/// For every surface cell, the best cameras for a TOP-DOWN view: visible,
/// steep (near-vertical ray) and central in the photo.
fn rank_top_down(sp: &Sparse, dsm: &Dsm, idx: &CamIndex, ctx: &Ctx) -> Vec<Seen> {
    let done = AtomicUsize::new(0);
    let rows = dsm.ny;
    (0..dsm.ny)
        .into_par_iter()
        .flat_map_iter(|j| {
            let n = done.fetch_add(1, Ordering::Relaxed) + 1;
            if n % 64 == 0 || n == rows {
                ctx.report("visibility", n, rows, "Working out which photos see each spot");
            }
            (0..dsm.nx).map(move |i| {
                let z = dsm.at(i, j);
                if !z.is_finite() {
                    return [NONE; 3];
                }
                let p = V3::new(dsm.x0 + (i as f64 + 0.5) * dsm.cell, dsm.y0 + (j as f64 + 0.5) * dsm.cell, z as f64);
                let mut cands: Vec<(f64, u32)> = idx
                    .near(p.x, p.y)
                    .iter()
                    .filter_map(|&ci| {
                        let cam = &sp.cameras[ci as usize];
                        let k = &sp.intrinsics[cam.group];
                        let u = cam.pose.project(k, &p)?;
                        let c = centredness(&u, k.width as f64, k.height as f64);
                        if c < 0.02 {
                            return None;
                        }
                        let ray = (cam.pose.c - p).normalize();
                        if ray.z <= 0.1 {
                            return None;
                        }
                        Some((c.powi(8) * ray.z.powi(4), ci))
                    })
                    .collect();
                cands.sort_by(|a, b| b.0.total_cmp(&a.0));
                let mut out = [NONE; 3];
                let mut k = 0;
                let fallback = cands.first().copied();
                for (s, ci) in cands.into_iter().take(5) {
                    if visible(dsm, &p, &sp.cameras[ci as usize].pose.c) {
                        out[k] = (ci, s as f32);
                        k += 1;
                        if k == 3 {
                            break;
                        }
                    }
                }
                // Nobody passes the visibility test (rare: noisy surface):
                // the best-placed photo beats a black hole.
                if k == 0 {
                    if let Some((s, ci)) = fallback {
                        out[0] = (ci, s as f32);
                    }
                }
                out
            })
        })
        .collect()
}

/// Full-resolution ground sample distance of each camera at its mid depth.
fn cam_gsd(sp: &Sparse) -> Vec<f64> {
    sp.cameras
        .iter()
        .map(|c| ((c.depth_range.0 + c.depth_range.1) / 2.0) / sp.intrinsics[c.group].f)
        .collect()
}

pub(crate) fn project_px(pose: &Pose, k: &super::geometry::Intrinsics, p: &V3) -> Option<Vector2<f64>> {
    pose.project(k, p)
}

pub fn render_ortho(project: &Project, sp: &Sparse, dsm: &Dsm, profile: &Profile, ctx: &Ctx) -> Result<Manifest> {
    let gsds = cam_gsd(sp);
    let mut g: Vec<f64> = sp.cameras.iter().zip(&gsds).filter(|(c, _)| !c.oblique).map(|(_, g)| *g).collect();
    if g.is_empty() {
        g = gsds.clone();
    }
    g.sort_by(f64::total_cmp);
    let gsd = g[g.len() / 2] / project.file.settings.output_scale.clamp(0.05, 1.0);

    // Extent of the surface that exists.
    let (mut xa, mut xb, mut ya, mut yb) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for j in 0..dsm.ny {
        for i in 0..dsm.nx {
            if dsm.at(i, j).is_finite() {
                xa = xa.min(dsm.x0 + i as f64 * dsm.cell);
                xb = xb.max(dsm.x0 + (i + 1) as f64 * dsm.cell);
                ya = ya.min(dsm.y0 + j as f64 * dsm.cell);
                yb = yb.max(dsm.y0 + (j + 1) as f64 * dsm.cell);
            }
        }
    }
    if xa >= xb {
        bail!("no surface to build an orthophoto on");
    }
    let left = (xa / gsd).floor() * gsd;
    let top = (yb / gsd).ceil() * gsd;
    let width = ((xb - left) / gsd).ceil() as u64;
    let height = ((top - ya) / gsd).ceil() as u64;
    let max_zoom = max_zoom_for(width, height);
    let nx = (width as usize).div_ceil(TILE);
    let ny = (height as usize).div_ceil(TILE);

    let idx = CamIndex::new(sp, dsm);
    let seen = rank_top_down(sp, dsm, &idx, ctx);
    let dir = ortho_dir(project);
    let root = dir.join("tiles");
    if root.exists() {
        std::fs::remove_dir_all(&root)?;
    }
    std::fs::create_dir_all(&root)?;

    // Downsample factor per camera so photos are decoded near output scale.
    let factor: Vec<usize> = gsds
        .iter()
        .map(|cg| {
            let mut f = 1;
            while (f * 2) as f64 <= gsd / cg {
                f *= 2;
            }
            f
        })
        .collect();
    let cache = RasterCache::new(profile.render_cache_bytes);
    let tiles: Vec<(usize, usize)> = {
        const BLOCK: usize = 4;
        let mut t: Vec<(usize, usize)> = (0..ny).flat_map(|y| (0..nx).map(move |x| (x, y))).collect();
        let key = |&(x, y): &(usize, usize)| {
            let (bx, by) = (x / BLOCK, y / BLOCK);
            (by, if by % 2 == 0 { bx } else { usize::MAX - bx }, y, x)
        };
        t.sort_by_key(key);
        t
    };
    let written: Mutex<Vec<(usize, usize)>> = Mutex::new(Vec::new());
    let done = AtomicUsize::new(0);
    let total = tiles.len();
    for chunk in tiles.chunks(16) {
        ctx.check()?;
        chunk.par_iter().try_for_each(|&(tx, ty)| -> Result<()> {
            let x0 = left + (tx * TILE) as f64 * gsd;
            let y1 = top - (ty * TILE) as f64 * gsd;
            // Cameras used by the cells under this tile.
            let mut used: HashMap<u32, Option<Arc<Rgb>>> = HashMap::new();
            let ci0 = (((x0 - dsm.x0) / dsm.cell).floor().max(0.0) as usize).min(dsm.nx - 1);
            let ci1 = (((x0 + TILE as f64 * gsd - dsm.x0) / dsm.cell).floor().max(0.0) as usize).min(dsm.nx - 1);
            let cj0 = (((y1 - TILE as f64 * gsd - dsm.y0) / dsm.cell).floor().max(0.0) as usize).min(dsm.ny - 1);
            let cj1 = (((y1 - dsm.y0) / dsm.cell).floor().max(0.0) as usize).min(dsm.ny - 1);
            for j in cj0..=cj1 {
                for i in ci0..=ci1 {
                    for &(c, _) in &seen[j * dsm.nx + i] {
                        if c != u32::MAX {
                            used.entry(c).or_insert(None);
                        }
                    }
                }
            }
            if used.is_empty() {
                done.fetch_add(1, Ordering::Relaxed);
                return Ok(());
            }
            for (c, slot) in used.iter_mut() {
                let ci = *c as usize;
                let f = factor[ci];
                *slot = cache.get(ci, || {
                    let img = load_rgb(&sp.cameras[ci].path).ok()?;
                    Some(if f > 1 { box_down(&img, f) } else { img })
                });
            }
            let mut out = vec![0u8; TILE * TILE * 3];
            let mut any = false;
            for py in 0..TILE {
                let y = y1 - (py as f64 + 0.5) * gsd;
                for px in 0..TILE {
                    let x = x0 + (px as f64 + 0.5) * gsd;
                    let i = ((x - dsm.x0) / dsm.cell).floor();
                    let j = ((y - dsm.y0) / dsm.cell).floor();
                    if i < 0.0 || j < 0.0 || i >= dsm.nx as f64 || j >= dsm.ny as f64 {
                        continue;
                    }
                    let s = &seen[j as usize * dsm.nx + i as usize];
                    if s[0].0 == u32::MAX {
                        continue;
                    }
                    let Some(z) = dsm.height(x, y) else { continue };
                    let p = V3::new(x, y, z as f64);
                    let (mut acc, mut ws) = ([0f64; 3], 0f64);
                    for &(c, w) in s {
                        if c == u32::MAX {
                            continue;
                        }
                        let ci = c as usize;
                        let Some(Some(img)) = used.get(&c) else { continue };
                        let cam = &sp.cameras[ci];
                        let k = &sp.intrinsics[cam.group];
                        let Some(u) = project_px(&cam.pose, k, &p) else { continue };
                        let f = factor[ci] as f64;
                        let Some(col) = img.sample(u.x / f, u.y / f) else { continue };
                        // Per-pixel centredness keeps the hand-over between
                        // photos smooth inside a cell, not only at its edges.
                        let ww = w as f64 * centredness(&u, k.width as f64, k.height as f64).powi(8) + 1e-12;
                        for c in 0..3 {
                            acc[c] += col[c] as f64 * ww;
                        }
                        ws += ww;
                    }
                    if ws <= 0.0 {
                        continue;
                    }
                    any = true;
                    let o = (py * TILE + px) * 3;
                    for c in 0..3 {
                        out[o + c] = (acc[c] / ws).round().clamp(0.0, 255.0) as u8;
                    }
                    let m = out[o].max(out[o + 1]).max(out[o + 2]);
                    if m < DATA_FLOOR {
                        let lift = DATA_FLOOR - m;
                        for c in 0..3 {
                            out[o + c] += lift;
                        }
                    }
                }
            }
            if any {
                write_jpeg(&tile_path(&root, max_zoom, tx, ty), &out, TILE, TILE)?;
                written.lock().unwrap().push((tx, ty));
            }
            let n = done.fetch_add(1, Ordering::Relaxed) + 1;
            if n % 8 == 0 || n == total {
                ctx.report("ortho", n, total, "Drawing the true orthophoto");
            }
            Ok(())
        })?;
    }
    drop(cache);
    build_pyramid(&root, max_zoom, written.into_inner().unwrap(), ctx)?;

    let manifest = Manifest {
        width,
        height,
        tile_size: TILE,
        max_zoom,
        gsd,
        left,
        top,
        epsg: Some(sp.zone.epsg()),
        utm_left: Some(sp.origin_e + left),
        utm_top: Some(sp.origin_n + top),
        photos: sp.cameras.len(),
        version: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0),
    };
    std::fs::write(dir.join("manifest.json"), serde_json::to_vec_pretty(&manifest)?)?;
    Ok(manifest)
}
