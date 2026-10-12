//! Tile renderer. The mosaic is never held in memory: each 512×512 tile is
//! drawn from the photos that cover it, written as JPEG, and dropped. Decoded
//! photos sit in a cache with a byte budget from the [`Profile`], so memory
//! stays flat whether the survey has 50 photos or 5,000.
//!
//! Near-black means "no data": real pixels are lifted so their brightest
//! channel is at least [`DATA_FLOOR`], and anything whose brightest channel is
//! below [`NODATA_MAX`] is treated as empty. The gap absorbs JPEG ringing at
//! the map's edge.

use crate::imgops::{box_down, load_rgb, Rgb};
use crate::pipeline::{Alignment, Frame, Placed};
use crate::profile::Profile;
use crate::project::Project;
use crate::sim::Sim2;
use crate::Ctx;
use anyhow::{bail, Result};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock, Weak};

pub const TILE: usize = 512;
const JPEG_QUALITY: u8 = 88;
/// Sharpness of the blend between overlapping photos: higher favours each
/// photo's centre, giving near-hard seams with a short soft transition.
const BLEND_POW: i32 = 16;
pub const DATA_FLOOR: u8 = 12;
pub const NODATA_MAX: u8 = 6;

#[inline]
pub fn is_nodata(p: &[u8]) -> bool {
    p[0].max(p[1]).max(p[2]) < NODATA_MAX
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Manifest {
    pub width: u64,
    pub height: u64,
    pub tile_size: usize,
    /// Zoom levels 0..=max_zoom; max_zoom is full resolution.
    pub max_zoom: u32,
    /// Frame units (metres when georeferenced) per output pixel.
    pub gsd: f64,
    /// Frame coordinate of the canvas's top-left corner.
    pub left: f64,
    pub top: f64,
    pub epsg: Option<u32>,
    /// Absolute UTM coordinates of the top-left corner, when georeferenced.
    pub utm_left: Option<f64>,
    pub utm_top: Option<f64>,
    pub photos: usize,
    /// Changes on every render so viewers don't show stale cached tiles.
    pub version: u64,
}

pub fn tile_path(root: &Path, z: u32, x: usize, y: usize) -> PathBuf {
    root.join(z.to_string()).join(x.to_string()).join(format!("{y}.jpg"))
}

pub fn tiles_dir(project: &Project) -> PathBuf {
    project.result_dir().join("tiles")
}

pub fn read_manifest(project: &Project) -> Result<Manifest> {
    Ok(serde_json::from_slice(&std::fs::read(project.result_dir().join("manifest.json"))?)?)
}

struct Slot {
    /// The current (or in-flight) decode. Replaced when the photo has to be
    /// loaded again after being fully released.
    cell: Arc<OnceLock<Option<Arc<Rgb>>>>,
    /// Held by the cache; counts against the budget.
    strong: Option<Arc<Rgb>>,
    /// Still reachable after eviction while a tile is drawing from it — reused
    /// rather than decoded a second time.
    weak: Weak<Rgb>,
    last: u64,
    bytes: usize,
}

/// Byte-budgeted LRU of decoded (and possibly downsampled) photos.
///
/// Two properties keep memory bounded: concurrent requests for one photo wait
/// on a single decode, and a photo evicted while still in use is handed back
/// from its weak reference instead of being loaded again. (Without the second,
/// a budget smaller than the working set thrashes and accumulates duplicate
/// copies — the first real run on the Lite profile peaked at 5.4 GB.)
struct RasterCache {
    budget: usize,
    tick: AtomicU64,
    map: Mutex<(HashMap<usize, Slot>, usize)>,
}

impl RasterCache {
    fn new(budget: usize) -> Self {
        Self { budget, tick: AtomicU64::new(0), map: Mutex::new((HashMap::new(), 0)) }
    }

    fn evict(&self, map: &mut HashMap<usize, Slot>, total: &mut usize, keep: usize) {
        while *total > self.budget {
            let victim = map
                .iter()
                .filter(|(k, s)| **k != keep && s.strong.is_some())
                .min_by_key(|(_, s)| s.last)
                .map(|(k, _)| *k);
            let Some(k) = victim else { break };
            let s = map.get_mut(&k).unwrap();
            s.strong = None;
            // The decode cell holds its own strong reference; replace it too,
            // or the photo is never actually freed.
            s.cell = Arc::new(OnceLock::new());
            *total -= s.bytes;
        }
    }

    fn get(&self, idx: usize, load: impl FnOnce() -> Option<Rgb>) -> Option<Arc<Rgb>> {
        let t = self.tick.fetch_add(1, Ordering::Relaxed);
        let cell = {
            let mut g = self.map.lock().unwrap();
            let (map, total) = &mut *g;
            let slot = map.entry(idx).or_insert_with(|| Slot {
                cell: Arc::new(OnceLock::new()),
                strong: None,
                weak: Weak::new(),
                last: t,
                bytes: 0,
            });
            slot.last = t;
            if let Some(a) = &slot.strong {
                return Some(a.clone());
            }
            if let Some(a) = slot.weak.upgrade() {
                slot.strong = Some(a.clone());
                *total += slot.bytes;
                self.evict(map, total, idx);
                return Some(a);
            }
            if slot.cell.get().is_some() {
                // Fully released since the last decode: start a new one.
                slot.cell = Arc::new(OnceLock::new());
            }
            slot.cell.clone()
        };
        let mut fresh = false;
        let v = cell
            .get_or_init(|| {
                fresh = true;
                load().map(Arc::new)
            })
            .clone();
        if fresh {
            if let Some(a) = &v {
                let mut g = self.map.lock().unwrap();
                let (map, total) = &mut *g;
                if let Some(s) = map.get_mut(&idx) {
                    s.bytes = a.bytes();
                    s.weak = Arc::downgrade(a);
                    s.strong = Some(a.clone());
                    *total += s.bytes;
                }
                self.evict(map, total, idx);
            }
        }
        v
    }
}

struct Source<'a> {
    p: &'a Placed,
    inv: Sim2,
    /// Downsample factor applied when loading (power of two).
    f: usize,
    bbox: (f64, f64, f64, f64), // frame: minx, miny, maxx, maxy
}

fn corners(p: &Placed) -> [(f64, f64); 4] {
    let (w, h) = (p.width as f64, p.height as f64);
    [p.full_to_frame(0.0, 0.0), p.full_to_frame(w, 0.0), p.full_to_frame(w, h), p.full_to_frame(0.0, h)]
}

pub fn render(project: &Project, al: &Alignment, profile: &Profile, ctx: &Ctx) -> Result<Manifest> {
    if al.placed.is_empty() {
        bail!("nothing to render");
    }
    let mut gsds: Vec<f64> = al.placed.iter().map(|p| p.gsd()).collect();
    gsds.sort_by(f64::total_cmp);
    let native = gsds[gsds.len() / 2];
    let scale = project.file.settings.output_scale.clamp(0.05, 1.0);
    let gsd = native / scale;

    let mut sources = Vec::with_capacity(al.placed.len());
    let (mut minx, mut miny, mut maxx, mut maxy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for p in &al.placed {
        let c = corners(p);
        let bx = c.iter().map(|v| v.0).fold(f64::MAX, f64::min);
        let by = c.iter().map(|v| v.1).fold(f64::MAX, f64::min);
        let bx2 = c.iter().map(|v| v.0).fold(f64::MIN, f64::max);
        let by2 = c.iter().map(|v| v.1).fold(f64::MIN, f64::max);
        minx = minx.min(bx);
        miny = miny.min(by);
        maxx = maxx.max(bx2);
        maxy = maxy.max(by2);
        let ratio = gsd / p.gsd();
        let mut f = 1usize;
        while (f * 2) as f64 <= ratio {
            f *= 2;
        }
        sources.push(Source { p, inv: p.sim.inverse(), f, bbox: (bx, by, bx2, by2) });
    }
    let left = (minx / gsd).floor() * gsd;
    let top = (maxy / gsd).ceil() * gsd;
    let width = ((maxx - left) / gsd).ceil() as u64;
    let height = ((top - miny) / gsd).ceil() as u64;
    if width.saturating_mul(height) > 400_000_000_000 {
        bail!("the mosaic came out implausibly large ({width}×{height}); some photos are probably misplaced");
    }
    let nx = (width as usize).div_ceil(TILE);
    let ny = (height as usize).div_ceil(TILE);
    let max_zoom = {
        let mut z = 0u32;
        while (TILE << z) < width.max(height) as usize {
            z += 1;
        }
        z
    };

    // Spatial index: photos overlapping each tile column/row band.
    let tile_frame = |tx: usize, ty: usize| {
        let x0 = left + (tx * TILE) as f64 * gsd;
        let y1 = top - (ty * TILE) as f64 * gsd;
        (x0, y1 - TILE as f64 * gsd, x0 + TILE as f64 * gsd, y1)
    };
    let mut index: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (si, s) in sources.iter().enumerate() {
        let tx0 = (((s.bbox.0 - left) / gsd) as usize / TILE).min(nx - 1);
        let tx1 = (((s.bbox.2 - left) / gsd) as usize / TILE).min(nx - 1);
        let ty0 = (((top - s.bbox.3) / gsd).max(0.0) as usize / TILE).min(ny - 1);
        let ty1 = (((top - s.bbox.1) / gsd).max(0.0) as usize / TILE).min(ny - 1);
        for ty in ty0..=ty1 {
            for tx in tx0..=tx1 {
                index.entry((tx, ty)).or_default().push(si);
            }
        }
    }

    let root = tiles_dir(project);
    if root.exists() {
        std::fs::remove_dir_all(&root)?;
    }
    std::fs::create_dir_all(&root)?;

    let cache = RasterCache::new(profile.render_cache_bytes);
    let total_tiles = index.len();
    let done = AtomicUsize::new(0);
    ctx.report("render", 0, total_tiles, "Drawing the map");

    // Draw in compact blocks of BLOCK×BLOCK tiles, blocks in serpentine order.
    // A block needs only the handful of photos over it, so the working set
    // fits the cache budget; whole tile rows across a wide survey did not.
    const BLOCK: usize = 4;
    let mut todo: Vec<(usize, usize)> = index.keys().copied().collect();
    let block_key = |&(x, y): &(usize, usize)| {
        let (bx, by) = (x / BLOCK, y / BLOCK);
        (by, if by % 2 == 0 { bx } else { usize::MAX - bx })
    };
    todo.sort_by_key(|t| (block_key(t), t.1, t.0));
    let written: Mutex<Vec<(usize, usize)>> = Mutex::new(Vec::new());
    for row in todo.chunk_by(|a, b| block_key(a) == block_key(b)) {
        ctx.check()?;
        row.par_iter().try_for_each(|&(tx, ty)| -> Result<()> {
            let fr = tile_frame(tx, ty);
            let rasters: Vec<(&Source, Arc<Rgb>)> = contributors(&sources, &index[&(tx, ty)], fr)
                .into_iter()
                .filter_map(|idx| {
                    let s = &sources[idx];
                    let r = cache.get(idx, || match load_rgb(&s.p.path) {
                        Ok(img) => Some(if s.f > 1 { box_down(&img, s.f) } else { img }),
                        Err(e) => {
                            eprintln!("render: {e:#}");
                            None
                        }
                    })?;
                    Some((s, r))
                })
                .collect();
            if let Some(buf) = draw_tile(&rasters, fr.0, fr.3, gsd) {
                write_jpeg(&tile_path(&root, max_zoom, tx, ty), &buf, TILE, TILE)?;
                written.lock().unwrap().push((tx, ty));
            }
            let n = done.fetch_add(1, Ordering::Relaxed) + 1;
            if n % 4 == 0 || n == total_tiles {
                ctx.report("render", n, total_tiles, "Drawing the map");
            }
            Ok(())
        })?;
    }
    drop(cache);

    let mut level: Vec<(usize, usize)> = written.into_inner().unwrap();
    for z in (0..max_zoom).rev() {
        ctx.check()?;
        ctx.report("pyramid", (max_zoom - z) as usize, max_zoom as usize, "Building zoom levels");
        let mut parents: Vec<(usize, usize)> = level.iter().map(|&(x, y)| (x / 2, y / 2)).collect();
        parents.sort();
        parents.dedup();
        parents.par_iter().try_for_each(|&(px, py)| -> Result<()> {
            let buf = downsample_children(&root, z + 1, px, py)?;
            write_jpeg(&tile_path(&root, z, px, py), &buf, TILE, TILE)
        })?;
        level = parents;
    }

    let (epsg, utm_left, utm_top) = match &al.frame {
        Frame::Utm { zone, origin_e, origin_n } => (Some(zone.epsg()), Some(origin_e + left), Some(origin_n + top)),
        Frame::Local => (None, None, None),
    };
    let manifest = Manifest {
        width,
        height,
        tile_size: TILE,
        max_zoom,
        gsd,
        left,
        top,
        epsg,
        utm_left,
        utm_top,
        photos: al.placed.len(),
        version: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0),
    };
    std::fs::write(project.result_dir().join("manifest.json"), serde_json::to_vec_pretty(&manifest)?)?;
    ctx.report("done", 1, 1, "Map ready");
    Ok(manifest)
}

/// Blend weight of a frame point in a photo: `(dx·dy)^BLEND_POW`, where dx and
/// dy run from 0 at the photo's edge to 1 at its centre. 0 outside the photo.
fn weight(s: &Source, fx: f64, fy: f64) -> f64 {
    let (x, y) = s.p.frame_to_full(&s.inv, fx, fy);
    let (w, h) = (s.p.width as f64, s.p.height as f64);
    if x < 0.0 || y < 0.0 || x >= w || y >= h {
        return 0.0;
    }
    let dx = x.min(w - x) / (w / 2.0);
    let dy = y.min(h - y) / (h / 2.0);
    (dx * dy).powi(BLEND_POW)
}

/// The photos that visibly contribute to a tile. Drone surveys overlap 70–80%,
/// so a tile sits under a dozen or more photos — but with the sharp blend only
/// the two or three most central ones carry any weight. Loading just those is
/// what keeps memory within the profile's budget (and makes rendering faster).
fn contributors(sources: &[Source], cands: &[usize], fr: (f64, f64, f64, f64)) -> Vec<usize> {
    let cands: Vec<usize> = cands
        .iter()
        .copied()
        .filter(|&i| {
            let s = &sources[i];
            s.bbox.0 < fr.2 && s.bbox.2 > fr.0 && s.bbox.1 < fr.3 && s.bbox.3 > fr.1
        })
        .collect();
    if cands.len() <= 2 {
        return cands;
    }
    // Sample a 5×5 grid over the tile; keep a photo if, at any sample, it
    // carries at least 0.2% of the weight of the strongest photo there.
    let mut keep = vec![false; cands.len()];
    let mut ws = vec![0.0; cands.len()];
    for sy in 0..5 {
        for sx in 0..5 {
            let fx = fr.0 + (fr.2 - fr.0) * sx as f64 / 4.0;
            let fy = fr.1 + (fr.3 - fr.1) * sy as f64 / 4.0;
            let mut best: f64 = 0.0;
            for (k, &i) in cands.iter().enumerate() {
                ws[k] = weight(&sources[i], fx, fy);
                best = best.max(ws[k]);
            }
            if best > 0.0 {
                for k in 0..cands.len() {
                    if ws[k] >= best * 2e-3 {
                        keep[k] = true;
                    }
                }
            }
        }
    }
    // A photo can still cover a tile edge between samples; if no sample hit
    // anything, fall back to every candidate rather than leave a hole.
    if !keep.iter().any(|&k| k) {
        return cands;
    }
    cands.into_iter().zip(keep).filter(|(_, k)| *k).map(|(i, _)| i).collect()
}

fn draw_tile(rasters: &[(&Source, Arc<Rgb>)], x0: f64, y1: f64, gsd: f64) -> Option<Vec<u8>> {
    if rasters.is_empty() {
        return None;
    }
    let mut out = vec![0u8; TILE * TILE * 3];
    let mut any = false;
    for py in 0..TILE {
        let fy = y1 - (py as f64 + 0.5) * gsd;
        for px in 0..TILE {
            let fx = x0 + (px as f64 + 0.5) * gsd;
            let (mut acc, mut wsum) = ([0f32; 3], 0f32);
            for (s, r) in rasters {
                if fx < s.bbox.0 || fx > s.bbox.2 || fy < s.bbox.1 || fy > s.bbox.3 {
                    continue;
                }
                let (x, y) = s.p.frame_to_full(&s.inv, fx, fy);
                let (fw, fh) = (s.p.width as f64, s.p.height as f64);
                if x < 0.0 || y < 0.0 || x >= fw || y >= fh {
                    continue;
                }
                let f = s.f as f64;
                let Some(c) = r.sample(x / f, y / f) else { continue };
                let dx = (x.min(fw - x) / (fw / 2.0)) as f32;
                let dy = (y.min(fh - y) / (fh / 2.0)) as f32;
                let w = (dx * dy).powi(BLEND_POW) + 1e-30;
                for k in 0..3 {
                    acc[k] += c[k] * w;
                }
                wsum += w;
            }
            if wsum > 0.0 {
                any = true;
                let o = (py * TILE + px) * 3;
                for k in 0..3 {
                    out[o + k] = (acc[k] / wsum).round().clamp(0.0, 255.0) as u8;
                }
                let m = out[o].max(out[o + 1]).max(out[o + 2]);
                if m < DATA_FLOOR {
                    let lift = DATA_FLOOR - m;
                    for k in 0..3 {
                        out[o + k] += lift;
                    }
                }
            }
        }
    }
    any.then_some(out)
}

pub fn read_tile(root: &Path, z: u32, x: usize, y: usize) -> Result<Option<Vec<u8>>> {
    let p = tile_path(root, z, x, y);
    if !p.exists() {
        return Ok(None);
    }
    let img = image::open(&p)?.into_rgb8();
    Ok(Some(img.into_raw()))
}

/// Averages each 2×2 block of the four children, ignoring no-data pixels so
/// the map's edge doesn't darken at lower zooms.
fn downsample_children(root: &Path, z: u32, px: usize, py: usize) -> Result<Vec<u8>> {
    let mut kids: [[Option<Vec<u8>>; 2]; 2] = Default::default();
    for (dy, row) in kids.iter_mut().enumerate() {
        for (dx, slot) in row.iter_mut().enumerate() {
            *slot = read_tile(root, z, px * 2 + dx, py * 2 + dy)?;
        }
    }
    let half = TILE / 2;
    let mut out = vec![0u8; TILE * TILE * 3];
    for (dy, row) in kids.iter().enumerate() {
        for (dx, kid) in row.iter().enumerate() {
            let Some(k) = kid else { continue };
            for y in 0..half {
                for x in 0..half {
                    let (mut acc, mut n) = ([0u32; 3], 0u32);
                    for (sx, sy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                        let i = ((y * 2 + sy) * TILE + x * 2 + sx) * 3;
                        let p = &k[i..i + 3];
                        if !is_nodata(p) {
                            acc[0] += p[0] as u32;
                            acc[1] += p[1] as u32;
                            acc[2] += p[2] as u32;
                            n += 1;
                        }
                    }
                    if n > 0 {
                        let o = ((dy * half + y) * TILE + dx * half + x) * 3;
                        for c in 0..3 {
                            out[o + c] = ((acc[c] + n / 2) / n) as u8;
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
            }
        }
    }
    Ok(out)
}

pub fn write_jpeg(path: &Path, rgb: &[u8], w: usize, h: usize) -> Result<()> {
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d)?;
    }
    let file = std::io::BufWriter::new(std::fs::File::create(path)?);
    let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(file, JPEG_QUALITY);
    enc.encode(rgb, w as u32, h as u32, image::ExtendedColorType::Rgb8)?;
    Ok(())
}
