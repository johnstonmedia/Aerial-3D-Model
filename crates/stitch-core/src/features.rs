//! ORB-style features: FAST-9 corners ranked by Harris response, spread over
//! a grid, oriented by intensity centroid, described by steered 256-bit BRIEF.
//!
//! Rotation invariance matters for drone surveys: on a lawnmower pattern every
//! other flight line is shot facing the opposite way.

use crate::imgops::{box_blur, Gray};
use anyhow::{bail, Result};
use std::io::{Read, Write};
use std::sync::OnceLock;

pub type Desc = [u64; 4];

#[derive(Debug, Clone, Copy)]
pub struct Keypoint {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub score: f32,
}

/// Features for one photo, in working-image pixel coordinates.
pub struct Features {
    /// Working image size.
    pub w: u32,
    pub h: u32,
    /// working / full-resolution scale.
    pub scale: f64,
    pub kps: Vec<Keypoint>,
    pub desc: Vec<Desc>,
}

const PATCH_R: i32 = 15;
const BORDER: usize = 18;
const ANGLE_BINS: usize = 30;

const CIRCLE: [(i32, i32); 16] = [
    (0, -3), (1, -3), (2, -2), (3, -1), (3, 0), (3, 1), (2, 2), (1, 3),
    (0, 3), (-1, 3), (-2, 2), (-3, 1), (-3, 0), (-3, -1), (-2, -2), (-1, -3),
];

/// 256 point pairs, pre-rotated into `ANGLE_BINS` orientations.
fn patterns() -> &'static Vec<Vec<[(i8, i8); 2]>> {
    static P: OnceLock<Vec<Vec<[(i8, i8); 2]>>> = OnceLock::new();
    P.get_or_init(|| {
        // Deterministic LCG so descriptors are identical on every machine and
        // cached features stay valid across runs.
        let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut rnd = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 11) as f64) / ((1u64 << 53) as f64)
        };
        let mut gauss = || {
            let u1: f64 = rnd().max(1e-12);
            let u2: f64 = rnd();
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos() * 6.2
        };
        let mut base = Vec::with_capacity(256);
        while base.len() < 256 {
            let p = (gauss(), gauss());
            let q = (gauss(), gauss());
            let ok = |v: (f64, f64)| v.0 * v.0 + v.1 * v.1 <= 13.0 * 13.0;
            if ok(p) && ok(q) {
                base.push([p, q]);
            }
        }
        (0..ANGLE_BINS)
            .map(|bin| {
                let a = bin as f64 * 2.0 * std::f64::consts::PI / ANGLE_BINS as f64;
                let (sn, cs) = a.sin_cos();
                base.iter()
                    .map(|pair| {
                        let r = |v: (f64, f64)| {
                            ((v.0 * cs - v.1 * sn).round() as i8, (v.0 * sn + v.1 * cs).round() as i8)
                        };
                        [r(pair[0]), r(pair[1])]
                    })
                    .collect()
            })
            .collect()
    })
}

fn is_fast_corner(g: &Gray, x: usize, y: usize, t: i16) -> bool {
    let c = g.at(x, y) as i16;
    let px = |k: usize| {
        let (dx, dy) = CIRCLE[k % 16];
        g.at((x as i32 + dx) as usize, (y as i32 + dy) as usize) as i16
    };
    // Quick reject on the four compass points: a 9-arc needs ≥ 2 of them.
    let quad = [px(0), px(4), px(8), px(12)];
    let bright = quad.iter().filter(|&&v| v > c + t).count();
    let dark = quad.iter().filter(|&&v| v < c - t).count();
    if bright < 2 && dark < 2 {
        return false;
    }
    let mut ring = [0i8; 16];
    for (k, r) in ring.iter_mut().enumerate() {
        let v = px(k);
        *r = if v > c + t { 1 } else if v < c - t { -1 } else { 0 };
    }
    for sign in [1i8, -1] {
        let mut run = 0;
        for k in 0..25 {
            if ring[k % 16] == sign {
                run += 1;
                if run >= 9 {
                    return true;
                }
            } else {
                run = 0;
            }
        }
    }
    false
}

fn harris(g: &Gray, x: usize, y: usize) -> f32 {
    let (mut sxx, mut syy, mut sxy) = (0f32, 0f32, 0f32);
    for yy in y - 3..=y + 3 {
        for xx in x - 3..=x + 3 {
            let dx = g.at(xx + 1, yy) as f32 - g.at(xx - 1, yy) as f32;
            let dy = g.at(xx, yy + 1) as f32 - g.at(xx, yy - 1) as f32;
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }
    }
    sxx * syy - sxy * sxy - 0.04 * (sxx + syy) * (sxx + syy)
}

fn orientation(g: &Gray, x: usize, y: usize) -> f32 {
    let (mut m01, mut m10) = (0i64, 0i64);
    for dy in -PATCH_R..=PATCH_R {
        let span = ((PATCH_R * PATCH_R - dy * dy) as f64).sqrt() as i32;
        for dx in -span..=span {
            let v = g.at((x as i32 + dx) as usize, (y as i32 + dy) as usize) as i64;
            m10 += dx as i64 * v;
            m01 += dy as i64 * v;
        }
    }
    (m01 as f32).atan2(m10 as f32)
}

pub fn detect(gray: &Gray, max_features: usize, scale: f64) -> Features {
    let (w, h) = (gray.w, gray.h);
    let mut cands: Vec<(usize, usize)> = Vec::new();
    if w > 2 * BORDER && h > 2 * BORDER {
        let pre = box_blur(gray, 1);
        for t in [20i16, 12, 7] {
            cands.clear();
            for y in BORDER..h - BORDER {
                for x in BORDER..w - BORDER {
                    if is_fast_corner(&pre, x, y, t) {
                        cands.push((x, y));
                    }
                }
            }
            if cands.len() >= max_features * 2 {
                break;
            }
        }
    }

    // Score, then non-max suppress in 3x3 and spread over a grid.
    let mut scored: Vec<(usize, usize, f32)> =
        cands.iter().map(|&(x, y)| (x, y, harris(gray, x, y))).collect();
    let mut score_map = std::collections::HashMap::with_capacity(scored.len());
    for &(x, y, s) in &scored {
        score_map.insert((x, y), s);
    }
    scored.retain(|&(x, y, s)| {
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                if (dx, dy) != (0, 0) {
                    if let Some(&o) = score_map.get(&((x as i32 + dx) as usize, (y as i32 + dy) as usize)) {
                        if o > s {
                            return false;
                        }
                    }
                }
            }
        }
        true
    });

    const GRID: usize = 12;
    let cell_w = (w / GRID).max(1);
    let cell_h = (h / GRID).max(1);
    let mut cells: Vec<Vec<(usize, usize, f32)>> = vec![Vec::new(); GRID * GRID];
    for p in scored {
        let cx = (p.0 / cell_w).min(GRID - 1);
        let cy = (p.1 / cell_h).min(GRID - 1);
        cells[cy * GRID + cx].push(p);
    }
    let quota = (max_features / (GRID * GRID)).max(1);
    let mut chosen = Vec::with_capacity(max_features);
    let mut leftovers = Vec::new();
    for mut c in cells {
        c.sort_by(|a, b| b.2.total_cmp(&a.2));
        let k = quota.min(c.len());
        chosen.extend_from_slice(&c[..k]);
        leftovers.extend_from_slice(&c[k..]);
    }
    if chosen.len() < max_features {
        leftovers.sort_by(|a, b| b.2.total_cmp(&a.2));
        let need = max_features - chosen.len();
        chosen.extend(leftovers.into_iter().take(need));
    }

    let smooth = box_blur(gray, 2);
    let pats = patterns();
    let mut kps = Vec::with_capacity(chosen.len());
    let mut desc = Vec::with_capacity(chosen.len());
    for (x, y, s) in chosen {
        let angle = orientation(&smooth, x, y);
        let bin = (((angle as f64).rem_euclid(2.0 * std::f64::consts::PI))
            / (2.0 * std::f64::consts::PI)
            * ANGLE_BINS as f64)
            .round() as usize
            % ANGLE_BINS;
        let mut d: Desc = [0; 4];
        for (i, pair) in pats[bin].iter().enumerate() {
            let a = smooth.at((x as i32 + pair[0].0 as i32) as usize, (y as i32 + pair[0].1 as i32) as usize);
            let b = smooth.at((x as i32 + pair[1].0 as i32) as usize, (y as i32 + pair[1].1 as i32) as usize);
            if a < b {
                d[i / 64] |= 1 << (i % 64);
            }
        }
        kps.push(Keypoint { x: x as f32 + 0.5, y: y as f32 + 0.5, angle, score: s });
        desc.push(d);
    }
    Features { w: w as u32, h: h as u32, scale, kps, desc }
}

const MAGIC: &[u8; 4] = b"AF01";

impl Features {
    pub fn write(&self, mut out: impl Write) -> Result<()> {
        out.write_all(MAGIC)?;
        out.write_all(&self.w.to_le_bytes())?;
        out.write_all(&self.h.to_le_bytes())?;
        out.write_all(&self.scale.to_le_bytes())?;
        out.write_all(&(self.kps.len() as u32).to_le_bytes())?;
        for k in &self.kps {
            for v in [k.x, k.y, k.angle, k.score] {
                out.write_all(&v.to_le_bytes())?;
            }
        }
        for d in &self.desc {
            for v in d {
                out.write_all(&v.to_le_bytes())?;
            }
        }
        Ok(())
    }

    pub fn read(mut inp: impl Read) -> Result<Self> {
        let mut buf = Vec::new();
        inp.read_to_end(&mut buf)?;
        if buf.len() < 24 || &buf[..4] != MAGIC {
            bail!("bad feature file");
        }
        let u32_at = |o: usize| u32::from_le_bytes(buf[o..o + 4].try_into().unwrap());
        let f32_at = |o: usize| f32::from_le_bytes(buf[o..o + 4].try_into().unwrap());
        let w = u32_at(4);
        let h = u32_at(8);
        let scale = f64::from_le_bytes(buf[12..20].try_into().unwrap());
        let n = u32_at(20) as usize;
        let need = 24 + n * 16 + n * 32;
        if buf.len() != need {
            bail!("truncated feature file");
        }
        let mut kps = Vec::with_capacity(n);
        for i in 0..n {
            let o = 24 + i * 16;
            kps.push(Keypoint { x: f32_at(o), y: f32_at(o + 4), angle: f32_at(o + 8), score: f32_at(o + 12) });
        }
        let mut desc = Vec::with_capacity(n);
        let base = 24 + n * 16;
        for i in 0..n {
            let mut d = [0u64; 4];
            for (j, v) in d.iter_mut().enumerate() {
                let o = base + i * 32 + j * 8;
                *v = u64::from_le_bytes(buf[o..o + 8].try_into().unwrap());
            }
            desc.push(d);
        }
        Ok(Features { w, h, scale, kps, desc })
    }
}
