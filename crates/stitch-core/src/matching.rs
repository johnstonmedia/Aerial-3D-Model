//! Pairwise matching: brute-force Hamming with ratio + mutual tests, then a
//! RANSAC similarity fit. Nadir drone photos from one altitude differ by
//! rotation and translation (plus a little scale), which a similarity covers;
//! a homography would mostly fit noise from terrain relief.

use crate::features::{Desc, Features};
use crate::sim::Sim2;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PairMatch {
    /// Inlier correspondences in working-image pixels: (xa, ya, xb, yb).
    pub points: Vec<[f32; 4]>,
    /// Maps A's working pixels onto B's.
    pub sim: Sim2,
}

#[inline]
fn hamming(a: &Desc, b: &Desc) -> u32 {
    (a[0] ^ b[0]).count_ones()
        + (a[1] ^ b[1]).count_ones()
        + (a[2] ^ b[2]).count_ones()
        + (a[3] ^ b[3]).count_ones()
}

fn best_two(d: &Desc, set: &[Desc]) -> (usize, u32, u32) {
    let (mut bi, mut b1, mut b2) = (usize::MAX, u32::MAX, u32::MAX);
    for (i, s) in set.iter().enumerate() {
        let h = hamming(d, s);
        if h < b1 {
            b2 = b1;
            b1 = h;
            bi = i;
        } else if h < b2 {
            b2 = h;
        }
    }
    (bi, b1, b2)
}

/// Putative correspondences (index in A, index in B).
pub fn putative(a: &Features, b: &Features) -> Vec<(usize, usize)> {
    let back: Vec<usize> = b.desc.iter().map(|d| best_two(d, &a.desc).0).collect();
    let mut out = Vec::new();
    for (i, d) in a.desc.iter().enumerate() {
        let (j, d1, d2) = best_two(d, &b.desc);
        if j == usize::MAX || d1 > 64 {
            continue;
        }
        if (d1 as f32) > 0.8 * d2 as f32 {
            continue;
        }
        if back[j] != i {
            continue;
        }
        out.push((i, j));
    }
    out
}

pub struct MatchParams {
    pub min_inliers: usize,
    pub threshold_px: f64,
    pub iterations: usize,
}

impl Default for MatchParams {
    fn default() -> Self {
        Self {
            min_inliers: 16,
            threshold_px: 3.0,
            iterations: 1500,
        }
    }
}

pub fn match_pair(a: &Features, b: &Features, seed: u64, p: &MatchParams) -> Option<PairMatch> {
    let pairs = putative(a, b);
    if pairs.len() < p.min_inliers {
        return None;
    }
    let src: Vec<(f64, f64)> = pairs
        .iter()
        .map(|&(i, _)| (a.kps[i].x as f64, a.kps[i].y as f64))
        .collect();
    let dst: Vec<(f64, f64)> = pairs
        .iter()
        .map(|&(_, j)| (b.kps[j].x as f64, b.kps[j].y as f64))
        .collect();

    let mut s = seed | 1;
    let mut rnd = |n: usize| {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s % n as u64) as usize
    };
    let t2 = p.threshold_px * p.threshold_px;
    let count = |m: &Sim2, keep: Option<&mut Vec<usize>>| {
        let mut n = 0;
        let mut keep = keep;
        for k in 0..src.len() {
            let (x, y) = m.apply(src[k].0, src[k].1);
            let (dx, dy) = (x - dst[k].0, y - dst[k].1);
            if dx * dx + dy * dy < t2 {
                n += 1;
                if let Some(v) = keep.as_deref_mut() {
                    v.push(k);
                }
            }
        }
        n
    };
    let plausible = |m: &Sim2| (0.33..3.0).contains(&m.scale());

    let mut best: Option<(usize, Sim2)> = None;
    for _ in 0..p.iterations {
        let i = rnd(src.len());
        let j = rnd(src.len());
        if i == j {
            continue;
        }
        let Some(m) = Sim2::fit(&[src[i], src[j]], &[dst[i], dst[j]]) else {
            continue;
        };
        if !plausible(&m) {
            continue;
        }
        let n = count(&m, None);
        if best.as_ref().map_or(true, |b| n > b.0) {
            best = Some((n, m));
        }
    }
    let (_, mut model) = best?;
    // Refine on inliers twice; the second pass picks up points the
    // two-point hypothesis was slightly too strict about.
    let mut idx = Vec::new();
    for _ in 0..2 {
        idx.clear();
        count(&model, Some(&mut idx));
        if idx.len() < p.min_inliers {
            return None;
        }
        let s2: Vec<_> = idx.iter().map(|&k| src[k]).collect();
        let d2: Vec<_> = idx.iter().map(|&k| dst[k]).collect();
        model = Sim2::fit(&s2, &d2)?;
    }
    idx.clear();
    count(&model, Some(&mut idx));
    if idx.len() < p.min_inliers || !plausible(&model) {
        return None;
    }
    // Reject degenerate clusters: inliers must cover a reasonable area,
    // otherwise one repeated texture patch can fake a match.
    let (mut minx, mut maxx, mut miny, mut maxy) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for &k in &idx {
        minx = minx.min(src[k].0);
        maxx = maxx.max(src[k].0);
        miny = miny.min(src[k].1);
        maxy = maxy.max(src[k].1);
    }
    let span = (maxx - minx).max(maxy - miny);
    if span < 0.08 * a.w.max(a.h) as f64 {
        return None;
    }
    let points = idx
        .iter()
        .map(|&k| {
            [
                src[k].0 as f32,
                src[k].1 as f32,
                dst[k].0 as f32,
                dst[k].1 as f32,
            ]
        })
        .collect();
    Some(PairMatch { points, sim: model })
}
