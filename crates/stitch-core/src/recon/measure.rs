//! Measurements on the surface model: area and volume inside an outline.
//!
//! Volume is measured against a level (a horizontal plane):
//! * **fill** — space between the surface and the level where the surface is
//!   below it (the water an empty pond would hold, filled to that level);
//! * **cut** — material above the level (a stockpile, a mound).
//!
//! The level can be the lowest point on the outline (where water would spill
//! over), the average height along the outline (a stockpile's base), or a
//! height the user types in.

use super::dsm::Dsm;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Level {
    /// Lowest surface point on the outline — "fill to the brim".
    LowestEdge,
    /// Mean surface height along the outline — the base of a stockpile.
    MeanEdge,
    /// Highest surface point on the outline.
    HighestEdge,
    /// A fixed height in the model's height reference (metres).
    Height { z: f64 },
}

#[derive(Debug, Clone, Serialize)]
pub struct VolumeResult {
    /// Horizontal (map) area inside the outline, m².
    pub area_m2: f64,
    /// Real surface area inside the outline (follows slopes), m².
    pub surface_area_m2: f64,
    /// Height of the level used, metres.
    pub level_z: f64,
    pub fill_m3: f64,
    pub cut_m3: f64,
    /// Lowest and highest surface point inside, metres.
    pub min_z: f64,
    pub max_z: f64,
    /// Cells inside the outline that had no surface data.
    pub missing_fraction: f64,
}

fn inside(poly: &[(f64, f64)], x: f64, y: f64) -> bool {
    let mut c = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            c = !c;
        }
        j = i;
    }
    c
}

/// Heights sampled along the outline every half cell.
fn edge_heights(dsm: &Dsm, poly: &[(f64, f64)]) -> Vec<f64> {
    let mut out = Vec::new();
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        let n = ((len / (dsm.cell * 0.5)).ceil() as usize).max(1);
        for k in 0..n {
            let t = k as f64 / n as f64;
            if let Some(z) = dsm.height(a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t) {
                out.push(z as f64);
            }
        }
    }
    out
}

pub fn volume(dsm: &Dsm, poly: &[(f64, f64)], level: Level) -> Option<VolumeResult> {
    if poly.len() < 3 {
        return None;
    }
    let edge = edge_heights(dsm, poly);
    let level_z = match level {
        Level::Height { z } => z,
        _ if edge.is_empty() => return None,
        Level::LowestEdge => edge.iter().copied().fold(f64::MAX, f64::min),
        Level::HighestEdge => edge.iter().copied().fold(f64::MIN, f64::max),
        Level::MeanEdge => edge.iter().sum::<f64>() / edge.len() as f64,
    };
    let xa = poly.iter().map(|p| p.0).fold(f64::MAX, f64::min);
    let xb = poly.iter().map(|p| p.0).fold(f64::MIN, f64::max);
    let ya = poly.iter().map(|p| p.1).fold(f64::MAX, f64::min);
    let yb = poly.iter().map(|p| p.1).fold(f64::MIN, f64::max);
    let c = dsm.cell;
    let i0 = ((xa - dsm.x0) / c).floor().max(0.0) as usize;
    let i1 = (((xb - dsm.x0) / c).ceil().max(0.0) as usize).min(dsm.nx);
    let j0 = ((ya - dsm.y0) / c).floor().max(0.0) as usize;
    let j1 = (((yb - dsm.y0) / c).ceil().max(0.0) as usize).min(dsm.ny);
    let (mut n_in, mut n_missing) = (0usize, 0usize);
    let (mut fill, mut cut, mut surf) = (0.0, 0.0, 0.0);
    let (mut zmin, mut zmax) = (f64::MAX, f64::MIN);
    for j in j0..j1 {
        for i in i0..i1 {
            let (x, y) = (dsm.x0 + (i as f64 + 0.5) * c, dsm.y0 + (j as f64 + 0.5) * c);
            if !inside(poly, x, y) {
                continue;
            }
            n_in += 1;
            let z = dsm.at(i, j);
            if !z.is_finite() {
                n_missing += 1;
                continue;
            }
            let z = z as f64;
            zmin = zmin.min(z);
            zmax = zmax.max(z);
            let d = z - level_z;
            if d > 0.0 {
                cut += d * c * c;
            } else {
                fill += -d * c * c;
            }
            // Slope from central differences gives the true surface area.
            let gx = match (dsm.height(x + c, y), dsm.height(x - c, y)) {
                (Some(a), Some(b)) => (a - b) as f64 / (2.0 * c),
                _ => 0.0,
            };
            let gy = match (dsm.height(x, y + c), dsm.height(x, y - c)) {
                (Some(a), Some(b)) => (a - b) as f64 / (2.0 * c),
                _ => 0.0,
            };
            surf += c * c * (1.0 + gx * gx + gy * gy).sqrt();
        }
    }
    if n_in == 0 {
        return None;
    }
    let area = n_in as f64 * c * c;
    // Cells with no data are assumed to sit at the average of the others,
    // so the volume isn't silently short; the fraction is reported.
    let known = (n_in - n_missing).max(1) as f64;
    let scale = n_in as f64 / known;
    Some(VolumeResult {
        area_m2: area,
        surface_area_m2: surf * scale,
        level_z,
        fill_m3: fill * scale,
        cut_m3: cut * scale,
        min_z: if zmin == f64::MAX { level_z } else { zmin },
        max_z: if zmax == f64::MIN { level_z } else { zmax },
        missing_fraction: n_missing as f64 / n_in as f64,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 20 × 20 m flat field at 10 m with a 10 × 10 m pit, 2 m deep.
    fn field() -> Dsm {
        let (nx, ny, cell) = (100, 100, 0.2);
        let mut h = vec![10.0f32; nx * ny];
        for j in 25..75 {
            for i in 25..75 {
                h[j * nx + i] = 8.0;
            }
        }
        Dsm {
            x0: 0.0,
            y0: 0.0,
            cell,
            nx,
            ny,
            h,
            zmax: 10.0,
        }
    }

    #[test]
    fn empty_pond_volume_to_the_brim() {
        let d = field();
        let poly = [(3.0, 3.0), (17.0, 3.0), (17.0, 17.0), (3.0, 17.0)];
        let r = volume(&d, &poly, Level::LowestEdge).unwrap();
        assert!((r.level_z - 10.0).abs() < 1e-6);
        // 10 m × 10 m × 2 m = 200 m³ of water.
        assert!((r.fill_m3 - 200.0).abs() < 1.0, "{r:?}");
        assert!(r.cut_m3 < 1e-6);
        assert!((r.area_m2 - 196.0).abs() < 1.0);
    }

    #[test]
    fn stockpile_above_its_base_and_custom_level() {
        let mut d = field();
        for v in d.h.iter_mut() {
            *v = if *v < 9.0 { 13.0 } else { 10.0 }; // a 3 m-high block
        }
        let poly = [(3.0, 3.0), (17.0, 3.0), (17.0, 17.0), (3.0, 17.0)];
        let r = volume(&d, &poly, Level::MeanEdge).unwrap();
        assert!((r.cut_m3 - 300.0).abs() < 1.0, "{r:?}");
        let half = volume(&d, &poly, Level::Height { z: 11.5 }).unwrap();
        assert!((half.cut_m3 - 150.0).abs() < 1.0, "{half:?}");
        // The block's steep sides add real surface area beyond the footprint.
        assert!(r.surface_area_m2 > r.area_m2 + 10.0);
    }
}
