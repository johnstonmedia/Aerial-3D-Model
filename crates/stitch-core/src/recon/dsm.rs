//! Digital surface model: the height of the top surface (ground, roofs,
//! canopy) on a regular grid, built from the dense point cloud.
//!
//! It is what makes a true orthophoto possible (every pixel is projected from
//! its real height, so nothing leans) and it is the shape of the textured
//! surface model.

use super::ply::read_ply;
use crate::profile::{Profile, ProfileKind};
use anyhow::{bail, Result};
use std::path::Path;

#[derive(Clone)]
pub struct Dsm {
    /// World coordinate of the south-west corner of cell (0, 0).
    pub x0: f64,
    pub y0: f64,
    pub cell: f64,
    pub nx: usize,
    pub ny: usize,
    /// Row-major from the south (row 0 = southernmost). NaN = no surface.
    pub h: Vec<f32>,
    pub zmax: f32,
}

impl Dsm {
    /// Binary save: JSON can't carry NaN (it becomes null), and NaN is how an
    /// empty cell is marked.
    pub fn save(&self, path: &Path) -> Result<()> {
        let mut b = Vec::with_capacity(48 + self.h.len() * 4);
        b.extend_from_slice(b"DSM1");
        for v in [self.x0, self.y0, self.cell] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        b.extend_from_slice(&(self.nx as u64).to_le_bytes());
        b.extend_from_slice(&(self.ny as u64).to_le_bytes());
        b.extend_from_slice(&self.zmax.to_le_bytes());
        for v in &self.h {
            b.extend_from_slice(&v.to_le_bytes());
        }
        let tmp = path.with_extension("tmp");
        std::fs::write(&tmp, b)?;
        std::fs::rename(&tmp, path)?;
        Ok(())
    }

    pub fn load(path: &Path) -> Result<Dsm> {
        let b = std::fs::read(path)?;
        if b.len() < 48 || &b[..4] != b"DSM1" {
            bail!("damaged surface model file");
        }
        let f64_at = |o: usize| f64::from_le_bytes(b[o..o + 8].try_into().unwrap());
        let (x0, y0, cell) = (f64_at(4), f64_at(12), f64_at(20));
        let nx = u64::from_le_bytes(b[28..36].try_into()?) as usize;
        let ny = u64::from_le_bytes(b[36..44].try_into()?) as usize;
        let zmax = f32::from_le_bytes(b[44..48].try_into()?);
        if b.len() != 48 + nx * ny * 4 {
            bail!("truncated surface model file");
        }
        let h = b[48..]
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
            .collect();
        Ok(Dsm {
            x0,
            y0,
            cell,
            nx,
            ny,
            h,
            zmax,
        })
    }

    #[inline]
    pub fn at(&self, i: usize, j: usize) -> f32 {
        self.h[j * self.nx + i]
    }

    /// Bilinear height at a world position (cell centres are the samples).
    /// Falls back to the nearest valid sample at the edge of data.
    pub fn height(&self, x: f64, y: f64) -> Option<f32> {
        let fx = (x - self.x0) / self.cell - 0.5;
        let fy = (y - self.y0) / self.cell - 0.5;
        if fx < -0.5 || fy < -0.5 || fx > self.nx as f64 - 0.5 || fy > self.ny as f64 - 0.5 {
            return None;
        }
        let i0 = (fx.floor().max(0.0) as usize).min(self.nx - 1);
        let j0 = (fy.floor().max(0.0) as usize).min(self.ny - 1);
        let i1 = (i0 + 1).min(self.nx - 1);
        let j1 = (j0 + 1).min(self.ny - 1);
        let ax = (fx - i0 as f64).clamp(0.0, 1.0) as f32;
        let ay = (fy - j0 as f64).clamp(0.0, 1.0) as f32;
        let v = [
            self.at(i0, j0),
            self.at(i1, j0),
            self.at(i0, j1),
            self.at(i1, j1),
        ];
        let w = [
            (1.0 - ax) * (1.0 - ay),
            ax * (1.0 - ay),
            (1.0 - ax) * ay,
            ax * ay,
        ];
        let (mut s, mut ws) = (0.0, 0.0);
        for k in 0..4 {
            if v[k].is_finite() {
                s += v[k] * w[k];
                ws += w[k];
            }
        }
        if ws > 1e-6 {
            return Some(s / ws);
        }
        // Sitting exactly on an empty sample: take any valid neighbour.
        v.iter().copied().find(|x| x.is_finite())
    }
}

fn budget_cells(p: &Profile) -> usize {
    match p.kind {
        ProfileKind::Lite => 3_000_000,
        ProfileKind::Standard => 8_000_000,
        ProfileKind::High => 20_000_000,
    }
}

pub fn build_dsm(dense_ply: &Path, voxel_m: f64, profile: &Profile) -> Result<Dsm> {
    // Extent: 0.5–99.5th percentile, so a few stray points can't blow it up.
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    read_ply(dense_ply, |p, _| {
        xs.push(p[0]);
        ys.push(p[1]);
    })?;
    if xs.len() < 100 {
        bail!("the point cloud is too small to build a surface");
    }
    xs.sort_by(f32::total_cmp);
    ys.sort_by(f32::total_cmp);
    let q = |v: &[f32], f: f64| v[((v.len() - 1) as f64 * f) as usize] as f64;
    let (xa, xb, ya, yb) = (q(&xs, 0.005), q(&xs, 0.995), q(&ys, 0.005), q(&ys, 0.995));
    drop((xs, ys));
    let area = (xb - xa) * (yb - ya);
    let cell = (voxel_m * 1.5).max((area / budget_cells(profile) as f64).sqrt());
    let nx = ((xb - xa) / cell).ceil() as usize + 1;
    let ny = ((yb - ya) / cell).ceil() as usize + 1;

    // Top three heights per cell: the second highest rejects a lone spike
    // while still following roofs and canopy rather than the ground under them.
    let mut top = vec![[f32::NEG_INFINITY; 3]; nx * ny];
    let mut count = vec![0u16; nx * ny];
    read_ply(dense_ply, |p, _| {
        let i = ((p[0] as f64 - xa) / cell).floor();
        let j = ((p[1] as f64 - ya) / cell).floor();
        if i < 0.0 || j < 0.0 || i >= nx as f64 || j >= ny as f64 {
            return;
        }
        let k = j as usize * nx + i as usize;
        count[k] = count[k].saturating_add(1);
        let t = &mut top[k];
        let z = p[2];
        if z > t[0] {
            t[2] = t[1];
            t[1] = t[0];
            t[0] = z;
        } else if z > t[1] {
            t[2] = t[1];
            t[1] = z;
        } else if z > t[2] {
            t[2] = z;
        }
    })?;
    let mut h: Vec<f32> = (0..nx * ny)
        .map(|k| match count[k] {
            0 => f32::NAN,
            1 => top[k][0],
            _ => top[k][1],
        })
        .collect();
    drop(top);

    // Despike: a thinly supported cell far from its neighbourhood is noise.
    let med = |h: &[f32], i: usize, j: usize, r: i64| -> Option<f32> {
        let mut v = Vec::with_capacity(25);
        for dj in -r..=r {
            for di in -r..=r {
                let (a, b) = (i as i64 + di, j as i64 + dj);
                if a >= 0 && b >= 0 && (a as usize) < nx && (b as usize) < ny {
                    let x = h[b as usize * nx + a as usize];
                    if x.is_finite() {
                        v.push(x);
                    }
                }
            }
        }
        if v.len() < 5 {
            return None;
        }
        v.sort_by(f32::total_cmp);
        Some(v[v.len() / 2])
    };
    let snapshot = h.clone();
    for j in 0..ny {
        for i in 0..nx {
            let k = j * nx + i;
            if !snapshot[k].is_finite() {
                continue;
            }
            if let Some(m) = med(&snapshot, i, j, 2) {
                if (snapshot[k] - m).abs() > 3.0 && count[k] <= 2 {
                    h[k] = m;
                }
            } else if count[k] <= 2 {
                h[k] = f32::NAN; // isolated: no surface here
            }
        }
    }

    // Drop small islands: clumps of stray points floating off the edge of the
    // survey would otherwise become specks of surface in the orthophoto.
    remove_islands(&mut h, nx, ny, (25.0 / (cell * cell)) as usize);
    // Holes fully enclosed by surface (ponds, plain concrete) are filled
    // completely; holes open to the outside only along their rim.
    fill_enclosed(&mut h, nx, ny);

    // Fill small holes (water, plain concrete) from their surroundings.
    for _ in 0..8 {
        let snap = h.clone();
        for j in 0..ny {
            for i in 0..nx {
                let k = j * nx + i;
                if snap[k].is_finite() {
                    continue;
                }
                let (mut s, mut c) = (0.0, 0);
                for (di, dj) in [
                    (-1i64, 0i64),
                    (1, 0),
                    (0, -1),
                    (0, 1),
                    (-1, -1),
                    (1, 1),
                    (-1, 1),
                    (1, -1),
                ] {
                    let (a, b) = (i as i64 + di, j as i64 + dj);
                    if a >= 0 && b >= 0 && (a as usize) < nx && (b as usize) < ny {
                        let x = snap[b as usize * nx + a as usize];
                        if x.is_finite() {
                            s += x;
                            c += 1;
                        }
                    }
                }
                if c >= 3 {
                    h[k] = s / c as f32;
                }
            }
        }
    }

    // Light 3×3 median: keeps roof edges, removes speckle.
    let snap = h.clone();
    for j in 0..ny {
        for i in 0..nx {
            let k = j * nx + i;
            if snap[k].is_finite() {
                if let Some(m) = med(&snap, i, j, 1) {
                    h[k] = m;
                }
            }
        }
    }
    let zmax = h
        .iter()
        .copied()
        .filter(|v| v.is_finite())
        .fold(f32::MIN, f32::max);
    Ok(Dsm {
        x0: xa,
        y0: ya,
        cell,
        nx,
        ny,
        h,
        zmax,
    })
}

/// 4-connected components of valid cells smaller than `min_cells` are removed.
fn remove_islands(h: &mut [f32], nx: usize, ny: usize, min_cells: usize) {
    let mut label = vec![u32::MAX; nx * ny];
    let mut stack = Vec::new();
    let mut comp = Vec::new();
    for start in 0..nx * ny {
        if !h[start].is_finite() || label[start] != u32::MAX {
            continue;
        }
        comp.clear();
        stack.push(start);
        label[start] = 0;
        while let Some(k) = stack.pop() {
            comp.push(k);
            let (i, j) = (k % nx, k / nx);
            let mut push = |kk: usize| {
                if h[kk].is_finite() && label[kk] == u32::MAX {
                    label[kk] = 0;
                    stack.push(kk);
                }
            };
            if i > 0 {
                push(k - 1);
            }
            if i + 1 < nx {
                push(k + 1);
            }
            if j > 0 {
                push(k - nx);
            }
            if j + 1 < ny {
                push(k + nx);
            }
        }
        if comp.len() < min_cells {
            for &k in &comp {
                h[k] = f32::NAN;
            }
        }
    }
}

/// Fills empty regions that don't touch the grid border, by repeated
/// averaging from their rim until closed.
fn fill_enclosed(h: &mut [f32], nx: usize, ny: usize) {
    // Empty cells reachable from the border are "outside".
    let mut outside = vec![false; nx * ny];
    let mut stack: Vec<usize> = Vec::new();
    for i in 0..nx {
        stack.push(i);
        stack.push((ny - 1) * nx + i);
    }
    for j in 0..ny {
        stack.push(j * nx);
        stack.push(j * nx + nx - 1);
    }
    while let Some(k) = stack.pop() {
        if h[k].is_finite() || outside[k] {
            continue;
        }
        outside[k] = true;
        let (i, j) = (k % nx, k / nx);
        if i > 0 {
            stack.push(k - 1);
        }
        if i + 1 < nx {
            stack.push(k + 1);
        }
        if j > 0 {
            stack.push(k - nx);
        }
        if j + 1 < ny {
            stack.push(k + nx);
        }
    }
    let mut todo: Vec<usize> = (0..nx * ny)
        .filter(|&k| !h[k].is_finite() && !outside[k])
        .collect();
    let mut guard = 0;
    while !todo.is_empty() && guard < 10_000 {
        guard += 1;
        let snap = h.to_vec();
        todo.retain(|&k| {
            let (i, j) = (k % nx, k / nx);
            let (mut s, mut c) = (0.0, 0);
            for (di, dj) in [(-1i64, 0i64), (1, 0), (0, -1), (0, 1)] {
                let (a, b) = (i as i64 + di, j as i64 + dj);
                if a >= 0 && b >= 0 && (a as usize) < nx && (b as usize) < ny {
                    let x = snap[b as usize * nx + a as usize];
                    if x.is_finite() {
                        s += x;
                        c += 1;
                    }
                }
            }
            if c > 0 {
                h[k] = s / c as f32;
                false
            } else {
                true
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bilinear_height_and_gaps() {
        let mut d = Dsm {
            x0: 0.0,
            y0: 0.0,
            cell: 1.0,
            nx: 3,
            ny: 3,
            h: vec![10.0; 9],
            zmax: 12.0,
        };
        d.h[4] = 12.0; // centre cell (1,1) at world (1.5, 1.5)
        assert!((d.height(1.5, 1.5).unwrap() - 12.0).abs() < 1e-5);
        assert!((d.height(1.0, 1.5).unwrap() - 11.0).abs() < 1e-5);
        d.h[0] = f32::NAN;
        assert!(d.height(0.5, 0.5).is_some()); // falls back to valid neighbours
        assert!(d.height(-3.0, 0.0).is_none());
        let p = std::env::temp_dir().join(format!("dsm{}.bin", std::process::id()));
        d.save(&p).unwrap();
        let e = Dsm::load(&p).unwrap();
        assert!(e.h[0].is_nan() && e.h[4] == 12.0 && e.nx == 3 && e.cell == 1.0);
        std::fs::remove_file(p).ok();
    }

    #[test]
    fn islands_removed_and_enclosed_holes_filled() {
        let (nx, ny) = (10, 10);
        let mut h = vec![f32::NAN; nx * ny];
        // A 6×6 block with a 2×2 hole in it, plus a lone cell far away.
        for j in 2..8 {
            for i in 2..8 {
                h[j * nx + i] = 5.0;
            }
        }
        h[4 * nx + 4] = f32::NAN;
        h[4 * nx + 5] = f32::NAN;
        h[5 * nx + 4] = f32::NAN;
        h[5 * nx + 5] = f32::NAN;
        h[0] = 9.0;
        remove_islands(&mut h, nx, ny, 3);
        assert!(h[0].is_nan());
        fill_enclosed(&mut h, nx, ny);
        assert_eq!(h[4 * nx + 4], 5.0);
        assert!(h[nx * 9 + 9].is_nan()); // outside stays empty
    }
}
