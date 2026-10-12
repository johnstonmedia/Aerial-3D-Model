//! Bundle adjustment: Levenberg–Marquardt over camera poses, shared lens
//! intrinsics and 3D points, with a robust (Cauchy) loss and GPS priors on the
//! camera centres.
//!
//! Points are eliminated with the Schur complement, leaving a sparse block
//! system over cameras (+ one block per lens) that is solved with
//! block-Jacobi preconditioned conjugate gradients. Nothing dense in the
//! number of cameras is ever formed, so memory grows with observations, not
//! with cameras squared.

use super::geometry::{exp_so3, skew, Intrinsics, Pose, M3, V3};
use nalgebra::{SMatrix, SVector, Vector2};
use rayon::prelude::*;
use std::collections::HashMap;

type M6 = SMatrix<f64, 6, 6>;
type M63 = SMatrix<f64, 6, 3>;
type V6 = SVector<f64, 6>;
type M23 = SMatrix<f64, 2, 3>;

#[derive(Debug, Clone, Copy)]
pub struct Obs {
    pub cam: u32,
    pub pt: u32,
    pub u: Vector2<f64>,
}

pub struct Problem {
    pub intr: Vec<Intrinsics>,
    pub poses: Vec<Pose>,
    /// Lens group of each camera (index into `intr`).
    pub group: Vec<usize>,
    /// Prior on each camera centre: (position, per-axis sigma in metres).
    pub prior: Vec<Option<(V3, V3)>>,
    pub points: Vec<V3>,
    /// Sorted by point.
    pub obs: Vec<Obs>,
    pub refine_intrinsics: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct BaStats {
    pub iterations: usize,
    pub initial_rms: f64,
    pub final_rms: f64,
}

struct Lin {
    e: Vector2<f64>,
    jc: SMatrix<f64, 2, 6>,
    jg: M23,
    jp: M23,
    w: f64,
}

fn linearise(p: &Problem, o: &Obs, sigma: f64) -> Option<Lin> {
    let pose = &p.poses[o.cam as usize];
    let k = &p.intr[p.group[o.cam as usize]];
    let x = &p.points[o.pt as usize];
    let xc = pose.to_cam(x);
    if xc.z <= 1e-6 {
        return None;
    }
    let iz = 1.0 / xc.z;
    let pn = Vector2::new(xc.x * iz, xc.y * iz);
    let r2 = pn.norm_squared();
    let d = 1.0 + k.k1 * r2 + k.k2 * r2 * r2;
    let q = pn * d;
    let pred = Vector2::new(k.f * q.x + k.cx, k.f * q.y + k.cy);
    let e = o.u - pred;
    // du/dp = f (d I + 2 (k1 + 2 k2 r2) p pᵀ)
    let c = 2.0 * (k.k1 + 2.0 * k.k2 * r2);
    let dudp = nalgebra::Matrix2::new(
        d + c * pn.x * pn.x,
        c * pn.x * pn.y,
        c * pn.x * pn.y,
        d + c * pn.y * pn.y,
    ) * k.f;
    let dpdx = M23::new(iz, 0.0, -xc.x * iz * iz, 0.0, iz, -xc.y * iz * iz);
    let dudx = dudp * dpdx; // 2×3, w.r.t. camera-frame point
    let j_rot = dudx * (-skew(&xc));
    let j_c = dudx * (-pose.r);
    let jp = dudx * pose.r;
    let mut jc = SMatrix::<f64, 2, 6>::zeros();
    jc.fixed_view_mut::<2, 3>(0, 0).copy_from(&j_rot);
    jc.fixed_view_mut::<2, 3>(0, 3).copy_from(&j_c);
    let jg = if p.refine_intrinsics {
        M23::new(
            q.x,
            k.f * pn.x * r2,
            k.f * pn.x * r2 * r2,
            q.y,
            k.f * pn.y * r2,
            k.f * pn.y * r2 * r2,
        )
    } else {
        M23::zeros()
    };
    let w = 1.0 / (1.0 + e.norm_squared() / (sigma * sigma));
    Some(Lin { e, jc, jg, jp, w })
}

fn robust_cost(p: &Problem, sigma: f64) -> (f64, f64, usize) {
    let s2 = sigma * sigma;
    let (cost, sq, n) = p
        .obs
        .par_iter()
        .map(|o| {
            let pose = &p.poses[o.cam as usize];
            let k = &p.intr[p.group[o.cam as usize]];
            match pose.project(k, &p.points[o.pt as usize]) {
                Some(u) => {
                    let e2 = (o.u - u).norm_squared();
                    (s2 * (1.0 + e2 / s2).ln(), e2, 1usize)
                }
                None => (s2 * 10.0, 0.0, 0),
            }
        })
        .reduce(|| (0.0, 0.0, 0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2));
    let mut prior = 0.0;
    for (i, pr) in p.prior.iter().enumerate() {
        if let Some((c, s)) = pr {
            let d = p.poses[i].c - c;
            prior += (d.x / s.x).powi(2) + (d.y / s.y).powi(2) + (d.z / s.z).powi(2);
        }
    }
    (cost + prior, (sq / n.max(1) as f64).sqrt(), n)
}

/// Sparse symmetric block matrix (full storage), blocks of 6.
struct Sparse {
    rows: Vec<HashMap<usize, M6>>,
}

impl Sparse {
    fn new(n: usize) -> Self {
        Self {
            rows: (0..n).map(|_| HashMap::new()).collect(),
        }
    }
    fn add(&mut self, i: usize, k: usize, m: &M6) {
        *self.rows[i].entry(k).or_insert_with(M6::zeros) += m;
    }
    fn mul(&self, x: &[V6], out: &mut [V6]) {
        out.par_iter_mut().enumerate().for_each(|(i, o)| {
            let mut s = V6::zeros();
            for (k, m) in &self.rows[i] {
                s += m * x[*k];
            }
            *o = s;
        });
    }
}

fn pcg(s: &Sparse, b: &[V6], max_iter: usize) -> Vec<V6> {
    let n = b.len();
    let pre: Vec<M6> = (0..n)
        .map(|i| {
            let d = s.rows[i].get(&i).copied().unwrap_or_else(M6::identity);
            d.try_inverse().unwrap_or_else(M6::identity)
        })
        .collect();
    let dot = |a: &[V6], c: &[V6]| a.iter().zip(c).map(|(x, y)| x.dot(y)).sum::<f64>();
    let mut x = vec![V6::zeros(); n];
    let mut r: Vec<V6> = b.to_vec();
    let mut z: Vec<V6> = r.iter().zip(&pre).map(|(r, m)| m * r).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let b2 = dot(b, b).max(1e-300);
    let mut q = vec![V6::zeros(); n];
    for _ in 0..max_iter {
        s.mul(&p, &mut q);
        let pq = dot(&p, &q);
        if pq.abs() < 1e-300 {
            break;
        }
        let a = rz / pq;
        for i in 0..n {
            x[i] += p[i] * a;
            r[i] -= q[i] * a;
        }
        if dot(&r, &r) < 1e-20 * b2 {
            break;
        }
        for i in 0..n {
            z[i] = pre[i] * r[i];
        }
        let rz2 = dot(&r, &z);
        let beta = rz2 / rz;
        rz = rz2;
        for i in 0..n {
            p[i] = z[i] + p[i] * beta;
        }
    }
    x
}

pub fn bundle_adjust(p: &mut Problem, max_iter: usize, sigma_px: f64) -> BaStats {
    let nc = p.poses.len();
    let ng = p.intr.len();
    let nb = nc + ng;
    // Point ranges into the sorted observation list.
    let mut ranges = vec![(0usize, 0usize); p.points.len()];
    {
        let mut i = 0;
        while i < p.obs.len() {
            let pt = p.obs[i].pt as usize;
            let s = i;
            while i < p.obs.len() && p.obs[i].pt as usize == pt {
                i += 1;
            }
            ranges[pt] = (s, i);
        }
    }

    let (mut cost, initial_rms, _) = robust_cost(p, sigma_px);
    let mut lambda = 1e-4;
    let mut iterations = 0;
    for _ in 0..max_iter {
        iterations += 1;
        // ── Linearise ──
        let lins: Vec<Option<Lin>> = p
            .obs
            .par_iter()
            .map(|o| linearise(p, o, sigma_px))
            .collect();

        // ── Per-point blocks (parallel) ──
        struct PointSys {
            vinv: nalgebra::Matrix3<f64>,
            v: V3,
            y: Vec<(usize, M63)>,
        }
        let point_sys: Vec<Option<PointSys>> = (0..p.points.len())
            .into_par_iter()
            .map(|j| {
                let (s, e) = ranges[j];
                if e - s < 2 {
                    return None;
                }
                let mut vm = M3::zeros();
                let mut v = V3::zeros();
                let mut y: Vec<(usize, M63)> = Vec::with_capacity(e - s + 1);
                for k in s..e {
                    let Some(l) = &lins[k] else { continue };
                    let o = &p.obs[k];
                    vm += l.jp.transpose() * l.jp * l.w;
                    v += l.jp.transpose() * l.e * l.w;
                    let yc: M63 = l.jc.transpose() * l.jp * l.w;
                    y.push((o.cam as usize, yc));
                    if p.refine_intrinsics {
                        let mut yg = M63::zeros();
                        yg.fixed_view_mut::<3, 3>(0, 0)
                            .copy_from(&(l.jg.transpose() * l.jp * l.w));
                        let g = nc + p.group[o.cam as usize];
                        match y.iter_mut().find(|(b, _)| *b == g) {
                            Some((_, m)) => *m += yg,
                            None => y.push((g, yg)),
                        }
                    }
                }
                for d in 0..3 {
                    vm[(d, d)] = vm[(d, d)] * (1.0 + lambda) + 1e-9;
                }
                let vinv = vm.try_inverse()?;
                Some(PointSys { vinv, v, y })
            })
            .collect();

        // ── Reduced system ──
        let mut s = Sparse::new(nb);
        let mut g = vec![V6::zeros(); nb];
        for (k, l) in lins.iter().enumerate() {
            let Some(l) = l else { continue };
            let o = &p.obs[k];
            if point_sys[o.pt as usize].is_none() {
                continue;
            }
            let ci = o.cam as usize;
            s.add(ci, ci, &(l.jc.transpose() * l.jc * l.w));
            g[ci] += l.jc.transpose() * l.e * l.w;
            if p.refine_intrinsics {
                let gi = nc + p.group[ci];
                let mut jg6 = SMatrix::<f64, 2, 6>::zeros();
                jg6.fixed_view_mut::<2, 3>(0, 0).copy_from(&l.jg);
                s.add(gi, gi, &(jg6.transpose() * jg6 * l.w));
                g[gi] += jg6.transpose() * l.e * l.w;
                let cg = l.jc.transpose() * jg6 * l.w;
                s.add(ci, gi, &cg);
                s.add(gi, ci, &cg.transpose());
            }
        }
        for (i, pr) in p.prior.iter().enumerate() {
            if let Some((c, sd)) = pr {
                let d = c - p.poses[i].c;
                let mut m = M6::zeros();
                for a in 0..3 {
                    let w = 1.0 / (sd[a] * sd[a]);
                    m[(3 + a, 3 + a)] = w;
                    g[i][3 + a] += w * d[a];
                }
                s.add(i, i, &m);
            }
        }
        for b in 0..nb {
            let d = s.rows[b].entry(b).or_insert_with(M6::zeros);
            for k in 0..6 {
                d[(k, k)] = d[(k, k)] * (1.0 + lambda) + 1e-9;
            }
            if b >= nc {
                // Padding dims of lens blocks (only 3 parameters are real).
                for k in 3..6 {
                    d[(k, k)] = 1.0;
                }
                if !p.refine_intrinsics {
                    *d = M6::identity();
                }
            }
        }
        for ps in point_sys.iter().flatten() {
            let vy: Vec<SMatrix<f64, 3, 6>> =
                ps.y.iter().map(|(_, y)| ps.vinv * y.transpose()).collect();
            for (a, (ba, ya)) in ps.y.iter().enumerate() {
                g[*ba] -= ya * (ps.vinv * ps.v);
                for (bidx, (bb, _)) in ps.y.iter().enumerate() {
                    let m = ya * vy[bidx];
                    s.add(*ba, *bb, &(-m));
                }
                let _ = a;
            }
        }

        // ── Solve and back-substitute ──
        let dx = pcg(&s, &g, 400);
        let old_poses = p.poses.clone();
        let old_intr = p.intr.clone();
        let old_points = p.points.clone();
        for i in 0..nc {
            let w = V3::new(dx[i][0], dx[i][1], dx[i][2]);
            p.poses[i].r = exp_so3(&w) * p.poses[i].r;
            p.poses[i].c += V3::new(dx[i][3], dx[i][4], dx[i][5]);
        }
        if p.refine_intrinsics {
            for gi in 0..ng {
                let d = dx[nc + gi];
                p.intr[gi].f += d[0];
                p.intr[gi].k1 += d[1];
                p.intr[gi].k2 += d[2];
            }
        }
        let new_pts: Vec<Option<V3>> = point_sys
            .par_iter()
            .map(|ps| {
                let ps = ps.as_ref()?;
                let mut rhs = ps.v;
                for (b, y) in &ps.y {
                    rhs -= y.transpose() * dx[*b];
                }
                Some(ps.vinv * rhs)
            })
            .collect();
        for (j, d) in new_pts.into_iter().enumerate() {
            if let Some(d) = d {
                p.points[j] += d;
            }
        }
        let (new_cost, _, _) = robust_cost(p, sigma_px);
        if new_cost < cost {
            let rel = (cost - new_cost) / cost.max(1e-12);
            cost = new_cost;
            lambda = (lambda / 3.0).max(1e-9);
            if rel < 1e-6 {
                break;
            }
        } else {
            p.poses = old_poses;
            p.intr = old_intr;
            p.points = old_points;
            lambda *= 5.0;
            if lambda > 1e6 {
                break;
            }
        }
    }
    let (_, final_rms, _) = robust_cost(p, sigma_px);
    BaStats {
        iterations,
        initial_rms,
        final_rms,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthetic survey: 3 × 4 nadir cameras over bumpy ground, perturbed
    /// poses, focal length off by 3% — BA must recover sub-pixel reprojection.
    #[test]
    fn recovers_perturbed_survey() {
        let truth_k = Intrinsics {
            f: 3000.0,
            cx: 2000.0,
            cy: 1500.0,
            k1: -0.03,
            k2: 0.0,
            width: 4000,
            height: 3000,
        };
        let mut truth = Vec::new();
        for r in 0..3 {
            for c in 0..4 {
                let yaw = if r % 2 == 0 { 90.0 } else { 270.0 };
                truth.push(Pose::from_angles(
                    V3::new(c as f64 * 25.0, r as f64 * 35.0, 80.0),
                    yaw,
                    -90.0,
                    0.0,
                ));
            }
        }
        let mut s: u64 = 99;
        let mut rnd = || {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((s >> 11) as f64 / (1u64 << 53) as f64) - 0.5
        };
        let mut points = Vec::new();
        let mut obs = Vec::new();
        for _ in 0..1500 {
            let x = V3::new(rnd() * 120.0 + 37.0, rnd() * 110.0 + 35.0, rnd() * 8.0);
            let views: Vec<(usize, Vector2<f64>)> = truth
                .iter()
                .enumerate()
                .filter_map(|(i, ps)| {
                    let u = ps.project(&truth_k, &x)?;
                    (u.x > 0.0 && u.y > 0.0 && u.x < 4000.0 && u.y < 3000.0).then_some((i, u))
                })
                .collect();
            if views.len() < 2 {
                continue;
            }
            let pt = points.len() as u32;
            points.push(x + V3::new(rnd(), rnd(), rnd()) * 2.0);
            for (i, u) in views {
                obs.push(Obs {
                    cam: i as u32,
                    pt,
                    u: u + Vector2::new(rnd(), rnd()) * 0.5,
                });
            }
        }
        let poses: Vec<Pose> = truth
            .iter()
            .map(|t| Pose {
                r: exp_so3(&(V3::new(rnd(), rnd(), rnd()) * 0.02)) * t.r,
                c: t.c + V3::new(rnd(), rnd(), rnd()) * 3.0,
            })
            .collect();
        let prior = truth
            .iter()
            .map(|t| {
                Some((
                    t.c + V3::new(rnd(), rnd(), rnd()) * 1.0,
                    V3::new(2.0, 2.0, 3.0),
                ))
            })
            .collect();
        let mut k0 = truth_k;
        k0.f *= 1.03;
        k0.k1 = 0.0;
        let mut p = Problem {
            intr: vec![k0],
            poses,
            group: vec![0; truth.len()],
            prior,
            points,
            obs,
            refine_intrinsics: true,
        };
        let st = bundle_adjust(&mut p, 60, 4.0);
        assert!(st.initial_rms > 20.0, "{st:?}");
        assert!(st.final_rms < 0.8, "{st:?}");
        // Nadir photos at one height can't separate focal length from flying
        // height (both scale the image alike), so f is only pinned as well as
        // the altitude prior allows. Angled photos break that tie.
        assert!(
            (p.intr[0].f / truth_k.f - 1.0).abs() < 0.025,
            "f = {}",
            p.intr[0].f
        );
    }
}
