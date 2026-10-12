//! Camera model and multi-view geometry for 3D reconstruction.
//!
//! World frame: local east/north/up metres (ENU). Camera frame: OpenCV
//! convention — x right, y down, z forward. A pose stores `R` (world→camera)
//! and the camera centre `C`; a world point X lands at `R·(X − C)`.

use nalgebra::{DMatrix, Matrix3, SMatrix, Vector2, Vector3};
use serde::{Deserialize, Serialize};

pub type V3 = Vector3<f64>;
pub type M3 = Matrix3<f64>;

/// Pinhole camera with two radial distortion terms, in full-resolution pixels.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Intrinsics {
    pub f: f64,
    pub cx: f64,
    pub cy: f64,
    pub k1: f64,
    pub k2: f64,
    pub width: u32,
    pub height: u32,
}

impl Intrinsics {
    /// Normalised image coordinates → distorted pixel.
    #[inline]
    pub fn distort(&self, p: Vector2<f64>) -> Vector2<f64> {
        let r2 = p.norm_squared();
        let d = 1.0 + self.k1 * r2 + self.k2 * r2 * r2;
        Vector2::new(self.f * d * p.x + self.cx, self.f * d * p.y + self.cy)
    }

    /// Pixel → normalised coordinates, inverting distortion by fixed point.
    pub fn undistort(&self, u: Vector2<f64>) -> Vector2<f64> {
        let q = Vector2::new((u.x - self.cx) / self.f, (u.y - self.cy) / self.f);
        let mut p = q;
        for _ in 0..8 {
            let r2 = p.norm_squared();
            let d = 1.0 + self.k1 * r2 + self.k2 * r2 * r2;
            p = q / d;
        }
        p
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Pose {
    /// World → camera rotation (row-major when serialised).
    pub r: M3,
    /// Camera centre in world coordinates.
    pub c: V3,
}

impl Pose {
    #[inline]
    pub fn to_cam(&self, x: &V3) -> V3 {
        self.r * (x - self.c)
    }

    /// Viewing direction (camera +z) in world coordinates.
    pub fn axis(&self) -> V3 {
        self.r.row(2).transpose()
    }

    pub fn project(&self, k: &Intrinsics, x: &V3) -> Option<Vector2<f64>> {
        let xc = self.to_cam(x);
        if xc.z <= 1e-6 {
            return None;
        }
        Some(k.distort(Vector2::new(xc.x / xc.z, xc.y / xc.z)))
    }

    /// Pose of a camera at `c` facing `yaw` (degrees clockwise from north)
    /// tilted by `pitch` (−90 straight down, 0 level) with `roll` (degrees).
    pub fn from_angles(c: V3, yaw_deg: f64, pitch_deg: f64, roll_deg: f64) -> Pose {
        let (sy, cy) = yaw_deg.to_radians().sin_cos();
        let (sp, cp) = pitch_deg.to_radians().sin_cos();
        let heading = V3::new(sy, cy, 0.0);
        let up = V3::new(0.0, 0.0, 1.0);
        let x = V3::new(cy, -sy, 0.0); // right
        let z = heading * cp + up * sp; // forward
        let y = z.cross(&x); // down (for a level camera)
        let mut r_wc = M3::from_columns(&[x, y, z]);
        if roll_deg.abs() > 1e-9 {
            r_wc *= rot_axis(&V3::new(0.0, 0.0, 1.0), roll_deg.to_radians());
        }
        Pose { r: r_wc.transpose(), c }
    }
}

pub fn skew(v: &V3) -> M3 {
    M3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Rotation by `angle` (radians) about unit `axis`.
pub fn rot_axis(axis: &V3, angle: f64) -> M3 {
    let k = skew(axis);
    M3::identity() + k * angle.sin() + k * k * (1.0 - angle.cos())
}

/// Rodrigues: rotation vector → matrix.
pub fn exp_so3(w: &V3) -> M3 {
    let th = w.norm();
    if th < 1e-12 {
        return M3::identity() + skew(w);
    }
    rot_axis(&(w / th), th)
}

/// Linear multi-view triangulation from normalised coordinates.
pub fn triangulate(obs: &[(Pose, Vector2<f64>)]) -> Option<V3> {
    if obs.len() < 2 {
        return None;
    }
    let mut a = DMatrix::<f64>::zeros(obs.len() * 2, 4);
    for (i, (pose, p)) in obs.iter().enumerate() {
        let t = -(pose.r * pose.c);
        let row = |k: usize| [pose.r[(k, 0)], pose.r[(k, 1)], pose.r[(k, 2)], t[k]];
        let (p0, p1, p2) = (row(0), row(1), row(2));
        for c in 0..4 {
            a[(2 * i, c)] = p.x * p2[c] - p0[c];
            a[(2 * i + 1, c)] = p.y * p2[c] - p1[c];
        }
    }
    // Smallest right singular vector via the 4×4 normal matrix.
    let ata = a.transpose() * &a;
    let eig = ata.symmetric_eigen();
    let (mut best, mut bv) = (0, f64::MAX);
    for i in 0..4 {
        if eig.eigenvalues[i] < bv {
            bv = eig.eigenvalues[i];
            best = i;
        }
    }
    let v = eig.eigenvectors.column(best);
    if v[3].abs() < 1e-12 {
        return None;
    }
    Some(V3::new(v[0] / v[3], v[1] / v[3], v[2] / v[3]))
}

/// Largest angle (degrees) between viewing rays to `x` from any two cameras.
pub fn max_ray_angle(x: &V3, centres: &[V3]) -> f64 {
    let rays: Vec<V3> = centres.iter().map(|c| (x - c).normalize()).collect();
    let mut best: f64 = 0.0;
    for i in 0..rays.len() {
        for j in i + 1..rays.len() {
            best = best.max(rays[i].dot(&rays[j]).clamp(-1.0, 1.0).acos());
        }
    }
    best.to_degrees()
}

// ───────────────────────── Fundamental matrix ─────────────────────────

type M9 = SMatrix<f64, 9, 9>;

fn normalise(pts: &[(f64, f64)]) -> (Vec<(f64, f64)>, M3) {
    let n = pts.len() as f64;
    let (mx, my) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0 / n, a.1 + p.1 / n));
    let d = pts.iter().map(|p| ((p.0 - mx).powi(2) + (p.1 - my).powi(2)).sqrt()).sum::<f64>() / n;
    let s = if d > 1e-12 { std::f64::consts::SQRT_2 / d } else { 1.0 };
    let t = M3::new(s, 0.0, -s * mx, 0.0, s, -s * my, 0.0, 0.0, 1.0);
    (pts.iter().map(|p| ((p.0 - mx) * s, (p.1 - my) * s)).collect(), t)
}

/// Normalised 8-point fundamental matrix (x2ᵀ F x1 = 0), rank 2 enforced.
pub fn fundamental(a: &[(f64, f64)], b: &[(f64, f64)]) -> Option<M3> {
    if a.len() < 8 {
        return None;
    }
    let (na, ta) = normalise(a);
    let (nb, tb) = normalise(b);
    let mut m = M9::zeros();
    for (p, q) in na.iter().zip(&nb) {
        let row = [q.0 * p.0, q.0 * p.1, q.0, q.1 * p.0, q.1 * p.1, q.1, p.0, p.1, 1.0];
        for i in 0..9 {
            for j in 0..9 {
                m[(i, j)] += row[i] * row[j];
            }
        }
    }
    let eig = m.symmetric_eigen();
    let mut k = 0;
    for i in 1..9 {
        if eig.eigenvalues[i] < eig.eigenvalues[k] {
            k = i;
        }
    }
    let f = eig.eigenvectors.column(k);
    let fm = M3::new(f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8]);
    let svd = fm.svd(true, true);
    let (u, vt) = (svd.u?, svd.v_t?);
    let mut s = svd.singular_values;
    s[2] = 0.0;
    let f2 = u * M3::from_diagonal(&s) * vt;
    let full = tb.transpose() * f2 * ta;
    let norm = full.norm();
    (norm > 1e-15).then(|| full / norm)
}

/// Squared Sampson distance of a correspondence to F.
#[inline]
pub fn sampson(f: &M3, a: (f64, f64), b: (f64, f64)) -> f64 {
    let x1 = V3::new(a.0, a.1, 1.0);
    let x2 = V3::new(b.0, b.1, 1.0);
    let fx1 = f * x1;
    let ftx2 = f.transpose() * x2;
    let e = x2.dot(&fx1);
    let d = fx1.x * fx1.x + fx1.y * fx1.y + ftx2.x * ftx2.x + ftx2.y * ftx2.y;
    if d < 1e-18 { f64::MAX } else { e * e / d }
}

/// RANSAC over the 8-point solver. Returns inlier indices.
pub fn ransac_fundamental(a: &[(f64, f64)], b: &[(f64, f64)], thresh_px: f64, iters: usize, seed: u64) -> Vec<usize> {
    let n = a.len();
    if n < 12 {
        return Vec::new();
    }
    let t2 = thresh_px * thresh_px;
    let mut s = seed | 1;
    let mut rnd = |m: usize| {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s % m as u64) as usize
    };
    let mut best: Vec<usize> = Vec::new();
    let mut idx = [0usize; 8];
    for _ in 0..iters {
        let mut k = 0;
        while k < 8 {
            let c = rnd(n);
            if !idx[..k].contains(&c) {
                idx[k] = c;
                k += 1;
            }
        }
        let sa: Vec<_> = idx.iter().map(|&i| a[i]).collect();
        let sb: Vec<_> = idx.iter().map(|&i| b[i]).collect();
        let Some(f) = fundamental(&sa, &sb) else { continue };
        let inl: Vec<usize> = (0..n).filter(|&i| sampson(&f, a[i], b[i]) < t2).collect();
        if inl.len() > best.len() {
            best = inl;
        }
    }
    // Refit on all inliers once and re-select.
    if best.len() >= 8 {
        let sa: Vec<_> = best.iter().map(|&i| a[i]).collect();
        let sb: Vec<_> = best.iter().map(|&i| b[i]).collect();
        if let Some(f) = fundamental(&sa, &sb) {
            let refit: Vec<usize> = (0..n).filter(|&i| sampson(&f, a[i], b[i]) < t2).collect();
            if refit.len() >= best.len() {
                best = refit;
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    fn k() -> Intrinsics {
        Intrinsics { f: 3000.0, cx: 2000.0, cy: 1500.0, k1: -0.02, k2: 0.005, width: 4000, height: 3000 }
    }

    #[test]
    fn nadir_camera_looks_down_with_image_up_along_heading() {
        // Heading east: image "up" (−y) should point east, image right south.
        let p = Pose::from_angles(V3::new(0.0, 0.0, 100.0), 90.0, -90.0, 0.0);
        assert!((p.axis() - V3::new(0.0, 0.0, -1.0)).norm() < 1e-9);
        let r_wc = p.r.transpose();
        assert!((-r_wc.column(1) - V3::new(1.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((r_wc.column(0) - V3::new(0.0, -1.0, 0.0)).norm() < 1e-9);
        // A point east of the drone appears above centre.
        let u = p.project(&k(), &V3::new(10.0, 0.0, 0.0)).unwrap();
        assert!(u.y < 1500.0 && (u.x - 2000.0).abs() < 1e-6);
        // 45° oblique looking north points down-and-north.
        let o = Pose::from_angles(V3::zeros(), 0.0, -45.0, 0.0);
        assert!((o.axis() - V3::new(0.0, 0.5f64.sqrt(), -(0.5f64.sqrt()))).norm() < 1e-9);
    }

    #[test]
    fn distortion_round_trips_and_triangulation_recovers_point() {
        let k = k();
        let p = Vector2::new(0.31, -0.22);
        assert!((k.undistort(k.distort(p)) - p).norm() < 1e-9);
        let x = V3::new(3.0, -4.0, 1.5);
        let poses = [
            Pose::from_angles(V3::new(0.0, 0.0, 60.0), 10.0, -90.0, 0.0),
            Pose::from_angles(V3::new(20.0, 5.0, 61.0), 190.0, -80.0, 0.0),
            Pose::from_angles(V3::new(-15.0, 30.0, 59.0), 45.0, -50.0, 0.0),
        ];
        let obs: Vec<_> = poses.iter().map(|ps| (*ps, k.undistort(ps.project(&k, &x).unwrap()))).collect();
        let t = triangulate(&obs).unwrap();
        assert!((t - x).norm() < 1e-6, "{t:?}");
    }

    #[test]
    fn fundamental_ransac_separates_inliers() {
        let k = k();
        let a = Pose::from_angles(V3::new(0.0, 0.0, 80.0), 0.0, -90.0, 0.0);
        let b = Pose::from_angles(V3::new(25.0, 3.0, 81.0), 5.0, -88.0, 0.0);
        let mut pa = Vec::new();
        let mut pb = Vec::new();
        let mut s: u64 = 7;
        let mut r = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((s >> 33) as f64 / (1u64 << 31) as f64) - 0.5
        };
        for i in 0..200 {
            let x = V3::new(r() * 60.0 + 12.0, r() * 50.0, r() * 15.0);
            let (Some(ua), Some(ub)) = (a.project(&k, &x), b.project(&k, &x)) else { continue };
            pa.push((ua.x, ua.y));
            pb.push(if i % 4 == 0 { (r() * 4000.0 + 2000.0, r() * 3000.0 + 1500.0) } else { (ub.x, ub.y) });
        }
        let inl = ransac_fundamental(&pa, &pb, 2.0, 600, 3);
        let true_inl = (0..pa.len()).filter(|i| i % 4 != 0).count();
        assert!(inl.len() >= true_inl * 9 / 10 && inl.len() <= true_inl + 3, "{} vs {}", inl.len(), true_inl);
    }
}
