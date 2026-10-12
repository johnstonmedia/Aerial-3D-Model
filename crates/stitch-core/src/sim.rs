//! 2D similarity transforms (rotation + uniform scale + translation).
//!
//! Written as complex arithmetic: `p' = c·p + t` with `c = a + ib`. That form
//! is linear in `(a, b, tx, ty)`, which is what lets the global solve be a
//! single linear least-squares problem.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Sim2 {
    pub a: f64,
    pub b: f64,
    pub tx: f64,
    pub ty: f64,
}

impl Default for Sim2 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Sim2 {
    pub const IDENTITY: Sim2 = Sim2 { a: 1.0, b: 0.0, tx: 0.0, ty: 0.0 };

    #[inline]
    pub fn apply(&self, x: f64, y: f64) -> (f64, f64) {
        (self.a * x - self.b * y + self.tx, self.b * x + self.a * y + self.ty)
    }

    pub fn scale(&self) -> f64 {
        (self.a * self.a + self.b * self.b).sqrt()
    }

    pub fn angle(&self) -> f64 {
        self.b.atan2(self.a)
    }

    pub fn inverse(&self) -> Sim2 {
        let d = self.a * self.a + self.b * self.b;
        let ia = self.a / d;
        let ib = -self.b / d;
        // t' = -c⁻¹·t
        let tx = -(ia * self.tx - ib * self.ty);
        let ty = -(ib * self.tx + ia * self.ty);
        Sim2 { a: ia, b: ib, tx, ty }
    }

    /// `self ∘ other`: applies `other` first.
    pub fn compose(&self, other: &Sim2) -> Sim2 {
        let a = self.a * other.a - self.b * other.b;
        let b = self.a * other.b + self.b * other.a;
        let (tx, ty) = self.apply(other.tx, other.ty);
        Sim2 { a, b, tx, ty }
    }

    /// Least-squares similarity mapping `src[i]` onto `dst[i]`.
    pub fn fit(src: &[(f64, f64)], dst: &[(f64, f64)]) -> Option<Sim2> {
        let n = src.len();
        if n < 2 || n != dst.len() {
            return None;
        }
        let (mut sx, mut sy, mut dx, mut dy) = (0.0, 0.0, 0.0, 0.0);
        for i in 0..n {
            sx += src[i].0;
            sy += src[i].1;
            dx += dst[i].0;
            dy += dst[i].1;
        }
        let nf = n as f64;
        let (sx, sy, dx, dy) = (sx / nf, sy / nf, dx / nf, dy / nf);
        let (mut num_a, mut num_b, mut den) = (0.0, 0.0, 0.0);
        for i in 0..n {
            let (px, py) = (src[i].0 - sx, src[i].1 - sy);
            let (qx, qy) = (dst[i].0 - dx, dst[i].1 - dy);
            num_a += px * qx + py * qy;
            num_b += px * qy - py * qx;
            den += px * px + py * py;
        }
        if den < 1e-12 {
            return None;
        }
        let a = num_a / den;
        let b = num_b / den;
        let tx = dx - (a * sx - b * sy);
        let ty = dy - (b * sx + a * sy);
        Some(Sim2 { a, b, tx, ty })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_recovers_transform_and_inverse_round_trips() {
        let t = Sim2 { a: 0.8, b: 0.3, tx: 12.0, ty: -4.0 };
        let src = [(0.0, 0.0), (10.0, 3.0), (-5.0, 7.0), (2.0, -9.0)];
        let dst: Vec<_> = src.iter().map(|&(x, y)| t.apply(x, y)).collect();
        let f = Sim2::fit(&src, &dst).unwrap();
        assert!((f.a - t.a).abs() < 1e-9 && (f.ty - t.ty).abs() < 1e-9);
        let (x, y) = t.inverse().apply(dst[2].0, dst[2].1);
        assert!((x + 5.0).abs() < 1e-9 && (y - 7.0).abs() < 1e-9);
        let c = t.compose(&t.inverse());
        assert!((c.a - 1.0).abs() < 1e-9 && c.tx.abs() < 1e-9);
    }
}
