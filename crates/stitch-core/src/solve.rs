//! Global placement: one sparse linear least-squares problem.
//!
//! Each photo `i` has a similarity `Tᵢ` from its *centred, y-up* working
//! coordinates into the output frame (UTM metres when GPS exists). Unknowns
//! are `(aᵢ, bᵢ, txᵢ, tyᵢ)`, and every constraint is linear in them:
//!
//! * a match:   `Tᵢ(pᵢ) − Tⱼ(pⱼ) = 0`
//! * GPS:       `Tᵢ(0,0) = (E, N)`   (the photo centre is the nadir point)
//! * stability: `(aᵢ, bᵢ) ≈ (a₀, b₀)` very weakly, so a photo with no
//!   matches still has a defined rotation and scale.
//!
//! Solved with Jacobi-preconditioned conjugate gradients on the normal
//! equations, without ever forming the matrix: memory is O(constraints).

use crate::sim::Sim2;

#[derive(Clone, Copy)]
struct Entry {
    col: u32,
    val: f64,
}

struct Row {
    entries: [Entry; 8],
    len: u8,
    rhs: f64,
}

pub struct LinearSystem {
    n: usize,
    rows: Vec<Row>,
}

impl LinearSystem {
    pub fn new(images: usize) -> Self {
        Self { n: images * 4, rows: Vec::new() }
    }

    fn push(&mut self, terms: &[(usize, f64)], rhs: f64, w: f64) {
        let mut entries = [Entry { col: 0, val: 0.0 }; 8];
        for (k, &(c, v)) in terms.iter().enumerate() {
            entries[k] = Entry { col: c as u32, val: v * w };
        }
        self.rows.push(Row { entries, len: terms.len() as u8, rhs: rhs * w });
    }

    /// Point `(u, v)` in photo `i` and `(s, t)` in photo `j` are the same ground point.
    pub fn add_match(&mut self, i: usize, (u, v): (f64, f64), j: usize, (s, t): (f64, f64), w: f64) {
        let (bi, bj) = (i * 4, j * 4);
        // x: a·u − b·v + tx
        self.push(&[(bi, u), (bi + 1, -v), (bi + 2, 1.0), (bj, -s), (bj + 1, t), (bj + 2, -1.0)], 0.0, w);
        // y: b·u + a·v + ty
        self.push(&[(bi, v), (bi + 1, u), (bi + 3, 1.0), (bj, -t), (bj + 1, -s), (bj + 3, -1.0)], 0.0, w);
    }

    /// Photo `i`'s centre lands at `(x, y)`.
    pub fn add_position(&mut self, i: usize, x: f64, y: f64, w: f64) {
        self.push(&[(i * 4 + 2, 1.0)], x, w);
        self.push(&[(i * 4 + 3, 1.0)], y, w);
    }

    /// Weak pull of photo `i`'s rotation/scale towards `(a, b)`.
    pub fn add_rotation_prior(&mut self, i: usize, a: f64, b: f64, w: f64) {
        self.push(&[(i * 4, 1.0)], a, w);
        self.push(&[(i * 4 + 1, 1.0)], b, w);
    }

    pub fn solve(&self, x0: &[Sim2], max_iter: usize) -> Vec<Sim2> {
        let n = self.n;
        let mut x: Vec<f64> = x0.iter().flat_map(|s| [s.a, s.b, s.tx, s.ty]).collect();
        x.resize(n, 0.0);

        let at_times = |r: &[f64], out: &mut [f64]| {
            out.iter_mut().for_each(|v| *v = 0.0);
            for (row, rv) in self.rows.iter().zip(r) {
                for e in &row.entries[..row.len as usize] {
                    out[e.col as usize] += e.val * rv;
                }
            }
        };
        let a_times = |v: &[f64], out: &mut Vec<f64>| {
            out.clear();
            for row in &self.rows {
                let mut s = 0.0;
                for e in &row.entries[..row.len as usize] {
                    s += e.val * v[e.col as usize];
                }
                out.push(s);
            }
        };

        let mut diag = vec![0.0; n];
        for row in &self.rows {
            for e in &row.entries[..row.len as usize] {
                diag[e.col as usize] += e.val * e.val;
            }
        }
        let inv_d: Vec<f64> = diag.iter().map(|&d| if d > 0.0 { 1.0 / d } else { 0.0 }).collect();

        // r = Aᵀ(b − Ax)
        let mut ax = Vec::with_capacity(self.rows.len());
        a_times(&x, &mut ax);
        let resid: Vec<f64> = self.rows.iter().zip(&ax).map(|(row, v)| row.rhs - v).collect();
        let mut r = vec![0.0; n];
        at_times(&resid, &mut r);
        let mut z: Vec<f64> = r.iter().zip(&inv_d).map(|(a, b)| a * b).collect();
        let mut p = z.clone();
        let mut rz: f64 = r.iter().zip(&z).map(|(a, b)| a * b).sum();
        let r0 = rz.abs().max(1e-300);
        let mut ap = Vec::with_capacity(self.rows.len());
        let mut q = vec![0.0; n];
        for _ in 0..max_iter {
            if rz.abs() < r0 * 1e-24 {
                break;
            }
            a_times(&p, &mut ap);
            at_times(&ap, &mut q);
            let pq: f64 = p.iter().zip(&q).map(|(a, b)| a * b).sum();
            if pq.abs() < 1e-300 {
                break;
            }
            let alpha = rz / pq;
            for k in 0..n {
                x[k] += alpha * p[k];
                r[k] -= alpha * q[k];
            }
            for k in 0..n {
                z[k] = r[k] * inv_d[k];
            }
            let rz_new: f64 = r.iter().zip(&z).map(|(a, b)| a * b).sum();
            let beta = rz_new / rz;
            rz = rz_new;
            for k in 0..n {
                p[k] = z[k] + beta * p[k];
            }
        }
        x.chunks_exact(4).map(|c| Sim2 { a: c[0], b: c[1], tx: c[2], ty: c[3] }).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_three_photo_strip() {
        // Ground truth: photos 100 units apart, the middle one rotated 90°.
        let truth = [
            Sim2 { a: 0.1, b: 0.0, tx: 0.0, ty: 0.0 },
            Sim2 { a: 0.0, b: 0.1, tx: 50.0, ty: 3.0 },
            Sim2 { a: 0.1, b: 0.0, tx: 100.0, ty: 0.0 },
        ];
        let mut sys = LinearSystem::new(3);
        for (i, j) in [(0, 1), (1, 2)] {
            for k in 0..20 {
                let g = ((k as f64) * 13.0 % 40.0 + 10.0 + 50.0 * i as f64, (k as f64 * 7.0) % 30.0 - 15.0);
                let pi = truth[i].inverse().apply(g.0, g.1);
                let pj = truth[j].inverse().apply(g.0, g.1);
                sys.add_match(i, pi, j, pj, 1.0);
            }
        }
        // GPS with ±2 noise on the two ends only.
        sys.add_position(0, 1.5, -2.0, 0.01);
        sys.add_position(2, 98.5, 2.0, 0.01);
        for i in 0..3 {
            sys.add_rotation_prior(i, 0.1, 0.0, 1e-6);
        }
        let got = sys.solve(&[Sim2::IDENTITY; 3], 5000);
        let rel = got[0].inverse().compose(&got[1]);
        let rel_t = truth[0].inverse().compose(&truth[1]);
        assert!((rel.a - rel_t.a).abs() < 1e-3 && (rel.b - rel_t.b).abs() < 1e-3, "{rel:?} vs {rel_t:?}");
    }
}
