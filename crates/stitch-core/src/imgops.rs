//! Small, allocation-conscious raster helpers. Kept in-house rather than
//! pulling a resize library so the core stays pure Rust and cross-compiles
//! to Windows ARM64 and Apple Silicon without native dependencies.

use anyhow::{Context, Result};
use std::path::Path;

#[derive(Clone)]
pub struct Gray {
    pub w: usize,
    pub h: usize,
    pub data: Vec<u8>,
}

#[derive(Clone)]
pub struct Rgb {
    pub w: usize,
    pub h: usize,
    pub data: Vec<u8>,
}

impl Gray {
    #[inline]
    pub fn at(&self, x: usize, y: usize) -> u8 {
        self.data[y * self.w + x]
    }
}

impl Rgb {
    pub fn bytes(&self) -> usize {
        self.data.len()
    }

    pub fn to_gray(&self) -> Gray {
        let mut data = Vec::with_capacity(self.w * self.h);
        for p in self.data.chunks_exact(3) {
            data.push(((p[0] as u32 * 77 + p[1] as u32 * 150 + p[2] as u32 * 29) >> 8) as u8);
        }
        Gray {
            w: self.w,
            h: self.h,
            data,
        }
    }

    /// Bilinear sample at continuous pixel coordinates (pixel centres at +0.5).
    /// Returns None outside the image.
    #[inline]
    pub fn sample(&self, x: f64, y: f64) -> Option<[f32; 3]> {
        let fx = x - 0.5;
        let fy = y - 0.5;
        if fx < 0.0 || fy < 0.0 {
            return None;
        }
        let x0 = fx as usize;
        let y0 = fy as usize;
        if x0 + 1 >= self.w || y0 + 1 >= self.h {
            return None;
        }
        let ax = (fx - x0 as f64) as f32;
        let ay = (fy - y0 as f64) as f32;
        let i00 = (y0 * self.w + x0) * 3;
        let i10 = i00 + 3;
        let i01 = i00 + self.w * 3;
        let i11 = i01 + 3;
        let d = &self.data;
        let mut out = [0f32; 3];
        for c in 0..3 {
            let top = d[i00 + c] as f32 * (1.0 - ax) + d[i10 + c] as f32 * ax;
            let bot = d[i01 + c] as f32 * (1.0 - ax) + d[i11 + c] as f32 * ax;
            out[c] = top * (1.0 - ay) + bot * ay;
        }
        Some(out)
    }
}

pub fn load_rgb(path: &Path) -> Result<Rgb> {
    let img = image::open(path).with_context(|| format!("decoding {}", path.display()))?;
    let rgb = img.into_rgb8();
    let (w, h) = rgb.dimensions();
    Ok(Rgb {
        w: w as usize,
        h: h as usize,
        data: rgb.into_raw(),
    })
}

/// Integer box downsample by `f` (area average, exact for photos).
pub fn box_down(src: &Rgb, f: usize) -> Rgb {
    if f <= 1 {
        return src.clone();
    }
    let w = src.w / f;
    let h = src.h / f;
    let mut data = vec![0u8; w * h * 3];
    let area = (f * f) as u32;
    let mut acc = vec![0u32; w * 3];
    for oy in 0..h {
        acc.iter_mut().for_each(|v| *v = 0);
        for sy in oy * f..oy * f + f {
            let row = &src.data[sy * src.w * 3..(sy * src.w + w * f) * 3];
            for ox in 0..w {
                let base = ox * f * 3;
                let a = &mut acc[ox * 3..ox * 3 + 3];
                for k in 0..f {
                    let p = base + k * 3;
                    a[0] += row[p] as u32;
                    a[1] += row[p + 1] as u32;
                    a[2] += row[p + 2] as u32;
                }
            }
        }
        let out = &mut data[oy * w * 3..(oy + 1) * w * 3];
        for (o, a) in out.iter_mut().zip(acc.iter()) {
            *o = ((a + area / 2) / area) as u8;
        }
    }
    Rgb { w, h, data }
}

/// Bilinear resize to an exact size; used only for the final step after a
/// box downsample has brought the image within 2x of the target.
pub fn resize_bilinear(src: &Rgb, w: usize, h: usize) -> Rgb {
    let mut data = vec![0u8; w * h * 3];
    let sx = src.w as f64 / w as f64;
    let sy = src.h as f64 / h as f64;
    for y in 0..h {
        let fy = ((y as f64 + 0.5) * sy - 0.5).clamp(0.0, (src.h - 1) as f64);
        let y0 = fy as usize;
        let y1 = (y0 + 1).min(src.h - 1);
        let ay = fy - y0 as f64;
        for x in 0..w {
            let fx = ((x as f64 + 0.5) * sx - 0.5).clamp(0.0, (src.w - 1) as f64);
            let x0 = fx as usize;
            let x1 = (x0 + 1).min(src.w - 1);
            let ax = fx - x0 as f64;
            for c in 0..3 {
                let p = |xx: usize, yy: usize| src.data[(yy * src.w + xx) * 3 + c] as f64;
                let v = (p(x0, y0) * (1.0 - ax) + p(x1, y0) * ax) * (1.0 - ay)
                    + (p(x0, y1) * (1.0 - ax) + p(x1, y1) * ax) * ay;
                data[(y * w + x) * 3 + c] = v.round() as u8;
            }
        }
    }
    Rgb { w, h, data }
}

/// Shrinks so the long side is at most `max_side`. Returns the image and the
/// scale factor applied (new / old).
pub fn shrink_to(src: Rgb, max_side: usize) -> (Rgb, f64) {
    let long = src.w.max(src.h);
    if long <= max_side {
        return (src, 1.0);
    }
    let f = long / max_side;
    let boxed = if f >= 2 {
        box_down(&src, f)
    } else {
        src.clone()
    };
    drop(src);
    let s = max_side as f64 / long as f64;
    let w = ((boxed.w as f64) * (max_side as f64 / boxed.w.max(boxed.h) as f64)).round() as usize;
    let h = ((boxed.h as f64) * (max_side as f64 / boxed.w.max(boxed.h) as f64)).round() as usize;
    let out = if w == boxed.w && h == boxed.h {
        boxed
    } else {
        resize_bilinear(&boxed, w.max(1), h.max(1))
    };
    (out, s)
}

/// Separable box blur of radius `r` (used to stabilise binary descriptors).
pub fn box_blur(g: &Gray, r: usize) -> Gray {
    let (w, h) = (g.w, g.h);
    let n = (2 * r + 1) as u32;
    let mut tmp = vec![0u8; w * h];
    for y in 0..h {
        let row = &g.data[y * w..(y + 1) * w];
        let mut sum: u32 = 0;
        for k in 0..=2 * r {
            sum += row[k.saturating_sub(r).min(w - 1)] as u32;
        }
        for x in 0..w {
            tmp[y * w + x] = (sum / n) as u8;
            let add = (x + r + 1).min(w - 1);
            let sub = x.saturating_sub(r);
            sum = sum + row[add] as u32 - row[sub] as u32;
        }
    }
    let mut out = vec![0u8; w * h];
    for x in 0..w {
        let mut sum: u32 = 0;
        for k in 0..=2 * r {
            sum += tmp[k.saturating_sub(r).min(h - 1) * w + x] as u32;
        }
        for y in 0..h {
            out[y * w + x] = (sum / n) as u8;
            let add = (y + r + 1).min(h - 1);
            let sub = y.saturating_sub(r);
            sum = sum + tmp[add * w + x] as u32 - tmp[sub * w + x] as u32;
        }
    }
    Gray { w, h, data: out }
}
