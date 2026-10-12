//! WGS84 ⇄ UTM (Krüger series, sub-millimetre within a zone).
//!
//! The mosaic is solved and rendered directly in UTM metres, north up, so the
//! output needs no reprojection to be a valid GeoTIFF.

use serde::{Deserialize, Serialize};

const A: f64 = 6_378_137.0;
const F: f64 = 1.0 / 298.257_223_563;
const K0: f64 = 0.9996;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct UtmZone {
    pub zone: u8,
    pub north: bool,
}

impl UtmZone {
    pub fn for_lonlat(lon: f64, lat: f64) -> Self {
        let zone = (((lon + 180.0) / 6.0).floor() as i32).clamp(0, 59) as u8 + 1;
        UtmZone {
            zone,
            north: lat >= 0.0,
        }
    }

    pub fn epsg(&self) -> u32 {
        if self.north {
            32600 + self.zone as u32
        } else {
            32700 + self.zone as u32
        }
    }

    fn central_meridian(&self) -> f64 {
        (self.zone as f64 - 1.0) * 6.0 - 180.0 + 3.0
    }
}

struct Series {
    n: f64,
    big_a: f64,
    alpha: [f64; 3],
    beta: [f64; 3],
}

fn series() -> Series {
    let n = F / (2.0 - F);
    let (n2, n3) = (n * n, n * n * n);
    Series {
        n,
        big_a: A / (1.0 + n) * (1.0 + n2 / 4.0 + n2 * n2 / 64.0),
        alpha: [
            n / 2.0 - 2.0 / 3.0 * n2 + 5.0 / 16.0 * n3,
            13.0 / 48.0 * n2 - 3.0 / 5.0 * n3,
            61.0 / 240.0 * n3,
        ],
        beta: [
            n / 2.0 - 2.0 / 3.0 * n2 + 37.0 / 96.0 * n3,
            1.0 / 48.0 * n2 + 1.0 / 15.0 * n3,
            17.0 / 480.0 * n3,
        ],
    }
}

/// Returns (easting, northing) in metres.
pub fn to_utm(lon: f64, lat: f64, zone: UtmZone) -> (f64, f64) {
    let s = series();
    let phi = lat.to_radians();
    let lam = (lon - zone.central_meridian()).to_radians();
    let e2n = 2.0 * s.n.sqrt() / (1.0 + s.n);
    let t = (phi.sin().atanh() - e2n * (e2n * phi.sin()).atanh()).sinh();
    let xi = t.atan2(lam.cos());
    let eta = (lam.sin() / (1.0 + t * t).sqrt()).atanh();
    let mut e = eta;
    let mut nn = xi;
    for (j, a) in s.alpha.iter().enumerate() {
        let k = 2.0 * (j as f64 + 1.0);
        e += a * (k * xi).cos() * (k * eta).sinh();
        nn += a * (k * xi).sin() * (k * eta).cosh();
    }
    let easting = 500_000.0 + K0 * s.big_a * e;
    let mut northing = K0 * s.big_a * nn;
    if !zone.north {
        northing += 10_000_000.0;
    }
    (easting, northing)
}

/// Returns (lon, lat) in degrees.
pub fn from_utm(easting: f64, northing: f64, zone: UtmZone) -> (f64, f64) {
    let s = series();
    let n_off = if zone.north { 0.0 } else { 10_000_000.0 };
    let xi = (northing - n_off) / (K0 * s.big_a);
    let eta = (easting - 500_000.0) / (K0 * s.big_a);
    let mut xp = xi;
    let mut ep = eta;
    for (j, b) in s.beta.iter().enumerate() {
        let k = 2.0 * (j as f64 + 1.0);
        xp -= b * (k * xi).sin() * (k * eta).cosh();
        ep -= b * (k * xi).cos() * (k * eta).sinh();
    }

    // Conformal latitude → geodetic latitude (Karney's Newton iteration on tan φ).
    let tau_p = xp.sin() / (ep.sinh().powi(2) + xp.cos().powi(2)).sqrt();
    let e2 = F * (2.0 - F);
    let e = e2.sqrt();
    let mut tau = tau_p;
    for _ in 0..6 {
        let h = (1.0 + tau * tau).sqrt();
        let sigma = (e * (e * tau / h).atanh()).sinh();
        let tau_i = tau * (1.0 + sigma * sigma).sqrt() - sigma * h;
        let d = (tau_p - tau_i) / (1.0 + tau_i * tau_i).sqrt() * (1.0 + (1.0 - e2) * tau * tau)
            / ((1.0 - e2) * h);
        tau += d;
        if d.abs() < 1e-14 {
            break;
        }
    }
    let lam = ep.sinh().atan2(xp.cos());
    (
        zone.central_meridian() + lam.to_degrees(),
        tau.atan().to_degrees(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_point_round_trip() {
        // Reference values from PROJ (pyproj) for EPSG:32756.
        let (lon, lat) = (151.2153, -33.8568);
        let z = UtmZone::for_lonlat(lon, lat);
        assert_eq!(z.epsg(), 32756);
        let (e, n) = to_utm(lon, lat, z);
        assert!((e - 334_900.569_652).abs() < 0.001, "e={e}");
        assert!((n - 6_252_288.752_888).abs() < 0.001, "n={n}");
        let (lo, la) = from_utm(e, n, z);
        assert!(
            (lo - lon).abs() < 1e-7 && (la - lat).abs() < 1e-7,
            "{lo} {la}"
        );
    }

    #[test]
    fn northern_hemisphere_matches_proj() {
        let z = UtmZone::for_lonlat(15.5, 60.1);
        assert_eq!(z.epsg(), 32633);
        let (e, n) = to_utm(15.5, 60.1, z);
        assert!(
            (e - 527_804.454_278).abs() < 0.001 && (n - 6_662_653.220_260).abs() < 0.001,
            "{e} {n}"
        );
    }
}
