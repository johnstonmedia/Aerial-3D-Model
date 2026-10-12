//! Per-photo metadata: pixel size, GPS position and (for DJI drones) the
//! gimbal yaw and pitch from the XMP block.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::io::{BufReader, Read};
use std::path::Path;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct PhotoMeta {
    pub width: u32,
    pub height: u32,
    pub lon: Option<f64>,
    pub lat: Option<f64>,
    /// Altitude above sea level, metres (EXIF).
    pub alt: Option<f64>,
    /// Height above take-off, metres (DJI XMP).
    pub rel_alt: Option<f64>,
    /// Gimbal yaw, degrees clockwise from north (DJI XMP).
    pub yaw: Option<f64>,
    /// Gimbal pitch, degrees; -90 is straight down (DJI XMP).
    pub pitch: Option<f64>,
    /// Gimbal roll, degrees (DJI XMP).
    pub roll: Option<f64>,
    pub timestamp: Option<String>,
    /// Lens focal length, mm (EXIF).
    pub focal_mm: Option<f64>,
    /// 35 mm-equivalent focal length (EXIF), when the camera records it.
    pub focal_35mm: Option<f64>,
    pub make: Option<String>,
    pub model: Option<String>,
}

impl PhotoMeta {
    pub fn has_gps(&self) -> bool {
        self.lon.is_some() && self.lat.is_some()
    }
}

pub fn read_meta(path: &Path) -> Result<PhotoMeta> {
    let (width, height) =
        image::image_dimensions(path).with_context(|| format!("reading {}", path.display()))?;
    let mut m = PhotoMeta {
        width,
        height,
        ..Default::default()
    };

    if let Ok(file) = std::fs::File::open(path) {
        let mut reader = BufReader::new(file);
        if let Ok(exif) = exif::Reader::new().read_from_container(&mut reader) {
            read_exif(&exif, &mut m);
        }
    }
    if let Ok(xmp) = read_head(path, 256 * 1024) {
        read_dji_xmp(&xmp, &mut m);
    }
    Ok(m)
}

fn read_exif(exif: &exif::Exif, m: &mut PhotoMeta) {
    use exif::{In, Tag, Value};
    let rationals = |tag| match exif.get_field(tag, In::PRIMARY).map(|f| &f.value) {
        Some(Value::Rational(v)) => Some(v.iter().map(|r| r.to_f64()).collect::<Vec<_>>()),
        _ => None,
    };
    let refc = |tag| {
        exif.get_field(tag, In::PRIMARY).map(|f| {
            f.display_value()
                .to_string()
                .trim_matches('"')
                .to_uppercase()
        })
    };
    let dms =
        |v: Vec<f64>| -> Option<f64> { (v.len() == 3).then(|| v[0] + v[1] / 60.0 + v[2] / 3600.0) };
    if let (Some(lat), Some(lon)) = (
        rationals(Tag::GPSLatitude).and_then(dms),
        rationals(Tag::GPSLongitude).and_then(dms),
    ) {
        let lat = if refc(Tag::GPSLatitudeRef).is_some_and(|r| r.starts_with('S')) {
            -lat
        } else {
            lat
        };
        let lon = if refc(Tag::GPSLongitudeRef).is_some_and(|r| r.starts_with('W')) {
            -lon
        } else {
            lon
        };
        if lat.is_finite() && lon.is_finite() && (lat != 0.0 || lon != 0.0) {
            m.lat = Some(lat);
            m.lon = Some(lon);
        }
    }
    if let Some(v) = rationals(Tag::GPSAltitude) {
        if let Some(a) = v.first() {
            let below = matches!(
                exif.get_field(Tag::GPSAltitudeRef, In::PRIMARY)
                    .and_then(|f| f.value.get_uint(0)),
                Some(1)
            );
            m.alt = Some(if below { -a } else { *a });
        }
    }
    if let Some(v) = rationals(Tag::FocalLength) {
        m.focal_mm = v.first().copied().filter(|f| *f > 0.0);
    }
    if let Some(f) = exif.get_field(Tag::FocalLengthIn35mmFilm, In::PRIMARY) {
        m.focal_35mm = f.value.get_uint(0).map(|v| v as f64).filter(|v| *v > 0.0);
    }
    let text = |tag| {
        exif.get_field(tag, In::PRIMARY)
            .map(|f| {
                f.display_value()
                    .to_string()
                    .trim_matches('"')
                    .trim()
                    .to_string()
            })
            .filter(|s| !s.is_empty())
    };
    m.make = text(Tag::Make);
    m.model = text(Tag::Model);
    if let Some(f) = exif.get_field(Tag::DateTimeOriginal, In::PRIMARY) {
        m.timestamp = Some(f.display_value().to_string());
    }
}

fn read_head(path: &Path, max: usize) -> Result<String> {
    let mut buf = Vec::with_capacity(max);
    std::fs::File::open(path)?
        .take(max as u64)
        .read_to_end(&mut buf)?;
    Ok(String::from_utf8_lossy(&buf).into_owned())
}

/// DJI writes `drone-dji:Key="value"` (attribute form) or
/// `<drone-dji:Key>value</drone-dji:Key>` (element form).
fn xmp_value(xmp: &str, key: &str) -> Option<f64> {
    let attr = format!("drone-dji:{key}=\"");
    if let Some(i) = xmp.find(&attr) {
        let rest = &xmp[i + attr.len()..];
        return rest[..rest.find('"')?].trim().parse().ok();
    }
    let open = format!("<drone-dji:{key}>");
    let i = xmp.find(&open)?;
    let rest = &xmp[i + open.len()..];
    rest[..rest.find('<')?].trim().parse().ok()
}

fn read_dji_xmp(xmp: &str, m: &mut PhotoMeta) {
    if !xmp.contains("drone-dji:") {
        return;
    }
    m.rel_alt = xmp_value(xmp, "RelativeAltitude");
    m.yaw = xmp_value(xmp, "GimbalYawDegree").or_else(|| xmp_value(xmp, "FlightYawDegree"));
    m.pitch = xmp_value(xmp, "GimbalPitchDegree");
    m.roll = xmp_value(xmp, "GimbalRollDegree");
    if m.lat.is_none() {
        if let (Some(lat), Some(lon)) = (
            xmp_value(xmp, "GpsLatitude"),
            xmp_value(xmp, "GpsLongitude"),
        ) {
            m.lat = Some(lat);
            m.lon = Some(lon);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_both_xmp_forms() {
        let a = r#"<rdf:Description drone-dji:GimbalYawDegree="-87.40" drone-dji:RelativeAltitude="+80.20">"#;
        let b = "<drone-dji:GimbalPitchDegree>-90.0</drone-dji:GimbalPitchDegree>";
        let mut m = PhotoMeta::default();
        read_dji_xmp(&format!("{a}{b}"), &mut m);
        assert_eq!(m.yaw, Some(-87.4));
        assert_eq!(m.rel_alt, Some(80.2));
        assert_eq!(m.pitch, Some(-90.0));
    }
}
