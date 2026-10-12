//! Binary PLY point clouds — opened directly by CloudCompare, MeshLab and
//! Blender, and by the app's 3D viewer.

use anyhow::Result;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Writes `xyz` (local metres) + 8-bit colour as little-endian binary PLY.
/// Positions are stored as f32 relative to the project origin; the origin
/// is written as a comment so the absolute location is never lost.
pub fn write_ply(path: &Path, points: &[[f32; 3]], colors: &[[u8; 3]], comment: &str) -> Result<()> {
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d)?;
    }
    let tmp = path.with_extension("ply.tmp");
    {
        let mut w = BufWriter::new(std::fs::File::create(&tmp)?);
        write!(
            w,
            "ply\nformat binary_little_endian 1.0\ncomment {comment}\nelement vertex {}\n\
             property float x\nproperty float y\nproperty float z\n\
             property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
            points.len()
        )?;
        for (p, c) in points.iter().zip(colors) {
            for v in p {
                w.write_all(&v.to_le_bytes())?;
            }
            w.write_all(c)?;
        }
        w.flush()?;
    }
    std::fs::rename(&tmp, path)?;
    Ok(())
}
