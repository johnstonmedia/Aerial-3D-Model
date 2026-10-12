//! Binary PLY point clouds — opened directly by CloudCompare, MeshLab and
//! Blender, and by the app's 3D viewer.

use anyhow::Result;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Writes `xyz` (local metres) + 8-bit colour as little-endian binary PLY.
/// Positions are stored as f32 relative to the project origin; the origin
/// is written as a comment so the absolute location is never lost.
pub fn write_ply(
    path: &Path,
    points: &[[f32; 3]],
    colors: &[[u8; 3]],
    comment: &str,
) -> Result<()> {
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

/// Streams the points of a PLY written by [`write_ply`] (xyz f32 + rgb u8),
/// a chunk at a time, so a cloud of any size is read in constant memory.
pub fn read_ply(path: &Path, mut each: impl FnMut([f32; 3], [u8; 3])) -> Result<usize> {
    use std::io::{BufRead, BufReader, Read};
    let mut r = BufReader::with_capacity(1 << 20, std::fs::File::open(path)?);
    let mut n = 0usize;
    let mut line = String::new();
    loop {
        line.clear();
        if r.read_line(&mut line)? == 0 {
            anyhow::bail!("{} has no PLY header", path.display());
        }
        if let Some(v) = line.strip_prefix("element vertex ") {
            n = v.trim().parse()?;
        }
        if line.trim() == "end_header" {
            break;
        }
    }
    let mut buf = vec![0u8; 15 * 65536];
    let mut left = n;
    while left > 0 {
        let k = left.min(65536);
        r.read_exact(&mut buf[..k * 15])?;
        for rec in buf[..k * 15].chunks_exact(15) {
            let f = |o: usize| f32::from_le_bytes(rec[o..o + 4].try_into().unwrap());
            each([f(0), f(4), f(8)], [rec[12], rec[13], rec[14]]);
        }
        left -= k;
    }
    Ok(n)
}
