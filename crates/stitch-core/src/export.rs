//! Exports read the finished tile pyramid one row of tiles at a time, so even a
//! multi-gigapixel map exports in constant memory (GeoTIFF and PNG stream;
//! JPEG is capped by the format at 65,500 px a side and is built in memory).

use crate::project::Project;
use crate::render::{is_nodata, read_manifest, read_tile, tiles_dir, Manifest, TILE};
use crate::Ctx;
use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Format {
    GeoTiff,
    Png,
    Jpeg,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportOptions {
    pub format: Format,
    /// Zoom level to export; `None` = full resolution.
    pub zoom: Option<u32>,
    pub path: PathBuf,
}

#[derive(Debug, Clone, Serialize)]
pub struct LevelInfo {
    pub zoom: u32,
    pub width: u64,
    pub height: u64,
    pub gsd: f64,
}

pub fn levels(m: &Manifest) -> Vec<LevelInfo> {
    (0..=m.max_zoom)
        .rev()
        .map(|z| {
            let f = 1u64 << (m.max_zoom - z);
            LevelInfo { zoom: z, width: m.width.div_ceil(f), height: m.height.div_ceil(f), gsd: m.gsd * f as f64 }
        })
        .collect()
}

pub fn export(project: &Project, opts: &ExportOptions, ctx: &Ctx) -> Result<PathBuf> {
    let m = read_manifest(project).context("process the project before exporting")?;
    let z = opts.zoom.unwrap_or(m.max_zoom).min(m.max_zoom);
    let lv = levels(&m).into_iter().find(|l| l.zoom == z).unwrap();
    let (w, h) = (lv.width as usize, lv.height as usize);
    let root = tiles_dir(project);
    let path = opts.path.clone();
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d)?;
    }

    // Streams `h` RGBA rows to `sink`, one band of tiles at a time.
    let mut rows = |sink: &mut dyn FnMut(&[u8]) -> Result<()>| -> Result<()> {
        let nx = w.div_ceil(TILE);
        let ny = h.div_ceil(TILE);
        let mut band = vec![0u8; nx * TILE * TILE * 4];
        for ty in 0..ny {
            ctx.check()?;
            ctx.report("export", ty, ny, "Writing image");
            band.iter_mut().for_each(|v| *v = 0);
            for tx in 0..nx {
                if let Some(t) = read_tile(&root, z, tx, ty)? {
                    for y in 0..TILE {
                        for x in 0..TILE {
                            let s = (y * TILE + x) * 3;
                            let d = (y * nx * TILE + tx * TILE + x) * 4;
                            let p = &t[s..s + 3];
                            if !is_nodata(p) {
                                band[d..d + 3].copy_from_slice(p);
                                band[d + 3] = 255;
                            }
                        }
                    }
                }
            }
            let rows_here = TILE.min(h - ty * TILE);
            for y in 0..rows_here {
                let start = y * nx * TILE * 4;
                sink(&band[start..start + w * 4])?;
            }
        }
        ctx.report("export", ny, ny, "Export finished");
        Ok(())
    };

    match opts.format {
        Format::Png => {
            let file = BufWriter::new(std::fs::File::create(&path)?);
            let mut enc = png::Encoder::new(file, w as u32, h as u32);
            enc.set_color(png::ColorType::Rgba);
            enc.set_depth(png::BitDepth::Eight);
            enc.set_compression(png::Compression::Fast);
            let mut writer = enc.write_header()?;
            let mut stream = writer.stream_writer()?;
            rows(&mut |row| Ok(stream.write_all(row)?))?;
            stream.finish()?;
            write_world_file(&path, "pgw", &m, lv.gsd)?;
        }
        Format::Jpeg => {
            if w > 65_500 || h > 65_500 {
                bail!("JPEG can't exceed 65,500 pixels a side; pick a lower resolution or GeoTIFF");
            }
            let mut buf = Vec::with_capacity(w * h * 3);
            rows(&mut |row| {
                for p in row.chunks_exact(4) {
                    // No alpha in JPEG: empty areas become white.
                    if p[3] == 0 {
                        buf.extend_from_slice(&[255, 255, 255]);
                    } else {
                        buf.extend_from_slice(&p[..3]);
                    }
                }
                Ok(())
            })?;
            let file = BufWriter::new(std::fs::File::create(&path)?);
            let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(file, 92);
            enc.encode(&buf, w as u32, h as u32, image::ExtendedColorType::Rgb8)?;
            write_world_file(&path, "jgw", &m, lv.gsd)?;
        }
        Format::GeoTiff => write_geotiff(&path, w, h, &m, lv.gsd, &mut rows)?,
    }
    Ok(path)
}

/// ESRI world file: lets GIS tools place a PNG/JPEG without GeoTIFF tags.
fn write_world_file(path: &Path, ext: &str, m: &Manifest, gsd: f64) -> Result<()> {
    let (Some(l), Some(t)) = (m.utm_left, m.utm_top) else { return Ok(()) };
    let text = format!("{gsd:.6}\n0\n0\n{:.6}\n{:.4}\n{:.4}\n", -gsd, l + gsd / 2.0, t - gsd / 2.0);
    std::fs::write(path.with_extension(ext), text)?;
    Ok(())
}

fn write_geotiff(
    path: &Path,
    w: usize,
    h: usize,
    m: &Manifest,
    gsd: f64,
    rows: &mut dyn FnMut(&mut dyn FnMut(&[u8]) -> Result<()>) -> Result<()>,
) -> Result<()> {
    use tiff::encoder::{colortype, TiffEncoder};
    use tiff::tags::Tag;

    let file = BufWriter::new(std::fs::File::create(path)?);
    let bytes = (w * h * 4) as u64;
    // Classic TIFF tops out at 4 GB; switch to BigTIFF above ~3.5 GB.
    if bytes > 3_500_000_000 {
        let mut enc = TiffEncoder::new_big(file)?;
        let mut img = enc.new_image::<colortype::RGBA8>(w as u32, h as u32)?;
        geo_tags(img.encoder(), m, gsd)?;
        stream_strips(&mut img, w, rows)?;
        img.finish()?;
    } else {
        let mut enc = TiffEncoder::new(file)?;
        let mut img = enc.new_image::<colortype::RGBA8>(w as u32, h as u32)?;
        geo_tags(img.encoder(), m, gsd)?;
        stream_strips(&mut img, w, rows)?;
        img.finish()?;
    }
    let _ = Tag::ImageWidth;
    Ok(())
}

fn geo_tags<W: std::io::Write + std::io::Seek, K: tiff::encoder::TiffKind>(
    dir: &mut tiff::encoder::DirectoryEncoder<W, K>,
    m: &Manifest,
    gsd: f64,
) -> Result<()> {
    use tiff::tags::Tag;
    let (Some(l), Some(t), Some(epsg)) = (m.utm_left, m.utm_top, m.epsg) else { return Ok(()) };
    dir.write_tag(Tag::Unknown(33550), &[gsd, gsd, 0.0][..])?; // ModelPixelScale
    dir.write_tag(Tag::Unknown(33922), &[0.0, 0.0, 0.0, l, t, 0.0][..])?; // ModelTiepoint
    // GeoKeyDirectory: version 1.1.0, 3 keys.
    let keys: [u16; 16] = [
        1, 1, 0, 3, //
        1024, 0, 1, 1, // GTModelType = projected
        1025, 0, 1, 1, // GTRasterType = PixelIsArea
        3072, 0, 1, epsg as u16, // ProjectedCSType = EPSG
    ];
    dir.write_tag(Tag::Unknown(34735), &keys[..])?;
    Ok(())
}

fn stream_strips<W: std::io::Write + std::io::Seek, K: tiff::encoder::TiffKind>(
    img: &mut tiff::encoder::ImageEncoder<W, tiff::encoder::colortype::RGBA8, K>,
    w: usize,
    rows: &mut dyn FnMut(&mut dyn FnMut(&[u8]) -> Result<()>) -> Result<()>,
) -> Result<()> {
    const STRIP_ROWS: usize = 16;
    img.rows_per_strip(STRIP_ROWS as u32)?;
    let mut strip = Vec::with_capacity(w * 4 * STRIP_ROWS);
    rows(&mut |row| {
        strip.extend_from_slice(row);
        if strip.len() == w * 4 * STRIP_ROWS {
            img.write_strip(&strip)?;
            strip.clear();
        }
        Ok(())
    })?;
    if !strip.is_empty() {
        img.write_strip(&strip)?;
    }
    Ok(())
}
