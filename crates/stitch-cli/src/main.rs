//! Command-line front end, mostly for testing and batch runs:
//!
//!   aerial-stitch process <project-dir> [photo-folder ...]
//!   aerial-stitch export  <project-dir> <out.tif|out.png|out.jpg> [zoom]

use anyhow::{bail, Result};
use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use stitch_core::export::{export, ExportOptions, Format};
use stitch_core::{Ctx, Project};

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let ctx = Ctx::new(
        |p| eprintln!("[{:>8}] {:>5}/{:<5} {}", p.stage, p.done, p.total, p.message),
        Arc::new(AtomicBool::new(false)),
    );
    match args.first().map(String::as_str) {
        Some("process") if args.len() >= 2 => {
            let dir = PathBuf::from(&args[1]);
            let mut project = if dir.join("project.json").exists() {
                Project::open(&dir)?
            } else {
                Project::create(&dir, dir.file_name().and_then(|n| n.to_str()).unwrap_or("Project"))?
            };
            let photos: Vec<PathBuf> = args[2..].iter().map(PathBuf::from).collect();
            if !photos.is_empty() {
                let (added, failed) = project.add_photos(&photos)?;
                eprintln!("added {added} photos, {} unreadable", failed.len());
            }
            let t = std::time::Instant::now();
            let s = stitch_core::pipeline::process(&project, &ctx)?;
            println!("{}", serde_json_like(&s));
            eprintln!("took {:.1}s", t.elapsed().as_secs_f64());
        }
        Some("export") if args.len() >= 3 => {
            let project = Project::open(&PathBuf::from(&args[1]))?;
            let path = PathBuf::from(&args[2]);
            let format = match path.extension().and_then(|e| e.to_str()).map(str::to_lowercase).as_deref() {
                Some("tif" | "tiff") => Format::GeoTiff,
                Some("png") => Format::Png,
                Some("jpg" | "jpeg") => Format::Jpeg,
                _ => bail!("output must end in .tif, .png or .jpg"),
            };
            let zoom = args.get(3).map(|z| z.parse()).transpose()?;
            let out = export(&project, &ExportOptions { format, zoom, path }, &ctx)?;
            println!("wrote {}", out.display());
        }
        Some("plan") => {
            // Reads a PlanInput as JSON on stdin, prints the Plan as JSON;
            // with an output folder, also writes the Litchi CSVs and KML.
            let mut text = String::new();
            std::io::Read::read_to_string(&mut std::io::stdin(), &mut text)?;
            let input: stitch_core::plan::PlanInput = serde_json::from_str(&text)?;
            let p = stitch_core::plan::plan(&input)?;
            if let Some(dir) = args.get(1) {
                std::fs::create_dir_all(dir)?;
                for (i, m) in p.missions.iter().enumerate() {
                    let path = PathBuf::from(dir).join(format!("mission_{}.csv", i + 1));
                    std::fs::write(path, stitch_core::plan::litchi_csv(&input, m, p.stats.speed_ms))?;
                }
                std::fs::write(PathBuf::from(dir).join("route.kml"), stitch_core::plan::plan_kml(&input, &p))?;
            }
            println!("{}", serde_json::to_string(&p)?);
        }
        _ => {
            eprintln!("usage:\n  aerial-stitch process <project-dir> [photos-or-folders ...]\n  aerial-stitch export <project-dir> <out.tif|.png|.jpg> [zoom]\n  aerial-stitch plan [out-dir] < plan.json");
            std::process::exit(2);
        }
    }
    Ok(())
}

fn serde_json_like(s: &stitch_core::pipeline::Summary) -> String {
    format!(
        "photos={} placed={} unplaced={} pairs={}/{} rms={:.2}px size={}x{} gsd={:.4} georef={}",
        s.photos, s.placed, s.unplaced, s.pairs_matched, s.pairs_tried, s.rms_px, s.width, s.height, s.gsd, s.georeferenced
    )
}
