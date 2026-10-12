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
            let zoom = args.get(3).filter(|a| !a.starts_with("--")).map(|z| z.parse()).transpose()?;
            let layer = if args.iter().any(|a| a == "--ortho") {
                stitch_core::render::Layer::Ortho
            } else {
                stitch_core::render::Layer::Mosaic
            };
            let out = export(&project, &ExportOptions { format, layer, zoom, path }, &ctx)?;
            println!("wrote {}", out.display());
        }
        Some("sparse") if args.len() >= 2 => {
            let project = Project::open(&PathBuf::from(&args[1]))?;
            let t = std::time::Instant::now();
            let s = stitch_core::recon::run_sparse(&project, &ctx)?;
            println!(
                "cameras={} skipped={} points={} rms={:.2}px f={:.1} k1={:.4} k2={:.4} took {:.1}s",
                s.cameras.len(), s.skipped.len(), s.points.len(), s.rms_px,
                s.intrinsics[0].f, s.intrinsics[0].k1, s.intrinsics[0].k2, t.elapsed().as_secs_f64()
            );
        }
        Some("dense") if args.len() >= 2 => {
            let project = Project::open(&PathBuf::from(&args[1]))?;
            let t = std::time::Instant::now();
            let (n, voxel) = stitch_core::recon::run_dense(&project, &ctx)?;
            println!("dense points={n} voxel={voxel:.3}m took {:.1}s", t.elapsed().as_secs_f64());
        }
        Some("ortho") if args.len() >= 2 => {
            let project = Project::open(&PathBuf::from(&args[1]))?;
            let t = std::time::Instant::now();
            let m = stitch_core::recon::run_ortho(&project, &ctx)?;
            println!("true ortho {}x{} gsd={:.4} took {:.1}s", m.width, m.height, m.gsd, t.elapsed().as_secs_f64());
        }
        Some("mesh") if args.len() >= 2 => {
            let project = Project::open(&PathBuf::from(&args[1]))?;
            let t = std::time::Instant::now();
            let m = stitch_core::recon::run_mesh(&project, &ctx)?;
            println!(
                "mesh vertices={} triangles={} walls={} (from oblique {}) took {:.1}s",
                m.vertices, m.triangles, m.wall_triangles, m.walls_from_oblique, t.elapsed().as_secs_f64()
            );
        }
        Some("api") => {
            // The web demo's backend: one request as JSON on stdin, the
            // answer as JSON on stdout, progress as `@progress {json}` lines
            // on stderr. See stitch_core::appapi.
            let mut text = String::new();
            std::io::Read::read_to_string(&mut std::io::stdin(), &mut text)?;
            let req: stitch_core::appapi::Request = serde_json::from_str(&text)?;
            let last = std::sync::Mutex::new(None::<std::time::Instant>);
            let ctx = Ctx::new(
                move |p| {
                    let mut l = last.lock().unwrap();
                    let due = l.map_or(true, |t| t.elapsed().as_millis() >= 150);
                    if due || p.done == p.total || p.done == 0 {
                        *l = Some(std::time::Instant::now());
                        eprintln!("@progress {}", serde_json::to_string(&p).unwrap_or_default());
                    }
                },
                Arc::new(AtomicBool::new(false)),
            );
            let out = match stitch_core::appapi::dispatch(&req, &ctx) {
                Ok(v) => serde_json::json!({ "ok": true, "value": v }),
                Err(e) => serde_json::json!({ "ok": false, "error": format!("{e:#}") }),
            };
            println!("{out}");
        }
        Some("volume") if args.len() >= 3 => {
            // volume <project> "x,y x,y x,y ..." [lowest|mean|highest|<height m>]
            // Outline in the 3D model's frame (as the true ortho shows it).
            let project = Project::open(&PathBuf::from(&args[1]))?;
            let poly: Vec<(f64, f64)> = args[2]
                .split_whitespace()
                .filter_map(|p| {
                    let (x, y) = p.split_once(',')?;
                    Some((x.parse().ok()?, y.parse().ok()?))
                })
                .collect();
            use stitch_core::recon::measure::{volume, Level};
            let level = match args.get(3).map(String::as_str) {
                None | Some("lowest") => Level::LowestEdge,
                Some("mean") => Level::MeanEdge,
                Some("highest") => Level::HighestEdge,
                Some(z) => Level::Height { z: z.parse()? },
            };
            let dsm = stitch_core::recon::dsm::Dsm::load(&stitch_core::recon::out_dir(&project).join("dsm.bin"))?;
            match volume(&dsm, &poly, level) {
                Some(r) => println!("{}", serde_json::to_string_pretty(&r)?),
                None => anyhow::bail!("the outline doesn't cover the surface"),
            }
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
            eprintln!("usage:\n  aerial-stitch process <project-dir> [photos-or-folders ...]\n  aerial-stitch export <project-dir> <out.tif|.png|.jpg> [zoom]\n  aerial-stitch plan [out-dir] < plan.json\n  aerial-stitch sparse|dense|ortho|mesh <project-dir>\n  aerial-stitch volume <project-dir> \"x,y x,y x,y\" [lowest|mean|highest|<height>]");
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
