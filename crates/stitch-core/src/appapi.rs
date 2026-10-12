//! The operations the app's UI calls, shared by both front ends:
//!
//! * the desktop app (`app/src-tauri`) wraps each one in a Tauri command;
//! * the web demo runs this crate as WebAssembly and reaches them through
//!   [`dispatch`] (JSON in, JSON out) via `aerial-stitch api`.
//!
//! Keeping them here is what makes the web demo the same program as the
//! desktop app rather than a look-alike.

use crate::export::{export, levels, ExportOptions, Format, LevelInfo};
use crate::plan::{camera_presets, litchi_csv, parse_kml_boundary, plan, plan_kml, Camera, Plan, PlanInput};
use crate::project::Settings;
use crate::render::{read_layer_manifest, read_manifest, Layer, Manifest};
use crate::{Ctx, ProfileKind, Project};
use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

#[derive(Serialize)]
pub struct PhotoView {
    pub id: u32,
    pub name: String,
    pub path: String,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub yaw: Option<f64>,
    pub width: u32,
    pub height: u32,
    pub missing: bool,
}

#[derive(Serialize)]
pub struct ProjectView {
    pub dir: String,
    pub name: String,
    pub photos: Vec<PhotoView>,
    pub settings: Settings,
    pub manifest: Option<Manifest>,
    pub levels: Vec<LevelInfo>,
    /// True orthophoto, once the 3D model has been built.
    pub ortho: Option<Manifest>,
    pub ortho_levels: Vec<LevelInfo>,
    pub model3d: Option<Model3d>,
}

#[derive(Serialize)]
pub struct Model3d {
    /// Points in the dense cloud (0 if only the sparse stage has run).
    pub points: u64,
    pub sparse_points: u64,
    /// The textured surface model (model_view.glb) exists.
    pub mesh: bool,
    /// The surface model offset that maps viewer coordinates back to the
    /// project frame (glTF x, y, z = east, up, −north, minus this offset).
    pub mesh_offset: Option<[f64; 3]>,
    /// Changes when the model is rebuilt, so the viewer reloads it.
    pub version: u64,
}

/// Vertex count from a PLY header, without reading the body.
fn ply_count(path: &Path) -> Option<u64> {
    use std::io::Read;
    let mut head = vec![0u8; 1024];
    let n = std::fs::File::open(path).ok()?.read(&mut head).ok()?;
    let text = String::from_utf8_lossy(&head[..n]);
    text.lines().find_map(|l| l.strip_prefix("element vertex ")?.trim().parse().ok())
}

/// Reads the recentring offset from the GLB's JSON chunk (asset.extras).
fn mesh_offset(dir: &Path) -> Option<[f64; 3]> {
    use std::io::Read;
    let mut f = std::fs::File::open(dir.join("model_view.glb")).ok()?;
    let mut head = [0u8; 20];
    f.read_exact(&mut head).ok()?;
    let len = u32::from_le_bytes(head[12..16].try_into().ok()?) as usize;
    let mut js = vec![0u8; len];
    f.read_exact(&mut js).ok()?;
    let v: Value = serde_json::from_slice(&js).ok()?;
    let e = &v["asset"]["extras"];
    Some([e["local_offset"][0].as_f64()?, e["local_offset"][1].as_f64()?, e["local_offset"][2].as_f64()?])
}

fn model3d(p: &Project) -> Option<Model3d> {
    let dir = crate::recon::out_dir(p);
    let sparse_points = ply_count(&dir.join("sparse.ply"))?;
    let dense = dir.join("dense.ply");
    let version = std::fs::metadata(if dense.exists() { &dense } else { &dir })
        .and_then(|m| m.modified())
        .ok()
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map(|d| d.as_millis() as u64)
        // No modification times (the web demo's file system): a counter of
        // sorts is still needed so the viewer reloads after a rebuild.
        .unwrap_or_else(|| ply_count(&dense).unwrap_or(0) ^ sparse_points);
    let mesh = dir.join("model_view.glb").exists();
    Some(Model3d { points: ply_count(&dense).unwrap_or(0), sparse_points, mesh, mesh_offset: mesh_offset(&dir), version })
}

pub fn view(p: &Project) -> ProjectView {
    let manifest = read_manifest(p).ok();
    let ortho = read_layer_manifest(p, Layer::Ortho).ok();
    ProjectView {
        dir: p.dir.to_string_lossy().into_owned(),
        name: p.file.name.clone(),
        photos: p
            .file
            .photos
            .iter()
            .map(|ph| PhotoView {
                id: ph.id,
                name: ph.path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default(),
                path: ph.path.to_string_lossy().into_owned(),
                lat: ph.meta.lat,
                lon: ph.meta.lon,
                yaw: ph.meta.yaw,
                width: ph.meta.width,
                height: ph.meta.height,
                missing: !ph.path.exists(),
            })
            .collect(),
        settings: p.file.settings.clone(),
        levels: manifest.as_ref().map(levels).unwrap_or_default(),
        manifest,
        ortho_levels: ortho.as_ref().map(levels).unwrap_or_default(),
        ortho,
        model3d: model3d(p),
    }
}

pub fn create_project(dir: &Path, name: &str) -> Result<ProjectView> {
    Ok(view(&Project::create(dir, name)?))
}

pub fn open_project(dir: &Path) -> Result<ProjectView> {
    let mut p = Project::open(dir)?;
    p.refresh()?;
    Ok(view(&p))
}

#[derive(Serialize)]
pub struct AddResult {
    pub added: usize,
    pub failed: Vec<(String, String)>,
    pub project: ProjectView,
}

pub fn add_photos(dir: &Path, paths: &[PathBuf]) -> Result<AddResult> {
    let mut p = Project::open(dir)?;
    let (added, failed) = p.add_photos(paths)?;
    Ok(AddResult {
        added,
        failed: failed.into_iter().map(|(p, e)| (p.to_string_lossy().into_owned(), e)).collect(),
        project: view(&p),
    })
}

pub fn remove_photos(dir: &Path, ids: &[u32]) -> Result<ProjectView> {
    let mut p = Project::open(dir)?;
    p.remove_photos(ids)?;
    Ok(view(&p))
}

pub fn save_settings(dir: &Path, settings: Settings) -> Result<ProjectView> {
    let mut p = Project::open(dir)?;
    p.file.settings = settings;
    p.save()?;
    Ok(view(&p))
}

pub fn set_profile(dir: &Path, profile: Option<ProfileKind>) -> Result<ProjectView> {
    let mut p = Project::open(dir)?;
    p.file.settings.profile = profile;
    p.save()?;
    Ok(view(&p))
}

// ─────────────────────────────── Long jobs ───────────────────────────────

/// Outcome of a long job, as the UI's `job-finished` event carries it.
#[derive(Clone, Serialize)]
pub struct JobFinished {
    pub kind: &'static str,
    pub ok: bool,
    pub message: String,
    pub detail: Value,
}

impl JobFinished {
    fn failed(kind: &'static str, e: anyhow::Error, ctx: &Ctx) -> Self {
        let message = if ctx.cancelled() { "Cancelled".into() } else { format!("{e:#}") };
        JobFinished { kind, ok: false, message, detail: Value::Null }
    }
}

pub fn process(dir: &Path, ctx: &Ctx) -> JobFinished {
    match Project::open(dir).and_then(|p| crate::pipeline::process(&p, ctx)) {
        Ok(s) => JobFinished {
            kind: "process",
            ok: true,
            message: if s.oblique_skipped > 0 {
                format!("Placed {} of {} photos · {} angled photos skipped (kept for 3D)", s.placed, s.photos, s.oblique_skipped)
            } else {
                format!("Placed {} of {} photos", s.placed, s.photos)
            },
            detail: serde_json::to_value(&s).unwrap_or_default(),
        },
        Err(e) => JobFinished::failed("process", e, ctx),
    }
}

pub fn export_map(dir: &Path, format: Format, layer: Option<Layer>, zoom: Option<u32>, path: &Path, ctx: &Ctx) -> JobFinished {
    let result = Project::open(dir).and_then(|p| {
        export(&p, &ExportOptions { format, layer: layer.unwrap_or_default(), zoom, path: path.to_path_buf() }, ctx)
    });
    match result {
        Ok(out) => JobFinished {
            kind: "export",
            ok: true,
            message: format!("Saved {}", out.display()),
            detail: json!({ "path": out }),
        },
        Err(e) => JobFinished::failed("export", e, ctx),
    }
}

/// Sparse → dense → surface + true ortho → textured mesh.
pub fn build_3d(dir: &Path, ctx: &Ctx) -> JobFinished {
    let result = Project::open(dir).and_then(|p| {
        let s = crate::recon::run_sparse(&p, ctx)?;
        let (n, voxel) = crate::recon::run_dense(&p, ctx)?;
        crate::recon::run_ortho(&p, ctx)?;
        crate::recon::run_mesh(&p, ctx)?;
        Ok((s, n, voxel))
    });
    match result {
        Ok((s, n, voxel)) => JobFinished {
            kind: "3d",
            ok: true,
            message: format!(
                "3D surface, point cloud and true orthophoto ready: {} points from {} photos ({} angled)",
                n,
                s.cameras.len(),
                s.cameras.iter().filter(|c| c.oblique).count()
            ),
            detail: json!({ "points": n, "voxel_m": voxel, "rms_px": s.rms_px }),
        },
        Err(e) => JobFinished::failed("3d", e, ctx),
    }
}

// ─────────────────────────────── 3D outputs ───────────────────────────────

pub fn measure_volume(
    dir: &Path,
    polygon: &[(f64, f64)],
    level: crate::recon::measure::Level,
) -> Result<crate::recon::measure::VolumeResult> {
    let p = Project::open(dir)?;
    let d = crate::recon::dsm::Dsm::load(&crate::recon::out_dir(&p).join("dsm.bin"))
        .map_err(|_| anyhow!("build the 3D model first: volume is measured on its surface"))?;
    crate::recon::measure::volume(&d, polygon, level).ok_or_else(|| anyhow!("that outline doesn't cover any of the surface"))
}

/// Copies the full-detail textured surface (GLB) out.
pub fn export_model(dir: &Path, path: &Path) -> Result<String> {
    let p = Project::open(dir)?;
    std::fs::copy(crate::recon::out_dir(&p).join("model.glb"), path)?;
    Ok(path.to_string_lossy().into_owned())
}

/// Copies the dense cloud (or the sparse one, if that's all there is) out.
pub fn export_cloud(dir: &Path, path: &Path) -> Result<String> {
    let p = Project::open(dir)?;
    let d = crate::recon::out_dir(&p);
    let src = if d.join("dense.ply").exists() { d.join("dense.ply") } else { d.join("sparse.ply") };
    std::fs::copy(&src, path)?;
    Ok(path.to_string_lossy().into_owned())
}

// ─────────────────────────────── Flight planner ───────────────────────────────

pub fn plan_defaults() -> (PlanInput, Vec<Camera>) {
    (PlanInput::default(), camera_presets())
}

pub fn plan_flight(input: &PlanInput) -> Result<Plan> {
    plan(input)
}

fn safe_name(s: &str) -> String {
    let t: String = s.chars().map(|c| if c.is_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
    if t.is_empty() {
        "survey".into()
    } else {
        t
    }
}

/// Writes one Litchi CSV per mission plus a KML of the whole route.
pub fn export_litchi(input: &PlanInput, dir: &Path) -> Result<Vec<String>> {
    let p = plan(input)?;
    std::fs::create_dir_all(dir)?;
    let base = safe_name(&input.name);
    let mut written = Vec::new();
    let n = p.missions.len();
    for (i, m) in p.missions.iter().enumerate() {
        let file = if n == 1 { format!("{base}_litchi.csv") } else { format!("{base}_litchi_{}of{n}.csv", i + 1) };
        let path = dir.join(file);
        std::fs::write(&path, litchi_csv(input, m, p.stats.speed_ms))?;
        written.push(path.to_string_lossy().into_owned());
    }
    let kml = dir.join(format!("{base}_route.kml"));
    std::fs::write(&kml, plan_kml(input, &p))?;
    written.push(kml.to_string_lossy().into_owned());
    Ok(written)
}

/// Boundary from a .kml, or the first .kml inside a .kmz.
pub fn import_boundary(path: &Path) -> Result<Vec<(f64, f64)>> {
    let bytes = std::fs::read(path)?;
    let is_kmz = path.extension().is_some_and(|e| e.eq_ignore_ascii_case("kmz"));
    let text = if is_kmz {
        let mut zip = zip::ZipArchive::new(std::io::Cursor::new(bytes))?;
        let name = (0..zip.len())
            .filter_map(|i| zip.by_index(i).ok().map(|f| f.name().to_string()))
            .find(|n| n.to_lowercase().ends_with(".kml"))
            .context("no KML inside that KMZ")?;
        let mut s = String::new();
        std::io::Read::read_to_string(&mut zip.by_name(&name)?, &mut s)?;
        s
    } else {
        String::from_utf8_lossy(&bytes).into_owned()
    };
    parse_kml_boundary(&text)
}

pub fn save_plan(path: &Path, input: &PlanInput) -> Result<()> {
    Ok(std::fs::write(path, serde_json::to_vec_pretty(input)?)?)
}

pub fn load_plan(path: &Path) -> Result<PlanInput> {
    Ok(serde_json::from_slice(&std::fs::read(path)?)?)
}

// ─────────────────────────────── JSON dispatch ───────────────────────────────

/// One call from the web demo: a command name and its arguments, named as
/// the UI's `invoke()` sends them.
#[derive(Deserialize)]
pub struct Request {
    pub cmd: String,
    #[serde(default)]
    pub args: Value,
}

fn arg<T: serde::de::DeserializeOwned>(args: &Value, name: &str) -> Result<T> {
    serde_json::from_value(args.get(name).cloned().unwrap_or(Value::Null)).with_context(|| format!("argument `{name}`"))
}

fn path_arg(args: &Value, name: &str) -> Result<PathBuf> {
    Ok(PathBuf::from(arg::<String>(args, name)?))
}

/// Runs a command by name. Long jobs (`process`, `export_map`, `build_3d`)
/// run to completion here and return their [`JobFinished`].
pub fn dispatch(req: &Request, ctx: &Ctx) -> Result<Value> {
    let a = &req.args;
    let v = match req.cmd.as_str() {
        "create_project" => json!(create_project(&path_arg(a, "dir")?, &arg::<String>(a, "name")?)?),
        "open_project" => json!(open_project(&path_arg(a, "dir")?)?),
        "add_photos" => {
            let paths: Vec<PathBuf> = arg::<Vec<String>>(a, "paths")?.into_iter().map(PathBuf::from).collect();
            json!(add_photos(&path_arg(a, "dir")?, &paths)?)
        }
        "remove_photos" => json!(remove_photos(&path_arg(a, "dir")?, &arg::<Vec<u32>>(a, "ids")?)?),
        "save_settings" => json!(save_settings(&path_arg(a, "dir")?, arg(a, "settings")?)?),
        "set_profile" => json!(set_profile(&path_arg(a, "dir")?, arg(a, "profile")?)?),
        "process" => json!(process(&path_arg(a, "dir")?, ctx)),
        "export_map" => json!(export_map(
            &path_arg(a, "dir")?,
            arg(a, "format")?,
            arg(a, "layer")?,
            arg(a, "zoom")?,
            &path_arg(a, "path")?,
            ctx
        )),
        "build_3d" => json!(build_3d(&path_arg(a, "dir")?, ctx)),
        "measure_volume" => {
            json!(measure_volume(&path_arg(a, "dir")?, &arg::<Vec<(f64, f64)>>(a, "polygon")?, arg(a, "level")?)?)
        }
        "export_model" => json!(export_model(&path_arg(a, "dir")?, &path_arg(a, "path")?)?),
        "export_cloud" => json!(export_cloud(&path_arg(a, "dir")?, &path_arg(a, "path")?)?),
        "plan_defaults" => json!(plan_defaults()),
        "plan_flight" => json!(plan_flight(&arg(a, "input")?)?),
        "export_litchi" => json!(export_litchi(&arg(a, "input")?, &path_arg(a, "dir")?)?),
        "import_boundary" => json!(import_boundary(&path_arg(a, "path")?)?),
        "save_plan" => json!(save_plan(&path_arg(a, "path")?, &arg(a, "input")?)?),
        "load_plan" => json!(load_plan(&path_arg(a, "path")?)?),
        other => return Err(anyhow!("unknown command `{other}`")),
    };
    Ok(v)
}
