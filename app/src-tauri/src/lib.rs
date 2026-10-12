//! Tauri shell: thin commands over `stitch-core`. Long jobs run on their own
//! thread and report through events (`progress`, `job-finished`), so the
//! window never blocks.
//!
//! Tiles are served through a custom `tiles://` protocol that reads from the
//! open project's result folder — no file-system scope has to be opened up to
//! the webview.

use serde::Serialize;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use stitch_core::export::{export, levels, ExportOptions, Format, LevelInfo};
use stitch_core::project::Settings;
use stitch_core::render::{read_manifest, Manifest};
use stitch_core::{Ctx, Profile, ProfileKind, Project};
use tauri::{AppHandle, Emitter, Manager, State};

#[derive(Default)]
struct AppState {
    /// Folder of the project whose tiles `tiles://` serves.
    current: Mutex<Option<PathBuf>>,
    /// Set while a job runs; flipping it cancels the job.
    job: Mutex<Option<Arc<AtomicBool>>>,
}

type CmdResult<T> = Result<T, String>;

fn err(e: impl std::fmt::Display) -> String {
    e.to_string()
}

fn anyerr(e: anyhow::Error) -> String {
    format!("{e:#}")
}

#[derive(Serialize)]
struct SystemInfo {
    os: &'static str,
    arch: &'static str,
    ram_gb: f64,
    cores: usize,
    profile: Profile,
}

#[tauri::command]
fn system_info() -> SystemInfo {
    SystemInfo {
        os: std::env::consts::OS,
        arch: std::env::consts::ARCH,
        ram_gb: stitch_core::profile::total_ram_bytes() as f64 / (1u64 << 30) as f64,
        cores: std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1),
        profile: Profile::detect(),
    }
}

#[derive(Serialize)]
struct PhotoView {
    id: u32,
    name: String,
    path: String,
    lat: Option<f64>,
    lon: Option<f64>,
    yaw: Option<f64>,
    width: u32,
    height: u32,
    missing: bool,
}

#[derive(Serialize)]
struct ProjectView {
    dir: String,
    name: String,
    photos: Vec<PhotoView>,
    settings: Settings,
    manifest: Option<Manifest>,
    levels: Vec<LevelInfo>,
}

fn view(p: &Project) -> ProjectView {
    let manifest = read_manifest(p).ok();
    let levels = manifest.as_ref().map(levels).unwrap_or_default();
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
        manifest,
        levels,
    }
}

fn set_current(state: &AppState, app: &AppHandle, dir: &Path) {
    *state.current.lock().unwrap() = Some(dir.to_path_buf());
    remember_recent(app, dir);
}

#[tauri::command]
fn create_project(app: AppHandle, state: State<AppState>, dir: String, name: String) -> CmdResult<ProjectView> {
    let dir = PathBuf::from(dir);
    let p = Project::create(&dir, &name).map_err(anyerr)?;
    set_current(&state, &app, &dir);
    Ok(view(&p))
}

#[tauri::command]
fn open_project(app: AppHandle, state: State<AppState>, dir: String) -> CmdResult<ProjectView> {
    let dir = PathBuf::from(dir);
    let mut p = Project::open(&dir).map_err(anyerr)?;
    p.refresh().map_err(anyerr)?;
    set_current(&state, &app, &dir);
    Ok(view(&p))
}

#[derive(Serialize)]
struct AddResult {
    added: usize,
    failed: Vec<(String, String)>,
    project: ProjectView,
}

#[tauri::command]
async fn add_photos(dir: String, paths: Vec<String>) -> CmdResult<AddResult> {
    // Reading EXIF from thousands of files takes a moment; keep it off the UI thread.
    tauri::async_runtime::spawn_blocking(move || {
        let mut p = Project::open(Path::new(&dir)).map_err(anyerr)?;
        let paths: Vec<PathBuf> = paths.into_iter().map(PathBuf::from).collect();
        let (added, failed) = p.add_photos(&paths).map_err(anyerr)?;
        Ok(AddResult {
            added,
            failed: failed.into_iter().map(|(p, e)| (p.to_string_lossy().into_owned(), e)).collect(),
            project: view(&p),
        })
    })
    .await
    .map_err(err)?
}

#[tauri::command]
fn remove_photos(dir: String, ids: Vec<u32>) -> CmdResult<ProjectView> {
    let mut p = Project::open(Path::new(&dir)).map_err(anyerr)?;
    p.remove_photos(&ids).map_err(anyerr)?;
    Ok(view(&p))
}

#[tauri::command]
fn save_settings(dir: String, settings: Settings) -> CmdResult<ProjectView> {
    let mut p = Project::open(Path::new(&dir)).map_err(anyerr)?;
    p.file.settings = settings;
    p.save().map_err(anyerr)?;
    Ok(view(&p))
}

#[derive(Clone, Serialize)]
struct JobFinished {
    kind: &'static str,
    ok: bool,
    message: String,
    detail: serde_json::Value,
}

fn start_job(state: &AppState) -> CmdResult<Arc<AtomicBool>> {
    let mut job = state.job.lock().unwrap();
    if job.is_some() {
        return Err("another job is already running".into());
    }
    let flag = Arc::new(AtomicBool::new(false));
    *job = Some(flag.clone());
    Ok(flag)
}

fn finish_job(app: &AppHandle, done: JobFinished) {
    *app.state::<AppState>().job.lock().unwrap() = None;
    let _ = app.emit("job-finished", done);
}

fn ctx_for(app: &AppHandle, cancel: Arc<AtomicBool>) -> Ctx {
    let app = app.clone();
    // Throttle: rendering can report thousands of times a second.
    let last = Mutex::new(std::time::Instant::now() - std::time::Duration::from_secs(1));
    Ctx::new(
        move |p| {
            let mut l = last.lock().unwrap();
            if p.done == p.total || p.done == 0 || l.elapsed().as_millis() >= 100 {
                *l = std::time::Instant::now();
                let _ = app.emit("progress", p);
            }
        },
        cancel,
    )
}

#[tauri::command]
fn process(app: AppHandle, state: State<AppState>, dir: String) -> CmdResult<()> {
    let cancel = start_job(&state)?;
    let ctx = ctx_for(&app, cancel.clone());
    std::thread::spawn(move || {
        let result = Project::open(Path::new(&dir)).and_then(|p| stitch_core::pipeline::process(&p, &ctx));
        let done = match result {
            Ok(s) => JobFinished {
                kind: "process",
                ok: true,
                message: format!("Placed {} of {} photos", s.placed, s.photos),
                detail: serde_json::to_value(&s).unwrap_or_default(),
            },
            Err(e) => JobFinished {
                kind: "process",
                ok: false,
                message: if cancel.load(Ordering::Relaxed) { "Cancelled".into() } else { format!("{e:#}") },
                detail: serde_json::Value::Null,
            },
        };
        finish_job(&app, done);
    });
    Ok(())
}

#[tauri::command]
fn export_map(
    app: AppHandle,
    state: State<AppState>,
    dir: String,
    format: Format,
    zoom: Option<u32>,
    path: String,
) -> CmdResult<()> {
    let cancel = start_job(&state)?;
    let ctx = ctx_for(&app, cancel.clone());
    std::thread::spawn(move || {
        let result = Project::open(Path::new(&dir))
            .and_then(|p| export(&p, &ExportOptions { format, zoom, path: PathBuf::from(&path) }, &ctx));
        let done = match result {
            Ok(out) => JobFinished {
                kind: "export",
                ok: true,
                message: format!("Saved {}", out.display()),
                detail: serde_json::json!({ "path": out }),
            },
            Err(e) => JobFinished {
                kind: "export",
                ok: false,
                message: if cancel.load(Ordering::Relaxed) { "Cancelled".into() } else { format!("{e:#}") },
                detail: serde_json::Value::Null,
            },
        };
        finish_job(&app, done);
    });
    Ok(())
}

#[tauri::command]
fn cancel_job(state: State<AppState>) {
    if let Some(f) = state.job.lock().unwrap().as_ref() {
        f.store(true, Ordering::Relaxed);
    }
}

#[tauri::command]
fn set_profile(dir: String, profile: Option<ProfileKind>) -> CmdResult<ProjectView> {
    let mut p = Project::open(Path::new(&dir)).map_err(anyerr)?;
    p.file.settings.profile = profile;
    p.save().map_err(anyerr)?;
    Ok(view(&p))
}

fn recent_file(app: &AppHandle) -> Option<PathBuf> {
    app.path().app_config_dir().ok().map(|d| d.join("recent.json"))
}

fn read_recent(app: &AppHandle) -> Vec<String> {
    recent_file(app)
        .and_then(|f| std::fs::read(f).ok())
        .and_then(|b| serde_json::from_slice(&b).ok())
        .unwrap_or_default()
}

fn remember_recent(app: &AppHandle, dir: &Path) {
    let Some(file) = recent_file(app) else { return };
    let d = dir.to_string_lossy().into_owned();
    let mut list = read_recent(app);
    list.retain(|x| x != &d);
    list.insert(0, d);
    list.truncate(12);
    if let Some(parent) = file.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let _ = std::fs::write(file, serde_json::to_vec(&list).unwrap_or_default());
}

#[derive(Serialize)]
struct RecentProject {
    dir: String,
    name: String,
    photos: usize,
}

#[tauri::command]
fn recent_projects(app: AppHandle) -> Vec<RecentProject> {
    read_recent(&app)
        .into_iter()
        .filter_map(|d| {
            let p = Project::open(Path::new(&d)).ok()?;
            Some(RecentProject { dir: d, name: p.file.name.clone(), photos: p.file.photos.len() })
        })
        .collect()
}

/// `tiles://localhost/{z}/{x}/{y}.jpg` → `<project>/result/tiles/{z}/{x}/{y}.jpg`.
fn serve_tile(app: &AppHandle, uri_path: &str) -> tauri::http::Response<Vec<u8>> {
    let not_found = || tauri::http::Response::builder().status(404).body(Vec::new()).unwrap();
    let Some(dir) = app.state::<AppState>().current.lock().unwrap().clone() else { return not_found() };
    let parts: Vec<&str> = uri_path.trim_start_matches('/').split('/').collect();
    let [z, x, y] = parts[..] else { return not_found() };
    let (Ok(z), Ok(x)) = (z.parse::<u32>(), x.parse::<usize>()) else { return not_found() };
    let Ok(y) = y.trim_end_matches(".jpg").parse::<usize>() else { return not_found() };
    let path = stitch_core::render::tile_path(&dir.join("result").join("tiles"), z, x, y);
    match std::fs::read(path) {
        Ok(bytes) => tauri::http::Response::builder()
            .header("Content-Type", "image/jpeg")
            .header("Cache-Control", "max-age=31536000")
            .body(bytes)
            .unwrap(),
        Err(_) => not_found(),
    }
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(AppState::default())
        .register_uri_scheme_protocol("tiles", |ctx, req| serve_tile(ctx.app_handle(), req.uri().path()))
        .invoke_handler(tauri::generate_handler![
            system_info,
            create_project,
            open_project,
            add_photos,
            remove_photos,
            save_settings,
            set_profile,
            process,
            export_map,
            cancel_job,
            recent_projects,
        ])
        .run(tauri::generate_context!())
        .expect("error while running Aerial Stitch");
}
