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
use stitch_core::export::Format;
use stitch_core::project::Settings;
use stitch_core::plan::{Camera, Plan, PlanInput};
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

use stitch_core::appapi::{self, AddResult, JobFinished, ProjectView};

fn set_current(state: &AppState, app: &AppHandle, dir: &Path) {
    *state.current.lock().unwrap() = Some(dir.to_path_buf());
    remember_recent(app, dir);
}

#[tauri::command]
fn create_project(app: AppHandle, state: State<AppState>, dir: String, name: String) -> CmdResult<ProjectView> {
    let dir = PathBuf::from(dir);
    let v = appapi::create_project(&dir, &name).map_err(anyerr)?;
    set_current(&state, &app, &dir);
    Ok(v)
}

#[tauri::command]
fn open_project(app: AppHandle, state: State<AppState>, dir: String) -> CmdResult<ProjectView> {
    let dir = PathBuf::from(dir);
    let v = appapi::open_project(&dir).map_err(anyerr)?;
    set_current(&state, &app, &dir);
    Ok(v)
}

#[tauri::command]
async fn add_photos(dir: String, paths: Vec<String>) -> CmdResult<AddResult> {
    // Reading EXIF from thousands of files takes a moment; keep it off the UI thread.
    tauri::async_runtime::spawn_blocking(move || {
        let paths: Vec<PathBuf> = paths.into_iter().map(PathBuf::from).collect();
        appapi::add_photos(Path::new(&dir), &paths).map_err(anyerr)
    })
    .await
    .map_err(err)?
}

#[tauri::command]
fn remove_photos(dir: String, ids: Vec<u32>) -> CmdResult<ProjectView> {
    appapi::remove_photos(Path::new(&dir), &ids).map_err(anyerr)
}

#[tauri::command]
fn save_settings(dir: String, settings: Settings) -> CmdResult<ProjectView> {
    appapi::save_settings(Path::new(&dir), settings).map_err(anyerr)
}

#[tauri::command]
fn set_profile(dir: String, profile: Option<ProfileKind>) -> CmdResult<ProjectView> {
    appapi::set_profile(Path::new(&dir), profile).map_err(anyerr)
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

/// Runs a long job on its own thread; the UI hears about it through events.
fn spawn_job(app: AppHandle, state: &AppState, job: impl FnOnce(&Ctx) -> JobFinished + Send + 'static) -> CmdResult<()> {
    let cancel = start_job(state)?;
    let ctx = ctx_for(&app, cancel);
    std::thread::spawn(move || {
        let done = job(&ctx);
        *app.state::<AppState>().job.lock().unwrap() = None;
        let _ = app.emit("job-finished", done);
    });
    Ok(())
}

#[tauri::command]
fn process(app: AppHandle, state: State<AppState>, dir: String) -> CmdResult<()> {
    spawn_job(app, &state, move |ctx| appapi::process(Path::new(&dir), ctx))
}

#[tauri::command]
fn export_map(
    app: AppHandle,
    state: State<AppState>,
    dir: String,
    format: Format,
    layer: Option<stitch_core::render::Layer>,
    zoom: Option<u32>,
    path: String,
) -> CmdResult<()> {
    spawn_job(app, &state, move |ctx| appapi::export_map(Path::new(&dir), format, layer, zoom, Path::new(&path), ctx))
}

#[tauri::command]
fn build_3d(app: AppHandle, state: State<AppState>, dir: String) -> CmdResult<()> {
    spawn_job(app, &state, move |ctx| appapi::build_3d(Path::new(&dir), ctx))
}

#[tauri::command]
fn measure_volume(
    dir: String,
    polygon: Vec<(f64, f64)>,
    level: stitch_core::recon::measure::Level,
) -> CmdResult<stitch_core::recon::measure::VolumeResult> {
    appapi::measure_volume(Path::new(&dir), &polygon, level).map_err(anyerr)
}

#[tauri::command]
fn export_model(dir: String, path: String) -> CmdResult<String> {
    appapi::export_model(Path::new(&dir), Path::new(&path)).map_err(anyerr)
}

#[tauri::command]
fn export_cloud(dir: String, path: String) -> CmdResult<String> {
    appapi::export_cloud(Path::new(&dir), Path::new(&path)).map_err(anyerr)
}

#[tauri::command]
fn cancel_job(state: State<AppState>) {
    if let Some(f) = state.job.lock().unwrap().as_ref() {
        f.store(true, Ordering::Relaxed);
    }
}

// ─────────────────────────────── Flight planner ───────────────────────────────

#[tauri::command]
fn plan_defaults() -> (PlanInput, Vec<Camera>) {
    appapi::plan_defaults()
}

#[tauri::command]
fn plan_flight(input: PlanInput) -> CmdResult<Plan> {
    appapi::plan_flight(&input).map_err(anyerr)
}

#[tauri::command]
fn export_litchi(input: PlanInput, dir: String) -> CmdResult<Vec<String>> {
    appapi::export_litchi(&input, Path::new(&dir)).map_err(anyerr)
}

#[tauri::command]
fn import_boundary(path: String) -> CmdResult<Vec<(f64, f64)>> {
    appapi::import_boundary(Path::new(&path)).map_err(anyerr)
}

#[tauri::command]
fn save_plan(path: String, input: PlanInput) -> CmdResult<()> {
    appapi::save_plan(Path::new(&path), &input).map_err(anyerr)
}

#[tauri::command]
fn load_plan(path: String) -> CmdResult<PlanInput> {
    appapi::load_plan(Path::new(&path)).map_err(anyerr)
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
    if let ["3d", name] = parts[..] {
        if !matches!(name, "view.ply" | "dense.ply" | "sparse.ply" | "model_view.glb") {
            return not_found();
        }
        return match std::fs::read(dir.join("result").join("3d").join(name)) {
            Ok(bytes) => tauri::http::Response::builder()
                .header("Content-Type", "application/octet-stream")
                .header("Cache-Control", "no-cache")
                .body(bytes)
                .unwrap(),
            Err(_) => not_found(),
        };
    }
    let (base, parts): (PathBuf, Vec<&str>) = match parts.first() {
        Some(&"ortho") => (dir.join("result").join("ortho").join("tiles"), parts[1..].to_vec()),
        _ => (dir.join("result").join("tiles"), parts),
    };
    let [z, x, y] = parts[..] else { return not_found() };
    let (Ok(z), Ok(x)) = (z.parse::<u32>(), x.parse::<usize>()) else { return not_found() };
    let Ok(y) = y.trim_end_matches(".jpg").parse::<usize>() else { return not_found() };
    let path = stitch_core::render::tile_path(&base, z, x, y);
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
            build_3d,
            export_cloud,
            measure_volume,
            export_model,
            recent_projects,
            plan_defaults,
            plan_flight,
            export_litchi,
            import_boundary,
            save_plan,
            load_plan,
        ])
        .run(tauri::generate_context!())
        .expect("error while running Aerial Stitch");
}
