//! 3D reconstruction: sparse (cameras + tie points), then dense point cloud.
//! Uses every photo, angled ones included — they are what give walls and
//! tree sides their shape.

pub mod ba;
pub mod dense;
pub mod geometry;
pub mod ply;
pub mod sparse;

use crate::project::Project;
use crate::Ctx;
use anyhow::Result;
use std::path::PathBuf;

pub fn out_dir(project: &Project) -> PathBuf {
    project.result_dir().join("3d")
}

/// Runs the sparse stage and writes `cameras.json` + `sparse.ply`.
pub fn run_sparse(project: &Project, ctx: &Ctx) -> Result<sparse::Sparse> {
    let s = sparse::reconstruct_sparse(project, ctx)?;
    let dir = out_dir(project);
    std::fs::create_dir_all(&dir)?;
    std::fs::write(dir.join("cameras.json"), serde_json::to_vec(&s)?)?;
    let pts: Vec<[f32; 3]> = s.points.iter().map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]).collect();
    ply::write_ply(&dir.join("sparse.ply"), &pts, &s.colors, &origin_comment(&s))?;
    Ok(s)
}

pub fn origin_comment(s: &sparse::Sparse) -> String {
    format!(
        "origin EPSG:{} E {} N {} height {}",
        s.zone.epsg(),
        s.origin_e,
        s.origin_n,
        s.height_ref
    )
}

/// Loads the sparse result written by [`run_sparse`].
pub fn load_sparse(project: &Project) -> Result<sparse::Sparse> {
    Ok(serde_json::from_slice(&std::fs::read(out_dir(project).join("cameras.json"))?)?)
}

/// Dense point cloud from an existing sparse result; writes `dense.ply`.
pub fn run_dense(project: &Project, ctx: &Ctx) -> Result<(usize, f64)> {
    let s = load_sparse(project)?;
    let profile = crate::pipeline::effective_profile(project);
    let pool = rayon::ThreadPoolBuilder::new().num_threads(profile.threads).build()?;
    let cloud = pool.install(|| dense::reconstruct_dense(project, &s, &profile, ctx))?;
    ply::write_ply(&out_dir(project).join("dense.ply"), &cloud.points, &cloud.colors, &origin_comment(&s))?;
    Ok((cloud.points.len(), cloud.voxel_m))
}
