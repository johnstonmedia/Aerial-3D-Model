//! 3D reconstruction: sparse (cameras + tie points), then dense point cloud.
//! Uses every photo, angled ones included — they are what give walls and
//! tree sides their shape.

pub mod ba;
pub mod dense;
pub mod dsm;
pub mod geometry;
pub mod glb;
pub mod measure;
pub mod mesh;
pub mod ortho;
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
    let pts: Vec<[f32; 3]> = s
        .points
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    ply::write_ply(
        &dir.join("sparse.ply"),
        &pts,
        &s.colors,
        &origin_comment(&s),
    )?;
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

/// Points in the viewer's copy of the dense cloud.
pub const VIEW_MAX_POINTS: usize = 1_500_000;

/// Loads the sparse result written by [`run_sparse`].
pub fn load_sparse(project: &Project) -> Result<sparse::Sparse> {
    Ok(serde_json::from_slice(&std::fs::read(
        out_dir(project).join("cameras.json"),
    )?)?)
}

/// Dense point cloud from an existing sparse result; writes `dense.ply`.
pub fn run_dense(project: &Project, ctx: &Ctx) -> Result<(usize, f64)> {
    let s = load_sparse(project)?;
    let profile = crate::pipeline::effective_profile(project);
    let cloud = crate::with_pool(profile.threads, || {
        dense::reconstruct_dense(project, &s, &profile, ctx)
    })?;
    let dir = out_dir(project);
    ply::write_ply(
        &dir.join("dense.ply"),
        &cloud.points,
        &cloud.colors,
        &origin_comment(&s),
    )?;
    // A lighter copy for the in-app viewer: a few million points is fine to
    // export but slow to load and orbit on an integrated GPU.
    let step = cloud.points.len().div_ceil(VIEW_MAX_POINTS).max(1);
    let vp: Vec<[f32; 3]> = cloud.points.iter().step_by(step).copied().collect();
    let vc: Vec<[u8; 3]> = cloud.colors.iter().step_by(step).copied().collect();
    ply::write_ply(&dir.join("view.ply"), &vp, &vc, &origin_comment(&s))?;
    std::fs::write(dir.join("dense.voxel"), format!("{}", cloud.voxel_m))?;
    Ok((cloud.points.len(), cloud.voxel_m))
}

/// Surface model + true orthophoto from the dense cloud. Writes
/// `result/3d/dsm.bin` and `result/ortho/` (tiles + manifest).
pub fn run_ortho(project: &Project, ctx: &Ctx) -> Result<crate::render::Manifest> {
    let s = load_sparse(project)?;
    let profile = crate::pipeline::effective_profile(project);
    crate::with_pool(profile.threads, || {
        ctx.report("dsm", 0, 1, "Building the surface model");
        let voxel = dense_voxel(project).unwrap_or(0.25);
        let d = dsm::build_dsm(&out_dir(project).join("dense.ply"), voxel, &profile)?;
        d.save(&out_dir(project).join("dsm.bin"))?;
        ortho::render_ortho(project, &s, &d, &profile, ctx)
    })
}

/// The voxel size the dense cloud was fused at (recorded by `run_dense`).
fn dense_voxel(project: &Project) -> Option<f64> {
    std::fs::read_to_string(out_dir(project).join("dense.voxel"))
        .ok()?
        .trim()
        .parse()
        .ok()
}

/// Textured surface model from the DSM + true orthophoto; writes
/// `result/3d/model.glb` (full detail) and `model_view.glb` (for the viewer).
pub fn run_mesh(project: &Project, ctx: &Ctx) -> Result<mesh::MeshStats> {
    let s = load_sparse(project)?;
    let d = dsm::Dsm::load(&out_dir(project).join("dsm.bin"))?;
    let om = crate::render::read_layer_manifest(project, crate::render::Layer::Ortho)?;
    let profile = crate::pipeline::effective_profile(project);
    crate::with_pool(profile.threads, || {
        mesh::build_mesh(
            &out_dir(project),
            &crate::render::layer_tiles_dir(project, crate::render::Layer::Ortho),
            &om,
            &s,
            &d,
            &profile,
            ctx,
        )
    })
}
