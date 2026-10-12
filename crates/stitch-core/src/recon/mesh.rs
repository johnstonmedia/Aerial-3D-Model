//! Textured surface model (2.5D): a triangle mesh draped over the surface
//! model, written as glTF binary.
//!
//! * **Tops** — ground, roofs, canopy — take their texture from the true
//!   orthophoto, so the model and the map are the same imagery.
//! * **Steep faces** — building sides, tree edges — each get their own patch
//!   in a texture atlas, painted from the photo that faces them most squarely
//!   and can actually see them. With angled (oblique) photos in the survey,
//!   that is real facade imagery rather than a smear of the roof edge.
//!
//! Being a height field, it can't represent overhangs (under eaves, under
//! canopy); walls come out as near-vertical faces between roof and ground.

use super::dsm::Dsm;
use super::geometry::V3;
use super::glb::{encode_jpeg, Glb, Primitive};
use super::ortho::{centredness, CamIndex};
use super::sparse::Sparse;
use crate::imgops::{box_down, load_rgb};
use crate::profile::{Profile, ProfileKind};
use crate::render::{read_tile, Manifest, RasterCache, TILE};
use crate::Ctx;
use anyhow::{bail, Result};
use rayon::prelude::*;
use std::collections::HashMap;
use std::path::Path;

struct Params {
    max_vertices: usize,
    /// Texels budget for the top (orthophoto) texture in the exported model.
    top_texels: usize,
    /// Pixel budget for the wall atlas.
    wall_pixels: usize,
    /// Wall texel size relative to the orthophoto's ground sampling.
    wall_mult: f64,
}

fn params(p: &Profile) -> Params {
    match p.kind {
        ProfileKind::Lite => Params {
            max_vertices: 250_000,
            top_texels: 16 << 20,
            wall_pixels: 8 << 20,
            wall_mult: 3.0,
        },
        ProfileKind::Standard => Params {
            max_vertices: 600_000,
            top_texels: 48 << 20,
            wall_pixels: 24 << 20,
            wall_mult: 2.0,
        },
        ProfileKind::High => Params {
            max_vertices: 1_500_000,
            top_texels: 128 << 20,
            wall_pixels: 64 << 20,
            wall_mult: 1.5,
        },
    }
}

/// Texels in the in-app viewer's copy (integrated GPUs, quick loading).
const VIEW_TEXELS: usize = 10 << 20;
const PAGE: usize = 2048;
const CHUNK: usize = 4096;
const PAD: usize = 32;
/// Faces steeper than ~53° are textured as walls.
const STEEP_NZ: f64 = 0.6;

/// Is the line from `start` to `c` clear of the surface?
fn clear(dsm: &Dsm, start: &V3, c: &V3) -> bool {
    let d = c - start;
    let len = d.norm();
    if len < 1e-6 {
        return true;
    }
    let dir = d / len;
    let step = dsm.cell * 0.8;
    let mut t = step;
    for _ in 0..4000 {
        let q = start + dir * t;
        if q.z > dsm.zmax as f64 + 1.0 || t >= len {
            return true;
        }
        if let Some(h) = dsm.height(q.x, q.y) {
            if h as f64 > q.z + 0.5 {
                return false;
            }
        }
        t += step;
    }
    true
}

struct Tex {
    w: usize,
    h: usize,
    rgb: Vec<u8>,
}

impl Tex {
    fn down(&self, f: usize) -> Tex {
        if f <= 1 {
            return Tex {
                w: self.w,
                h: self.h,
                rgb: self.rgb.clone(),
            };
        }
        let r = crate::imgops::Rgb {
            w: self.w,
            h: self.h,
            data: self.rgb.clone(),
        };
        let d = box_down(&r, f);
        Tex {
            w: d.w,
            h: d.h,
            rgb: d.data,
        }
    }
}

pub struct MeshStats {
    pub vertices: usize,
    pub triangles: usize,
    pub wall_triangles: usize,
    pub walls_from_oblique: usize,
}

pub fn build_mesh(
    out_dir: &Path,
    ortho_root: &Path,
    ortho: &Manifest,
    sp: &Sparse,
    dsm: &Dsm,
    profile: &Profile,
    ctx: &Ctx,
) -> Result<MeshStats> {
    let prm = params(profile);
    ctx.report("mesh", 0, 3, "Building the surface mesh");

    // ── Grid of vertices (strided DSM cells) ──
    let valid = dsm.h.iter().filter(|v| v.is_finite()).count();
    if valid < 16 {
        bail!("no surface to mesh");
    }
    let s = ((valid as f64 / prm.max_vertices as f64).sqrt().ceil() as usize).max(1);
    let gnx = dsm.nx.div_ceil(s);
    let gny = dsm.ny.div_ceil(s);
    let node = |gi: usize, gj: usize| -> Option<V3> {
        let (i, j) = ((gi * s).min(dsm.nx - 1), (gj * s).min(dsm.ny - 1));
        let z = dsm.at(i, j);
        z.is_finite().then(|| {
            V3::new(
                dsm.x0 + (i as f64 + 0.5) * dsm.cell,
                dsm.y0 + (j as f64 + 0.5) * dsm.cell,
                z as f64,
            )
        })
    };
    let verts: Vec<Option<V3>> = (0..gny)
        .flat_map(|gj| (0..gnx).map(move |gi| (gi, gj)))
        .map(|(gi, gj)| node(gi, gj))
        .collect();
    let mut tris: Vec<[usize; 3]> = Vec::new();
    for gj in 0..gny.saturating_sub(1) {
        for gi in 0..gnx.saturating_sub(1) {
            let (a, b, c, d) = (
                gj * gnx + gi,
                gj * gnx + gi + 1,
                (gj + 1) * gnx + gi + 1,
                (gj + 1) * gnx + gi,
            );
            let (Some(pa), Some(pb), Some(pc), Some(pd)) = (verts[a], verts[b], verts[c], verts[d])
            else {
                continue;
            };
            // Split along the shorter diagonal: keeps roof edges sharp.
            if (pa - pc).norm() <= (pb - pd).norm() {
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            } else {
                tris.push([a, b, d]);
                tris.push([b, c, d]);
            }
        }
    }
    if tris.is_empty() {
        bail!("the surface is too fragmented to mesh");
    }
    let normal = |t: &[usize; 3]| {
        let (a, b, c) = (
            verts[t[0]].unwrap(),
            verts[t[1]].unwrap(),
            verts[t[2]].unwrap(),
        );
        (b - a).cross(&(c - a)).normalize()
    };

    // ── Wall faces: best photo that faces them and can see them ──
    let idx = CamIndex::new(sp, dsm);
    let steep: Vec<usize> = (0..tris.len())
        .filter(|&k| normal(&tris[k]).z < STEEP_NZ)
        .collect();
    ctx.report(
        "mesh",
        1,
        3,
        format!("Choosing photos for {} wall faces", steep.len()),
    );
    let wall_cam: Vec<Option<u32>> = steep
        .par_iter()
        .map(|&k| {
            let t = &tris[k];
            let n = normal(t);
            let g = (verts[t[0]].unwrap() + verts[t[1]].unwrap() + verts[t[2]].unwrap()) / 3.0;
            let mut cands: Vec<(f64, u32)> = idx
                .near(g.x, g.y)
                .iter()
                .filter_map(|&ci| {
                    let cam = &sp.cameras[ci as usize];
                    let kk = &sp.intrinsics[cam.group];
                    let u = cam.pose.project(kk, &g)?;
                    let c = centredness(&u, kk.width as f64, kk.height as f64);
                    if c < 0.05 {
                        return None;
                    }
                    let facing = n.dot(&(cam.pose.c - g).normalize());
                    (facing > 0.15).then(|| (facing * facing * c.powi(4), ci))
                })
                .collect();
            cands.sort_by(|a, b| b.0.total_cmp(&a.0));
            let start = g + n * 0.4 + V3::new(0.0, 0.0, 0.2);
            for &(_, ci) in cands.iter().take(6) {
                if clear(dsm, &start, &sp.cameras[ci as usize].pose.c) {
                    return Some(ci);
                }
            }
            cands.first().map(|c| c.1)
        })
        .collect();
    let mut is_wall = vec![false; tris.len()];
    let mut walls: Vec<(usize, u32)> = Vec::new();
    for (i, &k) in steep.iter().enumerate() {
        if let Some(c) = wall_cam[i] {
            is_wall[k] = true;
            walls.push((k, c));
        }
    }
    let walls_from_oblique = walls
        .iter()
        .filter(|(_, c)| sp.cameras[*c as usize].oblique)
        .count();

    // ── Top texture: orthophoto at the finest level within budget ──
    let mut zt = ortho.max_zoom;
    loop {
        let f = 1u64 << (ortho.max_zoom - zt);
        let (w, h) = (ortho.width.div_ceil(f), ortho.height.div_ceil(f));
        if (w * h) as usize <= prm.top_texels || zt == 0 {
            break;
        }
        zt -= 1;
    }
    let fz = (1u64 << (ortho.max_zoom - zt)) as f64;
    let gsd_z = ortho.gsd * fz;
    let (wz, hz) = (
        (ortho.width as f64 / fz).ceil() as usize,
        (ortho.height as f64 / fz).ceil() as usize,
    );
    let cnx = wz.div_ceil(CHUNK);
    let cny = hz.div_ceil(CHUNK);
    let to_px = |p: &V3| ((p.x - ortho.left) / gsd_z, (ortho.top - p.y) / gsd_z);

    // Assign top triangles to chunks by centroid.
    let mut chunk_tris: Vec<Vec<usize>> = vec![Vec::new(); cnx * cny];
    for (k, t) in tris.iter().enumerate() {
        if is_wall[k] {
            continue;
        }
        let g = (verts[t[0]].unwrap() + verts[t[1]].unwrap() + verts[t[2]].unwrap()) / 3.0;
        let (px, py) = to_px(&g);
        let cx = ((px / CHUNK as f64).floor().max(0.0) as usize).min(cnx - 1);
        let cy = ((py / CHUNK as f64).floor().max(0.0) as usize).min(cny - 1);
        chunk_tris[cy * cnx + cx].push(k);
    }
    let mut textures: Vec<Tex> = Vec::new();
    let mut prims: Vec<(Vec<V3>, Vec<[f32; 2]>, Vec<u32>, usize)> = Vec::new();
    for cy in 0..cny {
        for cx in 0..cnx {
            let list = &chunk_tris[cy * cnx + cx];
            if list.is_empty() {
                continue;
            }
            ctx.check()?;
            // Texture covers the chunk plus a margin for straddling triangles.
            let x0 = (cx * CHUNK) as i64 - PAD as i64;
            let y0 = (cy * CHUNK) as i64 - PAD as i64;
            let tw = (CHUNK + 2 * PAD)
                .min(wz + PAD - (cx * CHUNK).min(wz))
                .max(1);
            let th = (CHUNK + 2 * PAD)
                .min(hz + PAD - (cy * CHUNK).min(hz))
                .max(1);
            let mut rgb = vec![0u8; tw * th * 3];
            let (tx0, ty0) = ((x0.max(0) as usize) / TILE, (y0.max(0) as usize) / TILE);
            let (tx1, ty1) = (
                ((x0 + tw as i64).max(0) as usize) / TILE,
                ((y0 + th as i64).max(0) as usize) / TILE,
            );
            for ty in ty0..=ty1 {
                for tx in tx0..=tx1 {
                    let Some(tile) = read_tile(ortho_root, zt, tx, ty)? else {
                        continue;
                    };
                    for yy in 0..TILE {
                        let gy = (ty * TILE + yy) as i64 - y0;
                        if gy < 0 || gy >= th as i64 {
                            continue;
                        }
                        for xx in 0..TILE {
                            let gx = (tx * TILE + xx) as i64 - x0;
                            if gx < 0 || gx >= tw as i64 {
                                continue;
                            }
                            let s = (yy * TILE + xx) * 3;
                            let d = (gy as usize * tw + gx as usize) * 3;
                            rgb[d..d + 3].copy_from_slice(&tile[s..s + 3]);
                        }
                    }
                }
            }
            let tex_i = textures.len();
            textures.push(Tex { w: tw, h: th, rgb });
            let mut remap: HashMap<usize, u32> = HashMap::new();
            let (mut pos, mut uvs, mut ind) = (Vec::new(), Vec::new(), Vec::new());
            for &k in list {
                for &vi in &tris[k] {
                    let id = *remap.entry(vi).or_insert_with(|| {
                        let p = verts[vi].unwrap();
                        let (px, py) = to_px(&p);
                        pos.push(p);
                        uvs.push([
                            ((px - x0 as f64) / tw as f64) as f32,
                            ((py - y0 as f64) / th as f64) as f32,
                        ]);
                        (pos.len() - 1) as u32
                    });
                    ind.push(id);
                }
            }
            prims.push((pos, uvs, ind, tex_i));
        }
    }

    // ── Wall atlas ──
    ctx.report("mesh", 2, 3, "Painting building sides from the photos");
    let mut wt = ortho.gsd * prm.wall_mult;
    // Local 2D layout of each wall triangle in its own plane.
    struct Patch {
        k: usize,
        cam: u32,
        a: V3,
        e1: V3,
        e2: V3,
        min: (f64, f64),
        loc: [(f64, f64); 3],
        w: usize,
        h: usize,
        page: usize,
        x: usize,
        y: usize,
    }
    let layout = |wt: f64| -> (Vec<Patch>, usize) {
        let mut v: Vec<Patch> = walls
            .iter()
            .map(|&(k, cam)| {
                let t = &tris[k];
                let (a, b, c) = (
                    verts[t[0]].unwrap(),
                    verts[t[1]].unwrap(),
                    verts[t[2]].unwrap(),
                );
                let n = (b - a).cross(&(c - a)).normalize();
                let e1 = (b - a).normalize();
                let e2 = n.cross(&e1);
                let loc = [
                    (0.0, 0.0),
                    ((b - a).dot(&e1), (b - a).dot(&e2)),
                    ((c - a).dot(&e1), (c - a).dot(&e2)),
                ];
                let minx = loc.iter().map(|p| p.0).fold(f64::MAX, f64::min);
                let maxx = loc.iter().map(|p| p.0).fold(f64::MIN, f64::max);
                let miny = loc.iter().map(|p| p.1).fold(f64::MAX, f64::min);
                let maxy = loc.iter().map(|p| p.1).fold(f64::MIN, f64::max);
                let w = (((maxx - minx) / wt).ceil() as usize + 3).min(PAGE);
                let h = (((maxy - miny) / wt).ceil() as usize + 3).min(PAGE);
                Patch {
                    k,
                    cam,
                    a,
                    e1,
                    e2,
                    min: (minx, miny),
                    loc,
                    w,
                    h,
                    page: 0,
                    x: 0,
                    y: 0,
                }
            })
            .collect();
        let px: usize = v.iter().map(|p| p.w * p.h).sum();
        // Shelf packing, tallest first.
        v.sort_by(|a, b| b.h.cmp(&a.h));
        let (mut page, mut x, mut y, mut shelf) = (0, 0, 0, 0);
        for p in v.iter_mut() {
            if x + p.w > PAGE {
                x = 0;
                y += shelf;
                shelf = 0;
            }
            if y + p.h > PAGE {
                page += 1;
                x = 0;
                y = 0;
                shelf = 0;
            }
            p.page = page;
            p.x = x;
            p.y = y;
            x += p.w;
            shelf = shelf.max(p.h);
        }
        (v, px)
    };
    let (mut patches, px) = layout(wt);
    if px > prm.wall_pixels {
        wt *= (px as f64 / prm.wall_pixels as f64).sqrt();
        patches = layout(wt).0;
    }
    let pages = patches.iter().map(|p| p.page + 1).max().unwrap_or(0);
    let gsds: Vec<f64> = sp
        .cameras
        .iter()
        .map(|c| ((c.depth_range.0 + c.depth_range.1) / 2.0) / sp.intrinsics[c.group].f)
        .collect();
    let factor: Vec<usize> = gsds
        .iter()
        .map(|g| {
            let mut f = 1;
            while (f * 2) as f64 <= wt / g {
                f *= 2;
            }
            f
        })
        .collect();
    let cache = RasterCache::new(profile.render_cache_bytes);
    // Paint patches grouped by camera so each photo is decoded once.
    patches.sort_by_key(|p| p.cam);
    let painted: Vec<Vec<u8>> = patches
        .par_chunk_by(|a, b| a.cam == b.cam)
        .flat_map_iter(|group| {
            let ci = group[0].cam as usize;
            let f = factor[ci];
            let img = cache.get(ci, || {
                let i = load_rgb(&sp.cameras[ci].path).ok()?;
                Some(if f > 1 { box_down(&i, f) } else { i })
            });
            let cam = &sp.cameras[ci];
            let kk = &sp.intrinsics[cam.group];
            group
                .iter()
                .map(|p| {
                    let mut buf = vec![110u8; p.w * p.h * 3];
                    if let Some(img) = &img {
                        for yy in 0..p.h {
                            for xx in 0..p.w {
                                let u = p.min.0 + (xx as f64 - 1.0 + 0.5) * wt;
                                let v = p.min.1 + (yy as f64 - 1.0 + 0.5) * wt;
                                let q = p.a + p.e1 * u + p.e2 * v;
                                if let Some(uv) = cam.pose.project(kk, &q) {
                                    if let Some(c) = img.sample(uv.x / f as f64, uv.y / f as f64) {
                                        let o = (yy * p.w + xx) * 3;
                                        buf[o] = c[0] as u8;
                                        buf[o + 1] = c[1] as u8;
                                        buf[o + 2] = c[2] as u8;
                                    }
                                }
                            }
                        }
                    }
                    buf
                })
                .collect::<Vec<_>>()
        })
        .collect();
    drop(cache);
    let first_wall_tex = textures.len();
    for _ in 0..pages {
        textures.push(Tex {
            w: PAGE,
            h: PAGE,
            rgb: vec![110u8; PAGE * PAGE * 3],
        });
    }
    let mut wall_prims: Vec<(Vec<V3>, Vec<[f32; 2]>, Vec<u32>)> =
        vec![(Vec::new(), Vec::new(), Vec::new()); pages];
    for (p, buf) in patches.iter().zip(&painted) {
        let tex = &mut textures[first_wall_tex + p.page];
        for yy in 0..p.h {
            let d = ((p.y + yy) * PAGE + p.x) * 3;
            tex.rgb[d..d + p.w * 3].copy_from_slice(&buf[yy * p.w * 3..(yy + 1) * p.w * 3]);
        }
        let (pos, uvs, ind) = &mut wall_prims[p.page];
        let t = &tris[p.k];
        for (i, &vi) in t.iter().enumerate() {
            let (lu, lv) = p.loc[i];
            ind.push(pos.len() as u32);
            pos.push(verts[vi].unwrap());
            uvs.push([
                ((p.x as f64 + 1.0 + (lu - p.min.0) / wt) / PAGE as f64) as f32,
                ((p.y as f64 + 1.0 + (lv - p.min.1) / wt) / PAGE as f64) as f32,
            ]);
        }
    }
    for (i, (pos, uvs, ind)) in wall_prims.into_iter().enumerate() {
        prims.push((pos, uvs, ind, first_wall_tex + i));
    }

    // ── Write the export model and the lighter viewer copy ──
    let mut zs: Vec<f32> = dsm.h.iter().copied().filter(|v| v.is_finite()).collect();
    zs.sort_by(f32::total_cmp);
    let off = V3::new(
        ortho.left + ortho.width as f64 * ortho.gsd / 2.0,
        ortho.top - ortho.height as f64 * ortho.gsd / 2.0,
        zs[zs.len() / 2] as f64,
    );
    let extras = serde_json::json!({
        "epsg": sp.zone.epsg(),
        "utm_origin": [sp.origin_e + off.x, sp.origin_n + off.y],
        "height_offset": off.z,
        "height_reference": sp.height_ref,
        // Project-frame (local ENU) offset: frame = (x + o0, −z + o1, y + o2).
        "local_offset": [off.x, off.y, off.z],
        "axes": "x east, y up, z south (glTF); add utm_origin to x and -z for UTM",
    });
    let to_gltf = |p: &V3| {
        [
            (p.x - off.x) as f32,
            (p.z - off.z) as f32,
            -(p.y - off.y) as f32,
        ]
    };
    let total_texels: usize = textures.iter().map(|t| t.w * t.h).sum();
    let view_f = {
        let mut f = 1;
        while total_texels / (f * f) > VIEW_TEXELS {
            f *= 2;
        }
        f
    };
    let write = |path: &Path, f: usize, q: u8| -> Result<()> {
        let mut glb = Glb {
            extras: extras.clone(),
            ..Default::default()
        };
        for t in &textures {
            let d = t.down(f);
            glb.textures.push(encode_jpeg(&d.rgb, d.w, d.h, q)?);
        }
        for (pos, uvs, ind, tex) in &prims {
            glb.primitives.push(Primitive {
                positions: pos.iter().map(to_gltf).collect(),
                uvs: uvs.clone(),
                indices: ind.clone(),
                texture: *tex,
            });
        }
        glb.write(path)
    };
    write(&out_dir.join("model.glb"), 1, 88)?;
    write(&out_dir.join("model_view.glb"), view_f, 82)?;
    ctx.report("mesh", 3, 3, "Surface model ready");
    Ok(MeshStats {
        vertices: verts.iter().filter(|v| v.is_some()).count(),
        triangles: tris.len(),
        wall_triangles: walls.len(),
        walls_from_oblique,
    })
}
