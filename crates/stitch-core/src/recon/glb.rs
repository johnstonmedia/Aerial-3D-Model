//! Minimal glTF 2.0 binary (.glb) writer: textured triangle meshes with
//! embedded JPEG textures and unlit materials (the photos already carry the
//! real lighting). Opens in Blender, Windows 3D Viewer, three.js and most
//! web 3D tools.

use anyhow::Result;
use serde_json::{json, Value};
use std::io::Write;
use std::path::Path;

pub struct Primitive {
    /// glTF axes (y up), already recentred.
    pub positions: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
    /// Index into `Glb::textures`.
    pub texture: usize,
}

#[derive(Default)]
pub struct Glb {
    pub primitives: Vec<Primitive>,
    /// JPEG-encoded images.
    pub textures: Vec<Vec<u8>>,
    pub extras: Value,
}

fn pad4(v: &mut Vec<u8>, byte: u8) {
    while v.len() % 4 != 0 {
        v.push(byte);
    }
}

impl Glb {
    pub fn write(&self, path: &Path) -> Result<()> {
        let mut bin: Vec<u8> = Vec::new();
        let mut views = Vec::new();
        let mut accessors = Vec::new();
        let mut view = |bin: &mut Vec<u8>, data: &[u8], target: Option<u32>| -> usize {
            pad4(bin, 0);
            let off = bin.len();
            bin.extend_from_slice(data);
            let mut v = json!({ "buffer": 0, "byteOffset": off, "byteLength": data.len() });
            if let Some(t) = target {
                v["target"] = json!(t);
            }
            views.push(v);
            views.len() - 1
        };
        let mut prims = Vec::new();
        for p in &self.primitives {
            if p.indices.is_empty() {
                continue;
            }
            let mut lo = [f32::MAX; 3];
            let mut hi = [f32::MIN; 3];
            let mut buf = Vec::with_capacity(p.positions.len() * 12);
            for v in &p.positions {
                for k in 0..3 {
                    lo[k] = lo[k].min(v[k]);
                    hi[k] = hi[k].max(v[k]);
                    buf.extend_from_slice(&v[k].to_le_bytes());
                }
            }
            let pv = view(&mut bin, &buf, Some(34962));
            accessors.push(json!({ "bufferView": pv, "componentType": 5126, "count": p.positions.len(),
                                   "type": "VEC3", "min": lo, "max": hi }));
            let pa = accessors.len() - 1;
            let buf: Vec<u8> = p.uvs.iter().flat_map(|uv| [uv[0].to_le_bytes(), uv[1].to_le_bytes()].concat()).collect();
            let uvv = view(&mut bin, &buf, Some(34962));
            accessors.push(json!({ "bufferView": uvv, "componentType": 5126, "count": p.uvs.len(), "type": "VEC2" }));
            let ua = accessors.len() - 1;
            let buf: Vec<u8> = p.indices.iter().flat_map(|i| i.to_le_bytes()).collect();
            let iv = view(&mut bin, &buf, Some(34963));
            accessors.push(json!({ "bufferView": iv, "componentType": 5125, "count": p.indices.len(), "type": "SCALAR" }));
            let ia = accessors.len() - 1;
            prims.push(json!({ "attributes": { "POSITION": pa, "TEXCOORD_0": ua }, "indices": ia, "material": p.texture }));
        }
        let mut images = Vec::new();
        for t in &self.textures {
            let v = view(&mut bin, t, None);
            images.push(json!({ "bufferView": v, "mimeType": "image/jpeg" }));
        }
        let n = self.textures.len();
        let doc = json!({
            "asset": { "version": "2.0", "generator": "Aerial Stitch", "extras": self.extras },
            "extensionsUsed": ["KHR_materials_unlit"],
            "scene": 0,
            "scenes": [{ "nodes": [0] }],
            "nodes": [{ "mesh": 0, "name": "surface" }],
            "meshes": [{ "primitives": prims }],
            "materials": (0..n).map(|i| json!({
                "pbrMetallicRoughness": { "baseColorTexture": { "index": i }, "metallicFactor": 0.0, "roughnessFactor": 1.0 },
                "extensions": { "KHR_materials_unlit": {} },
                "doubleSided": true
            })).collect::<Vec<_>>(),
            "textures": (0..n).map(|i| json!({ "sampler": 0, "source": i })).collect::<Vec<_>>(),
            "samplers": [{ "magFilter": 9729, "minFilter": 9987, "wrapS": 33071, "wrapT": 33071 }],
            "images": images,
            "accessors": accessors,
            "bufferViews": views,
            "buffers": [{ "byteLength": bin.len() }],
        });
        let mut js = serde_json::to_vec(&doc)?;
        pad4(&mut js, b' ');
        pad4(&mut bin, 0);
        let total = 12 + 8 + js.len() + 8 + bin.len();
        let tmp = path.with_extension("glb.tmp");
        {
            let mut f = std::io::BufWriter::new(std::fs::File::create(&tmp)?);
            f.write_all(b"glTF")?;
            f.write_all(&2u32.to_le_bytes())?;
            f.write_all(&(total as u32).to_le_bytes())?;
            f.write_all(&(js.len() as u32).to_le_bytes())?;
            f.write_all(b"JSON")?;
            f.write_all(&js)?;
            f.write_all(&(bin.len() as u32).to_le_bytes())?;
            f.write_all(b"BIN\0")?;
            f.write_all(&bin)?;
            f.flush()?;
        }
        std::fs::rename(&tmp, path)?;
        Ok(())
    }
}

pub fn encode_jpeg(rgb: &[u8], w: usize, h: usize, quality: u8) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, quality).encode(
        rgb,
        w as u32,
        h as u32,
        image::ExtendedColorType::Rgb8,
    )?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_a_valid_glb_header_and_chunks() {
        let tex = encode_jpeg(&[200u8; 4 * 4 * 3], 4, 4, 90).unwrap();
        let g = Glb {
            primitives: vec![Primitive {
                positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
                uvs: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
                indices: vec![0, 1, 2],
                texture: 0,
            }],
            textures: vec![tex],
            extras: json!({ "epsg": 32756 }),
        };
        let dir = std::env::temp_dir().join(format!("glbtest{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("t.glb");
        g.write(&p).unwrap();
        let b = std::fs::read(&p).unwrap();
        assert_eq!(&b[0..4], b"glTF");
        assert_eq!(u32::from_le_bytes(b[8..12].try_into().unwrap()) as usize, b.len());
        let jl = u32::from_le_bytes(b[12..16].try_into().unwrap()) as usize;
        assert_eq!(&b[16..20], b"JSON");
        let doc: Value = serde_json::from_slice(&b[20..20 + jl]).unwrap();
        assert_eq!(doc["accessors"][0]["count"], 3);
        assert_eq!(&b[20 + jl + 4..20 + jl + 8], b"BIN\0");
        std::fs::remove_dir_all(dir).ok();
    }
}
