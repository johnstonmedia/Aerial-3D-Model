//! A project is a folder:
//!
//! ```text
//! <project>/
//!   project.json            photos, settings
//!   cache/features/*.af     per-photo keypoints (keyed by file fingerprint)
//!   cache/matches/*.json    per-pair matches
//!   result/alignment.json   solved placement
//!   result/manifest.json    tile pyramid description (what the viewer reads)
//!   result/tiles/z/x/y.jpg
//! ```
//!
//! Photos are referenced where they are, not copied — a survey can be tens of
//! gigabytes. Caches are keyed by a fingerprint of each file, so adding photos
//! only computes what is new.

use crate::meta::{read_meta, PhotoMeta};
use crate::profile::ProfileKind;
use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub const PROJECT_FILE: &str = "project.json";
pub const FORMAT_VERSION: u32 = 1;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhotoEntry {
    pub id: u32,
    pub path: PathBuf,
    /// Fingerprint of the file (size + mtime + name); cache key.
    pub key: String,
    pub meta: PhotoMeta,
    #[serde(default = "yes")]
    pub enabled: bool,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Settings {
    /// Output resolution relative to the photos' native ground resolution.
    /// 1.0 = full detail; 0.5 = half (a quarter of the pixels).
    pub output_scale: f64,
    /// None = pick from installed RAM.
    pub profile: Option<ProfileKind>,
    /// How many GPS neighbours each photo is compared against.
    pub neighbours: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Self { output_scale: 1.0, profile: None, neighbours: 10 }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectFile {
    pub format: u32,
    pub name: String,
    pub photos: Vec<PhotoEntry>,
    pub settings: Settings,
    pub next_id: u32,
}

pub struct Project {
    pub dir: PathBuf,
    pub file: ProjectFile,
}

/// FNV-1a over size, mtime and file name. Content hashing a 20 MB photo per
/// open would be slow; this catches edits and replacements in practice.
pub fn fingerprint(path: &Path) -> Result<String> {
    let md = std::fs::metadata(path).with_context(|| format!("reading {}", path.display()))?;
    let mtime = md
        .modified()
        .ok()
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut feed = |bytes: &[u8]| {
        for b in bytes {
            h ^= *b as u64;
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
    };
    feed(&md.len().to_le_bytes());
    feed(&mtime.to_le_bytes());
    feed(path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default().as_bytes());
    Ok(format!("{h:016x}"))
}

pub fn is_photo(path: &Path) -> bool {
    matches!(
        path.extension().map(|e| e.to_string_lossy().to_lowercase()).as_deref(),
        Some("jpg" | "jpeg" | "png" | "tif" | "tiff")
    )
}

impl Project {
    pub fn create(dir: &Path, name: &str) -> Result<Project> {
        std::fs::create_dir_all(dir)?;
        if dir.join(PROJECT_FILE).exists() {
            bail!("a project already exists in {}", dir.display());
        }
        let p = Project {
            dir: dir.to_path_buf(),
            file: ProjectFile {
                format: FORMAT_VERSION,
                name: name.to_string(),
                photos: Vec::new(),
                settings: Settings::default(),
                next_id: 0,
            },
        };
        p.save()?;
        Ok(p)
    }

    pub fn open(dir: &Path) -> Result<Project> {
        let path = dir.join(PROJECT_FILE);
        let text = std::fs::read_to_string(&path)
            .with_context(|| format!("no project at {}", dir.display()))?;
        let file: ProjectFile = serde_json::from_str(&text).context("project.json is damaged")?;
        if file.format > FORMAT_VERSION {
            bail!("this project was made by a newer version of the app");
        }
        Ok(Project { dir: dir.to_path_buf(), file })
    }

    /// Writes atomically (temp file + rename) so a crash can't leave a
    /// half-written project.json.
    pub fn save(&self) -> Result<()> {
        let tmp = self.dir.join("project.json.tmp");
        std::fs::write(&tmp, serde_json::to_vec_pretty(&self.file)?)?;
        std::fs::rename(&tmp, self.dir.join(PROJECT_FILE))?;
        Ok(())
    }

    pub fn cache_dir(&self, kind: &str) -> PathBuf {
        self.dir.join("cache").join(kind)
    }

    pub fn result_dir(&self) -> PathBuf {
        self.dir.join("result")
    }

    /// Expands folders, skips non-photos and duplicates. Returns how many
    /// were added and the paths that couldn't be read.
    pub fn add_photos(&mut self, paths: &[PathBuf]) -> Result<(usize, Vec<(PathBuf, String)>)> {
        let mut files = Vec::new();
        for p in paths {
            if p.is_dir() {
                collect_dir(p, &mut files)?;
            } else if is_photo(p) {
                files.push(p.clone());
            }
        }
        // Photos are referenced where they are, never copied. Canonical paths
        // stop the same file sneaking in twice under different spellings
        // (relative/absolute, a symlinked folder, different case on Windows).
        let canon = |p: &Path| canonical(p);
        let mut files: Vec<PathBuf> = files.iter().map(|f| canon(f)).collect();
        files.sort();
        files.dedup();
        let existing: std::collections::HashSet<PathBuf> =
            self.file.photos.iter().map(|p| canon(&p.path)).collect();
        files.retain(|f| !existing.contains(f));

        use rayon::prelude::*;
        let read: Vec<(PathBuf, Result<(String, PhotoMeta)>)> = files
            .into_par_iter()
            .map(|f| {
                let r = fingerprint(&f).and_then(|k| Ok((k, read_meta(&f)?)));
                (f, r)
            })
            .collect();
        let mut added = 0;
        let mut failed = Vec::new();
        // The same photo copied to another folder has the same fingerprint
        // (name, size, modified time): keep the one already in the project.
        let mut keys: std::collections::HashSet<String> = self.file.photos.iter().map(|p| p.key.clone()).collect();
        for (path, r) in read {
            match r {
                Ok((key, _)) if keys.contains(&key) => {
                    failed.push((path, "skipped: the same photo is already in the project".to_string()));
                }
                Ok((key, meta)) => {
                    keys.insert(key.clone());
                    let id = self.file.next_id;
                    self.file.next_id += 1;
                    self.file.photos.push(PhotoEntry { id, path, key, meta, enabled: true });
                    added += 1;
                }
                Err(e) => failed.push((path, format!("{e:#}"))),
            }
        }
        self.save()?;
        Ok((added, failed))
    }

    pub fn remove_photos(&mut self, ids: &[u32]) -> Result<()> {
        self.file.photos.retain(|p| !ids.contains(&p.id));
        self.save()
    }

    /// Re-fingerprints photos whose files changed on disk (and reports missing ones).
    pub fn refresh(&mut self) -> Result<Vec<PathBuf>> {
        let mut missing = Vec::new();
        for p in &mut self.file.photos {
            match fingerprint(&p.path) {
                Ok(k) if k != p.key => {
                    p.key = k;
                    if let Ok(m) = read_meta(&p.path) {
                        p.meta = m;
                    }
                }
                Ok(_) => {}
                Err(_) => missing.push(p.path.clone()),
            }
        }
        self.save()?;
        Ok(missing)
    }
}

/// Absolute, symlink-free path, without Windows' `\\?\` verbatim prefix
/// (which some tools can't open) unless it's genuinely needed.
fn canonical(p: &Path) -> PathBuf {
    let c = std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf());
    #[cfg(windows)]
    {
        let s = c.to_string_lossy();
        if let Some(rest) = s.strip_prefix(r"\\?\") {
            if !rest.starts_with("UNC\\") && rest.len() < 260 {
                return PathBuf::from(rest);
            }
        }
    }
    c
}

fn collect_dir(dir: &Path, out: &mut Vec<PathBuf>) -> Result<()> {
    for entry in std::fs::read_dir(dir)? {
        let p = entry?.path();
        if p.is_dir() {
            collect_dir(&p, out)?;
        } else if is_photo(&p) {
            out.push(p);
        }
    }
    Ok(())
}
