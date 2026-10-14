//! A project is a site: one folder holding any number of models (each a
//! processing of a set of photos — a map and/or a 3D model) and the flight
//! plans made for it.
//!
//! ```text
//! <project>/
//!   aerialproject.json      name, models (in order)
//!   models/<id>/            one model: project.json, result/, cache/ (see project.rs)
//!   plans/<id>.json         saved flight plans (PlanInput)
//!   cache/features|matches  shared by every model in the project
//! ```
//!
//! A folder from before projects held models — `project.json` at the top —
//! opens as a project with that one model, left exactly where it is.

use crate::plan::PlanInput;
use crate::project::{Project, PROJECT_FILE};
use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub const WORKSPACE_FILE: &str = "aerialproject.json";
const FORMAT: u32 = 1;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelRef {
    pub id: String,
    pub name: String,
    /// Folder relative to the project ("." for a folder from before models).
    pub dir: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkspaceFile {
    pub format: u32,
    pub name: String,
    #[serde(default)]
    pub models: Vec<ModelRef>,
}

pub struct Workspace {
    pub dir: PathBuf,
    pub file: WorkspaceFile,
}

/// Lower-case letters, digits and dashes; never empty.
fn slug(name: &str) -> String {
    let mut s = String::new();
    for c in name.chars() {
        if c.is_ascii_alphanumeric() {
            s.push(c.to_ascii_lowercase());
        } else if !s.ends_with('-') {
            s.push('-');
        }
    }
    let s = s.trim_matches('-').chars().take(40).collect::<String>();
    if s.is_empty() {
        "model".into()
    } else {
        s
    }
}

impl Workspace {
    pub fn create(dir: &Path, name: &str) -> Result<Workspace> {
        std::fs::create_dir_all(dir)?;
        if dir.join(WORKSPACE_FILE).exists() || dir.join(PROJECT_FILE).exists() {
            bail!("there is already a project in {}", dir.display());
        }
        let w = Workspace {
            dir: dir.to_path_buf(),
            file: WorkspaceFile {
                format: FORMAT,
                name: name.to_string(),
                models: Vec::new(),
            },
        };
        w.save()?;
        Ok(w)
    }

    /// Opens a project, upgrading a single-model folder in place.
    pub fn open(dir: &Path) -> Result<Workspace> {
        let path = dir.join(WORKSPACE_FILE);
        if !path.exists() {
            if dir.join(PROJECT_FILE).exists() {
                let old = Project::open(dir)?;
                let w = Workspace {
                    dir: dir.to_path_buf(),
                    file: WorkspaceFile {
                        format: FORMAT,
                        name: old.file.name.clone(),
                        models: vec![ModelRef {
                            id: "main".into(),
                            name: old.file.name.clone(),
                            dir: ".".into(),
                        }],
                    },
                };
                w.save()?;
                return Ok(w);
            }
            // A model folder inside a project: open its project.
            if let Some(parent) = Self::of_model(dir) {
                return Self::open(&parent);
            }
            bail!("no project at {}", dir.display());
        }
        let text = std::fs::read_to_string(&path)?;
        let file: WorkspaceFile =
            serde_json::from_str(&text).context("aerialproject.json is damaged")?;
        if file.format > FORMAT {
            bail!("this project was made by a newer version of the app");
        }
        Ok(Workspace {
            dir: dir.to_path_buf(),
            file,
        })
    }

    /// The project folder a model folder belongs to, if any.
    pub fn of_model(model_dir: &Path) -> Option<PathBuf> {
        if model_dir.join(WORKSPACE_FILE).exists() {
            return Some(model_dir.to_path_buf()); // a single-model folder
        }
        let models = model_dir.parent()?;
        let root = models.parent()?;
        (models.file_name()? == "models" && root.join(WORKSPACE_FILE).exists())
            .then(|| root.to_path_buf())
    }

    pub fn save(&self) -> Result<()> {
        let tmp = self.dir.join("aerialproject.json.tmp");
        std::fs::write(&tmp, serde_json::to_vec_pretty(&self.file)?)?;
        std::fs::rename(&tmp, self.dir.join(WORKSPACE_FILE))?;
        Ok(())
    }

    pub fn model_dir(&self, m: &ModelRef) -> PathBuf {
        if m.dir == "." {
            self.dir.clone()
        } else {
            self.dir.join(&m.dir)
        }
    }

    pub fn find(&self, id: &str) -> Result<&ModelRef> {
        self.file
            .models
            .iter()
            .find(|m| m.id == id)
            .context("that model no longer exists")
    }

    pub fn add_model(&mut self, name: &str) -> Result<ModelRef> {
        let base = slug(name);
        let mut id = base.clone();
        let mut n = 2;
        while self.file.models.iter().any(|m| m.id == id)
            || self.dir.join("models").join(&id).exists()
        {
            id = format!("{base}-{n}");
            n += 1;
        }
        let r = ModelRef {
            id: id.clone(),
            name: name.to_string(),
            dir: format!("models/{id}"),
        };
        Project::create(&self.model_dir(&r), name)?;
        self.file.models.push(r.clone());
        self.save()?;
        Ok(r)
    }

    pub fn rename_model(&mut self, id: &str, name: &str) -> Result<()> {
        let dir = self.model_dir(self.find(id)?);
        let mut p = Project::open(&dir)?;
        p.file.name = name.to_string();
        p.save()?;
        if let Some(m) = self.file.models.iter_mut().find(|m| m.id == id) {
            m.name = name.to_string();
        }
        self.save()
    }

    /// Deletes a model's own folder (its results and caches) — never the
    /// photos, which live wherever the user keeps them. A model from before
    /// projects held models IS the project folder, so it is only unlisted.
    pub fn delete_model(&mut self, id: &str) -> Result<()> {
        let m = self.find(id)?.clone();
        if m.dir != "." {
            let dir = self.model_dir(&m);
            // Only ever remove a folder that is plainly ours.
            if dir.join(PROJECT_FILE).exists() && dir.starts_with(self.dir.join("models")) {
                std::fs::remove_dir_all(&dir)?;
            }
        }
        self.file.models.retain(|x| x.id != id);
        self.save()
    }

    // ── Plans ──

    fn plans_dir(&self) -> PathBuf {
        self.dir.join("plans")
    }

    pub fn plans(&self) -> Vec<(String, PathBuf, PlanInput)> {
        let mut out: Vec<(String, PathBuf, PlanInput)> = std::fs::read_dir(self.plans_dir())
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|e| {
                let path = e.path();
                let id = path.file_stem()?.to_string_lossy().into_owned();
                if path.extension()? != "json" {
                    return None;
                }
                let input: PlanInput = serde_json::from_slice(&std::fs::read(&path).ok()?).ok()?;
                Some((id, path, input))
            })
            .collect();
        out.sort_by(|a, b| a.2.name.to_lowercase().cmp(&b.2.name.to_lowercase()));
        out
    }

    /// Saves a plan into the project; a new plan gets an id from its name.
    pub fn save_plan(&self, id: Option<&str>, input: &PlanInput) -> Result<String> {
        std::fs::create_dir_all(self.plans_dir())?;
        let id = match id {
            Some(id) if !id.is_empty() => slug(id),
            _ => {
                let base = slug(&input.name);
                let mut id = base.clone();
                let mut n = 2;
                while self.plans_dir().join(format!("{id}.json")).exists() {
                    id = format!("{base}-{n}");
                    n += 1;
                }
                id
            }
        };
        std::fs::write(
            self.plans_dir().join(format!("{id}.json")),
            serde_json::to_vec_pretty(input)?,
        )?;
        Ok(id)
    }

    pub fn delete_plan(&self, id: &str) -> Result<()> {
        let path = self.plans_dir().join(format!("{}.json", slug(id)));
        if path.exists() {
            std::fs::remove_file(path)?;
        }
        Ok(())
    }
}

/// Feature and match caches depend only on the photos and the profile, so
/// every model in a project can share them: the same photo in two models is
/// analysed once.
pub fn shared_cache(model_dir: &Path, kind: &str) -> Option<PathBuf> {
    if !matches!(kind, "features" | "matches") {
        return None;
    }
    Workspace::of_model(model_dir).map(|root| root.join("cache").join(kind))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("ws-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    #[test]
    fn models_and_plans_live_in_the_project() {
        let d = tmp("models");
        let mut w = Workspace::create(&d, "Farm").unwrap();
        let a = w.add_model("North paddock").unwrap();
        let b = w.add_model("North paddock").unwrap();
        assert_eq!(a.id, "north-paddock");
        assert_eq!(b.id, "north-paddock-2");
        assert!(d.join("models/north-paddock/project.json").exists());
        assert_eq!(Workspace::of_model(&w.model_dir(&a)).unwrap(), d);
        // Shared caches resolve to the project.
        assert_eq!(
            shared_cache(&w.model_dir(&a), "features").unwrap(),
            d.join("cache/features")
        );
        assert!(shared_cache(&w.model_dir(&a), "dense").is_none());

        w.rename_model(&a.id, "North").unwrap();
        let w2 = Workspace::open(&d).unwrap();
        assert_eq!(w2.file.models[0].name, "North");
        assert_eq!(
            Project::open(&w2.model_dir(&w2.file.models[0]))
                .unwrap()
                .file
                .name,
            "North"
        );

        let id = w2
            .save_plan(
                None,
                &PlanInput {
                    name: "Survey 1".into(),
                    ..Default::default()
                },
            )
            .unwrap();
        assert_eq!(id, "survey-1");
        assert_eq!(w2.plans().len(), 1);
        w2.delete_plan(&id).unwrap();
        assert!(w2.plans().is_empty());

        let mut w3 = Workspace::open(&d).unwrap();
        w3.delete_model(&b.id).unwrap();
        assert!(!d.join("models/north-paddock-2").exists());
        assert_eq!(w3.file.models.len(), 1);
        std::fs::remove_dir_all(d).ok();
    }

    #[test]
    fn a_folder_from_before_models_opens_as_a_project() {
        let d = tmp("legacy");
        Project::create(&d, "Old survey").unwrap();
        let w = Workspace::open(&d).unwrap();
        assert_eq!(w.file.name, "Old survey");
        assert_eq!(w.file.models.len(), 1);
        assert_eq!(w.model_dir(&w.file.models[0]), d);
        // Unlisting the old model never deletes the folder.
        let mut w = w;
        w.delete_model("main").unwrap();
        assert!(d.join(PROJECT_FILE).exists());
        std::fs::remove_dir_all(d).ok();
    }
}
