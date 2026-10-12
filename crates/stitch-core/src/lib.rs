//! Aerial image stitching engine.
//!
//! The pipeline never holds the whole image set in memory:
//!
//! 1. **Features** — each photo is decoded once, shrunk to a working size, and
//!    reduced to a few thousand ORB-style keypoints, cached on disk.
//! 2. **Pairs** — photos are only compared with their GPS neighbours (or their
//!    neighbours in capture order when there is no GPS), so cost grows linearly
//!    with the number of photos instead of quadratically.
//! 3. **Matches** — each candidate pair is matched and verified with RANSAC
//!    under a similarity model; results are cached per pair.
//! 4. **Solve** — one sparse linear least-squares problem places every photo,
//!    using the matches for relative accuracy and GPS for absolute position.
//!    With GPS the output frame is UTM metres, north up.
//! 5. **Render** — the mosaic is drawn tile by tile straight to disk with a
//!    byte-budgeted image cache, then a zoom pyramid is built from the tiles.
//!
//! Memory use is governed by a [`Profile`], picked from installed RAM.

pub mod export;
pub mod features;
pub mod geo;
pub mod imgops;
pub mod matching;
pub mod meta;
pub mod pipeline;
pub mod profile;
pub mod project;
pub mod render;
pub mod sim;
pub mod solve;

pub use profile::{Profile, ProfileKind};
pub use project::Project;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Progress report sent from long-running stages.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Progress {
    pub stage: String,
    pub done: usize,
    pub total: usize,
    pub message: String,
}

/// Callback + cancellation handle threaded through every stage.
#[derive(Clone)]
pub struct Ctx {
    report: Arc<dyn Fn(Progress) + Send + Sync>,
    cancel: Arc<AtomicBool>,
}

impl Ctx {
    pub fn new(report: impl Fn(Progress) + Send + Sync + 'static, cancel: Arc<AtomicBool>) -> Self {
        Self { report: Arc::new(report), cancel }
    }

    /// A context that reports nothing and can't be cancelled.
    pub fn silent() -> Self {
        Self::new(|_| {}, Arc::new(AtomicBool::new(false)))
    }

    pub fn report(&self, stage: &str, done: usize, total: usize, message: impl Into<String>) {
        (self.report)(Progress { stage: stage.into(), done, total, message: message.into() });
    }

    pub fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }

    pub fn check(&self) -> anyhow::Result<()> {
        if self.cancelled() {
            anyhow::bail!("cancelled");
        }
        Ok(())
    }
}
