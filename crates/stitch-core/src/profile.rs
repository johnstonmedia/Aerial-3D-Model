//! Memory profiles. Every size the pipeline picks comes from here, so a
//! machine with 8 GB and one with 32 GB run the same code with different knobs.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ProfileKind {
    /// ≤ 12 GB installed — e.g. MacBook Neo (8 GB), entry Windows laptops.
    Lite,
    /// 12–28 GB — e.g. 16 GB Snapdragon / M-series machines.
    Standard,
    /// ≥ 28 GB.
    High,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Profile {
    pub kind: ProfileKind,
    /// Long side, in pixels, that photos are shrunk to for feature detection.
    pub work_size: u32,
    /// Keypoints kept per photo.
    pub max_features: usize,
    /// Byte budget for decoded photos held while rendering.
    pub render_cache_bytes: usize,
    /// Worker threads.
    pub threads: usize,
}

impl Profile {
    pub fn for_kind(kind: ProfileKind) -> Self {
        let cores = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
        match kind {
            ProfileKind::Lite => Profile {
                kind,
                work_size: 1024,
                max_features: 1500,
                render_cache_bytes: 900 << 20,
                threads: cores.min(4),
            },
            ProfileKind::Standard => Profile {
                kind,
                work_size: 1600,
                max_features: 2500,
                render_cache_bytes: 3 << 30,
                threads: cores,
            },
            ProfileKind::High => Profile {
                kind,
                work_size: 2048,
                max_features: 4000,
                render_cache_bytes: 8 << 30,
                threads: cores,
            },
        }
    }

    /// Picks a profile from installed RAM.
    pub fn detect() -> Self {
        Self::for_kind(Self::kind_for_ram(total_ram_bytes()))
    }

    pub fn kind_for_ram(bytes: u64) -> ProfileKind {
        let gb = bytes as f64 / (1u64 << 30) as f64;
        if gb < 12.0 {
            ProfileKind::Lite
        } else if gb < 28.0 {
            ProfileKind::Standard
        } else {
            ProfileKind::High
        }
    }
}

pub fn total_ram_bytes() -> u64 {
    let mut sys = sysinfo::System::new();
    sys.refresh_memory();
    sys.total_memory()
}
