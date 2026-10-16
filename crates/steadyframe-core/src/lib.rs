//! # steadyframe-core
//!
//! A self-contained video stabilization engine written in pure Rust. It has no
//! native dependencies, so the same code runs in the browser (WebAssembly), in a
//! CLI, and inside a GitHub Action.
//!
//! The pipeline has two stages:
//!
//! 1. **Analysis** ([`Analyzer`]) — every frame is converted to a small grayscale
//!    image; Shi–Tomasi corners are tracked with pyramidal Lucas–Kanade optical
//!    flow and a robust (RANSAC) motion model is fitted between consecutive frames.
//!    The result is a [`MotionTrack`], which is cheap to store and re-use.
//! 2. **Planning and rendering** ([`Plan`]) — the camera path is smoothed
//!    (Gaussian, moving average, Kalman/RTS or L1-optimal), optionally locked
//!    (tripod, horizon, zoom), fused with gyro data, cropped/zoomed to hide
//!    borders, and each frame is warped with optional rolling-shutter correction.
//!
//! Because planning is separate from analysis, settings can be changed and the
//! result previewed instantly without re-analysing the video.

#![allow(clippy::needless_range_loop, clippy::too_many_arguments, clippy::type_complexity)]

pub mod config;
pub mod crop;
pub mod error;
pub mod estimate;
pub mod features;
pub mod flow;
pub mod geometry;
pub mod gpmf;
pub mod gyro;
pub mod image;
pub mod metrics;
pub mod pipeline;
pub mod smooth;
pub mod warp;

pub use config::*;
pub use error::{Error, Result};
pub use geometry::{Mat3, Params};
pub use gyro::{GyroData, SyncResult};
pub use pipeline::{Analyzer, FrameMotion, FramePlan, MotionTrack, Plan, Renderer, Stats};
pub use warp::compose_compare;

/// Library version, handy for reports.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
