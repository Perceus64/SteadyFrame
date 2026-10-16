//! Every tunable knob of the stabilizer, with defaults and named presets.
//!
//! Configs are plain serde structs, so they round-trip through JSON. Partial JSON
//! is accepted everywhere: missing fields keep their current value
//! (see [`Config::merged_with_json`]).

use crate::error::{Error, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;

macro_rules! str_enum {
    ($(#[$m:meta])* $name:ident { $($(#[$vm:meta])* $var:ident => $s:literal),+ $(,)? }) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
        #[serde(rename_all = "snake_case")]
        pub enum $name { $($(#[$vm])* $var),+ }

        impl $name {
            /// All accepted names, in declaration order.
            pub const NAMES: &'static [&'static str] = &[$($s),+];
            pub fn as_str(&self) -> &'static str { match self { $($name::$var => $s),+ } }
        }

        impl std::str::FromStr for $name {
            type Err = Error;
            fn from_str(s: &str) -> Result<Self> {
                let norm = s.trim().to_ascii_lowercase().replace('-', "_");
                match norm.as_str() {
                    $($s => Ok($name::$var),)+
                    _ => Err(Error::Config(format!(
                        "unknown {} '{}' (expected one of: {})",
                        stringify!($name), s, Self::NAMES.join(", ")
                    ))),
                }
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { f.write_str(self.as_str()) }
        }
    };
}

str_enum!(
    /// Which geometric model is fitted between consecutive frames.
    MotionModel {
        /// x/y shift only. Fastest, most robust, ignores roll and zoom.
        Translation => "translation",
        /// Shift + rotation + uniform scale. The best default.
        Similarity => "similarity",
        /// Adds non-uniform scale and shear.
        Affine => "affine",
        /// Full projective model; corrects perspective wobble (needs texture).
        Homography => "homography",
    }
);

str_enum!(
    /// Where camera motion comes from.
    MotionSource {
        OpticalFlow => "optical_flow",
        Gyro => "gyro",
        /// Gyro for rotation (robust to blur), optical flow for translation/zoom.
        Hybrid => "hybrid",
    }
);

str_enum!(
    /// Camera-path smoothing algorithm.
    SmoothingMethod {
        Gaussian => "gaussian",
        MovingAverage => "moving_average",
        /// Constant-velocity Kalman filter with Rauch–Tung–Striebel smoothing.
        Kalman => "kalman",
        /// L1-optimal camera path (Grundmann et al. 2011): piecewise static,
        /// constant-velocity and constant-acceleration segments. "Cinematic".
        L1 => "l1",
    }
);

str_enum!(
    /// How borders exposed by the correction are hidden.
    CropMode {
        /// No zoom; exposed borders are filled according to the border mode.
        None => "none",
        /// One zoom factor for the whole clip, just enough to hide all borders.
        Fixed => "fixed",
        /// Zoom varies smoothly over time (less crop on calm parts).
        Dynamic => "dynamic",
    }
);

str_enum!(
    Interpolation {
        Nearest => "nearest",
        Bilinear => "bilinear",
        Bicubic => "bicubic",
    }
);

str_enum!(
    /// What to draw where the warped frame doesn't cover the output.
    BorderMode {
        Black => "black",
        Color => "color",
        Replicate => "replicate",
        Reflect => "reflect",
        /// Keep the previous output frame's pixels there (temporal fill).
        Previous => "previous",
    }
);

str_enum!(
    /// Debug/preview output layouts.
    CompareMode {
        None => "none",
        SideBySide => "side_by_side",
        Split => "split",
    }
);

str_enum!(
    TimeUnit {
        Auto => "auto",
        Seconds => "s",
        Milliseconds => "ms",
        Microseconds => "us",
        Nanoseconds => "ns",
    }
);

str_enum!(
    AngleUnit {
        Auto => "auto",
        Radians => "rad",
        Degrees => "deg",
    }
);

/// Motion analysis (feature detection, tracking and model fitting).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AnalysisConfig {
    /// Frames are downscaled to this width for analysis. 480–960 is a good range.
    pub analysis_width: u32,
    pub motion_model: MotionModel,
    pub motion_source: MotionSource,
    /// Maximum number of corners tracked per frame.
    pub max_features: u32,
    /// Corner quality relative to the strongest corner (0..1).
    pub feature_quality: f32,
    /// Minimum distance between corners, in analysis pixels.
    pub min_distance: f32,
    /// Lucas–Kanade window radius (window is 2r+1 square).
    pub lk_radius: u32,
    /// Pyramid levels for Lucas–Kanade.
    pub lk_levels: u32,
    pub lk_iterations: u32,
    /// Forward–backward consistency threshold in analysis pixels.
    pub fb_threshold: f32,
    /// RANSAC inlier threshold in analysis pixels.
    pub ransac_threshold: f32,
    pub ransac_iterations: u32,
    /// Frames with fewer inliers are treated as "tracking lost" (no motion).
    pub min_inliers: u32,
}

impl Default for AnalysisConfig {
    fn default() -> Self {
        Self {
            analysis_width: 640,
            motion_model: MotionModel::Similarity,
            motion_source: MotionSource::OpticalFlow,
            max_features: 400,
            feature_quality: 0.01,
            min_distance: 10.0,
            lk_radius: 7,
            lk_levels: 4,
            lk_iterations: 20,
            fb_threshold: 1.0,
            ransac_threshold: 1.5,
            ransac_iterations: 400,
            min_inliers: 12,
        }
    }
}

/// Camera-path smoothing and locks.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SmoothingConfig {
    pub method: SmoothingMethod,
    /// Smoothing strength in frames (≈ how many frames of motion are averaged).
    pub radius: f64,
    /// Per-channel multipliers of `radius`. 0 disables smoothing (no correction)
    /// for that channel.
    pub translation: f64,
    pub rotation: f64,
    pub zoom: f64,
    /// L1 weights for velocity, acceleration and jerk.
    pub l1_weights: [f64; 3],
    pub l1_iterations: u32,
    /// Lock the camera completely to the reference frame (virtual tripod).
    pub tripod: bool,
    /// Reference frame for tripod/horizon/zoom locks.
    pub reference_frame: u32,
    /// Keep the horizon at a fixed angle (removes all roll).
    pub horizon_lock: bool,
    /// Extra output rotation in degrees when horizon lock is on (levels a
    /// horizon that was already tilted in the reference frame).
    pub horizon_angle: f64,
    /// Remove all zoom/scale changes (e.g. walking towards the subject).
    pub zoom_lock: bool,
    /// Maximum correction shift as a fraction of the frame size (≤0 = unlimited).
    pub limit_shift: f64,
    /// Maximum correction rotation in degrees (≤0 = unlimited).
    pub limit_rotation: f64,
    /// Maximum correction scale change as a fraction (≤0 = unlimited).
    pub limit_scale: f64,
}

impl Default for SmoothingConfig {
    fn default() -> Self {
        Self {
            method: SmoothingMethod::Gaussian,
            radius: 30.0,
            translation: 1.0,
            rotation: 1.0,
            zoom: 1.0,
            l1_weights: [10.0, 1.0, 100.0],
            l1_iterations: 400,
            tripod: false,
            reference_frame: 0,
            horizon_lock: false,
            horizon_angle: 0.0,
            zoom_lock: false,
            limit_shift: 0.15,
            limit_rotation: 8.0,
            limit_scale: 0.15,
        }
    }
}

/// Zoom/crop used to hide exposed borders.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CropConfig {
    pub mode: CropMode,
    /// Never zoom in more than this (borders may show beyond it).
    pub max_zoom: f64,
    /// Additional constant zoom on top of the automatic one.
    pub extra_zoom: f64,
    /// Smoothing (frames) of the zoom curve in dynamic mode.
    pub dynamic_smoothing: f64,
}

impl Default for CropConfig {
    fn default() -> Self {
        Self { mode: CropMode::Fixed, max_zoom: 1.4, extra_zoom: 1.0, dynamic_smoothing: 45.0 }
    }
}

/// Final frame rendering.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RenderConfig {
    pub interpolation: Interpolation,
    pub border: BorderMode,
    /// RGBA fill used by `border = "color"`.
    pub border_color: [u8; 4],
    /// Output size (0 = same as input). If the aspect ratio differs, the
    /// stabilized frame is scaled to cover the output.
    pub output_width: u32,
    pub output_height: u32,
    pub compare: CompareMode,
}

impl Default for RenderConfig {
    fn default() -> Self {
        Self {
            interpolation: Interpolation::Bilinear,
            border: BorderMode::Black,
            border_color: [0, 0, 0, 255],
            output_width: 0,
            output_height: 0,
            compare: CompareMode::None,
        }
    }
}

/// CMOS rolling-shutter ("jello") correction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RollingShutterConfig {
    pub enabled: bool,
    /// Sensor readout time as a fraction of the frame interval (0..1).
    /// Typical phones: 0.3–0.8. Negative values handle bottom-to-top sensors.
    pub readout: f64,
}

impl Default for RollingShutterConfig {
    fn default() -> Self {
        Self { enabled: false, readout: 0.5 }
    }
}

/// Gyroscope data handling.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GyroConfig {
    /// Maps gyro axes to camera axes (x right, y down, z forward). Each letter
    /// names the gyro axis feeding that camera axis; lowercase negates it.
    /// "XYZ" = identity, "YxZ" = camera x ← gyro y, camera y ← −gyro x, …
    pub orientation: String,
    /// Seconds added to gyro timestamps to align them with the video.
    pub time_offset: f64,
    /// Horizontal field of view in degrees (used when focal_px is 0).
    pub hfov: f64,
    /// Focal length in full-resolution pixels (0 = derive from hfov).
    pub focal_px: f64,
    /// Estimate time offset, axis orientation and focal length by matching gyro
    /// rotation against optical flow.
    pub auto_sync: bool,
    /// Search ±this many seconds for the time offset during auto-sync.
    pub sync_range: f64,
    /// Search all 48 axis orientations during auto-sync.
    pub sync_orientation: bool,
    /// Estimate focal length during auto-sync.
    pub sync_focal: bool,
    pub time_unit: TimeUnit,
    pub angle_unit: AngleUnit,
}

impl Default for GyroConfig {
    fn default() -> Self {
        Self {
            orientation: "XYZ".into(),
            time_offset: 0.0,
            hfov: 75.0,
            focal_px: 0.0,
            auto_sync: true,
            sync_range: 1.0,
            sync_orientation: true,
            sync_focal: true,
            time_unit: TimeUnit::Auto,
            angle_unit: AngleUnit::Auto,
        }
    }
}

/// Complete stabilizer configuration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct Config {
    pub analysis: AnalysisConfig,
    pub smoothing: SmoothingConfig,
    pub crop: CropConfig,
    pub render: RenderConfig,
    pub rolling_shutter: RollingShutterConfig,
    pub gyro: GyroConfig,
}

/// Named presets with a one-line description each.
pub const PRESETS: &[(&str, &str)] = &[
    ("handheld", "Balanced default for handheld clips"),
    ("smooth", "Strong Gaussian smoothing, dynamic crop"),
    ("walking", "Walking/vlog footage: heavy translation smoothing, horizon lock"),
    ("vehicle", "Dashcams/drones: Kalman smoothing, rolling-shutter fix"),
    ("action", "Action cameras: fast Kalman, rolling shutter, homography, bigger crop budget"),
    ("cinematic", "L1-optimal path: static shots and clean pans"),
    ("tripod", "Lock the frame completely (virtual tripod)"),
    ("gentle", "Light smoothing that keeps intentional motion"),
    ("gyro", "Gyro-driven (GoPro GPMF or gyro CSV) with auto-sync"),
];

impl Config {
    /// Build a config from a preset name.
    pub fn preset(name: &str) -> Result<Config> {
        let mut c = Config::default();
        match name.trim().to_ascii_lowercase().as_str() {
            "handheld" | "default" => {}
            "smooth" => {
                c.smoothing.radius = 60.0;
                c.crop.mode = CropMode::Dynamic;
                c.crop.max_zoom = 1.5;
                c.smoothing.limit_shift = 0.2;
            }
            "walking" => {
                c.smoothing.radius = 45.0;
                c.smoothing.translation = 1.4;
                c.smoothing.horizon_lock = true;
                c.smoothing.limit_shift = 0.2;
                c.crop.mode = CropMode::Dynamic;
                c.crop.max_zoom = 1.5;
            }
            "vehicle" => {
                c.smoothing.method = SmoothingMethod::Kalman;
                c.smoothing.radius = 20.0;
                c.rolling_shutter.enabled = true;
                c.rolling_shutter.readout = 0.5;
            }
            "action" => {
                c.analysis.motion_model = MotionModel::Homography;
                c.smoothing.method = SmoothingMethod::Kalman;
                c.smoothing.radius = 15.0;
                c.smoothing.limit_shift = 0.2;
                c.smoothing.limit_rotation = 15.0;
                c.rolling_shutter.enabled = true;
                c.crop.mode = CropMode::Dynamic;
                c.crop.max_zoom = 1.6;
            }
            "cinematic" => {
                c.smoothing.method = SmoothingMethod::L1;
                c.smoothing.radius = 60.0;
                c.crop.max_zoom = 1.3;
            }
            "tripod" => {
                c.smoothing.tripod = true;
                c.smoothing.limit_shift = 0.0;
                c.smoothing.limit_rotation = 0.0;
                c.smoothing.limit_scale = 0.0;
                c.crop.mode = CropMode::Fixed;
                c.crop.max_zoom = 1.5;
            }
            "gentle" => {
                c.smoothing.radius = 12.0;
                c.smoothing.limit_shift = 0.08;
                c.crop.max_zoom = 1.15;
            }
            "gyro" => {
                c.analysis.motion_source = MotionSource::Hybrid;
                c.rolling_shutter.enabled = true;
                c.crop.mode = CropMode::Dynamic;
                c.crop.max_zoom = 1.5;
            }
            other => {
                return Err(Error::Config(format!(
                    "unknown preset '{other}' (available: {})",
                    PRESETS.iter().map(|p| p.0).collect::<Vec<_>>().join(", ")
                )))
            }
        }
        Ok(c)
    }

    /// Parse a full or partial JSON config on top of the defaults.
    pub fn from_json(json: &str) -> Result<Config> {
        Config::default().merged_with_json(json)
    }

    /// Apply a (partial) JSON object on top of this config.
    pub fn merged_with_json(&self, json: &str) -> Result<Config> {
        if json.trim().is_empty() {
            return Ok(self.clone());
        }
        let patch: Value = serde_json::from_str(json)?;
        self.merged_with_value(&patch)
    }

    pub fn merged_with_value(&self, patch: &Value) -> Result<Config> {
        let mut base = serde_json::to_value(self)?;
        merge_json(&mut base, patch);
        let cfg: Config = serde_json::from_value(base)?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn to_json_pretty(&self) -> String {
        serde_json::to_string_pretty(self).expect("config serializes")
    }

    /// Check ranges; returns a descriptive error for the first bad value.
    pub fn validate(&self) -> Result<()> {
        let bad = |m: &str| Err(Error::Config(m.to_string()));
        let a = &self.analysis;
        if a.analysis_width < 64 {
            return bad("analysis.analysis_width must be >= 64");
        }
        if a.max_features < 8 {
            return bad("analysis.max_features must be >= 8");
        }
        if !(0.0..1.0).contains(&a.feature_quality) {
            return bad("analysis.feature_quality must be in [0, 1)");
        }
        if a.lk_radius < 2 || a.lk_radius > 31 {
            return bad("analysis.lk_radius must be in 2..=31");
        }
        if a.lk_levels == 0 || a.lk_levels > 8 {
            return bad("analysis.lk_levels must be in 1..=8");
        }
        if a.ransac_threshold <= 0.0 {
            return bad("analysis.ransac_threshold must be > 0");
        }
        let s = &self.smoothing;
        if s.radius < 0.0 || !s.radius.is_finite() {
            return bad("smoothing.radius must be >= 0");
        }
        if s.translation < 0.0 || s.rotation < 0.0 || s.zoom < 0.0 {
            return bad("smoothing channel multipliers must be >= 0");
        }
        if s.l1_weights.iter().any(|w| *w < 0.0) {
            return bad("smoothing.l1_weights must be >= 0");
        }
        let c = &self.crop;
        if c.max_zoom < 1.0 || c.max_zoom > 8.0 {
            return bad("crop.max_zoom must be in [1, 8]");
        }
        if c.extra_zoom < 0.1 || c.extra_zoom > 8.0 {
            return bad("crop.extra_zoom must be in [0.1, 8]");
        }
        let r = &self.render;
        if (r.output_width == 0) != (r.output_height == 0) {
            return bad("render.output_width and output_height must both be set or both be 0");
        }
        if r.output_width % 2 == 1 || r.output_height % 2 == 1 {
            return bad("render output size must be even (required by most encoders)");
        }
        if self.rolling_shutter.readout.abs() > 1.5 {
            return bad("rolling_shutter.readout must be within [-1.5, 1.5]");
        }
        let g = &self.gyro;
        crate::gyro::Orientation::parse(&g.orientation)?;
        if !(1.0..=179.0).contains(&g.hfov) {
            return bad("gyro.hfov must be in [1, 179] degrees");
        }
        if g.focal_px < 0.0 {
            return bad("gyro.focal_px must be >= 0");
        }
        Ok(())
    }
}

/// Deep-merge `patch` into `base` (objects merge, everything else replaces).
pub fn merge_json(base: &mut Value, patch: &Value) {
    match (base, patch) {
        (Value::Object(b), Value::Object(p)) => {
            for (k, v) in p {
                match b.get_mut(k) {
                    Some(slot) => merge_json(slot, v),
                    None => {
                        b.insert(k.clone(), v.clone());
                    }
                }
            }
        }
        (slot, v) => *slot = v.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_are_valid() {
        for (name, _) in PRESETS {
            Config::preset(name).unwrap().validate().unwrap();
        }
        assert!(Config::preset("nope").is_err());
    }

    #[test]
    fn partial_json_merges() {
        let c = Config::preset("tripod").unwrap();
        let c2 = c.merged_with_json(r#"{"crop":{"max_zoom":1.2},"render":{"border":"reflect"}}"#).unwrap();
        assert!(c2.smoothing.tripod);
        assert_eq!(c2.crop.max_zoom, 1.2);
        assert_eq!(c2.render.border, BorderMode::Reflect);
        assert!(c.merged_with_json(r#"{"crop":{"max_zoom":0.5}}"#).is_err());
    }

    #[test]
    fn enums_parse_kebab_and_snake() {
        assert_eq!("side-by-side".parse::<CompareMode>().unwrap(), CompareMode::SideBySide);
        assert_eq!("Moving_Average".parse::<SmoothingMethod>().unwrap(), SmoothingMethod::MovingAverage);
        assert!("wobble".parse::<MotionModel>().is_err());
    }
}
