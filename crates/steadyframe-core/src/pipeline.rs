//! High-level pipeline: [`Analyzer`] → [`MotionTrack`] → [`Plan`] → rendering.

use crate::config::{CompareMode, Config, MotionModel, MotionSource, RenderConfig};
use crate::crop::{required_zoom, zoom_series};
use crate::error::{Error, Result};
use crate::estimate::{estimate, Rng};
use crate::features::detect;
use crate::flow::{track_checked, LkParams};
use crate::geometry::{unwrap_angle, Mat3, Params, ROT, TX, TY};
use crate::gyro::{auto_sync, focal_from_hfov, gyro_motions, GyroData, Orientation, SyncResult};
use crate::image::{luma_from_rgba, resample_to_gray, Pyramid};
use crate::metrics::{jitter, reduction_pct, Jitter};
use crate::smooth::smooth_trajectory;
use crate::warp::{compare_size, compose_compare, warp_rgba, RsLut, WarpSpec};
use serde::{Deserialize, Serialize};

/// Estimated motion from the previous frame to this one.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct FrameMotion {
    /// Maps previous-frame points to this frame (full-res, centred coords).
    pub m: Mat3,
    /// 0 = tracking lost, 1 = excellent.
    pub confidence: f32,
    pub inliers: u32,
    pub tracked: u32,
}

impl FrameMotion {
    pub fn identity() -> Self {
        FrameMotion { m: Mat3::IDENTITY, confidence: 0.0, inliers: 0, tracked: 0 }
    }
}

/// Output of analysis: per-frame motion. Re-usable with any smoothing settings.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct MotionTrack {
    pub version: u32,
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    pub model: MotionModel,
    pub analysis_width: u32,
    pub motions: Vec<FrameMotion>,
}

impl MotionTrack {
    /// A track with no optical flow (for gyro-only stabilization).
    pub fn empty(width: u32, height: u32, fps: f64, frames: usize) -> MotionTrack {
        MotionTrack {
            version: 1,
            width,
            height,
            fps,
            model: MotionModel::Similarity,
            analysis_width: 0,
            motions: vec![FrameMotion::identity(); frames],
        }
    }

    pub fn len(&self) -> usize {
        self.motions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.motions.is_empty()
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("track serializes")
    }

    pub fn from_json(s: &str) -> Result<MotionTrack> {
        Ok(serde_json::from_str(s)?)
    }
}

/// Streaming frame analyser. Push frames in order, then [`finish`](Self::finish).
pub struct Analyzer {
    cfg: crate::config::AnalysisConfig,
    width: u32,
    height: u32,
    fps: f64,
    aw: usize,
    ah: usize,
    prev: Option<(Pyramid, Vec<(f32, f32)>)>,
    motions: Vec<FrameMotion>,
    rng: Rng,
    lk: LkParams,
}

impl Analyzer {
    pub fn new(cfg: &Config, width: u32, height: u32, fps: f64) -> Result<Analyzer> {
        if width < 16 || height < 16 {
            return Err(Error::InvalidInput(format!("frame size {width}x{height} is too small")));
        }
        if !(fps.is_finite() && fps > 0.0) {
            return Err(Error::InvalidInput(format!("invalid fps {fps}")));
        }
        cfg.validate()?;
        let a = cfg.analysis.clone();
        let aw = (a.analysis_width.min(width)) as usize;
        let ah = ((height as f64 * aw as f64 / width as f64).round() as usize).max(16);
        let lk =
            LkParams { radius: a.lk_radius as i32, iterations: a.lk_iterations as usize, epsilon: 0.01, min_eig: 0.05 };
        Ok(Analyzer {
            cfg: a,
            width,
            height,
            fps,
            aw,
            ah,
            prev: None,
            motions: Vec::new(),
            rng: Rng::new(0x5EED_F00D),
            lk,
        })
    }

    /// Resolution frames are analysed at (pass frames of this size to save work).
    pub fn analysis_size(&self) -> (u32, u32) {
        (self.aw as u32, self.ah as u32)
    }

    pub fn frames(&self) -> usize {
        self.motions.len()
    }

    /// Push an RGBA frame of any size (it's resampled to the analysis size).
    pub fn push_rgba(&mut self, rgba: &[u8], w: u32, h: u32) -> Result<FrameMotion> {
        if rgba.len() < (w * h * 4) as usize {
            return Err(Error::InvalidInput(format!("RGBA buffer too small for {w}x{h}")));
        }
        let luma = luma_from_rgba(rgba, w as usize, h as usize);
        self.push_luma(&luma, w, h)
    }

    /// Push an 8-bit grayscale frame of any size.
    pub fn push_luma(&mut self, luma: &[u8], w: u32, h: u32) -> Result<FrameMotion> {
        let (w, h) = (w as usize, h as usize);
        if w < 16 || h < 16 || luma.len() < w * h {
            return Err(Error::InvalidInput(format!("luma buffer too small for {w}x{h}")));
        }
        let gray = resample_to_gray(luma, w, h, self.aw, self.ah);
        let pyr = Pyramid::build(gray, self.cfg.lk_levels as usize);
        let border = self.cfg.lk_radius as usize + 2;

        let motion = match self.prev.take() {
            None => FrameMotion::identity(),
            Some((prev_pyr, pts)) => self.estimate_motion(&prev_pyr, &pyr, &pts),
        };
        let pts = detect(
            &pyr.levels[0],
            self.cfg.max_features as usize,
            self.cfg.feature_quality,
            self.cfg.min_distance,
            border,
        );
        self.prev = Some((pyr, pts));
        self.motions.push(motion);
        Ok(motion)
    }

    fn estimate_motion(&mut self, prev: &Pyramid, next: &Pyramid, pts: &[(f32, f32)]) -> FrameMotion {
        let pairs = track_checked(prev, next, pts, &self.lk, self.cfg.fb_threshold);
        let (cx, cy) = (self.aw as f64 / 2.0, self.ah as f64 / 2.0);
        let src: Vec<(f64, f64)> = pairs.iter().map(|(a, _)| (a.0 as f64 - cx, a.1 as f64 - cy)).collect();
        let dst: Vec<(f64, f64)> = pairs.iter().map(|(_, b)| (b.0 as f64 - cx, b.1 as f64 - cy)).collect();
        let tracked = pairs.len() as u32;
        let est = estimate(
            &src,
            &dst,
            self.cfg.motion_model,
            self.cfg.ransac_threshold as f64,
            self.cfg.ransac_iterations as usize,
            &mut self.rng,
        );
        match est {
            Some(e) if e.inliers as u32 >= self.cfg.min_inliers => {
                let scale = self.width as f64 / self.aw as f64;
                let ratio = e.inliers as f32 / tracked.max(1) as f32;
                let amount = (e.inliers as f32 / (2 * self.cfg.min_inliers) as f32).min(1.0);
                FrameMotion {
                    m: e.m.rescaled(scale),
                    confidence: (ratio * amount).clamp(0.0, 1.0),
                    inliers: e.inliers as u32,
                    tracked,
                }
            }
            Some(e) => FrameMotion { inliers: e.inliers as u32, tracked, ..FrameMotion::identity() },
            None => FrameMotion { tracked, ..FrameMotion::identity() },
        }
    }

    pub fn track(&self) -> MotionTrack {
        MotionTrack {
            version: 1,
            width: self.width,
            height: self.height,
            fps: self.fps,
            model: self.cfg.motion_model,
            analysis_width: self.aw as u32,
            motions: self.motions.clone(),
        }
    }

    pub fn finish(self) -> MotionTrack {
        MotionTrack {
            version: 1,
            width: self.width,
            height: self.height,
            fps: self.fps,
            model: self.cfg.motion_model,
            analysis_width: self.aw as u32,
            motions: self.motions,
        }
    }
}

/// Per-frame render instructions.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct FramePlan {
    /// Output-centred → input-centred pixel transform.
    pub inv: Mat3,
    pub zoom: f64,
    /// Per-frame motion used for rolling-shutter correction.
    pub rs_velocity: Option<Params>,
}

/// Summary numbers for reports and UIs.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Stats {
    pub frames: usize,
    pub fps: f64,
    pub motion_source: MotionSource,
    pub jitter_before: Jitter,
    pub jitter_after: Jitter,
    /// Overall shake reduction in percent.
    pub reduction_pct: f64,
    pub mean_zoom: f64,
    pub max_zoom: f64,
    /// Frames where optical-flow tracking was unreliable.
    pub low_confidence_frames: usize,
    pub gyro_sync: Option<SyncResult>,
    /// Human-readable warnings (e.g. gyro mismatch).
    #[serde(default)]
    pub notes: Vec<String>,
}

/// Everything needed to render the stabilized video.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Plan {
    pub version: u32,
    pub in_width: u32,
    pub in_height: u32,
    /// Stabilized frame size (before any compare layout).
    pub out_width: u32,
    pub out_height: u32,
    pub fps: f64,
    pub readout: f64,
    pub render: RenderConfig,
    pub frames: Vec<FramePlan>,
    /// Camera path `[tx px, ty px, rotation°]` before and after smoothing.
    pub raw_path: Vec<[f64; 3]>,
    pub smooth_path: Vec<[f64; 3]>,
    pub stats: Stats,
}

impl Plan {
    /// Smooth the camera path and compute per-frame warps.
    pub fn build(cfg: &Config, track: &MotionTrack, gyro: Option<&GyroData>) -> Result<Plan> {
        cfg.validate()?;
        let n = track.motions.len();
        if n == 0 {
            return Err(Error::InvalidInput("no frames were analysed".into()));
        }
        let (w, h) = (track.width as f64, track.height as f64);
        let fps = track.fps;
        let source = cfg.analysis.motion_source;
        let mut sync: Option<SyncResult> = None;
        let mut notes: Vec<String> = Vec::new();

        let motions: Vec<Mat3> = match source {
            MotionSource::OpticalFlow => fill_lost_motions(&track.motions),
            MotionSource::Gyro | MotionSource::Hybrid => {
                let g = gyro.ok_or_else(|| {
                    Error::Gyro(format!("motion_source '{source}' needs gyro data (CSV or GoPro GPMF)"))
                })?;
                let gc = &cfg.gyro;
                let mut orient = Orientation::parse(&gc.orientation)?;
                let mut offset = gc.time_offset;
                let mut f = if gc.focal_px > 0.0 { gc.focal_px } else { focal_from_hfov(w, gc.hfov) };
                let mut gyro_ok = true;
                if gc.auto_sync {
                    match auto_sync(g, &track.motions, fps, w, gc) {
                        Some(s) => {
                            if s.score > 0.6 {
                                gyro_ok = false;
                                notes.push(format!(
                                    "gyro data does not match the video motion (match {:.0}%); check that it belongs to this clip",
                                    (1.0 - s.score).max(0.0) * 100.0
                                ));
                            }
                            orient = Orientation::parse(&s.orientation)?;
                            offset = s.offset;
                            f = s.focal;
                            sync = Some(s);
                        }
                        None => notes.push(
                            "gyro auto-sync needs optical-flow analysis with trackable frames; using the configured orientation/offset/FOV".into(),
                        ),
                    }
                }
                let gm = gyro_motions(g, &orient, offset, f, fps, n);
                if source == MotionSource::Hybrid && !gyro_ok {
                    notes.push("falling back to optical flow only".into());
                    fill_lost_motions(&track.motions)
                } else if source == MotionSource::Gyro {
                    gm
                } else {
                    gm.iter()
                        .zip(&track.motions)
                        .map(|(g, fl)| {
                            if fl.confidence >= 0.5 {
                                let mut p = Params::from_mat(&fl.m);
                                p.0[ROT] = Params::from_mat(g).0[ROT];
                                p.to_mat()
                            } else {
                                *g
                            }
                        })
                        .collect()
                }
            }
        };

        // Cumulative camera path.
        let mut cum = Vec::with_capacity(n);
        let mut c = Mat3::IDENTITY;
        let mut path: Vec<Params> = Vec::with_capacity(n);
        let mut prev_rot = 0.0;
        for (t, m) in motions.iter().enumerate() {
            if t > 0 {
                c = m.mul(&c).normalized();
            }
            cum.push(c);
            let mut p = Params::from_mat(&c);
            p.0[ROT] = unwrap_angle(p.0[ROT], prev_rot);
            prev_rot = p.0[ROT];
            path.push(p);
        }

        let smooth = smooth_trajectory(&path, &cfg.smoothing, w, h);

        let r = &cfg.render;
        // Video encoders (4:2:0) need even dimensions; drop one pixel from odd sizes.
        let (ow, oh) = if r.output_width > 0 {
            (r.output_width as f64, r.output_height as f64)
        } else {
            (((track.width & !1).max(2)) as f64, ((track.height & !1).max(2)) as f64)
        };
        let k = (ow / w).max(oh / h);
        let (hw, hh) = (ow / k / 2.0, oh / k / 2.0);

        let base_inv: Vec<Mat3> =
            cum.iter().zip(&smooth).map(|(c, s)| c.mul(&s.to_mat().inverse_or_identity())).collect();
        let required: Vec<f64> =
            base_inv.iter().map(|m| required_zoom(m, hw, hh, w / 2.0, h / 2.0, cfg.crop.max_zoom)).collect();
        let zooms = zoom_series(&required, &cfg.crop);

        let rs_on = cfg.rolling_shutter.enabled && cfg.rolling_shutter.readout.abs() > 1e-6;
        let mparams: Vec<Params> = motions.iter().map(Params::from_mat).collect();
        let frames: Vec<FramePlan> = (0..n)
            .map(|t| {
                let z = zooms[t];
                let inv = base_inv[t].mul(&Mat3::scale(1.0 / (z * k), 1.0 / (z * k)));
                let rs_velocity = if rs_on && n > 1 {
                    Some(if t == 0 {
                        mparams[1]
                    } else if t + 1 >= n {
                        mparams[t]
                    } else {
                        mparams[t].add(&mparams[t + 1]).scaled(0.5)
                    })
                } else {
                    None
                };
                FramePlan { inv, zoom: z, rs_velocity }
            })
            .collect();

        let jb = jitter(&path);
        let ja = jitter(&smooth);
        let to3 = |p: &Params| [p.0[TX], p.0[TY], p.0[ROT].to_degrees()];
        let low_conf = if source == MotionSource::Gyro {
            0
        } else {
            track.motions.iter().skip(1).filter(|m| m.confidence < 0.2).count()
        };
        let stats = Stats {
            frames: n,
            fps,
            motion_source: source,
            jitter_before: jb,
            jitter_after: ja,
            reduction_pct: reduction_pct(&jb, &ja, w),
            mean_zoom: zooms.iter().sum::<f64>() / n as f64,
            max_zoom: zooms.iter().cloned().fold(0.0, f64::max),
            low_confidence_frames: low_conf,
            gyro_sync: sync,
            notes,
        };
        Ok(Plan {
            version: 1,
            in_width: track.width,
            in_height: track.height,
            out_width: ow as u32,
            out_height: oh as u32,
            fps,
            readout: cfg.rolling_shutter.readout,
            render: cfg.render.clone(),
            frames,
            raw_path: path.iter().map(to3).collect(),
            smooth_path: smooth.iter().map(to3).collect(),
            stats,
        })
    }

    pub fn len(&self) -> usize {
        self.frames.len()
    }

    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Final frame size including the compare layout.
    pub fn final_size(&self) -> (u32, u32) {
        let (w, h) = compare_size(self.render.compare, self.out_width as usize, self.out_height as usize);
        (w as u32, h as u32)
    }

    /// Warp frame `idx` from `src` (input RGBA) into `dst` (out_width×out_height RGBA).
    /// With the `previous` border mode, `dst` should contain the previous output.
    pub fn render(&self, idx: usize, src: &[u8], dst: &mut [u8]) -> Result<()> {
        let (iw, ih) = (self.in_width as usize, self.in_height as usize);
        let (ow, oh) = (self.out_width as usize, self.out_height as usize);
        if src.len() < iw * ih * 4 {
            return Err(Error::InvalidInput(format!("source frame must be {iw}x{ih} RGBA ({} bytes)", iw * ih * 4)));
        }
        if dst.len() < ow * oh * 4 {
            return Err(Error::InvalidInput(format!("destination must be {ow}x{oh} RGBA ({} bytes)", ow * oh * 4)));
        }
        if self.frames.is_empty() {
            return Err(Error::InvalidInput("plan has no frames".into()));
        }
        let fp = &self.frames[idx.min(self.frames.len() - 1)];
        let lut = fp.rs_velocity.map(|v| RsLut::new(&v, self.readout, ih as f64));
        let spec = WarpSpec {
            inv: fp.inv,
            rs: lut.as_ref(),
            interpolation: self.render.interpolation,
            border: self.render.border,
            border_color: self.render.border_color,
        };
        warp_rgba(src, iw, ih, &mut dst[..ow * oh * 4], ow, oh, &spec);
        Ok(())
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("plan serializes")
    }

    pub fn from_json(s: &str) -> Result<Plan> {
        Ok(serde_json::from_str(s)?)
    }
}

/// Replace motions of frames where tracking was lost (confidence 0) by
/// interpolating the nearest good neighbours, so a single blurred frame doesn't
/// freeze the camera path for one step and cause a visible jump.
pub fn fill_lost_motions(motions: &[FrameMotion]) -> Vec<Mat3> {
    let n = motions.len();
    let good: Vec<bool> = motions.iter().enumerate().map(|(t, m)| t == 0 || m.confidence > 0.0).collect();
    let params: Vec<Params> = motions.iter().map(|m| Params::from_mat(&m.m)).collect();
    (0..n)
        .map(|t| {
            if good[t] {
                return motions[t].m;
            }
            let prev = (1..t).rev().find(|&i| good[i]);
            let next = (t + 1..n).find(|&i| good[i]);
            match (prev, next) {
                (Some(a), Some(b)) => {
                    let f = (t - a) as f64 / (b - a) as f64;
                    params[a].scaled(1.0 - f).add(&params[b].scaled(f)).to_mat()
                }
                (Some(a), None) => params[a].to_mat(),
                (None, Some(b)) => params[b].to_mat(),
                (None, None) => Mat3::IDENTITY,
            }
        })
        .collect()
}

/// Stateful renderer that keeps the previous frame (for the `previous` border
/// mode) and applies the compare layout.
#[derive(Default)]
pub struct Renderer {
    stab: Vec<u8>,
    out: Vec<u8>,
}

impl Renderer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Render frame `idx`; returns the final RGBA buffer (size = `plan.final_size()`).
    pub fn render(&mut self, plan: &Plan, idx: usize, src: &[u8]) -> Result<&[u8]> {
        let n = plan.out_width as usize * plan.out_height as usize * 4;
        if self.stab.len() != n {
            self.stab = vec![0; n];
            for px in self.stab.chunks_exact_mut(4) {
                px[3] = 255;
            }
        }
        plan.render(idx, src, &mut self.stab)?;
        if plan.render.compare == CompareMode::None {
            return Ok(&self.stab);
        }
        let (out, _, _) = compose_compare(
            src,
            plan.in_width as usize,
            plan.in_height as usize,
            &self.stab,
            plan.out_width as usize,
            plan.out_height as usize,
            plan.render.compare,
        );
        self.out = out;
        Ok(&self.out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fm(tx: f64, conf: f32) -> FrameMotion {
        FrameMotion { m: Mat3::translation(tx, 0.0), confidence: conf, inliers: 50, tracked: 50 }
    }

    #[test]
    fn lost_frames_are_interpolated() {
        let m = vec![FrameMotion::identity(), fm(2.0, 1.0), fm(0.0, 0.0), fm(0.0, 0.0), fm(8.0, 1.0), fm(0.0, 0.0)];
        let f = fill_lost_motions(&m);
        let tx: Vec<f64> = f.iter().map(|m| m.0[2]).collect();
        assert_eq!(tx[0], 0.0);
        assert!((tx[2] - 4.0).abs() < 1e-9 && (tx[3] - 6.0).abs() < 1e-9, "{tx:?}");
        assert!((tx[5] - 8.0).abs() < 1e-9, "trailing lost frame reuses last good motion");
    }

    #[test]
    fn odd_sizes_render_even_output() {
        let t = MotionTrack { motions: vec![fm(1.0, 1.0); 10], ..MotionTrack::empty(321, 241, 30.0, 10) };
        let p = Plan::build(&Config::default(), &t, None).unwrap();
        assert_eq!((p.out_width, p.out_height), (320, 240));
        let src = vec![128u8; 321 * 241 * 4];
        let mut dst = vec![0u8; 320 * 240 * 4];
        p.render(3, &src, &mut dst).unwrap();
    }

    #[test]
    fn single_frame_and_empty_inputs() {
        let t = MotionTrack::empty(64, 48, 30.0, 1);
        for preset in ["handheld", "cinematic", "vehicle", "tripod"] {
            Plan::build(&Config::preset(preset).unwrap(), &t, None).unwrap();
        }
        assert!(Plan::build(&Config::default(), &MotionTrack::empty(64, 48, 30.0, 0), None).is_err());
        let t = MotionTrack::empty(64, 48, 30.0, 2);
        Plan::build(&Config::preset("cinematic").unwrap(), &t, None).unwrap();
    }
}
