#![allow(clippy::needless_range_loop)]
//! End-to-end tests on synthetic shaky footage with known ground truth.

use steadyframe_core::config::*;
use steadyframe_core::estimate::Rng;
use steadyframe_core::geometry::{Mat3, Params, ROT, TX, TY};
use steadyframe_core::gyro::{auto_sync, gyro_motions, GyroData, Orientation};
use steadyframe_core::warp::{warp_rgba, WarpSpec};
use steadyframe_core::{Analyzer, FrameMotion, MotionTrack, Plan, Renderer};

const W: usize = 480;
const H: usize = 272;

/// Smooth random texture with some sharp detail.
fn texture(w: usize, h: usize, seed: u64) -> Vec<u8> {
    let mut rng = Rng::new(seed);
    let blobs: Vec<(f64, f64, f64, f64)> = (0..400)
        .map(|_| (rng.unit() * w as f64, rng.unit() * h as f64, 3.0 + rng.unit() * 18.0, rng.unit() * 2.0 - 1.0))
        .collect();
    let mut img = vec![0u8; w * h * 4];
    for y in 0..h {
        for x in 0..w {
            let mut v = 128.0;
            for &(bx, by, r, a) in &blobs {
                let d2 = (x as f64 - bx).powi(2) + (y as f64 - by).powi(2);
                if d2 < 9.0 * r * r {
                    v += 90.0 * a * (-d2 / (2.0 * r * r)).exp();
                }
            }
            let v = v.clamp(0.0, 255.0) as u8;
            let i = (y * w + x) * 4;
            img[i] = v;
            img[i + 1] = v.wrapping_add(40);
            img[i + 2] = 255 - v;
            img[i + 3] = 255;
        }
    }
    img
}

/// Camera poses (output-centred → base-centred) for a shaky pan.
fn poses(n: usize, seed: u64) -> Vec<Mat3> {
    let mut rng = Rng::new(seed);
    (0..n)
        .map(|t| {
            let jx = (rng.unit() - 0.5) * 12.0;
            let jy = (rng.unit() - 0.5) * 12.0;
            let jr = (rng.unit() - 0.5) * 2.0f64.to_radians();
            Mat3::translation(1.2 * t as f64 + jx - 40.0, jy).mul(&Mat3::rotation(jr))
        })
        .collect()
}

fn render_frames(base: &[u8], bw: usize, bh: usize, poses: &[Mat3]) -> Vec<Vec<u8>> {
    poses
        .iter()
        .map(|p| {
            let mut f = vec![0u8; W * H * 4];
            let spec = WarpSpec {
                inv: *p,
                rs: None,
                interpolation: Interpolation::Bilinear,
                border: BorderMode::Replicate,
                border_color: [0; 4],
            };
            warp_rgba(base, bw, bh, &mut f, W, H, &spec);
            f
        })
        .collect()
}

fn analyse(cfg: &Config, frames: &[Vec<u8>]) -> MotionTrack {
    let mut a = Analyzer::new(cfg, W as u32, H as u32, 30.0).unwrap();
    for f in frames {
        a.push_rgba(f, W as u32, H as u32).unwrap();
    }
    a.finish()
}

#[test]
fn recovers_motion_and_reduces_shake() {
    let (bw, bh) = (720, 400);
    let base = texture(bw, bh, 11);
    let n = 60;
    let ps = poses(n, 5);
    let frames = render_frames(&base, bw, bh, &ps);
    let mut cfg = Config::default();
    cfg.analysis.analysis_width = 480;
    let track = analyse(&cfg, &frames);
    assert_eq!(track.len(), n);

    let mut errs = Vec::new();
    for t in 1..n {
        let truth = ps[t].inverse().unwrap().mul(&ps[t - 1]);
        let tp = Params::from_mat(&truth);
        let ep = Params::from_mat(&track.motions[t].m);
        assert!(track.motions[t].confidence > 0.3, "frame {t} confidence {}", track.motions[t].confidence);
        errs.push((
            (tp.0[TX] - ep.0[TX]).abs(),
            (tp.0[TY] - ep.0[TY]).abs(),
            (tp.0[ROT] - ep.0[ROT]).abs().to_degrees(),
        ));
    }
    errs.sort_by(|a, b| (a.0 + a.1).partial_cmp(&(b.0 + b.1)).unwrap());
    let med = errs[errs.len() / 2];
    assert!(med.0 < 0.3 && med.1 < 0.3 && med.2 < 0.05, "median error {med:?}");

    for preset in ["handheld", "smooth", "vehicle", "cinematic", "tripod", "gentle", "walking", "action"] {
        let mut c = Config::preset(preset).unwrap();
        c.analysis.analysis_width = 480;
        let plan = Plan::build(&c, &track, None).unwrap();
        assert_eq!(plan.len(), n);
        assert!(plan.stats.reduction_pct > 50.0, "{preset}: reduction {}", plan.stats.reduction_pct);
        assert!(plan.stats.max_zoom <= c.crop.max_zoom + 1e-9);
        let mut r = Renderer::new();
        let out = r.render(&plan, 10, &frames[10]).unwrap();
        let (fw, fh) = plan.final_size();
        assert_eq!(out.len(), (fw * fh * 4) as usize);
    }

    // Tripod output should be nearly static: compare two rendered frames.
    let mut c = Config::preset("tripod").unwrap();
    c.crop.mode = CropMode::None;
    let plan = Plan::build(&c, &track, None).unwrap();
    let mut a = vec![0u8; W * H * 4];
    let mut b = vec![0u8; W * H * 4];
    plan.render(5, &frames[5], &mut a).unwrap();
    plan.render(25, &frames[25], &mut b).unwrap();
    let mut diff = 0.0;
    let mut cnt = 0.0;
    for y in H / 4..3 * H / 4 {
        for x in W / 4..3 * W / 4 {
            let i = (y * W + x) * 4;
            diff += (a[i] as f64 - b[i] as f64).abs();
            cnt += 1.0;
        }
    }
    assert!(diff / cnt < 6.0, "tripod residual {}", diff / cnt);
}

#[test]
fn output_size_and_compare_modes() {
    let (bw, bh) = (720, 400);
    let base = texture(bw, bh, 3);
    let ps = poses(12, 9);
    let frames = render_frames(&base, bw, bh, &ps);
    let track = analyse(&Config::default(), &frames);
    let mut c = Config::default();
    c.render.output_width = 320;
    c.render.output_height = 320;
    c.render.compare = CompareMode::SideBySide;
    c.render.border = BorderMode::Previous;
    c.rolling_shutter.enabled = true;
    c.render.interpolation = Interpolation::Bicubic;
    let plan = Plan::build(&c, &track, None).unwrap();
    assert_eq!(plan.final_size(), (640, 320));
    let mut r = Renderer::new();
    for (i, f) in frames.iter().enumerate() {
        let out = r.render(&plan, i, f).unwrap();
        assert_eq!(out.len(), 640 * 320 * 4);
    }
    // Plans and tracks round-trip through JSON.
    let p2 = Plan::from_json(&plan.to_json()).unwrap();
    assert_eq!(p2.frames.len(), plan.frames.len());
    assert_eq!(MotionTrack::from_json(&track.to_json()).unwrap(), track);
}

#[test]
fn gyro_auto_sync_recovers_offset_orientation_and_focal() {
    let mut rng = Rng::new(42);
    let rate = 200.0;
    let fps = 30.0;
    let dur = 8.0;
    // Smooth random angular velocity in camera axes.
    let mut w = [0.0f64; 3];
    let mut raw = Vec::new();
    let mut cam = Vec::new();
    let truth = Orientation::parse("zXy").unwrap();
    for i in 0..(dur * rate) as usize {
        for k in 0..3 {
            w[k] = 0.9 * w[k] + 0.4 * (rng.unit() - 0.5);
        }
        let t = i as f64 / rate;
        raw.push((t, w));
        cam.push((t, truth.apply(w)));
    }
    let raw = GyroData::new(raw).unwrap();
    let cam = GyroData::new(cam).unwrap();
    let n = ((dur - 1.0) * fps) as usize;
    let (offset, focal) = (0.137, 520.0);
    let motions: Vec<FrameMotion> = gyro_motions(&cam, &Orientation::IDENTITY, offset, focal, fps, n)
        .into_iter()
        .map(|m| FrameMotion { m, confidence: 1.0, inliers: 100, tracked: 100 })
        .collect();
    let cfg = GyroConfig::default();
    let s = auto_sync(&raw, &motions, fps, 640.0, &cfg).expect("sync");
    assert_eq!(s.orientation, "zXy");
    assert!((s.offset - offset).abs() < 0.004, "offset {}", s.offset);
    assert!((s.focal - focal).abs() / focal < 0.05, "focal {}", s.focal);
    assert!(s.score < 0.05, "score {}", s.score);

    // Gyro-only plan using the raw data and auto-sync.
    let track = MotionTrack { motions, ..MotionTrack::empty(640, 360, fps, n) };
    let mut c = Config::preset("gyro").unwrap();
    c.analysis.motion_source = MotionSource::Gyro;
    let plan = Plan::build(&c, &track, Some(&raw)).unwrap();
    assert!(plan.stats.gyro_sync.is_some());
    assert!(plan.stats.reduction_pct > 50.0, "{}", plan.stats.reduction_pct);
}
