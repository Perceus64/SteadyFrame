//! Automatic zoom so the corrected frames cover the output without borders.

use crate::config::{CropConfig, CropMode};
use crate::geometry::Mat3;
use crate::smooth::gaussian;

/// Smallest zoom `z ∈ [1, max_zoom]` such that the output rectangle
/// (half-extents `hw × hh`, in input-pixel units) shrunk by `z` and mapped
/// through `inv` (output → input, centred coords) stays inside the input
/// frame (half-extents `iw × ih`). Returns `max_zoom` if even that isn't enough.
pub fn required_zoom(inv: &Mat3, hw: f64, hh: f64, iw: f64, ih: f64, max_zoom: f64) -> f64 {
    let ok = |z: f64| {
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (0.0, -1.0), (0.0, 1.0), (-1.0, 0.0), (1.0, 0.0)]
            .iter()
            .all(|&(sx, sy)| {
                let (x, y) = inv.apply(sx * hw / z, sy * hh / z);
                x.is_finite() && y.is_finite() && x.abs() <= iw + 1e-6 && y.abs() <= ih + 1e-6
            })
    };
    if ok(1.0) {
        return 1.0;
    }
    if !ok(max_zoom) {
        return max_zoom;
    }
    let (mut lo, mut hi) = (1.0, max_zoom);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if ok(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

/// Turn per-frame required zooms into the zoom actually used per frame.
pub fn zoom_series(required: &[f64], cfg: &CropConfig) -> Vec<f64> {
    let n = required.len();
    if n == 0 {
        return Vec::new();
    }
    let base: Vec<f64> = match cfg.mode {
        CropMode::None => vec![1.0; n],
        CropMode::Fixed => {
            let m = required.iter().cloned().fold(1.0, f64::max);
            vec![m; n]
        }
        CropMode::Dynamic => {
            let sigma = (cfg.dynamic_smoothing / 2.0).max(1.0);
            let r = (2.0 * sigma).ceil() as usize;
            // Max-filter first so the smoothed curve stays above the requirement.
            let maxed: Vec<f64> = (0..n)
                .map(|i| {
                    let a = i.saturating_sub(r);
                    let b = (i + r + 1).min(n);
                    required[a..b].iter().cloned().fold(1.0, f64::max)
                })
                .collect();
            let sm = gaussian(&maxed, sigma);
            sm.iter().zip(required).map(|(s, r)| s.max(*r)).collect()
        }
    };
    base.iter().map(|z| (z * cfg.extra_zoom).clamp(0.1, cfg.max_zoom.max(cfg.extra_zoom))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zoom_for_shift_and_rotation() {
        // Shifted 10% of width: need ≥ 1/(1-0.2) zoom on x.
        let inv = Mat3::translation(64.0, 0.0);
        let z = required_zoom(&inv, 320.0, 180.0, 320.0, 180.0, 2.0);
        assert!((z - 320.0 / 256.0).abs() < 1e-3, "{z}");
        assert_eq!(required_zoom(&Mat3::IDENTITY, 320.0, 180.0, 320.0, 180.0, 2.0), 1.0);
        let rot = Mat3::rotation(0.05);
        let z = required_zoom(&rot, 320.0, 180.0, 320.0, 180.0, 2.0);
        assert!(z > 1.05 && z < 1.2, "{z}");
    }

    #[test]
    fn dynamic_covers_requirement() {
        let req: Vec<f64> = (0..200).map(|i| if (80..90).contains(&i) { 1.3 } else { 1.05 }).collect();
        let cfg = CropConfig { mode: CropMode::Dynamic, max_zoom: 2.0, ..Default::default() };
        let z = zoom_series(&req, &cfg);
        assert!(z.iter().zip(&req).all(|(a, b)| a >= b));
        assert!(z[0] < 1.2);
    }
}
