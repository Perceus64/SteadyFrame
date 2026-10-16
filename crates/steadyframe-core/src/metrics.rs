//! Shakiness metrics used in reports (before vs after).

use crate::geometry::{Params, LSX, ROT, TX, TY};
use crate::smooth::gaussian;
use serde::{Deserialize, Serialize};

/// High-frequency camera motion (RMS of the path minus its low-passed version).
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Jitter {
    /// Combined x/y jitter in full-resolution pixels.
    pub translation_px: f64,
    pub rotation_deg: f64,
    pub zoom_pct: f64,
}

fn hf_rms(x: &[f64]) -> f64 {
    if x.len() < 3 {
        return 0.0;
    }
    let lp = gaussian(x, 4.0);
    (x.iter().zip(&lp).map(|(a, b)| (a - b).powi(2)).sum::<f64>() / x.len() as f64).sqrt()
}

pub fn jitter(path: &[Params]) -> Jitter {
    let ch = |c: usize| path.iter().map(|p| p.0[c]).collect::<Vec<_>>();
    let tx = hf_rms(&ch(TX));
    let ty = hf_rms(&ch(TY));
    Jitter {
        translation_px: (tx * tx + ty * ty).sqrt(),
        rotation_deg: hf_rms(&ch(ROT)).to_degrees(),
        zoom_pct: hf_rms(&ch(LSX)) * 100.0,
    }
}

/// Percentage reduction of combined jitter (translation + rotation scaled to px).
pub fn reduction_pct(before: &Jitter, after: &Jitter, width: f64) -> f64 {
    // Express rotation as edge displacement so both terms share units.
    let rot_px = |j: &Jitter| j.rotation_deg.to_radians() * width / 2.0;
    let b = (before.translation_px.powi(2) + rot_px(before).powi(2)).sqrt();
    let a = (after.translation_px.powi(2) + rot_px(after).powi(2)).sqrt();
    if b < 1e-9 {
        0.0
    } else {
        (1.0 - a / b) * 100.0
    }
}
