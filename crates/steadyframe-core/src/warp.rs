//! Frame warping (RGBA) with interpolation, border handling and per-row
//! rolling-shutter correction, plus comparison layouts.

use crate::config::{BorderMode, CompareMode, Interpolation};
use crate::geometry::{Mat3, Params};

/// Lookup table of per-row rolling-shutter corrections.
///
/// A row captured `τ` frame-intervals away from mid-exposure saw the scene
/// displaced by the camera motion over `τ`; `velocity` is the per-frame
/// motion (as parameters), so the row's displacement is `velocity · τ`.
pub struct RsLut {
    mats: Vec<Mat3>,
    h: f64,
}

impl RsLut {
    const N: usize = 129;

    pub fn new(velocity: &Params, readout: f64, h: f64) -> RsLut {
        let mats = (0..Self::N)
            .map(|i| {
                let t = i as f64 / (Self::N - 1) as f64 - 0.5; // -0.5..0.5 of the frame height
                velocity.scaled(readout * t).to_mat()
            })
            .collect();
        RsLut { mats, h }
    }

    #[inline]
    fn get(&self, y_centered: f64) -> &Mat3 {
        let t = (y_centered / self.h + 0.5).clamp(0.0, 1.0);
        &self.mats[(t * (Self::N - 1) as f64).round() as usize]
    }
}

/// Everything needed to warp one frame.
pub struct WarpSpec<'a> {
    /// Output-centred → input-centred pixel transform.
    pub inv: Mat3,
    pub rs: Option<&'a RsLut>,
    pub interpolation: Interpolation,
    pub border: BorderMode,
    pub border_color: [u8; 4],
}

#[inline]
fn map_index(i: isize, n: isize, border: BorderMode) -> Option<usize> {
    if i >= 0 && i < n {
        return Some(i as usize);
    }
    match border {
        BorderMode::Reflect => {
            if n == 1 {
                return Some(0);
            }
            let period = 2 * n;
            let mut j = i.rem_euclid(period);
            if j >= n {
                j = period - 1 - j;
            }
            Some(j as usize)
        }
        _ => Some(i.clamp(0, n - 1) as usize),
    }
}

#[inline]
fn px(src: &[u8], w: usize, x: usize, y: usize) -> [f32; 4] {
    let i = (y * w + x) * 4;
    [src[i] as f32, src[i + 1] as f32, src[i + 2] as f32, src[i + 3] as f32]
}

#[inline]
fn cubic_w(t: f32) -> [f32; 4] {
    // Catmull–Rom
    let t2 = t * t;
    let t3 = t2 * t;
    [-0.5 * t3 + t2 - 0.5 * t, 1.5 * t3 - 2.5 * t2 + 1.0, -1.5 * t3 + 2.0 * t2 + 0.5 * t, 0.5 * t3 - 0.5 * t2]
}

/// Sample at continuous pixel coordinates (pixel centres at integers).
/// Returns None when outside the image for fill-type border modes.
#[inline]
fn sample(
    src: &[u8],
    w: usize,
    h: usize,
    x: f64,
    y: f64,
    interp: Interpolation,
    border: BorderMode,
) -> Option<[f32; 4]> {
    let fill_mode = matches!(border, BorderMode::Black | BorderMode::Color | BorderMode::Previous);
    if fill_mode && (x < -0.5 || y < -0.5 || x > w as f64 - 0.5 || y > h as f64 - 0.5) {
        return None;
    }
    if !x.is_finite() || !y.is_finite() {
        return None;
    }
    let (wi, hi) = (w as isize, h as isize);
    match interp {
        Interpolation::Nearest => {
            let xi = map_index(x.round() as isize, wi, border)?;
            let yi = map_index(y.round() as isize, hi, border)?;
            Some(px(src, w, xi, yi))
        }
        Interpolation::Bilinear => {
            let x0 = x.floor();
            let y0 = y.floor();
            let fx = (x - x0) as f32;
            let fy = (y - y0) as f32;
            let (xi, yi) = (x0 as isize, y0 as isize);
            if xi >= 0 && yi >= 0 && xi + 1 < wi && yi + 1 < hi {
                // Fast path: all four taps inside the image.
                let i = (yi as usize * w + xi as usize) * 4;
                let j = i + w * 4;
                let s = &src[i..i + 8];
                let t = &src[j..j + 8];
                let mut o = [0.0f32; 4];
                for k in 0..4 {
                    let top = s[k] as f32 + (s[k + 4] as f32 - s[k] as f32) * fx;
                    let bot = t[k] as f32 + (t[k + 4] as f32 - t[k] as f32) * fx;
                    o[k] = top + (bot - top) * fy;
                }
                return Some(o);
            }
            let xa = map_index(x0 as isize, wi, border)?;
            let xb = map_index(x0 as isize + 1, wi, border)?;
            let ya = map_index(y0 as isize, hi, border)?;
            let yb = map_index(y0 as isize + 1, hi, border)?;
            let a = px(src, w, xa, ya);
            let b = px(src, w, xb, ya);
            let c = px(src, w, xa, yb);
            let d = px(src, w, xb, yb);
            let mut o = [0.0f32; 4];
            for k in 0..4 {
                let top = a[k] + (b[k] - a[k]) * fx;
                let bot = c[k] + (d[k] - c[k]) * fx;
                o[k] = top + (bot - top) * fy;
            }
            Some(o)
        }
        Interpolation::Bicubic => {
            let x0 = x.floor();
            let y0 = y.floor();
            let wx = cubic_w((x - x0) as f32);
            let wy = cubic_w((y - y0) as f32);
            let mut o = [0.0f32; 4];
            for (j, wyj) in wy.iter().enumerate() {
                let yy = map_index(y0 as isize + j as isize - 1, hi, border)?;
                let mut row = [0.0f32; 4];
                for (i, wxi) in wx.iter().enumerate() {
                    let xx = map_index(x0 as isize + i as isize - 1, wi, border)?;
                    let p = px(src, w, xx, yy);
                    for k in 0..4 {
                        row[k] += wxi * p[k];
                    }
                }
                for k in 0..4 {
                    o[k] += wyj * row[k];
                }
            }
            Some(o)
        }
    }
}

fn warp_row(src: &[u8], sw: usize, sh: usize, row: &mut [u8], oy: usize, dw: usize, dh: usize, spec: &WarpSpec) {
    let qy = oy as f64 + 0.5 - dh as f64 / 2.0;
    let (hw, hh) = (sw as f64 / 2.0, sh as f64 / 2.0);
    let fill = match spec.border {
        BorderMode::Color => spec.border_color,
        _ => [0, 0, 0, 255],
    };
    // The homography numerators/denominator are linear in x: step them.
    let m = &spec.inv.0;
    let qx0 = 0.5 - dw as f64 / 2.0;
    let mut nx = m[0] * qx0 + m[1] * qy + m[2];
    let mut ny = m[3] * qx0 + m[4] * qy + m[5];
    let mut nw = m[6] * qx0 + m[7] * qy + m[8];
    let (wi, hi) = (sw as isize, sh as isize);
    let fast_bilinear = spec.interpolation == Interpolation::Bilinear;
    for ox in 0..dw {
        let iw = if nw.abs() < 1e-12 { 1e12 } else { 1.0 / nw };
        let (mut x, mut y) = (nx * iw, ny * iw);
        nx += m[0];
        ny += m[3];
        nw += m[6];
        if let Some(lut) = spec.rs {
            let r = lut.get(y).apply(x, y);
            // One fixed-point refinement: use the row the sample actually lands on.
            let r2 = lut.get(r.1).apply(x, y);
            x = r2.0;
            y = r2.1;
        }
        let o = &mut row[ox * 4..ox * 4 + 4];
        let (sx, sy) = (x + hw - 0.5, y + hh - 0.5);
        if fast_bilinear && sx >= 0.0 && sy >= 0.0 {
            let (xi, yi) = (sx as isize, sy as isize);
            if xi + 1 < wi && yi + 1 < hi {
                let fx = (sx - xi as f64) as f32;
                let fy = (sy - yi as f64) as f32;
                let i = (yi as usize * sw + xi as usize) * 4;
                let j = i + sw * 4;
                let a = &src[i..i + 8];
                let b = &src[j..j + 8];
                for k in 0..4 {
                    let top = a[k] as f32 + (a[k + 4] as f32 - a[k] as f32) * fx;
                    let bot = b[k] as f32 + (b[k + 4] as f32 - b[k] as f32) * fx;
                    o[k] = (top + (bot - top) * fy + 0.5) as u8;
                }
                continue;
            }
        }
        match sample(src, sw, sh, sx, sy, spec.interpolation, spec.border) {
            Some(p) => {
                for k in 0..4 {
                    o[k] = (p[k].clamp(0.0, 255.0) + 0.5) as u8;
                }
            }
            None => {
                if spec.border != BorderMode::Previous {
                    o.copy_from_slice(&fill);
                }
            }
        }
    }
}

/// Warp `src` (sw×sh RGBA) into `dst` (dw×dh RGBA).
/// With `BorderMode::Previous`, `dst` should hold the previous output frame.
pub fn warp_rgba(src: &[u8], sw: usize, sh: usize, dst: &mut [u8], dw: usize, dh: usize, spec: &WarpSpec) {
    let stride = dw * 4;
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        dst.par_chunks_mut(stride).enumerate().for_each(|(oy, row)| warp_row(src, sw, sh, row, oy, dw, dh, spec));
    }
    #[cfg(not(feature = "parallel"))]
    {
        for (oy, row) in dst.chunks_mut(stride).enumerate() {
            warp_row(src, sw, sh, row, oy, dw, dh, spec);
        }
    }
}

/// Bilinear RGBA resize.
pub fn resize_rgba(src: &[u8], sw: usize, sh: usize, dw: usize, dh: usize) -> Vec<u8> {
    if sw == dw && sh == dh {
        return src.to_vec();
    }
    let mut out = vec![0u8; dw * dh * 4];
    let inv = Mat3::scale(sw as f64 / dw as f64, sh as f64 / dh as f64);
    let spec = WarpSpec {
        inv,
        rs: None,
        interpolation: Interpolation::Bilinear,
        border: BorderMode::Replicate,
        border_color: [0, 0, 0, 255],
    };
    warp_rgba(src, sw, sh, &mut out, dw, dh, &spec);
    out
}

/// Size of the comparison output for a stabilized frame of `w × h`.
pub fn compare_size(mode: CompareMode, w: usize, h: usize) -> (usize, usize) {
    match mode {
        CompareMode::SideBySide => (w * 2, h),
        _ => (w, h),
    }
}

/// Combine the original (any size) and stabilized frame into a comparison view.
pub fn compose_compare(
    orig: &[u8],
    ow: usize,
    oh: usize,
    stab: &[u8],
    w: usize,
    h: usize,
    mode: CompareMode,
) -> (Vec<u8>, usize, usize) {
    match mode {
        CompareMode::None => (stab.to_vec(), w, h),
        CompareMode::SideBySide => {
            let o = resize_rgba(orig, ow, oh, w, h);
            let mut out = vec![0u8; w * 2 * h * 4];
            for y in 0..h {
                let dst = &mut out[y * w * 8..(y + 1) * w * 8];
                dst[..w * 4].copy_from_slice(&o[y * w * 4..(y + 1) * w * 4]);
                dst[w * 4..].copy_from_slice(&stab[y * w * 4..(y + 1) * w * 4]);
            }
            (out, w * 2, h)
        }
        CompareMode::Split => {
            let o = resize_rgba(orig, ow, oh, w, h);
            let mut out = stab.to_vec();
            let half = w / 2;
            for y in 0..h {
                let r = y * w * 4;
                out[r..r + half * 4].copy_from_slice(&o[r..r + half * 4]);
                for x in half.saturating_sub(1)..(half + 1).min(w) {
                    out[r + x * 4..r + x * 4 + 4].copy_from_slice(&[255, 255, 255, 255]);
                }
            }
            (out, w, h)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checker(w: usize, h: usize) -> Vec<u8> {
        let mut v = vec![0u8; w * h * 4];
        for y in 0..h {
            for x in 0..w {
                let c = if (x / 4 + y / 4) % 2 == 0 { 255 } else { 0 };
                v[(y * w + x) * 4..(y * w + x) * 4 + 4].copy_from_slice(&[c, x as u8, y as u8, 255]);
            }
        }
        v
    }

    #[test]
    fn identity_warp_is_exact() {
        let (w, h) = (32, 24);
        let src = checker(w, h);
        for interp in [Interpolation::Nearest, Interpolation::Bilinear, Interpolation::Bicubic] {
            let mut dst = vec![0u8; w * h * 4];
            let spec = WarpSpec {
                inv: Mat3::IDENTITY,
                rs: None,
                interpolation: interp,
                border: BorderMode::Black,
                border_color: [0; 4],
            };
            warp_rgba(&src, w, h, &mut dst, w, h, &spec);
            assert_eq!(src, dst, "{interp:?}");
        }
    }

    #[test]
    fn integer_shift_and_borders() {
        let (w, h) = (16, 8);
        let src = checker(w, h);
        let mut dst = vec![7u8; w * h * 4];
        // Output pixel x samples input x+3.
        let spec = WarpSpec {
            inv: Mat3::translation(3.0, 0.0),
            rs: None,
            interpolation: Interpolation::Bilinear,
            border: BorderMode::Previous,
            border_color: [0; 4],
        };
        warp_rgba(&src, w, h, &mut dst, w, h, &spec);
        assert_eq!(&dst[0..4], &src[12..16]);
        assert_eq!(&dst[(w - 1) * 4..w * 4], &[7, 7, 7, 7]);
        let spec = WarpSpec { border: BorderMode::Color, border_color: [1, 2, 3, 4], ..spec };
        warp_rgba(&src, w, h, &mut dst, w, h, &spec);
        assert_eq!(&dst[(w - 1) * 4..w * 4], &[1, 2, 3, 4]);
    }

    #[test]
    fn compare_layouts() {
        let a = vec![10u8; 8 * 4 * 4];
        let b = vec![200u8; 8 * 4 * 4];
        let (o, w, h) = compose_compare(&a, 8, 4, &b, 8, 4, CompareMode::SideBySide);
        assert_eq!((w, h), (16, 4));
        assert_eq!(o[0], 10);
        assert_eq!(o[8 * 4], 200);
    }
}
