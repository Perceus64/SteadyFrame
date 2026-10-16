//! 3×3 projective transforms and a smoothing-friendly parameterisation.
//!
//! All transforms in the engine work in *centred* pixel coordinates (origin at
//! the image centre, x right, y down) so rotation and zoom happen about the
//! centre of the frame.

use serde::{Deserialize, Serialize};

/// Row-major 3×3 matrix.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mat3(pub [f64; 9]);

impl Default for Mat3 {
    fn default() -> Self {
        Mat3::IDENTITY
    }
}

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);

    pub fn translation(tx: f64, ty: f64) -> Mat3 {
        Mat3([1.0, 0.0, tx, 0.0, 1.0, ty, 0.0, 0.0, 1.0])
    }

    pub fn scale(sx: f64, sy: f64) -> Mat3 {
        Mat3([sx, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 1.0])
    }

    pub fn rotation(theta: f64) -> Mat3 {
        let (s, c) = theta.sin_cos();
        Mat3([c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0])
    }

    /// `self * o` (apply `o` first, then `self`).
    pub fn mul(&self, o: &Mat3) -> Mat3 {
        let a = &self.0;
        let b = &o.0;
        let mut r = [0.0; 9];
        for i in 0..3 {
            for j in 0..3 {
                r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
            }
        }
        Mat3(r)
    }

    pub fn det(&self) -> f64 {
        let m = &self.0;
        m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6])
    }

    pub fn inverse(&self) -> Option<Mat3> {
        let m = &self.0;
        let det = self.det();
        if !det.is_finite() || det.abs() < 1e-14 {
            return None;
        }
        let inv = 1.0 / det;
        Some(Mat3([
            (m[4] * m[8] - m[5] * m[7]) * inv,
            (m[2] * m[7] - m[1] * m[8]) * inv,
            (m[1] * m[5] - m[2] * m[4]) * inv,
            (m[5] * m[6] - m[3] * m[8]) * inv,
            (m[0] * m[8] - m[2] * m[6]) * inv,
            (m[2] * m[3] - m[0] * m[5]) * inv,
            (m[3] * m[7] - m[4] * m[6]) * inv,
            (m[1] * m[6] - m[0] * m[7]) * inv,
            (m[0] * m[4] - m[1] * m[3]) * inv,
        ]))
    }

    /// Inverse, falling back to identity for singular matrices.
    pub fn inverse_or_identity(&self) -> Mat3 {
        self.inverse().unwrap_or(Mat3::IDENTITY)
    }

    /// Transform a point (with perspective divide).
    #[inline]
    pub fn apply(&self, x: f64, y: f64) -> (f64, f64) {
        let m = &self.0;
        let w = m[6] * x + m[7] * y + m[8];
        let w = if w.abs() < 1e-12 { 1e-12 } else { w };
        ((m[0] * x + m[1] * y + m[2]) / w, (m[3] * x + m[4] * y + m[5]) / w)
    }

    /// Scale so that the bottom-right element is 1.
    pub fn normalized(&self) -> Mat3 {
        let k = self.0[8];
        if k.abs() < 1e-14 {
            return *self;
        }
        let mut r = self.0;
        for v in r.iter_mut() {
            *v /= k;
        }
        Mat3(r)
    }

    /// Change of coordinate scale: if `self` maps points in a frame of one size,
    /// returns the equivalent transform for coordinates multiplied by `s`
    /// (S·H·S⁻¹ with S = diag(s, s, 1)).
    pub fn rescaled(&self, s: f64) -> Mat3 {
        let m = &self.0;
        Mat3([m[0], m[1], m[2] * s, m[3], m[4], m[5] * s, m[6] / s, m[7] / s, m[8]])
    }

    pub fn is_finite(&self) -> bool {
        self.0.iter().all(|v| v.is_finite())
    }

    pub fn is_affine(&self) -> bool {
        self.0[6].abs() < 1e-12 && self.0[7].abs() < 1e-12
    }
}

pub const NPARAMS: usize = 8;
pub const TX: usize = 0;
pub const TY: usize = 1;
pub const ROT: usize = 2;
pub const LSX: usize = 3;
pub const LSY: usize = 4;
pub const SHEAR: usize = 5;
pub const PX: usize = 6;
pub const PY: usize = 7;

/// Additive parameterisation of a homography for path smoothing:
/// `[tx, ty, rotation (rad), ln sx, ln sy, shear, persp_x, persp_y]`.
///
/// The linear part is factored as `A = R(θ) · [[sx, k], [0, sy]]`; identity is
/// all zeros, and each parameter has an intuitive meaning, so smoothing each
/// channel independently behaves well.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Params(pub [f64; NPARAMS]);

impl Params {
    pub fn from_mat(m: &Mat3) -> Params {
        let m = m.normalized();
        let [a, b, tx, c, d, ty, g, h, _] = m.0;
        let theta = c.atan2(a);
        let (s, co) = theta.sin_cos();
        let sx = (a * a + c * c).sqrt().max(1e-9);
        let k = co * b + s * d;
        let sy = (-s * b + co * d).abs().max(1e-9);
        Params([tx, ty, theta, sx.ln(), sy.ln(), k, g, h])
    }

    pub fn to_mat(&self) -> Mat3 {
        let p = &self.0;
        let (s, c) = p[ROT].sin_cos();
        let sx = p[LSX].exp();
        let sy = p[LSY].exp();
        let k = p[SHEAR];
        Mat3([c * sx, c * k - s * sy, p[TX], s * sx, s * k + c * sy, p[TY], p[PX], p[PY], 1.0])
    }

    pub fn scaled(&self, f: f64) -> Params {
        let mut r = self.0;
        for v in r.iter_mut() {
            *v *= f;
        }
        Params(r)
    }

    pub fn add(&self, o: &Params) -> Params {
        let mut r = self.0;
        for (a, b) in r.iter_mut().zip(o.0.iter()) {
            *a += *b;
        }
        Params(r)
    }
}

/// Unwrap an angle so it's within π of `reference`.
pub fn unwrap_angle(angle: f64, reference: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    angle + two_pi * ((reference - angle) / two_pi).round()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &Mat3, b: &Mat3, eps: f64) -> bool {
        a.0.iter().zip(b.0.iter()).all(|(x, y)| (x - y).abs() < eps)
    }

    #[test]
    fn inverse_roundtrip() {
        let m = Mat3([1.1, 0.2, 5.0, -0.1, 0.9, -3.0, 1e-4, -2e-4, 1.0]);
        let i = m.inverse().unwrap();
        assert!(close(&m.mul(&i), &Mat3::IDENTITY, 1e-9));
    }

    #[test]
    fn params_roundtrip() {
        for m in [
            Mat3::IDENTITY,
            Mat3::rotation(0.3).mul(&Mat3::translation(4.0, -7.0)),
            Mat3([1.05, 0.03, 2.0, -0.02, 0.97, 1.0, 2e-5, -1e-5, 1.0]),
            Mat3::rotation(-2.9).mul(&Mat3::scale(1.2, 0.8)),
        ] {
            let p = Params::from_mat(&m);
            assert!(close(&p.to_mat(), &m.normalized(), 1e-9), "{m:?} -> {p:?}");
        }
    }

    #[test]
    fn rescale_matches_coordinate_change() {
        let m = Mat3([1.01, 0.02, 3.0, -0.02, 0.99, -1.5, 1e-4, 2e-4, 1.0]);
        let s = 3.0;
        let big = m.rescaled(s);
        let (x, y) = m.apply(10.0, -20.0);
        let (bx, by) = big.apply(30.0, -60.0);
        assert!((bx - x * s).abs() < 1e-9 && (by - y * s).abs() < 1e-9);
    }

    #[test]
    fn unwrap() {
        let a = unwrap_angle(-3.1, 3.1);
        assert!((a - (-3.1 + std::f64::consts::TAU)).abs() < 1e-12);
    }
}
