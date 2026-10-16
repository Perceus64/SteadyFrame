//! Camera-path smoothing: Gaussian, moving average, Kalman/RTS and L1-optimal,
//! plus tripod/horizon/zoom locks and correction limits.

use crate::config::{SmoothingConfig, SmoothingMethod};
use crate::geometry::{Params, LSX, LSY, NPARAMS, PX, PY, ROT, SHEAR, TX, TY};

/// Pad `x` with `r` samples on each side using odd reflection about a robust
/// end-point anchor (a least-squares line through the first/last `fit`
/// samples). This keeps linear trends at the edges, so the start/end of a pan
/// isn't pulled back, and doesn't latch onto a noisy first/last frame.
fn padded(x: &[f64], r: usize, fit: usize) -> Vec<f64> {
    let n = x.len();
    if n < 2 {
        return vec![x.first().copied().unwrap_or(0.0); n + 2 * r];
    }
    let m = fit.max(2).min(n);
    let line_at = |idx: &[usize], at: f64| -> f64 {
        let k = idx.len() as f64;
        let mx = idx.iter().map(|&i| i as f64).sum::<f64>() / k;
        let my = idx.iter().map(|&i| x[i]).sum::<f64>() / k;
        let (mut sxy, mut sxx) = (0.0, 0.0);
        for &i in idx {
            sxy += (i as f64 - mx) * (x[i] - my);
            sxx += (i as f64 - mx).powi(2);
        }
        let b = if sxx > 0.0 { sxy / sxx } else { 0.0 };
        my + b * (at - mx)
    };
    let head: Vec<usize> = (0..m).collect();
    let tail: Vec<usize> = (n - m..n).collect();
    let a0 = line_at(&head, 0.0);
    let a1 = line_at(&tail, (n - 1) as f64);
    let mut out = Vec::with_capacity(n + 2 * r);
    for k in (1..=r).rev() {
        let j = k.min(n - 1);
        out.push(2.0 * a0 - x[j]);
    }
    out.extend_from_slice(x);
    for k in 1..=r {
        let j = (n - 1).saturating_sub(k);
        out.push(2.0 * a1 - x[j]);
    }
    out
}

pub fn gaussian(x: &[f64], sigma: f64) -> Vec<f64> {
    if sigma < 1e-6 || x.len() < 3 {
        return x.to_vec();
    }
    let r = (3.0 * sigma).ceil() as usize;
    let kernel: Vec<f64> = (0..=2 * r)
        .map(|k| {
            let d = k as f64 - r as f64;
            (-(d * d) / (2.0 * sigma * sigma)).exp()
        })
        .collect();
    let norm: f64 = kernel.iter().sum();
    let p = padded(x, r, (sigma.ceil() as usize).max(3));
    (0..x.len()).map(|i| p[i..i + 2 * r + 1].iter().zip(&kernel).map(|(a, b)| a * b).sum::<f64>() / norm).collect()
}

pub fn moving_average(x: &[f64], radius: usize) -> Vec<f64> {
    if radius == 0 || x.len() < 3 {
        return x.to_vec();
    }
    let r = radius;
    let n = (2 * r + 1) as f64;
    let p = padded(x, r + 1, radius.max(3));
    // p index of x[i] is i + r + 1.
    let mut sum: f64 = p[1..2 * r + 2].iter().sum();
    let mut out = Vec::with_capacity(x.len());
    for i in 0..x.len() {
        out.push(sum / n);
        if i + 1 < x.len() {
            sum += p[i + 2 * r + 2] - p[i + 1];
        }
    }
    out
}

/// Constant-velocity Kalman filter followed by an RTS smoother.
/// `ratio` = process noise / measurement noise (smaller = smoother).
pub fn kalman_rts(x: &[f64], ratio: f64) -> Vec<f64> {
    let n = x.len();
    if n < 3 || ratio <= 0.0 {
        return x.to_vec();
    }
    let r = 1.0;
    let q = ratio;
    let qm = [q / 3.0, q / 2.0, q / 2.0, q];
    // Forward pass
    let mut xs = vec![[0.0f64; 2]; n];
    let mut ps = vec![[0.0f64; 4]; n];
    let mut xp = vec![[0.0f64; 2]; n];
    let mut pp = vec![[0.0f64; 4]; n];
    let mut s = [x[0], 0.0];
    let mut p = [r, 0.0, 0.0, 1e2];
    for k in 0..n {
        if k > 0 {
            // predict: F = [[1,1],[0,1]]
            s = [s[0] + s[1], s[1]];
            let (a, b, c, d) = (p[0], p[1], p[2], p[3]);
            // F P F^T
            p = [a + b + c + d + qm[0], b + d + qm[1], c + d + qm[2], d + qm[3]];
        }
        xp[k] = s;
        pp[k] = p;
        // update with H = [1, 0]
        let sk = p[0] + r;
        let k0 = p[0] / sk;
        let k1 = p[2] / sk;
        let innov = x[k] - s[0];
        s = [s[0] + k0 * innov, s[1] + k1 * innov];
        p = [(1.0 - k0) * p[0], (1.0 - k0) * p[1], p[2] - k1 * p[0], p[3] - k1 * p[1]];
        xs[k] = s;
        ps[k] = p;
    }
    // Backward RTS pass
    let mut out = vec![0.0; n];
    let mut sm = xs[n - 1];
    out[n - 1] = sm[0];
    for k in (0..n - 1).rev() {
        let p = ps[k];
        let pn = pp[k + 1];
        // C = P F^T Pn^{-1}
        let pft = [p[0] + p[1], p[1], p[2] + p[3], p[3]];
        let det = pn[0] * pn[3] - pn[1] * pn[2];
        if det.abs() < 1e-18 {
            sm = xs[k];
            out[k] = sm[0];
            continue;
        }
        let inv = [pn[3] / det, -pn[1] / det, -pn[2] / det, pn[0] / det];
        let c = [
            pft[0] * inv[0] + pft[1] * inv[2],
            pft[0] * inv[1] + pft[1] * inv[3],
            pft[2] * inv[0] + pft[3] * inv[2],
            pft[2] * inv[1] + pft[3] * inv[3],
        ];
        let dx = [sm[0] - xp[k + 1][0], sm[1] - xp[k + 1][1]];
        sm = [xs[k][0] + c[0] * dx[0] + c[1] * dx[1], xs[k][1] + c[2] * dx[0] + c[3] * dx[1]];
        out[k] = sm[0];
    }
    out
}

const D1: [f64; 2] = [-1.0, 1.0];
const D2: [f64; 3] = [1.0, -2.0, 1.0];
const D3: [f64; 4] = [-1.0, 3.0, -3.0, 1.0];

fn dapply(c: &[f64], p: &[f64]) -> Vec<f64> {
    let m = c.len();
    if p.len() < m {
        return Vec::new();
    }
    (0..=p.len() - m).map(|i| c.iter().enumerate().map(|(j, cj)| cj * p[i + j]).sum()).collect()
}

fn dapply_t(c: &[f64], y: &[f64], n: usize, out: &mut [f64]) {
    for (i, &yi) in y.iter().enumerate() {
        for (j, cj) in c.iter().enumerate() {
            out[i + j] += cj * yi;
        }
    }
    let _ = n;
}

/// L1-optimal camera path via ADMM:
///
/// minimise  λ/2‖p − c‖² + w₁‖D₁p‖₁ + w₂‖D₂p‖₁ + w₃‖D₃p‖₁   s.t. |p − c| ≤ bound
///
/// The L1 penalties on velocity, acceleration and jerk produce paths made of
/// static, constant-velocity and smoothly-easing segments (Grundmann et al.,
/// "Auto-Directed Video Stabilization with Robust L1 Optimal Camera Paths").
///
/// The allowed tube `|p − c|` is the smaller of `bound` and the correction a
/// Gaussian smoother of the same `radius` would need (×2.5), so `radius` acts
/// as the strength knob exactly like for the other methods.
pub fn l1_path(c: &[f64], w: [f64; 3], radius: f64, bound: Option<f64>, iters: usize) -> Vec<f64> {
    l1_path_rho(c, w, radius, bound, iters, 1.0)
}

pub fn l1_path_rho(c: &[f64], w: [f64; 3], radius: f64, bound: Option<f64>, iters: usize, rho: f64) -> Vec<f64> {
    let n = c.len();
    if n < 5 {
        return c.to_vec();
    }
    // Scale = RMS correction of an equivalent Gaussian smoother; this makes the
    // weights scale-free and defines the default tube width.
    let mean = c.iter().sum::<f64>() / n as f64;
    let g = gaussian(c, (radius / 2.0).max(0.5));
    let dev = (c.iter().zip(&g).map(|(a, b)| (a - b).powi(2)).sum::<f64>() / n as f64).sqrt();
    let peak = c.iter().fold(0.0f64, |m, v| m.max((v - mean).abs()));
    let s = dev.max(1e-6 * (1.0 + peak)).max(1e-12);
    if dev < 1e-9 {
        return c.to_vec();
    }
    let cn: Vec<f64> = c.iter().map(|v| (v - mean) / s).collect();
    let tube = 2.5;
    let bn = Some(bound.map(|b| (b / s).min(tube)).unwrap_or(tube));
    let lambda = 1e-3;

    let mut p = gaussian(&cn, (radius / 2.0).max(0.5));
    let ops: [&[f64]; 3] = [&D1, &D2, &D3];
    let mut z: Vec<Vec<f64>> = ops.iter().map(|o| dapply(o, &p)).collect();
    let mut u: Vec<Vec<f64>> = z.iter().map(|v| vec![0.0; v.len()]).collect();
    let mut z0 = p.clone();
    let mut u0 = vec![0.0; n];

    let apply_a = |v: &[f64], out: &mut Vec<f64>| {
        out.clear();
        out.extend(v.iter().map(|x| lambda * x + if bn.is_some() { rho * x } else { 0.0 }));
        for o in ops.iter() {
            let d = dapply(o, v);
            let mut t = vec![0.0; n];
            dapply_t(o, &d, n, &mut t);
            for i in 0..n {
                out[i] += rho * t[i];
            }
        }
    };

    let mut rhs = vec![0.0; n];
    let mut ap = Vec::with_capacity(n);
    for _ in 0..iters.max(1) {
        // p-update: solve A p = rhs with conjugate gradients (A is SPD).
        for i in 0..n {
            rhs[i] = lambda * cn[i] + if bn.is_some() { rho * (z0[i] - u0[i]) } else { 0.0 };
        }
        for k in 0..3 {
            let diff: Vec<f64> = z[k].iter().zip(&u[k]).map(|(a, b)| a - b).collect();
            let mut t = vec![0.0; n];
            dapply_t(ops[k], &diff, n, &mut t);
            for i in 0..n {
                rhs[i] += rho * t[i];
            }
        }
        apply_a(&p, &mut ap);
        let mut r: Vec<f64> = rhs.iter().zip(&ap).map(|(a, b)| a - b).collect();
        let mut d = r.clone();
        let mut rr: f64 = r.iter().map(|v| v * v).sum();
        for _ in 0..40 {
            if rr < 1e-10 * n as f64 {
                break;
            }
            apply_a(&d, &mut ap);
            let dad: f64 = d.iter().zip(&ap).map(|(a, b)| a * b).sum();
            if dad.abs() < 1e-30 {
                break;
            }
            let alpha = rr / dad;
            for i in 0..n {
                p[i] += alpha * d[i];
                r[i] -= alpha * ap[i];
            }
            let rr2: f64 = r.iter().map(|v| v * v).sum();
            let beta = rr2 / rr;
            rr = rr2;
            for i in 0..n {
                d[i] = r[i] + beta * d[i];
            }
        }
        // z-updates (soft threshold) and dual updates.
        for k in 0..3 {
            let dp = dapply(ops[k], &p);
            let t = w[k] / rho;
            for i in 0..dp.len() {
                let v = dp[i] + u[k][i];
                let zz = if v > t {
                    v - t
                } else if v < -t {
                    v + t
                } else {
                    0.0
                };
                z[k][i] = zz;
                u[k][i] = v - zz;
            }
        }
        if let Some(b) = bn {
            for i in 0..n {
                let v = p[i] + u0[i];
                let zz = v.clamp(cn[i] - b, cn[i] + b);
                z0[i] = zz;
                u0[i] = v - zz;
            }
        }
    }
    let res = if bn.is_some() { &z0 } else { &p };
    res.iter().map(|v| v * s + mean).collect()
}

/// Smooth one scalar channel with the configured method and radius (frames).
pub fn smooth_channel(
    x: &[f64],
    method: SmoothingMethod,
    radius: f64,
    cfg: &SmoothingConfig,
    bound: Option<f64>,
) -> Vec<f64> {
    if radius <= 0.0 {
        return x.to_vec();
    }
    match method {
        SmoothingMethod::Gaussian => gaussian(x, radius / 2.0),
        SmoothingMethod::MovingAverage => moving_average(x, radius.round().max(1.0) as usize),
        SmoothingMethod::Kalman => kalman_rts(x, 4.0 / radius.powi(4)),
        SmoothingMethod::L1 => l1_path(x, cfg.l1_weights, radius, bound, cfg.l1_iterations as usize),
    }
}

/// Correction bound per parameter channel (None = unlimited).
pub fn channel_bounds(cfg: &SmoothingConfig, w: f64, h: f64) -> [Option<f64>; NPARAMS] {
    let pos = |v: f64| if v > 0.0 { Some(v) } else { None };
    let mut b = [None; NPARAMS];
    b[TX] = pos(cfg.limit_shift).map(|f| f * w);
    b[TY] = pos(cfg.limit_shift).map(|f| f * h);
    b[ROT] = pos(cfg.limit_rotation).map(|d| d.to_radians());
    b[LSX] = pos(cfg.limit_scale).map(|f| (1.0 + f).ln());
    b[LSY] = b[LSX];
    b[SHEAR] = pos(cfg.limit_scale);
    b
}

/// Smooth a whole camera trajectory (cumulative transforms as parameters).
pub fn smooth_trajectory(c: &[Params], cfg: &SmoothingConfig, w: f64, h: f64) -> Vec<Params> {
    let n = c.len();
    if n == 0 {
        return Vec::new();
    }
    let bounds = channel_bounds(cfg, w, h);
    let mut out = c.to_vec();
    let refi = (cfg.reference_frame as usize).min(n - 1);
    for ch in 0..NPARAMS {
        let mult = match ch {
            TX | TY => cfg.translation,
            ROT => cfg.rotation,
            _ => cfg.zoom,
        };
        let series: Vec<f64> = c.iter().map(|p| p.0[ch]).collect();
        let locked = cfg.tripod
            || (cfg.horizon_lock && ch == ROT)
            || (cfg.zoom_lock && matches!(ch, LSX | LSY | SHEAR | PX | PY));
        let smoothed = if locked {
            let mut v = series[refi];
            if ch == ROT && cfg.horizon_lock {
                v -= cfg.horizon_angle.to_radians();
            }
            vec![v; n]
        } else {
            smooth_channel(&series, cfg.method, cfg.radius * mult, cfg, bounds[ch])
        };
        for i in 0..n {
            let mut v = smoothed[i];
            if let Some(b) = bounds[ch] {
                v = v.clamp(series[i] - b, series[i] + b);
            }
            out[i].0[ch] = v;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimate::Rng;

    fn noisy_pan(n: usize, seed: u64) -> (Vec<f64>, Vec<f64>) {
        let mut rng = Rng::new(seed);
        let clean: Vec<f64> = (0..n).map(|i| 2.0 * i as f64).collect();
        let noisy = clean.iter().map(|v| v + (rng.unit() - 0.5) * 20.0).collect();
        (clean, noisy)
    }

    fn rms(a: &[f64], b: &[f64]) -> f64 {
        (a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum::<f64>() / a.len() as f64).sqrt()
    }

    #[test]
    fn all_methods_denoise_a_pan() {
        let (clean, noisy) = noisy_pan(300, 3);
        let before = rms(&clean, &noisy);
        let cfg = SmoothingConfig::default();
        for m in
            [SmoothingMethod::Gaussian, SmoothingMethod::MovingAverage, SmoothingMethod::Kalman, SmoothingMethod::L1]
        {
            let s = smooth_channel(&noisy, m, 30.0, &cfg, None);
            let after = rms(&clean, &s);
            // L1 trades exact tracking for piecewise-linear "cinematic" paths.
            let limit = if m == SmoothingMethod::L1 { 0.5 } else { 0.35 };
            assert!(after < before * limit, "{m:?}: {after} vs {before}");
        }
    }

    #[test]
    fn l1_respects_bound_and_goes_static() {
        let mut rng = Rng::new(9);
        let noisy: Vec<f64> = (0..200).map(|_| (rng.unit() - 0.5) * 10.0).collect();
        let s = l1_path(&noisy, [10.0, 1.0, 100.0], 30.0, Some(6.0), 400);
        for (a, b) in noisy.iter().zip(&s) {
            assert!((a - b).abs() <= 6.0 + 1e-6);
        }
        let spread = s.iter().fold(f64::MIN, |m, v| m.max(*v)) - s.iter().fold(f64::MAX, |m, v| m.min(*v));
        assert!(spread < 3.0, "spread {spread}");
    }

    #[test]
    fn tripod_and_limits() {
        let c: Vec<Params> = (0..50)
            .map(|i| {
                let mut p = Params::default();
                p.0[TX] = i as f64 * 3.0;
                p.0[ROT] = 0.01 * (i as f64).sin();
                p
            })
            .collect();
        let cfg = SmoothingConfig { tripod: true, limit_shift: 0.0, limit_rotation: 0.0, ..Default::default() };
        let s = smooth_trajectory(&c, &cfg, 640.0, 360.0);
        assert!(s.iter().all(|p| p.0[TX] == 0.0 && p.0[ROT] == 0.0));
        let cfg = SmoothingConfig { tripod: true, limit_shift: 0.05, ..Default::default() };
        let s = smooth_trajectory(&c, &cfg, 640.0, 360.0);
        assert!((s[49].0[TX] - (147.0 - 32.0)).abs() < 1e-9);
    }
}
