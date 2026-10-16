//! Robust motion-model fitting (translation / similarity / affine / homography)
//! with RANSAC and least-squares refinement.

use crate::config::MotionModel;
use crate::geometry::Mat3;

/// Small deterministic PRNG (xorshift64*), so results are reproducible.
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed.max(1))
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
    /// Uniform in [0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Estimate {
    pub m: Mat3,
    pub inliers: usize,
    pub total: usize,
}

pub fn min_samples(model: MotionModel) -> usize {
    match model {
        MotionModel::Translation => 1,
        MotionModel::Similarity => 2,
        MotionModel::Affine => 3,
        MotionModel::Homography => 4,
    }
}

#[inline]
fn err2(m: &Mat3, s: (f64, f64), d: (f64, f64)) -> f64 {
    let (x, y) = m.apply(s.0, s.1);
    (x - d.0).powi(2) + (y - d.1).powi(2)
}

/// Fit `model` mapping `src → dst` robustly.
pub fn estimate(
    src: &[(f64, f64)],
    dst: &[(f64, f64)],
    model: MotionModel,
    thresh: f64,
    max_iters: usize,
    rng: &mut Rng,
) -> Option<Estimate> {
    let n = src.len().min(dst.len());
    let k = min_samples(model);
    if n < k.max(2) {
        return None;
    }
    let thr2 = thresh * thresh;
    let mut best: Vec<usize> = Vec::new();
    let mut needed = max_iters.max(1);
    let mut sample = vec![0usize; k];
    let mut it = 0;
    while it < needed.min(max_iters.max(1)) {
        it += 1;
        for i in 0..k {
            loop {
                let c = rng.below(n);
                if !sample[..i].contains(&c) {
                    sample[i] = c;
                    break;
                }
            }
        }
        let Some(m) = fit(model, src, dst, &sample) else { continue };
        let mut count = 0;
        for i in 0..n {
            if err2(&m, src[i], dst[i]) < thr2 {
                count += 1;
            }
        }
        if count > best.len() {
            best = (0..n).filter(|&i| err2(&m, src[i], dst[i]) < thr2).collect();
            let w = best.len() as f64 / n as f64;
            let p_bad = 1.0 - w.powi(k as i32);
            needed = if p_bad <= 1e-9 { it } else { (((1.0f64 - 0.995).ln() / p_bad.ln()).ceil() as usize).max(it) };
        }
    }
    if best.len() < k.max(2) {
        return None;
    }
    let mut m = fit(model, src, dst, &best)?;
    for _ in 0..2 {
        let inl: Vec<usize> = (0..n).filter(|&i| err2(&m, src[i], dst[i]) < thr2).collect();
        if inl.len() < k.max(2) {
            break;
        }
        match fit(model, src, dst, &inl) {
            Some(mm) => {
                m = mm;
                best = inl;
            }
            None => break,
        }
    }
    if model == MotionModel::Homography && !plausible_homography(&m) {
        m = fit(MotionModel::Affine, src, dst, &best)?;
    }
    if !m.is_finite() {
        return None;
    }
    Some(Estimate { m, inliers: best.len(), total: n })
}

/// Reject wild homographies (in centred analysis-resolution coordinates).
fn plausible_homography(m: &Mat3) -> bool {
    let m = m.normalized();
    let det = m.0[0] * m.0[4] - m.0[1] * m.0[3];
    m.is_finite() && det > 0.25 && det < 4.0 && m.0[6].abs() < 2e-3 && m.0[7].abs() < 2e-3
}

/// Least-squares fit of `model` on the subset `idx`.
pub fn fit(model: MotionModel, src: &[(f64, f64)], dst: &[(f64, f64)], idx: &[usize]) -> Option<Mat3> {
    match model {
        MotionModel::Translation => {
            let n = idx.len() as f64;
            if n < 1.0 {
                return None;
            }
            let (mut tx, mut ty) = (0.0, 0.0);
            for &i in idx {
                tx += dst[i].0 - src[i].0;
                ty += dst[i].1 - src[i].1;
            }
            Some(Mat3::translation(tx / n, ty / n))
        }
        MotionModel::Similarity => fit_similarity(src, dst, idx),
        MotionModel::Affine => fit_affine(src, dst, idx),
        MotionModel::Homography => fit_homography(src, dst, idx),
    }
}

fn fit_similarity(src: &[(f64, f64)], dst: &[(f64, f64)], idx: &[usize]) -> Option<Mat3> {
    let n = idx.len() as f64;
    if idx.len() < 2 {
        return None;
    }
    let (mut msx, mut msy, mut mdx, mut mdy) = (0.0, 0.0, 0.0, 0.0);
    for &i in idx {
        msx += src[i].0;
        msy += src[i].1;
        mdx += dst[i].0;
        mdy += dst[i].1;
    }
    msx /= n;
    msy /= n;
    mdx /= n;
    mdy /= n;
    let (mut na, mut nb, mut den) = (0.0, 0.0, 0.0);
    for &i in idx {
        let (sx, sy) = (src[i].0 - msx, src[i].1 - msy);
        let (dx, dy) = (dst[i].0 - mdx, dst[i].1 - mdy);
        na += sx * dx + sy * dy;
        nb += sx * dy - sy * dx;
        den += sx * sx + sy * sy;
    }
    if den < 1e-9 {
        return None;
    }
    let a = na / den;
    let b = nb / den;
    let tx = mdx - (a * msx - b * msy);
    let ty = mdy - (b * msx + a * msy);
    Some(Mat3([a, -b, tx, b, a, ty, 0.0, 0.0, 1.0]))
}

fn fit_affine(src: &[(f64, f64)], dst: &[(f64, f64)], idx: &[usize]) -> Option<Mat3> {
    if idx.len() < 3 {
        return None;
    }
    let mut ata = [0.0f64; 9];
    let mut bu = [0.0f64; 3];
    let mut bv = [0.0f64; 3];
    for &i in idx {
        let r = [src[i].0, src[i].1, 1.0];
        for a in 0..3 {
            for b in 0..3 {
                ata[a * 3 + b] += r[a] * r[b];
            }
            bu[a] += r[a] * dst[i].0;
            bv[a] += r[a] * dst[i].1;
        }
    }
    let u = solve(3, &mut ata.clone(), &mut bu)?;
    let v = solve(3, &mut ata, &mut bv)?;
    Some(Mat3([u[0], u[1], u[2], v[0], v[1], v[2], 0.0, 0.0, 1.0]))
}

/// Hartley-normalised DLT with h33 = 1, solved via normal equations.
fn fit_homography(src: &[(f64, f64)], dst: &[(f64, f64)], idx: &[usize]) -> Option<Mat3> {
    if idx.len() < 4 {
        return None;
    }
    let norm = |pts: &dyn Fn(usize) -> (f64, f64)| -> Option<Mat3> {
        let n = idx.len() as f64;
        let (mut mx, mut my) = (0.0, 0.0);
        for &i in idx {
            let p = pts(i);
            mx += p.0;
            my += p.1;
        }
        mx /= n;
        my /= n;
        let mut d = 0.0;
        for &i in idx {
            let p = pts(i);
            d += ((p.0 - mx).powi(2) + (p.1 - my).powi(2)).sqrt();
        }
        d /= n;
        if d < 1e-9 {
            return None;
        }
        let s = std::f64::consts::SQRT_2 / d;
        Some(Mat3([s, 0.0, -s * mx, 0.0, s, -s * my, 0.0, 0.0, 1.0]))
    };
    let ts = norm(&|i| src[i])?;
    let td = norm(&|i| dst[i])?;
    let mut ata = [0.0f64; 64];
    let mut atb = [0.0f64; 8];
    for &i in idx {
        let (x, y) = ts.apply(src[i].0, src[i].1);
        let (u, v) = td.apply(dst[i].0, dst[i].1);
        let r1 = [x, y, 1.0, 0.0, 0.0, 0.0, -x * u, -y * u];
        let r2 = [0.0, 0.0, 0.0, x, y, 1.0, -x * v, -y * v];
        for a in 0..8 {
            for b in 0..8 {
                ata[a * 8 + b] += r1[a] * r1[b] + r2[a] * r2[b];
            }
            atb[a] += r1[a] * u + r2[a] * v;
        }
    }
    let h = solve(8, &mut ata, &mut atb)?;
    let hn = Mat3([h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0]);
    let m = td.inverse()?.mul(&hn).mul(&ts).normalized();
    if m.is_finite() {
        Some(m)
    } else {
        None
    }
}

/// Gaussian elimination with partial pivoting for a dense `n×n` system.
pub fn solve(n: usize, a: &mut [f64], b: &mut [f64]) -> Option<Vec<f64>> {
    for col in 0..n {
        let mut piv = col;
        let mut best = a[col * n + col].abs();
        for row in col + 1..n {
            let v = a[row * n + col].abs();
            if v > best {
                best = v;
                piv = row;
            }
        }
        if best < 1e-12 {
            return None;
        }
        if piv != col {
            for k in 0..n {
                a.swap(col * n + k, piv * n + k);
            }
            b.swap(col, piv);
        }
        let d = a[col * n + col];
        for row in col + 1..n {
            let f = a[row * n + col] / d;
            if f == 0.0 {
                continue;
            }
            for k in col..n {
                a[row * n + k] -= f * a[col * n + k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut s = b[row];
        for k in row + 1..n {
            s -= a[row * n + k] * x[k];
        }
        x[row] = s / a[row * n + row];
    }
    if x.iter().all(|v| v.is_finite()) {
        Some(x)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make(m: &Mat3, rng: &mut Rng, n: usize, outliers: usize) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
        let mut s = Vec::new();
        let mut d = Vec::new();
        for i in 0..n {
            let p = (rng.unit() * 600.0 - 300.0, rng.unit() * 400.0 - 200.0);
            let mut q = m.apply(p.0, p.1);
            if i < outliers {
                q.0 += rng.unit() * 80.0 - 40.0;
                q.1 += rng.unit() * 80.0 - 40.0;
            } else {
                q.0 += (rng.unit() - 0.5) * 0.4;
                q.1 += (rng.unit() - 0.5) * 0.4;
            }
            s.push(p);
            d.push(q);
        }
        (s, d)
    }

    #[test]
    fn recovers_models_with_outliers() {
        let truth = [
            (MotionModel::Translation, Mat3::translation(5.0, -3.0)),
            (
                MotionModel::Similarity,
                Mat3::rotation(0.02).mul(&Mat3::scale(1.01, 1.01)).mul(&Mat3::translation(3.0, 2.0)),
            ),
            (MotionModel::Affine, Mat3([1.02, 0.01, 4.0, -0.015, 0.99, -2.0, 0.0, 0.0, 1.0])),
            (MotionModel::Homography, Mat3([1.01, 0.01, 4.0, -0.01, 0.99, -2.0, 3e-5, -2e-5, 1.0])),
        ];
        for (model, m) in truth {
            let mut rng = Rng::new(7);
            let (s, d) = make(&m, &mut rng, 300, 90);
            let e = estimate(&s, &d, model, 1.5, 500, &mut rng).unwrap();
            assert!(e.inliers >= 200, "{model:?} inliers {}", e.inliers);
            for p in [(-250.0, -150.0), (250.0, 180.0), (0.0, 0.0)] {
                let a = m.apply(p.0, p.1);
                let b = e.m.apply(p.0, p.1);
                assert!((a.0 - b.0).abs() < 0.5 && (a.1 - b.1).abs() < 0.5, "{model:?}: {a:?} vs {b:?}");
            }
        }
    }
}
