//! Pyramidal Lucas–Kanade sparse optical flow (Bouguet's formulation) with a
//! forward–backward consistency check.

use crate::image::Pyramid;

#[derive(Clone, Copy, Debug)]
pub struct LkParams {
    pub radius: i32,
    pub iterations: usize,
    pub epsilon: f32,
    /// Minimum normalised eigenvalue of the structure tensor (texture check).
    pub min_eig: f32,
}

impl Default for LkParams {
    fn default() -> Self {
        LkParams { radius: 7, iterations: 20, epsilon: 0.01, min_eig: 0.05 }
    }
}

/// Track `pts` from `prev` into `next`. `None` marks lost points.
pub fn track(prev: &Pyramid, next: &Pyramid, pts: &[(f32, f32)], p: &LkParams) -> Vec<Option<(f32, f32)>> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        pts.par_iter().map(|&pt| track_one(prev, next, pt, p)).collect()
    }
    #[cfg(not(feature = "parallel"))]
    {
        pts.iter().map(|&pt| track_one(prev, next, pt, p)).collect()
    }
}

/// Track forward and back; keep points whose round trip lands within `fb_thresh`.
/// Returns `(from, to)` pairs.
pub fn track_checked(
    prev: &Pyramid,
    next: &Pyramid,
    pts: &[(f32, f32)],
    p: &LkParams,
    fb_thresh: f32,
) -> Vec<((f32, f32), (f32, f32))> {
    let fwd = track(prev, next, pts, p);
    let ok_idx: Vec<usize> = (0..pts.len()).filter(|&i| fwd[i].is_some()).collect();
    let fwd_pts: Vec<(f32, f32)> = ok_idx.iter().map(|&i| fwd[i].unwrap()).collect();
    let back = track(next, prev, &fwd_pts, p);
    let t2 = fb_thresh * fb_thresh;
    ok_idx
        .iter()
        .zip(back.iter())
        .zip(fwd_pts.iter())
        .filter_map(|((&i, b), &f)| {
            let b = (*b)?;
            let o = pts[i];
            if (b.0 - o.0).powi(2) + (b.1 - o.1).powi(2) <= t2 {
                Some((o, f))
            } else {
                None
            }
        })
        .collect()
}

fn track_one(prev: &Pyramid, next: &Pyramid, pt: (f32, f32), p: &LkParams) -> Option<(f32, f32)> {
    let nl = prev.levels.len().min(next.levels.len());
    let r = p.radius;
    let win = ((2 * r + 1) * (2 * r + 1)) as usize;
    let mut ival = vec![0.0f32; win];
    let mut ixs = vec![0.0f32; win];
    let mut iys = vec![0.0f32; win];
    let mut g = (0.0f32, 0.0f32);

    for l in (0..nl).rev() {
        let scale = 1.0 / (1u32 << l) as f32;
        let (px, py) = (pt.0 * scale, pt.1 * scale);
        let lp = &prev.levels[l];
        let ln = &next.levels[l];
        let (mut gxx, mut gxy, mut gyy) = (0.0f32, 0.0f32, 0.0f32);
        let mut k = 0;
        for dy in -r..=r {
            for dx in -r..=r {
                let x = px + dx as f32;
                let y = py + dy as f32;
                let ix = lp.gx.sample(x, y);
                let iy = lp.gy.sample(x, y);
                ival[k] = lp.img.sample(x, y);
                ixs[k] = ix;
                iys[k] = iy;
                gxx += ix * ix;
                gxy += ix * iy;
                gyy += iy * iy;
                k += 1;
            }
        }
        let det = gxx * gyy - gxy * gxy;
        let min_eig = 0.5 * (gxx + gyy - ((gxx - gyy).powi(2) + 4.0 * gxy * gxy).sqrt()) / win as f32;

        let mut d = g;
        if min_eig >= p.min_eig && det.abs() > 1e-9 {
            let mut v = (0.0f32, 0.0f32);
            for _ in 0..p.iterations {
                let (mut bx, mut by) = (0.0f32, 0.0f32);
                let mut k = 0;
                let ox = px + g.0 + v.0;
                let oy = py + g.1 + v.1;
                for dy in -r..=r {
                    for dx in -r..=r {
                        let j = ln.img.sample(ox + dx as f32, oy + dy as f32);
                        let it = ival[k] - j;
                        bx += it * ixs[k];
                        by += it * iys[k];
                        k += 1;
                    }
                }
                let ex = (gyy * bx - gxy * by) / det;
                let ey = (gxx * by - gxy * bx) / det;
                if !ex.is_finite() || !ey.is_finite() {
                    return None;
                }
                v.0 += ex;
                v.1 += ey;
                if ex * ex + ey * ey < p.epsilon * p.epsilon {
                    break;
                }
            }
            d = (g.0 + v.0, g.1 + v.1);
        } else if l == 0 {
            return None;
        }
        g = if l > 0 { (2.0 * d.0, 2.0 * d.1) } else { d };
    }

    let res = (pt.0 + g.0, pt.1 + g.1);
    let base = &next.levels[0].img;
    if res.0 < 0.0 || res.1 < 0.0 || res.0 > (base.w - 1) as f32 || res.1 > (base.h - 1) as f32 {
        return None;
    }
    Some(res)
}
