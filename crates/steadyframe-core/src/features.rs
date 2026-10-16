//! Shi–Tomasi ("good features to track") corner detection with spatial balancing.

use crate::image::Level;

/// Detect up to `max` corners on a pyramid level.
///
/// * `quality` — fraction of the strongest response a corner must reach.
/// * `min_dist` — minimum spacing between accepted corners.
/// * `border` — ignore this many pixels at each edge.
///
/// Corners are spread over a 4×4 grid of regions first, so tracking isn't
/// dominated by one highly textured area (e.g. foliage on one side).
pub fn detect(level: &Level, max: usize, quality: f32, min_dist: f32, border: usize) -> Vec<(f32, f32)> {
    let (w, h) = (level.img.w, level.img.h);
    let border = border.max(2);
    if max == 0 || w < 2 * border + 3 || h < 2 * border + 3 {
        return Vec::new();
    }
    let n = w * h;
    let mut xx = vec![0.0f32; n];
    let mut xy = vec![0.0f32; n];
    let mut yy = vec![0.0f32; n];
    for i in 0..n {
        let gx = level.gx.data[i];
        let gy = level.gy.data[i];
        xx[i] = gx * gx;
        xy[i] = gx * gy;
        yy[i] = gy * gy;
    }
    let xx = box_blur(&xx, w, h, 2);
    let xy = box_blur(&xy, w, h, 2);
    let yy = box_blur(&yy, w, h, 2);

    let mut resp = vec![0.0f32; n];
    let mut maxr = 0.0f32;
    for y in border..h - border {
        for x in border..w - border {
            let i = y * w + x;
            let (a, b, c) = (xx[i], xy[i], yy[i]);
            let r = 0.5 * (a + c) - (0.25 * (a - c) * (a - c) + b * b).sqrt();
            resp[i] = r;
            if r > maxr {
                maxr = r;
            }
        }
    }
    if maxr <= 1e-6 {
        return Vec::new();
    }
    let thr = maxr * quality.max(1e-4);

    let mut cands: Vec<(f32, usize, usize)> = Vec::new();
    for y in border..h - border {
        for x in border..w - border {
            let i = y * w + x;
            let r = resp[i];
            if r <= thr {
                continue;
            }
            let mut is_max = true;
            'nb: for dy in -1isize..=1 {
                for dx in -1isize..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let j = (y as isize + dy) as usize * w + (x as isize + dx) as usize;
                    if resp[j] > r {
                        is_max = false;
                        break 'nb;
                    }
                }
            }
            if is_max {
                cands.push((r, x, y));
            }
        }
    }
    cands.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    // Spatial hash for the min-distance test.
    let cell = min_dist.max(1.0);
    let gw = (w as f32 / cell).ceil() as usize + 1;
    let gh = (h as f32 / cell).ceil() as usize + 1;
    let mut grid: Vec<Vec<(f32, f32)>> = vec![Vec::new(); gw * gh];
    let md2 = min_dist * min_dist;
    const REG: usize = 4;
    let region_cap = (max / (REG * REG)).max(1);
    let mut region_count = [0usize; REG * REG];
    let mut taken = vec![false; cands.len()];
    let mut out = Vec::with_capacity(max);

    for pass in 0..2 {
        for (k, &(_, x, y)) in cands.iter().enumerate() {
            if out.len() >= max {
                break;
            }
            if taken[k] {
                continue;
            }
            let reg = (y * REG / h).min(REG - 1) * REG + (x * REG / w).min(REG - 1);
            if pass == 0 && region_count[reg] >= region_cap {
                continue;
            }
            let (fx, fy) = (x as f32, y as f32);
            let cx = (fx / cell) as usize;
            let cy = (fy / cell) as usize;
            let mut ok = true;
            'chk: for gy in cy.saturating_sub(1)..=(cy + 1).min(gh - 1) {
                for gx in cx.saturating_sub(1)..=(cx + 1).min(gw - 1) {
                    for &(px, py) in &grid[gy * gw + gx] {
                        if (px - fx).powi(2) + (py - fy).powi(2) < md2 {
                            ok = false;
                            break 'chk;
                        }
                    }
                }
            }
            if !ok {
                continue;
            }
            taken[k] = true;
            region_count[reg] += 1;
            grid[cy * gw + cx].push((fx, fy));
            out.push((fx, fy));
        }
    }
    out
}

/// Separable box blur with radius `r` and edge clamping.
fn box_blur(src: &[f32], w: usize, h: usize, r: usize) -> Vec<f32> {
    let mut tmp = vec![0.0f32; w * h];
    let norm = 1.0 / (2 * r + 1) as f32;
    for y in 0..h {
        for x in 0..w {
            let mut s = 0.0;
            for k in 0..=2 * r {
                let xx = (x as isize + k as isize - r as isize).clamp(0, w as isize - 1) as usize;
                s += src[y * w + xx];
            }
            tmp[y * w + x] = s * norm;
        }
    }
    let mut out = vec![0.0f32; w * h];
    for y in 0..h {
        for x in 0..w {
            let mut s = 0.0;
            for k in 0..=2 * r {
                let yy = (y as isize + k as isize - r as isize).clamp(0, h as isize - 1) as usize;
                s += tmp[yy * w + x];
            }
            out[y * w + x] = s * norm;
        }
    }
    out
}
