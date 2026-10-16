//! Grayscale float images, resampling and image pyramids with gradients.

/// Single-channel f32 image (0..255 range).
#[derive(Clone, Debug)]
pub struct Gray {
    pub w: usize,
    pub h: usize,
    pub data: Vec<f32>,
}

impl Gray {
    pub fn new(w: usize, h: usize) -> Self {
        Gray { w, h, data: vec![0.0; w * h] }
    }

    #[inline]
    pub fn at(&self, x: usize, y: usize) -> f32 {
        self.data[y * self.w + x]
    }

    #[inline]
    pub fn at_clamped(&self, x: isize, y: isize) -> f32 {
        let x = x.clamp(0, self.w as isize - 1) as usize;
        let y = y.clamp(0, self.h as isize - 1) as usize;
        self.data[y * self.w + x]
    }

    /// Bilinear sample with edge clamping.
    #[inline]
    pub fn sample(&self, x: f32, y: f32) -> f32 {
        let x0 = x.floor();
        let y0 = y.floor();
        let fx = x - x0;
        let fy = y - y0;
        let xi = x0 as isize;
        let yi = y0 as isize;
        if xi >= 0 && yi >= 0 && xi + 1 < self.w as isize && yi + 1 < self.h as isize {
            let i = yi as usize * self.w + xi as usize;
            let a = self.data[i];
            let b = self.data[i + 1];
            let c = self.data[i + self.w];
            let d = self.data[i + self.w + 1];
            let top = a + (b - a) * fx;
            let bot = c + (d - c) * fx;
            top + (bot - top) * fy
        } else {
            let a = self.at_clamped(xi, yi);
            let b = self.at_clamped(xi + 1, yi);
            let c = self.at_clamped(xi, yi + 1);
            let d = self.at_clamped(xi + 1, yi + 1);
            let top = a + (b - a) * fx;
            let bot = c + (d - c) * fx;
            top + (bot - top) * fy
        }
    }

    /// Blur with a 5-tap binomial kernel and decimate by 2.
    pub fn downsample(&self) -> Gray {
        const K: [f32; 5] = [1.0 / 16.0, 4.0 / 16.0, 6.0 / 16.0, 4.0 / 16.0, 1.0 / 16.0];
        let nw = self.w.div_ceil(2);
        let nh = self.h.div_ceil(2);
        let mut tmp = vec![0.0f32; nw * self.h];
        for y in 0..self.h {
            for nx in 0..nw {
                let cx = (nx * 2) as isize;
                let mut s = 0.0;
                for (k, wk) in K.iter().enumerate() {
                    s += wk * self.at_clamped(cx + k as isize - 2, y as isize);
                }
                tmp[y * nw + nx] = s;
            }
        }
        let mut out = Gray::new(nw, nh);
        for ny in 0..nh {
            let cy = (ny * 2) as isize;
            for nx in 0..nw {
                let mut s = 0.0;
                for (k, wk) in K.iter().enumerate() {
                    let yy = (cy + k as isize - 2).clamp(0, self.h as isize - 1) as usize;
                    s += wk * tmp[yy * nw + nx];
                }
                out.data[ny * nw + nx] = s;
            }
        }
        out
    }
}

/// Rec.601-ish luma from interleaved RGBA.
pub fn luma_from_rgba(rgba: &[u8], w: usize, h: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(w * h);
    for px in rgba.chunks_exact(4).take(w * h) {
        let y = (77 * px[0] as u32 + 150 * px[1] as u32 + 29 * px[2] as u32) >> 8;
        out.push(y as u8);
    }
    out
}

/// Area-average resample of an 8-bit luma image to `dw × dh` floats.
pub fn resample_to_gray(src: &[u8], sw: usize, sh: usize, dw: usize, dh: usize) -> Gray {
    let ranges = |s: usize, d: usize| -> Vec<(usize, usize)> {
        let f = s as f64 / d as f64;
        (0..d)
            .map(|i| {
                let a = ((i as f64 * f).floor() as usize).min(s - 1);
                let b = (((i + 1) as f64 * f).floor() as usize).clamp(a + 1, s);
                (a, b)
            })
            .collect()
    };
    let xr = ranges(sw, dw);
    let yr = ranges(sh, dh);
    // Horizontal pass into row sums, then vertical.
    let mut tmp = vec![0.0f32; dw * sh];
    for y in 0..sh {
        let row = &src[y * sw..(y + 1) * sw];
        for (i, &(a, b)) in xr.iter().enumerate() {
            let mut s = 0u32;
            for &v in &row[a..b] {
                s += v as u32;
            }
            tmp[y * dw + i] = s as f32 / (b - a) as f32;
        }
    }
    let mut out = Gray::new(dw, dh);
    for (j, &(a, b)) in yr.iter().enumerate() {
        for i in 0..dw {
            let mut s = 0.0;
            for y in a..b {
                s += tmp[y * dw + i];
            }
            out.data[j * dw + i] = s / (b - a) as f32;
        }
    }
    out
}

/// One pyramid level: image plus Scharr gradients.
pub struct Level {
    pub img: Gray,
    pub gx: Gray,
    pub gy: Gray,
}

pub struct Pyramid {
    pub levels: Vec<Level>,
}

impl Pyramid {
    pub fn build(base: Gray, max_levels: usize) -> Pyramid {
        let mut imgs = vec![base];
        while imgs.len() < max_levels.max(1) {
            let last = imgs.last().unwrap();
            if last.w < 40 || last.h < 40 {
                break;
            }
            let next = last.downsample();
            imgs.push(next);
        }
        let levels = imgs
            .into_iter()
            .map(|img| {
                let (gx, gy) = scharr(&img);
                Level { img, gx, gy }
            })
            .collect();
        Pyramid { levels }
    }
}

/// Scharr derivative (normalised to intensity units per pixel).
pub fn scharr(img: &Gray) -> (Gray, Gray) {
    let (w, h) = (img.w, img.h);
    let mut gx = Gray::new(w, h);
    let mut gy = Gray::new(w, h);
    for y in 0..h as isize {
        for x in 0..w as isize {
            let p = |dx: isize, dy: isize| img.at_clamped(x + dx, y + dy);
            let dx = 3.0 * (p(1, -1) - p(-1, -1)) + 10.0 * (p(1, 0) - p(-1, 0)) + 3.0 * (p(1, 1) - p(-1, 1));
            let dy = 3.0 * (p(-1, 1) - p(-1, -1)) + 10.0 * (p(0, 1) - p(0, -1)) + 3.0 * (p(1, 1) - p(1, -1));
            let i = y as usize * w + x as usize;
            gx.data[i] = dx / 32.0;
            gy.data[i] = dy / 32.0;
        }
    }
    (gx, gy)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resample_preserves_mean() {
        let w = 37;
        let h = 23;
        let src: Vec<u8> = (0..w * h).map(|i| (i * 7 % 256) as u8).collect();
        let g = resample_to_gray(&src, w, h, 10, 6);
        assert_eq!((g.w, g.h), (10, 6));
        let mean_src = src.iter().map(|&v| v as f64).sum::<f64>() / src.len() as f64;
        let mean_dst = g.data.iter().map(|&v| v as f64).sum::<f64>() / g.data.len() as f64;
        assert!((mean_src - mean_dst).abs() < 15.0);
    }

    #[test]
    fn gradient_of_ramp() {
        let mut g = Gray::new(10, 10);
        for y in 0..10 {
            for x in 0..10 {
                g.data[y * 10 + x] = 2.0 * x as f32;
            }
        }
        let (gx, gy) = scharr(&g);
        assert!((gx.at(5, 5) - 2.0).abs() < 1e-5);
        assert!(gy.at(5, 5).abs() < 1e-5);
    }
}
