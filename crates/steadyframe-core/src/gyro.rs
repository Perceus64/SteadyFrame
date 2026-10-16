//! Gyroscope support: CSV parsing, integration, rotation homographies and
//! automatic synchronisation against optical flow.
//!
//! Camera axes follow the computer-vision convention: x right, y down,
//! z forward. Angular velocity is in rad/s in that frame (after applying
//! [`Orientation`]).

use crate::config::{AngleUnit, GyroConfig, TimeUnit};
use crate::error::{Error, Result};
use crate::geometry::{Mat3, Params, ROT, TX, TY};
use crate::pipeline::FrameMotion;
use serde::{Deserialize, Serialize};

/// Time-stamped angular velocity samples (seconds from first sample, rad/s).
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct GyroData {
    pub t: Vec<f64>,
    pub w: Vec<[f64; 3]>,
}

impl GyroData {
    /// Build from unsorted samples; sorts, drops duplicates, rebases time to 0.
    pub fn new(mut samples: Vec<(f64, [f64; 3])>) -> Result<GyroData> {
        samples.retain(|(t, w)| t.is_finite() && w.iter().all(|v| v.is_finite()));
        samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        samples.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);
        if samples.len() < 2 {
            return Err(Error::Gyro("need at least two gyro samples".into()));
        }
        let t0 = samples[0].0;
        Ok(GyroData { t: samples.iter().map(|s| s.0 - t0).collect(), w: samples.iter().map(|s| s.1).collect() })
    }

    pub fn duration(&self) -> f64 {
        self.t.last().copied().unwrap_or(0.0)
    }

    pub fn len(&self) -> usize {
        self.t.len()
    }

    pub fn is_empty(&self) -> bool {
        self.t.is_empty()
    }

    /// Parse CSV/TSV gyro logs.
    ///
    /// Accepts a header row (`t,gx,gy,gz`, `time,gyro_x,…`, Gyroflow's
    /// `GYROFLOW IMU LOG` with `tscale`/`gscale` lines) or headerless
    /// `time, x, y, z` rows. Units are auto-detected unless configured.
    pub fn from_csv(text: &str, time_unit: TimeUnit, angle_unit: AngleUnit) -> Result<GyroData> {
        let mut tscale: Option<f64> = None;
        let mut gscale: Option<f64> = None;
        let mut cols: Option<[usize; 4]> = None;
        let mut rows: Vec<(f64, [f64; 3])> = Vec::new();
        for raw in text.lines() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
                continue;
            }
            let fields: Vec<&str> = line
                .split([',', ';', '\t', ' '])
                .map(|s| s.trim().trim_matches('"'))
                .filter(|s| !s.is_empty())
                .collect();
            if fields.is_empty() {
                continue;
            }
            let lower0 = fields[0].to_ascii_lowercase();
            if lower0 == "tscale" && fields.len() > 1 {
                tscale = fields[1].parse().ok();
                continue;
            }
            if lower0 == "gscale" && fields.len() > 1 {
                gscale = fields[1].parse().ok();
                continue;
            }
            let nums: Vec<Option<f64>> = fields.iter().map(|f| f.parse::<f64>().ok()).collect();
            if nums.iter().all(|n| n.is_none()) || (cols.is_none() && nums[0].is_none()) {
                // Header (or metadata) line.
                if cols.is_none() {
                    cols = header_columns(&fields);
                }
                continue;
            }
            let c = cols.unwrap_or([0, 1, 2, 3]);
            let get = |i: usize| nums.get(i).copied().flatten();
            if let (Some(t), Some(x), Some(y), Some(z)) = (get(c[0]), get(c[1]), get(c[2]), get(c[3])) {
                rows.push((t, [x, y, z]));
            }
        }
        if rows.len() < 2 {
            return Err(Error::Parse("gyro CSV: found fewer than two data rows with time + 3 axes".into()));
        }
        // Time units
        let tmul = if let Some(s) = tscale {
            s
        } else {
            match time_unit {
                TimeUnit::Seconds => 1.0,
                TimeUnit::Milliseconds => 1e-3,
                TimeUnit::Microseconds => 1e-6,
                TimeUnit::Nanoseconds => 1e-9,
                TimeUnit::Auto => {
                    let mut dts: Vec<f64> =
                        rows.windows(2).map(|w| (w[1].0 - w[0].0).abs()).filter(|d| *d > 0.0).collect();
                    dts.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    let med = dts.get(dts.len() / 2).copied().unwrap_or(0.005);
                    if med < 0.1 {
                        1.0
                    } else if med < 100.0 {
                        1e-3
                    } else if med < 1e5 {
                        1e-6
                    } else {
                        1e-9
                    }
                }
            }
        };
        let gmul = if let Some(s) = gscale {
            s
        } else {
            match angle_unit {
                AngleUnit::Radians => 1.0,
                AngleUnit::Degrees => std::f64::consts::PI / 180.0,
                AngleUnit::Auto => {
                    let peak = rows.iter().flat_map(|r| r.1.iter()).fold(0.0f64, |m, v| m.max(v.abs()));
                    if peak > 40.0 {
                        std::f64::consts::PI / 180.0
                    } else {
                        1.0
                    }
                }
            }
        };
        GyroData::new(rows.into_iter().map(|(t, w)| (t * tmul, [w[0] * gmul, w[1] * gmul, w[2] * gmul])).collect())
    }

    /// Serialize to a simple `t,gx,gy,gz` CSV (seconds, rad/s).
    pub fn to_csv(&self) -> String {
        let mut s = String::from("t,gx,gy,gz\n");
        for (t, w) in self.t.iter().zip(&self.w) {
            s.push_str(&format!("{t:.6},{:.6},{:.6},{:.6}\n", w[0], w[1], w[2]));
        }
        s
    }
}

fn header_columns(fields: &[&str]) -> Option<[usize; 4]> {
    let low: Vec<String> = fields.iter().map(|f| f.to_ascii_lowercase()).collect();
    let find = |pred: &dyn Fn(&str) -> bool| low.iter().position(|f| pred(f));
    let t = find(&|f| f == "t" || f.starts_with("time") || f.starts_with("timestamp") || f == "ts")?;
    let axis = |a: char| {
        find(&|f: &str| {
            let f = f.replace(['_', '-', ' ', '(', ')', '[', ']'], "");
            f == format!("g{a}")
                || f == format!("gyro{a}")
                || f == format!("gyr{a}")
                || f == format!("w{a}")
                || f == format!("rot{a}")
                || f == format!("rotationrate{a}")
                || f == format!("angularvelocity{a}")
                || f.starts_with(&format!("gyro{a}"))
                || f == a.to_string()
        })
    };
    Some([t, axis('x')?, axis('y')?, axis('z')?])
}

/// Signed axis permutation from gyro axes to camera axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Orientation {
    pub src: [usize; 3],
    pub sign: [f64; 3],
}

impl Orientation {
    pub const IDENTITY: Orientation = Orientation { src: [0, 1, 2], sign: [1.0, 1.0, 1.0] };

    /// Parse strings like `XYZ`, `yXz`, `ZxY`.
    pub fn parse(s: &str) -> Result<Orientation> {
        let chars: Vec<char> = s.trim().chars().collect();
        if chars.len() != 3 {
            return Err(Error::Config(format!("gyro orientation '{s}' must have 3 letters (e.g. XYZ, yXz)")));
        }
        let mut src = [0usize; 3];
        let mut sign = [1.0; 3];
        let mut seen = [false; 3];
        for (i, c) in chars.iter().enumerate() {
            let idx = match c.to_ascii_uppercase() {
                'X' => 0,
                'Y' => 1,
                'Z' => 2,
                _ => return Err(Error::Config(format!("gyro orientation '{s}': only X/Y/Z allowed"))),
            };
            if seen[idx] {
                return Err(Error::Config(format!("gyro orientation '{s}' repeats an axis")));
            }
            seen[idx] = true;
            src[i] = idx;
            sign[i] = if c.is_ascii_lowercase() { -1.0 } else { 1.0 };
        }
        Ok(Orientation { src, sign })
    }

    #[inline]
    pub fn apply(&self, v: [f64; 3]) -> [f64; 3] {
        [self.sign[0] * v[self.src[0]], self.sign[1] * v[self.src[1]], self.sign[2] * v[self.src[2]]]
    }

    /// All 48 signed permutations.
    pub fn all() -> Vec<Orientation> {
        const PERMS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
        let mut v = Vec::with_capacity(48);
        for p in PERMS {
            for m in 0..8 {
                let sign = [
                    if m & 1 == 0 { 1.0 } else { -1.0 },
                    if m & 2 == 0 { 1.0 } else { -1.0 },
                    if m & 4 == 0 { 1.0 } else { -1.0 },
                ];
                v.push(Orientation { src: p, sign });
            }
        }
        v
    }
}

impl std::fmt::Display for Orientation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for i in 0..3 {
            let c = ['X', 'Y', 'Z'][self.src[i]];
            let c = if self.sign[i] < 0.0 { c.to_ascii_lowercase() } else { c };
            write!(f, "{c}")?;
        }
        Ok(())
    }
}

/// Integrated rotation vector (small-angle accumulation of ω·dt).
pub struct Integrator<'a> {
    g: &'a GyroData,
    cum: Vec<[f64; 3]>,
}

impl<'a> Integrator<'a> {
    pub fn new(g: &'a GyroData) -> Self {
        let mut cum = Vec::with_capacity(g.len());
        let mut acc = [0.0; 3];
        cum.push(acc);
        for i in 1..g.len() {
            let dt = g.t[i] - g.t[i - 1];
            for k in 0..3 {
                acc[k] += 0.5 * (g.w[i][k] + g.w[i - 1][k]) * dt;
            }
            cum.push(acc);
        }
        Integrator { g, cum }
    }

    /// Integral of ω from the first sample to time `t` (gyro clock).
    pub fn at(&self, t: f64) -> [f64; 3] {
        let ts = &self.g.t;
        if t <= ts[0] {
            return self.cum[0];
        }
        let n = ts.len();
        if t >= ts[n - 1] {
            return self.cum[n - 1];
        }
        let i = match ts.binary_search_by(|v| v.partial_cmp(&t).unwrap()) {
            Ok(i) => return self.cum[i],
            Err(i) => i - 1,
        };
        // Exact trapezoid within the interval using linear ω.
        let dt = t - ts[i];
        let span = ts[i + 1] - ts[i];
        let a = self.g.w[i];
        let b = self.g.w[i + 1];
        let mut out = self.cum[i];
        for k in 0..3 {
            let wt = a[k] + (b[k] - a[k]) * dt / span;
            out[k] += 0.5 * (a[k] + wt) * dt;
        }
        out
    }

    pub fn between(&self, t0: f64, t1: f64) -> [f64; 3] {
        let a = self.at(t0);
        let b = self.at(t1);
        [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
    }
}

/// Focal length in pixels from horizontal FOV.
pub fn focal_from_hfov(width: f64, hfov_deg: f64) -> f64 {
    (width / 2.0) / (hfov_deg.to_radians() / 2.0).tan()
}

/// Rodrigues rotation matrix (3×3, row-major) for rotation vector `r`.
pub fn rodrigues(r: [f64; 3]) -> [f64; 9] {
    let th = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
    if th < 1e-12 {
        return [1.0, -r[2], r[1], r[2], 1.0, -r[0], -r[1], r[0], 1.0];
    }
    let (x, y, z) = (r[0] / th, r[1] / th, r[2] / th);
    let (s, c) = th.sin_cos();
    let v = 1.0 - c;
    [
        c + x * x * v,
        x * y * v - z * s,
        x * z * v + y * s,
        y * x * v + z * s,
        c + y * y * v,
        y * z * v - x * s,
        z * x * v - y * s,
        z * y * v + x * s,
        c + z * z * v,
    ]
}

/// Image-space homography for a camera rotation `phi` (rad, camera axes),
/// focal `f`, principal point at the centre: points move by K·R(−φ)·K⁻¹.
pub fn rotation_homography(phi: [f64; 3], f: f64) -> Mat3 {
    let r = rodrigues([-phi[0], -phi[1], -phi[2]]);
    Mat3([r[0], r[1], f * r[2], r[3], r[4], f * r[5], r[6] / f, r[7] / f, r[8]]).normalized()
}

/// Per-frame motions (frame t−1 → t, full-res centred coords) from gyro.
/// Video time = gyro time + `offset`.
pub fn gyro_motions(g: &GyroData, o: &Orientation, offset: f64, f: f64, fps: f64, n: usize) -> Vec<Mat3> {
    let integ = Integrator::new(g);
    let mut out = Vec::with_capacity(n);
    for t in 0..n {
        if t == 0 {
            out.push(Mat3::IDENTITY);
            continue;
        }
        let t0 = (t - 1) as f64 / fps - offset;
        let t1 = t as f64 / fps - offset;
        let phi = o.apply(integ.between(t0, t1));
        out.push(rotation_homography(phi, f));
    }
    out
}

/// Result of automatic gyro ↔ video synchronisation.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SyncResult {
    /// Seconds added to gyro time to match video time.
    pub offset: f64,
    pub orientation: String,
    /// Focal length (full-resolution pixels).
    pub focal: f64,
    /// Unexplained fraction of the optical-flow motion (0 = perfect, ~1 = no match).
    pub score: f64,
    /// Frames used for matching.
    pub frames_used: usize,
}

struct Target {
    t: usize,
    tx: f64,
    ty: f64,
    rot: f64,
}

/// Find time offset, axis orientation and focal length that best explain the
/// optical-flow motion with gyro rotation.
pub fn auto_sync(g: &GyroData, flow: &[FrameMotion], fps: f64, width: f64, cfg: &GyroConfig) -> Option<SyncResult> {
    let targets: Vec<Target> = flow
        .iter()
        .enumerate()
        .skip(1)
        .filter(|(_, m)| m.confidence >= 0.3)
        .map(|(t, m)| {
            let p = Params::from_mat(&m.m);
            Target { t, tx: p.0[TX], ty: p.0[TY], rot: p.0[ROT] }
        })
        .collect();
    if targets.len() < 10 {
        return None;
    }
    let integ = Integrator::new(g);
    let fixed_f = if cfg.focal_px > 0.0 { cfg.focal_px } else { focal_from_hfov(width, cfg.hfov) };
    let orients =
        if cfg.sync_orientation { Orientation::all() } else { vec![Orientation::parse(&cfg.orientation).ok()?] };

    // Rotation vectors per target frame for a given offset (gyro frame).
    let phis = |offset: f64| -> Vec<[f64; 3]> {
        targets.iter().map(|tg| integ.between((tg.t - 1) as f64 / fps - offset, tg.t as f64 / fps - offset)).collect()
    };
    let score = |phi_g: &[[f64; 3]], o: &Orientation| -> Option<(f64, f64)> {
        let phi: Vec<[f64; 3]> = phi_g.iter().map(|v| o.apply(*v)).collect();
        let f = if cfg.sync_focal {
            let (mut num, mut den) = (0.0, 0.0);
            for (tg, p) in targets.iter().zip(&phi) {
                num += -p[1] * tg.tx + p[0] * tg.ty;
                den += p[0] * p[0] + p[1] * p[1];
            }
            if den < 1e-12 {
                return None;
            }
            let f = num / den;
            if !(0.1 * width..=10.0 * width).contains(&f) {
                return None;
            }
            f
        } else {
            fixed_f
        };
        let (mut res, mut norm) = (0.0, 0.0);
        for (tg, p) in targets.iter().zip(&phi) {
            let (ax, ay, ar) = (tg.tx / f, tg.ty / f, tg.rot);
            res += (ax + p[1]).powi(2) + (ay - p[0]).powi(2) + (ar + p[2]).powi(2);
            norm += ax * ax + ay * ay + ar * ar;
        }
        Some((res / norm.max(1e-12), f))
    };

    let base = cfg.time_offset;
    let range = cfg.sync_range.max(0.0);
    let coarse = 0.5 / fps;
    let steps = (range / coarse).ceil() as i64;
    let mut cand: Vec<(f64, usize, f64, f64)> = Vec::new(); // score, orient idx, offset, f
    for s in -steps..=steps {
        let off = base + s as f64 * coarse;
        let pg = phis(off);
        for (oi, o) in orients.iter().enumerate() {
            if let Some((sc, f)) = score(&pg, o) {
                cand.push((sc, oi, off, f));
            }
        }
    }
    cand.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let mut best: Option<(f64, usize, f64, f64)> = None;
    let mut tried = Vec::new();
    for &(_, oi, off, _) in cand.iter() {
        if tried.len() >= 4 {
            break;
        }
        if tried.iter().any(|&(o2, f2): &(usize, f64)| o2 == oi && (f2 - off).abs() < 2.0 * coarse) {
            continue;
        }
        tried.push((oi, off));
        // Golden-section-ish refinement by dense local scan.
        let mut local = (f64::MAX, off, 0.0);
        for k in -20..=20 {
            let o2 = off + k as f64 * coarse / 10.0;
            if let Some((sc, f)) = score(&phis(o2), &orients[oi]) {
                if sc < local.0 {
                    local = (sc, o2, f);
                }
            }
        }
        if best.map_or(true, |b| local.0 < b.0) {
            best = Some((local.0, oi, local.1, local.2));
        }
    }
    let (sc, oi, off, f) = best?;
    Some(SyncResult {
        offset: off,
        orientation: orients[oi].to_string(),
        focal: f,
        score: sc,
        frames_used: targets.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orientation_parse_and_display() {
        let o = Orientation::parse("yXz").unwrap();
        assert_eq!(o.apply([1.0, 2.0, 3.0]), [-2.0, 1.0, -3.0]);
        assert_eq!(o.to_string(), "yXz");
        assert!(Orientation::parse("XXZ").is_err());
        assert_eq!(Orientation::all().len(), 48);
    }

    #[test]
    fn csv_variants() {
        let a = "time,gyro_x,gyro_y,gyro_z\n0,0.1,0.2,0.3\n5,0.1,0.2,0.3\n10,0.1,0.2,0.3\n";
        let g = GyroData::from_csv(a, TimeUnit::Auto, AngleUnit::Auto).unwrap();
        assert!((g.t[1] - 0.005).abs() < 1e-12, "ms detected");
        let b = "GYROFLOW IMU LOG\ntscale,0.001\ngscale,0.5\nt,gx,gy,gz\n0,2,0,0\n10,2,0,0\n";
        let g = GyroData::from_csv(b, TimeUnit::Auto, AngleUnit::Auto).unwrap();
        assert!((g.t[1] - 0.01).abs() < 1e-12 && (g.w[0][0] - 1.0).abs() < 1e-12);
        let c = "0.0 10 0 0\n0.01 10 0 0\n0.02 10 0 0\n0.03 100 0 0";
        let g = GyroData::from_csv(c, TimeUnit::Auto, AngleUnit::Auto).unwrap();
        assert!((g.w[0][0] - 10f64.to_radians()).abs() < 1e-9, "deg detected");
    }

    #[test]
    fn integrator_constant_rate() {
        let g = GyroData::new((0..=100).map(|i| (i as f64 * 0.01, [1.0, -2.0, 0.5])).collect()).unwrap();
        let it = Integrator::new(&g);
        let d = it.between(0.105, 0.505);
        assert!((d[0] - 0.4).abs() < 1e-9 && (d[1] + 0.8).abs() < 1e-9);
    }

    #[test]
    fn yaw_moves_image_horizontally() {
        // Small rotation about camera y (yaw right) moves scene left in the image.
        let h = rotation_homography([0.0, 0.01, 0.0], 1000.0);
        let (x, _) = h.apply(0.0, 0.0);
        assert!((x + 10.0).abs() < 0.1, "{x}");
    }
}
