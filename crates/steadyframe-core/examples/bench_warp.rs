use std::time::Instant;
use steadyframe_core::config::*;
use steadyframe_core::geometry::Mat3;
use steadyframe_core::warp::{warp_rgba, WarpSpec};
fn main() {
    let (w, h) = (1280usize, 720usize);
    let src: Vec<u8> = (0..w * h * 4).map(|i| (i * 31 % 251) as u8).collect();
    let mut dst = vec![0u8; w * h * 4];
    for interp in [Interpolation::Bilinear, Interpolation::Bicubic] {
        let spec = WarpSpec {
            inv: Mat3::rotation(0.02).mul(&Mat3::scale(0.8, 0.8)),
            rs: None,
            interpolation: interp,
            border: BorderMode::Black,
            border_color: [0; 4],
        };
        let t = Instant::now();
        for _ in 0..20 {
            warp_rgba(&src, w, h, &mut dst, w, h, &spec);
        }
        println!("{interp:?}: {:.1} ms/frame", t.elapsed().as_secs_f64() * 1000.0 / 20.0);
    }
}
