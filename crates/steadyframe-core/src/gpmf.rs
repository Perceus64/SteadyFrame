//! Minimal GoPro Metadata Format (GPMF) parser that extracts the GYRO stream.
//!
//! Input is the raw `gpmd` data track (e.g. extracted with
//! `ffmpeg -i GX010001.MP4 -map 0:d:0 -c copy -f rawvideo gpmf.bin`; the CLI does
//! this automatically). Each payload is KLV-encoded: a 4-byte key, a type byte,
//! a struct-size byte and a big-endian 16-bit repeat count, padded to 4 bytes.
//!
//! Sample timing uses the payload `STMP` timestamps when present, otherwise one
//! payload per second (GoPro's default). The axis order from `ORIN` is applied
//! when present; remaining sign/axis ambiguity is resolved by gyro auto-sync.

use crate::error::{Error, Result};
use crate::gyro::GyroData;

#[derive(Default, Debug)]
struct Chunk {
    stmp: Option<f64>,
    samples: Vec<[f64; 3]>,
}

#[derive(Default)]
struct StreamState {
    scal: Vec<f64>,
    orin: Option<String>,
    stmp: Option<f64>,
    raw: Vec<[f64; 3]>,
    has_gyro: bool,
}

fn read_values(ty: u8, size: usize, data: &[u8]) -> Vec<f64> {
    let elem = match ty {
        b'b' | b'B' | b'c' => 1,
        b's' | b'S' => 2,
        b'l' | b'L' | b'f' => 4,
        b'j' | b'J' | b'd' => 8,
        _ => return Vec::new(),
    };
    let _ = size;
    data.chunks_exact(elem)
        .map(|c| match ty {
            b'b' => c[0] as i8 as f64,
            b'B' | b'c' => c[0] as f64,
            b's' => i16::from_be_bytes([c[0], c[1]]) as f64,
            b'S' => u16::from_be_bytes([c[0], c[1]]) as f64,
            b'l' => i32::from_be_bytes([c[0], c[1], c[2], c[3]]) as f64,
            b'L' => u32::from_be_bytes([c[0], c[1], c[2], c[3]]) as f64,
            b'f' => f32::from_be_bytes([c[0], c[1], c[2], c[3]]) as f64,
            b'j' => i64::from_be_bytes(c.try_into().unwrap()) as f64,
            b'J' => u64::from_be_bytes(c.try_into().unwrap()) as f64,
            b'd' => f64::from_be_bytes(c.try_into().unwrap()),
            _ => 0.0,
        })
        .collect()
}

fn walk(buf: &[u8], chunks: &mut Vec<Chunk>, depth: usize) -> Result<()> {
    if depth > 8 {
        return Err(Error::Parse("GPMF nesting too deep".into()));
    }
    let mut pos = 0;
    let mut st = StreamState::default();
    while pos + 8 <= buf.len() {
        let key = &buf[pos..pos + 4];
        let ty = buf[pos + 4];
        let size = buf[pos + 5] as usize;
        let repeat = u16::from_be_bytes([buf[pos + 6], buf[pos + 7]]) as usize;
        let len = size * repeat;
        let padded = (len + 3) & !3;
        let start = pos + 8;
        if key == [0, 0, 0, 0] {
            break;
        }
        if start + len > buf.len() {
            // A cut-off final record (e.g. a recording that ended abruptly):
            // keep everything parsed so far instead of failing.
            if depth == 0 && !chunks.is_empty() {
                break;
            }
            return Err(Error::Parse(format!("GPMF: truncated '{}' entry", String::from_utf8_lossy(key))));
        }
        let data = &buf[start..start + len];
        if ty == 0 {
            walk(data, chunks, depth + 1)?;
        } else {
            match key {
                b"SCAL" => st.scal = read_values(ty, size, data),
                b"ORIN" => st.orin = Some(String::from_utf8_lossy(data).trim_end_matches('\0').to_string()),
                b"STMP" => st.stmp = read_values(ty, size, data).first().copied().map(|us| us * 1e-6),
                b"GYRO" => {
                    st.has_gyro = true;
                    let vals = read_values(ty, size, data);
                    for v in vals.chunks_exact(3) {
                        st.raw.push([v[0], v[1], v[2]]);
                    }
                }
                _ => {}
            }
        }
        pos = start + padded;
    }
    if st.has_gyro && !st.raw.is_empty() {
        let sc = |i: usize| -> f64 {
            match st.scal.len() {
                0 => 1.0,
                1 => st.scal[0],
                _ => st.scal.get(i).copied().unwrap_or(1.0),
            }
        };
        let mut samples: Vec<[f64; 3]> = st
            .raw
            .iter()
            .map(|v| [v[0] / sc(0).max(1e-12), v[1] / sc(1).max(1e-12), v[2] / sc(2).max(1e-12)])
            .collect();
        if let Some(orin) = &st.orin {
            samples = samples.iter().map(|v| apply_orin(orin, *v)).collect();
        }
        chunks.push(Chunk { stmp: st.stmp, samples });
    }
    Ok(())
}

/// ORIN gives, per data column, which camera axis it is (lowercase = negated).
/// Reorder into (X, Y, Z).
fn apply_orin(orin: &str, v: [f64; 3]) -> [f64; 3] {
    let chars: Vec<char> = orin.chars().take(3).collect();
    if chars.len() != 3 {
        return v;
    }
    let mut out = [0.0; 3];
    for (col, c) in chars.iter().enumerate() {
        let axis = match c.to_ascii_uppercase() {
            'X' => 0,
            'Y' => 1,
            'Z' => 2,
            _ => return v,
        };
        out[axis] = if c.is_ascii_lowercase() { -v[col] } else { v[col] };
    }
    out
}

/// Parse a raw GPMF track into gyro samples (rad/s).
pub fn parse_gpmf(buf: &[u8]) -> Result<GyroData> {
    let mut chunks = Vec::new();
    walk(buf, &mut chunks, 0)?;
    if chunks.is_empty() {
        return Err(Error::Gyro("no GYRO stream found in GPMF data".into()));
    }
    let have_stmp = chunks.iter().all(|c| c.stmp.is_some());
    let starts: Vec<f64> = if have_stmp {
        chunks.iter().map(|c| c.stmp.unwrap()).collect()
    } else {
        (0..chunks.len()).map(|i| i as f64).collect()
    };
    let mut samples = Vec::new();
    for (i, c) in chunks.iter().enumerate() {
        let start = starts[i];
        let dur = if i + 1 < chunks.len() {
            starts[i + 1] - start
        } else if i > 0 {
            // Same sample rate as the previous chunk.
            let prev = (starts[i] - starts[i - 1]) / chunks[i - 1].samples.len() as f64;
            prev * c.samples.len() as f64
        } else {
            1.0
        };
        let n = c.samples.len().max(1) as f64;
        for (k, s) in c.samples.iter().enumerate() {
            samples.push((start + dur * k as f64 / n, *s));
        }
    }
    GyroData::new(samples)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn klv(key: &[u8; 4], ty: u8, size: u8, repeat: u16, data: &[u8]) -> Vec<u8> {
        let mut v = key.to_vec();
        v.push(ty);
        v.push(size);
        v.extend_from_slice(&repeat.to_be_bytes());
        v.extend_from_slice(data);
        while v.len() % 4 != 0 {
            v.push(0);
        }
        v
    }

    #[test]
    fn parses_nested_gyro() {
        let mut payloads = Vec::new();
        for p in 0..2u64 {
            let mut strm = Vec::new();
            strm.extend(klv(b"STMP", b'J', 8, 1, &(p * 1_000_000).to_be_bytes()));
            strm.extend(klv(b"SCAL", b's', 2, 1, &100i16.to_be_bytes()));
            strm.extend(klv(b"ORIN", b'c', 1, 3, b"ZXY"));
            let mut g = Vec::new();
            for i in 0..4i16 {
                for v in [10 * i, 20, -30] {
                    g.extend_from_slice(&v.to_be_bytes());
                }
            }
            strm.extend(klv(b"GYRO", b's', 6, 4, &g));
            let s = klv(b"STRM", 0, 1, strm.len() as u16, &strm);
            payloads.extend(klv(b"DEVC", 0, 1, s.len() as u16, &s));
        }
        let gd = parse_gpmf(&payloads).unwrap();
        assert_eq!(gd.len(), 8);
        assert!((gd.t[4] - 1.0).abs() < 1e-9);
        // Columns were Z,X,Y → sample 1: Z=0.1, X=0.2, Y=-0.3
        assert_eq!(gd.w[1], [0.2, -0.3, 0.1]);

        // A truncated trailing payload keeps the samples parsed before it.
        let mut cut = payloads.clone();
        cut.extend_from_slice(b"DEVC\0\x01\x01\x00");
        assert_eq!(parse_gpmf(&cut).unwrap().len(), 8);
    }
}
