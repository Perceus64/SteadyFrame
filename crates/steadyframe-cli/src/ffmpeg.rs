//! Thin wrappers around the `ffmpeg` / `ffprobe` executables.

use anyhow::{anyhow, bail, Context, Result};
use serde_json::Value;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

#[derive(Debug, Clone)]
pub struct Probe {
    pub width: u32,
    pub height: u32,
    /// Exact rate string for the encoder (e.g. "30000/1001").
    pub fps_str: String,
    pub fps: f64,
    pub frames_estimate: Option<usize>,
    /// Codec of the first audio stream, if any.
    pub audio_codec: Option<String>,
    /// Stream index of a GoPro GPMF (`gpmd`) data track, if any.
    pub gpmf_stream: Option<usize>,
    pub codec: String,
}

fn parse_rate(s: &str) -> Option<f64> {
    let mut it = s.split('/');
    let n: f64 = it.next()?.trim().parse().ok()?;
    let d: f64 = it.next().map(|d| d.trim().parse().ok()).unwrap_or(Some(1.0))?;
    if d == 0.0 || n <= 0.0 {
        None
    } else {
        Some(n / d)
    }
}

pub struct Tools {
    pub ffmpeg: String,
    pub ffprobe: String,
}

impl Tools {
    pub fn check(&self) -> Result<()> {
        for exe in [&self.ffmpeg, &self.ffprobe] {
            let ok = Command::new(exe).arg("-version").stdout(Stdio::null()).stderr(Stdio::null()).status();
            if !matches!(ok, Ok(s) if s.success()) {
                bail!("'{exe}' was not found. Install FFmpeg (https://ffmpeg.org/download.html) or pass --ffmpeg/--ffprobe");
            }
        }
        Ok(())
    }

    pub fn probe(&self, input: &Path) -> Result<Probe> {
        let out = Command::new(&self.ffprobe)
            .args(["-v", "error", "-print_format", "json", "-show_streams", "-show_format"])
            .arg(input)
            .output()
            .with_context(|| format!("running {}", self.ffprobe))?;
        if !out.status.success() {
            bail!("ffprobe failed on {}: {}", input.display(), String::from_utf8_lossy(&out.stderr).trim());
        }
        let v: Value = serde_json::from_slice(&out.stdout).context("parsing ffprobe output")?;
        let streams = v["streams"].as_array().cloned().unwrap_or_default();
        let video = streams
            .iter()
            .find(|s| s["codec_type"] == "video" && s["disposition"]["attached_pic"].as_i64() != Some(1))
            .ok_or_else(|| anyhow!("{} has no video stream", input.display()))?;
        let mut width = video["width"].as_u64().unwrap_or(0) as u32;
        let mut height = video["height"].as_u64().unwrap_or(0) as u32;
        // Phone videos carry a rotation; ffmpeg auto-rotates when decoding.
        let mut rotation = video["tags"]["rotate"].as_str().and_then(|r| r.parse::<i64>().ok()).unwrap_or(0);
        if let Some(sd) = video["side_data_list"].as_array() {
            for d in sd {
                if let Some(r) = d["rotation"].as_i64() {
                    rotation = r;
                }
            }
        }
        if rotation.rem_euclid(180) == 90 {
            std::mem::swap(&mut width, &mut height);
        }
        let r_rate = video["r_frame_rate"].as_str().unwrap_or("");
        let a_rate = video["avg_frame_rate"].as_str().unwrap_or("");
        let (fps_str, fps) = match (parse_rate(a_rate), parse_rate(r_rate)) {
            (Some(a), Some(r)) if (a - r).abs() / r < 0.01 => (r_rate.to_string(), r),
            (Some(a), _) if a < 1000.0 => (a_rate.to_string(), a),
            (_, Some(r)) if r < 1000.0 => (r_rate.to_string(), r),
            _ => ("30".to_string(), 30.0),
        };
        let duration =
            video["duration"].as_str().or_else(|| v["format"]["duration"].as_str()).and_then(|d| d.parse::<f64>().ok());
        let frames_estimate = video["nb_frames"]
            .as_str()
            .and_then(|n| n.parse::<usize>().ok())
            .filter(|n| *n > 0)
            .or_else(|| duration.map(|d| (d * fps).round() as usize));
        let audio_codec = streams
            .iter()
            .find(|s| s["codec_type"] == "audio")
            .map(|s| s["codec_name"].as_str().unwrap_or("unknown").to_string());
        let gpmf_stream = streams
            .iter()
            .find(|s| {
                s["codec_tag_string"] == "gpmd"
                    || s["tags"]["handler_name"].as_str().is_some_and(|h| h.contains("GoPro MET"))
            })
            .and_then(|s| s["index"].as_u64())
            .map(|i| i as usize);
        if width < 16 || height < 16 {
            bail!("{}: unsupported video size {width}x{height}", input.display());
        }
        Ok(Probe {
            width,
            height,
            fps_str,
            fps,
            frames_estimate,
            audio_codec,
            gpmf_stream,
            codec: video["codec_name"].as_str().unwrap_or("?").to_string(),
        })
    }

    /// Exact frame count by counting packets (fast, no decode).
    pub fn count_frames(&self, input: &Path) -> Result<usize> {
        let out = Command::new(&self.ffprobe)
            .args([
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-count_packets",
                "-show_entries",
                "stream=nb_read_packets",
                "-of",
                "csv=p=0",
            ])
            .arg(input)
            .output()?;
        let s = String::from_utf8_lossy(&out.stdout);
        s.trim()
            .trim_end_matches(',')
            .parse::<usize>()
            .map_err(|_| anyhow!("could not count frames of {}", input.display()))
    }

    /// Extract the raw GPMF data track into memory.
    pub fn extract_gpmf(&self, input: &Path, stream: usize) -> Result<Vec<u8>> {
        let tmp: PathBuf = std::env::temp_dir().join(format!("steadyframe-{}-{stream}.gpmf", std::process::id()));
        let st = Command::new(&self.ffmpeg)
            .args(["-v", "error", "-nostdin", "-y", "-i"])
            .arg(input)
            .args(["-map", &format!("0:{stream}"), "-c", "copy", "-f", "rawvideo"])
            .arg(&tmp)
            .status()?;
        if !st.success() {
            bail!("ffmpeg could not extract the GPMF track from {}", input.display());
        }
        let data = std::fs::read(&tmp)?;
        let _ = std::fs::remove_file(&tmp);
        Ok(data)
    }

    /// Start decoding raw frames. `scale` resizes; `pix_fmt` is "gray" or "rgba".
    pub fn decoder(
        &self,
        input: &Path,
        scale: Option<(u32, u32)>,
        pix_fmt: &str,
        frame_bytes: usize,
    ) -> Result<Decoder> {
        let mut cmd = Command::new(&self.ffmpeg);
        cmd.args(["-v", "error", "-nostdin", "-i"])
            .arg(input)
            .args(["-map", "0:v:0", "-vsync", "0", "-an", "-sn", "-dn"]);
        if let Some((w, h)) = scale {
            cmd.args(["-vf", &format!("scale={w}:{h}:flags=area")]);
        }
        cmd.args(["-f", "rawvideo", "-pix_fmt", pix_fmt, "-"]);
        let mut child = cmd
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .with_context(|| format!("starting {}", self.ffmpeg))?;
        let stdout = child.stdout.take().unwrap();
        Ok(Decoder { child, stdout, frame_bytes })
    }

    pub fn encoder(
        &self,
        opts: &EncodeOptions,
        input: &Path,
        output: &Path,
        w: u32,
        h: u32,
        fps_str: &str,
        audio_codec: Option<&str>,
    ) -> Result<Encoder> {
        let mut cmd = Command::new(&self.ffmpeg);
        cmd.args([
            "-v",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgba",
            "-s",
            &format!("{w}x{h}"),
            "-r",
            fps_str,
            "-i",
            "-",
        ]);
        let audio = audio_codec.is_some() && opts.audio != "none";
        let ext = output.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
        let mp4_like = matches!(ext.as_str(), "mp4" | "mov" | "m4v");
        // "copy" only works when the container can hold the source audio codec
        // (e.g. camera .MOV files often carry PCM, which MP4 can't).
        let audio_codec_arg = match (opts.audio.as_str(), audio_codec) {
            ("copy", Some(c)) if ext == "mp4" || ext == "m4v" => {
                if matches!(c, "aac" | "mp3" | "alac" | "ac3" | "eac3" | "opus" | "flac") {
                    "copy".to_string()
                } else {
                    "aac".to_string()
                }
            }
            ("copy", Some(c)) if ext == "webm" && !matches!(c, "opus" | "vorbis") => "libopus".to_string(),
            (a, _) => a.to_string(),
        };
        if audio {
            cmd.arg("-i").arg(input);
        }
        cmd.args(["-map", "0:v:0"]);
        if audio {
            cmd.args(["-map", "1:a?", "-c:a", &audio_codec_arg]);
            if audio_codec_arg == "aac" {
                cmd.args(["-b:a", "192k"]);
            }
        }
        cmd.args(["-c:v", &opts.codec]);
        let c = opts.codec.as_str();
        if matches!(c, "libx264" | "libx265" | "libvpx-vp9" | "libaom-av1" | "libsvtav1") {
            cmd.args(["-crf", &opts.crf.to_string()]);
        }
        if c == "libvpx-vp9" {
            cmd.args(["-b:v", "0"]);
        }
        if matches!(c, "libx264" | "libx265") {
            cmd.args(["-preset", &opts.preset]);
        }
        if !opts.pix_fmt.is_empty() {
            if opts.pix_fmt.starts_with("yuv") {
                // Convert RGB→YUV with BT.709 (HD) coefficients and tag the stream,
                // otherwise players shift colours slightly.
                cmd.args(["-vf", &format!("scale=out_color_matrix=bt709:out_range=tv,format={}", opts.pix_fmt)]);
                cmd.args([
                    "-colorspace",
                    "bt709",
                    "-color_primaries",
                    "bt709",
                    "-color_trc",
                    "bt709",
                    "-color_range",
                    "tv",
                ]);
            } else {
                cmd.args(["-pix_fmt", &opts.pix_fmt]);
            }
        }
        if mp4_like {
            cmd.args(["-movflags", "+faststart"]);
            if c == "libx265" {
                cmd.args(["-tag:v", "hvc1"]);
            }
        }
        for a in opts.extra.split_whitespace() {
            cmd.arg(a);
        }
        cmd.arg(output);
        let mut child = cmd
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()
            .with_context(|| format!("starting {}", self.ffmpeg))?;
        let stdin = child.stdin.take().unwrap();
        Ok(Encoder { child, stdin: Some(stdin) })
    }
}

#[derive(Debug, Clone)]
pub struct EncodeOptions {
    pub codec: String,
    pub crf: u32,
    pub preset: String,
    pub pix_fmt: String,
    pub audio: String,
    pub extra: String,
}

pub struct Decoder {
    child: Child,
    stdout: ChildStdout,
    frame_bytes: usize,
}

impl Decoder {
    /// Read the next frame into `buf`; Ok(false) at end of stream.
    pub fn next_frame(&mut self, buf: &mut Vec<u8>) -> Result<bool> {
        buf.resize(self.frame_bytes, 0);
        let mut filled = 0;
        while filled < self.frame_bytes {
            let n = self.stdout.read(&mut buf[filled..])?;
            if n == 0 {
                if filled == 0 {
                    return Ok(false);
                }
                bail!("decoder produced a truncated frame ({filled}/{} bytes)", self.frame_bytes);
            }
            filled += n;
        }
        Ok(true)
    }

    pub fn finish(mut self) -> Result<()> {
        let mut err = String::new();
        if let Some(mut e) = self.child.stderr.take() {
            let _ = e.read_to_string(&mut err);
        }
        let st = self.child.wait()?;
        if !st.success() {
            bail!("ffmpeg decoder failed: {}", err.trim());
        }
        Ok(())
    }
}

pub struct Encoder {
    child: Child,
    stdin: Option<ChildStdin>,
}

impl Encoder {
    pub fn write(&mut self, frame: &[u8]) -> Result<()> {
        match self.stdin.as_mut() {
            Some(s) => s
                .write_all(frame)
                .map_err(|e| anyhow!("encoder stopped accepting frames ({e}); see the ffmpeg error below")),
            None => bail!("encoder already closed"),
        }
    }

    pub fn finish(mut self) -> Result<()> {
        drop(self.stdin.take());
        let mut err = String::new();
        if let Some(mut e) = self.child.stderr.take() {
            let _ = e.read_to_string(&mut err);
        }
        let st = self.child.wait()?;
        if !st.success() {
            bail!("ffmpeg encoder failed: {}", err.trim());
        }
        Ok(())
    }
}
