//! `steadyframe` — stabilize videos from the command line.

#![allow(clippy::too_many_arguments)]

mod ffmpeg;

use anyhow::{anyhow, bail, Context, Result};
use clap::Parser;
use ffmpeg::{EncodeOptions, Probe, Tools};
use serde_json::{json, Value};
use std::io::{IsTerminal, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;
use steadyframe_core::config::*;
use steadyframe_core::gpmf::parse_gpmf;
use steadyframe_core::{Analyzer, Config, GyroData, MotionTrack, Plan, Renderer};

const EXAMPLES: &str = "\
EXAMPLES:
  steadyframe shaky.mp4                          # → shaky_stabilized.mp4
  steadyframe in.mp4 -o out.mp4 --preset walking --horizon-lock
  steadyframe *.mp4 --output-dir stabilized --preset cinematic
  steadyframe in.mp4 --smoothing kalman --radius 20 --crop dynamic --max-zoom 1.3
  steadyframe in.mp4 --tripod --compare side-by-side
  steadyframe GX010042.MP4 --preset gyro          # GoPro: uses embedded GPMF gyro
  steadyframe phone.mp4 --gyro phone_gyro.csv --rolling-shutter 0.6
  steadyframe in.mp4 --config my.json --set smoothing.radius=50 --set render.border=reflect
  steadyframe in.mp4 --analyze-only --export-track in.track.json --report report.json
  steadyframe in.mp4 --import-track in.track.json --preset smooth   # re-render, no re-analysis";

#[derive(Parser, Debug)]
#[command(name = "steadyframe", version, args_override_self = true, about = "Fast, customizable video stabilization (Rust, runs anywhere ffmpeg does)", after_help = EXAMPLES)]
struct Cli {
    /// Input video file(s).
    #[arg(required_unless_present_any = ["list_presets", "print_config"])]
    inputs: Vec<PathBuf>,

    /// Output file (single input only). Default: <input><suffix>.mp4
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Directory for outputs (default: next to each input).
    #[arg(long)]
    output_dir: Option<PathBuf>,
    /// Suffix appended to output names.
    #[arg(long, default_value = "_stabilized")]
    suffix: String,
    /// Overwrite existing outputs.
    #[arg(short = 'y', long)]
    overwrite: bool,

    // ── configuration ─────────────────────────────────────────────
    /// Starting preset: handheld, smooth, walking, vehicle, action, cinematic, tripod, gentle, gyro.
    #[arg(short, long, default_value = "handheld", help_heading = "Configuration")]
    preset: String,
    /// JSON config file (full or partial) applied on top of the preset.
    #[arg(short, long, help_heading = "Configuration")]
    config: Option<PathBuf>,
    /// Override any config value: --set smoothing.radius=40 (repeatable).
    #[arg(long = "set", value_name = "KEY=VALUE", help_heading = "Configuration")]
    sets: Vec<String>,
    /// Print the effective configuration as JSON and exit.
    #[arg(long, help_heading = "Configuration")]
    print_config: bool,
    /// List presets and exit.
    #[arg(long, help_heading = "Configuration")]
    list_presets: bool,

    // ── analysis ──────────────────────────────────────────────────
    /// translation | similarity | affine | homography
    #[arg(long, help_heading = "Motion analysis")]
    motion_model: Option<MotionModel>,
    /// optical-flow | gyro | hybrid
    #[arg(long, help_heading = "Motion analysis")]
    motion_source: Option<MotionSource>,
    /// Width frames are analysed at (smaller = faster).
    #[arg(long, help_heading = "Motion analysis")]
    analysis_width: Option<u32>,
    #[arg(long, help_heading = "Motion analysis")]
    max_features: Option<u32>,

    // ── smoothing ─────────────────────────────────────────────────
    /// gaussian | moving-average | kalman | l1
    #[arg(long, help_heading = "Smoothing")]
    smoothing: Option<SmoothingMethod>,
    /// Smoothing strength in frames.
    #[arg(short, long, help_heading = "Smoothing")]
    radius: Option<f64>,
    /// Multiplier of radius for x/y (0 = keep original translation).
    #[arg(long, help_heading = "Smoothing")]
    translation_smoothing: Option<f64>,
    /// Multiplier of radius for rotation (0 = keep original rotation).
    #[arg(long, help_heading = "Smoothing")]
    rotation_smoothing: Option<f64>,
    /// Multiplier of radius for zoom/shear/perspective.
    #[arg(long, help_heading = "Smoothing")]
    zoom_smoothing: Option<f64>,
    /// Lock the frame completely (virtual tripod).
    #[arg(long, help_heading = "Smoothing")]
    tripod: bool,
    /// Remove all roll (keep the horizon level).
    #[arg(long, help_heading = "Smoothing")]
    horizon_lock: bool,
    /// Extra output rotation in degrees with --horizon-lock.
    #[arg(long, allow_hyphen_values = true, help_heading = "Smoothing")]
    horizon_angle: Option<f64>,
    /// Remove all zoom changes.
    #[arg(long, help_heading = "Smoothing")]
    zoom_lock: bool,
    /// Reference frame for tripod/horizon/zoom locks.
    #[arg(long, help_heading = "Smoothing")]
    reference_frame: Option<u32>,
    /// Max correction shift as a fraction of the frame (0 = unlimited).
    #[arg(long, help_heading = "Smoothing")]
    limit_shift: Option<f64>,
    /// Max correction rotation in degrees (0 = unlimited).
    #[arg(long, help_heading = "Smoothing")]
    limit_rotation: Option<f64>,
    /// Max correction scale change as a fraction (0 = unlimited).
    #[arg(long, help_heading = "Smoothing")]
    limit_scale: Option<f64>,

    // ── crop / render ─────────────────────────────────────────────
    /// none | fixed | dynamic
    #[arg(long, help_heading = "Crop & render")]
    crop: Option<CropMode>,
    #[arg(long, help_heading = "Crop & render")]
    max_zoom: Option<f64>,
    #[arg(long, help_heading = "Crop & render")]
    extra_zoom: Option<f64>,
    /// nearest | bilinear | bicubic
    #[arg(long, help_heading = "Crop & render")]
    interpolation: Option<Interpolation>,
    /// black | color | replicate | reflect | previous
    #[arg(long, help_heading = "Crop & render")]
    border: Option<BorderMode>,
    /// Border color as hex (e.g. 1a1a1a); implies --border color.
    #[arg(long, help_heading = "Crop & render")]
    border_color: Option<String>,
    /// Output width (with --height).
    #[arg(long, requires = "height", help_heading = "Crop & render")]
    width: Option<u32>,
    /// Output height (with --width).
    #[arg(long, requires = "width", help_heading = "Crop & render")]
    height: Option<u32>,
    /// none | side-by-side | split — render a before/after comparison.
    #[arg(long, help_heading = "Crop & render")]
    compare: Option<CompareMode>,
    /// Enable rolling-shutter correction with this readout (fraction of frame time, e.g. 0.5).
    #[arg(long, allow_hyphen_values = true, help_heading = "Crop & render")]
    rolling_shutter: Option<f64>,

    // ── gyro ──────────────────────────────────────────────────────
    /// auto | gpmf | none | <file.csv|file.bin>. auto uses embedded GoPro GPMF when needed.
    #[arg(long, help_heading = "Gyro")]
    gyro: Option<String>,
    /// Gyro→camera axis mapping, e.g. XYZ or yXz.
    #[arg(long, help_heading = "Gyro")]
    gyro_orientation: Option<String>,
    /// Gyro time offset in seconds.
    #[arg(long, allow_hyphen_values = true, help_heading = "Gyro")]
    gyro_offset: Option<f64>,
    /// Horizontal field of view (degrees).
    #[arg(long, help_heading = "Gyro")]
    hfov: Option<f64>,
    /// Focal length in pixels (overrides --hfov).
    #[arg(long, help_heading = "Gyro")]
    focal: Option<f64>,
    /// Disable automatic gyro synchronisation.
    #[arg(long, help_heading = "Gyro")]
    no_gyro_sync: bool,

    // ── encoding ──────────────────────────────────────────────────
    #[arg(long, default_value = "libx264", help_heading = "Encoding")]
    codec: String,
    #[arg(long, default_value_t = 18, help_heading = "Encoding")]
    crf: u32,
    /// x264/x265 speed preset.
    #[arg(long, default_value = "medium", help_heading = "Encoding")]
    encoder_preset: String,
    #[arg(long, default_value = "yuv420p", help_heading = "Encoding")]
    pix_fmt: String,
    /// Audio codec for the output ("copy", "aac", or "none" to drop audio).
    #[arg(long, default_value = "copy", help_heading = "Encoding")]
    audio: String,
    /// Extra ffmpeg output arguments, e.g. "-b:v 8M -tune film".
    #[arg(long, default_value = "", allow_hyphen_values = true, help_heading = "Encoding")]
    ffmpeg_args: String,
    #[arg(long, default_value = "ffmpeg", help_heading = "Encoding")]
    ffmpeg: String,
    #[arg(long, default_value = "ffprobe", help_heading = "Encoding")]
    ffprobe: String,

    // ── data & reports ────────────────────────────────────────────
    /// Save the analysed motion track (JSON). A directory when processing several inputs.
    #[arg(long, help_heading = "Data & reports")]
    export_track: Option<PathBuf>,
    /// Re-use a saved motion track instead of analysing.
    #[arg(long, help_heading = "Data & reports")]
    import_track: Option<PathBuf>,
    /// Save per-frame transforms (JSON).
    #[arg(long, help_heading = "Data & reports")]
    export_plan: Option<PathBuf>,
    /// Write a JSON report with shake metrics for all inputs.
    #[arg(long, help_heading = "Data & reports")]
    report: Option<PathBuf>,
    /// Analyse and report only; don't render a video.
    #[arg(long, help_heading = "Data & reports")]
    analyze_only: bool,
    /// Process at most this many frames (for quick tests).
    #[arg(long, help_heading = "Data & reports")]
    max_frames: Option<usize>,
    /// Worker threads (default: all cores).
    #[arg(short = 'j', long, help_heading = "Data & reports")]
    threads: Option<usize>,
    #[arg(short, long, help_heading = "Data & reports")]
    quiet: bool,
}

fn main() {
    let cli = Cli::parse();
    match run(cli) {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("error: {e:#}");
            std::process::exit(2);
        }
    }
}

/// Build a nested JSON object from a dotted key, parsing the value as JSON when possible.
fn set_to_json(spec: &str) -> Result<Value> {
    let (k, v) = spec.split_once('=').ok_or_else(|| anyhow!("--set expects KEY=VALUE, got '{spec}'"))?;
    let val: Value = serde_json::from_str(v.trim()).unwrap_or_else(|_| Value::String(v.trim().to_string()));
    let mut out = val;
    for part in k.trim().split('.').rev() {
        let mut m = serde_json::Map::new();
        m.insert(part.to_string(), out);
        out = Value::Object(m);
    }
    Ok(out)
}

fn build_config(cli: &Cli) -> Result<Config> {
    let mut cfg = Config::preset(&cli.preset)?;
    if let Some(p) = &cli.config {
        let s = std::fs::read_to_string(p).with_context(|| format!("reading {}", p.display()))?;
        cfg = cfg.merged_with_json(&s).with_context(|| format!("in config file {}", p.display()))?;
    }
    for s in &cli.sets {
        cfg = cfg.merged_with_value(&set_to_json(s)?).with_context(|| format!("in --set {s}"))?;
    }
    let a = &mut cfg.analysis;
    if let Some(v) = cli.motion_model {
        a.motion_model = v
    }
    if let Some(v) = cli.motion_source {
        a.motion_source = v
    }
    if let Some(v) = cli.analysis_width {
        a.analysis_width = v
    }
    if let Some(v) = cli.max_features {
        a.max_features = v
    }
    let s = &mut cfg.smoothing;
    if let Some(v) = cli.smoothing {
        s.method = v
    }
    if let Some(v) = cli.radius {
        s.radius = v
    }
    if let Some(v) = cli.translation_smoothing {
        s.translation = v
    }
    if let Some(v) = cli.rotation_smoothing {
        s.rotation = v
    }
    if let Some(v) = cli.zoom_smoothing {
        s.zoom = v
    }
    if cli.tripod {
        s.tripod = true
    }
    if cli.horizon_lock {
        s.horizon_lock = true
    }
    if let Some(v) = cli.horizon_angle {
        s.horizon_angle = v
    }
    if cli.zoom_lock {
        s.zoom_lock = true
    }
    if let Some(v) = cli.reference_frame {
        s.reference_frame = v
    }
    if let Some(v) = cli.limit_shift {
        s.limit_shift = v
    }
    if let Some(v) = cli.limit_rotation {
        s.limit_rotation = v
    }
    if let Some(v) = cli.limit_scale {
        s.limit_scale = v
    }
    let c = &mut cfg.crop;
    if let Some(v) = cli.crop {
        c.mode = v
    }
    if let Some(v) = cli.max_zoom {
        c.max_zoom = v
    }
    if let Some(v) = cli.extra_zoom {
        c.extra_zoom = v
    }
    let r = &mut cfg.render;
    if let Some(v) = cli.interpolation {
        r.interpolation = v
    }
    if let Some(v) = cli.border {
        r.border = v
    }
    if let Some(hex) = &cli.border_color {
        r.border_color = parse_hex(hex)?;
        if cli.border.is_none() {
            r.border = BorderMode::Color;
        }
    }
    if let (Some(w), Some(h)) = (cli.width, cli.height) {
        r.output_width = w;
        r.output_height = h;
    }
    if let Some(v) = cli.compare {
        r.compare = v
    }
    if let Some(v) = cli.rolling_shutter {
        cfg.rolling_shutter.enabled = v != 0.0;
        cfg.rolling_shutter.readout = v;
    }
    let g = &mut cfg.gyro;
    if let Some(v) = &cli.gyro_orientation {
        g.orientation = v.clone()
    }
    if let Some(v) = cli.gyro_offset {
        g.time_offset = v
    }
    if let Some(v) = cli.hfov {
        g.hfov = v
    }
    if let Some(v) = cli.focal {
        g.focal_px = v
    }
    if cli.no_gyro_sync {
        g.auto_sync = false
    }
    cfg.validate()?;
    Ok(cfg)
}

fn parse_hex(s: &str) -> Result<[u8; 4]> {
    let h = s.trim().trim_start_matches('#');
    let byte = |i: usize| u8::from_str_radix(&h[i..i + 2], 16).map_err(|_| anyhow!("invalid hex color '{s}'"));
    match h.len() {
        6 => Ok([byte(0)?, byte(2)?, byte(4)?, 255]),
        8 => Ok([byte(0)?, byte(2)?, byte(4)?, byte(6)?]),
        _ => bail!("border color must be RRGGBB or RRGGBBAA, got '{s}'"),
    }
}

/// Simple single-line progress meter on stderr.
struct Progress {
    label: String,
    total: Option<usize>,
    start: Instant,
    last: Instant,
    enabled: bool,
    tty: bool,
}

impl Progress {
    fn new(label: String, total: Option<usize>, enabled: bool) -> Self {
        let now = Instant::now();
        Progress {
            label,
            total,
            start: now,
            last: now - std::time::Duration::from_secs(1),
            enabled,
            tty: std::io::stderr().is_terminal(),
        }
    }

    fn tick(&mut self, done: usize) {
        if !self.enabled {
            return;
        }
        let interval = if self.tty { 0.2 } else { 5.0 };
        if self.last.elapsed().as_secs_f64() < interval {
            return;
        }
        self.last = Instant::now();
        self.print(done, false);
    }

    fn print(&self, done: usize, end: bool) {
        let el = self.start.elapsed().as_secs_f64().max(1e-3);
        let fps = done as f64 / el;
        let line = match self.total {
            Some(t) if t > 0 => {
                let pct = (done as f64 / t as f64 * 100.0).min(100.0);
                let filled = (pct / 5.0) as usize;
                format!(
                    "{:<12} [{}{}] {:>5.1}% {done}/{t} frames  {fps:.1} fps",
                    self.label,
                    "█".repeat(filled),
                    "·".repeat(20 - filled.min(20)),
                    pct
                )
            }
            _ => format!("{:<12} {done} frames  {fps:.1} fps", self.label),
        };
        if self.tty {
            eprint!("\r{line}\x1b[K{}", if end { "\n" } else { "" });
        } else {
            eprintln!("{line}");
        }
        let _ = std::io::stderr().flush();
    }

    fn done(&self, done: usize) {
        if self.enabled {
            self.print(done, true);
        }
    }
}

fn output_path(cli: &Cli, input: &Path) -> PathBuf {
    if let Some(o) = &cli.output {
        return o.clone();
    }
    let stem = input.file_stem().and_then(|s| s.to_str()).unwrap_or("output");
    let name = format!("{stem}{}.mp4", cli.suffix);
    match &cli.output_dir {
        Some(d) => d.join(name),
        None => input.with_file_name(name),
    }
}

fn side_path(base: &Path, multi: bool, input: &Path, ext: &str) -> PathBuf {
    if multi {
        let stem = input.file_stem().and_then(|s| s.to_str()).unwrap_or("video");
        base.join(format!("{stem}.{ext}"))
    } else {
        base.to_path_buf()
    }
}

fn load_gyro(
    cli: &Cli,
    cfg: &mut Config,
    tools: &Tools,
    input: &Path,
    probe: &Probe,
    log: &dyn Fn(String),
) -> Result<Option<GyroData>> {
    let needs = matches!(cfg.analysis.motion_source, MotionSource::Gyro | MotionSource::Hybrid);
    let choice = cli.gyro.clone().unwrap_or_else(|| "auto".into());
    let gyro = match choice.as_str() {
        "none" => None,
        "auto" | "gpmf" => {
            if choice == "gpmf" || needs {
                match probe.gpmf_stream {
                    Some(idx) => {
                        let bytes = tools.extract_gpmf(input, idx)?;
                        let g = parse_gpmf(&bytes)?;
                        log(format!("gyro: GoPro GPMF track, {} samples over {:.1}s", g.len(), g.duration()));
                        Some(g)
                    }
                    None if choice == "gpmf" => bail!("{} has no GoPro GPMF metadata track", input.display()),
                    None => None,
                }
            } else {
                None
            }
        }
        path => {
            let p = Path::new(path);
            let bytes = std::fs::read(p).with_context(|| format!("reading gyro file {path}"))?;
            let ext = p.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
            let g = if matches!(ext.as_str(), "bin" | "gpmf" | "gpmd") {
                parse_gpmf(&bytes)?
            } else {
                GyroData::from_csv(&String::from_utf8_lossy(&bytes), cfg.gyro.time_unit, cfg.gyro.angle_unit)?
            };
            log(format!("gyro: {} samples over {:.1}s from {path}", g.len(), g.duration()));
            if cfg.analysis.motion_source == MotionSource::OpticalFlow {
                cfg.analysis.motion_source = MotionSource::Hybrid;
                log("gyro: switching motion source to hybrid (gyro rotation + optical flow)".into());
            }
            Some(g)
        }
    };
    if needs && gyro.is_none() {
        bail!(
            "motion source '{}' needs gyro data: pass --gyro <file.csv> or use a GoPro file with GPMF metadata",
            cfg.analysis.motion_source
        );
    }
    Ok(gyro)
}

fn process(cli: &Cli, base_cfg: &Config, tools: &Tools, input: &Path, idx: usize, total: usize) -> Result<Value> {
    let t0 = Instant::now();
    let multi = total > 1;
    let tag = if multi { format!("[{}/{}] ", idx + 1, total) } else { String::new() };
    let quiet = cli.quiet;
    let log = |m: String| {
        if !quiet {
            eprintln!("{tag}{m}");
        }
    };
    let probe = tools.probe(input)?;
    log(format!(
        "{}: {}x{} @ {:.3} fps, {} frames, {}",
        input.display(),
        probe.width,
        probe.height,
        probe.fps,
        probe.frames_estimate.map(|f| f.to_string()).unwrap_or_else(|| "?".into()),
        probe.codec
    ));
    let mut cfg = base_cfg.clone();
    let gyro = load_gyro(cli, &mut cfg, tools, input, &probe, &log)?;
    let total_frames = probe.frames_estimate.map(|f| cli.max_frames.map_or(f, |m| f.min(m)));

    // ── Stage 1: motion track ─────────────────────────────────────
    let track: MotionTrack = if let Some(p) = &cli.import_track {
        let path = side_path(p, multi, input, "track.json");
        let t = MotionTrack::from_json(
            &std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?,
        )?;
        if t.width != probe.width || t.height != probe.height {
            bail!(
                "track {} is for {}x{}, but the video is {}x{}",
                path.display(),
                t.width,
                t.height,
                probe.width,
                probe.height
            );
        }
        log(format!("loaded motion track ({} frames) from {}", t.len(), path.display()));
        t
    } else if cfg.analysis.motion_source == MotionSource::Gyro && !cfg.gyro.auto_sync {
        let n = tools
            .count_frames(input)
            .ok()
            .or(probe.frames_estimate)
            .ok_or_else(|| anyhow!("could not determine frame count"))?;
        let n = cli.max_frames.map_or(n, |m| n.min(m));
        MotionTrack::empty(probe.width, probe.height, probe.fps, n)
    } else {
        let mut an = Analyzer::new(&cfg, probe.width, probe.height, probe.fps)?;
        let (aw, ah) = an.analysis_size();
        let mut dec = tools.decoder(input, Some((aw, ah)), "gray", (aw * ah) as usize)?;
        let mut buf = Vec::new();
        let mut pr = Progress::new(format!("{tag}analyzing"), total_frames, !quiet);
        let mut n = 0;
        while dec.next_frame(&mut buf)? {
            an.push_luma(&buf, aw, ah)?;
            n += 1;
            pr.tick(n);
            if cli.max_frames.is_some_and(|m| n >= m) {
                break;
            }
        }
        pr.done(n);
        if cli.max_frames.is_none() {
            dec.finish()?;
        }
        if n == 0 {
            bail!("no frames could be decoded from {}", input.display());
        }
        an.finish()
    };
    if let Some(p) = &cli.export_track {
        let path = side_path(p, multi, input, "track.json");
        write_file(&path, &track.to_json())?;
        log(format!("motion track → {}", path.display()));
    }

    // ── Stage 2: plan ─────────────────────────────────────────────
    let plan = Plan::build(&cfg, &track, gyro.as_ref())?;
    let st = &plan.stats;
    if let Some(s) = &st.gyro_sync {
        log(format!(
            "gyro sync: offset {:+.3}s, orientation {}, focal {:.0}px, match {:.0}%",
            s.offset,
            s.orientation,
            s.focal,
            (1.0 - s.score).clamp(0.0, 1.0) * 100.0
        ));
    }
    for note in &st.notes {
        log(format!("warning: {note}"));
    }
    log(format!(
        "shake: {:.2}px / {:.2}° → {:.2}px / {:.2}°  ({:.0}% less), zoom {:.2}× (max {:.2}×){}",
        st.jitter_before.translation_px,
        st.jitter_before.rotation_deg,
        st.jitter_after.translation_px,
        st.jitter_after.rotation_deg,
        st.reduction_pct,
        st.mean_zoom,
        st.max_zoom,
        if st.low_confidence_frames > 0 {
            format!(", {} low-confidence frames", st.low_confidence_frames)
        } else {
            String::new()
        }
    ));
    if let Some(p) = &cli.export_plan {
        let path = side_path(p, multi, input, "plan.json");
        write_file(&path, &plan.to_json())?;
        log(format!("transforms → {}", path.display()));
    }

    // ── Stage 3: render ───────────────────────────────────────────
    let mut out_path = None;
    if !cli.analyze_only {
        let out = output_path(cli, input);
        if out.exists() && !cli.overwrite {
            bail!("{} already exists (use -y to overwrite)", out.display());
        }
        if let Some(parent) = out.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        if std::fs::canonicalize(input).ok() == std::fs::canonicalize(&out).ok() && out.exists() {
            bail!("output would overwrite the input file");
        }
        let (fw, fh) = plan.final_size();
        let enc_opts = EncodeOptions {
            codec: cli.codec.clone(),
            crf: cli.crf,
            preset: cli.encoder_preset.clone(),
            pix_fmt: cli.pix_fmt.clone(),
            audio: cli.audio.clone(),
            extra: cli.ffmpeg_args.clone(),
        };
        let dec = tools.decoder(input, None, "rgba", (probe.width * probe.height * 4) as usize)?;
        let enc = tools.encoder(&enc_opts, input, &out, fw, fh, &probe.fps_str, probe.audio_codec.as_deref())?;
        // Pipeline: decode thread → render (this thread, multi-core warp) → encode thread.
        let limit = cli.max_frames.map(|_| plan.len());
        let (ftx, frx) = std::sync::mpsc::sync_channel::<Vec<u8>>(3);
        let (otx, orx) = std::sync::mpsc::sync_channel::<Vec<u8>>(3);
        let dec_thread = std::thread::spawn(move || -> Result<()> {
            let mut dec = dec;
            let mut sent = 0usize;
            loop {
                let mut b = Vec::new();
                if !dec.next_frame(&mut b)? {
                    break;
                }
                sent += 1;
                if ftx.send(b).is_err() || limit.is_some_and(|l| sent >= l) {
                    return Ok(()); // stopped early; ffmpeg exits on the closed pipe
                }
            }
            dec.finish()
        });
        let enc_thread = std::thread::spawn(move || -> Result<()> {
            let mut enc = enc;
            for f in orx {
                if let Err(e) = enc.write(&f) {
                    return Err(enc.finish().err().unwrap_or(e));
                }
            }
            enc.finish()
        });
        let mut renderer = Renderer::new();
        let mut pr = Progress::new(format!("{tag}rendering"), Some(plan.len()), !quiet);
        let mut n = 0;
        let mut render_err = None;
        for buf in frx {
            match renderer.render(&plan, n, &buf) {
                Ok(frame) => {
                    if otx.send(frame.to_vec()).is_err() {
                        break;
                    }
                }
                Err(e) => {
                    render_err = Some(e);
                    break;
                }
            }
            n += 1;
            pr.tick(n);
        }
        drop(otx);
        pr.done(n);
        let enc_res = enc_thread.join().map_err(|_| anyhow!("encoder thread panicked"))?;
        let dec_res = dec_thread.join().map_err(|_| anyhow!("decoder thread panicked"))?;
        if let Some(e) = render_err {
            return Err(e.into());
        }
        enc_res?;
        dec_res?;
        log(format!("✓ {} ({}x{}) in {:.1}s", out.display(), fw, fh, t0.elapsed().as_secs_f64()));
        out_path = Some(out);
    }

    Ok(json!({
        "input": input.display().to_string(),
        "output": out_path.map(|p| p.display().to_string()),
        "width": probe.width,
        "height": probe.height,
        "fps": probe.fps,
        "frames": plan.len(),
        "seconds": t0.elapsed().as_secs_f64(),
        "stats": serde_json::to_value(&plan.stats)?,
    }))
}

fn write_file(path: &Path, content: &str) -> Result<()> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    std::fs::write(path, content).with_context(|| format!("writing {}", path.display()))
}

fn run(cli: Cli) -> Result<i32> {
    if cli.list_presets {
        for (name, desc) in PRESETS {
            println!("{name:<10} {desc}");
        }
        return Ok(0);
    }
    let cfg = build_config(&cli)?;
    if cli.print_config {
        println!("{}", cfg.to_json_pretty());
        return Ok(0);
    }
    if cli.output.is_some() && cli.inputs.len() > 1 {
        bail!("--output works with a single input; use --output-dir for several");
    }
    if let Some(t) = cli.threads {
        rayon::ThreadPoolBuilder::new().num_threads(t.max(1)).build_global().ok();
    }
    let tools = Tools { ffmpeg: cli.ffmpeg.clone(), ffprobe: cli.ffprobe.clone() };
    tools.check()?;

    let mut results = Vec::new();
    let mut failed = 0;
    for (i, input) in cli.inputs.iter().enumerate() {
        match process(&cli, &cfg, &tools, input, i, cli.inputs.len()) {
            Ok(v) => results.push(v),
            Err(e) => {
                failed += 1;
                eprintln!("error: {}: {e:#}", input.display());
                results.push(json!({ "input": input.display().to_string(), "error": format!("{e:#}") }));
            }
        }
    }
    if let Some(r) = &cli.report {
        let report = json!({
            "tool": "steadyframe",
            "version": steadyframe_core::VERSION,
            "preset": cli.preset,
            "config": serde_json::to_value(&cfg)?,
            "results": results,
        });
        write_file(r, &serde_json::to_string_pretty(&report)?)?;
    }
    Ok(if failed > 0 { 1 } else { 0 })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_parsing() {
        let v = set_to_json("smoothing.radius=42").unwrap();
        assert_eq!(v["smoothing"]["radius"], 42);
        let v = set_to_json("render.border=reflect").unwrap();
        assert_eq!(v["render"]["border"], "reflect");
        assert!(set_to_json("nope").is_err());
    }

    #[test]
    fn cli_overrides_apply() {
        let cli = Cli::parse_from([
            "steadyframe",
            "in.mp4",
            "--preset",
            "walking",
            "--smoothing",
            "l1",
            "--border-color",
            "#102030",
            "--rolling-shutter",
            "0.4",
            "--set",
            "crop.max_zoom=1.25",
            "--horizon-angle",
            "-2",
        ]);
        let c = build_config(&cli).unwrap();
        assert_eq!(c.smoothing.method, SmoothingMethod::L1);
        assert!(c.smoothing.horizon_lock);
        assert_eq!(c.render.border, BorderMode::Color);
        assert_eq!(c.render.border_color, [16, 32, 48, 255]);
        assert!(c.rolling_shutter.enabled);
        assert_eq!(c.crop.max_zoom, 1.25);
        assert_eq!(c.smoothing.horizon_angle, -2.0);
    }
}
