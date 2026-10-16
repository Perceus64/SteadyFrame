# Steadyframe

Video stabilization written in Rust that runs in three places from one engine:

- **In the browser** (WebAssembly): drop a clip, tune settings with a live before/after preview, export MP4. Nothing is uploaded.
- **On the command line**: a fast native CLI that pipes frames through FFmpeg and keeps your audio.
- **In GitHub Actions**: stabilize every video pushed to a repository, with a shake report in the job summary.

## Features

**Motion analysis**
- Shi–Tomasi corners spread across the frame, pyramidal Lucas–Kanade optical flow with forward–backward checks
- Robust RANSAC fitting of translation, similarity, affine or full homography models
- Per-frame confidence; frames with lost tracking are handled gracefully

**Camera-path smoothing**
- Gaussian, moving average, Kalman/RTS, and **L1-optimal** paths (static shots + clean pans, as in Grundmann et al. 2011)
- Separate strength for position, rotation and zoom
- Virtual **tripod**, **horizon lock** (with level adjustment) and **zoom lock**
- Correction limits so the stabilizer never fights intentional motion

**Rendering**
- Fixed or **dynamic** auto-zoom to hide borders, with a max-zoom cap
- Frames where tracking is lost (heavy blur, flashes) are bridged by interpolating their neighbours
- Border fill: black, color, stretch, mirror, or previous frame
- Nearest, bilinear or bicubic interpolation
- **Rolling-shutter (jello) correction** per row
- Any output size or aspect ratio (e.g. vertical 1080×1920 from landscape)
- Before/after **side-by-side** or **split** comparison renders

**Gyro stabilization**
- GoPro **GPMF** metadata read automatically from the MP4
- Gyro CSV logs (phones, Gyroflow format, custom loggers)
- **Auto-sync**: finds time offset, axis orientation and lens focal length by matching gyro rotation to optical flow
- Hybrid mode: gyro rotation (robust to motion blur) + optical-flow translation

**Workflow**
- 9 presets, JSON configs shared by all surfaces, `--set key=value` overrides
- Save the motion track once, re-render with different settings without re-analysing
- Export per-frame transforms as JSON
- Shake metrics (before/after jitter, reduction %, zoom) in every report

## Quick start

### Browser

Open the hosted app (GitHub Pages, deployed by `.github/workflows/pages.yml`), or run it locally:

```bash
cargo install wasm-pack          # once
rustup target add wasm32-unknown-unknown
wasm-pack build crates/steadyframe-wasm --release --target web --out-dir ../../web/pkg
python3 -m http.server -d web 8080   # then open http://localhost:8080
```

Browser exports use WebCodecs (H.264 in Chrome/Edge/Safari; VP9/AV1 in MP4 where H.264 encoding isn't available) and don't include audio. Use the CLI or Action to keep audio.

### CLI

Requires FFmpeg on your PATH.

```bash
cargo install --path crates/steadyframe-cli     # or download a release binary

steadyframe shaky.mp4                           # → shaky_stabilized.mp4
steadyframe in.mp4 -o out.mp4 --preset walking
steadyframe *.mp4 --output-dir stabilized --preset cinematic
steadyframe in.mp4 --tripod --compare side-by-side
steadyframe GX010042.MP4 --preset gyro          # GoPro: embedded gyro, auto-synced
steadyframe phone.mp4 --gyro gyro.csv --rolling-shutter 0.6
steadyframe in.mp4 --width 1080 --height 1920 --crop dynamic   # vertical cut
steadyframe in.mp4 --analyze-only --export-track t.json --report r.json
steadyframe in.mp4 --import-track t.json --preset smooth       # re-render only
steadyframe --help
```

### GitHub Action

```yaml
permissions:
  contents: write
steps:
  - uses: actions/checkout@v4
  - uses: Perceus64/steadyframe@v0.1.0
    with:
      files: "media/**/*.mp4"
      preset: walking
      args: --crop dynamic --crf 20
      commit: true            # push stabilized videos back
```

| Input | Default | Description |
|---|---|---|
| `files` | `**/*.mp4 **/*.MP4 **/*.mov **/*.MOV` | Glob pattern(s) |
| `output-dir` | `stabilized` | Output folder (input folder structure is kept); empty = next to inputs |
| `suffix` | `_stabilized` | Output name suffix |
| `preset` | `handheld` | Starting preset |
| `config` | | JSON config file (e.g. saved from the web app) |
| `args` | | Any extra CLI flags |
| `compare` | `none` | Also render `side-by-side` or `split` comparisons |
| `skip-existing` | `true` | Don't reprocess videos that already have an output |
| `fail-on-error` | `true` | Fail the step if any video fails |
| `upload-artifact` / `artifact-name` | `true` / `stabilized-videos` | Upload results and report |
| `commit` / `commit-message` | `false` | Commit outputs back to the repo |
| `version` | action ref | Release tag, `latest`, or `source` to build from source |

Outputs: `files`, `count`, `failed`, `report`. A full workflow is in [`examples/stabilize-videos.yml`](examples/stabilize-videos.yml). The Action runs on Linux and macOS runners (Windows works when FFmpeg installs via Chocolatey).

## Presets

| Preset | Use for |
|---|---|
| `handheld` | Balanced default |
| `smooth` | Strong smoothing with dynamic zoom |
| `walking` | Vlogs and walking shots; horizon lock |
| `vehicle` | Dashcams, drones; Kalman + rolling shutter |
| `action` | Action cams; homography, rolling shutter, bigger crop budget |
| `cinematic` | L1 path: locked-off shots and clean pans |
| `tripod` | Freeze the frame completely |
| `gentle` | Light touch that keeps intentional motion |
| `gyro` | Gyro-driven (GoPro or CSV), auto-synced |

All settings: [docs/CONFIG.md](docs/CONFIG.md).

## How it works

```
frames ──► grayscale @ analysis width ──► corners ──► pyramidal LK flow ──► RANSAC model
                                                                             │ per-frame motion Mₜ
gyro (GPMF/CSV) ──► integrate ──► auto-sync (offset, axes, focal) ─────────┤ (optional)
                                                                             ▼
                 camera path Cₜ = Mₜ·Cₜ₋₁  ──►  smoothing / locks / limits  ──►  Sₜ
                                                                             ▼
             warp Wₜ = Sₜ·Cₜ⁻¹ · zoom  ──►  per-row rolling-shutter fix  ──►  output frame
```

The camera path is decomposed into independent channels (x, y, roll, log-scale, shear, perspective) so each can be smoothed, limited or locked on its own. Analysis is the slow part; the resulting motion track is small and reusable, which is why the web app can re-plan instantly while you move sliders.

## Project layout

```
crates/steadyframe-core   engine (pure Rust, no native deps; optional rayon parallelism)
crates/steadyframe-cli    `steadyframe` binary (FFmpeg pipeline)
crates/steadyframe-wasm   wasm-bindgen API used by the web app
web/                      browser app (worker-based, vendored mp4-muxer)
action.yml, action/       composite GitHub Action
.github/workflows/        CI, releases (Linux/macOS/Windows binaries + web bundle), Pages
```

## Development

```bash
cargo test --workspace          # unit + synthetic end-to-end tests (known ground-truth motion)
cargo clippy --workspace --all-targets -- -D warnings
cargo run --release -p steadyframe-core --example bench_warp
```

To publish a release: push a tag like `v0.1.0`. The release workflow builds binaries for five targets and the web bundle; the Action downloads the matching binary automatically when used at that tag.

## Limitations

- Stabilization is 2D (projective). Strong parallax (close foreground passing a distant background) can't be fully removed by any 2D method.
- Browser analysis extracts frames by seeking, so very long clips analyse slower in the browser than in the CLI.
- Variable-frame-rate video is treated as constant frame rate.
- Output is 8-bit SDR (BT.709). HDR/10-bit sources are converted, so HDR footage will look flatter than the original.
- Audio is copied when the output container supports it; otherwise (e.g. PCM from camera `.MOV` files into MP4) it is re-encoded to AAC 192 kb/s.
- GoPro GPMF timing uses payload timestamps; very old firmware without them falls back to one payload per second, which auto-sync then corrects.

## License

MIT. Bundles [mp4-muxer](https://github.com/Vanilagy/mp4-muxer) (MIT) in `web/vendor`.
