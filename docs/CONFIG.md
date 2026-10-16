# Configuration reference

Every surface (web app, CLI, GitHub Action, Rust API) uses the same JSON configuration. Files can be **partial**: anything you leave out keeps the value from the chosen preset. Settings saved from the web app work unchanged with `steadyframe --config` and the Action's `config` input.

Precedence in the CLI: preset → `--config` file → `--set key=value` → individual flags.

```bash
steadyframe in.mp4 --preset walking --config my.json --set crop.max_zoom=1.25 --horizon-lock
```

The full default config is in [`examples/config.default.json`](../examples/config.default.json).

## analysis

| Key | Default | Meaning |
|---|---|---|
| `analysis_width` | `640` | Frames are downscaled to this width for motion analysis. Lower is faster; higher is more precise on small detail. |
| `motion_model` | `similarity` | `translation`, `similarity` (shift + roll + zoom), `affine` (adds shear/non-uniform scale), `homography` (adds perspective; best for wide-angle wobble, needs texture). |
| `motion_source` | `optical_flow` | `optical_flow`, `gyro`, or `hybrid` (gyro rotation + optical-flow translation/zoom). |
| `max_features` | `400` | Corners tracked per frame. |
| `feature_quality` | `0.01` | Minimum corner strength relative to the strongest corner. |
| `min_distance` | `10` | Minimum spacing between corners (analysis pixels). |
| `lk_radius`, `lk_levels`, `lk_iterations` | `7`, `4`, `20` | Lucas–Kanade window radius, pyramid depth and iterations. Raise `lk_levels` for very fast motion. |
| `fb_threshold` | `1.0` | Forward–backward tracking check (pixels). |
| `ransac_threshold` | `1.5` | Inlier threshold for model fitting (pixels). Raise for noisy/compressed footage. |
| `ransac_iterations` | `400` | Upper bound; RANSAC stops early when confident. |
| `min_inliers` | `12` | Frames with fewer inliers count as "tracking lost" and keep their original motion. |

## smoothing

| Key | Default | Meaning |
|---|---|---|
| `method` | `gaussian` | `gaussian`, `moving_average`, `kalman` (constant-velocity Kalman + RTS smoother; follows motion closely), `l1` (L1-optimal path: static shots, constant-speed pans and smooth ease-in/out). |
| `radius` | `30` | Strength in frames. Rule of thumb: 1 s of footage (= fps) is strong, ¼ s is gentle. |
| `translation`, `rotation`, `zoom` | `1.0` | Per-channel multipliers of `radius`. `0` disables correction of that channel (e.g. keep intentional zooms with `zoom: 0`). |
| `l1_weights` | `[10, 1, 100]` | L1 weights on velocity, acceleration and jerk. |
| `l1_iterations` | `400` | ADMM iterations for the L1 solver. |
| `tripod` | `false` | Lock the view to the reference frame entirely. |
| `reference_frame` | `0` | Frame used by tripod, horizon and zoom locks. |
| `horizon_lock` | `false` | Remove all roll relative to the reference frame. |
| `horizon_angle` | `0` | Extra output rotation (degrees) with `horizon_lock`, to level a horizon that was tilted in the reference frame. |
| `zoom_lock` | `false` | Remove all scale/perspective changes. |
| `limit_shift` | `0.15` | Max correction shift as a fraction of the frame (`0` = unlimited). Bigger limits allow stronger stabilization but need more zoom. |
| `limit_rotation` | `8` | Max correction rotation in degrees (`0` = unlimited). |
| `limit_scale` | `0.15` | Max correction zoom change as a fraction (`0` = unlimited). |

## crop

| Key | Default | Meaning |
|---|---|---|
| `mode` | `fixed` | `fixed`: one zoom for the whole clip; `dynamic`: zoom follows how much correction is needed, smoothly; `none`: no zoom, exposed borders are filled. |
| `max_zoom` | `1.4` | Never zoom in further; borders can show beyond this. |
| `extra_zoom` | `1.0` | Additional constant zoom. |
| `dynamic_smoothing` | `45` | Smoothing of the zoom curve in `dynamic` mode (frames). |

## render

| Key | Default | Meaning |
|---|---|---|
| `interpolation` | `bilinear` | `nearest`, `bilinear`, `bicubic` (sharper, ~4× slower). |
| `border` | `black` | `black`, `color`, `replicate` (stretch edge), `reflect` (mirror), `previous` (keep the previous output frame's pixels). |
| `border_color` | `[0,0,0,255]` | RGBA used by `border: "color"`. |
| `output_width`, `output_height` | `0` | Output size (`0` = same as input). A different aspect ratio crops to cover, e.g. 1080×1920 for vertical video from landscape footage. Must be even. |
| `compare` | `none` | `side_by_side` or `split` renders a before/after video. |

## rolling_shutter

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `false` | Correct CMOS "jello"/skew per row. |
| `readout` | `0.5` | Sensor readout time as a fraction of the frame interval (phones ≈ 0.3–0.8, action cams ≈ 0.4–0.9). Negative for sensors that read bottom-to-top. |

## gyro

| Key | Default | Meaning |
|---|---|---|
| `orientation` | `XYZ` | Gyro→camera axis mapping. Camera axes: x right, y down, z forward. Each letter names the gyro axis for that camera axis; lowercase negates. |
| `time_offset` | `0` | Seconds added to gyro timestamps. |
| `hfov` | `75` | Horizontal field of view in degrees (used when `focal_px` is 0). |
| `focal_px` | `0` | Focal length in pixels. |
| `auto_sync` | `true` | Match gyro rotation to optical flow to find offset, orientation and focal length. |
| `sync_range` | `1.0` | Search ±seconds for the offset. |
| `sync_orientation` | `true` | Try all 48 axis orientations. |
| `sync_focal` | `true` | Estimate focal length. |
| `time_unit` | `auto` | CSV time unit: `auto`, `s`, `ms`, `us`, `ns`. |
| `angle_unit` | `auto` | CSV angular-velocity unit: `auto`, `rad`, `deg` (per second). |

### Gyro CSV format

Any delimiter (comma, semicolon, tab, space). A header row is optional; recognised column names include `t`/`time`/`timestamp` and `gx`/`gyro_x`/`wx`/`rotationRate x`. Headerless files are read as `time, x, y, z`. Gyroflow's `GYROFLOW IMU LOG` format with `tscale`/`gscale` lines is supported.

```csv
time,gyro_x,gyro_y,gyro_z
0.000,0.012,-0.034,0.002
0.005,0.015,-0.031,0.001
```
