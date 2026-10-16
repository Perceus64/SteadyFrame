// Steadyframe browser app: frame extraction, settings UI, preview and export.
// All stabilization math runs in Rust/WASM inside worker.js.

const $ = (id) => document.getElementById(id);
const video = $("video");
const view = $("view");
const vctx = view.getContext("2d");
const graph = $("graph");

// ── Worker RPC ───────────────────────────────────────────────────────────
const worker = new Worker(new URL("./worker.js", import.meta.url), { type: "module" });
let nextId = 1;
const pending = new Map();
worker.onmessage = (e) => {
  const { id, ok, result, error } = e.data;
  const p = pending.get(id);
  if (!p) return;
  pending.delete(id);
  ok ? p.resolve(result) : p.reject(new Error(error));
};
function rpc(type, payload = {}, transfer = []) {
  return new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject });
    worker.postMessage({ id, type, ...payload }, transfer);
  });
}

// ── State ────────────────────────────────────────────────────────────────
const S = {
  presets: [],
  defaults: null,
  cfg: null,
  analysedWith: null, // JSON of analysis settings used for the current track
  file: null,
  w: 0, h: 0, fps: 30, frames: 0,
  analysed: false,
  plan: null, // { stats, paths, ow, oh }
  frame: 0,
  mode: "split",
  split: 0.5,
  playing: false,
  busy: false,
  cancel: false,
  hasGyro: false,
  cache: new Map(), // frame index → { orig: ImageBitmap|ImageData, stab: ImageData }
};

// ── Helpers ──────────────────────────────────────────────────────────────
let toastTimer;
function toast(msg, isError = false) {
  const t = $("toast");
  t.textContent = msg;
  t.className = "toast" + (isError ? " error" : "");
  t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (t.hidden = true), isError ? 8000 : 3500);
}
const getPath = (o, p) => p.split(".").reduce((a, k) => (a == null ? a : a[k]), o);
function setPath(o, p, v) {
  const ks = p.split(".");
  let t = o;
  for (const k of ks.slice(0, -1)) t = t[k] ??= {};
  t[ks.at(-1)] = v;
}
const clone = (o) => JSON.parse(JSON.stringify(o));
function download(name, data, type = "application/json") {
  const url = URL.createObjectURL(data instanceof Blob ? data : new Blob([data], { type }));
  const a = Object.assign(document.createElement("a"), { href: url, download: name });
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
}
const baseName = () => (S.file?.name || "video").replace(/\.[^.]+$/, "");
const analysisKey = (cfg) => JSON.stringify({ ...cfg.analysis, motion_source: undefined });

function setBusy(text, frac) {
  $("busy").hidden = text == null;
  if (text != null) {
    $("busyText").textContent = text;
    $("busyBar").style.width = `${Math.round((frac || 0) * 100)}%`;
  }
}

// ── Frame access via the <video> element ────────────────────────────────
const grabCanvas = new OffscreenCanvas(16, 16);
const grabCtx = grabCanvas.getContext("2d", { willReadFrequently: true });

function seek(index) {
  const t = Math.min((index + 0.5) / S.fps, Math.max(0, video.duration - 1e-3));
  return new Promise((resolve) => {
    let done = false;
    const finish = () => {
      if (done) return;
      done = true;
      resolve();
    };
    const onSeeked = () => {
      video.removeEventListener("seeked", onSeeked);
      if ("requestVideoFrameCallback" in video) {
        video.requestVideoFrameCallback(() => finish());
        setTimeout(finish, 120); // same frame twice: no new frame callback
      } else finish();
    };
    if (Math.abs(video.currentTime - t) < 1e-4) return finish();
    video.addEventListener("seeked", onSeeked);
    video.currentTime = t;
  });
}

function grab(w, h) {
  if (grabCanvas.width !== w || grabCanvas.height !== h) {
    grabCanvas.width = w;
    grabCanvas.height = h;
  }
  grabCtx.drawImage(video, 0, 0, w, h);
  return grabCtx.getImageData(0, 0, w, h);
}

async function detectFps() {
  if (!("requestVideoFrameCallback" in video)) return 30;
  const times = [];
  await new Promise((resolve) => {
    const cb = (_now, meta) => {
      times.push(meta.mediaTime);
      if (times.length < 16) video.requestVideoFrameCallback(cb);
      else resolve();
    };
    video.requestVideoFrameCallback(cb);
    video.play().catch(resolve);
    setTimeout(resolve, 2500);
  });
  video.pause();
  const d = [];
  for (let i = 1; i < times.length; i++) if (times[i] > times[i - 1]) d.push(times[i] - times[i - 1]);
  if (d.length < 3) return 30;
  d.sort((a, b) => a - b);
  const fps = 1 / d[Math.floor(d.length / 4)]; // lower quartile: robust to dropped frames
  const common = [23.976, 24, 25, 29.97, 30, 48, 50, 59.94, 60, 100, 119.88, 120, 240];
  const best = common.reduce((a, c) => (Math.abs(c - fps) < Math.abs(a - fps) ? c : a));
  return Math.abs(best - fps) / best < 0.03 ? best : Math.round(fps * 100) / 100;
}

// ── Settings UI ──────────────────────────────────────────────────────────
const L = (o) => o; // readability
const SECTIONS = [
  {
    title: "Smoothing", open: true, fields: [
      L({ path: "smoothing.method", type: "select", label: "Method", options: [["gaussian", "Gaussian"], ["moving_average", "Moving average"], ["kalman", "Kalman (follows motion)"], ["l1", "L1 cinematic (static + clean pans)"]] }),
      L({ path: "smoothing.radius", type: "range", label: "Strength", min: 0, max: 150, step: 1, fmt: (v) => `${v} frames` }),
      L({ path: "smoothing.translation", type: "range", label: "Position smoothing", min: 0, max: 3, step: 0.05, fmt: (v) => `${v}×` }),
      L({ path: "smoothing.rotation", type: "range", label: "Rotation smoothing", min: 0, max: 3, step: 0.05, fmt: (v) => `${v}×` }),
      L({ path: "smoothing.zoom", type: "range", label: "Zoom smoothing", min: 0, max: 3, step: 0.05, fmt: (v) => `${v}×` }),
      L({ path: "smoothing.tripod", type: "toggle", label: "Tripod lock" }),
      L({ path: "smoothing.horizon_lock", type: "toggle", label: "Horizon lock" }),
      L({ path: "smoothing.horizon_angle", type: "range", label: "Horizon angle", min: -20, max: 20, step: 0.1, fmt: (v) => `${v}°` }),
      L({ path: "smoothing.zoom_lock", type: "toggle", label: "Zoom lock" }),
      L({ path: "smoothing.reference_frame", type: "number", label: "Lock reference frame", min: 0, step: 1 }),
      L({ path: "smoothing.limit_shift", type: "range", label: "Max shift", min: 0, max: 0.5, step: 0.01, fmt: (v) => (v > 0 ? `${Math.round(v * 100)}% of frame` : "unlimited") }),
      L({ path: "smoothing.limit_rotation", type: "range", label: "Max rotation", min: 0, max: 30, step: 0.5, fmt: (v) => (v > 0 ? `${v}°` : "unlimited") }),
      L({ path: "smoothing.limit_scale", type: "range", label: "Max zoom change", min: 0, max: 0.5, step: 0.01, fmt: (v) => (v > 0 ? `${Math.round(v * 100)}%` : "unlimited") }),
    ],
  },
  {
    title: "Crop & zoom", fields: [
      L({ path: "crop.mode", type: "select", label: "Hide borders", options: [["fixed", "Fixed zoom"], ["dynamic", "Dynamic zoom"], ["none", "Don't zoom"]] }),
      L({ path: "crop.max_zoom", type: "range", label: "Max zoom", min: 1, max: 2.5, step: 0.01, fmt: (v) => `${v}×` }),
      L({ path: "crop.extra_zoom", type: "range", label: "Extra zoom", min: 0.8, max: 1.5, step: 0.01, fmt: (v) => `${v}×` }),
      L({ path: "crop.dynamic_smoothing", type: "range", label: "Dynamic zoom smoothing", min: 5, max: 150, step: 1, fmt: (v) => `${v} frames` }),
    ],
  },
  {
    title: "Rendering", fields: [
      L({ path: "render.interpolation", type: "select", label: "Interpolation", options: [["bilinear", "Bilinear"], ["bicubic", "Bicubic (sharper)"], ["nearest", "Nearest"]] }),
      L({ path: "render.border", type: "select", label: "Border fill", options: [["black", "Black"], ["color", "Color"], ["replicate", "Stretch edge"], ["reflect", "Mirror"], ["previous", "Previous frame"]] }),
      L({ path: "render.border_color", type: "color", label: "Border color" }),
      L({ path: "render.output_width", type: "number", label: "Output width (0 = same)", min: 0, step: 2 }),
      L({ path: "render.output_height", type: "number", label: "Output height (0 = same)", min: 0, step: 2 }),
    ],
  },
  {
    title: "Rolling shutter", fields: [
      L({ path: "rolling_shutter.enabled", type: "toggle", label: "Correct jello / skew" }),
      L({ path: "rolling_shutter.readout", type: "range", label: "Sensor readout", min: -1, max: 1, step: 0.01, fmt: (v) => `${Math.round(v * 100)}% of frame` }),
    ],
  },
  {
    title: "Motion analysis", fields: [
      L({ path: "analysis.motion_model", type: "select", label: "Motion model", reanalyze: true, options: [["similarity", "Similarity (shift, roll, zoom)"], ["translation", "Translation only"], ["affine", "Affine"], ["homography", "Homography (perspective)"]] }),
      L({ path: "analysis.analysis_width", type: "select", label: "Analysis resolution", reanalyze: true, num: true, options: [[320, "320 px (fastest)"], [480, "480 px"], [640, "640 px"], [960, "960 px"], [1280, "1280 px (most precise)"]] }),
      L({ path: "analysis.max_features", type: "range", label: "Tracked points", reanalyze: true, min: 50, max: 1000, step: 10 }),
      L({ path: "analysis.min_distance", type: "range", label: "Point spacing", reanalyze: true, min: 3, max: 40, step: 1, fmt: (v) => `${v} px` }),
      L({ path: "analysis.ransac_threshold", type: "range", label: "Outlier threshold", reanalyze: true, min: 0.5, max: 6, step: 0.1, fmt: (v) => `${v} px` }),
    ],
  },
];
const GYRO_FIELDS = [
  L({ path: "analysis.motion_source", type: "select", label: "Motion from", options: [["optical_flow", "Video only"], ["hybrid", "Gyro rotation + video"], ["gyro", "Gyro only"]] }),
  L({ path: "gyro.auto_sync", type: "toggle", label: "Auto-sync to video" }),
  L({ path: "gyro.orientation", type: "text", label: "Axis orientation (e.g. XYZ, yXz)" }),
  L({ path: "gyro.time_offset", type: "number", label: "Time offset (s)", step: 0.001 }),
  L({ path: "gyro.hfov", type: "range", label: "Lens field of view", min: 20, max: 170, step: 1, fmt: (v) => `${v}°` }),
  L({ path: "gyro.sync_range", type: "range", label: "Sync search range", min: 0.1, max: 5, step: 0.1, fmt: (v) => `±${v} s` }),
];

const controls = [];

function hexOf(rgba) {
  return "#" + rgba.slice(0, 3).map((v) => v.toString(16).padStart(2, "0")).join("");
}

function buildField(f, host) {
  const wrap = document.createElement("label");
  wrap.className = "field" + (f.type === "toggle" ? " toggle" : "");
  const head = document.createElement("span");
  head.textContent = f.label;
  const out = document.createElement("output");
  let input;
  if (f.type === "select") {
    input = document.createElement("select");
    for (const [v, t] of f.options) input.add(new Option(t, v));
  } else {
    input = document.createElement("input");
    input.type = { range: "range", toggle: "checkbox", number: "number", color: "color", text: "text" }[f.type];
    for (const k of ["min", "max", "step"]) if (f[k] != null) input[k] = f[k];
  }
  if (f.type === "range") head.append(out);
  if (f.reanalyze) {
    const r = document.createElement("span");
    r.className = "reanalyze";
    r.hidden = true;
    r.textContent = "re-analyze to apply";
    wrap.append(head, r, input);
    f.badge = r;
  } else wrap.append(head, input);
  host.append(wrap);

  const read = () => {
    switch (f.type) {
      case "toggle": return input.checked;
      case "range": case "number": return Number(input.value);
      case "color": {
        const h = input.value;
        return [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16)).concat(255);
      }
      case "select": return f.num ? Number(input.value) : input.value;
      default: return input.value;
    }
  };
  const write = (v) => {
    if (f.type === "toggle") input.checked = !!v;
    else if (f.type === "color") input.value = hexOf(v || [0, 0, 0, 255]);
    else input.value = v;
    if (f.type === "range") out.textContent = f.fmt ? f.fmt(Number(v)) : v;
  };
  input.addEventListener("input", () => {
    if (f.type === "range") out.textContent = f.fmt ? f.fmt(read()) : read();
    if (f.type === "text" || f.type === "number") return; // apply on change
    onSetting(f, read());
  });
  input.addEventListener("change", () => {
    if (f.type === "text" || f.type === "number") onSetting(f, read());
  });
  controls.push({ f, write, wrap });
}

function renderControls() {
  for (const c of controls) {
    c.write(getPath(S.cfg, c.f.path));
    const def = S.defaults && getPath(S.defaults, c.f.path);
    c.wrap.classList.toggle("changed", JSON.stringify(def) !== JSON.stringify(getPath(S.cfg, c.f.path)));
  }
  updateReanalyzeBadges();
}

function updateReanalyzeBadges() {
  const stale = S.analysed && S.analysedWith && S.analysedWith !== analysisKey(S.cfg);
  for (const c of controls) if (c.f.badge) c.f.badge.hidden = !stale;
}

function buildSettings() {
  const host = $("sections");
  for (const sec of SECTIONS) {
    const d = document.createElement("details");
    d.className = "section";
    d.open = !!sec.open;
    const s = document.createElement("summary");
    s.textContent = sec.title;
    const fields = document.createElement("div");
    fields.className = "fields";
    d.append(s, fields);
    host.append(d);
    for (const f of sec.fields) buildField(f, fields);
  }
  for (const f of GYRO_FIELDS) buildField(f, $("gyroFields"));
}

let planTimer;
function onSetting(f, value) {
  setPath(S.cfg, f.path, value);
  $("preset").value = "";
  $("presetDesc").textContent = "Custom settings";
  renderControls();
  if (f.reanalyze) return;
  clearTimeout(planTimer);
  planTimer = setTimeout(replan, 120);
}

async function applyPreset(name) {
  S.cfg = await rpc("preset", { name });
  const p = S.presets.find((p) => p[0] === name);
  $("presetDesc").textContent = p ? p[1] : "";
  renderControls();
  if (S.analysed && S.analysedWith !== analysisKey(S.cfg)) {
    toast("This preset uses different analysis settings — re-analyzing.");
    await analyze();
  } else replan();
}

// ── Pipeline ─────────────────────────────────────────────────────────────
async function loadVideo(file) {
  if (S.busy) return toast("Wait for the current task to finish.", true);
  S.file = file;
  S.analysed = false;
  S.plan = null;
  S.cache.clear();
  if (video.src) URL.revokeObjectURL(video.src);
  video.src = URL.createObjectURL(file);
  try {
    await new Promise((res, rej) => {
      video.onloadeddata = res;
      video.onerror = () => rej(new Error("This browser can't decode that video. Try an H.264 MP4."));
    });
  } catch (e) {
    return toast(e.message, true);
  }
  S.w = video.videoWidth;
  S.h = video.videoHeight;
  $("drop").hidden = true;
  $("workspace").hidden = false;
  setBusy("Measuring frame rate…", 0);
  S.fps = await detectFps();
  S.frames = Math.max(1, Math.floor(video.duration * S.fps + 1e-6));
  $("meta").textContent = `${file.name}, ${S.w}×${S.h}, ${S.fps} fps, ${S.frames} frames`;
  $("scrub").max = S.frames - 1;
  sizeCanvas(S.w, S.h);
  try {
    await rpc("init", { width: S.w, height: S.h, fps: S.fps, config: JSON.stringify(S.cfg) });
  } catch (e) {
    setBusy(null);
    return toast(e.message, true);
  }
  S.hasGyro = false;
  updateGyroUi();
  await analyze();
}

async function analyze() {
  if (S.busy) return;
  S.busy = true;
  S.cancel = false;
  enableUi(false);
  try {
    await rpc("config", { config: JSON.stringify(S.cfg) });
    const { aw, ah } = await rpc("reset");
    const t0 = performance.now();
    for (let i = 0; i < S.frames; i++) {
      if (S.cancel) throw new Error("Analysis cancelled.");
      await seek(i);
      const img = grab(aw, ah);
      await rpc("push", { buf: img.data.buffer, w: aw, h: ah }, [img.data.buffer]);
      if (i % 3 === 0) {
        const rate = (i + 1) / ((performance.now() - t0) / 1000);
        const left = Math.max(0, Math.round((S.frames - i) / rate));
        setBusy(`Analyzing motion… ${i + 1} / ${S.frames} (about ${left}s left)`, (i + 1) / S.frames);
      }
    }
    await rpc("finish");
    S.analysed = true;
    S.analysedWith = analysisKey(S.cfg);
  } catch (e) {
    toast(e.message, true);
  } finally {
    S.busy = false;
    setBusy(null);
  }
  if (S.analysed) await replan();
  enableUi(S.analysed);
}

function enableUi(on) {
  for (const id of ["export", "play", "scrub", "saveTrack", "savePlan"]) $(id).disabled = !on;
}

async function replan() {
  if (!S.analysed || S.busy) return;
  try {
    await rpc("config", { config: JSON.stringify(S.cfg) });
    S.plan = await rpc("plan");
    S.cache.clear();
    showStats();
    drawGraph();
    await showFrame(S.frame);
  } catch (e) {
    toast(e.message, true);
  }
}

function showStats() {
  const st = S.plan.stats;
  const jb = st.jitter_before, ja = st.jitter_after;
  const parts = [
    [`${Math.max(0, st.reduction_pct).toFixed(0)}%`, "less shake"],
    [`${jb.translation_px.toFixed(1)} → ${ja.translation_px.toFixed(1)} px`, "positional jitter"],
    [`${jb.rotation_deg.toFixed(2)} → ${ja.rotation_deg.toFixed(2)}°`, "rotational jitter"],
    [`${st.mean_zoom.toFixed(2)}×`, st.max_zoom > st.mean_zoom + 0.005 ? `average zoom, up to ${st.max_zoom.toFixed(2)}×` : "zoom to hide borders"],
  ];
  if (st.gyro_sync) {
    const g = st.gyro_sync;
    parts.push([`${(g.offset * 1000).toFixed(0)} ms`, `gyro offset, ${g.orientation}, ${Math.round(Math.max(0, 1 - g.score) * 100)}% match`]);
  }
  const box = $("stats");
  box.replaceChildren(
    ...parts.map(([a, b]) => {
      const d = document.createElement("div");
      const s = document.createElement("strong");
      s.textContent = a;
      d.append(s, b);
      return d;
    }),
  );
  const notes = [...st.notes];
  if (st.low_confidence_frames > 0) notes.push(`${st.low_confidence_frames} frames had too little texture to track; they keep their original motion.`);
  for (const n of notes) {
    const p = document.createElement("p");
    p.className = "note";
    p.textContent = n;
    box.append(p);
  }
}

// ── Preview ──────────────────────────────────────────────────────────────
function sizeCanvas(w, h) {
  view.width = w;
  view.height = h;
  $("viewer").style.aspectRatio = `${w} / ${h}`;
}

async function getFrame(index) {
  let c = S.cache.get(index);
  if (c) return c;
  await seek(index);
  const full = grab(S.w, S.h);
  const orig = await createImageBitmap(full);
  const r = await rpc("render", { index, buf: full.data.buffer }, [full.data.buffer]);
  const stab = new ImageData(new Uint8ClampedArray(r.buf), r.w, r.h);
  c = { orig, stab };
  S.cache.set(index, c);
  if (S.cache.size > 24) {
    const k = S.cache.keys().next().value;
    S.cache.get(k).orig.close?.();
    S.cache.delete(k);
  }
  return c;
}

const stabCanvas = new OffscreenCanvas(16, 16);
const stabCtx = stabCanvas.getContext("2d");

async function showFrame(index) {
  S.frame = index;
  $("scrub").value = index;
  $("frameNo").textContent = `${index + 1} / ${S.frames}`;
  if (!S.plan) return;
  const { orig, stab } = await getFrame(index);
  if (S.frame !== index) return;
  if (stabCanvas.width !== stab.width || stabCanvas.height !== stab.height) {
    stabCanvas.width = stab.width;
    stabCanvas.height = stab.height;
  }
  stabCtx.putImageData(stab, 0, 0);
  const sw = stab.width, sh = stab.height;
  const mode = S.mode;
  $("divider").hidden = mode !== "split";
  if (mode === "side") {
    sizeCanvas(sw * 2, sh);
    vctx.drawImage(orig, 0, 0, sw, sh);
    vctx.drawImage(stabCanvas, sw, 0);
  } else {
    sizeCanvas(sw, sh);
    if (mode === "orig") vctx.drawImage(orig, 0, 0, sw, sh);
    else vctx.drawImage(stabCanvas, 0, 0);
    if (mode === "split") {
      const x = Math.round(S.split * sw);
      vctx.save();
      vctx.beginPath();
      vctx.rect(0, 0, x, sh);
      vctx.clip();
      vctx.drawImage(orig, 0, 0, sw, sh);
      vctx.restore();
      positionDivider();
    }
  }
  drawGraph();
}

function positionDivider() {
  const r = view.getBoundingClientRect();
  const pr = $("viewer").getBoundingClientRect();
  $("divider").style.left = `${r.left - pr.left + S.split * r.width}px`;
}

async function play() {
  if (S.playing) {
    S.playing = false;
    return;
  }
  S.playing = true;
  $("playIcon").setAttribute("d", "M6 5h4v14H6zM14 5h4v14h-4z");
  $("play").setAttribute("aria-label", "Pause preview");
  let i = S.frame >= S.frames - 1 ? 0 : S.frame;
  while (S.playing && i < S.frames) {
    const t = performance.now();
    await showFrame(i);
    const wait = 1000 / S.fps - (performance.now() - t);
    if (wait > 0) await new Promise((r) => setTimeout(r, wait));
    i++;
  }
  S.playing = false;
  $("playIcon").setAttribute("d", "M7 5l12 7-12 7z");
  $("play").setAttribute("aria-label", "Play preview");
}

// ── Path graph ───────────────────────────────────────────────────────────
function drawGraph() {
  const dpr = window.devicePixelRatio || 1;
  const cw = graph.clientWidth, ch = graph.clientHeight;
  if (graph.width !== Math.round(cw * dpr)) {
    graph.width = Math.round(cw * dpr);
    graph.height = Math.round(ch * dpr);
  }
  const g = graph.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, cw, ch);
  if (!S.plan) return;
  const css = getComputedStyle(document.documentElement);
  const col = (n) => css.getPropertyValue(n).trim();
  const ch_ = Number($("channel").value);
  const raw = S.plan.paths.raw.map((p) => p[ch_]);
  const sm = S.plan.paths.smooth.map((p) => p[ch_]);
  const all = raw.concat(sm);
  let lo = Math.min(...all), hi = Math.max(...all);
  if (hi - lo < 1e-6) { lo -= 1; hi += 1; }
  const pad = 10;
  const X = (i) => pad + (i / Math.max(1, raw.length - 1)) * (cw - 2 * pad);
  const Y = (v) => ch - pad - ((v - lo) / (hi - lo)) * (ch - 2 * pad);
  g.lineWidth = 1;
  g.strokeStyle = col("--line");
  g.beginPath();
  for (let k = 0; k <= 4; k++) {
    const y = pad + (k / 4) * (ch - 2 * pad);
    g.moveTo(pad, y);
    g.lineTo(cw - pad, y);
  }
  g.stroke();
  const line = (arr, color, width) => {
    g.strokeStyle = color;
    g.lineWidth = width;
    g.lineJoin = "round";
    g.beginPath();
    arr.forEach((v, i) => (i ? g.lineTo(X(i), Y(v)) : g.moveTo(X(i), Y(v))));
    g.stroke();
  };
  line(raw, col("--raw"), 1.25);
  line(sm, col("--smooth"), 2.5);
  const x = X(S.frame);
  g.strokeStyle = col("--level");
  g.lineWidth = 2;
  g.beginPath();
  g.moveTo(x, 4);
  g.lineTo(x, ch - 4);
  g.stroke();
  g.fillStyle = col("--muted");
  g.font = "12px Manrope, sans-serif";
  const unit = ch_ === 2 ? "°" : " px";
  g.fillText(`${hi.toFixed(ch_ === 2 ? 2 : 0)}${unit}`, pad + 2, pad + 10);
  g.fillText(`${lo.toFixed(ch_ === 2 ? 2 : 0)}${unit}`, pad + 2, ch - pad - 3);
}

// ── Export ───────────────────────────────────────────────────────────────
async function exportVideo() {
  if (S.busy || !S.analysed) return;
  S.playing = false;
  S.busy = true;
  S.cancel = false;
  enableUi(false);
  const compare = $("exportCompare").checked;
  const exportCfg = clone(S.cfg);
  exportCfg.render.compare = compare ? "side_by_side" : "none";
  try {
    await rpc("config", { config: JSON.stringify(exportCfg) });
    const plan = await rpc("plan");
    const { ow, oh } = plan;
    const bitrate = Number($("bitrate").value) * 1e6;
    const blob = "VideoEncoder" in window ? await encodeMp4(ow, oh, bitrate) : await encodeWebm(ow, oh, bitrate);
    if (blob) {
      download(`${baseName()}_stabilized.${blob.type.includes("webm") ? "webm" : "mp4"}`, blob);
      toast("Export finished.");
    }
  } catch (e) {
    toast(`Export failed: ${e.message}`, true);
  } finally {
    S.busy = false;
    setBusy(null);
    enableUi(true);
    await rpc("config", { config: JSON.stringify(S.cfg) }).catch(() => {});
    await replan();
  }
}

async function pickCodec(w, h, bitrate) {
  // H.264 first (plays everywhere), then VP9 and AV1 in MP4 for browsers without an H.264 encoder.
  const candidates = [
    ["avc", "avc1.640034"], ["avc", "avc1.640033"], ["avc", "avc1.640028"], ["avc", "avc1.4d0034"], ["avc", "avc1.42003e"],
    ["vp9", "vp09.00.41.08"], ["vp9", "vp09.00.10.08"], ["av1", "av01.0.08M.08"],
  ];
  for (const [mux, codec] of candidates) {
    const config = { codec, width: w, height: h, bitrate, framerate: S.fps };
    if (mux === "avc") config.avc = { format: "avc" };
    try {
      const r = await VideoEncoder.isConfigSupported(config);
      if (r.supported) return { mux, config: r.config };
    } catch {}
  }
  return null;
}

async function encodeMp4(w, h, bitrate) {
  const picked = await pickCodec(w, h, bitrate);
  if (!picked) return encodeWebm(w, h, bitrate);
  const { Muxer, ArrayBufferTarget } = await import("./vendor/mp4-muxer.mjs");
  const target = new ArrayBufferTarget();
  const muxer = new Muxer({ target, video: { codec: picked.mux, width: w, height: h, frameRate: S.fps }, fastStart: "in-memory" });
  let encErr = null;
  const enc = new VideoEncoder({ output: (c, m) => muxer.addVideoChunk(c, m), error: (e) => (encErr = e) });
  enc.configure(picked.config);
  const dur = 1e6 / S.fps;
  const gop = Math.max(1, Math.round(S.fps * 2));
  for (let i = 0; i < S.frames; i++) {
    if (S.cancel) {
      enc.close();
      toast("Export cancelled.");
      return null;
    }
    if (encErr) throw encErr;
    await seek(i);
    const full = grab(S.w, S.h);
    const r = await rpc("render", { index: i, buf: full.data.buffer }, [full.data.buffer]);
    const frame = new VideoFrame(new Uint8Array(r.buf), { format: "RGBA", codedWidth: r.w, codedHeight: r.h, timestamp: Math.round(i * dur), duration: Math.round(dur) });
    enc.encode(frame, { keyFrame: i % gop === 0 });
    frame.close();
    while (enc.encodeQueueSize > 6) await new Promise((res) => setTimeout(res, 5));
    if (i % 3 === 0) setBusy(`Exporting… ${i + 1} / ${S.frames}`, (i + 1) / S.frames);
  }
  await enc.flush();
  if (encErr) throw encErr;
  muxer.finalize();
  return new Blob([target.buffer], { type: "video/mp4" });
}

async function encodeWebm(w, h, bitrate) {
  // Fallback for browsers without WebCodecs: record a canvas.
  const c = document.createElement("canvas");
  c.width = w;
  c.height = h;
  const ctx = c.getContext("2d");
  const stream = c.captureStream(0);
  const track = stream.getVideoTracks()[0];
  const rec = new MediaRecorder(stream, { mimeType: "video/webm", videoBitsPerSecond: bitrate });
  const chunks = [];
  rec.ondataavailable = (e) => e.data.size && chunks.push(e.data);
  const stopped = new Promise((r) => (rec.onstop = r));
  rec.start();
  for (let i = 0; i < S.frames && !S.cancel; i++) {
    const t = performance.now();
    await seek(i);
    const full = grab(S.w, S.h);
    const r = await rpc("render", { index: i, buf: full.data.buffer }, [full.data.buffer]);
    ctx.putImageData(new ImageData(new Uint8ClampedArray(r.buf), r.w, r.h), 0, 0);
    track.requestFrame?.();
    const wait = 1000 / S.fps - (performance.now() - t);
    if (wait > 0) await new Promise((res) => setTimeout(res, wait));
    if (i % 3 === 0) setBusy(`Exporting (real-time fallback)… ${i + 1} / ${S.frames}`, (i + 1) / S.frames);
  }
  rec.stop();
  await stopped;
  return S.cancel ? null : new Blob(chunks, { type: "video/webm" });
}

// ── Gyro ─────────────────────────────────────────────────────────────────
function updateGyroUi() {
  $("gyroClear").hidden = !S.hasGyro;
  if (!S.hasGyro) $("gyroInfo").textContent = "";
}

async function loadGyro(file) {
  if (!S.file) return toast("Open a video first.", true);
  try {
    const isBin = /\.(bin|gpmf)$/i.test(file.name);
    const info = isBin
      ? await rpc("gpmf", { buf: await file.arrayBuffer() })
      : await rpc("gyroCsv", { text: await file.text() });
    S.hasGyro = true;
    $("gyroInfo").textContent = `${file.name}: ${info.samples} samples over ${info.duration.toFixed(1)} s`;
    if (S.cfg.analysis.motion_source === "optical_flow") S.cfg.analysis.motion_source = "hybrid";
    renderControls();
    updateGyroUi();
    await replan();
  } catch (e) {
    toast(e.message, true);
  }
}

// ── Wiring ───────────────────────────────────────────────────────────────
function readJsonFile(file) {
  return file.text().then((t) => t);
}

async function main() {
  buildSettings();
  const meta = await rpc("meta");
  S.presets = meta.presets;
  S.defaults = meta.defaults;
  $("version").textContent = `Steadyframe ${meta.version}, built with Rust and WebAssembly`;
  const sel = $("preset");
  for (const [name] of S.presets) sel.add(new Option(name[0].toUpperCase() + name.slice(1), name));
  sel.add(new Option("Custom", ""));
  sel.value = "handheld";
  S.cfg = await rpc("preset", { name: "handheld" });
  $("presetDesc").textContent = S.presets[0][1];
  renderControls();

  sel.addEventListener("change", () => sel.value && applyPreset(sel.value));
  $("file").addEventListener("change", (e) => e.target.files[0] && loadVideo(e.target.files[0]));
  const drop = $("drop");
  for (const ev of ["dragenter", "dragover"]) drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.add("over"); });
  for (const ev of ["dragleave", "drop"]) drop.addEventListener(ev, () => drop.classList.remove("over"));
  drop.addEventListener("drop", (e) => {
    e.preventDefault();
    const f = [...e.dataTransfer.files].find((f) => f.type.startsWith("video/") || /\.(mp4|mov|webm|mkv|m4v)$/i.test(f.name));
    f ? loadVideo(f) : toast("Drop a video file (MP4, MOV or WebM).", true);
  });

  $("cancel").addEventListener("click", () => (S.cancel = true));
  $("scrub").addEventListener("input", (e) => {
    S.playing = false;
    showFrame(Number(e.target.value));
  });
  $("play").addEventListener("click", play);
  $("channel").addEventListener("change", drawGraph);
  graph.addEventListener("click", (e) => {
    if (!S.plan) return;
    const r = graph.getBoundingClientRect();
    const f = Math.round(((e.clientX - r.left - 10) / (r.width - 20)) * (S.frames - 1));
    S.playing = false;
    showFrame(Math.max(0, Math.min(S.frames - 1, f)));
  });
  for (const b of document.querySelectorAll(".segmented button")) {
    b.addEventListener("click", () => {
      S.mode = b.dataset.mode;
      for (const o of document.querySelectorAll(".segmented button")) o.setAttribute("aria-checked", String(o === b));
      showFrame(S.frame);
    });
  }
  // Split divider drag
  const div = $("divider");
  const drag = (e) => {
    const r = view.getBoundingClientRect();
    S.split = Math.min(1, Math.max(0, (e.clientX - r.left) / r.width));
    showFrame(S.frame);
  };
  div.addEventListener("pointerdown", (e) => {
    div.setPointerCapture(e.pointerId);
    div.addEventListener("pointermove", drag);
  });
  div.addEventListener("pointerup", () => div.removeEventListener("pointermove", drag));
  window.addEventListener("resize", () => { drawGraph(); if (S.mode === "split") positionDivider(); });
  document.addEventListener("keydown", (e) => {
    if (!S.plan || e.target.matches("input, select, textarea")) return;
    if (e.key === "ArrowRight") showFrame(Math.min(S.frames - 1, S.frame + 1));
    else if (e.key === "ArrowLeft") showFrame(Math.max(0, S.frame - 1));
    else if (e.key === " ") { e.preventDefault(); play(); }
  });

  $("export").addEventListener("click", exportVideo);
  $("gyroFile").addEventListener("change", (e) => e.target.files[0] && loadGyro(e.target.files[0]));
  $("gyroClear").addEventListener("click", async () => {
    await rpc("clearGyro");
    S.hasGyro = false;
    S.cfg.analysis.motion_source = "optical_flow";
    renderControls();
    updateGyroUi();
    replan();
  });
  $("saveTrack").addEventListener("click", async () => download(`${baseName()}.track.json`, await rpc("exportTrack")));
  $("savePlan").addEventListener("click", async () => download(`${baseName()}.plan.json`, await rpc("exportPlan")));
  $("saveConfig").addEventListener("click", () => download("steadyframe.config.json", JSON.stringify(S.cfg, null, 2)));
  $("loadConfig").addEventListener("change", async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    try {
      S.cfg = await rpc("validate", { config: await readJsonFile(f) });
      $("preset").value = "";
      $("presetDesc").textContent = `Loaded from ${f.name}`;
      renderControls();
      replan();
    } catch (err) {
      toast(`Couldn't load settings: ${err.message}`, true);
    }
  });
  $("loadTrack").addEventListener("change", async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    if (!S.file) return toast("Open the matching video first.", true);
    try {
      await rpc("importTrack", { json: await readJsonFile(f) });
      S.analysed = true;
      S.analysedWith = analysisKey(S.cfg);
      enableUi(true);
      await replan();
      toast("Motion track loaded.");
    } catch (err) {
      toast(`Couldn't load track: ${err.message}`, true);
    }
  });
}

main().catch((e) => toast(`Failed to start: ${e.message}. Was the WASM package built (see README)?`, true));
