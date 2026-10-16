// Runs the Rust/WASM stabilizer off the main thread.
import init, * as sf from "./pkg/steadyframe_wasm.js";

const ready = init();
let st = null;

function need() {
  if (!st) throw new Error("No video loaded");
  return st;
}

const handlers = {
  meta: () => ({ presets: JSON.parse(sf.presetList()), version: sf.version(), defaults: JSON.parse(sf.defaultConfig()) }),
  preset: (m) => JSON.parse(sf.presetConfig(m.name)),
  validate: (m) => JSON.parse(sf.validateConfig(m.config)),
  init: (m) => {
    if (st) st.free();
    st = new sf.Stabilizer(m.width, m.height, m.fps, m.config);
    return { aw: st.analysisWidth(), ah: st.analysisHeight() };
  },
  config: (m) => { need().replaceConfig(m.config); return true; },
  reset: () => { need().resetAnalysis(); return { aw: st.analysisWidth(), ah: st.analysisHeight() }; },
  push: (m) => need().pushFrame(new Uint8Array(m.buf), m.w, m.h),
  finish: () => { need().finishAnalysis(); return true; },
  skip: (m) => { need().skipAnalysis(m.frames); return true; },
  plan: () => {
    const s = need();
    return { stats: JSON.parse(s.buildPlan()), paths: JSON.parse(s.paths()), ow: s.outputWidth(), oh: s.outputHeight() };
  },
  gyroCsv: (m) => JSON.parse(need().loadGyroCsv(m.text)),
  gpmf: (m) => JSON.parse(need().loadGpmf(new Uint8Array(m.buf))),
  clearGyro: () => { need().clearGyro(); return true; },
  exportTrack: () => need().exportTrack(),
  importTrack: (m) => { need().importTrack(m.json); return true; },
  exportPlan: () => need().exportPlan(),
};

self.onmessage = async (e) => {
  const m = e.data;
  try {
    await ready;
    if (m.type === "render") {
      const s = need();
      const out = s.render(m.index, new Uint8Array(m.buf));
      self.postMessage({ id: m.id, ok: true, result: { buf: out.buffer, w: s.outputWidth(), h: s.outputHeight() } }, [out.buffer]);
      return;
    }
    const h = handlers[m.type];
    if (!h) throw new Error(`unknown message ${m.type}`);
    self.postMessage({ id: m.id, ok: true, result: h(m) });
  } catch (err) {
    self.postMessage({ id: m.id, ok: false, error: String(err && err.message ? err.message : err) });
  }
};
