// Loads the built WASM package in Node and stabilizes a synthetic shaky clip.
// Run after: wasm-pack build crates/steadyframe-wasm --release --target web --out-dir ../../web/pkg
import { readFileSync } from "node:fs";
import init, * as sf from "../web/pkg/steadyframe_wasm.js";

await init({ module_or_path: readFileSync(new URL("../web/pkg/steadyframe_wasm_bg.wasm", import.meta.url)) });

const W = 480, H = 270, BW = 640, BH = 400, N = 45;
// Random blob texture
let seed = 7;
const rnd = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
const base = new Float32Array(BW * BH).fill(128);
for (let k = 0; k < 300; k++) {
  const bx = rnd() * BW, by = rnd() * BH, r = 3 + rnd() * 15, a = rnd() * 2 - 1;
  for (let y = Math.max(0, by - 3 * r | 0); y < Math.min(BH, by + 3 * r); y++)
    for (let x = Math.max(0, bx - 3 * r | 0); x < Math.min(BW, bx + 3 * r); x++)
      base[y * BW + x] += 90 * a * Math.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2 * r * r));
}
function frame(dx, dy) {
  const out = new Uint8Array(W * H * 4);
  for (let y = 0; y < H; y++)
    for (let x = 0; x < W; x++) {
      const sx = Math.min(BW - 1, Math.max(0, Math.round(x + dx + (BW - W) / 2)));
      const sy = Math.min(BH - 1, Math.max(0, Math.round(y + dy + (BH - H) / 2)));
      const v = Math.max(0, Math.min(255, base[sy * BW + sx]));
      const i = (y * W + x) * 4;
      out[i] = out[i + 1] = out[i + 2] = v;
      out[i + 3] = 255;
    }
  return out;
}

const s = new sf.Stabilizer(W, H, 30, sf.presetConfig("handheld"));
const frames = [];
for (let i = 0; i < N; i++) {
  const f = frame(i * 0.8 + (rnd() - 0.5) * 14, (rnd() - 0.5) * 14);
  frames.push(f);
  s.pushFrame(f, W, H);
}
s.finishAnalysis();
const stats = JSON.parse(s.buildPlan());
const out = s.render(10, frames[10]);
console.log(`version ${sf.version()}, ${stats.frames} frames, shake reduced ${stats.reduction_pct.toFixed(1)}%`);
if (stats.frames !== N) throw new Error("frame count mismatch");
if (!(stats.reduction_pct > 50)) throw new Error("stabilization too weak");
if (out.length !== s.outputWidth() * s.outputHeight() * 4) throw new Error("bad output size");
console.log("WASM smoke test passed");
