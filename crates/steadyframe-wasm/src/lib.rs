//! WebAssembly API for steadyframe.
//!
//! ```js
//! import init, { Stabilizer, presetConfig } from "./pkg/steadyframe_wasm.js";
//! await init();
//! const s = new Stabilizer(1920, 1080, 30, presetConfig("handheld"));
//! for (const frame of analysisFrames) s.pushFrame(frame.data, frame.width, frame.height);
//! s.finishAnalysis();
//! const stats = JSON.parse(s.buildPlan());
//! const out = s.render(index, fullResRgba);   // Uint8Array, outputWidth × outputHeight
//! ```

use serde_json::json;
use steadyframe_core::gpmf::parse_gpmf;
use steadyframe_core::{Analyzer, Config, GyroData, MotionTrack, Plan, Renderer, PRESETS};
use wasm_bindgen::prelude::*;

fn err<E: std::fmt::Display>(e: E) -> JsError {
    JsError::new(&e.to_string())
}

#[wasm_bindgen(start)]
pub fn start() {
    console_error_panic_hook::set_once();
}

/// Library version.
#[wasm_bindgen]
pub fn version() -> String {
    steadyframe_core::VERSION.to_string()
}

/// Default configuration as JSON.
#[wasm_bindgen(js_name = defaultConfig)]
pub fn default_config() -> String {
    Config::default().to_json_pretty()
}

/// A preset's configuration as JSON.
#[wasm_bindgen(js_name = presetConfig)]
pub fn preset_config(name: &str) -> Result<String, JsError> {
    Ok(Config::preset(name).map_err(err)?.to_json_pretty())
}

/// `[[name, description], …]` as JSON.
#[wasm_bindgen(js_name = presetList)]
pub fn preset_list() -> String {
    serde_json::to_string(PRESETS).unwrap()
}

/// Validate a (partial) config; returns the merged full config JSON or throws.
#[wasm_bindgen(js_name = validateConfig)]
pub fn validate_config(json: &str) -> Result<String, JsError> {
    Ok(Config::from_json(json).map_err(err)?.to_json_pretty())
}

/// Stateful stabilizer: analyse frames, build a plan, render frames.
#[wasm_bindgen]
pub struct Stabilizer {
    cfg: Config,
    width: u32,
    height: u32,
    fps: f64,
    analyzer: Option<Analyzer>,
    track: Option<MotionTrack>,
    plan: Option<Plan>,
    gyro: Option<GyroData>,
    renderer: Renderer,
}

#[wasm_bindgen]
impl Stabilizer {
    /// `config_json` may be empty, partial or complete.
    #[wasm_bindgen(constructor)]
    pub fn new(width: u32, height: u32, fps: f64, config_json: &str) -> Result<Stabilizer, JsError> {
        let cfg = Config::from_json(config_json).map_err(err)?;
        let analyzer = Analyzer::new(&cfg, width, height, fps).map_err(err)?;
        Ok(Stabilizer {
            cfg,
            width,
            height,
            fps,
            analyzer: Some(analyzer),
            track: None,
            plan: None,
            gyro: None,
            renderer: Renderer::new(),
        })
    }

    /// Merge a (partial) JSON config into the current one. Analysis settings
    /// only take effect after `resetAnalysis()`.
    #[wasm_bindgen(js_name = setConfig)]
    pub fn set_config(&mut self, json: &str) -> Result<(), JsError> {
        self.cfg = self.cfg.merged_with_json(json).map_err(err)?;
        self.plan = None;
        Ok(())
    }

    /// Replace the whole config (missing fields take defaults).
    #[wasm_bindgen(js_name = replaceConfig)]
    pub fn replace_config(&mut self, json: &str) -> Result<(), JsError> {
        self.cfg = Config::from_json(json).map_err(err)?;
        self.plan = None;
        Ok(())
    }

    #[wasm_bindgen(js_name = getConfig)]
    pub fn get_config(&self) -> String {
        self.cfg.to_json_pretty()
    }

    /// Recommended width for frames passed to `pushFrame` (any size works).
    #[wasm_bindgen(js_name = analysisWidth)]
    pub fn analysis_width(&self) -> u32 {
        self.analyzer.as_ref().map(|a| a.analysis_size().0).unwrap_or(0)
    }

    #[wasm_bindgen(js_name = analysisHeight)]
    pub fn analysis_height(&self) -> u32 {
        self.analyzer.as_ref().map(|a| a.analysis_size().1).unwrap_or(0)
    }

    /// Start a fresh analysis with the current config.
    #[wasm_bindgen(js_name = resetAnalysis)]
    pub fn reset_analysis(&mut self) -> Result<(), JsError> {
        self.analyzer = Some(Analyzer::new(&self.cfg, self.width, self.height, self.fps).map_err(err)?);
        self.track = None;
        self.plan = None;
        Ok(())
    }

    /// Analyse the next frame (RGBA, any size). Returns tracking confidence 0..1.
    #[wasm_bindgen(js_name = pushFrame)]
    pub fn push_frame(&mut self, rgba: &[u8], width: u32, height: u32) -> Result<f32, JsError> {
        let a = self.analyzer.as_mut().ok_or_else(|| JsError::new("analysis finished; call resetAnalysis() first"))?;
        Ok(a.push_rgba(rgba, width, height).map_err(err)?.confidence)
    }

    /// Analyse the next frame given as 8-bit grayscale.
    #[wasm_bindgen(js_name = pushLuma)]
    pub fn push_luma(&mut self, luma: &[u8], width: u32, height: u32) -> Result<f32, JsError> {
        let a = self.analyzer.as_mut().ok_or_else(|| JsError::new("analysis finished; call resetAnalysis() first"))?;
        Ok(a.push_luma(luma, width, height).map_err(err)?.confidence)
    }

    #[wasm_bindgen(js_name = frameCount)]
    pub fn frame_count(&self) -> usize {
        match (&self.analyzer, &self.track) {
            (Some(a), _) => a.frames(),
            (None, Some(t)) => t.len(),
            _ => 0,
        }
    }

    /// Finish analysis (no more frames).
    #[wasm_bindgen(js_name = finishAnalysis)]
    pub fn finish_analysis(&mut self) -> Result<(), JsError> {
        let a = self.analyzer.take().ok_or_else(|| JsError::new("analysis already finished"))?;
        if a.frames() == 0 {
            self.analyzer = Some(a);
            return Err(JsError::new("no frames were analysed"));
        }
        self.track = Some(a.finish());
        self.plan = None;
        Ok(())
    }

    /// Use gyro-only motion without analysing frames (`frames` = frame count).
    #[wasm_bindgen(js_name = skipAnalysis)]
    pub fn skip_analysis(&mut self, frames: usize) {
        self.analyzer = None;
        self.track = Some(MotionTrack::empty(self.width, self.height, self.fps, frames));
        self.plan = None;
    }

    /// Load a gyro CSV log. Returns `{samples, duration}` JSON.
    #[wasm_bindgen(js_name = loadGyroCsv)]
    pub fn load_gyro_csv(&mut self, text: &str) -> Result<String, JsError> {
        let g = GyroData::from_csv(text, self.cfg.gyro.time_unit, self.cfg.gyro.angle_unit).map_err(err)?;
        let s = json!({ "samples": g.len(), "duration": g.duration() }).to_string();
        self.gyro = Some(g);
        self.plan = None;
        Ok(s)
    }

    /// Load a raw GoPro GPMF track. Returns `{samples, duration}` JSON.
    #[wasm_bindgen(js_name = loadGpmf)]
    pub fn load_gpmf(&mut self, bytes: &[u8]) -> Result<String, JsError> {
        let g = parse_gpmf(bytes).map_err(err)?;
        let s = json!({ "samples": g.len(), "duration": g.duration() }).to_string();
        self.gyro = Some(g);
        self.plan = None;
        Ok(s)
    }

    #[wasm_bindgen(js_name = clearGyro)]
    pub fn clear_gyro(&mut self) {
        self.gyro = None;
        self.plan = None;
    }

    #[wasm_bindgen(js_name = hasGyro)]
    pub fn has_gyro(&self) -> bool {
        self.gyro.is_some()
    }

    /// Smooth the path with the current config. Returns stats JSON.
    #[wasm_bindgen(js_name = buildPlan)]
    pub fn build_plan(&mut self) -> Result<String, JsError> {
        let track = self.track.as_ref().ok_or_else(|| JsError::new("analyse the video first"))?;
        let plan = Plan::build(&self.cfg, track, self.gyro.as_ref()).map_err(err)?;
        let stats = serde_json::to_string(&plan.stats).unwrap();
        self.plan = Some(plan);
        self.renderer = Renderer::new();
        Ok(stats)
    }

    fn plan(&self) -> Result<&Plan, JsError> {
        self.plan.as_ref().ok_or_else(|| JsError::new("call buildPlan() first"))
    }

    /// Final output width (includes compare layout).
    #[wasm_bindgen(js_name = outputWidth)]
    pub fn output_width(&self) -> u32 {
        self.plan.as_ref().map(|p| p.final_size().0).unwrap_or(self.width)
    }

    #[wasm_bindgen(js_name = outputHeight)]
    pub fn output_height(&self) -> u32 {
        self.plan.as_ref().map(|p| p.final_size().1).unwrap_or(self.height)
    }

    /// Render frame `index` from full-resolution RGBA. Returns RGBA bytes.
    pub fn render(&mut self, index: usize, src: &[u8]) -> Result<Vec<u8>, JsError> {
        let plan = self.plan.as_ref().ok_or_else(|| JsError::new("call buildPlan() first"))?;
        Ok(self.renderer.render(plan, index, src).map_err(err)?.to_vec())
    }

    /// Render into an existing buffer (avoids an allocation per frame).
    #[wasm_bindgen(js_name = renderInto)]
    pub fn render_into(&mut self, index: usize, src: &[u8], dst: &mut [u8]) -> Result<(), JsError> {
        let plan = self.plan.as_ref().ok_or_else(|| JsError::new("call buildPlan() first"))?;
        let out = self.renderer.render(plan, index, src).map_err(err)?;
        if dst.len() < out.len() {
            return Err(JsError::new("destination buffer too small"));
        }
        dst[..out.len()].copy_from_slice(out);
        Ok(())
    }

    #[wasm_bindgen(js_name = exportTrack)]
    pub fn export_track(&self) -> Result<String, JsError> {
        Ok(self.track.as_ref().ok_or_else(|| JsError::new("no motion track yet"))?.to_json())
    }

    #[wasm_bindgen(js_name = importTrack)]
    pub fn import_track(&mut self, json: &str) -> Result<(), JsError> {
        let t = MotionTrack::from_json(json).map_err(err)?;
        if t.width != self.width || t.height != self.height {
            return Err(JsError::new(&format!(
                "track is for {}x{}, video is {}x{}",
                t.width, t.height, self.width, self.height
            )));
        }
        self.analyzer = None;
        self.track = Some(t);
        self.plan = None;
        Ok(())
    }

    #[wasm_bindgen(js_name = exportPlan)]
    pub fn export_plan(&self) -> Result<String, JsError> {
        Ok(self.plan()?.to_json())
    }

    /// Graph data: `{raw, smooth, zoom, confidence}`.
    pub fn paths(&self) -> Result<String, JsError> {
        let p = self.plan()?;
        let conf: Vec<f32> =
            self.track.as_ref().map(|t| t.motions.iter().map(|m| m.confidence).collect()).unwrap_or_default();
        Ok(json!({
            "raw": p.raw_path,
            "smooth": p.smooth_path,
            "zoom": p.frames.iter().map(|f| f.zoom).collect::<Vec<_>>(),
            "confidence": conf,
        })
        .to_string())
    }
}
