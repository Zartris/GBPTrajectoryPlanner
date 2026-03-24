// src/bins/visualiser/src/settings_panel.rs
//
// Settings panel — left-side egui window with live-tunable GBP parameters,
// simulation controls, and a "Reset to Defaults" button. Changes are detected
// per-frame and pushed over the WS outbox as `set_params` JSON commands.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::state::{LiveParams, RobotStates, WsOutbox};
use crate::ui::SimPaused;

/// Plugin that registers the settings panel UI and change-detection system.
pub struct SettingsPanelPlugin;

impl Plugin for SettingsPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LiveParams>()
            .add_systems(EguiPrimaryContextPass, draw_settings_panel)
            .add_systems(Update, detect_param_changes);
    }
}

// ── Styling constants ────────────────────────────────────────────────────────

const ACCENT: egui::Color32 = egui::Color32::from_rgb(100, 200, 220);
const DIM: egui::Color32 = egui::Color32::from_rgb(120, 130, 150);
const HEADING_COLOR: egui::Color32 = egui::Color32::from_rgb(200, 210, 230);
const LABEL_COLOR: egui::Color32 = egui::Color32::from_rgb(180, 190, 210);

// ── Settings Panel System ────────────────────────────────────────────────────

#[allow(clippy::too_many_lines)]
fn draw_settings_panel(
    mut ctxs: EguiContexts,
    mut params: ResMut<LiveParams>,
    states: Res<RobotStates>,
    mut paused: ResMut<SimPaused>,
    outbox: Res<WsOutbox>,
) -> Result {
    let ctx = ctxs.ctx_mut()?;

    egui::Window::new(egui::RichText::new("Settings").color(HEADING_COLOR).size(14.0))
        .anchor(egui::Align2::LEFT_TOP, [8.0, 8.0])
        .default_width(260.0)
        .resizable(true)
        .collapsible(true)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .max_height(ui.available_height())
                .show(ui, |ui| {
                    // ── Simulation ────────────────────────────────────────
                    egui::CollapsingHeader::new(
                        egui::RichText::new("Simulation").color(ACCENT).strong().size(12.0),
                    )
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.add_space(4.0);

                        // Pause / Resume / Step
                        ui.horizontal(|ui| {
                            let btn_text = if paused.0 { "Resume" } else { "Pause" };
                            if ui.button(egui::RichText::new(btn_text).size(12.0)).clicked() {
                                paused.0 = !paused.0;
                                let cmd = if paused.0 {
                                    r#"{"command":"pause"}"#
                                } else {
                                    r#"{"command":"resume"}"#
                                };
                                outbox.0.lock().unwrap_or_else(|e| e.into_inner())
                                    .push_back(cmd.to_string());
                            }
                            #[allow(clippy::collapsible_if)]
                            if paused.0 {
                                if ui.button(egui::RichText::new("Step").size(12.0)).clicked() {
                                    outbox.0.lock().unwrap_or_else(|e| e.into_inner())
                                        .push_back(r#"{"command":"step"}"#.to_string());
                                }
                            }
                        });

                        ui.add_space(2.0);
                        ui.label(
                            egui::RichText::new(format!("{} robots connected", states.0.len()))
                                .color(DIM)
                                .size(11.0),
                        );

                        ui.add_space(4.0);
                        param_slider_f32(ui, "Timescale", &mut params.timescale, 0.1..=10.0);
                    });

                    ui.add_space(2.0);

                    // ── GBP Solver ────────────────────────────────────────
                    egui::CollapsingHeader::new(
                        egui::RichText::new("GBP Solver").color(ACCENT).strong().size(12.0),
                    )
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.add_space(4.0);
                        param_slider_f32(ui, "msg_damping", &mut params.msg_damping, 0.0..=1.0);
                        param_slider_u8(ui, "internal_iters", &mut params.internal_iters, 1..=50);
                        param_slider_u8(ui, "external_iters", &mut params.external_iters, 1..=50);
                        param_slider_f32(ui, "gbp_timestep", &mut params.gbp_timestep, 0.01..=1.0);
                        param_slider_f32(ui, "init_variance", &mut params.init_variance, 1.0..=1000.0);
                        param_slider_f32(ui, "anchor_precision", &mut params.anchor_precision, 1.0..=10000.0);
                    });

                    ui.add_space(2.0);

                    // ── Factor Weights ────────────────────────────────────
                    egui::CollapsingHeader::new(
                        egui::RichText::new("Factor Weights").color(ACCENT).strong().size(12.0),
                    )
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.add_space(4.0);
                        section_label(ui, "DYNAMICS");
                        param_slider_f32(ui, "sigma", &mut params.sigma_dynamics, 0.01..=5.0);

                        ui.add_space(4.0);
                        section_label(ui, "INTER-ROBOT");
                        param_slider_f32(ui, "d_safe", &mut params.d_safe, 0.1..=5.0);
                        param_slider_f32(ui, "sigma", &mut params.sigma_interrobot, 0.01..=2.0);
                        param_slider_f32(ui, "activation_range", &mut params.ir_activation_range, 0.5..=20.0);
                        param_slider_f32(ui, "decay_alpha", &mut params.ir_decay_alpha, 0.1..=10.0);
                        param_slider_f32(ui, "front_damping", &mut params.front_damping, 0.0..=1.0);
                    });

                    ui.add_space(2.0);

                    // ── Velocity Bounds ───────────────────────────────────
                    egui::CollapsingHeader::new(
                        egui::RichText::new("Velocity Bounds").color(ACCENT).strong().size(12.0),
                    )
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.add_space(4.0);
                        param_slider_f32(ui, "v_min", &mut params.v_min, -2.0..=0.0);
                        param_slider_f32(ui, "v_max_default", &mut params.v_max_default, 0.5..=10.0);
                        param_slider_f32(ui, "kappa", &mut params.vb_kappa, 0.1..=100.0);
                        param_slider_f32(ui, "margin", &mut params.vb_margin, 0.01..=5.0);
                        param_slider_f32(ui, "max_precision", &mut params.vb_max_precision, 1.0..=1000.0);
                    });

                    ui.add_space(2.0);

                    // ── Robot Constraints ─────────────────────────────────
                    egui::CollapsingHeader::new(
                        egui::RichText::new("Robot Constraints").color(ACCENT).strong().size(12.0),
                    )
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.add_space(4.0);
                        param_slider_f32(ui, "max_accel", &mut params.max_accel, 0.1..=10.0);
                        param_slider_f32(ui, "max_jerk", &mut params.max_jerk, 0.1..=20.0);
                        param_slider_f32(ui, "max_speed", &mut params.max_speed, 0.5..=10.0);
                    });

                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(4.0);

                    // ── Reset to Defaults ─────────────────────────────────
                    if ui
                        .button(egui::RichText::new("Reset to Defaults").size(12.0))
                        .clicked()
                    {
                        let timescale = params.timescale; // preserve timescale across reset
                        *params = LiveParams::default();
                        params.timescale = timescale;
                    }
                });
        });

    Ok(())
}

// ── Change Detection System ──────────────────────────────────────────────────

/// Compares the current `LiveParams` against the last-sent snapshot each frame.
/// When a change is detected, serializes the full param set as a `set_params`
/// command and pushes it into the WS outbox.
fn detect_param_changes(
    params: Res<LiveParams>,
    outbox: Res<WsOutbox>,
    mut last_sent: Local<Option<LiveParams>>,
) {
    let changed = match last_sent.as_ref() {
        None => true, // first frame — send initial state
        Some(prev) => params.differs_from(prev),
    };

    if changed {
        let json = params.to_set_params_json();
        outbox
            .0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push_back(json);
        *last_sent = Some(params.clone());
    }
}

// ── UI Helpers ───────────────────────────────────────────────────────────────

/// Subcategory label (uppercase, muted), matching the style in ui.rs.
fn section_label(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        ui.add_space(2.0);
        ui.label(
            egui::RichText::new(text)
                .color(egui::Color32::from_rgb(160, 170, 190))
                .size(9.5)
                .strong(),
        );
    });
    ui.add(egui::Separator::default().spacing(1.0));
}

/// Slider row for an f32 parameter.
fn param_slider_f32(ui: &mut egui::Ui, label: &str, value: &mut f32, range: std::ops::RangeInclusive<f32>) {
    ui.horizontal(|ui| {
        ui.add_space(4.0);
        ui.label(egui::RichText::new(label).color(LABEL_COLOR).size(11.0));
    });
    ui.horizontal(|ui| {
        ui.add_space(8.0);
        ui.spacing_mut().slider_width = (ui.available_width() - 16.0).max(60.0);
        ui.add(
            egui::Slider::new(value, range)
                .max_decimals(3)
                .show_value(true),
        );
    });
}

/// Slider row for a u8 parameter (cast through i32 for egui compatibility).
fn param_slider_u8(ui: &mut egui::Ui, label: &str, value: &mut u8, range: std::ops::RangeInclusive<u8>) {
    let mut v = *value as i32;
    let r = *range.start() as i32..=*range.end() as i32;
    ui.horizontal(|ui| {
        ui.add_space(4.0);
        ui.label(egui::RichText::new(label).color(LABEL_COLOR).size(11.0));
    });
    ui.horizontal(|ui| {
        ui.add_space(8.0);
        ui.spacing_mut().slider_width = (ui.available_width() - 16.0).max(60.0);
        ui.add(egui::Slider::new(&mut v, r).show_value(true));
    });
    *value = v.clamp(0, 255) as u8;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn live_params_default_matches_config() {
        let p = LiveParams::default();
        assert!((p.msg_damping - 0.5).abs() < 1e-6);
        assert_eq!(p.internal_iters, 10);
        assert_eq!(p.external_iters, 10);
        assert!((p.d_safe - 1.3).abs() < 1e-6);
        assert!((p.sigma_interrobot - 0.12).abs() < 1e-6);
        assert!((p.timescale - 1.0).abs() < 1e-6);
    }

    #[test]
    fn to_set_params_json_is_valid() {
        let p = LiveParams::default();
        let json = p.to_set_params_json();
        // Should be valid JSON with the expected command
        assert!(json.starts_with(r#"{"command":"set_params","params":{"#));
        assert!(json.ends_with("}}"));
        assert!(json.contains("\"msg_damping\""));
        assert!(json.contains("\"timescale\""));
        // Should parse as valid JSON
        let v: serde_json::Value = serde_json::from_str(&json).expect("valid JSON");
        assert_eq!(v["command"], "set_params");
        assert!((v["params"]["msg_damping"].as_f64().unwrap() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn differs_from_detects_changes() {
        let a = LiveParams::default();
        let mut b = a.clone();
        assert!(!a.differs_from(&b));

        b.msg_damping = 0.6;
        assert!(a.differs_from(&b));

        b = a.clone();
        b.internal_iters = 5;
        assert!(a.differs_from(&b));
    }

    #[test]
    fn differs_from_ignores_tiny_float_noise() {
        let a = LiveParams::default();
        let mut b = a.clone();
        b.msg_damping += 1e-8; // below epsilon
        assert!(!a.differs_from(&b));
    }
}
