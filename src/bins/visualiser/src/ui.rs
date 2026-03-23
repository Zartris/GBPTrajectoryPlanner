// src/bins/visualiser/src/ui.rs
use bevy::prelude::*;
use tracing::info;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::gizmos::config::{DefaultGizmoConfigGroup, GizmoConfigStore};
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::state::{DrawConfig, InspectorVisible, MetricsVisible, RobotStates, WsOutbox};

/// Shared pause flag.
#[derive(Resource, Default)]
pub struct SimPaused(pub bool);


/// Tracks backend message rate by counting messages per second.
#[derive(Resource)]
pub struct BackendStats {
    pub msg_hz: f32,
    msg_count: u32,
    window_start: std::time::Instant,
}

impl Default for BackendStats {
    fn default() -> Self {
        Self {
            msg_hz: 0.0,
            msg_count: 0,
            window_start: std::time::Instant::now(),
        }
    }
}

impl BackendStats {
    pub fn record_message(&mut self) {
        self.msg_count += 1;
        let elapsed = self.window_start.elapsed().as_secs_f32();
        if elapsed >= 1.0 {
            self.msg_hz = self.msg_count as f32 / elapsed;
            self.msg_count = 0;
            self.window_start = std::time::Instant::now();
        }
    }
}

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
           .add_plugins(bevy::diagnostic::EntityCountDiagnosticsPlugin::default())
           .init_resource::<SimPaused>()
           .init_resource::<BackendStats>()
           
           .init_resource::<MetricsVisible>()
           .init_resource::<InspectorVisible>()
           // Toggle runs in Update (once per frame) — NOT in EguiPrimaryContextPass
           // which runs multiple times per frame (multipass) and would double-toggle.
           .add_systems(Update, toggle_overlays)
           .add_systems(EguiPrimaryContextPass, draw_hud);
    }
}

/// Toggle F1/F2 overlays. Runs in Update (once per frame).
fn toggle_overlays(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut metrics_vis: ResMut<MetricsVisible>,
    mut inspector_vis: ResMut<InspectorVisible>,
) {
    if keyboard.just_pressed(KeyCode::F1) {
        inspector_vis.0 = !inspector_vis.0;
        info!("[ui] F1 toggled inspector: {}", inspector_vis.0);
    }
    if keyboard.just_pressed(KeyCode::F2) {
        metrics_vis.0 = !metrics_vis.0;
        info!("[ui] F2 toggled metrics: {}", metrics_vis.0);
    }
}

pub fn fmt_velocity(v: f32) -> String {
    format!("{:.2} m/s", v)
}

pub fn fmt_factor_count(n: usize) -> String {
    if n == 1 { "1 factor".into() } else { format!("{} factors", n) }
}

fn draw_hud(
    mut ctxs: EguiContexts,
    states: Res<RobotStates>,
    mut paused: ResMut<SimPaused>,
    diagnostics: Res<DiagnosticsStore>,
    backend: Res<BackendStats>,
    outbox: Res<WsOutbox>,
    mut draw: ResMut<DrawConfig>,
    mut gizmo_store: ResMut<GizmoConfigStore>,
    metrics_vis: Res<MetricsVisible>,
    inspector_vis: Res<InspectorVisible>,
) -> Result {
    let ctx = ctxs.ctx_mut()?;

    // ── Global egui style — dark, clean, generous spacing ──────────────
    ctx.style_mut(|s| {
        s.visuals.window_shadow = egui::Shadow::NONE;
        s.visuals.window_stroke = egui::Stroke::new(1.0, egui::Color32::from_white_alpha(30));
        s.visuals.widgets.noninteractive.bg_fill = egui::Color32::from_rgba_premultiplied(15, 18, 35, 220);
        s.spacing.item_spacing = egui::vec2(8.0, 5.0);
        s.spacing.window_margin = egui::Margin::same(12);
    });

    // FPS / Performance overlay (top-right, minimal)
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);
    let frame_ms = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FRAME_TIME)
        .and_then(|d| d.smoothed())
        .map(|ms| ms * 1000.0)
        .unwrap_or(0.0);

    egui::Window::new("Performance")
        .anchor(egui::Align2::RIGHT_TOP, [-8.0, 8.0])
        .title_bar(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label(egui::RichText::new(format!("FPS: {:.0}  ({:.1}ms)", fps, frame_ms))
                .monospace().size(11.0).color(egui::Color32::from_rgb(180, 200, 220)));
            ui.label(egui::RichText::new(format!("Backend: {:.0} Hz", backend.msg_hz))
                .monospace().size(11.0).color(egui::Color32::from_rgb(180, 200, 220)));
            ui.add_space(2.0);
            ui.label(egui::RichText::new("F1 Inspector  ·  F2 Metrics")
                .size(10.0).color(egui::Color32::from_rgb(100, 110, 130)));
        });

    // ── Control Panel ─────────────────────────────────────────────────
    let accent = egui::Color32::from_rgb(100, 200, 220); // teal accent
    let dim = egui::Color32::from_rgb(120, 130, 150);    // muted text
    let heading_color = egui::Color32::from_rgb(200, 210, 230);

    egui::Window::new(egui::RichText::new("◈  Control").color(heading_color).size(14.0))
        .default_width(220.0)
        .show(ctx, |ui| {
        // ── Simulation ──
        ui.add_space(2.0);
        let btn_text = if paused.0 { "▶  Resume" } else { "⏸  Pause" };
        if ui.button(egui::RichText::new(btn_text).size(13.0)).clicked() {
            paused.0 = !paused.0;
            let cmd = if paused.0 { r#"{"command":"pause"}"# } else { r#"{"command":"resume"}"# };
            outbox.0.lock().unwrap_or_else(|e| e.into_inner()).push_back(cmd.to_string());
        }
        ui.add_space(2.0);
        ui.label(egui::RichText::new(format!("⬡  {} robots connected", states.0.len()))
            .color(dim).size(11.0));

        ui.add_space(6.0);
        ui.separator();
        ui.add_space(4.0);

        // ── Draw Toggles ──
        egui::CollapsingHeader::new(egui::RichText::new("◉  Draw Layers").color(accent).size(12.0))
            .default_open(false)
            .show(ui, |ui| {
                ui.add_space(4.0);

                // Environment group
                section_heading(ui, "⬡  Environment", heading_color);
                styled_toggle(ui, &mut draw.physical_track,     "Physical track");
                styled_toggle(ui, &mut draw.magnetic_mainlines, "Magnetic mainlines");
                styled_toggle(ui, &mut draw.magnetic_markers,   "Magnetic markers");
                styled_toggle(ui, &mut draw.node_spheres,       "Node spheres");
                styled_toggle(ui, &mut draw.edge_lines,         "Edge lines");
                styled_toggle(ui, &mut draw.infinite_grid,      "Infinite grid");

                ui.add_space(6.0);

                // Robots group
                section_heading(ui, "⊕  Robots", heading_color);
                styled_toggle(ui, &mut draw.robots,              "Robots");
                styled_toggle(ui, &mut draw.planned_paths,       "Planned paths");
                styled_toggle(ui, &mut draw.belief_tubes,        "Belief tubes");
                styled_toggle(ui, &mut draw.factor_links,        "Factor links");
                styled_toggle(ui, &mut draw.robot_colliders,     "Robot colliders");
                styled_toggle(ui, &mut draw.uncertainty_bars,    "Uncertainty bars");
                styled_toggle(ui, &mut draw.path_traces,         "Path traces");
                styled_toggle(ui, &mut draw.comm_radius_circles, "Comm radius");
                styled_toggle(ui, &mut draw.ir_safety_distance,  "IR safety dist");
                styled_toggle(ui, &mut draw.collision_markers,   "Collisions");

                ui.add_space(6.0);

                // Gizmo master
                section_heading(ui, "⚙  Gizmos", heading_color);
                let (gizmo_cfg, _) = gizmo_store.config_mut::<DefaultGizmoConfigGroup>();
                styled_toggle(ui, &mut gizmo_cfg.enabled, "All gizmos");

                ui.add_space(6.0);
                ui.separator();
                ui.add_space(4.0);

                // Bulk buttons
                ui.horizontal(|ui| {
                    for (label, action) in [("None", false), ("All", true)] {
                        if ui.small_button(label).clicked() {
                            set_all_draw(&mut draw, action);
                        }
                    }
                    if ui.small_button("Flip").clicked() {
                        flip_all_draw(&mut draw);
                    }
                    if ui.small_button("Reset").clicked() {
                        *draw = DrawConfig::default();
                    }
                });
            });
    });

    // ── Per-robot panels ────────────────────────────────────────────
    let robot_colors = [
        egui::Color32::from_rgb(30, 180, 255),  // blue
        egui::Color32::from_rgb(255, 120, 40),   // orange
        egui::Color32::from_rgb(60, 255, 120),   // green
        egui::Color32::from_rgb(255, 60, 200),   // pink
    ];
    let mut sorted_ids: Vec<u32> = states.0.keys().copied().collect();
    sorted_ids.sort();
    for id in sorted_ids {
        if let Some(state) = states.0.get(&id) {
            let rc = robot_colors[(id as usize) % robot_colors.len()];
            egui::Window::new(
                egui::RichText::new(format!("◈  Robot {}", id)).color(rc).size(13.0)
            ).show(ctx, |ui| {
                stat_row(ui, "cmd_v", &fmt_velocity(state.velocity), dim);
                stat_row(ui, "gbp_v", &fmt_velocity(state.raw_gbp_velocity), dim);
                stat_row(ui, "edge",  &format!("{:?}  s={:.2}", state.current_edge, state.position_s), dim);
                let dist_str = if state.min_neighbour_dist_3d < 100.0 {
                    format!("{:.2}m", state.min_neighbour_dist_3d)
                } else { "—".into() };
                stat_row(ui, "dist3d", &dist_str, dim);
                stat_row(ui, "factors", &fmt_factor_count(state.active_factor_count), dim);
            });
        }
    }

    if states.0.is_empty() {
        egui::Window::new(egui::RichText::new("◈  Status").color(dim).size(13.0))
            .show(ctx, |ui| {
            ui.label(egui::RichText::new("No robots connected").color(dim).italics());
        });
    }

    Ok(())
}

// ── UI Helpers ──────────────────────────────────────────────────────────────

/// Colored section heading with a thin separator line beneath.
fn section_heading(ui: &mut egui::Ui, text: &str, color: egui::Color32) {
    ui.label(egui::RichText::new(text).color(color).size(11.5).strong());
    ui.add(egui::Separator::default().spacing(2.0));
}

/// Toggle switch styled as a compact row: label on left, toggle on right.
fn styled_toggle(ui: &mut egui::Ui, value: &mut bool, label: &str) {
    let row_bg = if *value {
        egui::Color32::from_rgba_premultiplied(40, 55, 80, 60)
    } else {
        egui::Color32::TRANSPARENT
    };
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), 18.0),
        egui::Sense::hover(),
    );
    if row_bg != egui::Color32::TRANSPARENT {
        ui.painter().rect_filled(rect, 2.0, row_bg);
    }
    ui.horizontal(|ui| {
        ui.add_space(4.0);
        let dot = if *value { "●" } else { "○" };
        let dot_color = if *value {
            egui::Color32::from_rgb(100, 220, 180)
        } else {
            egui::Color32::from_rgb(80, 85, 100)
        };
        if ui.add(egui::Label::new(
            egui::RichText::new(dot).color(dot_color).size(10.0)
        ).sense(egui::Sense::click())).clicked() {
            *value = !*value;
        }
        if ui.add(egui::Label::new(
            egui::RichText::new(label).size(11.0)
        ).sense(egui::Sense::click())).clicked() {
            *value = !*value;
        }
    });
}

/// Key-value stat row for robot panels.
fn stat_row(ui: &mut egui::Ui, key: &str, value: &str, dim: egui::Color32) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(key).color(dim).monospace().size(10.5));
        ui.label(egui::RichText::new(value).monospace().size(10.5));
    });
}

fn set_all_draw(draw: &mut DrawConfig, on: bool) {
    draw.physical_track = on; draw.magnetic_mainlines = on; draw.magnetic_markers = on;
    draw.node_spheres = on; draw.edge_lines = on; draw.infinite_grid = on;
    draw.robots = on; draw.planned_paths = on; draw.belief_tubes = on;
    draw.factor_links = on; draw.robot_colliders = on; draw.uncertainty_bars = on;
    draw.path_traces = on; draw.comm_radius_circles = on;
    draw.ir_safety_distance = on; draw.collision_markers = on;
}

fn flip_all_draw(draw: &mut DrawConfig) {
    draw.physical_track = !draw.physical_track; draw.magnetic_mainlines = !draw.magnetic_mainlines;
    draw.magnetic_markers = !draw.magnetic_markers; draw.node_spheres = !draw.node_spheres;
    draw.edge_lines = !draw.edge_lines; draw.infinite_grid = !draw.infinite_grid;
    draw.robots = !draw.robots; draw.planned_paths = !draw.planned_paths;
    draw.belief_tubes = !draw.belief_tubes; draw.factor_links = !draw.factor_links;
    draw.robot_colliders = !draw.robot_colliders; draw.uncertainty_bars = !draw.uncertainty_bars;
    draw.path_traces = !draw.path_traces; draw.comm_radius_circles = !draw.comm_radius_circles;
    draw.ir_safety_distance = !draw.ir_safety_distance; draw.collision_markers = !draw.collision_markers;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_velocity_string_two_decimal() {
        assert_eq!(fmt_velocity(2.0), "2.00 m/s");
        assert_eq!(fmt_velocity(0.0), "0.00 m/s");
    }

    #[test]
    fn sim_paused_default_is_false() {
        let p = SimPaused::default();
        assert!(!p.0);
    }

    #[test]
    fn format_factor_count_singular_and_plural() {
        assert_eq!(fmt_factor_count(0), "0 factors");
        assert_eq!(fmt_factor_count(1), "1 factor");
        assert_eq!(fmt_factor_count(3), "3 factors");
    }
}
