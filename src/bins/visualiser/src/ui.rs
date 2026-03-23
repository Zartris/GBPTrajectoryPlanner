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

    // FPS / Performance overlay (top-left, minimal)
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
        .anchor(egui::Align2::RIGHT_TOP, [-5.0, 5.0])
        .title_bar(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label(format!("FPS: {:.0}  ({:.1}ms)", fps, frame_ms));
            ui.label(format!("Backend: {:.0} Hz", backend.msg_hz));
            ui.label("F1: Inspector  F2: Metrics");
        });

    // Metrics (toggled by F2) — rendered inside Control panel since separate windows fail to render
    if metrics_vis.0 {
        let active_ir_total: usize = states.0.values().map(|s| s.active_factor_count).sum();
        let entity_count = diagnostics
            .get(&bevy::diagnostic::EntityCountDiagnosticsPlugin::ENTITY_COUNT)
            .and_then(|d| d.smoothed())
            .unwrap_or(0.0) as u64;

        egui::Window::new("Metrics")
            .default_pos(egui::pos2(200.0, 200.0))
            .show(ctx, |ui| {
                ui.label(format!("FPS: {:.0}  ({:.1}ms)", fps, frame_ms));
                ui.label(format!("Backend: {:.0} Hz", backend.msg_hz));
                ui.label(format!("Robots: {}", states.0.len()));
                ui.label(format!("Active IR factors: {}", active_ir_total));
                ui.label(format!("Entities: {}", entity_count));
            });
    }

    // Global control panel
    egui::Window::new("Control").show(ctx, |ui| {
        let label = if paused.0 { "Resume" } else { "Pause" };
        if ui.button(label).clicked() {
            paused.0 = !paused.0;
            let cmd = if paused.0 { r#"{"command":"pause"}"# } else { r#"{"command":"resume"}"# };
            outbox.0.lock().unwrap_or_else(|e| e.into_inner()).push_back(cmd.to_string());
        }
        ui.separator();
        ui.label(format!("Connected: {} robots", states.0.len()));
        // Debug: always show toggle state
        ui.label(format!("F1={} F2={}", inspector_vis.0, metrics_vis.0));

        // ── Metrics section (F2 toggle) ──
        if metrics_vis.0 {
            ui.separator();
            ui.label(egui::RichText::new("== Metrics (F2) ==").strong());
            let active_ir: usize = states.0.values().map(|s| s.active_factor_count).sum();
            ui.label(format!("FPS: {:.0}  ({:.1}ms)", fps, frame_ms));
            ui.label(format!("Backend: {:.0} Hz", backend.msg_hz));
            ui.label(format!("Active IR factors: {}", active_ir));
        }

        // ── Inspector indicator (F1 toggle) ──
        if inspector_vis.0 {
            ui.separator();
            ui.label(egui::RichText::new("== Inspector ON (F1) ==").strong());
            ui.label("Inspector window should be visible");
        }

        ui.separator();

        // Draw toggles section
        egui::CollapsingHeader::new("Draw")
            .default_open(false)
            .show(ui, |ui| {
                // --- Environment group ---
                ui.label(egui::RichText::new("Environment").strong());
                ui.checkbox(&mut draw.physical_track,     "Physical track");
                ui.checkbox(&mut draw.magnetic_mainlines, "Magnetic mainlines");
                ui.checkbox(&mut draw.magnetic_markers,   "Magnetic markers");
                ui.checkbox(&mut draw.node_spheres,       "Node spheres");
                ui.checkbox(&mut draw.edge_lines,         "Edge lines");
                ui.checkbox(&mut draw.infinite_grid,      "Infinite grid");

                ui.add_space(4.0);

                // --- Robots group ---
                ui.label(egui::RichText::new("Robots").strong());
                ui.checkbox(&mut draw.robots,              "Robots");
                ui.checkbox(&mut draw.planned_paths,       "Planned paths");
                ui.checkbox(&mut draw.belief_tubes,        "Belief tubes");
                ui.checkbox(&mut draw.factor_links,        "Factor links");
                ui.checkbox(&mut draw.robot_colliders,     "Robot colliders");
                ui.checkbox(&mut draw.uncertainty_bars,    "Uncertainty bars");
                ui.checkbox(&mut draw.path_traces,         "Path traces");
                ui.checkbox(&mut draw.comm_radius_circles, "Comm radius circles");
                ui.checkbox(&mut draw.ir_safety_distance,  "IR safety distance");
                ui.checkbox(&mut draw.collision_markers,   "Collision markers");

                ui.add_space(4.0);

                // --- Gizmo master toggle ---
                ui.label(egui::RichText::new("Gizmos").strong());
                let (gizmo_cfg, _) = gizmo_store.config_mut::<DefaultGizmoConfigGroup>();
                ui.checkbox(&mut gizmo_cfg.enabled, "Enable all gizmos");

                ui.add_space(4.0);

                // --- Bulk buttons ---
                ui.horizontal(|ui| {
                    if ui.button("None").clicked() {
                        draw.physical_track     = false;
                        draw.magnetic_mainlines = false;
                        draw.magnetic_markers   = false;
                        draw.node_spheres       = false;
                        draw.edge_lines         = false;
                        draw.infinite_grid      = false;
                        draw.robots             = false;
                        draw.planned_paths      = false;
                        draw.belief_tubes       = false;
                        draw.factor_links       = false;
                        draw.robot_colliders    = false;
                        draw.uncertainty_bars   = false;
                        draw.path_traces        = false;
                        draw.comm_radius_circles = false;
                        draw.ir_safety_distance = false;
                        draw.collision_markers  = false;
                    }
                    if ui.button("All").clicked() {
                        draw.physical_track     = true;
                        draw.magnetic_mainlines = true;
                        draw.magnetic_markers   = true;
                        draw.node_spheres       = true;
                        draw.edge_lines         = true;
                        draw.infinite_grid      = true;
                        draw.robots             = true;
                        draw.planned_paths      = true;
                        draw.belief_tubes       = true;
                        draw.factor_links       = true;
                        draw.robot_colliders    = true;
                        draw.uncertainty_bars   = true;
                        draw.path_traces        = true;
                        draw.comm_radius_circles = true;
                        draw.ir_safety_distance = true;
                        draw.collision_markers  = true;
                    }
                    if ui.button("Flip").clicked() {
                        draw.physical_track = !draw.physical_track;
                        draw.magnetic_mainlines = !draw.magnetic_mainlines;
                        draw.magnetic_markers = !draw.magnetic_markers;
                        draw.node_spheres = !draw.node_spheres;
                        draw.edge_lines = !draw.edge_lines;
                        draw.infinite_grid = !draw.infinite_grid;
                        draw.robots = !draw.robots;
                        draw.planned_paths = !draw.planned_paths;
                        draw.belief_tubes = !draw.belief_tubes;
                        draw.factor_links = !draw.factor_links;
                        draw.robot_colliders = !draw.robot_colliders;
                        draw.uncertainty_bars = !draw.uncertainty_bars;
                        draw.path_traces = !draw.path_traces;
                        draw.comm_radius_circles = !draw.comm_radius_circles;
                        draw.ir_safety_distance = !draw.ir_safety_distance;
                        draw.collision_markers = !draw.collision_markers;
                    }
                    if ui.button("Reset").clicked() {
                        *draw = DrawConfig::default();
                    }
                });
            });
    });

    // Per-robot side panels
    let mut sorted_ids: Vec<u32> = states.0.keys().copied().collect();
    sorted_ids.sort();
    for id in sorted_ids {
        if let Some(state) = states.0.get(&id) {
            egui::Window::new(format!("Robot {}", id)).show(ctx, |ui| {
                ui.label(format!("cmd_v:   {}", fmt_velocity(state.velocity)));
                ui.label(format!("gbp_v:   {}", fmt_velocity(state.raw_gbp_velocity)));
                ui.label(format!("Edge:    {:?} s={:.2}", state.current_edge, state.position_s));
                let dist_str = if state.min_neighbour_dist_3d < 100.0 {
                    format!("{:.2}m", state.min_neighbour_dist_3d)
                } else {
                    "—".into()
                };
                ui.label(format!("dist3d:  {}", dist_str));
                ui.label(fmt_factor_count(state.active_factor_count));
            });
        }
    }

    if states.0.is_empty() {
        egui::Window::new("Status").show(ctx, |ui| {
            ui.label("No robots connected");
        });
    }

    Ok(())
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
