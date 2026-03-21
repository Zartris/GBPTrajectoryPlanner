// src/bins/visualiser/src/ui.rs
use bevy::prelude::*;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::state::{RobotStates, WsOutbox};

/// Shared pause flag.
#[derive(Resource, Default)]
pub struct SimPaused(pub bool);

/// Global simulation parameters (read-only display for now; live tuning deferred to M6).
#[derive(Resource)]
pub struct GlobalParams {
    pub num_robots: usize,
    pub random_mode: bool,
}

impl Default for GlobalParams {
    fn default() -> Self {
        Self { num_robots: 4, random_mode: false }
    }
}

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
           .init_resource::<SimPaused>()
           .init_resource::<BackendStats>()
           .init_resource::<GlobalParams>()
           .add_systems(EguiPrimaryContextPass, draw_hud);
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
    params: Res<GlobalParams>,
    outbox: Res<WsOutbox>,
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
        });

    // Global control panel
    egui::Window::new("Control").show(ctx, |ui| {
        let label = if paused.0 { "Resume" } else { "Pause" };
        if ui.button(label).clicked() {
            paused.0 = !paused.0;
            let cmd = if paused.0 { r#"{"command":"pause"}"# } else { r#"{"command":"resume"}"# };
            outbox.0.lock().unwrap_or_else(|e| e.into_inner()).push_back(cmd.to_string());
        }
        ui.separator();
        ui.label(format!("Fleet size: {}", params.num_robots));
        ui.label(format!("Random mode: {}", if params.random_mode { "ON" } else { "OFF" }));
        ui.label(format!("Connected: {}", states.0.len()));
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
