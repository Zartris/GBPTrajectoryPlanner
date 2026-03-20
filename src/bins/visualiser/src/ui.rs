// src/bins/visualiser/src/ui.rs
use bevy::prelude::*;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::state::RobotStates;

/// Shared pause flag.
#[derive(Resource, Default)]
pub struct SimPaused(pub bool);

/// Tracks the last WS message timestamp for backend Hz calculation.
#[derive(Resource)]
pub struct BackendStats {
    pub last_msg_time: std::time::Instant,
    pub msg_hz: f32,
    /// Exponential moving average smoothing
    pub ema_alpha: f32,
}

impl Default for BackendStats {
    fn default() -> Self {
        Self {
            last_msg_time: std::time::Instant::now(),
            msg_hz: 0.0,
            ema_alpha: 0.1,
        }
    }
}

impl BackendStats {
    pub fn record_message(&mut self) {
        let now = std::time::Instant::now();
        let dt = now.duration_since(self.last_msg_time).as_secs_f32();
        if dt > 0.001 {
            let instant_hz = 1.0 / dt;
            self.msg_hz = self.msg_hz * (1.0 - self.ema_alpha) + instant_hz * self.ema_alpha;
        }
        self.last_msg_time = now;
    }
}

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
           .init_resource::<SimPaused>()
           .init_resource::<BackendStats>()
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
        }
    });

    // Per-robot side panels
    let mut sorted_ids: Vec<u32> = states.0.keys().copied().collect();
    sorted_ids.sort();
    for id in sorted_ids {
        if let Some(state) = states.0.get(&id) {
            egui::Window::new(format!("Robot {}", id)).show(ctx, |ui| {
                ui.label(format!("Speed:   {}", fmt_velocity(state.velocity)));
                ui.label(format!("Edge:    {:?} s={:.2}", state.current_edge, state.position_s));
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
