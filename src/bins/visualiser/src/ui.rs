// src/bins/visualiser/src/ui.rs
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::state::RobotStates;

/// Shared pause flag. The visualiser sets this; simulator reads it via the
/// WebSocket control channel (M6 feature -- button visible in M2 but actual
/// simulation pause is deferred to M6).
#[derive(Resource, Default)]
pub struct SimPaused(pub bool);

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimPaused>()
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
) -> Result {
    let ctx = ctxs.ctx_mut()?;

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
                ui.label(format!("sigma_dyn: {:.4}", state.sigma_dyn));
                ui.label(format!("sigma_r:   {:.4}", state.sigma_r));
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
