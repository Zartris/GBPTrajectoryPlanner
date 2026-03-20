// src/bins/visualiser/src/ui.rs
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
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
           .add_systems(Update, draw_hud);
    }
}

pub fn fmt_velocity(v: f32) -> String {
    format!("{:.2} m/s", v)
}

fn draw_hud(
    mut ctxs: EguiContexts,
    states: Res<RobotStates>,
    mut paused: ResMut<SimPaused>,
) {
    let Ok(ctx) = ctxs.ctx_mut() else { return; };
    egui::Window::new("Control").show(ctx, |ui| {
        let label = if paused.0 { "Resume" } else { "Pause" };
        if ui.button(label).clicked() {
            paused.0 = !paused.0;
        }
        ui.separator();
        for (id, state) in &states.0 {
            ui.label(format!("Robot {}: {}", id, fmt_velocity(state.velocity)));
        }
        if states.0.is_empty() {
            ui.label("No robots connected");
        }
    });
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
}
