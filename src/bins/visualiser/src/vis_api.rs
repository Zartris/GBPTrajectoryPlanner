// src/bins/visualiser/src/vis_api.rs
//
// VisApi — a Bevy SystemParam that bundles access to the main visualiser
// resources into a single, ergonomic handle for addon authors.
//
// Usage in an addon system:
//
//   fn my_system(mut api: VisApi, time: Res<Time>) {
//       if something { api.screenshot("/tmp/out.png"); }
//   }
//
// Only resources that exist on the main branch are included here.
// CameraState (planned for M6a) is NOT included yet.

use bevy::prelude::*;
// SystemParam *derive macro* is not re-exported from bevy::prelude — import explicitly.
use bevy::ecs::system::SystemParam;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
// In Bevy 0.18 AppExit is sent via MessageWriter (EventWriter was removed).
use bevy::ecs::message::MessageWriter;
use crate::state::{DrawConfig, RobotStates};
use crate::ui::SimPaused;

/// A Bevy [`SystemParam`] that provides a clean, stable API surface for
/// visualiser addons.  Addon authors add `mut api: VisApi` to their system
/// parameters instead of reaching directly into individual resources.
// Fields are accessed through the public methods below; suppress the dead_code
// lint that fires because the derive macro generates the access glue invisibly.
#[allow(dead_code)]
#[derive(SystemParam)]
pub struct VisApi<'w, 's> {
    commands: Commands<'w, 's>,
    draw: ResMut<'w, DrawConfig>,
    paused: ResMut<'w, SimPaused>,
    robot_states: Res<'w, RobotStates>,
    time: Res<'w, Time>,
    app_exit: MessageWriter<'w, AppExit>,
}

// All methods on VisApi are public API for addons; suppress dead_code lint.
#[allow(dead_code)]
impl<'w, 's> VisApi<'w, 's> {
    // -------------------------------------------------------------------------
    // Screenshot
    // -------------------------------------------------------------------------

    /// Capture a screenshot of the primary window and save it to `path`.
    ///
    /// The capture happens asynchronously on the next render frame; the file
    /// will appear on disk shortly after this call returns.
    pub fn screenshot(&mut self, path: impl Into<String>) {
        let path_string = path.into();
        self.commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path_string));
    }

    // -------------------------------------------------------------------------
    // Logging
    // -------------------------------------------------------------------------

    /// Emit an `info`-level log message tagged with `[addon]`.
    pub fn log(&self, msg: &str) {
        tracing::info!("[addon] {}", msg);
    }

    // -------------------------------------------------------------------------
    // DrawConfig toggles
    // -------------------------------------------------------------------------

    /// Set a named draw-config toggle.
    ///
    /// Recognised field names mirror the fields of [`DrawConfig`]:
    /// `"physical_track"`, `"magnetic_mainlines"`, `"magnetic_markers"`,
    /// `"node_spheres"`, `"edge_lines"`, `"robots"`, `"planned_paths"`,
    /// `"belief_tubes"`, `"factor_links"`.
    ///
    /// Unknown names are silently ignored (logged at warn level).
    pub fn set_draw(&mut self, field: &str, on: bool) {
        match field {
            "physical_track"     => self.draw.physical_track     = on,
            "magnetic_mainlines" => self.draw.magnetic_mainlines = on,
            "magnetic_markers"   => self.draw.magnetic_markers   = on,
            "node_spheres"       => self.draw.node_spheres       = on,
            "edge_lines"         => self.draw.edge_lines         = on,
            "robots"             => self.draw.robots             = on,
            "planned_paths"      => self.draw.planned_paths      = on,
            "belief_tubes"       => self.draw.belief_tubes       = on,
            "factor_links"       => self.draw.factor_links       = on,
            other => tracing::warn!("[addon] set_draw: unknown field '{}'", other),
        }
    }

    /// Enable or disable **all** draw-config toggles at once.
    ///
    /// Uses a struct literal so the compiler will catch any newly-added
    /// [`DrawConfig`] fields that are not yet handled here.
    pub fn set_draw_all(&mut self, on: bool) {
        *self.draw = DrawConfig {
            physical_track:     on,
            magnetic_mainlines: on,
            magnetic_markers:   on,
            node_spheres:       on,
            edge_lines:         on,
            robots:             on,
            planned_paths:      on,
            belief_tubes:       on,
            factor_links:       on,
        };
    }

    // -------------------------------------------------------------------------
    // Simulation state
    // -------------------------------------------------------------------------

    /// Returns `true` if the simulator is currently paused.
    pub fn is_paused(&self) -> bool {
        self.paused.0
    }

    /// Pause the simulation (no-op if already paused).
    pub fn pause(&mut self) {
        self.paused.0 = true;
    }

    /// Resume the simulation (no-op if already running).
    pub fn resume(&mut self) {
        self.paused.0 = false;
    }

    // -------------------------------------------------------------------------
    // Time
    // -------------------------------------------------------------------------

    /// Returns seconds elapsed since the Bevy app started.
    pub fn elapsed_secs(&self) -> f32 {
        self.time.elapsed_secs()
    }

    // -------------------------------------------------------------------------
    // Robot state queries
    // -------------------------------------------------------------------------

    /// Returns the number of robots currently tracked by the visualiser.
    pub fn robot_count(&self) -> usize {
        self.robot_states.0.len()
    }

    // -------------------------------------------------------------------------
    // App lifecycle
    // -------------------------------------------------------------------------

    /// Request a clean application exit.
    pub fn quit(&mut self) {
        self.app_exit.write(AppExit::Success);
    }
}

#[cfg(test)]
mod tests {
    // Unit tests for VisApi helpers that don't require a running Bevy app.
    // Integration tests (screenshot, quit) require a real App and are covered
    // by the addon integration tests in `addons/`.

    #[test]
    fn placeholder_vis_api_module_compiles() {
        // If this test runs, the module compiled successfully.
    }
}
