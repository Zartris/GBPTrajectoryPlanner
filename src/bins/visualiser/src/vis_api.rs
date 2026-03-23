// src/bins/visualiser/src/vis_api.rs
//
//! `VisApi` -- the primary API surface for visualiser addons and AI agents.
//!
//! # Overview
//!
//! `VisApi` is a Bevy [`SystemParam`] that bundles access to the main
//! visualiser resources into a single, ergonomic handle.  Addon authors add
//! `mut api: VisApi` to their system parameters instead of reaching directly
//! into individual resources.
//!
//! # Two patterns for addon systems
//!
//! ## Polling pattern (simplest)
//!
//! Query robot state every frame using `VisApi` methods:
//!
//! ```rust,ignore
//! fn my_polling_addon(mut api: VisApi) {
//!     for id in api.robot_ids() {
//!         if let Some(dist) = api.robot_min_distance(id) {
//!             if dist < 1.0 {
//!                 api.log(&format!("robot {} very close: {:.2}m", id, dist));
//!             }
//!         }
//!     }
//! }
//! ```
//!
//! ## Event-driven pattern (recommended for reactive logic)
//!
//! React to pre-computed events from the event bus via
//! [`MessageReader<T>`](bevy::ecs::message::MessageReader):
//!
//! ```rust,ignore
//! use bevy::ecs::message::MessageReader;
//! use crate::vis_events::{ProximityAlert, RobotStateChanged};
//!
//! fn my_event_addon(
//!     mut api: VisApi,
//!     mut proximity: MessageReader<ProximityAlert>,
//!     mut changes: MessageReader<RobotStateChanged>,
//! ) {
//!     for alert in proximity.read() {
//!         api.log(&format!(
//!             "robots {} & {} dist={:.2}m",
//!             alert.robot_a, alert.robot_b, alert.distance
//!         ));
//!         api.screenshot("/tmp/proximity.png");
//!     }
//!     for change in changes.read() {
//!         api.log(&format!("robot {} — {:?}", change.robot_id, change.event_type));
//!     }
//! }
//! ```
//!
//! # Available events
//!
//! See [`vis_events`](crate::vis_events) for the full list:
//!
//! - [`ProximityAlert`](crate::vis_events::ProximityAlert) — two robots closer
//!   than d_safe
//! - [`SimTickEvent`](crate::vis_events::SimTickEvent) — per-frame summary
//! - [`RobotStateChanged`](crate::vis_events::RobotStateChanged) — connect,
//!   disconnect, edge change, factor count change, near-collision
//!
//! # API reference
//!
//! | Category | Method | Description |
//! |----------|--------|-------------|
//! | Screenshot | [`screenshot`] | Capture window to PNG |
//! | Logging | [`log`] | Info-level log with `[addon]` tag |
//! | Draw config | [`set_draw`], [`set_draw_all`], [`ir_d_safe`] | Toggle visual layers |
//! | Simulation | [`is_paused`], [`pause`], [`resume`] | Local pause flag |
//! | Time | [`elapsed_secs`] | Wall-clock seconds since app start |
//! | Camera | [`set_camera_orbit`], [`reset_camera`], [`follow_robot`], [`exit_follow`], [`camera_position`] | Camera control |
//! | Robot state | [`robot_count`], [`robot_ids`], [`robot_position`], [`robot_velocity`], [`robot_edge`], [`robot_factor_count`], [`robot_min_distance`] | Query individual robots |
//! | Map data | [`map_node_count`], [`map_edge_count`], [`map_id`] | Read map structure |
//! | WebSocket | [`send_sim_command`], [`pause_sim`], [`resume_sim`] | Send commands to simulator |
//! | App lifecycle | [`quit`] | Clean exit |

use bevy::prelude::*;
// SystemParam *derive macro* is not re-exported from bevy::prelude — import explicitly.
use bevy::ecs::system::SystemParam;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
// In Bevy 0.18 AppExit is sent via MessageWriter (EventWriter was removed).
use bevy::ecs::message::MessageWriter;
use crate::camera::{CameraMode, CameraState};
use crate::state::{DrawConfig, MapRes, RobotStates, WsOutbox};
use crate::ui::SimPaused;
use gbp_map::map::EdgeId;

/// A Bevy [`SystemParam`] that provides a clean, stable API surface for
/// visualiser addons and AI agents.
///
/// Addon authors add `mut api: VisApi` to their system parameters instead of
/// reaching directly into individual resources. This decouples addons from the
/// internal resource layout and lets us evolve internals without breaking
/// addon code.
///
/// See the [module documentation](self) for usage examples and the full API
/// reference table.
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
    camera: ResMut<'w, CameraState>,
    ws_outbox: Res<'w, WsOutbox>,
    map: Res<'w, MapRes>,
}

// All methods on VisApi are public API for addons; suppress dead_code lint.
#[allow(dead_code)]
impl<'w, 's> VisApi<'w, 's> {
    // =========================================================================
    // Screenshot
    // =========================================================================

    /// Capture a screenshot of the primary window and save it to `path`.
    ///
    /// The capture happens asynchronously on the next render frame; the file
    /// will appear on disk shortly after this call returns.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// api.screenshot("/tmp/my_screenshot.png");
    /// ```
    pub fn screenshot(&mut self, path: impl Into<String>) {
        let path_string = path.into();
        self.commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path_string));
    }

    // =========================================================================
    // Logging
    // =========================================================================

    /// Emit an `info`-level log message tagged with `[addon]`.
    ///
    /// All addon log output goes through this method so it can be filtered
    /// with `RUST_LOG=visualiser=info` or searched for `[addon]`.
    pub fn log(&self, msg: &str) {
        tracing::info!("[addon] {}", msg);
    }

    // =========================================================================
    // DrawConfig toggles
    // =========================================================================

    /// Set a named draw-config toggle.
    ///
    /// Recognised field names mirror the fields of [`DrawConfig`]:
    /// `"physical_track"`, `"magnetic_mainlines"`, `"magnetic_markers"`,
    /// `"node_spheres"`, `"edge_lines"`, `"robots"`, `"planned_paths"`,
    /// `"belief_tubes"`, `"factor_links"`, `"uncertainty_bars"`,
    /// `"path_traces"`, `"comm_radius_circles"`, `"ir_safety_distance"`,
    /// `"robot_colliders"`, `"collision_markers"`, `"infinite_grid"`.
    ///
    /// Unknown names log a warning via `tracing::warn!` and are otherwise ignored.
    pub fn set_draw(&mut self, field: &str, on: bool) {
        match field {
            "physical_track"       => self.draw.physical_track       = on,
            "magnetic_mainlines"   => self.draw.magnetic_mainlines   = on,
            "magnetic_markers"     => self.draw.magnetic_markers     = on,
            "node_spheres"         => self.draw.node_spheres         = on,
            "edge_lines"           => self.draw.edge_lines           = on,
            "robots"               => self.draw.robots               = on,
            "planned_paths"        => self.draw.planned_paths        = on,
            "belief_tubes"         => self.draw.belief_tubes         = on,
            "factor_links"         => self.draw.factor_links         = on,
            "uncertainty_bars"     => self.draw.uncertainty_bars     = on,
            "path_traces"          => self.draw.path_traces          = on,
            "comm_radius_circles"  => self.draw.comm_radius_circles  = on,
            "ir_safety_distance"   => self.draw.ir_safety_distance   = on,
            "robot_colliders"      => self.draw.robot_colliders      = on,
            "collision_markers"    => self.draw.collision_markers    = on,
            "infinite_grid"        => self.draw.infinite_grid        = on,
            other => tracing::warn!("[addon] set_draw: unknown field '{}'", other),
        }
    }

    /// Enable or disable **all** draw-config toggles at once.
    ///
    /// Uses a struct literal so the compiler will catch any newly-added
    /// [`DrawConfig`] fields that are not yet handled here.
    /// Note: `ir_d_safe` is a numeric field, not a toggle — it retains its
    /// current value when this method is called.
    pub fn set_draw_all(&mut self, on: bool) {
        *self.draw = DrawConfig {
            physical_track:      on,
            magnetic_mainlines:  on,
            magnetic_markers:    on,
            node_spheres:        on,
            edge_lines:          on,
            robots:              on,
            planned_paths:       on,
            belief_tubes:        on,
            factor_links:        on,
            uncertainty_bars:    on,
            path_traces:         on,
            comm_radius_circles: on,
            ir_safety_distance:  on,
            robot_colliders:     on,
            collision_markers:   on,
            infinite_grid:       on,
            ir_d_safe:           self.draw.ir_d_safe,
        };
    }

    /// Returns the current inter-robot safety distance (metres).
    ///
    /// This value is used by the proximity alert system and the IR factor
    /// color gradient. Mirrors `[gbp.interrobot] d_safe` from config.toml.
    pub fn ir_d_safe(&self) -> f32 {
        self.draw.ir_d_safe
    }

    // =========================================================================
    // Simulation state
    // =========================================================================

    /// Returns `true` if the simulator is currently paused (local flag).
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

    // =========================================================================
    // Time
    // =========================================================================

    /// Returns seconds elapsed since the Bevy app started.
    pub fn elapsed_secs(&self) -> f32 {
        self.time.elapsed_secs()
    }

    // =========================================================================
    // Camera control
    // =========================================================================

    /// Set the camera to orbit mode with the given yaw (radians), pitch (radians),
    /// and distance (metres) from the current focus point.
    pub fn set_camera_orbit(&mut self, yaw: f32, pitch: f32, distance: f32) {
        self.camera.mode = CameraMode::Orbit;
        self.camera.yaw = yaw;
        self.camera.pitch = pitch;
        self.camera.distance = distance;
    }

    /// Reset the camera to its initial position (as set when the map was loaded).
    pub fn reset_camera(&mut self) {
        self.camera.reset();
    }

    /// Enter follow mode tracking the robot with the given ID.
    /// If no robot with that ID exists the camera will fall back to orbit mode
    /// on the next update tick.
    pub fn follow_robot(&mut self, robot_id: u32) {
        self.camera.mode = CameraMode::Follow(robot_id);
    }

    /// Exit follow mode and return to free orbit.
    pub fn exit_follow(&mut self) {
        self.camera.mode = CameraMode::Orbit;
    }

    /// Returns the current world-space position of the camera `[x, y, z]`.
    ///
    /// Useful for addons that need to know what the camera is looking at
    /// (e.g., to position UI elements or compute visibility).
    pub fn camera_position(&self) -> [f32; 3] {
        let pos = self.camera.world_position();
        [pos.x, pos.y, pos.z]
    }

    // =========================================================================
    // Robot state queries
    // =========================================================================

    /// Returns the number of robots currently tracked by the visualiser.
    pub fn robot_count(&self) -> usize {
        self.robot_states.0.len()
    }

    /// Returns a sorted list of all robot IDs currently tracked.
    pub fn robot_ids(&self) -> Vec<u32> {
        let mut ids: Vec<u32> = self.robot_states.0.keys().copied().collect();
        ids.sort_unstable();
        ids
    }

    /// Returns the latest 3-D world position `[x, y, z]` for the given robot,
    /// or `None` if the robot is not known.
    pub fn robot_position(&self, id: u32) -> Option<[f32; 3]> {
        self.robot_states.0.get(&id).map(|s| s.pos_3d)
    }

    /// Returns the latest scalar velocity (m/s along edge) for the given robot,
    /// or `None` if the robot is not known.
    pub fn robot_velocity(&self, id: u32) -> Option<f32> {
        self.robot_states.0.get(&id).map(|s| s.velocity)
    }

    /// Returns the current `(EdgeId, arc-length s)` for the given robot,
    /// or `None` if the robot is not known.
    pub fn robot_edge(&self, id: u32) -> Option<(EdgeId, f32)> {
        self.robot_states.0.get(&id).map(|s| (s.current_edge, s.position_s))
    }

    /// Returns the number of active inter-robot factors for the given robot,
    /// or `None` if the robot is not known.
    pub fn robot_factor_count(&self, id: u32) -> Option<usize> {
        self.robot_states.0.get(&id).map(|s| s.active_factor_count)
    }

    /// Returns the minimum 3-D distance to the nearest neighbour for the given
    /// robot, or `None` if the robot is not known.
    pub fn robot_min_distance(&self, id: u32) -> Option<f32> {
        self.robot_states.0.get(&id).map(|s| s.min_neighbour_dist_3d)
    }

    /// Returns the raw GBP-computed velocity for the given robot (before
    /// clamping/smoothing), or `None` if the robot is not known.
    pub fn robot_raw_gbp_velocity(&self, id: u32) -> Option<f32> {
        self.robot_states.0.get(&id).map(|s| s.raw_gbp_velocity)
    }

    // =========================================================================
    // Map data
    // =========================================================================

    /// Returns the number of nodes in the loaded map.
    pub fn map_node_count(&self) -> usize {
        self.map.0.nodes.len()
    }

    /// Returns the number of edges in the loaded map.
    pub fn map_edge_count(&self) -> usize {
        self.map.0.edges.len()
    }

    /// Returns the map identifier string (e.g., `"test_loop_map"`).
    pub fn map_id(&self) -> &str {
        self.map.0.map_id.as_str()
    }

    // =========================================================================
    // WebSocket command sending
    // =========================================================================

    /// Enqueue a raw JSON string to be sent to the simulator via WebSocket.
    ///
    /// The message is placed in the `WsOutbox` and will be sent on the next
    /// WebSocket tick (typically within 100 ms).
    pub fn send_sim_command(&mut self, json: &str) {
        let mut q = self.ws_outbox.0.lock().unwrap_or_else(|e| e.into_inner());
        q.push_back(json.to_owned());
    }

    /// Send a `{"cmd":"pause"}` command to the simulator.
    pub fn pause_sim(&mut self) {
        self.send_sim_command(r#"{"cmd":"pause"}"#);
    }

    /// Send a `{"cmd":"resume"}` command to the simulator.
    pub fn resume_sim(&mut self) {
        self.send_sim_command(r#"{"cmd":"resume"}"#);
    }

    // =========================================================================
    // App lifecycle
    // =========================================================================

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
