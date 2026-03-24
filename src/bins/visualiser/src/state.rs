// src/bins/visualiser/src/state.rs
use bevy::prelude::*;
use std::sync::{Arc, Mutex};
use std::collections::{VecDeque, HashMap};
use gbp_comms::RobotStateMsg;
use gbp_map::map::EdgeId;
use gbp_map::MAX_HORIZON;

/// Toggle for the F2 metrics overlay window.
#[derive(Resource, Default)]
pub struct MetricsVisible(pub bool);

/// Toggle for the F1 world inspector window.
#[derive(Resource, Default)]
pub struct InspectorVisible(pub bool);

/// Maximum entries in the WS inbox queue before dropping oldest.
pub const WS_INBOX_CAP: usize = 64;

/// Bevy resource: queue of incoming messages from WebSocket thread.
#[derive(Resource, Default)]
pub struct WsInbox(pub Arc<Mutex<VecDeque<RobotStateMsg>>>);

/// Bevy resource: queue of outgoing commands to send to simulator via WebSocket.
#[derive(Resource, Default)]
pub struct WsOutbox(pub Arc<Mutex<VecDeque<String>>>);

/// Toggles for which visual elements to render.
#[derive(Resource, Clone, Debug)]
pub struct DrawConfig {
    pub physical_track: bool,
    pub magnetic_mainlines: bool,
    pub magnetic_markers: bool,
    pub node_spheres: bool,
    pub edge_lines: bool,
    pub robots: bool,
    pub planned_paths: bool,
    pub belief_tubes: bool,
    pub factor_links: bool,
    pub uncertainty_bars: bool,
    pub path_traces: bool,
    pub comm_radius_circles: bool,
    pub ir_safety_distance: bool,
    pub robot_colliders: bool,
    pub collision_markers: bool,
    pub infinite_grid: bool,
    /// Minimum safe center-to-center distance (m) for IR factor color gradient.
    /// Mirrors [gbp.interrobot] d_safe from config.toml.
    pub ir_d_safe: f32,
}

impl Default for DrawConfig {
    fn default() -> Self {
        Self {
            physical_track: true,
            magnetic_mainlines: true,
            magnetic_markers: true,
            node_spheres: true,
            edge_lines: true,
            robots: true,
            planned_paths: true,
            belief_tubes: true,
            factor_links: true,
            uncertainty_bars: false,
            path_traces: false,
            comm_radius_circles: false,
            ir_safety_distance: false,
            robot_colliders: false,
            collision_markers: false,
            infinite_grid: false,
            ir_d_safe: 1.3,
        }
    }
}

/// Marker component for node sphere entities (toggled by DrawConfig::node_spheres).
#[derive(Component)]
pub struct NodeSphere;

/// Marker component for the physical track STL mesh entity.
#[derive(Component)]
pub struct PhysicalTrackMesh;

/// Marker component for the magnetic mainlines STL mesh entity.
#[derive(Component)]
pub struct MainlinesMesh;

/// Marker component for the magnetic markers STL mesh entity.
#[derive(Component)]
pub struct MarkersMesh;

/// Per-robot state for visualiser rendering.
#[derive(Clone, Debug)]
pub struct RobotState {
    pub current_edge: EdgeId,
    pub position_s: f32,
    pub velocity: f32,
    pub pos_3d: [f32; 3],
    pub planned_edges: heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
    pub belief_means: [f32; MAX_HORIZON],
    pub belief_vars: [f32; MAX_HORIZON],
    pub active_factor_count: usize,
    /// Which variable timesteps have active IR factors.
    pub active_ir_timesteps: heapless::Vec<u8, MAX_HORIZON>,
    pub raw_gbp_velocity: f32,
    pub min_neighbour_dist_3d: f32,
    /// How many consecutive zero-count updates we've seen (for debounce)
    zero_streak: u8,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            current_edge: EdgeId(0),
            position_s: 0.0,
            velocity: 0.0,
            pos_3d: [0.0; 3],
            planned_edges: heapless::Vec::new(),
            belief_means: [0.0; MAX_HORIZON],
            belief_vars: [0.0; MAX_HORIZON],
            active_factor_count: 0,
            active_ir_timesteps: heapless::Vec::new(),
            raw_gbp_velocity: 0.0,
            min_neighbour_dist_3d: f32::MAX,
            zero_streak: 0,
        }
    }
}

impl RobotState {
    /// Update from a received RobotStateMsg.
    pub fn update_from_msg(&mut self, msg: &RobotStateMsg) {
        self.current_edge = msg.current_edge;
        self.position_s = msg.position_s;
        self.velocity = msg.velocity;
        self.pos_3d = msg.pos_3d;
        self.planned_edges = msg.planned_edges.clone();
        self.belief_means = msg.belief_means;
        self.belief_vars = msg.belief_vars;
        self.active_ir_timesteps = msg.active_ir_timesteps.clone();
        self.raw_gbp_velocity = msg.raw_gbp_velocity;
        self.min_neighbour_dist_3d = msg.min_neighbour_dist_3d;
        // Debounce: only show 0 after 3 consecutive zero-count messages
        let new_count = msg.ir_factor_count as usize;
        if new_count > 0 {
            self.active_factor_count = new_count;
            self.zero_streak = 0;
        } else {
            self.zero_streak = self.zero_streak.saturating_add(1);
            if self.zero_streak >= 3 {
                self.active_factor_count = 0;
            }
        }
    }

}

/// Bevy resource: latest known state per robot, keyed by robot_id.
#[derive(Resource, Default)]
pub struct RobotStates(pub HashMap<u32, RobotState>);

/// Collision event received from simulator.
// Scaffolding — simulator sends these in later M6b tasks
#[derive(Clone, Debug, serde::Deserialize)]
pub struct CollisionEvent {
    pub robot_a: u32,
    pub robot_b: u32,
    pub pos: [f32; 3],
    pub dist: f32,
}

/// Response to a click-to-inspect query.
// Scaffolding — simulator sends these in later M6b tasks
#[derive(Clone, Debug, serde::Deserialize)]
pub struct InspectResponse {
    pub robot_id: u32,
    pub variable_k: usize,
    pub mean: f32,
    pub variance: f32,
    pub factors: Vec<InspectFactorInfo>,
}

#[derive(Clone, Debug, serde::Deserialize)]
pub struct InspectFactorInfo {
    pub kind: String,
    pub msg_eta: f32,
    pub msg_lambda: f32,
}

/// Bevy resource: queue of collision events from simulator.
///
/// # Construction
/// Must be explicitly constructed — do not rely on `Default`.
/// The inner `Arc<Mutex<VecDeque>>` must be shared with the WS client thread,
/// so construction and wiring must happen together.
#[derive(Resource)]
pub struct CollisionInbox(pub Arc<Mutex<VecDeque<CollisionEvent>>>);

impl CollisionInbox {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(VecDeque::new())))
    }
}

/// Bevy resource: queue of inspect responses from simulator.
///
/// # Construction
/// Must be explicitly constructed — do not rely on `Default`.
/// The inner `Arc<Mutex<VecDeque>>` must be shared with the WS client thread,
/// so construction and wiring must happen together.
#[derive(Resource)]
pub struct InspectInbox(pub Arc<Mutex<VecDeque<InspectResponse>>>);

impl InspectInbox {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(VecDeque::new())))
    }
}

/// Maximum number of positions retained per robot in the trace ring buffer.
pub const TRACE_CAP: usize = 10_000;

/// Bevy resource: per-robot ring buffer of recent world-space positions for path trace drawing.
#[derive(Resource, Default)]
pub struct TraceHistory {
    pub traces: HashMap<u32, VecDeque<Vec3>>,
    /// Stores `Time::elapsed_secs()` at the time of the last push for each robot.
    last_seen: HashMap<u32, f32>,
}

impl TraceHistory {
    /// Append a position to the ring buffer for the given robot, evicting the oldest if full.
    /// `elapsed` should be `time.elapsed_secs()` from Bevy's `Time` resource.
    pub fn push(&mut self, robot_id: u32, pos: Vec3, elapsed: f32) {
        let trace = self.traces.entry(robot_id).or_default();
        if trace.len() >= TRACE_CAP {
            trace.pop_front();
        }
        trace.push_back(pos);
        self.last_seen.insert(robot_id, elapsed);
    }

    /// Remove robots whose last update is older than `timeout_secs`.
    /// `now` should be `time.elapsed_secs()` from Bevy's `Time` resource.
    pub fn prune_stale(&mut self, now: f32, timeout_secs: f32) {
        let stale: Vec<u32> = self.last_seen.iter()
            .filter(|(_, &last)| now - last > timeout_secs)
            .map(|(id, _)| *id)
            .collect();
        for id in stale {
            self.traces.remove(&id);
            self.last_seen.remove(&id);
        }
    }
}

/// Live-tunable GBP and simulation parameters.
///
/// Mirrors `GbpConfig` fields from `gbp-core` plus a `timescale` for simulation speed.
/// The settings panel mutates this resource; `detect_param_changes` watches for diffs
/// and pushes `set_params` commands over the WebSocket outbox.
#[derive(Resource, Clone, Debug)]
pub struct LiveParams {
    // -- GBP Solver --
    pub msg_damping: f32,
    pub internal_iters: u8,
    pub external_iters: u8,

    // -- Dynamics factor --
    pub sigma_dynamics: f32,
    pub gbp_timestep: f32,

    // -- Inter-robot factor --
    pub d_safe: f32,
    pub sigma_interrobot: f32,
    pub ir_activation_range: f32,
    pub ir_decay_alpha: f32,
    pub front_damping: f32,

    // -- Velocity bounds (BIPM-inspired) --
    pub v_min: f32,
    pub v_max_default: f32,
    pub vb_kappa: f32,
    pub vb_margin: f32,
    pub vb_max_precision: f32,

    // -- Dynamic constraints (post-GBP) --
    pub max_accel: f32,
    pub max_jerk: f32,
    pub max_speed: f32,

    // -- Graph initialization --
    pub init_variance: f32,
    pub anchor_precision: f32,

    // -- Simulation (not GBP params, but displayed in the settings panel) --
    pub timescale: f32,
}

impl Default for LiveParams {
    fn default() -> Self {
        // Matches GbpConfig::default() in gbp-core/src/config.rs
        Self {
            msg_damping: 0.5,
            internal_iters: 10,
            external_iters: 10,
            sigma_dynamics: 0.5,
            gbp_timestep: 0.1,
            d_safe: 1.3,
            sigma_interrobot: 0.12,
            ir_activation_range: 3.0,
            ir_decay_alpha: 3.0,
            front_damping: 0.3,
            v_min: -0.3,
            v_max_default: 2.5,
            vb_kappa: 10.0,
            vb_margin: 1.0,
            vb_max_precision: 100.0,
            max_accel: 2.5,
            max_jerk: 5.0,
            max_speed: 2.5,
            init_variance: 100.0,
            anchor_precision: 1000.0,
            timescale: 1.0,
        }
    }
}

impl LiveParams {
    /// Build a JSON `set_params` command string for the simulator WebSocket.
    pub fn to_set_params_json(&self) -> String {
        format!(
            concat!(
                r#"{{"command":"set_params","params":{{"#,
                r#""msg_damping":{},"internal_iters":{},"external_iters":{},"#,
                r#""sigma_dynamics":{},"gbp_timestep":{},"#,
                r#""d_safe":{},"sigma_interrobot":{},"ir_activation_range":{},"ir_decay_alpha":{},"front_damping":{},"#,
                r#""v_min":{},"v_max_default":{},"vb_kappa":{},"vb_margin":{},"vb_max_precision":{},"#,
                r#""max_accel":{},"max_jerk":{},"max_speed":{},"#,
                r#""init_variance":{},"anchor_precision":{},"#,
                r#""timescale":{}"#,
                r#"}}}}"#,
            ),
            self.msg_damping, self.internal_iters, self.external_iters,
            self.sigma_dynamics, self.gbp_timestep,
            self.d_safe, self.sigma_interrobot, self.ir_activation_range, self.ir_decay_alpha, self.front_damping,
            self.v_min, self.v_max_default, self.vb_kappa, self.vb_margin, self.vb_max_precision,
            self.max_accel, self.max_jerk, self.max_speed,
            self.init_variance, self.anchor_precision,
            self.timescale,
        )
    }

    /// Returns true if any field differs from `other` (float comparison uses small epsilon).
    pub fn differs_from(&self, other: &Self) -> bool {
        const EPS: f32 = 1e-6;
        let f = |a: f32, b: f32| (a - b).abs() > EPS;
        f(self.msg_damping, other.msg_damping)
            || self.internal_iters != other.internal_iters
            || self.external_iters != other.external_iters
            || f(self.sigma_dynamics, other.sigma_dynamics)
            || f(self.gbp_timestep, other.gbp_timestep)
            || f(self.d_safe, other.d_safe)
            || f(self.sigma_interrobot, other.sigma_interrobot)
            || f(self.ir_activation_range, other.ir_activation_range)
            || f(self.ir_decay_alpha, other.ir_decay_alpha)
            || f(self.front_damping, other.front_damping)
            || f(self.v_min, other.v_min)
            || f(self.v_max_default, other.v_max_default)
            || f(self.vb_kappa, other.vb_kappa)
            || f(self.vb_margin, other.vb_margin)
            || f(self.vb_max_precision, other.vb_max_precision)
            || f(self.max_accel, other.max_accel)
            || f(self.max_jerk, other.max_jerk)
            || f(self.max_speed, other.max_speed)
            || f(self.init_variance, other.init_variance)
            || f(self.anchor_precision, other.anchor_precision)
            || f(self.timescale, other.timescale)
    }
}

/// Bevy resource: parsed map.
#[derive(Resource)]
pub struct MapRes(pub gbp_map::map::Map);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_state_default_has_zero_beliefs() {
        let s = RobotState::default();
        assert_eq!(s.belief_means.len(), MAX_HORIZON);
        assert!(s.belief_means.iter().all(|&v| v == 0.0));
        assert!(s.belief_vars.iter().all(|&v| v == 0.0));
        assert_eq!(s.active_factor_count, 0);
        assert_eq!(s.raw_gbp_velocity, 0.0);
        assert_eq!(s.min_neighbour_dist_3d, f32::MAX);
    }
}

#[cfg(test)]
mod trace_tests {
    use super::*;
    use bevy::math::Vec3;

    #[test]
    fn trace_push_evicts_at_cap() {
        let mut th = TraceHistory::default();
        for i in 0..TRACE_CAP + 5 {
            th.push(0, Vec3::new(i as f32, 0.0, 0.0), i as f32);
        }
        assert_eq!(th.traces[&0].len(), TRACE_CAP);
        // Oldest should have been evicted
        assert_eq!(th.traces[&0].front().unwrap().x, 5.0);
    }

    #[test]
    fn trace_prune_removes_stale() {
        let mut th = TraceHistory::default();
        th.push(0, Vec3::ZERO, 0.0);
        th.push(1, Vec3::ZERO, 0.0);
        // Robot 0 last seen at t=0, robot 1 at t=10
        th.push(1, Vec3::ZERO, 10.0);
        th.prune_stale(11.0, 5.0);
        // Robot 0 should be pruned (11 - 0 = 11 > 5), robot 1 kept (11 - 10 = 1 < 5)
        assert!(!th.traces.contains_key(&0));
        assert!(th.traces.contains_key(&1));
    }
}
