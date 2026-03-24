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
#[derive(Clone, Debug, serde::Deserialize)]
pub struct CollisionEvent {
    pub robot_a: u32,
    pub robot_b: u32,
    pub pos: [f32; 3],
    pub dist: f32,
}

/// Response to a click-to-inspect query.
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
#[derive(Resource, Default)]
pub struct CollisionInbox(pub Arc<Mutex<VecDeque<CollisionEvent>>>);

/// Bevy resource: queue of inspect responses from simulator.
#[derive(Resource, Default)]
pub struct InspectInbox(pub Arc<Mutex<VecDeque<InspectResponse>>>);

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
