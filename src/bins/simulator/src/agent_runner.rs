//! Wraps RobotAgent and drives its step() loop.
//! The trajectory is a single continuous arc-length from 0 to total_length.
//! Which edge the robot is on is derived from the trajectory, not managed by physics.

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use gbp_agent::RobotAgent;
use gbp_comms::{CommsInterface, ObservationUpdate, RobotBroadcast, RobotStateMsg, RobotSource};
use gbp_map::map::{EdgeId, Map};
use gbp_map::MAX_HORIZON;
use heapless::Vec as HVec;
use crate::physics::PhysicsState;
use crate::sim_comms::SimComms;

pub struct AgentRunner {
    _map: Arc<Map>,
    agent: RobotAgent<SimComms>,
    robot_id: u32,
    /// (EdgeId, edge_length) pairs — the full trajectory
    trajectory_edges: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>,
    /// Total path length (sum of all edge lengths)
    total_length: f32,
    /// Last reported edge (for transition logging)
    last_reported_edge: Option<EdgeId>,
    /// Tick counter for log throttling (per-robot, no shared mutable state)
    pub tick_count: u32,
}

impl AgentRunner {
    pub fn new(comms: SimComms, map: Arc<Map>, robot_id: u32) -> Self {
        let agent = RobotAgent::new(comms, &*map, robot_id);
        Self {
            _map: map,
            agent,
            robot_id,
            trajectory_edges: HVec::new(),
            total_length: 0.0,
            last_reported_edge: None,
            tick_count: 0,
        }
    }

    /// Set trajectory from A* result. Returns total path length.
    pub fn set_trajectory(&mut self, edges: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>) -> f32 {
        self.total_length = edges.iter().map(|(_, len)| len).sum();
        self.trajectory_edges = edges.clone();
        self.last_reported_edge = None;
        self.agent.set_trajectory(edges, 0.0);
        self.total_length
    }

    /// Set a single-edge trajectory for testing.
    pub fn set_single_edge_trajectory(&mut self, edge_id: EdgeId, length: f32) {
        let mut edges = HVec::new();
        let _ = edges.push((edge_id, length));
        self.set_trajectory(edges);
    }

    /// Map global arc-length s to (edge_id, local_s) by walking the trajectory.
    pub fn edge_at_s(&self, global_s: f32) -> (EdgeId, f32) {
        let mut cumulative = 0.0;
        for &(edge_id, length) in &self.trajectory_edges {
            if global_s < cumulative + length {
                return (edge_id, global_s - cumulative);
            }
            cumulative += length;
        }
        // Past the end — return last edge at its end
        if let Some(&(edge_id, length)) = self.trajectory_edges.last() {
            (edge_id, length)
        } else {
            (EdgeId(0), 0.0)
        }
    }

    /// Run one GBP step. Returns velocity and current edge info.
    pub fn step(&mut self, global_s: f32) -> StepOut {
        let (current_edge, local_s) = self.edge_at_s(global_s);

        // Log edge transitions
        if self.last_reported_edge != Some(current_edge) {
            if let Some(prev) = self.last_reported_edge {
                tracing::info!("robot {}: edge transition: {:?} -> {:?} (s={:.2})", self.robot_id, prev, current_edge, global_s);
            }
            self.last_reported_edge = Some(current_edge);
        }

        // Compute 3D position BEFORE step so safety cap can use it
        let pos_3d_map = match self._map.eval_position(current_edge, local_s) {
            Some(p) => p,
            None => [0.0, 0.0, 0.0],
        };
        self.agent.set_pos_3d(pos_3d_map);

        let obs = ObservationUpdate {
            position_s: global_s,
            velocity: 0.0,
            current_edge,
        };
        let out = self.agent.step(obs);

        let pos_3d = [pos_3d_map[0], pos_3d_map[2], -pos_3d_map[1]]; // map -> Bevy coords

        // Extract belief means and variances
        let belief_means = self.agent.belief_means();
        let belief_vars = self.agent.belief_vars();

        StepOut {
            velocity: out.velocity,
            raw_gbp_velocity: out.raw_gbp_velocity,
            min_neighbour_dist_3d: out.min_neighbour_dist_3d,
            current_edge,
            local_s,
            pos_3d,
            active_factor_count: self.agent.interrobot_factor_count(),
            active_ir_timesteps: self.agent.active_ir_timesteps(),
            belief_means,
            belief_vars,
            max_position: self.agent.last_max_position(),
            belief_spread: out.belief_spread,
        }
    }

    /// Broadcast this robot's current state so other robots' SimComms can receive it.
    pub fn broadcast_state(&mut self, current_edge: EdgeId, position_s: f32, pos_3d: [f32; 3]) {
        let msg = RobotBroadcast {
            robot_id: self.robot_id,
            current_edge,
            position_s,
            pos: pos_3d,
            planned_edges: self.planned_edges_snapshot(),
            ..Default::default()
        };
        let _ = self.agent.comms_mut().broadcast(&msg);
    }

    /// Edge IDs for the planned path (for visualiser dashed gizmo).
    pub fn planned_edge_ids(&self) -> HVec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
        let mut ids = HVec::new();
        for &(eid, _) in &self.trajectory_edges {
            let _ = ids.push(eid);
        }
        ids
    }

    /// Snapshot of planned edges for broadcast (using MAX_HORIZON capacity).
    fn planned_edges_snapshot(&self) -> HVec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
        let mut pe = HVec::new();
        for &(eid, _) in self.trajectory_edges.iter() {
            let _ = pe.push(eid);
        }
        pe
    }
}

pub struct StepOut {
    pub velocity: f32,
    pub raw_gbp_velocity: f32,
    pub min_neighbour_dist_3d: f32,
    pub current_edge: EdgeId,
    pub local_s: f32,
    pub pos_3d: [f32; 3],
    pub active_factor_count: usize,
    pub active_ir_timesteps: HVec<u8, MAX_HORIZON>,
    pub belief_means: [f32; MAX_HORIZON],
    pub belief_vars: [f32; MAX_HORIZON],
    pub max_position: f32,
    pub belief_spread: f32,
}

/// Runs at 50 Hz: reads global position, steps agent, writes velocity back.
pub async fn agent_task(
    physics: Arc<Mutex<PhysicsState>>,
    runner: Arc<Mutex<AgentRunner>>,
    tx: broadcast::Sender<RobotStateMsg>,
    robot_id: u32,
) {
    let mut ticker = interval(Duration::from_millis(20)); // 50 Hz (matches physics)
    loop {
        ticker.tick().await;

        let global_s = physics.lock().unwrap_or_else(|e| e.into_inner()).position_s;

        let out = {
            let mut r = runner.lock().unwrap_or_else(|e| e.into_inner());
            let out = r.step(global_s);
            // Only log every ~1s (every 50th tick at 50Hz) to reduce spam
            r.tick_count += 1;
            if r.tick_count % 50 == 0 && out.active_factor_count > 0 {
                tracing::info!(
                    "R{}: s={:.2} gbp_v={:.2} cmd_v={:.2} dist3d={:.2} ir={} spread={:.2}",
                    robot_id, global_s, out.raw_gbp_velocity, out.velocity,
                    out.min_neighbour_dist_3d, out.active_factor_count,
                    out.belief_spread,
                );
            }
            // Log belief oscillation: if spread > 5m, beliefs are flipping wildly
            if out.belief_spread > 5.0 && r.tick_count % 25 == 0 {
                tracing::warn!(
                    "R{} OSCILLATION: spread={:.1}m beliefs=[{:.1},{:.1},{:.1},...,{:.1},{:.1}] s={:.2}",
                    robot_id, out.belief_spread,
                    out.belief_means[0], out.belief_means[1], out.belief_means[2],
                    out.belief_means[10], out.belief_means[11],
                    global_s,
                );
            }
            // NOTE: step() already broadcasts full RobotBroadcast with belief_means/vars
            // via make_broadcast(). Do NOT call broadcast_state() here — it sends a partial
            // message with default beliefs, overwriting the full one.
            out
        };

        {
            let mut p = physics.lock().unwrap_or_else(|e| e.into_inner());
            p.velocity = out.velocity;
        }

        let planned_edges = runner.lock().unwrap_or_else(|e| e.into_inner()).planned_edge_ids();

        let msg = RobotStateMsg {
            robot_id,
            current_edge: out.current_edge,
            position_s: out.local_s,
            velocity: out.velocity,
            pos_3d: out.pos_3d,
            source: RobotSource::Simulated,
            belief_means: out.belief_means,
            belief_vars: out.belief_vars,
            planned_edges,
            ir_factor_count: out.active_factor_count as u16,
            active_factors: {
                let mut af = HVec::new();
                if out.active_factor_count > 0 {
                    let other = if robot_id == 0 { 1 } else { 0 };
                    let _ = af.push(other);
                }
                af
            },
            active_ir_timesteps: out.active_ir_timesteps,
            raw_gbp_velocity: out.raw_gbp_velocity,
            min_neighbour_dist_3d: out.min_neighbour_dist_3d,
        };
        let _ = tx.send(msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

    fn straight_map() -> Map {
        let mut m = Map::new("straight");
        m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(1), position: [5.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
        let sf = SafetyProfile { clearance: 0.3 };
        m.add_edge(Edge {
            id: EdgeId(0), start: NodeId(0), end: NodeId(1),
            geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [5.0,0.0,0.0], length: 5.0 },
            speed: sp, safety: sf,
        }).unwrap();
        m
    }

    fn two_edge_map() -> (Arc<Map>, HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>) {
        let mut m = Map::new("two");
        m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(1), position: [5.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(2), position: [10.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
        let sf = SafetyProfile { clearance: 0.3 };
        m.add_edge(Edge {
            id: EdgeId(0), start: NodeId(0), end: NodeId(1),
            geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [5.0,0.0,0.0], length: 5.0 },
            speed: sp.clone(), safety: sf.clone(),
        }).unwrap();
        m.add_edge(Edge {
            id: EdgeId(1), start: NodeId(1), end: NodeId(2),
            geometry: EdgeGeometry::Line { start: [5.0,0.0,0.0], end: [10.0,0.0,0.0], length: 5.0 },
            speed: sp, safety: sf,
        }).unwrap();
        let mut traj = HVec::new();
        traj.push((EdgeId(0), 5.0)).unwrap();
        traj.push((EdgeId(1), 5.0)).unwrap();
        (Arc::new(m), traj)
    }

    fn make_comms() -> SimComms {
        let (tx, rx) = tokio::sync::broadcast::channel::<RobotBroadcast>(64);
        SimComms::new(tx, rx)
    }

    #[test]
    fn edge_at_s_first_edge() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(make_comms(), map, 0);
        runner.set_trajectory(traj);
        let (edge, local) = runner.edge_at_s(2.5);
        assert_eq!(edge, EdgeId(0));
        assert!((local - 2.5).abs() < 1e-4);
    }

    #[test]
    fn edge_at_s_second_edge() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(make_comms(), map, 0);
        runner.set_trajectory(traj);
        let (edge, local) = runner.edge_at_s(7.0);
        assert_eq!(edge, EdgeId(1));
        assert!((local - 2.0).abs() < 1e-4); // 7.0 - 5.0 = 2.0
    }

    #[test]
    fn edge_at_s_past_end() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(make_comms(), map, 0);
        runner.set_trajectory(traj);
        let (edge, local) = runner.edge_at_s(12.0);
        assert_eq!(edge, EdgeId(1));
        assert!((local - 5.0).abs() < 1e-4); // clamped to last edge end
    }

    #[test]
    fn step_returns_positive_velocity() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(make_comms(), map, 0);
        runner.set_trajectory(traj);
        let out = runner.step(0.0);
        assert!(out.velocity > 0.0, "v={}", out.velocity);
    }

    #[test]
    fn total_length_is_sum_of_edges() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(make_comms(), map, 0);
        let total = runner.set_trajectory(traj);
        assert!((total - 10.0).abs() < 1e-4);
    }

    #[test]
    fn update_interrobot_factors_spawns_factor_when_same_edge() {
        let map = std::sync::Arc::new(straight_map());
        let (tx, rx0) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
        let rx1 = tx.subscribe();
        let comms0 = SimComms::new(tx.clone(), rx0);
        let comms1 = SimComms::new(tx.clone(), rx1);
        let mut runner0 = AgentRunner::new(comms0, map.clone(), 0);
        runner0.set_single_edge_trajectory(EdgeId(0), 5.0);
        let mut runner1 = AgentRunner::new(comms1, map.clone(), 1);
        runner1.set_single_edge_trajectory(EdgeId(0), 5.0);

        // Robot 1 broadcasts its state so robot 0 can see it (within IR_ACTIVATION_RANGE=3.0m)
        runner1.broadcast_state(EdgeId(0), 2.5, [2.5, 0.0, 0.0]);

        // Robot 0 steps — should spawn InterRobotFactor because they share EdgeId(0)
        let out = runner0.step(0.5);
        assert!(out.active_factor_count > 0, "expected inter-robot factor to be spawned");
    }

    #[test]
    fn iterate_runs_without_panic_with_two_robots() {
        let map = std::sync::Arc::new(straight_map());
        let (tx, rx0) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
        let rx1 = tx.subscribe();
        let comms0 = SimComms::new(tx.clone(), rx0);
        let comms1 = SimComms::new(tx.clone(), rx1);
        let mut runner0 = AgentRunner::new(comms0, map.clone(), 0);
        runner0.set_single_edge_trajectory(EdgeId(0), 5.0);
        let mut runner1 = AgentRunner::new(comms1, map.clone(), 1);
        runner1.set_single_edge_trajectory(EdgeId(0), 5.0);
        runner1.broadcast_state(EdgeId(0), 3.5, [3.5, 0.0, 0.0]);
        // Steps with GBP iterate — should not panic
        for s in [0.5_f32, 1.0, 1.5, 2.0] {
            let _ = runner0.step(s);
        }
    }
}
