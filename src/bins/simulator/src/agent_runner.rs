//! Wraps RobotAgent and drives its step() loop.
//! The trajectory is a single continuous arc-length from 0 to total_length.
//! Which edge the robot is on is derived from the trajectory, not managed by physics.

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use gbp_agent::RobotAgent;
use gbp_comms::{ObservationUpdate, RobotStateMsg, RobotSource};
use gbp_map::map::{EdgeId, Map};
use gbp_map::MAX_HORIZON;
use heapless::Vec as HVec;
use crate::physics::PhysicsState;
use crate::sim_comms::SimComms;

pub struct AgentRunner {
    _map: Arc<Map>,
    agent: RobotAgent<SimComms>,
    /// (EdgeId, edge_length) pairs — the full trajectory
    trajectory_edges: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>,
    /// Total path length (sum of all edge lengths)
    total_length: f32,
    /// Last reported edge (for transition logging)
    last_reported_edge: Option<EdgeId>,
}

impl AgentRunner {
    pub fn new(comms: SimComms, map: Arc<Map>, robot_id: u32) -> Self {
        let agent = RobotAgent::new(comms, &*map, robot_id);
        Self {
            _map: map,
            agent,
            trajectory_edges: HVec::new(),
            total_length: 0.0,
            last_reported_edge: None,
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
                tracing::info!("edge transition: {:?} -> {:?} (s={:.2})", prev, current_edge, global_s);
            }
            self.last_reported_edge = Some(current_edge);
        }

        let obs = ObservationUpdate {
            position_s: global_s,
            velocity: 0.0,
            current_edge,
        };
        let out = self.agent.step(obs);

        // 3D position from the map
        let pos_3d = match self._map.eval_position(current_edge, local_s) {
            Some(p) => [p[0], p[2], -p[1]],
            None => [0.0, 0.0, 0.0],
        };

        StepOut {
            velocity: out.velocity,
            current_edge,
            local_s,
            pos_3d,
        }
    }

    /// Edge IDs for the planned path (for visualiser dashed gizmo).
    pub fn planned_edge_ids(&self) -> HVec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
        let mut ids = HVec::new();
        for &(eid, _) in &self.trajectory_edges {
            let _ = ids.push(eid);
        }
        ids
    }
}

pub struct StepOut {
    pub velocity: f32,
    pub current_edge: EdgeId,
    pub local_s: f32,
    pub pos_3d: [f32; 3],
}

/// Runs at 20 Hz: reads global position, steps agent, writes velocity back.
pub async fn agent_task(
    physics: Arc<Mutex<PhysicsState>>,
    runner: Arc<Mutex<AgentRunner>>,
    tx: broadcast::Sender<RobotStateMsg>,
) {
    let mut ticker = interval(Duration::from_millis(50)); // 20 Hz
    loop {
        ticker.tick().await;

        let global_s = physics.lock().unwrap_or_else(|e| e.into_inner()).position_s;

        let out = runner.lock().unwrap_or_else(|e| e.into_inner()).step(global_s);

        physics.lock().unwrap_or_else(|e| e.into_inner()).velocity = out.velocity;

        let planned_edges = runner.lock().unwrap_or_else(|e| e.into_inner()).planned_edge_ids();

        let msg = RobotStateMsg {
            robot_id: 0,
            current_edge: out.current_edge,
            position_s: out.local_s,
            velocity: out.velocity,
            pos_3d: out.pos_3d,
            source: RobotSource::Simulated,
            belief_means: [0.0; MAX_HORIZON],
            belief_vars: [0.0; MAX_HORIZON],
            planned_edges,
            active_factors: HVec::new(),
        };
        let _ = tx.send(msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

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

    #[test]
    fn edge_at_s_first_edge() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(SimComms, map, 0);
        runner.set_trajectory(traj);
        let (edge, local) = runner.edge_at_s(2.5);
        assert_eq!(edge, EdgeId(0));
        assert!((local - 2.5).abs() < 1e-4);
    }

    #[test]
    fn edge_at_s_second_edge() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(SimComms, map, 0);
        runner.set_trajectory(traj);
        let (edge, local) = runner.edge_at_s(7.0);
        assert_eq!(edge, EdgeId(1));
        assert!((local - 2.0).abs() < 1e-4); // 7.0 - 5.0 = 2.0
    }

    #[test]
    fn edge_at_s_past_end() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(SimComms, map, 0);
        runner.set_trajectory(traj);
        let (edge, local) = runner.edge_at_s(12.0);
        assert_eq!(edge, EdgeId(1));
        assert!((local - 5.0).abs() < 1e-4); // clamped to last edge end
    }

    #[test]
    fn step_returns_positive_velocity() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(SimComms, map, 0);
        runner.set_trajectory(traj);
        let out = runner.step(0.0);
        assert!(out.velocity > 0.0, "v={}", out.velocity);
    }

    #[test]
    fn total_length_is_sum_of_edges() {
        let (map, traj) = two_edge_map();
        let mut runner = AgentRunner::new(SimComms, map, 0);
        let total = runner.set_trajectory(traj);
        assert!((total - 10.0).abs() < 1e-4);
    }
}
