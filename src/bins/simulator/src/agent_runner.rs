//! Wraps RobotAgent and drives its step() loop.
//! Replaces broadcast_task from M1.
//! Holds Arc<Map> to make the lifetime self-enforcing across Tokio tasks.

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use tracing::debug;
use gbp_agent::RobotAgent;
use gbp_comms::{ObservationUpdate, RobotStateMsg, RobotSource};
use gbp_map::map::{EdgeId, Map};
use gbp_map::MAX_HORIZON;
use heapless::Vec as HVec;
use crate::physics::PhysicsState;
use crate::sim_comms::SimComms;

pub struct AgentRunner {
    /// Holds the Arc so that RobotAgent's raw *const Map pointer remains valid
    /// for the lifetime of this struct.
    _map: Arc<Map>,
    agent: RobotAgent<SimComms>,
    /// Mirrors the full trajectory edge list so we can broadcast planned_edges
    /// without requiring RobotAgent to expose its internal trajectory.
    trajectory_edges: HVec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
    /// Tracks cumulative arc-length offset from edge transitions.
    /// When robot transitions to next edge, we add the completed edge's length.
    cumulative_s: f32,
    /// The edge we were on last step (to detect transitions).
    last_edge: Option<EdgeId>,
    /// Edge lengths from the trajectory, used to compute cumulative_s on transition.
    trajectory_edge_lengths: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>,
}

impl AgentRunner {
    /// `map` must be kept alive for as long as the AgentRunner -- holding the Arc satisfies this.
    pub fn new(comms: SimComms, map: Arc<Map>, robot_id: u32) -> Self {
        let agent = RobotAgent::new(comms, &*map, robot_id);
        Self {
            _map: map,
            agent,
            trajectory_edges: HVec::new(),
            cumulative_s: 0.0,
            last_edge: None,
            trajectory_edge_lengths: HVec::new(),
        }
    }

    pub fn set_single_edge_trajectory(&mut self, edge_id: EdgeId, edge_length: f32) {
        let mut edges: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }> = HVec::new();
        let _ = edges.push((edge_id, edge_length));
        self.trajectory_edges.clear();
        let _ = self.trajectory_edges.push(edge_id);
        self.trajectory_edge_lengths.clear();
        let _ = self.trajectory_edge_lengths.push((edge_id, edge_length));
        self.cumulative_s = 0.0;
        self.last_edge = None;
        self.agent.set_trajectory(edges, 0.0);
    }

    pub fn set_trajectory_from_edges(
        &mut self,
        edges: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>,
        start_s: f32,
    ) {
        self.trajectory_edges.clear();
        self.trajectory_edge_lengths.clear();
        for &(eid, len) in &edges {
            let _ = self.trajectory_edges.push(eid);
            let _ = self.trajectory_edge_lengths.push((eid, len));
        }
        self.cumulative_s = start_s;
        self.last_edge = None;
        self.agent.set_trajectory(edges, start_s);
    }

    /// Single step. Returns velocity, agent's authoritative current_edge, and 3D world position.
    pub fn step(&mut self, position_s: f32, current_edge: EdgeId) -> StepOut {
        // Detect edge transition: if current_edge changed from last step,
        // add the completed edge's length to cumulative_s.
        if let Some(prev) = self.last_edge {
            if prev != current_edge {
                // Find the length of the previous edge in our trajectory
                if let Some(&(_, prev_len)) = self
                    .trajectory_edge_lengths
                    .iter()
                    .find(|(eid, _)| *eid == prev)
                {
                    self.cumulative_s += prev_len;
                }
            }
        }
        self.last_edge = Some(current_edge);

        // Convert local position to global arc-length for the trajectory
        let global_s = self.cumulative_s + position_s;

        let obs = ObservationUpdate {
            position_s: global_s,
            velocity: 0.0,
            current_edge,
        };
        let out = self.agent.step(obs);
        // Evaluate 3D world position (Bevy Y-up: map(x,y,z) -> bevy(x,z,-y))
        let pos_3d = match self._map.eval_position(out.current_edge, out.position_s) {
            Some(p) => [p[0], p[2], -p[1]],
            None => [0.0, 0.0, 0.0],
        };
        StepOut {
            velocity: out.velocity,
            position_s: out.position_s,
            current_edge: out.current_edge,
            pos_3d,
        }
    }

    /// Returns the agent's planned edge sequence for broadcast in RobotStateMsg.
    pub fn planned_edges_snapshot(&self) -> HVec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
        self.trajectory_edges.clone()
    }
}

pub struct StepOut {
    pub velocity: f32,
    pub position_s: f32,
    pub current_edge: EdgeId,
    pub pos_3d: [f32; 3],
}

/// Runs at 20 Hz: steps the agent, drives edge transitions, broadcasts RobotStateMsg.
pub async fn agent_task(
    physics: Arc<Mutex<PhysicsState>>,
    runner: Arc<Mutex<AgentRunner>>,
    map: Arc<Map>,
    tx: broadcast::Sender<RobotStateMsg>,
) {
    let mut ticker = interval(Duration::from_millis(50)); // 20 Hz
    loop {
        ticker.tick().await;
        let (pos_s, phys_edge) = {
            let p = physics.lock().unwrap();
            (p.position_s, p.current_edge)
        };

        let out = { runner.lock().unwrap().step(pos_s, phys_edge) };

        // Drive edge transitions via agent's authoritative current_edge
        if out.current_edge != phys_edge {
            let new_len = map
                .edges
                .iter()
                .find(|e| e.id == out.current_edge)
                .map(|e| e.geometry.length())
                .unwrap_or(1.0);
            let mut p = physics.lock().unwrap();
            p.transition_to(out.current_edge, new_len);
        }
        {
            physics.lock().unwrap().velocity = out.velocity;
        }

        let msg = RobotStateMsg {
            robot_id: 0,
            current_edge: out.current_edge,
            position_s: out.position_s,
            velocity: out.velocity,
            pos_3d: out.pos_3d,
            source: RobotSource::Simulated,
            belief_means: [0.0; MAX_HORIZON],
            belief_vars: [0.0; MAX_HORIZON],
            planned_edges: {
                let snap = runner.lock().unwrap().planned_edges_snapshot();
                let mut pe: HVec<EdgeId, MAX_HORIZON> = HVec::new();
                for &eid in snap.iter().take(MAX_HORIZON) {
                    let _ = pe.push(eid);
                }
                pe
            },
            active_factors: HVec::new(),
        };
        debug!(
            "agent: s={:.3} v={:.3} edge={:?}",
            out.position_s, out.velocity, out.current_edge
        );
        let _ = tx.send(msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim_comms::SimComms;
    use gbp_map::map::*;

    fn straight_map() -> Map {
        let mut m = Map::new("s");
        m.add_node(Node {
            id: NodeId(0),
            position: [0.0, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        })
        .unwrap();
        m.add_node(Node {
            id: NodeId(1),
            position: [5.0, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        })
        .unwrap();
        m.add_edge(Edge {
            id: EdgeId(0),
            start: NodeId(0),
            end: NodeId(1),
            geometry: EdgeGeometry::Line {
                start: [0.0, 0.0, 0.0],
                end: [5.0, 0.0, 0.0],
                length: 5.0,
            },
            speed: SpeedProfile {
                max: 2.5,
                nominal: 2.0,
                accel_limit: 1.0,
                decel_limit: 1.0,
            },
            safety: SafetyProfile { clearance: 0.3 },
        })
        .unwrap();
        m
    }

    #[test]
    fn step_returns_positive_velocity_on_trajectory() {
        let map = std::sync::Arc::new(straight_map());
        let mut runner = AgentRunner::new(SimComms, map.clone(), 0);
        runner.set_single_edge_trajectory(EdgeId(0), 5.0);
        let out = runner.step(0.0, EdgeId(0));
        assert!(out.velocity > 0.0, "v={}", out.velocity);
    }

    #[test]
    fn step_velocity_tapers_near_end_of_final_edge() {
        let map = std::sync::Arc::new(straight_map());
        let mut runner = AgentRunner::new(SimComms, map.clone(), 0);
        runner.set_single_edge_trajectory(EdgeId(0), 5.0);
        let v_start = runner.step(0.0, EdgeId(0)).velocity;
        let v_mid = runner.step(2.5, EdgeId(0)).velocity;
        let v_near_end = runner.step(4.9, EdgeId(0)).velocity;
        assert!(v_start > 1.5, "v_start={} expected ~2.0", v_start);
        assert!(
            v_near_end < 0.8,
            "v_near_end={} expected ~0 (decel_limit=1.0, remaining=0.1m -> sqrt(0.2)~0.45)",
            v_near_end
        );
        assert!(v_near_end < v_mid, "expected monotone taper");
    }

    #[test]
    fn step_velocity_nominal_on_intermediate_edge_tapered_on_final() {
        let map = {
            let mut m = Map::new("two");
            m.add_node(Node {
                id: NodeId(0),
                position: [0.0, 0.0, 0.0],
                node_type: NodeType::Waypoint,
            })
            .unwrap();
            m.add_node(Node {
                id: NodeId(1),
                position: [5.0, 0.0, 0.0],
                node_type: NodeType::Waypoint,
            })
            .unwrap();
            m.add_node(Node {
                id: NodeId(2),
                position: [10.0, 0.0, 0.0],
                node_type: NodeType::Waypoint,
            })
            .unwrap();
            let sp = SpeedProfile {
                max: 2.5,
                nominal: 2.0,
                accel_limit: 1.0,
                decel_limit: 1.0,
            };
            let sf = SafetyProfile { clearance: 0.3 };
            m.add_edge(Edge {
                id: EdgeId(0),
                start: NodeId(0),
                end: NodeId(1),
                geometry: EdgeGeometry::Line {
                    start: [0.0, 0.0, 0.0],
                    end: [5.0, 0.0, 0.0],
                    length: 5.0,
                },
                speed: sp.clone(),
                safety: sf.clone(),
            })
            .unwrap();
            m.add_edge(Edge {
                id: EdgeId(1),
                start: NodeId(1),
                end: NodeId(2),
                geometry: EdgeGeometry::Line {
                    start: [5.0, 0.0, 0.0],
                    end: [10.0, 0.0, 0.0],
                    length: 5.0,
                },
                speed: sp,
                safety: sf,
            })
            .unwrap();
            std::sync::Arc::new(m)
        };
        let mut runner = AgentRunner::new(SimComms, map.clone(), 0);
        let mut traj: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }> = HVec::new();
        traj.push((EdgeId(0), 5.0)).unwrap();
        traj.push((EdgeId(1), 5.0)).unwrap();
        runner.set_trajectory_from_edges(traj, 0.0);

        // On intermediate edge 0 at mid-point: should be nominal (not tapered)
        let v_intermediate = runner.step(2.5, EdgeId(0)).velocity;
        assert!(
            v_intermediate > 1.8,
            "expected v_nom on intermediate edge, got {}",
            v_intermediate
        );

        // Now simulate transition to edge 1. The runner detects edge change
        // and adds edge 0's length (5.0) to cumulative_s.
        // position_s=4.9 on edge 1 -> global_s = 5.0 + 4.9 = 9.9
        let v_final_near_end = runner.step(4.9, EdgeId(1)).velocity;
        assert!(
            v_final_near_end < 0.8,
            "expected taper on final edge, got {}",
            v_final_near_end
        );
    }

    #[test]
    fn planned_edges_snapshot_returns_set_trajectory() {
        let map = std::sync::Arc::new(straight_map());
        let mut runner = AgentRunner::new(SimComms, map.clone(), 0);
        runner.set_single_edge_trajectory(EdgeId(0), 5.0);
        let snap = runner.planned_edges_snapshot();
        assert_eq!(snap.len(), 1);
        assert_eq!(snap[0], EdgeId(0));
    }
}
