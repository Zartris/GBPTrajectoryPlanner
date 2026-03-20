// src/bins/visualiser/src/state.rs
use bevy::prelude::*;
use std::sync::{Arc, Mutex};
use std::collections::{VecDeque, HashMap};
use gbp_comms::RobotStateMsg;
use gbp_map::map::EdgeId;
use gbp_map::MAX_HORIZON;

/// Maximum entries in the WS inbox queue before dropping oldest.
pub const WS_INBOX_CAP: usize = 64;

/// Bevy resource: queue of incoming messages from WebSocket thread.
#[derive(Resource, Default)]
pub struct WsInbox(pub Arc<Mutex<VecDeque<RobotStateMsg>>>);

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
    pub sigma_dyn: f32,
    pub sigma_r: f32,
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
            sigma_dyn: 0.0,
            sigma_r: 0.0,
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
        self.active_factor_count = msg.ir_factor_count as usize;
    }

}

/// Bevy resource: latest known state per robot, keyed by robot_id.
#[derive(Resource, Default)]
pub struct RobotStates(pub HashMap<u32, RobotState>);

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
        assert_eq!(s.sigma_dyn, 0.0);
        assert_eq!(s.sigma_r, 0.0);
    }
}
