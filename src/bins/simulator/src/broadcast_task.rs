//! Broadcast task for M1: publishes RobotStateMsg at 20 Hz.
//! Velocity is set directly to v_nom (no GBP -- introduced in M2).
//! Retained for reference; M2 uses agent_runner::agent_task instead.

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use tracing::debug;
use gbp_comms::{RobotStateMsg, RobotSource};
use gbp_map::map::EdgeId;
use gbp_map::MAX_HORIZON;
use crate::physics::PhysicsState;

/// Build a RobotStateMsg for a single robot on a single edge in M1.
/// pos_3d is a placeholder (s, 0, 0) -- the visualiser re-evaluates 3D position.
pub fn build_robot_state_msg(edge_id: EdgeId, position_s: f32, v_nom: f32) -> RobotStateMsg {
    RobotStateMsg {
        robot_id: 0,
        current_edge: edge_id,
        position_s,
        velocity: v_nom,
        pos_3d: [position_s, 0.0, 0.0],
        source: RobotSource::Simulated,
        belief_means: [0.0; MAX_HORIZON],
        belief_vars: [0.0; MAX_HORIZON],
        planned_edges: heapless::Vec::new(),
        active_factors: heapless::Vec::new(),
            ir_factor_count: 0,
            active_ir_timesteps: heapless::Vec::new(),
    }
}

/// Runs at 20 Hz. Sets physics velocity to v_nom each cycle and broadcasts position.
#[allow(dead_code)]
pub async fn broadcast_task(
    physics: Arc<Mutex<PhysicsState>>,
    edge_id: EdgeId,
    v_nom: f32,
    tx: broadcast::Sender<RobotStateMsg>,
) {
    let mut ticker = interval(Duration::from_millis(50)); // 20 Hz
    loop {
        ticker.tick().await;
        let pos_s = {
            let mut p = physics.lock().unwrap_or_else(|e| e.into_inner());
            p.velocity = v_nom; // M1: constant nominal velocity
            p.position_s
        };
        let msg = build_robot_state_msg(edge_id, pos_s, v_nom);
        debug!("broadcast: s={:.4} v={:.4}", pos_s, v_nom);
        let _ = tx.send(msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_msg_sets_velocity_to_v_nom() {
        let msg = build_robot_state_msg(
            EdgeId(0),
            2.5,   // position_s
            1.8,   // v_nom
        );
        assert!((msg.velocity - 1.8).abs() < 1e-6);
        assert!((msg.position_s - 2.5).abs() < 1e-6);
    }

    #[test]
    fn build_msg_sets_3d_pos_from_s() {
        let msg = build_robot_state_msg(EdgeId(0), 3.0, 2.0);
        assert!((msg.pos_3d[0] - 3.0).abs() < 1e-6);
    }
}
