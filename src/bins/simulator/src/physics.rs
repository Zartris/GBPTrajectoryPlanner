// src/bins/simulator/src/physics.rs
use gbp_map::map::EdgeId;
use std::sync::{Arc, Mutex};
use tokio::time::{interval, Duration};
use tracing::debug;

/// Shared mutable physics state for one robot.
#[derive(Clone, Debug)]
pub struct PhysicsState {
    /// Arc-length position along the current edge (m).
    pub position_s: f32,
    /// Current commanded velocity (m/s) -- written by agent task.
    pub velocity: f32,
    /// Length of the current edge (m).
    pub edge_length: f32,
    /// Which edge the robot is currently on.
    pub current_edge: EdgeId,
    /// Set to true when position_s reaches edge_length; cleared on transition_to().
    pub edge_done: bool,
}

impl PhysicsState {
    pub fn new(edge_id: EdgeId, edge_length: f32) -> Self {
        Self {
            position_s: 0.0,
            velocity: 0.0,
            edge_length,
            current_edge: edge_id,
            edge_done: false,
        }
    }

    /// Integrate one timestep. Sets edge_done when position reaches edge end.
    pub fn step(&mut self, dt: f32) {
        let new_s = self.position_s + self.velocity * dt;
        self.position_s = new_s.clamp(0.0, self.edge_length);
        if self.position_s >= self.edge_length - 0.01 {
            self.edge_done = true;
        }
    }

    /// Advance to the next edge, resetting position to 0.
    pub fn transition_to(&mut self, next_edge: EdgeId, next_length: f32) {
        self.current_edge = next_edge;
        self.edge_length = next_length;
        self.position_s = 0.0;
        self.edge_done = false;
    }
}

/// Runs at 50 Hz, integrating position from velocity.
pub async fn physics_task(state: Arc<Mutex<PhysicsState>>) {
    const DT: f32 = 1.0 / 50.0;
    let mut ticker = interval(Duration::from_millis(20)); // 50 Hz
    loop {
        ticker.tick().await;
        let mut s = state.lock().unwrap_or_else(|e| e.into_inner());
        s.step(DT);
        debug!("physics: s={:.4} edge={:?}", s.position_s, s.current_edge);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_advances_s_by_v_times_dt() {
        let mut state = PhysicsState {
            position_s: 0.0,
            velocity: 2.0,
            edge_length: 10.0,
            current_edge: EdgeId(0),
            edge_done: false,
        };
        state.step(0.02); // 50 Hz -> dt = 0.02 s
        assert!((state.position_s - 0.04).abs() < 1e-6, "got {}", state.position_s);
    }

    #[test]
    fn physics_clamps_at_edge_length() {
        let mut state = PhysicsState {
            position_s: 9.99,
            velocity: 5.0,
            edge_length: 10.0,
            current_edge: EdgeId(0),
            edge_done: false,
        };
        state.step(0.02);
        // 9.99 + 5.0 * 0.02 = 10.09 -> clamped to 10.0
        assert!(
            (state.position_s - 10.0).abs() < 1e-4,
            "got {}",
            state.position_s
        );
        // edge_done triggers when within 0.01m of edge end
        assert!(state.edge_done);
    }

    #[test]
    fn physics_zero_velocity_stays_put() {
        let mut state = PhysicsState {
            position_s: 3.0,
            velocity: 0.0,
            edge_length: 10.0,
            current_edge: EdgeId(0),
            edge_done: false,
        };
        state.step(0.02);
        assert!((state.position_s - 3.0).abs() < 1e-6);
    }

    #[test]
    fn physics_triggers_edge_done_near_end() {
        let mut state = PhysicsState {
            position_s: 9.98,
            velocity: 2.0,
            edge_length: 10.0,
            current_edge: EdgeId(0),
            edge_done: false,
        };
        state.step(0.02);
        // 9.98 + 2.0*0.02 = 10.02 -> clamped to 10.0, within 0.01m threshold
        assert!(state.edge_done);
    }

    #[test]
    fn physics_resets_edge_done_on_new_edge() {
        let mut state = PhysicsState {
            position_s: 10.0,
            velocity: 0.0,
            edge_length: 10.0,
            current_edge: EdgeId(0),
            edge_done: true,
        };
        state.transition_to(EdgeId(1), 8.0);
        assert_eq!(state.current_edge, EdgeId(1));
        assert_eq!(state.edge_length, 8.0);
        assert!((state.position_s).abs() < 1e-6);
        assert!(!state.edge_done);
    }
}
