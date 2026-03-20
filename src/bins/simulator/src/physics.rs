// src/bins/simulator/src/physics.rs
use std::sync::{Arc, Mutex};
use tokio::time::{interval, Duration};
use tracing::debug;

/// Shared mutable physics state for one robot.
#[derive(Clone, Debug)]
pub struct PhysicsState {
    /// Arc-length position along the current edge (m).
    pub position_s: f32,
    /// Current commanded velocity (m/s) -- written by broadcast task.
    pub velocity: f32,
    /// Length of the current edge (m) -- robot stops at end.
    pub edge_length: f32,
}

impl PhysicsState {
    pub fn new(edge_length: f32) -> Self {
        Self { position_s: 0.0, velocity: 0.0, edge_length }
    }

    /// Integrate one timestep. Wraps around to 0 when reaching edge_length (demo loop).
    pub fn step(&mut self, dt: f32) {
        self.position_s += self.velocity * dt;
        if self.position_s >= self.edge_length {
            self.position_s = 0.0; // loop back to start
        }
        if self.position_s < 0.0 {
            self.position_s = 0.0;
        }
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
        debug!("physics: s={:.4}", s.position_s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_advances_s_by_v_times_dt() {
        let mut state = PhysicsState { position_s: 0.0, velocity: 2.0, edge_length: 10.0 };
        state.step(0.02); // 50 Hz -> dt = 0.02 s
        assert!((state.position_s - 0.04).abs() < 1e-6, "got {}", state.position_s);
    }

    #[test]
    fn physics_clamps_to_edge_length() {
        let mut state = PhysicsState { position_s: 9.99, velocity: 5.0, edge_length: 10.0 };
        state.step(0.02);
        // 9.99 + 5.0 * 0.02 = 10.09 -> clamped to 10.0
        assert!((state.position_s - 10.0).abs() < 1e-6, "got {}", state.position_s);
    }

    #[test]
    fn physics_zero_velocity_stays_put() {
        let mut state = PhysicsState { position_s: 3.0, velocity: 0.0, edge_length: 10.0 };
        state.step(0.02);
        assert!((state.position_s - 3.0).abs() < 1e-6);
    }
}
