// src/bins/simulator/src/physics.rs
use std::sync::{Arc, Mutex};
use tokio::time::{interval, Duration};

/// Shared mutable physics state for one robot.
/// Uses a single global arc-length `s` across the entire trajectory.
/// Which edge the robot is on is derived from the trajectory, not tracked here.
#[derive(Clone, Debug)]
pub struct PhysicsState {
    /// Global arc-length position across the full trajectory (m).
    pub position_s: f32,
    /// Current commanded velocity (m/s) — written by agent task.
    pub velocity: f32,
    /// Total path length (m) — sum of all trajectory edge lengths.
    pub total_length: f32,
}

impl PhysicsState {
    pub fn new(total_length: f32) -> Self {
        Self { position_s: 0.0, velocity: 0.0, total_length }
    }

    /// Integrate one timestep. Clamps at total_length (robot stops at goal).
    pub fn step(&mut self, dt: f32) {
        self.position_s = (self.position_s + self.velocity * dt).clamp(0.0, self.total_length);
    }

    /// Whether the robot has reached the end of the trajectory.
    pub fn at_goal(&self) -> bool {
        self.position_s >= self.total_length - 0.01
    }
}

/// Runs at 50 Hz, integrating position from velocity.
pub async fn physics_task(state: Arc<Mutex<PhysicsState>>) {
    const DT: f32 = 1.0 / 50.0;
    let mut ticker = interval(Duration::from_millis(20)); // 50 Hz
    loop {
        ticker.tick().await;
        state.lock().unwrap_or_else(|e| e.into_inner()).step(DT);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_advances_s_by_v_times_dt() {
        let mut state = PhysicsState::new(100.0);
        state.velocity = 2.0;
        state.step(0.02);
        assert!((state.position_s - 0.04).abs() < 1e-6, "got {}", state.position_s);
    }

    #[test]
    fn physics_clamps_at_total_length() {
        let mut state = PhysicsState { position_s: 99.9, velocity: 10.0, total_length: 100.0 };
        state.step(0.02);
        assert!((state.position_s - 100.0).abs() < 1e-4, "got {}", state.position_s);
    }

    #[test]
    fn physics_zero_velocity_stays_put() {
        let mut state = PhysicsState { position_s: 3.0, velocity: 0.0, total_length: 100.0 };
        state.step(0.02);
        assert!((state.position_s - 3.0).abs() < 1e-6);
    }

    #[test]
    fn at_goal_when_near_end() {
        let state = PhysicsState { position_s: 99.995, velocity: 0.0, total_length: 100.0 };
        assert!(state.at_goal());
    }

    #[test]
    fn not_at_goal_when_far() {
        let state = PhysicsState { position_s: 50.0, velocity: 2.0, total_length: 100.0 };
        assert!(!state.at_goal());
    }
}
