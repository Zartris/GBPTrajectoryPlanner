//! Post-GBP dynamic constraints.
//!
//! These are NOT part of the factor graph. GBP solves for trajectory positions,
//! and we extract a raw velocity from (s1 - s0) / dt. This module applies
//! physical feasibility constraints (accel, jerk, speed limits) AFTER the GBP
//! solution, ensuring the robot can physically follow the planned trajectory.
//!
//! In the future, these constraints could be integrated into the factor graph
//! as higher-order dynamics factors with state [s, v, a] instead of just [s].
//! For now, post-processing is simpler and gives correct behavior.

/// Physical dynamic constraints for velocity smoothing.
/// Applied after GBP extracts raw velocity from the factor graph.
#[derive(Clone, Debug)]
pub struct DynamicConstraints {
    /// Maximum acceleration (m/s²)
    pub max_accel: f32,
    /// Maximum jerk — rate of acceleration change (m/s³)
    pub max_jerk: f32,
    /// Maximum speed (m/s) — typically from edge.speed.max
    pub max_speed: f32,

    // Internal state
    last_velocity: f32,
    last_accel: f32,
}

impl DynamicConstraints {
    pub fn new(max_accel: f32, max_jerk: f32, max_speed: f32) -> Self {
        Self {
            max_accel,
            max_jerk,
            max_speed,
            last_velocity: 0.0,
            last_accel: 0.0,
        }
    }

    /// Update max_speed (e.g. when transitioning to an edge with different limits).
    pub fn set_max_speed(&mut self, max_speed: f32) {
        self.max_speed = max_speed;
    }

    /// Apply jerk, acceleration, and speed limits to the raw GBP velocity.
    ///
    /// `raw_velocity`: the velocity extracted from the GBP factor graph (s1 - s0) / dt
    /// `dt`: time since last call (typically 0.02s for 50 Hz)
    ///
    /// Returns the constrained velocity that the robot should actually command.
    pub fn apply(&mut self, raw_velocity: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return self.last_velocity;
        }

        // Desired acceleration to reach raw_velocity
        let desired_accel = (raw_velocity - self.last_velocity) / dt;

        // Jerk limit: clamp rate of acceleration change
        let jerk = (desired_accel - self.last_accel) / dt;
        let clamped_jerk = jerk.clamp(-self.max_jerk, self.max_jerk);
        let accel = self.last_accel + clamped_jerk * dt;

        // Acceleration limit
        let clamped_accel = accel.clamp(-self.max_accel, self.max_accel);

        // Integrate to velocity, clamp to [v_min, max_speed].
        // v_min = -0.3 allows slow reverse creep for merge overshoot recovery.
        const V_MIN: f32 = -0.3;
        let velocity = (self.last_velocity + clamped_accel * dt).clamp(V_MIN, self.max_speed);

        self.last_accel = clamped_accel;
        self.last_velocity = velocity;

        velocity
    }

    /// Current velocity (for diagnostics).
    pub fn velocity(&self) -> f32 { self.last_velocity }

    /// Current acceleration (for diagnostics).
    pub fn accel(&self) -> f32 { self.last_accel }

    /// Reset state (e.g. on trajectory change).
    pub fn reset(&mut self) {
        self.last_velocity = 0.0;
        self.last_accel = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn respects_max_speed() {
        let mut dc = DynamicConstraints::new(10.0, 100.0, 2.5);
        // With high accel/jerk limits, should reach max_speed quickly
        for _ in 0..200 {
            dc.apply(10.0, 0.02); // request way above max
        }
        assert!((dc.velocity() - 2.5).abs() < 0.01, "v={}", dc.velocity());
    }

    #[test]
    fn respects_accel_limit() {
        let mut dc = DynamicConstraints::new(2.5, 1000.0, 10.0);
        // With high jerk but limited accel, velocity should increase by at most 2.5 * dt per step
        let v = dc.apply(10.0, 0.02);
        assert!(v <= 2.5 * 0.02 + 0.001, "v={} expected <= {}", v, 2.5 * 0.02);
    }

    #[test]
    fn smooth_ramp_up() {
        let mut dc = DynamicConstraints::new(2.5, 5.0, 2.5);
        let mut velocities = [0.0f32; 10];
        for i in 0..10 {
            velocities[i] = dc.apply(2.5, 0.02);
        }
        // Each velocity should be >= previous (monotone ramp-up to target)
        for i in 1..10 {
            assert!(velocities[i] >= velocities[i-1] - 0.001,
                "v[{}]={} < v[{}]={}", i, velocities[i], i-1, velocities[i-1]);
        }
    }

    #[test]
    fn zero_velocity_stays_zero() {
        let mut dc = DynamicConstraints::new(2.5, 5.0, 2.5);
        let v = dc.apply(0.0, 0.02);
        assert!(v.abs() < 0.001);
    }
}
