/// System-wide tunable parameters for the GBP trajectory planner.
///
/// Constructed by the simulator from a TOML config file and passed to
/// AgentRunner / RobotAgent at init time. Core crates never parse files —
/// they receive this struct by value.
///
/// The Default impl provides the current battle-tested values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GbpConfig {
    // -- GBP solver --
    pub msg_damping: f32,
    pub internal_iters: u8,
    pub external_iters: u8,

    // -- Dynamics factor --
    pub sigma_dynamics: f32,
    pub gbp_timestep: f32,

    // -- Inter-robot factor --
    pub d_safe: f32,
    pub sigma_interrobot: f32,
    pub ir_activation_range: f32,
    pub ir_decay_alpha: f32,
    pub front_damping: f32,

    // -- Velocity bounds (BIPM-inspired) --
    pub v_min: f32,
    pub v_max_default: f32,
    pub vb_kappa: f32,
    pub vb_margin: f32,
    pub vb_max_precision: f32,

    // -- Dynamic constraints (post-GBP) --
    pub max_accel: f32,
    pub max_jerk: f32,
    pub max_speed: f32,

    // -- Graph initialization --
    pub init_variance: f32,
    pub anchor_precision: f32,
}

impl Default for GbpConfig {
    fn default() -> Self {
        Self {
            msg_damping: 0.5,
            internal_iters: 10,
            external_iters: 10,
            sigma_dynamics: 0.5,
            gbp_timestep: 0.1,
            d_safe: 1.3,
            sigma_interrobot: 0.12,
            ir_activation_range: 3.0,
            ir_decay_alpha: 3.0,
            front_damping: 0.3,
            v_min: -0.3,
            v_max_default: 2.5,
            vb_kappa: 10.0,
            vb_margin: 1.0,
            vb_max_precision: 100.0,
            max_accel: 2.5,
            max_jerk: 5.0,
            max_speed: 2.5,
            init_variance: 100.0,
            anchor_precision: 1000.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_matches_hardcoded_values() {
        let c = GbpConfig::default();
        assert!((c.msg_damping - 0.5).abs() < 1e-6);
        assert_eq!(c.internal_iters, 10);
        assert_eq!(c.external_iters, 10);
        assert!((c.d_safe - 1.3).abs() < 1e-6);
        assert!((c.sigma_interrobot - 0.12).abs() < 1e-6);
        assert!((c.v_min - (-0.3)).abs() < 1e-6);
        assert!((c.max_accel - 2.5).abs() < 1e-6);
    }

    #[test]
    fn config_is_copy() {
        let a = GbpConfig::default();
        let b = a; // Copy
        assert_eq!(a, b);
    }
}
