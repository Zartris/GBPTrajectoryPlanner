//! TOML config and scenario file parsing.
//! Converts nested TOML structure into flat GbpConfig.

use serde::Deserialize;
use gbp_core::GbpConfig;

// -- Main config TOML structure --

#[derive(Deserialize, Default)]
pub struct TomlConfig {
    #[serde(default)]
    pub gbp: GbpSection,
    #[serde(default)]
    pub robot: RobotSection,
}

#[derive(Deserialize, Default)]
pub struct GbpSection {
    pub msg_damping: Option<f32>,
    pub internal_iters: Option<u8>,
    pub external_iters: Option<u8>,
    pub timestep: Option<f32>,
    pub init_variance: Option<f32>,
    pub anchor_precision: Option<f32>,
    #[serde(default)]
    pub dynamics: DynamicsSection,
    #[serde(default)]
    pub interrobot: InterrobotSection,
    #[serde(default)]
    pub velocity_bound: VelocityBoundSection,
}

#[derive(Deserialize, Default)]
pub struct DynamicsSection {
    pub sigma: Option<f32>,
}

#[derive(Deserialize, Default)]
pub struct InterrobotSection {
    pub d_safe: Option<f32>,
    pub sigma: Option<f32>,
    pub activation_range: Option<f32>,
    pub decay_alpha: Option<f32>,
    pub front_damping: Option<f32>,
}

#[derive(Deserialize, Default)]
pub struct VelocityBoundSection {
    pub v_min: Option<f32>,
    pub v_max_default: Option<f32>,
    pub kappa: Option<f32>,
    pub margin: Option<f32>,
    pub max_precision: Option<f32>,
}

#[derive(Deserialize, Default)]
pub struct RobotSection {
    pub max_accel: Option<f32>,
    pub max_jerk: Option<f32>,
    pub max_speed: Option<f32>,
    pub v_min: Option<f32>,
}

impl From<TomlConfig> for GbpConfig {
    fn from(t: TomlConfig) -> Self {
        let d = GbpConfig::default();
        GbpConfig {
            msg_damping: t.gbp.msg_damping.unwrap_or(d.msg_damping),
            internal_iters: t.gbp.internal_iters.unwrap_or(d.internal_iters),
            external_iters: t.gbp.external_iters.unwrap_or(d.external_iters),
            sigma_dynamics: t.gbp.dynamics.sigma.unwrap_or(d.sigma_dynamics),
            gbp_timestep: t.gbp.timestep.unwrap_or(d.gbp_timestep),
            d_safe: t.gbp.interrobot.d_safe.unwrap_or(d.d_safe),
            sigma_interrobot: t.gbp.interrobot.sigma.unwrap_or(d.sigma_interrobot),
            ir_activation_range: t.gbp.interrobot.activation_range.unwrap_or(d.ir_activation_range),
            ir_decay_alpha: t.gbp.interrobot.decay_alpha.unwrap_or(d.ir_decay_alpha),
            front_damping: t.gbp.interrobot.front_damping.unwrap_or(d.front_damping),
            v_min: t.gbp.velocity_bound.v_min.or(t.robot.v_min).unwrap_or(d.v_min),
            v_max_default: t.gbp.velocity_bound.v_max_default.unwrap_or(d.v_max_default),
            vb_kappa: t.gbp.velocity_bound.kappa.unwrap_or(d.vb_kappa),
            vb_margin: t.gbp.velocity_bound.margin.unwrap_or(d.vb_margin),
            vb_max_precision: t.gbp.velocity_bound.max_precision.unwrap_or(d.vb_max_precision),
            max_accel: t.robot.max_accel.unwrap_or(d.max_accel),
            max_jerk: t.robot.max_jerk.unwrap_or(d.max_jerk),
            max_speed: t.robot.max_speed.unwrap_or(d.max_speed),
            init_variance: t.gbp.init_variance.unwrap_or(d.init_variance),
            anchor_precision: t.gbp.anchor_precision.unwrap_or(d.anchor_precision),
        }
    }
}

/// Validate config values. Panics with descriptive error on invalid values.
pub fn validate(c: &GbpConfig) {
    assert!(c.msg_damping >= 0.0 && c.msg_damping <= 1.0,
        "msg_damping must be 0.0..=1.0, got {}", c.msg_damping);
    assert!(c.internal_iters >= 1, "internal_iters must be >= 1");
    assert!(c.external_iters >= 1, "external_iters must be >= 1");
    assert!(c.sigma_dynamics > 0.0, "sigma_dynamics must be > 0.0");
    assert!(c.gbp_timestep > 0.0, "gbp_timestep must be > 0.0");
    assert!(c.d_safe > 0.0, "d_safe must be > 0.0");
    assert!(c.sigma_interrobot > 0.0, "sigma_interrobot must be > 0.0");
    assert!(c.ir_activation_range > 0.0, "ir_activation_range must be > 0.0");
    assert!(c.ir_decay_alpha > 0.0, "ir_decay_alpha must be > 0.0");
    assert!(c.front_damping >= 0.0 && c.front_damping <= 1.0,
        "front_damping must be 0.0..=1.0, got {}", c.front_damping);
    assert!(c.v_min < c.v_max_default,
        "v_min ({}) must be < v_max_default ({})", c.v_min, c.v_max_default);
    assert!(c.vb_kappa > 0.0, "vb_kappa must be > 0.0");
    assert!(c.vb_margin > 0.0, "vb_margin must be > 0.0");
    assert!(c.vb_max_precision > 0.0, "vb_max_precision must be > 0.0");
    assert!(c.max_accel > 0.0, "max_accel must be > 0.0");
    assert!(c.max_jerk > 0.0, "max_jerk must be > 0.0");
    assert!(c.max_speed > 0.0, "max_speed must be > 0.0");
    assert!(c.init_variance > 0.0, "init_variance must be > 0.0");
    assert!(c.anchor_precision > 0.0, "anchor_precision must be > 0.0");
}

/// Parse a config TOML string into a validated GbpConfig.
pub fn parse_config(toml_str: &str) -> GbpConfig {
    let tc: TomlConfig = toml::from_str(toml_str)
        .unwrap_or_else(|e| panic!("config TOML parse error: {}", e));
    let config: GbpConfig = tc.into();
    validate(&config);
    config
}

// -- Scenario TOML --

#[derive(Deserialize)]
pub struct ScenarioFile {
    pub scenario: ScenarioSection,
    #[serde(default)]
    pub robots: Vec<RobotEntry>,
}

#[derive(Deserialize)]
pub struct ScenarioSection {
    pub name: String,
    pub map: String,
    #[serde(default)]
    pub auto_assign: bool,
    #[serde(default = "default_num_robots")]
    pub num_robots: usize,
}

fn default_num_robots() -> usize { 4 }

#[derive(Deserialize)]
pub struct RobotEntry {
    pub start: String,
    pub goal: String,
}

/// Parse a scenario TOML string.
pub fn parse_scenario(toml_str: &str) -> ScenarioFile {
    toml::from_str(toml_str)
        .unwrap_or_else(|e| panic!("scenario TOML parse error: {}", e))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_config_gives_defaults() {
        let config = parse_config("");
        assert_eq!(config, GbpConfig::default());
    }

    #[test]
    fn partial_config_overrides_only_specified() {
        let config = parse_config("[gbp]\nmsg_damping = 0.8\n");
        assert!((config.msg_damping - 0.8).abs() < 1e-6);
        assert_eq!(config.internal_iters, 10); // default
    }

    #[test]
    fn nested_section_overrides() {
        let config = parse_config("[gbp.interrobot]\nd_safe = 2.0\n");
        assert!((config.d_safe - 2.0).abs() < 1e-6);
        assert!((config.sigma_interrobot - 0.12).abs() < 1e-6); // default
    }

    #[test]
    fn scenario_with_robots_parses() {
        let s = parse_scenario(r#"
[scenario]
name = "test"
map = "maps/test.yaml"

[[robots]]
start = "P001"
goal = "P002"
"#);
        assert_eq!(s.scenario.name, "test");
        assert_eq!(s.robots.len(), 1);
        assert_eq!(s.robots[0].start, "P001");
    }

    #[test]
    fn scenario_auto_assign_parses() {
        let s = parse_scenario(r#"
[scenario]
name = "fleet"
map = "maps/test.yaml"
auto_assign = true
num_robots = 6
"#);
        assert!(s.scenario.auto_assign);
        assert_eq!(s.scenario.num_robots, 6);
        assert!(s.robots.is_empty());
    }

    #[test]
    #[should_panic(expected = "msg_damping must be 0.0..=1.0")]
    fn invalid_msg_damping_panics() {
        parse_config("[gbp]\nmsg_damping = 2.0\n");
    }

    #[test]
    #[should_panic(expected = "v_min")]
    fn invalid_v_min_greater_than_v_max_panics() {
        parse_config("[gbp.velocity_bound]\nv_min = 5.0\nv_max_default = 2.5\n");
    }
}
