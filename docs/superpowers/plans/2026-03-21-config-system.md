# Config System — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **CRITICAL for subagents:** Use the Edit tool for all changes to existing files — NEVER use Write to overwrite. Do NOT modify functions/sections not mentioned in your task. Leave all other code unchanged.

**Goal:** Replace hardcoded constants with a TOML-based config system — a main `config.toml` for GBP/physics tunables, and separate scenario TOML files for robot assignments.

**Architecture:** A `GbpConfig` struct (no_std, Copy) in `gbp-core` holds all tunables with a `Default` impl matching current values. The simulator parses TOML into an intermediate serde struct and converts to `GbpConfig`. Scenario files define map + robot start/goal by node name, resolved via a name→NodeId lookup table from the parser.

**Tech Stack:** Rust, `toml` crate (simulator only), `serde` (simulator only), existing `#![no_std]` core crates unchanged.

**Spec:** `docs/superpowers/specs/2026-03-21-config-system-design.md`

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `crates/gbp-core/src/config.rs` | `GbpConfig` struct + `Default` impl |
| Create | `src/bins/simulator/src/toml_config.rs` | TOML serde structs, `From<TomlConfig>` for `GbpConfig`, validation, scenario parsing |
| Create | `config/config.toml` | Default config with all current values |
| Create | `config/scenarios/merge.toml` | Merge scenario |
| Create | `config/scenarios/endcollision.toml` | Endcollision scenario |
| Create | `config/scenarios/fleet_4.toml` | 4-robot fleet scenario |
| Modify | `Cargo.toml` | Add `toml` workspace dep |
| Modify | `src/bins/simulator/Cargo.toml` | Add `toml` dep |
| Modify | `crates/gbp-core/src/lib.rs` | Export `config` module + `GbpConfig` |
| Modify | `crates/gbp-core/src/factor_graph.rs` | Accept `msg_damping` param |
| Modify | `crates/gbp-core/src/interrobot_factor.rs` | Accept `decay_alpha` param |
| Modify | `crates/gbp-core/src/velocity_bound_factor.rs` | Expand constructor to accept `v_min`, `kappa`, `margin`, `max_prec` |
| Modify | `crates/gbp-agent/src/robot_agent.rs` | Accept `&GbpConfig`, replace all hardcoded consts |
| Modify | `crates/gbp-agent/src/dynamic_constraints.rs` | Accept `v_min` in constructor |
| Modify | `crates/gbp-map/src/parser.rs` | Return name→NodeId lookup alongside Map |
| Modify | `src/bins/simulator/src/agent_runner.rs` | Pass `&GbpConfig` through to `RobotAgent` |
| Modify | `src/bins/simulator/src/main.rs` | Parse TOML files, new CLI args, remove hardcoded scenario logic |

---

## Task 1: Create `GbpConfig` struct in `gbp-core`

**Files:**
- Create: `crates/gbp-core/src/config.rs`
- Modify: `crates/gbp-core/src/lib.rs`

- [ ] **Step 1: Create config.rs with struct and Default impl**

```rust
// crates/gbp-core/src/config.rs

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
            internal_iters: 3,
            external_iters: 5,
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
        assert_eq!(c.internal_iters, 3);
        assert_eq!(c.external_iters, 5);
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
```

- [ ] **Step 2: Export from lib.rs**

Add to `crates/gbp-core/src/lib.rs` after the velocity_bound_factor line:

```rust
pub mod config;
pub use config::GbpConfig;
```

- [ ] **Step 3: Build and test**

```bash
cargo test -p gbp-core
```

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-core/src/config.rs crates/gbp-core/src/lib.rs
git commit -m "feat(config): add GbpConfig struct with Default impl in gbp-core"
```

---

## Task 2: Wire `GbpConfig` into core factor types

**Files:**
- Modify: `crates/gbp-core/src/factor_graph.rs`
- Modify: `crates/gbp-core/src/interrobot_factor.rs`
- Modify: `crates/gbp-core/src/velocity_bound_factor.rs`

The goal is to replace hardcoded constants in these files with values passed in from outside. The factor graph and factors don't know about `GbpConfig` directly — they accept individual parameters.

- [ ] **Step 1: factor_graph.rs — parameterize msg_damping**

Replace `const MSG_DAMPING: f32 = 0.5;` with a field on `FactorGraph`. Add `msg_damping: f32` field and accept it in the constructor.

In `crates/gbp-core/src/factor_graph.rs`:

Remove line 16:
```rust
const MSG_DAMPING: f32 = 0.5;
```

Add field to `FactorGraph` struct:
```rust
pub struct FactorGraph<const K: usize, const F: usize> {
    pub variables: [VariableNode; K],
    factors: Vec<FactorNode, F>,
    msg_damping: f32,
}
```

Update the `new` method to accept `msg_damping`:
```rust
pub fn new(init_mean: f32, init_variance: f32, msg_damping: f32) -> Self {
```

And store it: `msg_damping,` in the constructor body.

Replace all uses of `MSG_DAMPING` in the file with `self.msg_damping`.

- [ ] **Step 2: interrobot_factor.rs — parameterize decay_alpha**

Replace `const DECAY_ALPHA: f32 = 3.0;` (line 68 inside `linearize()`) with a field on `InterRobotFactor`.

Add `pub decay_alpha: f32,` to the struct fields.

Update `new()` to accept it (note: uses `var_idx_a: usize`, NOT an array):
```rust
pub fn new(var_idx_a: usize, d_safe: f32, sigma_r: f32, decay_alpha: f32) -> Self {
    Self {
        var_idx_a, d_safe, sigma_r, decay_alpha,
        active: true,
        jacobian_a: 0.0, jacobian_b: 0.0,
        ext_eta_b: 0.0, ext_lambda_b: 1.0,
        dist: f32::MAX, // starts inactive (well beyond d_safe)
    }
}
```

In `linearize()`, replace `const DECAY_ALPHA: f32 = 3.0;` with `let decay_alpha = self.decay_alpha;` and use `decay_alpha` where `DECAY_ALPHA` was used.

- [ ] **Step 3: velocity_bound_factor.rs — parameterize constructor**

The struct currently has fields: `var_indices`, `dt`, `v_max`, `v_min`, `kappa`, `margin`. Add `max_precision: f32` field.

Updated struct:
```rust
pub struct VelocityBoundFactor {
    var_indices: [usize; 2],
    dt: f32,
    pub v_max: f32,
    v_min: f32,
    kappa: f32,
    margin: f32,
    max_precision: f32,
}
```

Expand `new()` to accept all parameters:
```rust
pub fn new(var_indices: [usize; 2], dt: f32, v_max: f32, v_min: f32, kappa: f32, margin: f32, max_precision: f32) -> Self {
    Self {
        var_indices,
        dt: f32::max(dt, 1e-6),
        v_max, v_min, kappa, margin, max_precision,
    }
}
```

Change `barrier_precision` from static function to method so it can use `self.max_precision`:
```rust
fn barrier_precision(&self, g: f32) -> f32 {
    if g < -self.margin {
        0.0
    } else if g < -1e-4 {
        let raw = 1.0 / (self.kappa * g * g);
        if raw < self.max_precision { raw } else { self.max_precision }
    } else {
        self.max_precision
    }
}
```

Update all call sites of `barrier_precision` in the same file from `Self::barrier_precision(g, self.margin, self.kappa)` to `self.barrier_precision(g)`.

Update tests in the file to pass all 7 parameters to `new()`.

- [ ] **Step 4: Build and test**

```bash
cargo test -p gbp-core
```

Fix any compilation errors from changed constructor signatures in tests.

- [ ] **Step 5: Commit**

```bash
git add crates/gbp-core/src/factor_graph.rs crates/gbp-core/src/interrobot_factor.rs crates/gbp-core/src/velocity_bound_factor.rs
git commit -m "refactor(config): parameterize factor_graph, interrobot, and velocity_bound constructors"
```

---

## Task 3: Wire `GbpConfig` into `gbp-agent`

**Files:**
- Modify: `crates/gbp-agent/src/robot_agent.rs`
- Modify: `crates/gbp-agent/src/dynamic_constraints.rs`

This is the biggest task — `RobotAgent::new()` currently hardcodes all values.

- [ ] **Step 1: Update DynamicConstraints to accept v_min**

In `crates/gbp-agent/src/dynamic_constraints.rs`, change `new()` to accept `v_min`:

```rust
pub struct DynamicConstraints {
    pub max_accel: f32,
    pub max_jerk: f32,
    pub max_speed: f32,
    v_min: f32,
    last_velocity: f32,
    last_accel: f32,
}

pub fn new(max_accel: f32, max_jerk: f32, max_speed: f32, v_min: f32) -> Self {
    Self { max_accel, max_jerk, max_speed, v_min, last_velocity: 0.0, last_accel: 0.0 }
}
```

In `apply()`, replace `const V_MIN: f32 = -0.3;` with `self.v_min`.

Update tests to pass `v_min: -0.3` to `DynamicConstraints::new()`.

- [ ] **Step 2: Update RobotAgent to accept &GbpConfig**

In `crates/gbp-agent/src/robot_agent.rs`:

Add import:
```rust
use gbp_core::GbpConfig;
```

Remove these constants (lines ~20-28):
```rust
const GBP_INTERNAL_ITERS: usize = 3;
const GBP_EXTERNAL_ITERS: usize = 5;
const DT: f32 = 0.1;
const D_SAFE: f32 = 1.3;
const SIGMA_R: f32 = 0.12;
const IR_RAMP_START: f32 = 2.6;
const IR_ACTIVATION_RANGE: f32 = 3.0;
const AGENT_DT: f32 = 0.02;
```

Keep `AGENT_DT` as a const (infrastructure, not tuning).

Add a `config: GbpConfig` field to `RobotAgent<C>`.

Change `new()` signature:
```rust
pub fn new(comms: C, map: &Map, robot_id: RobotId, config: &GbpConfig) -> Self {
```

Replace hardcoded values in the constructor body:
- `FactorGraph::new(0.0, 100.0)` → `FactorGraph::new(0.0, config.init_variance, config.msg_damping)`
- `DynamicsFactor::new([k, k + 1], DT, 0.5, 0.0)` → `DynamicsFactor::new([k, k + 1], config.gbp_timestep, config.sigma_dynamics, 0.0)`
- `VelocityBoundFactor::new([k, k + 1], DT, 2.5)` → `VelocityBoundFactor::new([k, k + 1], config.gbp_timestep, config.v_max_default, config.v_min, config.vb_kappa, config.vb_margin, config.vb_max_precision)`
- `DynamicConstraints::new(2.5, 5.0, 2.5)` → `DynamicConstraints::new(config.max_accel, config.max_jerk, config.max_speed, config.v_min)`
- Store `config: *config` in Self

Replace hardcoded values throughout the step/update methods:
- `D_SAFE` → `self.config.d_safe`
- `SIGMA_R` → `self.config.sigma_interrobot`
- `IR_ACTIVATION_RANGE` → `self.config.ir_activation_range`
- `IR_RAMP_START` → `2.0 * self.config.d_safe` (derived, not a separate constant)
- `FRONT_DAMPING` (line ~432) → `self.config.front_damping`
- `GBP_INTERNAL_ITERS` → `self.config.internal_iters as usize`
- `GBP_EXTERNAL_ITERS` → `self.config.external_iters as usize`
- Anchor precision `1000.0` (line ~216-219) → `self.config.anchor_precision`

For `InterRobotFactor::new()` calls (uses single `var_idx_a: usize`, not an array), pass `self.config.ir_decay_alpha`:
```rust
InterRobotFactor::new(k, self.config.d_safe, self.config.sigma_interrobot, self.config.ir_decay_alpha)
```

For the anchor precision replacement (lines ~216-219), replace ALL occurrences of the literal `1000.0`:
```rust
// Before:
self.graph.variables[0].prior_eta = self.position_s * 1000.0;
self.graph.variables[0].prior_lambda = 1000.0;
// After:
self.graph.variables[0].prior_eta = self.position_s * self.config.anchor_precision;
self.graph.variables[0].prior_lambda = self.config.anchor_precision;
```

**Note:** There are ~7 test call sites for `AgentRunner::new` in `agent_runner.rs` — all must be updated to pass `&GbpConfig::default()`.

- [ ] **Step 3: Build and test**

```bash
cargo test -p gbp-agent
```

Fix compilation errors — any test that constructs `RobotAgent` directly will need to pass `&GbpConfig::default()`.

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-agent/src/robot_agent.rs crates/gbp-agent/src/dynamic_constraints.rs
git commit -m "refactor(config): wire GbpConfig into RobotAgent and DynamicConstraints"
```

---

## Task 4: Update simulator to pass `GbpConfig` through

**Files:**
- Modify: `src/bins/simulator/src/agent_runner.rs`
- Modify: `src/bins/simulator/src/main.rs`

This task just threads `GbpConfig` through without adding TOML parsing yet — uses `GbpConfig::default()`.

- [ ] **Step 1: Update AgentRunner::new to accept &GbpConfig**

In `src/bins/simulator/src/agent_runner.rs`, change:
```rust
pub fn new(comms: SimComms, map: Arc<Map>, robot_id: u32, config: &gbp_core::GbpConfig) -> Self {
    let agent = RobotAgent::new(comms, &*map, robot_id, config);
```

- [ ] **Step 2: Update agent_task to accept &GbpConfig if needed**

If `agent_task` constructs anything that needs config, update it. Otherwise just update the `AgentRunner::new` call sites in `main.rs`.

- [ ] **Step 3: Update main.rs spawn loop**

In the unified spawn loop, change:
```rust
let config = gbp_core::GbpConfig::default();
// ... in the loop:
let mut runner = AgentRunner::new(comms, map_arc.clone(), i as u32, &config);
```

- [ ] **Step 4: Build and test**

```bash
cargo test --workspace
```

Everything should pass identically — behavior unchanged, just threaded the default config through.

- [ ] **Step 5: Commit**

```bash
git add src/bins/simulator/src/agent_runner.rs src/bins/simulator/src/main.rs
git commit -m "refactor(config): thread GbpConfig::default() through simulator"
```

---

## Task 5: Add TOML parsing and config file loading

**Files:**
- Create: `src/bins/simulator/src/toml_config.rs`
- Modify: `Cargo.toml` (workspace)
- Modify: `src/bins/simulator/Cargo.toml`
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Add toml dependency**

In root `Cargo.toml`, add to `[workspace.dependencies]`:
```toml
toml = "0.8"
```

In `src/bins/simulator/Cargo.toml`, add:
```toml
toml = { workspace = true }
```

- [ ] **Step 2: Create toml_config.rs**

```rust
// src/bins/simulator/src/toml_config.rs

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
    assert!(c.msg_damping >= 0.0 && c.msg_damping <= 1.0, "msg_damping must be 0.0..=1.0, got {}", c.msg_damping);
    assert!(c.internal_iters >= 1, "internal_iters must be >= 1");
    assert!(c.external_iters >= 1, "external_iters must be >= 1");
    assert!(c.sigma_dynamics > 0.0, "sigma_dynamics must be > 0.0");
    assert!(c.gbp_timestep > 0.0, "gbp_timestep must be > 0.0");
    assert!(c.d_safe > 0.0, "d_safe must be > 0.0");
    assert!(c.sigma_interrobot > 0.0, "sigma_interrobot must be > 0.0");
    assert!(c.ir_activation_range > 0.0, "ir_activation_range must be > 0.0");
    assert!(c.v_min < c.v_max_default, "v_min ({}) must be < v_max_default ({})", c.v_min, c.v_max_default);
    assert!(c.max_accel > 0.0, "max_accel must be > 0.0");
    assert!(c.max_jerk > 0.0, "max_jerk must be > 0.0");
    assert!(c.max_speed > 0.0, "max_speed must be > 0.0");
    assert!(c.init_variance > 0.0, "init_variance must be > 0.0");
    assert!(c.anchor_precision > 0.0, "anchor_precision must be > 0.0");
    assert!(c.ir_decay_alpha > 0.0, "ir_decay_alpha must be > 0.0");
    assert!(c.front_damping >= 0.0 && c.front_damping <= 1.0, "front_damping must be 0.0..=1.0, got {}", c.front_damping);
    assert!(c.vb_kappa > 0.0, "vb_kappa must be > 0.0");
    assert!(c.vb_margin > 0.0, "vb_margin must be > 0.0");
    assert!(c.vb_max_precision > 0.0, "vb_max_precision must be > 0.0");
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
        assert_eq!(config.internal_iters, 3); // default
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
}
```

- [ ] **Step 3: Register module in main.rs**

Add `mod toml_config;` at the top of `src/bins/simulator/src/main.rs`.

- [ ] **Step 4: Build and test**

```bash
cargo test -p simulator
```

- [ ] **Step 5: Commit**

```bash
git add Cargo.toml src/bins/simulator/Cargo.toml src/bins/simulator/src/toml_config.rs src/bins/simulator/src/main.rs
git commit -m "feat(config): add TOML config and scenario parsing for simulator"
```

---

## Task 6: Return name→NodeId lookup from parser

**Files:**
- Modify: `crates/gbp-map/src/parser.rs`

- [ ] **Step 1: Change parse_yaml return type**

The parser already builds `node_id_map: HashMap<String, u16>` internally. Change the return type to include it:

```rust
pub fn parse_yaml(yaml: &str) -> Result<(Map, std::collections::HashMap<String, NodeId>), String> {
```

At the end of the function, convert the `HashMap<String, u16>` to `HashMap<String, NodeId>`:
```rust
let name_lookup: std::collections::HashMap<String, NodeId> = node_id_map
    .into_iter()
    .map(|(name, id)| (name, NodeId(id)))
    .collect();
Ok((map, name_lookup))
```

- [ ] **Step 2: Update all callers**

In `src/bins/simulator/src/main.rs`, change:
```rust
let map = gbp_map::parser::parse_yaml(&yaml)
```
to:
```rust
let (map, node_names) = gbp_map::parser::parse_yaml(&yaml)
```

In `src/bins/visualiser/src/main.rs`, change similarly (ignore the `node_names` with `_`):
```rust
let (map, _node_names) = gbp_map::parser::parse_yaml(&yaml)
```

- [ ] **Step 3: Build and test**

```bash
cargo test --workspace
```

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-map/src/parser.rs src/bins/simulator/src/main.rs src/bins/visualiser/src/main.rs
git commit -m "refactor(config): return name→NodeId lookup from parse_yaml"
```

---

## Task 7: Create config and scenario TOML files

**Files:**
- Create: `config/config.toml`
- Create: `config/scenarios/merge.toml`
- Create: `config/scenarios/endcollision.toml`
- Create: `config/scenarios/fleet_4.toml`

- [ ] **Step 1: Create config/config.toml**

```toml
# GBP Trajectory Planner — System Configuration
# All values shown are defaults. Remove or comment out lines to use defaults.

[gbp]
msg_damping      = 0.5
internal_iters   = 3
external_iters   = 5
timestep         = 0.1
init_variance    = 100.0
anchor_precision = 1000.0

[gbp.dynamics]
sigma = 0.5

[gbp.interrobot]
d_safe           = 1.3
sigma            = 0.12
activation_range = 3.0
decay_alpha      = 3.0
front_damping    = 0.3

[gbp.velocity_bound]
v_min         = -0.3
v_max_default = 2.5
kappa         = 10.0
margin        = 1.0
max_precision = 100.0

[robot]
max_accel = 2.5
max_jerk  = 5.0
max_speed = 2.5
v_min     = -0.3
```

- [ ] **Step 2: Create config/scenarios/merge.toml**

```toml
[scenario]
name = "merge"
map  = "maps/test_loop_map.yaml"

[[robots]]
start = "P004"
goal  = "P042"

[[robots]]
start = "P033"
goal  = "P042"
```

Note: Verify the actual goal node name in the map YAML. If `P042` doesn't exist, use the correct name for the last node (currently `default_goal` = last node in the map).

- [ ] **Step 3: Create config/scenarios/endcollision.toml**

```toml
[scenario]
name = "endcollision"
map  = "maps/test_loop_map.yaml"

[[robots]]
start = "P025"
goal  = "P012"

[[robots]]
start = "P024"
goal  = "P012"
```

Note: `P024` is the predecessor of `P025` — verify in the map YAML.

- [ ] **Step 4: Create config/scenarios/fleet_4.toml**

```toml
[scenario]
name       = "fleet"
map        = "maps/test_loop_map.yaml"
auto_assign = true
num_robots  = 4
```

- [ ] **Step 5: Commit**

```bash
git add config/
git commit -m "feat(config): add default config.toml and scenario files"
```

---

## Task 8: Wire TOML loading into simulator main

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

This replaces the hardcoded scenario logic with TOML-driven configuration.

- [ ] **Step 1: Update CLI args**

Replace the current `Args` struct:
```rust
#[derive(Parser)]
struct Args {
    /// Path to system config TOML (omit for defaults)
    #[arg(long)]
    config: Option<std::path::PathBuf>,
    /// Path to scenario TOML (if omitted, runs fleet of 4 on first available map)
    #[arg(long)]
    scenario: Option<std::path::PathBuf>,
    /// Address to bind the WebSocket server
    #[arg(long, default_value = "0.0.0.0:3000")]
    bind: String,
}
```

- [ ] **Step 2: Replace map loading and scenario logic**

In main(), replace the current map loading + scenario matching with:

```rust
// Load config
let config = if let Some(ref path) = args.config {
    let toml_str = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("cannot read config {}: {}", path.display(), e));
    toml_config::parse_config(&toml_str)
} else {
    gbp_core::GbpConfig::default()
};

// Load scenario
let scenario_str = std::fs::read_to_string(&args.scenario)
    .unwrap_or_else(|e| panic!("cannot read scenario {}: {}", args.scenario.display(), e));
let scenario = toml_config::parse_scenario(&scenario_str);

// Load map (path from scenario file)
let yaml = std::fs::read_to_string(&scenario.scenario.map)
    .unwrap_or_else(|e| panic!("cannot read map {}: {}", scenario.scenario.map, e));
let (map, node_names) = gbp_map::parser::parse_yaml(&yaml)
    .unwrap_or_else(|e| panic!("map parse error: {}", e));
```

- [ ] **Step 3: Replace assignment building with scenario-driven logic**

Replace the current `let assignments: Vec<(NodeId, NodeId)> = match args.scenario.as_str() { ... }` block with:

```rust
let assignments: std::vec::Vec<(NodeId, NodeId)> = if !scenario.robots.is_empty() {
    // Explicit robot assignments from scenario file
    scenario.robots.iter().map(|r| {
        let start = *node_names.get(&r.start)
            .unwrap_or_else(|| panic!("scenario node '{}' not found in map", r.start));
        let goal = *node_names.get(&r.goal)
            .unwrap_or_else(|| panic!("scenario node '{}' not found in map", r.goal));
        (start, goal)
    }).collect()
} else if scenario.scenario.auto_assign {
    let starts = pick_start_nodes(&map_arc, scenario.scenario.num_robots);
    starts.iter().enumerate().filter_map(|(i, &s)| {
        pick_random_goal(&map_arc, s, i as u32).map(|g| (s, g))
    }).collect()
} else {
    panic!("scenario must have [[robots]] entries or auto_assign = true");
};
```

- [ ] **Step 4: Pass config to spawn loop**

Update the spawn loop to use `&config`:
```rust
let mut runner = AgentRunner::new(comms, map_arc.clone(), i as u32, &config);
```

- [ ] **Step 5: Remove now-unused imports and functions**

Remove `pick_start_nodes`, `pick_random_goal`, `find_predecessor` if they are only used in the old hardcoded scenario matching (keep them if still used by auto_assign). Remove unused `NodeType` import if scenarios no longer reference it directly.

- [ ] **Step 6: Build and test**

```bash
cargo test --workspace
```

- [ ] **Step 7: Commit**

```bash
git add src/bins/simulator/src/main.rs
git commit -m "feat(config): load config and scenario from TOML files in simulator"
```

---

## Task 9: End-to-end verification

- [ ] **Step 1: Run all tests**

```bash
cargo test --workspace
```

- [ ] **Step 2: Run merge scenario**

```bash
cargo run -p simulator -- --config config/config.toml --scenario config/scenarios/merge.toml &
sleep 1 && cargo run -p visualiser
```

Verify: 2 robots from different start points, same goal. No D_SAFE hack.

- [ ] **Step 3: Run fleet scenario**

```bash
cargo run -p simulator -- --config config/config.toml --scenario config/scenarios/fleet_4.toml &
sleep 1 && cargo run -p visualiser
```

Verify: 4 robots, distributed starts, random goals.

- [ ] **Step 4: Run without --config (defaults)**

```bash
cargo run -p simulator -- --scenario config/scenarios/fleet_4.toml &
```

Verify: works identically to specifying the config file with default values.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(config): TOML config system — end-to-end verified"
```
