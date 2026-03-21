# Config System Design

## Goal

Replace hardcoded constants across the GBP crates and simulator with a TOML-based config system. Separate scenario definitions (which robots, where they start) from system tuning parameters (factor weights, velocity limits, iteration counts).

## Architecture

Two config files, both TOML:

1. **`config/config.toml`** — system-wide GBP/physics/safety parameters
2. **`config/scenarios/<name>.toml`** — per-run scenario (map, robot assignments)

Both are passed as CLI args: `--config config/config.toml --scenario config/scenarios/merge.toml`

A `GbpConfig` struct in `gbp-core` (no_std, Copy) holds all tunable parameters. The simulator parses TOML files and constructs this struct. Core crates never see TOML — they receive `GbpConfig` by value.

## Config Struct

Lives in `crates/gbp-core/src/config.rs`. All fields are `f32` or `u8` — no_std compatible.

```rust
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GbpConfig {
    // -- GBP solver --
    pub msg_damping: f32,          // 0.5  — message update damping (1.0=none, 0.5=avg old/new)
    pub internal_iters: u8,        // 3    — dynamics+velocity iters per external round
    pub external_iters: u8,        // 5    — inter-robot factor rounds

    // -- Dynamics factor --
    pub sigma_dynamics: f32,       // 0.5  — dynamics factor noise std dev
    pub gbp_timestep: f32,         // 0.1  — seconds per GBP timestep

    // -- Inter-robot factor --
    pub d_safe: f32,               // 1.3  — minimum center-to-center clearance (m)
    pub sigma_interrobot: f32,     // 0.12 — IR factor noise std dev
    pub ir_activation_range: f32,  // 3.0  — spawn/despawn distance (m)
    pub ir_decay_alpha: f32,       // 3.0  — exponential decay rate for IR precision
    pub front_damping: f32,        // 0.3  — asymmetric Jacobian dampening for front robot

    // -- Velocity bounds (BIPM-inspired) --
    pub v_min: f32,                // -0.3 — minimum velocity (allows slow reverse)
    pub v_max_default: f32,        // 2.5  — default max velocity (overridden by edge speed)
    pub vb_kappa: f32,             // 10.0 — barrier steepness
    pub vb_margin: f32,            // 1.0  — distance from limit where barrier activates (m/s)
    pub vb_max_precision: f32,     // 100.0 — hard precision cap at constraint boundary

    // -- Dynamic constraints (post-GBP) --
    pub max_accel: f32,            // 2.5  — maximum acceleration (m/s^2)
    pub max_jerk: f32,             // 5.0  — maximum jerk (m/s^3)
    pub max_speed: f32,            // 2.5  — maximum speed cap (m/s)

    // -- Graph initialization --
    pub init_variance: f32,        // 100.0  — initial variable uncertainty
    pub anchor_precision: f32,     // 1000.0 — position anchor (variable[0]) prior strength
}
```

A `Default` impl provides the current hardcoded values, so the system works identically without a config file.

**Derived constants** (not in config, computed from config values):
- `IR_RAMP_START` = `2.0 * d_safe` — always derived, the separate constant is removed.
- `AGENT_DT` = `0.02` — fixed at 50 Hz, tied to the physics tick rate. Stays hardcoded as it is an infrastructure constant, not a tuning parameter.

**Validation:** The simulator validates config values after parsing, before constructing `GbpConfig`. Valid ranges:
- `msg_damping`: 0.0..=1.0
- `internal_iters`, `external_iters`: >= 1
- `sigma_*`: > 0.0
- `d_safe`: > 0.0
- `v_min` < `v_max_default`
- `max_accel`, `max_jerk`, `max_speed`: > 0.0
- `init_variance`: > 0.0
- `anchor_precision`: > 0.0

Invalid values cause the simulator to panic with a descriptive error message. Validation lives in the simulator only (std), not in the no_std struct.

## Config TOML Format

```toml
# config/config.toml

[gbp]
msg_damping    = 0.5
internal_iters = 3
external_iters = 5
timestep       = 0.1
init_variance  = 100.0
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

Note: `v_min` appears in both `[gbp.velocity_bound]` (BIPM barrier activation) and `[robot]` (post-GBP velocity clamping in `DynamicConstraints`). Both use the same value from `GbpConfig.v_min`.

## TOML Deserialization Strategy

The TOML file uses nested sections (`[gbp.dynamics]`, `[gbp.interrobot]`, etc.) but `GbpConfig` is a flat no_std struct that cannot derive `serde::Deserialize`.

The simulator defines an intermediate `TomlConfig` struct that mirrors the TOML hierarchy:

```rust
// src/bins/simulator/src/toml_config.rs (std-only)

#[derive(Deserialize)]
struct TomlConfig {
    gbp: GbpSection,
    robot: RobotSection,
}

#[derive(Deserialize)]
struct GbpSection {
    msg_damping: Option<f32>,
    internal_iters: Option<u8>,
    // ...
    dynamics: Option<DynamicsSection>,
    interrobot: Option<InterrobotSection>,
    velocity_bound: Option<VelocityBoundSection>,
}

// etc.
```

All fields are `Option<T>` so partial configs work — missing fields fall back to `GbpConfig::default()`. A `impl From<TomlConfig> for GbpConfig` converts from the nested TOML structure to the flat config struct.

## Scenario TOML Format

```toml
# config/scenarios/merge.toml

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

```toml
# config/scenarios/fleet_4.toml

[scenario]
name       = "fleet"
map        = "maps/test_loop_map.yaml"
auto_assign = true
num_robots  = 4
```

When `[[robots]]` entries are present, each defines a robot by node name. When absent and `auto_assign = true`, the fleet logic picks distributed starts and random goals.

**Precedence:** If both `[[robots]]` entries and `auto_assign = true` are present, `[[robots]]` wins and `auto_assign` is ignored. The `map` field always points to a YAML file (parsed at runtime by the existing `gbp_map::parser`).

## Node Name Resolution

Scenario files reference nodes by name (e.g. "P004"). The `Node` struct in `gbp-map` does **not** have a name field — the YAML parser reads the string ID (e.g. "P004") but converts it to a numeric `NodeId(u16)` and discards the string.

Rather than adding a name field to the no_std `Node` struct (which would bloat every node on firmware), the name-to-ID mapping lives entirely in the simulator:

1. Modify `gbp_map::parser::parse_yaml()` to return both the `Map` and a `HashMap<String, NodeId>` lookup table (std-only, behind the `parse` feature gate).
2. Simulator loads the map, gets the lookup table.
3. Simulator parses scenario file to get node name strings.
4. Simulator resolves each name via the lookup table.
5. If a name isn't found, panic with a descriptive error: `"scenario node 'P099' not found in map"`.

The `Node` struct and no_std `Map` are unchanged. The lookup table is a transient std artifact used only at startup.

## Data Flow

```
CLI: --config config/config.toml --scenario config/scenarios/merge.toml
                  |                              |
            parse TOML                     parse TOML
                  |                              |
            GbpConfig (Copy)          Vec<(NodeId, NodeId)>
                  |                              |
                  +--------> main() <-----------+
                               |
                    for each (start, goal):
                      AgentRunner::new(comms, map, id, &config)
                        -> RobotAgent::new(&config)
                             -> FactorGraph: uses config.msg_damping
                             -> DynamicsFactor: uses config.sigma_dynamics
                             -> VelocityBoundFactor: uses config.v_min, vb_kappa, ...
                             -> InterRobotFactor: uses config.ir_decay_alpha, sigma_interrobot
```

## File Map

| Action | File | Purpose |
|--------|------|---------|
| Create | `crates/gbp-core/src/config.rs` | `GbpConfig` struct + `Default` impl |
| Create | `src/bins/simulator/src/toml_config.rs` | `TomlConfig` intermediate serde structs + `From<TomlConfig> for GbpConfig` |
| Create | `config/config.toml` | Default system config |
| Create | `config/scenarios/merge.toml` | Merge scenario |
| Create | `config/scenarios/endcollision.toml` | Endcollision scenario |
| Create | `config/scenarios/fleet_4.toml` | 4-robot fleet scenario |
| Modify | `Cargo.toml` | Add `toml` to workspace dependencies |
| Modify | `src/bins/simulator/Cargo.toml` | Add `toml` dependency |
| Modify | `crates/gbp-core/src/lib.rs` | Export `config` module |
| Modify | `crates/gbp-core/src/factor_graph.rs` | Accept `msg_damping` param instead of const |
| Modify | `crates/gbp-core/src/interrobot_factor.rs` | Accept `decay_alpha` param instead of const |
| Modify | `crates/gbp-core/src/velocity_bound_factor.rs` | Accept config params in constructor |
| Modify | `crates/gbp-agent/src/robot_agent.rs` | Accept `&GbpConfig`, replace all hardcoded consts, remove `IR_RAMP_START` (derive from `2*d_safe`) |
| Modify | `crates/gbp-agent/src/dynamic_constraints.rs` | Accept `v_min`, `max_accel`, `max_jerk`, `max_speed` from config |
| Modify | `crates/gbp-map/src/parser.rs` | Return `HashMap<String, NodeId>` alongside `Map` (name lookup table) |
| Modify | `src/bins/simulator/src/main.rs` | Parse TOML files, validate config, construct GbpConfig, resolve node names |

## CLI Changes

Before:
```bash
cargo run -p simulator -- --map maps/test_loop_map.yaml --scenario merge --num-robots 4
```

After:
```bash
cargo run -p simulator -- --config config/config.toml --scenario config/scenarios/merge.toml
```

`--map` and `--num-robots` are removed — they live in the scenario file. `--bind` stays as a CLI arg (infrastructure, not simulation config).

If `--config` is omitted, use `GbpConfig::default()`. If `--scenario` is omitted, fall back to a default fleet scenario (4 robots on the first map found, or require the arg).

## Dependencies

- `toml = "0.8"` — added to workspace deps, used only by the simulator binary
- `serde` with `derive` — already in workspace (needed for TOML deserialization in simulator)
- No new dependencies for core crates

## Constraints

- `GbpConfig` is `#![no_std]` compatible — no String, Vec, or alloc types
- TOML parsing happens only in `std` binaries (simulator)
- Core crates receive `GbpConfig` by value/reference, never parse files
- All current hardcoded values become the `Default` impl — behavior is identical without a config file
