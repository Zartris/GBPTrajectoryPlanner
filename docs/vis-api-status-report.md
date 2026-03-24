# VisApi Event System & Addon Expansion -- Status Report

**Branch:** `feature/vis-addon-api`
**Date:** 2026-03-24
**Build:** `cargo build -p visualiser` -- PASS (0 warnings)
**Tests:** `cargo test -p visualiser` -- 47/47 pass

---

## What Was Built

### 1. Event System (`vis_events.rs`)

Three Bevy `Message` types (buffered, pull-based) that addons subscribe to via `MessageReader<T>`:

| Message | Emitted When | Fields |
|---------|-------------|--------|
| `ProximityAlert` | Two robots closer than `ir_d_safe` | `robot_a`, `robot_b`, `distance`, `d_safe`, `position_a`, `position_b` |
| `DataReceived` | New WS data processed | `tick`, `robot_count`, `total_ir_factors`, `min_pair_distance` |
| `RobotStateChanged` | Robot state changes | `robot_id`, `event_type: RobotChangeType` |

`RobotChangeType` variants: `Connected`, `Disconnected`, `EdgeChanged{from, to}`, `FactorCountChanged{from, to}`, `NearCollision{distance}`.

### 2. Event Emission System (`vis_event_systems.rs`)

Single `emit_events_on_data` system registered by `VisEventPlugin`, running in `Update` with `resource_changed::<RobotStates>` run condition. Only executes when `drain_ws_inbox` has processed new WebSocket data:

- **Proximity check**: O(n^2) pairwise, rate-limited to 1 alert/sec per pair via `ProximityRateLimiter` (HashMap with stale-entry pruning).
- **State change detection**: Compares current `RobotStates` against a `Local<PreviousRobotStates>` snapshot. Detects connects, disconnects, edge transitions, factor count changes, and near-collision rising edges.
- **DataReceived summary**: Aggregates robot count, total IR factors, and min pair distance into a single message.

Replaces three per-frame systems (`proximity_emitter`, `sim_tick_emitter`, `state_change_emitter`) with zero overhead on idle frames.

### 3. Config-Based Addon Settings (`addon_config.rs`)

All addon settings live in `[addons]` section of `config/config.toml`, parsed into an `AddonConfig` resource:

| Addon | Config Section | Behavior |
|-------|---------------|----------|
| `StartupScreenshotAddon` | `[addons.screenshot]` | Screenshot after delay, optional quit |
| `DebugMonitorAddon` | `[addons.debug_monitor]` | Periodic robot state logging |
| `ProximityScreenshotAddon` | `[addons.proximity_screenshot]` | Screenshots + logs on `ProximityAlert` |
| `StateChangeLoggerAddon` | `[addons.state_change_logger]` | Logs all `RobotStateChanged` messages |

All addons registered unconditionally. Each checks `config.enabled` at runtime, allowing toggle via the Addons UI panel without restart. Env var configuration removed.

### 4. VisApi Expansion

New methods added to the `VisApi` SystemParam:

| Method | Description |
|--------|-------------|
| `map_node_count()` | Number of nodes in loaded map |
| `map_edge_count()` | Number of edges in loaded map |
| `map_id()` | Map identifier string |
| `camera_position()` | Current camera world-space `[x, y, z]` |
| `ir_d_safe()` | Safety distance threshold (metres) |
| `robot_raw_gbp_velocity(id)` | Raw GBP velocity before clamping |

### 5. Documentation

- `vis_api.rs`: Module-level doc with API reference table, two usage patterns (polling vs event-driven), code examples
- `vis_events.rs`: Module doc with event table, per-event doc with examples
- `vis_event_systems.rs`: Architecture diagram, per-system documentation
- `addons/mod.rs`: Step-by-step addon creation guide, event-driven addon example, built-in addon table

---

## What Works

- All 47 unit tests pass
- Clean build with 0 warnings from the visualiser crate
- Event types are `derive(Message)` (Bevy 0.18 buffered messages)
- Rate-limiting prevents proximity alert spam (1/sec per pair)
- State change detection uses rising-edge for NearCollision (only emits on transition into danger zone)
- Stale rate-limiter entries are pruned to prevent memory leaks
- Change-triggered emission: zero overhead on idle frames (runs only when RobotStates mutated)
- Config-based addon settings: TOML config + runtime UI toggles (sliders, text fields)
- All addons registered unconditionally, check config.enabled at runtime
- Addons can be toggled on/off at runtime without restart

---

## Known Limitations

1. **O(n^2) proximity check**: The proximity and min-pair-distance computations iterate all robot pairs. Fine for the expected fleet size (4-8 robots), but would need spatial indexing for 100+.

2. **No integration tests for emission systems**: Unit tests cover helpers (dist_3d, constructibility, snapshot clone). Full emission testing would require a Bevy App harness with World + Schedule, which is not currently set up.

3. **DataReceived emitted per WS batch**: Replaced per-frame SimTickEvent. Now only fires when drain_ws_inbox processes new data.

4. **No event persistence/replay**: Events are consumed once per frame. There is no event log or replay buffer for post-hoc analysis.

5. **MapRes is read-only**: The map data access methods provide counts and IDs only, not full geometry queries (edge positions, NURBS evaluation). Adding `eval_position(edge_id, s)` to VisApi would require exposing more of the map API.

---

## What's Missing (Deferred)

| Item | Target Milestone | Notes |
|------|-----------------|-------|
| WebSocket event forwarding | M6b | Stream events to external AI agents via WS |
| Event log/replay | M6c | Record events for post-hoc analysis |
| Custom factor injection | M6d | AI agent creates/removes IR factors via VisApi |
| Spatial indexing for proximity | M6c | Replace O(n^2) with grid/kd-tree if fleet > 16 |
| Full map geometry in VisApi | M6b | `eval_position(edge_id, s)` and path queries |
| Screenshot verification test | M6b | Automated headless screenshot + pixel comparison |

---

## API Reference

### VisApi Methods (25 total)

#### Screenshot & Logging
| Method | Signature | Description |
|--------|-----------|-------------|
| `screenshot` | `(&mut self, path: impl Into<String>)` | Capture window to PNG |
| `log` | `(&self, msg: &str)` | Info log with `[addon]` tag |

#### Draw Config
| Method | Signature | Description |
|--------|-----------|-------------|
| `set_draw` | `(&mut self, field: &str, on: bool)` | Toggle named draw layer |
| `set_draw_all` | `(&mut self, on: bool)` | Toggle all layers |
| `ir_d_safe` | `(&self) -> f32` | Safety distance threshold |

#### Simulation State
| Method | Signature | Description |
|--------|-----------|-------------|
| `is_paused` | `(&self) -> bool` | Local pause flag |
| `pause` | `(&mut self)` | Set paused |
| `resume` | `(&mut self)` | Clear paused |

#### Time
| Method | Signature | Description |
|--------|-----------|-------------|
| `elapsed_secs` | `(&self) -> f32` | Seconds since app start |

#### Camera Control
| Method | Signature | Description |
|--------|-----------|-------------|
| `set_camera_orbit` | `(&mut self, yaw, pitch, distance)` | Orbit mode |
| `reset_camera` | `(&mut self)` | Reset to initial |
| `follow_robot` | `(&mut self, robot_id: u32)` | Follow mode |
| `exit_follow` | `(&mut self)` | Back to orbit |
| `camera_position` | `(&self) -> [f32; 3]` | Current camera position |

#### Robot State Queries
| Method | Signature | Description |
|--------|-----------|-------------|
| `robot_count` | `(&self) -> usize` | Number of tracked robots |
| `robot_ids` | `(&self) -> Vec<u32>` | Sorted list of IDs |
| `robot_position` | `(&self, id) -> Option<[f32; 3]>` | 3D world position |
| `robot_velocity` | `(&self, id) -> Option<f32>` | Scalar velocity |
| `robot_edge` | `(&self, id) -> Option<(EdgeId, f32)>` | Current edge + arc-length |
| `robot_factor_count` | `(&self, id) -> Option<usize>` | Active IR factors |
| `robot_min_distance` | `(&self, id) -> Option<f32>` | Min distance to neighbour |
| `robot_raw_gbp_velocity` | `(&self, id) -> Option<f32>` | Raw GBP velocity |

#### Map Data
| Method | Signature | Description |
|--------|-----------|-------------|
| `map_node_count` | `(&self) -> usize` | Nodes in loaded map |
| `map_edge_count` | `(&self) -> usize` | Edges in loaded map |
| `map_id` | `(&self) -> &str` | Map identifier string |

#### WebSocket
| Method | Signature | Description |
|--------|-----------|-------------|
| `send_sim_command` | `(&mut self, json: &str)` | Send raw JSON to simulator |
| `pause_sim` | `(&mut self)` | Send pause command |
| `resume_sim` | `(&mut self)` | Send resume command |

#### App Lifecycle
| Method | Signature | Description |
|--------|-----------|-------------|
| `quit` | `(&mut self)` | Clean exit |

---

## Event Reference

### ProximityAlert

Emitted when two robots are closer than `DrawConfig::ir_d_safe`. Rate-limited to once per second per robot pair.

```rust
fn on_proximity(
    mut api: VisApi,
    mut alerts: MessageReader<ProximityAlert>,
) {
    for alert in alerts.read() {
        api.log(&format!(
            "robots {} and {} are {:.2}m apart (safety: {:.2}m)",
            alert.robot_a, alert.robot_b, alert.distance, alert.d_safe,
        ));
        api.screenshot("/tmp/proximity.png");
    }
}
```

### DataReceived

Emitted when new WebSocket data is processed (replaces per-frame SimTickEvent). Only fires when `drain_ws_inbox` mutates `RobotStates`.

```rust
fn on_data(mut batches: MessageReader<DataReceived>) {
    for batch in batches.read() {
        if batch.tick % 60 == 0 {
            tracing::info!("tick {} -- {} robots, min_dist={:.2}m",
                batch.tick, batch.robot_count, batch.min_pair_distance);
        }
    }
}
```

### RobotStateChanged

Emitted when a robot connects, disconnects, changes edge, changes factor count, or crosses the near-collision threshold.

```rust
fn on_state_change(mut changes: MessageReader<RobotStateChanged>) {
    for change in changes.read() {
        match &change.event_type {
            RobotChangeType::Connected =>
                tracing::info!("Robot {} connected", change.robot_id),
            RobotChangeType::Disconnected =>
                tracing::info!("Robot {} disconnected", change.robot_id),
            RobotChangeType::EdgeChanged { from, to } =>
                tracing::info!("Robot {} edge {:?} -> {:?}", change.robot_id, from, to),
            RobotChangeType::FactorCountChanged { from, to } =>
                tracing::info!("Robot {} factors {} -> {}", change.robot_id, from, to),
            RobotChangeType::NearCollision { distance } =>
                tracing::warn!("Robot {} near collision: {:.3}m", change.robot_id, distance),
        }
    }
}
```

---

## How to Create a New Addon (Step by Step)

1. **Create** `src/bins/visualiser/src/addons/my_addon.rs`

2. **Add** `mod my_addon;` to `addons/mod.rs`

3. **Write the plugin** (always registers systems; checks config at runtime):

```rust
use bevy::prelude::*;
use crate::addon_config::AddonConfig;
use crate::vis_api::VisApi;

pub struct MyAddon;

impl Plugin for MyAddon {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, my_system);
    }
}

fn my_system(api: VisApi, config: Res<AddonConfig>) {
    if !config.my_addon.enabled { return; }
    api.log(&format!("t={:.1}s robots={}", api.elapsed_secs(), api.robot_count()));
}
```

4. **Add config struct** to `addon_config.rs` and add a field to `AddonConfig`.

5. **Add config section** to `config/config.toml` under `[addons.my_addon]`.

6. **Add UI controls** in the Addons section of `ui.rs`.

7. **Register** in `AddonPlugins::build`:

```rust
app.add_plugins(my_addon::MyAddon);
```

5. **For event-driven addons**, add `MessageReader<T>` parameters:

```rust
use bevy::ecs::message::MessageReader;
use crate::vis_events::ProximityAlert;

fn my_reactive_system(
    mut api: VisApi,
    mut alerts: MessageReader<ProximityAlert>,
) {
    for alert in alerts.read() {
        api.log(&format!("Alert: r{} <-> r{}", alert.robot_a, alert.robot_b));
    }
}
```

---

## File Inventory

| File | Lines | Purpose |
|------|-------|---------|
| `addon_config.rs` | ~175 | AddonConfig resource + serde deserialization |
| `vis_events.rs` | ~242 | Event message type definitions (DataReceived, ProximityAlert, RobotStateChanged) |
| `vis_event_systems.rs` | ~389 | Single change-triggered emission system + VisEventPlugin |
| `vis_api.rs` | ~409 | VisApi SystemParam (expanded) |
| `addons/mod.rs` | ~113 | Addon registry + documentation |
| `addons/startup_screenshot.rs` | ~92 | Config-driven screenshot after delay |
| `addons/debug_monitor.rs` | ~72 | Config-driven periodic robot state logging |
| `addons/proximity_screenshot.rs` | ~84 | Config-driven screenshot on proximity alert |
| `addons/state_change_logger.rs` | ~82 | Config-driven state change logger |

**New tests:** 21 (addon_config: 6, events: 4, emission systems: 8, addons: 2, vis_api: 1).

---

## Review History

### Internal Review (Opus)
**Verdict:** APPROVED with minor revisions
- **Fixed I-1:** debug_monitor proximity spam → now uses ProximityAlert events (rate-limited)
- **Fixed I-3:** Clippy warning in proximity_emitter → iterator instead of index
- **Fixed M-7:** Added PartialEq to RobotChangeType for ergonomic matching
- **Deferred I-4:** send_sim_command could take &self instead of &mut self (interior mutability)

### Copilot Reviews (4 rounds total)
- **Round 1:** 8 comments — all addressed (set_draw_all struct literal, env var caching, Python compat, etc.)
- **Round 2:** 2 comments — both fixed (O(n²) duplication eliminated, addons gated behind env vars)
- All comments replied to on GitHub

### Refactor Session (2026-03-24)
- Replaced env-var addon configuration with TOML-based `AddonConfig` resource
- Refactored 3 per-frame emission systems into 1 change-triggered system
- Renamed `SimTickEvent` to `DataReceived` (emitted per WS batch, not per frame)
- Added Addons UI panel with runtime toggles, sliders, and text fields
- All addons now registered unconditionally; check `config.enabled` at runtime
- 47 tests pass, 0 warnings

### Backup
Branch `vis-addon-api-backup` at commit `2f5e29c` preserves the pre-event-system state for rollback if needed.
