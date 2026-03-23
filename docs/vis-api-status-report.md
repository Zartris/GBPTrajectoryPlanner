# VisApi Event System & Addon Expansion -- Status Report

**Branch:** `feature/vis-addon-api`
**Date:** 2026-03-23
**Build:** `cargo build -p visualiser` -- PASS (0 warnings)
**Tests:** `cargo test -p visualiser` -- 41/41 pass

---

## What Was Built

### 1. Event System (`vis_events.rs`)

Three Bevy `Message` types (buffered, pull-based) that addons subscribe to via `MessageReader<T>`:

| Message | Emitted When | Fields |
|---------|-------------|--------|
| `ProximityAlert` | Two robots closer than `ir_d_safe` | `robot_a`, `robot_b`, `distance`, `d_safe`, `position_a`, `position_b` |
| `SimTickEvent` | Every frame | `tick`, `robot_count`, `total_ir_factors`, `min_pair_distance` |
| `RobotStateChanged` | Robot state changes | `robot_id`, `event_type: RobotChangeType` |

`RobotChangeType` variants: `Connected`, `Disconnected`, `EdgeChanged{from, to}`, `FactorCountChanged{from, to}`, `NearCollision{distance}`.

### 2. Event Emission Systems (`vis_event_systems.rs`)

Three systems registered by `VisEventPlugin`, all running in `Update`:

- **`proximity_emitter`**: O(n^2) pairwise check, rate-limited to 1 alert/sec per pair via `ProximityRateLimiter` (HashMap with stale-entry pruning).
- **`sim_tick_emitter`**: Emits every frame with aggregated data and a monotonic tick counter.
- **`state_change_emitter`**: Compares current `RobotStates` against a `Local<PreviousRobotStates>` snapshot. Detects connects, disconnects, edge transitions, factor count changes, and near-collision rising edges.

### 3. Example Addons

| Addon | Env Var | Behavior |
|-------|---------|----------|
| `ProximityScreenshotAddon` | `VIS_PROXIMITY_SCREENSHOT=1` | Screenshots + logs on every `ProximityAlert` |
| `StateChangeLoggerAddon` | `VIS_LOG_STATE_CHANGES=1` | Logs all `RobotStateChanged` messages |

Both are gated behind env vars (disabled by default), registered in `AddonPlugins`.

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

- All 41 unit tests pass
- Clean build with 0 warnings from the visualiser crate
- Event types are `derive(Message)` (Bevy 0.18 buffered messages)
- Rate-limiting prevents proximity alert spam (1/sec per pair)
- State change detection uses rising-edge for NearCollision (only emits on transition into danger zone, not every frame)
- Stale rate-limiter entries are pruned to prevent memory leaks
- All addons gated behind env vars so they cost nothing when disabled

---

## Known Limitations

1. **O(n^2) proximity check**: The proximity and min-pair-distance computations iterate all robot pairs. Fine for the expected fleet size (4-8 robots), but would need spatial indexing for 100+.

2. **No integration tests for emission systems**: Unit tests cover helpers (dist_3d, constructibility, snapshot clone). Full emission testing would require a Bevy App harness with World + Schedule, which is not currently set up.

3. **SimTickEvent emitted every frame**: No configurable decimation yet. Addons that want periodic updates can filter `tick % N == 0` themselves.

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

### SimTickEvent

Emitted every frame with aggregate data. Useful for periodic monitoring.

```rust
fn on_tick(mut ticks: MessageReader<SimTickEvent>) {
    for tick in ticks.read() {
        if tick.tick % 60 == 0 {
            tracing::info!("tick {} -- {} robots, min_dist={:.2}m",
                tick.tick, tick.robot_count, tick.min_pair_distance);
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

3. **Write the plugin:**

```rust
use bevy::prelude::*;
use crate::vis_api::VisApi;

pub struct MyAddon;

impl Plugin for MyAddon {
    fn build(&self, app: &mut App) {
        // Optional: gate behind env var
        let enabled = std::env::var("VIS_MY_ADDON")
            .map(|v| v == "1")
            .unwrap_or(false);
        if enabled {
            app.add_systems(Update, my_system);
        }
    }
}

fn my_system(mut api: VisApi) {
    api.log(&format!("t={:.1}s robots={}", api.elapsed_secs(), api.robot_count()));
}
```

4. **Register** in `AddonPlugins::build`:

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
| `vis_events.rs` | 241 | Event message type definitions |
| `vis_event_systems.rs` | 413 | Emission systems + VisEventPlugin |
| `vis_api.rs` | 301 | VisApi SystemParam (expanded) |
| `addons/mod.rs` | 114 | Addon registry + documentation |
| `addons/proximity_screenshot.rs` | 78 | Event-driven screenshot addon |
| `addons/state_change_logger.rs` | 75 | Event-driven state logger addon |

**Total new code:** ~920 lines (including tests and documentation).
**New tests:** 15 (events: 4, emission systems: 8, addons: 2, vis_api: 1).
