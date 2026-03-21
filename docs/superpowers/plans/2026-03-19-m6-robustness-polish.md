# M6: Robustness + Polish — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Blocked-edge handling (robots replan around blocked edges), live `ParameterUpdate` handling (σ_dyn, σ_r, d_safe applied immediately), map live-reload without simulator restart, log panel in visualiser.

**Architecture:** Simulator exposes a WebSocket control channel alongside the `RobotStateMsg` stream. Control messages (`BlockEdge`, `UnblockEdge`, `ParameterUpdate`, `ReloadMap`) are JSON-encoded and handled in a new `control_handler` task. Blocked edges are tracked in a `BlockedEdges` set; A* skips blocked edges. `ParameterUpdate` is forwarded to all `AgentRunner` instances via a tokio watch channel. Map reload reads the YAML file from disk and replaces the `Arc<Map>` live. The visualiser log panel consumes timestamped `LogMsg` WebSocket messages.

**Tech Stack:** `tokio::sync::watch` for parameter broadcast; `serde_json` for control messages (already a dep). No new crates.

**Prerequisite:** M5 plan complete and passing.

---

## File Structure

```
src/bins/simulator/
  src/
    main.rs            — updated: control channel, BlockedEdges, watch channel for params
    agent_runner.rs    — updated: apply_params(), uses BlockedEdges in A*
    sim_comms.rs       — unchanged

src/bins/visualiser/
  src/
    ui.rs              — updated: block/unblock edge control, log panel
    ws_client.rs       — updated: consume LogMsg messages
    state.rs           — updated: add log buffer
    main.rs            — unchanged
    robot_render.rs    — unchanged
    map_scene.rs       — unchanged
```

---

## Chunk 1: Simulator — blocked edges + parameter updates + map reload

### Task 1: `astar_filtered` in `gbp-map` (prerequisite for blocked edges)

**Files:**
- Modify: `crates/gbp-map/src/astar.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In crates/gbp-map/src/astar.rs tests
#[test]
fn astar_filtered_skips_blocked_edge() {
    // two_edge_map: node 0 --edge0--> node 1 --edge1--> node 2
    // When edge 0 is filtered out, no path from node 0 to node 2 exists.
    let map = two_edge_map(); // helper also used in M4 tests
    let result = astar_filtered(&map, NodeId(0), NodeId(2), |eid| eid != EdgeId(0));
    assert!(result.is_none(), "filtered edge should prevent route");

    // Without filter, path exists
    let full = astar_filtered(&map, NodeId(0), NodeId(2), |_| true);
    assert!(full.is_some(), "unfiltered should find path");
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p gbp-map 2>&1 | head -20
```
Expected: compile error — `astar_filtered` doesn't exist.

- [ ] **Step 2: Implement `astar_filtered` in `crates/gbp-map/src/astar.rs`**

```rust
/// A* variant that accepts an edge-filter predicate.
/// `allow_edge(EdgeId) -> bool` — return false to block that edge from the search.
/// All existing callers continue to use `astar(map, start, goal)` which calls
/// `astar_filtered(map, start, goal, |_| true)` internally.
pub fn astar_filtered<F>(
    map:        &Map,
    start:      NodeId,
    goal:       NodeId,
    allow_edge: F,
) -> Option<heapless::Vec<NodeId, { MAX_PATH_EDGES }>>
where
    F: Fn(EdgeId) -> bool,
{
    // Re-implement A* body here (copy from astar()), but skip any edge where
    // !allow_edge(edge.id) when expanding neighbours.
    // ...
}

/// Convenience wrapper preserving the original signature.
pub fn astar(map: &Map, start: NodeId, goal: NodeId) -> Option<heapless::Vec<NodeId, { MAX_PATH_EDGES }>> {
    astar_filtered(map, start, goal, |_| true)
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p gbp-map 2>&1 | grep -E "test |FAILED|passed"
```
Expected: `astar_filtered_skips_blocked_edge` PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-map/src/astar.rs
git commit -m "feat(m6): astar_filtered with edge-filter predicate in gbp-map"
```

---

### Task 2: Blocked-edge tracking and control_handler in simulator

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// In main.rs tests
#[test]
fn blocked_edges_excludes_blocked_from_astar() {
    // When an edge is blocked, A* should not route through it.
    let map = two_edge_map(); // edges 0 and 1 in series; only path is through both
    let mut blocked: heapless::Vec<EdgeId, 16> = heapless::Vec::new();
    blocked.push(EdgeId(0)).unwrap(); // block the first edge

    // A* with edge 0 blocked should find no path from node 0 to node 2
    let result = astar_with_blocked(&map, NodeId(0), NodeId(2), &blocked);
    assert!(result.is_none(), "expected no path when direct route is blocked");

    // Without blocking, path exists
    let unblocked_result = astar_with_blocked(&map, NodeId(0), NodeId(2), &[]);
    assert!(unblocked_result.is_some());
}

#[test]
fn control_handler_block_edge_mutates_blocked_set() {
    // Parsing a BlockEdge control message adds the edge_id to the blocked set.
    use std::sync::{Arc, Mutex};
    let blocked: Arc<Mutex<heapless::Vec<EdgeId, 16>>> =
        Arc::new(Mutex::new(heapless::Vec::new()));

    // Simulate what control_handler does on "BlockEdge" JSON:
    let json = r#"{"type":"BlockEdge","edge_id":3}"#;
    let cmd: ControlCommand = serde_json::from_str(json).unwrap();
    match cmd {
        ControlCommand::BlockEdge { edge_id } => {
            blocked.lock().unwrap().push(EdgeId(edge_id)).ok();
        }
        _ => panic!("wrong variant"),
    }

    assert!(blocked.lock().unwrap().contains(&EdgeId(3)));
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator 2>&1 | head -20
```
Expected: compile error — `astar_with_blocked`, `ControlCommand` don't exist.

- [ ] **Step 2: Implement `astar_with_blocked`, `ControlCommand`, and `BlockedEdges` resource**

```rust
/// A* that skips blocked edges. Thin wrapper over astar_filtered.
pub fn astar_with_blocked(
    map: &Map,
    start: NodeId,
    goal:  NodeId,
    blocked: &[EdgeId],
) -> Option<heapless::Vec<NodeId, { gbp_map::MAX_PATH_EDGES }>> {
    astar_filtered(map, start, goal, |edge_id| !blocked.contains(&edge_id))
}

// In main.rs: shared blocked-edges state
let blocked_edges: Arc<Mutex<heapless::Vec<EdgeId, 16>>> =
    Arc::new(Mutex::new(heapless::Vec::new()));

// Control message types (JSON-deserialized from the control WebSocket):
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(tag = "type")]
pub enum ControlCommand {
    BlockEdge   { edge_id: u32 },
    UnblockEdge { edge_id: u32 },
    SetParam    { key: String, value: f32 },
    ReloadMap,
}
```

In `control_handler` async task (new task spawned from `main()`):
```rust
async fn control_handler(
    mut ws:       WebSocket,
    blocked:      Arc<Mutex<heapless::Vec<EdgeId, 16>>>,
    params_tx:    tokio::sync::watch::Sender<AgentParams>,
    log_tx:       tokio::sync::broadcast::Sender<LogMsg>,
    map_path:     std::path::PathBuf,
    shared_map:   Arc<std::sync::RwLock<Arc<Map>>>,
) {
    while let Some(Ok(msg)) = ws.recv().await {
        if let Ok(text) = msg.to_text() {
            if let Ok(cmd) = serde_json::from_str::<ControlCommand>(text) {
                match cmd {
                    ControlCommand::BlockEdge { edge_id } => {
                        blocked.lock().unwrap().push(EdgeId(edge_id)).ok();
                    }
                    ControlCommand::UnblockEdge { edge_id } => {
                        let mut b = blocked.lock().unwrap();
                        b.retain(|&e| e != EdgeId(edge_id));
                    }
                    // SetParam and ReloadMap handled in subsequent tasks
                    _ => {}
                }
            }
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator -p gbp-map 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/main.rs crates/gbp-map/src/astar.rs
git commit -m "feat(m6): blocked-edge support in A* (astar_with_blocked, control_handler)"
```

---

### Task 3: `ParameterUpdate` handling via watch channel

**Files:**
- Modify: `src/bins/simulator/src/agent_runner.rs`
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In agent_runner.rs tests
// Note: tokio::sync::broadcast::channel is synchronous — no async runtime needed.
#[test]
fn apply_params_updates_all_fields() {
    let map = std::sync::Arc::new(straight_map());
    let (tx, rx) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
    let comms = SimComms::new(tx.clone(), rx);
    let mut runner = AgentRunner::new(comms, map.clone(), 0);
    runner.set_single_edge_trajectory(EdgeId(0), 5.0);

    // Apply a full parameter update
    let update = AgentParams { d_safe: Some(0.5), sigma_dyn: Some(1.2), sigma_r: Some(0.8) };
    runner.apply_params(update);

    let p = runner.current_params();
    assert!((p.d_safe    - 0.5).abs() < 1e-6, "d_safe mismatch");
    assert!((p.sigma_dyn - 1.2).abs() < 1e-6, "sigma_dyn mismatch");
    assert!((p.sigma_r   - 0.8).abs() < 1e-6, "sigma_r mismatch");
}

#[test]
fn apply_params_partial_update_leaves_others_unchanged() {
    let map = std::sync::Arc::new(straight_map());
    let (tx, rx) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
    let comms = SimComms::new(tx.clone(), rx);
    let mut runner = AgentRunner::new(comms, map.clone(), 0);
    runner.set_single_edge_trajectory(EdgeId(0), 5.0);

    let initial_sigma_r = runner.current_params().sigma_r;
    // Only update d_safe; sigma_dyn and sigma_r should remain unchanged
    runner.apply_params(AgentParams { d_safe: Some(0.4), sigma_dyn: None, sigma_r: None });
    assert!((runner.current_params().d_safe - 0.4).abs() < 1e-6);
    assert!((runner.current_params().sigma_r - initial_sigma_r).abs() < 1e-6);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator 2>&1 | head -20
```
Expected: compile error — `apply_params`, `AgentParams`, `current_params` don't exist.

- [ ] **Step 2: Implement `apply_params` and `AgentParams`**

```rust
// In agent_runner.rs

#[derive(Clone)]
pub struct AgentParams {
    pub d_safe:    Option<f32>,
    pub sigma_dyn: Option<f32>,
    pub sigma_r:   Option<f32>,
}

impl AgentRunner {
    pub fn apply_params(&mut self, params: AgentParams) {
        if let Some(d) = params.d_safe    { self.agent.set_d_safe(d); }
        if let Some(s) = params.sigma_dyn { self.agent.set_sigma_dyn(s); }
        if let Some(s) = params.sigma_r   { self.agent.set_sigma_r(s); }
    }

    pub fn current_params(&self) -> CurrentAgentParams {
        CurrentAgentParams {
            d_safe:    self.agent.d_safe(),
            sigma_dyn: self.agent.sigma_dyn(),
            sigma_r:   self.agent.sigma_r(),
        }
    }
}

pub struct CurrentAgentParams { pub d_safe: f32, pub sigma_dyn: f32, pub sigma_r: f32 }
```

In `main.rs`: use `tokio::sync::watch` for parameter broadcast to all runners:
```rust
let (params_tx, params_rx) = tokio::sync::watch::channel(AgentParams { d_safe: None, sigma_dyn: None, sigma_r: None });

// In each agent_task loop, check for param updates:
if params_rx.has_changed().unwrap_or(false) {
    let p = params_rx.borrow_and_update().clone();
    runner.lock().unwrap().apply_params(p);
}
```

Note: `RobotAgent` must expose `set_d_safe()`, `set_sigma_dyn()`, `set_sigma_r()`, `d_safe()`, `sigma_dyn()`, `sigma_r()` as mutable accessors on the factor parameters. Add these to `crates/gbp-agent/src/lib.rs`.

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/agent_runner.rs src/bins/simulator/src/main.rs crates/gbp-agent/src/lib.rs
git commit -m "feat(m6): ParameterUpdate handling via watch channel in AgentRunner"
```

---

### Task 4: Map live-reload

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In main.rs tests
#[test]
fn reload_map_returns_new_arc() {
    // Reloading a map from a path returns a new Arc<Map> distinct from the old one.
    // Use a temp file copy of test_loop_map.yaml for test isolation.
    use std::path::PathBuf;
    let path = PathBuf::from("maps/test_loop_map.yaml");
    if !path.exists() { return; } // skip if running without map file

    let map1 = load_map_arc(&path).expect("first load");
    let map2 = load_map_arc(&path).expect("second load");
    // Both loads produce equal but non-identical Arcs (different allocations)
    assert!(!Arc::ptr_eq(&map1, &map2), "expected distinct Arc allocations");
    assert_eq!(map1.nodes.len(), map2.nodes.len(), "node count must match after reload");
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator 2>&1 | head -20
```

- [ ] **Step 2: Implement `load_map_arc` and live-reload handler**

```rust
/// Load a Map from a YAML file and return it wrapped in Arc.
pub fn load_map_arc(path: &std::path::Path) -> anyhow::Result<Arc<Map>> {
    let map = gbp_map::parse_map_yaml(path)?;
    Ok(Arc::new(map))
}
```

In `control_handler` (new async task), the `ReloadMap` arm is added to the existing match on `ControlCommand`:
```rust
// Handling a ReloadMap control message in control_handler:
ControlCommand::ReloadMap => {
    match load_map_arc(&map_path) {
        Ok(new_map) => {
            *shared_map.write().unwrap() = new_map;
            log_tx.send(LogMsg::info("Map reloaded from disk")).ok();
            // Robots currently mid-trajectory continue on old Arc (safe: Arc ref-counting).
            // Next goal assignment picks up the new map.
        }
        Err(e) => { log_tx.send(LogMsg::error(format!("Map reload failed: {}", e))).ok(); }
    }
}
```

`shared_map` is an `Arc<RwLock<Arc<Map>>>`. `AgentRunner` reads `Arc<Map>` at goal assignment time; the live map pointer is swapped on reload without interrupting running robots.

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/main.rs
git commit -m "feat(m6): map live-reload via load_map_arc"
```

---

## Chunk 2: Visualiser — block/unblock control + log panel

### Task 5: Log panel in `ui.rs` + `state.rs`

**Files:**
- Modify: `src/bins/visualiser/src/state.rs`
- Modify: `src/bins/visualiser/src/ui.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// In state.rs tests
#[test]
fn log_buffer_appends_and_caps_at_max() {
    let mut buf = LogBuffer::default();
    for i in 0..LOG_BUFFER_MAX + 5 {
        buf.push(format!("msg {}", i));
    }
    assert_eq!(buf.messages.len(), LOG_BUFFER_MAX, "log buffer must not exceed max");
    // Oldest messages are dropped (ring buffer behaviour)
    assert!(buf.messages.last().unwrap().contains(&format!("{}", LOG_BUFFER_MAX + 4)));
}

// In ui.rs tests
#[test]
fn log_level_prefix_error() {
    assert_eq!(log_level_prefix("error"), "[ERR] ");
    assert_eq!(log_level_prefix("info"),  "[INF] ");
    assert_eq!(log_level_prefix("warn"),  "[WRN] ");
    assert_eq!(log_level_prefix("other"), "[LOG] ");
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib 2>&1 | head -20
```

- [ ] **Step 2: Implement `LogBuffer` and log panel**

```rust
// In state.rs
pub const LOG_BUFFER_MAX: usize = 200;

#[derive(Resource, Default)]
pub struct LogBuffer {
    pub messages: heapless::Vec<String, LOG_BUFFER_MAX>,
}

impl LogBuffer {
    pub fn push(&mut self, msg: String) {
        if self.messages.len() == LOG_BUFFER_MAX {
            // Drop oldest: shift left (O(N) but N=200 is acceptable for UI)
            self.messages.remove(0);
        }
        let _ = self.messages.push(msg);
    }
}
```

```rust
// In ui.rs
pub fn log_level_prefix(level: &str) -> &'static str {
    match level {
        "error" => "[ERR] ",
        "warn"  => "[WRN] ",
        "info"  => "[INF] ",
        _       => "[LOG] ",
    }
}

fn draw_log_panel(
    mut ctxs: EguiContexts,
    log:      Res<LogBuffer>,
) {
    let ctx = ctxs.ctx_mut();
    egui::Window::new("Log").default_size([600.0, 200.0]).show(ctx, |ui| {
        egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
            for msg in log.messages.iter() {
                ui.label(egui::RichText::new(msg).monospace().size(11.0));
            }
        });
    });
}
```

Register in `UiPlugin::build`:
```rust
app.init_resource::<LogBuffer>()
   .add_systems(Update, (draw_hud, draw_global_panel, draw_map_inspector, draw_log_panel));
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/state.rs src/bins/visualiser/src/ui.rs
git commit -m "feat(m6): log buffer and log panel in visualiser"
```

---

### Task 6: Block/unblock edge control in `ui.rs` + `ws_client.rs`

**Files:**
- Modify: `src/bins/visualiser/src/ui.rs`
- Modify: `src/bins/visualiser/src/ws_client.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// In ui.rs tests
#[test]
fn block_edge_command_serializes_correctly() {
    let cmd = ControlCommand::BlockEdge { edge_id: 3 };
    let json = serde_json::to_string(&cmd).unwrap();
    assert!(json.contains("\"BlockEdge\""));
    assert!(json.contains("\"edge_id\":3"));
}

#[test]
fn param_update_command_serializes_d_safe() {
    let cmd = ControlCommand::SetParam { key: "d_safe".into(), value: 0.5 };
    let json = serde_json::to_string(&cmd).unwrap();
    assert!(json.contains("\"d_safe\""));
    assert!(json.contains("0.5"));
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib ui 2>&1 | head -20
```

- [ ] **Step 2: Implement `ControlCommand` and visualiser control channel**

```rust
// In ui.rs (or a new control.rs module)
use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type")]
pub enum ControlCommand {
    BlockEdge   { edge_id: u32 },
    UnblockEdge { edge_id: u32 },
    SetParam    { key: String, value: f32 },
    ReloadMap,
}
```

In `draw_map_inspector`, add a Block/Unblock button for a selected edge:
```rust
if let Some(edge_id) = selected_edge.0 {
    if ui.button("Block edge").clicked() {
        control_tx.send(ControlCommand::BlockEdge { edge_id: edge_id.0 }).ok();
    }
    if ui.button("Unblock edge").clicked() {
        control_tx.send(ControlCommand::UnblockEdge { edge_id: edge_id.0 }).ok();
    }
}
```

In the global panel, add sliders that send `SetParam` on change:
```rust
// Already implemented in M5's GlobalParams; now wire slider changes to ControlCommand:
// TODO(M6): add an `on_change` hook so slider changes send SetParam via ws control channel.
```

`ws_client.rs` sends `ControlCommand` JSON to the simulator's control WebSocket endpoint alongside the `RobotStateMsg` stream.

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib ui 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/ui.rs src/bins/visualiser/src/ws_client.rs
git commit -m "feat(m6): block/unblock edge and parameter update control commands"
```

---

### Task 7: End-to-end verification

- [ ] **Step 1: Run all tests**

```bash
cargo test -p simulator -p visualiser -p gbp-map 2>&1 | grep -E "test |FAILED|passed|error"
```

- [ ] **Step 2: Manual verification**

```bash
cargo run -p simulator -- --map maps/test_loop_map.yaml --num-robots 4 &
cargo run -p visualiser
```

Verify:
- Block an edge while robots are running — affected robots replan around it
- Adjust d_safe slider — change in clearance visible in belief tubes within one cycle
- Reload map (via reload button or control command) — robots recover to valid positions
- Log panel shows timestamped messages from simulator
- Unblocking an edge re-enables routing through it

- [ ] **Step 3: Commit and tag M6**

```bash
git add -A
git commit -m "feat(m6): robustness and polish complete"
git tag m6-complete
```

---

## Deferred from M4: config.yaml for tuning parameters

- [ ] **config.yaml**: Create a `config.yaml` file that externalises all hardcoded tuning constants. Parsed in simulator/visualiser (PC-only), passed as structs to `RobotAgent::new()` and `DynamicConstraints::new()`. The no_std crates receive values, never parse.

Parameters to include:
```yaml
robot:
  chassis_length: 1.15
  chassis_width: 0.90
  chassis_height: 0.126
gbp:
  d_safe: 1.3
  sigma_r: 0.15
  sigma_dyn: 0.5
  iterations: 15
  dt: 0.1
  msg_damping: 0.5
  ir_activation_range: 3.0
  ir_ramp_start: 2.6
constraints:
  max_accel: 2.5
  max_jerk: 5.0
  max_speed: 2.5
simulator:
  agent_hz: 50
  physics_hz: 50
```

CLI: `--config config.yaml` (defaults to sensible hardcoded values if not provided).
