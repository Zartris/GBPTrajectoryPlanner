# M1: One Robot, One Edge, Visible — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Minimal end-to-end stack — one robot travels a straight edge at nominal velocity, rendered as an arrow in the Bevy visualiser.

**Architecture:** Two new crates: `simulator` (Tokio async, axum WebSocket, physics at 50 Hz) and `visualiser` (Bevy 0.15, WebSocket client via tokio-tungstenite). Per the spec, **no GBP runs in M1** — the robot velocity is set directly to `edge.speed.nominal`. Physics integrates `s += v * dt` at 50 Hz. A broadcast task reads position at 20 Hz and publishes `RobotStateMsg` over a tokio broadcast channel → axum WebSocket. `RobotAgent` is introduced in M2 when trajectory following and `DynamicsFactor` are needed.

**Tech Stack:** `tokio 1` (full features), `axum 0.7` (ws feature), `serde_json 1`, `bevy 0.15`, `bevy_egui 0.30`, `tokio-tungstenite 0.24`, `futures-util 0.3`, `gbp-comms`, `gbp-map`(+parse).

**Prerequisite:** M0 crates (`gbp-map`, `gbp-core`, `gbp-comms`, `gbp-agent`) compile and all tests pass.

---

## File Structure

```
src/bins/simulator/
  Cargo.toml
  src/
    main.rs             — CLI args, map load, spawn tasks
    physics.rs          — PhysicsState, physics_task (50 Hz integration)
    broadcast_task.rs   — broadcast_task (20 Hz): sets v=v_nom, publishes RobotStateMsg
    ws_server.rs        — axum WebSocket server, relays JSON broadcast channel

src/bins/visualiser/
  Cargo.toml
  src/
    main.rs            — Bevy App setup, plugin registration
    ws_client.rs       — background tokio thread, WebSocket → channel → Bevy event
    map_scene.rs       — startup system: spawn edge line meshes, node sphere meshes
    robot_render.rs    — update system: move robot arrow to position_s along edge tangent
    state.rs           — Bevy resources: MapRes, RobotStates, WsChannel
```

**Key cross-crate boundaries:**
- `simulator` depends on `gbp-comms`, `gbp-map` (with `parse` feature). No `gbp-agent` in M1 — GBP is introduced in M2.
- `visualiser` depends on `gbp-comms`, `gbp-map` (with `parse` feature); does NOT depend on `gbp-agent` or `gbp-core`

---

## Chunk 1: Simulator Crate

### Task 1: Register simulator in workspace and write Cargo.toml

**Files:**
- Modify: `Cargo.toml` (root workspace)
- Create: `src/bins/simulator/Cargo.toml`

- [ ] **Step 1: Add simulator to workspace members**

In `/repo/Cargo.toml`, add to the `members` array:
```toml
[workspace]
resolver = "2"
members = [
    "crates/gbp-map",
    "crates/gbp-core",
    "crates/gbp-comms",
    "crates/gbp-agent",
    "src/bins/simulator",
]
```

Also add to `[workspace.dependencies]`:
```toml
tokio       = { version = "1",    features = ["full"] }
axum        = { version = "0.7",  features = ["ws"] }
serde_json  = { version = "1" }
tower       = { version = "0.4" }
tracing     = { version = "0.1" }
tracing-subscriber = { version = "0.3", features = ["env-filter"] }
```

- [ ] **Step 2: Write `src/bins/simulator/Cargo.toml`**

```toml
[package]
name    = "simulator"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "simulator"
path = "src/main.rs"

[dependencies]
# No gbp-agent in M1 — GBP is wired in at M2 when trajectory following is added.
gbp-comms  = { path = "../../crates/gbp-comms", features = ["serde"] }
gbp-map    = { path = "../../crates/gbp-map", features = ["parse"] }
tokio      = { workspace = true }
axum       = { workspace = true }
serde_json = { workspace = true }
futures-util = { version = "0.3" }
tracing    = { workspace = true }
tracing-subscriber = { workspace = true }
```

- [ ] **Step 3: Verify workspace parses**

```bash
cargo check -p simulator 2>&1 | head -5
```
Expected: error about missing `src/main.rs` — confirms crate is registered.

- [ ] **Step 4: Commit scaffolding**

```bash
git add Cargo.toml src/bins/simulator/Cargo.toml
git commit -m "feat(m1): add simulator to workspace"
```

---

### Task 2: (Deferred to M2)

`SimComms` is a no-op `CommsInterface` stub needed by `RobotAgent`. Since M1 does not use `RobotAgent`, `sim_comms.rs` is created in M2 alongside the agent wiring. Nothing to do here.

---

### Task 3: PhysicsState — 50 Hz arc-length integration

**Files:**
- Create: `src/bins/simulator/src/physics.rs`

- [ ] **Step 1: Write the failing test**

```rust
// Inline unit test inside physics.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_advances_s_by_v_times_dt() {
        let mut state = PhysicsState { position_s: 0.0, velocity: 2.0, edge_length: 10.0 };
        state.step(0.02); // 50 Hz → dt = 0.02 s
        assert!((state.position_s - 0.04).abs() < 1e-6, "got {}", state.position_s);
    }

    #[test]
    fn physics_clamps_to_edge_length() {
        let mut state = PhysicsState { position_s: 9.99, velocity: 5.0, edge_length: 10.0 };
        state.step(0.02);
        // 9.99 + 5.0 * 0.02 = 10.09 → clamped to 10.0
        assert!((state.position_s - 10.0).abs() < 1e-6, "got {}", state.position_s);
    }

    #[test]
    fn physics_zero_velocity_stays_put() {
        let mut state = PhysicsState { position_s: 3.0, velocity: 0.0, edge_length: 10.0 };
        state.step(0.02);
        assert!((state.position_s - 3.0).abs() < 1e-6);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator 2>&1 | head -10
```
Expected: compile error — `physics` module doesn't exist.

- [ ] **Step 2: Implement `physics.rs`**

```rust
// src/bins/simulator/src/physics.rs
use std::sync::{Arc, Mutex};
use tokio::time::{interval, Duration};
use tracing::debug;

/// Shared mutable physics state for one robot.
#[derive(Clone, Debug)]
pub struct PhysicsState {
    /// Arc-length position along the current edge (m).
    pub position_s: f32,
    /// Current commanded velocity (m/s) — written by GBP runner.
    pub velocity: f32,
    /// Length of the current edge (m) — robot stops at end.
    pub edge_length: f32,
}

impl PhysicsState {
    pub fn new(edge_length: f32) -> Self {
        Self { position_s: 0.0, velocity: 0.0, edge_length }
    }

    /// Integrate one timestep.
    pub fn step(&mut self, dt: f32) {
        self.position_s = (self.position_s + self.velocity * dt).clamp(0.0, self.edge_length);
    }
}

/// Runs at 50 Hz, integrating position from velocity.
/// Velocity is read from shared state; position is written back.
pub async fn physics_task(state: Arc<Mutex<PhysicsState>>) {
    const DT: f32 = 1.0 / 50.0;
    let mut ticker = interval(Duration::from_millis(20)); // 50 Hz
    loop {
        ticker.tick().await;
        let mut s = state.lock().unwrap();
        s.step(DT);
        debug!("physics: s={:.4}", s.position_s);
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```
Expected: `test physics::tests::physics_advances_s_by_v_times_dt ... ok` and all 3 tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/physics.rs
git commit -m "feat(m1): PhysicsState 50 Hz integration with unit tests"
```

---

### Task 4: Broadcast task — 20 Hz, v_nom direct, publishes RobotStateMsg

**Files:**
- Create: `src/bins/simulator/src/broadcast_task.rs`

Per spec: "No GBP running yet — robot moves at `v_nom` directly." The broadcast task sets the physics velocity to `edge.speed.nominal` and publishes the position at 20 Hz. `RobotAgent` is introduced in M2.

> **TODO(M2):** Replace `v = v_nom` with `v = agent.step(obs).velocity` when trajectory following is added.

- [ ] **Step 1: Write the failing test**

```rust
// Inline test in broadcast_task.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_msg_sets_velocity_to_v_nom() {
        let msg = build_robot_state_msg(
            gbp_map::map::EdgeId(0),
            2.5,   // position_s
            1.8,   // v_nom
        );
        assert!((msg.velocity - 1.8).abs() < 1e-6);
        assert!((msg.position_s - 2.5).abs() < 1e-6);
    }

    #[test]
    fn build_msg_sets_3d_pos_from_s() {
        // Linear edge along X: 3D pos = (s, 0, 0) after coord mapping
        let msg = build_robot_state_msg(gbp_map::map::EdgeId(0), 3.0, 2.0);
        // pos_3d is set to [position_s, 0.0, 0.0] in M1 (evaluated in map_scene for real)
        assert!((msg.pos_3d[0] - 3.0).abs() < 1e-6);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -10
```
Expected: compile error — `broadcast_task` module not found.

- [ ] **Step 2: Implement `broadcast_task.rs`**

```rust
// src/bins/simulator/src/broadcast_task.rs
//! Broadcast task for M1: publishes RobotStateMsg at 20 Hz.
//! Velocity is set directly to v_nom (no GBP — introduced in M2).

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use tracing::debug;
use gbp_comms::{RobotStateMsg, RobotSource};
use gbp_map::{EdgeId, MAX_HORIZON};
use heapless::Vec as HVec;
use crate::physics::PhysicsState;

/// Build a RobotStateMsg for a single robot on a single edge in M1.
/// pos_3d is a placeholder (s, 0, 0) — the visualiser re-evaluates 3D position.
pub fn build_robot_state_msg(edge_id: EdgeId, position_s: f32, v_nom: f32) -> RobotStateMsg {
    RobotStateMsg {
        robot_id: 0,
        current_edge: edge_id,
        position_s,
        velocity: v_nom,
        pos_3d: [position_s, 0.0, 0.0], // TODO(M2): evaluate from map
        source: RobotSource::Simulated,
        belief_means: [0.0; MAX_HORIZON],
        belief_vars:  [0.0; MAX_HORIZON],
        planned_edges: HVec::new(),
        active_factors: HVec::new(),
    }
}

/// Runs at 20 Hz. Sets physics velocity to v_nom each cycle and broadcasts position.
pub async fn broadcast_task(
    physics: Arc<Mutex<PhysicsState>>,
    edge_id: EdgeId,
    v_nom: f32,
    tx: broadcast::Sender<RobotStateMsg>,
) {
    let mut ticker = interval(Duration::from_millis(50)); // 20 Hz
    loop {
        ticker.tick().await;
        let pos_s = {
            let mut p = physics.lock().unwrap();
            p.velocity = v_nom; // M1: constant nominal velocity
            p.position_s
        };
        let msg = build_robot_state_msg(edge_id, pos_s, v_nom);
        debug!("broadcast: s={:.4} v={:.4}", pos_s, v_nom);
        let _ = tx.send(msg);
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```
Expected: all tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/broadcast_task.rs
git commit -m "feat(m1): broadcast_task 20 Hz, v_nom direct (no GBP in M1)"
```

---

### Task 5: WebSocket server (axum)

**Files:**
- Create: `src/bins/simulator/src/ws_server.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In ws_server.rs — integration-style smoke test
#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::broadcast;

    #[tokio::test]
    async fn ws_router_builds_without_panic() {
        let (tx, _rx) = broadcast::channel::<String>(16);
        // Should not panic
        let _router = build_router(tx);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -10
```
Expected: compile error.

- [ ] **Step 2: Implement `ws_server.rs`**

```rust
// src/bins/simulator/src/ws_server.rs
use axum::{
    extract::{State, WebSocketUpgrade},
    response::Response,
    routing::get,
    Router,
};
use axum::extract::ws::{Message, WebSocket};
use tokio::sync::broadcast;
use futures_util::{SinkExt, StreamExt};
use tracing::{info, warn};

pub type WsTx = broadcast::Sender<String>;

/// Build the axum router. Separated from bind so it can be tested.
pub fn build_router(tx: WsTx) -> Router {
    Router::new()
        .route("/ws", get(ws_handler))
        .with_state(tx)
}

async fn ws_handler(
    ws: WebSocketUpgrade,
    State(tx): State<WsTx>,
) -> Response {
    ws.on_upgrade(move |socket| handle_socket(socket, tx))
}

async fn handle_socket(mut socket: WebSocket, tx: WsTx) {
    let mut rx = tx.subscribe();
    info!("visualiser connected");
    loop {
        match rx.recv().await {
            Ok(json) => {
                if socket.send(Message::Text(json)).await.is_err() {
                    break;
                }
            }
            Err(broadcast::error::RecvError::Lagged(n)) => {
                warn!("ws lagged {} frames", n);
            }
            Err(broadcast::error::RecvError::Closed) => break,
        }
    }
    info!("visualiser disconnected");
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Add `futures-util` dependency**

Add to `src/bins/simulator/Cargo.toml`:
```toml
futures-util = { version = "0.3" }
```

- [ ] **Step 5: Commit**

```bash
git add src/bins/simulator/src/ws_server.rs src/bins/simulator/Cargo.toml
git commit -m "feat(m1): axum WebSocket server"
```

---

### Task 6: Simulator main entry point

**Files:**
- Create: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Add serde derives to message types**

`RobotStateMsg` needs `serde::Serialize`/`Deserialize` for JSON over WebSocket.

**In `crates/gbp-comms/Cargo.toml`**, add:
```toml
[features]
default = []
# Enables serde derives on all message types. Requires gbp-map/serde for EdgeId.
serde = ["dep:serde", "heapless/serde", "gbp-map/serde"]

[dependencies]
serde = { workspace = true, optional = true }
```

**In `crates/gbp-map/Cargo.toml`**, add:
```toml
[features]
default  = []
serde    = ["dep:serde", "heapless/serde"]
parse    = ["serde", "dep:serde_yaml"]

[dependencies]
serde    = { workspace = true, optional = true }
serde_yaml = { workspace = true, optional = true }
```

**In `crates/gbp-map/src/map.rs`**, add serde derives to `EdgeId`, `NodeId`, `NodeType`, `EdgeGeometry`, `NurbsGeometry`, `SpeedProfile`, `SafetyProfile`, `Edge`, `Node`:
```rust
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EdgeId(pub u16);

// ... repeat for all public map types
```

**In `crates/gbp-comms/src/lib.rs`**, add to `RobotStateMsg`, `RobotSource`, `GBPTimestep`, `RobotBroadcast`, `ObservationUpdate`, `TrajectoryCommand`:
```rust
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RobotStateMsg { ... }
```

> **Note:** `EdgeId` lives in `gbp-map`, not `gbp-comms`. The `serde` feature chain `gbp-comms/serde → gbp-map/serde` ensures `EdgeId`'s derive is enabled transitively. Do NOT add `EdgeId`'s derive to `gbp-comms/src/lib.rs`.

- [ ] **Step 2: Implement `main.rs`**

No TDD for the binary entry point — its correctness is verified by the end-to-end test (robot moves in visualiser).

```rust
// src/bins/simulator/src/main.rs
mod physics;
mod broadcast_task;
mod ws_server;

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tracing::info;
use tracing_subscriber::EnvFilter;
use gbp_comms::RobotStateMsg;
use physics::PhysicsState;

struct Args { map_path: String, bind_addr: String }

fn parse_args() -> Args {
    let mut args = std::env::args().skip(1);
    Args {
        map_path: args.next().unwrap_or_else(|| "maps/test_loop_map.yaml".to_string()),
        bind_addr: args.next().unwrap_or_else(|| "0.0.0.0:3000".to_string()),
    }
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env()
            .add_directive("simulator=debug".parse().unwrap()))
        .init();

    let args = parse_args();
    info!("loading map from {}", args.map_path);

    let yaml = std::fs::read_to_string(&args.map_path)
        .unwrap_or_else(|e| panic!("cannot read map: {}", e));
    let map = gbp_map::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("map parse error: {}", e));

    // M1: use the first edge; robot moves along it at nominal speed
    let first_edge = map.edges.first().expect("map has no edges");
    let edge_length = first_edge.geometry.length();
    let edge_id    = first_edge.id;
    let v_nom      = first_edge.speed.nominal;
    info!("edge {:?} length={:.2}m v_nom={:.2}m/s", edge_id, edge_length, v_nom);

    // Shared physics state
    let physics = Arc::new(Mutex::new(PhysicsState::new(edge_length)));

    // Domain channel: broadcast_task → relay → ws_server
    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    // JSON channel for WebSocket (ws_server is domain-agnostic)
    let (tx_json, _): (broadcast::Sender<String>, _) = broadcast::channel(16);

    // Relay: RobotStateMsg → JSON string.
    // The initial receiver is dropped intentionally; only subscribe()-based receivers count.
    let tx_json_relay = tx_json.clone();
    let mut rx_state = tx_state.subscribe();
    tokio::spawn(async move {
        loop {
            if let Ok(msg) = rx_state.recv().await {
                if let Ok(json) = serde_json::to_string(&msg) {
                    let _ = tx_json_relay.send(json);
                }
            }
        }
    });

    tokio::spawn(physics::physics_task(Arc::clone(&physics)));
    tokio::spawn(broadcast_task::broadcast_task(
        Arc::clone(&physics), edge_id, v_nom, tx_state,
    ));

    let router   = ws_server::build_router(tx_json);
    let listener = tokio::net::TcpListener::bind(&args.bind_addr).await
        .unwrap_or_else(|e| panic!("cannot bind {}: {}", args.bind_addr, e));
    info!("WebSocket server listening on ws://{}/ws", args.bind_addr);
    axum::serve(listener, router).await.unwrap();
}
```

- [ ] **Step 3: Build check**

```bash
cargo build -p simulator 2>&1 | tail -5
```
Expected: `Finished`.

- [ ] **Step 4: Smoke test**

```bash
cargo run -p simulator -- maps/test_loop_map.yaml &
sleep 2
curl -s --include --no-buffer \
  -H "Connection: Upgrade" -H "Upgrade: websocket" \
  -H "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==" \
  -H "Sec-WebSocket-Version: 13" \
  http://localhost:3000/ws | head -5
kill %1
```
Expected: HTTP 101 Switching Protocols header.

- [ ] **Step 5: Commit**

```bash
git add src/bins/simulator/src/main.rs crates/gbp-comms/src/lib.rs crates/gbp-comms/Cargo.toml \
        crates/gbp-map/src/map.rs crates/gbp-map/Cargo.toml
git commit -m "feat(m1): simulator main — physics + v_nom broadcast + WebSocket"
```

---

## Chunk 2: Visualiser Crate

### Task 7: Register visualiser in workspace and write Cargo.toml

**Files:**
- Modify: `Cargo.toml` (root workspace)
- Create: `src/bins/visualiser/Cargo.toml`

- [ ] **Step 1: Add visualiser to workspace members**

In `/repo/Cargo.toml`, add `"src/bins/visualiser"` to members.

Also add to `[workspace.dependencies]`:
```toml
bevy             = { version = "0.15", default-features = false, features = [
    "bevy_asset", "bevy_core_pipeline", "bevy_mesh",
    "bevy_pbr", "bevy_render", "bevy_winit", "bevy_gizmos",
    "tonemapping_luts", "default_font", "x11"
] }
bevy_egui        = { version = "0.30" }
tokio-tungstenite = { version = "0.24", features = ["native-tls"] }
futures-util      = { version = "0.3" }
```

- [ ] **Step 2: Write `src/bins/visualiser/Cargo.toml`**

```toml
[package]
name    = "visualiser"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "visualiser"
path = "src/main.rs"

[dependencies]
gbp-comms  = { path = "../../crates/gbp-comms", features = ["serde"] }
gbp-map    = { path = "../../crates/gbp-map", features = ["parse"] }
bevy       = { workspace = true }
bevy_egui  = { workspace = true }
tokio      = { workspace = true }
tokio-tungstenite = { workspace = true }
futures-util      = { workspace = true }
serde_json = { workspace = true }
tracing    = { workspace = true }
tracing-subscriber = { workspace = true }
```

- [ ] **Step 3: Verify**

```bash
cargo check -p visualiser 2>&1 | head -5
```
Expected: error about missing `src/main.rs`.

- [ ] **Step 4: Commit**

```bash
git add Cargo.toml src/bins/visualiser/Cargo.toml
git commit -m "feat(m1): add visualiser to workspace"
```

---

### Task 8: WebSocket client — background thread + Bevy channel

**Files:**
- Create: `src/bins/visualiser/src/ws_client.rs`
- Create: `src/bins/visualiser/src/state.rs`

- [ ] **Step 1: Write `state.rs`**

```rust
// src/bins/visualiser/src/state.rs
use bevy::prelude::*;
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;
use gbp_comms::RobotStateMsg;

/// Bevy resource: queue of incoming messages from WebSocket thread.
#[derive(Resource, Default)]
pub struct WsInbox(pub Arc<Mutex<VecDeque<RobotStateMsg>>>);

/// Bevy resource: latest known state per robot, keyed by robot_id.
#[derive(Resource, Default)]
pub struct RobotStates(pub std::collections::HashMap<u32, RobotStateMsg>);

/// Bevy resource: parsed map.
#[derive(Resource)]
pub struct MapRes(pub gbp_map::map::Map);
```

- [ ] **Step 1b: Write a failing test for JSON deserialization**

```rust
// In ws_client.rs (inline test)
#[cfg(test)]
mod tests {
    use super::*;
    use gbp_comms::{RobotStateMsg, RobotSource};
    use gbp_map::MAX_HORIZON;

    fn minimal_msg() -> RobotStateMsg {
        RobotStateMsg {
            robot_id: 1,
            current_edge: gbp_map::map::EdgeId(0),
            position_s: 2.5,
            velocity: 1.8,
            pos_3d: [2.5, 0.0, 0.0],
            source: RobotSource::Simulated,
            belief_means: [0.0; MAX_HORIZON],
            belief_vars:  [0.0; MAX_HORIZON],
            planned_edges: heapless::Vec::new(),
            active_factors: heapless::Vec::new(),
        }
    }

    #[test]
    fn deserialize_round_trip() {
        let msg = minimal_msg();
        let json = serde_json::to_string(&msg).expect("serialize");
        let decoded: RobotStateMsg = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(decoded.robot_id, 1);
        assert!((decoded.position_s - 2.5).abs() < 1e-6);
        assert!((decoded.velocity  - 1.8).abs() < 1e-6);
    }
}
```

- [ ] **Step 1c: Run to confirm failure**

```bash
cargo test -p visualiser --lib 2>&1 | head -10
```
Expected: compile error — `ws_client` module not defined yet.

- [ ] **Step 2: Write `ws_client.rs`**

```rust
// src/bins/visualiser/src/ws_client.rs
//! Spawns a background tokio thread that connects to the simulator WebSocket
//! and writes received RobotStateMsg values into WsInbox.

use std::sync::{Arc, Mutex};
use std::collections::VecDeque;
use futures_util::StreamExt;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;
use gbp_comms::RobotStateMsg;
use tracing::{info, warn, error};

pub fn spawn_ws_client(
    url: String,
    inbox: Arc<Mutex<VecDeque<RobotStateMsg>>>,
) {
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            loop {
                info!("connecting to {}", url);
                match connect_async(&url).await {
                    Ok((mut ws, _)) => {
                        info!("connected to simulator");
                        while let Some(msg) = ws.next().await {
                            match msg {
                                Ok(Message::Text(json)) => {
                                    match serde_json::from_str::<RobotStateMsg>(&json) {
                                        Ok(state) => {
                                            inbox.lock().unwrap().push_back(state);
                                        }
                                        Err(e) => warn!("bad msg: {}", e),
                                    }
                                }
                                Ok(Message::Close(_)) => break,
                                Err(e) => { error!("ws error: {}", e); break; }
                                _ => {}
                            }
                        }
                        warn!("disconnected, retrying in 1s");
                    }
                    Err(e) => {
                        error!("connect failed: {}, retrying in 1s", e);
                    }
                }
                tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
            }
        });
    });
}
```

- [ ] **Step 3: Commit**

```bash
git add src/bins/visualiser/src/ws_client.rs src/bins/visualiser/src/state.rs
git commit -m "feat(m1): visualiser WebSocket client and Bevy state resources"
```

---

### Task 9: Map scene — edge line meshes and node spheres

**Files:**
- Create: `src/bins/visualiser/src/map_scene.rs`

- [ ] **Step 1: Write the failing test**

```rust
// Inline test in map_scene.rs
#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

    #[test]
    fn edge_midpoints_computed_correctly() {
        // Verify the geometry helper used to place edge labels
        let start = [0.0f32, 0.0, 0.0];
        let end   = [2.0f32, 0.0, 0.0];
        let mid = midpoint(start, end);
        assert!((mid[0] - 1.0).abs() < 1e-5);
        assert!((mid[1] - 0.0).abs() < 1e-5);
    }

    #[test]
    fn node_type_color_waypoint_is_grey() {
        let c = node_color(NodeType::Waypoint);
        // Waypoint: grey (all channels equal, not black)
        assert!(c.r() > 0.3 && c.r() < 0.8);
        assert!((c.r() - c.g()).abs() < 0.01);
        assert!((c.r() - c.b()).abs() < 0.01);
    }

    #[test]
    fn node_type_color_merge_is_distinct_from_divert() {
        let merge  = node_color(NodeType::Merge);
        let divert = node_color(NodeType::Divert);
        // Hues should differ
        let diff = (merge.r() - divert.r()).abs()
                 + (merge.g() - divert.g()).abs()
                 + (merge.b() - divert.b()).abs();
        assert!(diff > 0.1, "merge and divert colours too similar");
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib 2>&1 | head -10
```
Expected: compile error.

- [ ] **Step 2: Implement `map_scene.rs`**

```rust
// src/bins/visualiser/src/map_scene.rs
use bevy::prelude::*;
use bevy::gizmos::gizmos::Gizmos;
use gbp_map::map::{EdgeGeometry, Map, NodeType};
use crate::state::MapRes;

pub struct MapScenePlugin;

impl Plugin for MapScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_map_scene)
           .add_systems(Update, draw_edge_gizmos);
    }
}

/// Colour by node type.
pub fn node_color(nt: NodeType) -> Color {
    match nt {
        NodeType::Waypoint  => Color::srgb(0.55, 0.55, 0.55),
        NodeType::Merge     => Color::srgb(0.2,  0.8,  0.2),
        NodeType::Divert    => Color::srgb(0.2,  0.5,  0.9),
        NodeType::Charger   => Color::srgb(0.9,  0.8,  0.1),
        NodeType::Toploader => Color::srgb(0.9,  0.4,  0.1),
        NodeType::Discharge => Color::srgb(0.7,  0.2,  0.7),
    }
}

pub fn midpoint(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [(a[0]+b[0])/2.0, (a[1]+b[1])/2.0, (a[2]+b[2])/2.0]
}

/// Spawn a sphere for each node.
fn spawn_map_scene(
    mut commands: Commands,
    map: Res<MapRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ambient light
    commands.insert_resource(AmbientLight { color: Color::WHITE, brightness: 400.0 });

    // Camera — positioned to see the full map
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5.0, 12.0, 12.0).looking_at(Vec3::new(5.0, 0.0, 5.0), Vec3::Y),
    ));

    let sphere_mesh = meshes.add(Sphere::new(0.08).mesh().ico(1).unwrap());

    for node in map.0.nodes.iter() {
        let color = node_color(node.node_type);
        let mat = materials.add(StandardMaterial::from_color(color));
        let [x, y, z] = node.position;
        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(mat),
            Transform::from_xyz(x, z, -y), // Bevy: Y-up, map uses Z for height
        ));
    }
}

/// Draw edge gizmos each frame (thin lines — no mesh allocation).
fn draw_edge_gizmos(
    map: Res<MapRes>,
    mut gizmos: Gizmos,
) {
    const NURBS_SAMPLES: usize = 32;
    for edge in map.0.edges.iter() {
        match &edge.geometry {
            EdgeGeometry::Line { start, end, .. } => {
                let s = Vec3::new(start[0], start[2], -start[1]);
                let e = Vec3::new(end[0],   end[2],   -end[1]);
                gizmos.line(s, e, Color::srgb(0.8, 0.8, 0.8));
            }
            EdgeGeometry::Nurbs(n) => {
                let mut prev = {
                    let p = gbp_map::nurbs::eval_point(0.0, &n.control_points, &n.knots, n.degree as usize);
                    Vec3::new(p[0], p[2], -p[1])
                };
                for i in 1..=NURBS_SAMPLES {
                    let t = i as f32 / NURBS_SAMPLES as f32;
                    let p = gbp_map::nurbs::eval_point(t, &n.control_points, &n.knots, n.degree as usize);
                    let cur = Vec3::new(p[0], p[2], -p[1]);
                    gizmos.line(prev, cur, Color::srgb(0.8, 0.8, 0.8));
                    prev = cur;
                }
            }
        }
    }
}
```

**Coordinate convention note:** The map uses `[x, y, z]` where `z` is height. Bevy uses Y-up. Mapping: `Bevy(x, y, z) = Map(x, z, -y)`.

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib 2>&1 | grep -E "test |FAILED|passed"
```
Expected: 3 tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/map_scene.rs
git commit -m "feat(m1): map scene — node spheres and edge gizmos"
```

---

### Task 10: Robot rendering — arrow mesh at position_s along edge tangent

**Files:**
- Create: `src/bins/visualiser/src/robot_render.rs`

- [ ] **Step 1: Write the failing test**

```rust
// Inline test in robot_render.rs
#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

    #[test]
    fn robot_position_on_line_edge_at_half() {
        let map = {
            let mut m = Map::new("t");
            m.add_node(Node { id:NodeId(0), position:[0.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_node(Node { id:NodeId(1), position:[4.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_edge(Edge {
                id:EdgeId(0), start:NodeId(0), end:NodeId(1),
                geometry: EdgeGeometry::Line { start:[0.0,0.0,0.0], end:[4.0,0.0,0.0], length:4.0 },
                speed: SpeedProfile { max:2.5, nominal:2.0, accel_limit:1.0, decel_limit:1.0 },
                safety: SafetyProfile { clearance:0.3 },
            }).unwrap();
            m
        };
        let pos = robot_world_pos(&map, EdgeId(0), 2.0);
        assert!((pos[0] - 2.0).abs() < 1e-4, "x={}", pos[0]);
        assert!((pos[1] - 0.0).abs() < 1e-4, "y={}", pos[1]);
    }

    #[test]
    fn robot_tangent_on_line_edge_is_unit_x() {
        let map = {
            let mut m = Map::new("t");
            m.add_node(Node { id:NodeId(0), position:[0.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_node(Node { id:NodeId(1), position:[4.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_edge(Edge {
                id:EdgeId(0), start:NodeId(0), end:NodeId(1),
                geometry: EdgeGeometry::Line { start:[0.0,0.0,0.0], end:[4.0,0.0,0.0], length:4.0 },
                speed: SpeedProfile { max:2.5, nominal:2.0, accel_limit:1.0, decel_limit:1.0 },
                safety: SafetyProfile { clearance:0.3 },
            }).unwrap();
            m
        };
        let tan = robot_tangent(&map, EdgeId(0), 2.0);
        assert!((tan[0] - 1.0).abs() < 1e-4, "tan.x={}", tan[0]);
        assert!((tan[1]).abs() < 1e-4);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib 2>&1 | head -10
```
Expected: compile error.

- [ ] **Step 2: Implement `robot_render.rs`**

```rust
// src/bins/visualiser/src/robot_render.rs
use bevy::prelude::*;
use gbp_map::map::Map;
use gbp_map::EdgeId;
use crate::state::{MapRes, RobotStates, WsInbox};

pub struct RobotRenderPlugin;

impl Plugin for RobotRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_robot_arrow)
           .add_systems(Update, (drain_ws_inbox, update_robot_transforms).chain());
    }
}

/// Marker component for the robot arrow entity.
#[derive(Component)]
pub struct RobotArrow { pub robot_id: u32 }

/// Compute 3D world position for a robot at arc-length s on an edge.
/// Applies the map→Bevy coordinate transform (Bevy Y-up; map Z = height).
pub fn robot_world_pos(map: &Map, edge_id: EdgeId, s: f32) -> [f32; 3] {
    let p = map.eval_position(edge_id, s);
    [p[0], p[2], -p[1]]
}

/// Compute world-space unit tangent for the robot's orientation.
pub fn robot_tangent(map: &Map, edge_id: EdgeId, s: f32) -> [f32; 3] {
    let t = map.eval_tangent(edge_id, s);
    [t[0], t[2], -t[1]]
}

fn spawn_robot_arrow(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Cone pointing along +Z in mesh space; we rotate to align with tangent.
    let cone = meshes.add(Cone { radius: 0.08, height: 0.25 }.mesh().build());
    let mat  = materials.add(StandardMaterial::from_color(Color::srgb(0.2, 0.6, 1.0)));
    commands.spawn((
        Mesh3d(cone),
        MeshMaterial3d(mat),
        Transform::IDENTITY,
        RobotArrow { robot_id: 0 },
    ));
}

/// Drain WsInbox and update RobotStates each frame.
fn drain_ws_inbox(
    inbox: Res<WsInbox>,
    mut states: ResMut<RobotStates>,
) {
    let mut q = inbox.0.lock().unwrap();
    while let Some(msg) = q.pop_front() {
        states.0.insert(msg.robot_id, msg);
    }
}

/// Move each robot arrow to its current position.
fn update_robot_transforms(
    map: Res<MapRes>,
    states: Res<RobotStates>,
    mut query: Query<(&RobotArrow, &mut Transform)>,
) {
    for (arrow, mut transform) in query.iter_mut() {
        if let Some(state) = states.0.get(&arrow.robot_id) {
            let pos = robot_world_pos(&map.0, state.current_edge, state.position_s);
            let tan = robot_tangent(&map.0, state.current_edge, state.position_s);
            transform.translation = Vec3::from(pos);
            // Align arrow to tangent (rotate default +Y up to tangent direction)
            let dir = Vec3::from(tan);
            if dir.length_squared() > 0.01 {
                transform.rotation = Quat::from_rotation_arc(Vec3::Y, dir);
            }
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib 2>&1 | grep -E "test |FAILED|passed"
```
Expected: tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/robot_render.rs
git commit -m "feat(m1): robot arrow mesh positioned at position_s along edge tangent"
```

---

### Task 11: Visualiser main and end-to-end smoke test

**Files:**
- Create: `src/bins/visualiser/src/main.rs`

- [ ] **Step 1: Implement `main.rs`**

```rust
// src/bins/visualiser/src/main.rs
mod state;
mod ws_client;
mod map_scene;
mod robot_render;

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use state::{MapRes, RobotStates, WsInbox};
use map_scene::MapScenePlugin;
use robot_render::RobotRenderPlugin;
use tracing_subscriber::EnvFilter;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env()
            .add_directive("visualiser=info".parse().unwrap()))
        .init();

    // Load map (default path or from env)
    let map_path = std::env::var("MAP_PATH")
        .unwrap_or_else(|_| "maps/test_loop_map.yaml".to_string());
    let yaml = std::fs::read_to_string(&map_path)
        .unwrap_or_else(|e| panic!("cannot read map {}: {}", map_path, e));
    let map = gbp_map::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("map parse error: {}", e));

    // Connect WebSocket client
    let ws_url = std::env::var("WS_URL")
        .unwrap_or_else(|_| "ws://localhost:3000/ws".to_string());
    let inbox: Arc<Mutex<VecDeque<_>>> = Arc::new(Mutex::new(VecDeque::new()));
    ws_client::spawn_ws_client(ws_url, Arc::clone(&inbox));

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "GBP Trajectory Planner".into(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin)
        .insert_resource(MapRes(map))
        .insert_resource(RobotStates::default())
        .insert_resource(WsInbox(inbox))
        .add_plugins(MapScenePlugin)
        .add_plugins(RobotRenderPlugin)
        .run();
}
```

- [ ] **Step 2: Build both binaries**

```bash
cargo build -p simulator -p visualiser 2>&1 | tail -5
```
Expected: `Finished`.

- [ ] **Step 3: Manual end-to-end verification**

```bash
# Terminal 1
RUST_LOG=simulator=debug cargo run -p simulator -- maps/test_loop_map.yaml

# Terminal 2 (new terminal)
MAP_PATH=maps/test_loop_map.yaml cargo run -p visualiser
```

Expected:
- Bevy window opens showing map edges (grey lines) and node spheres (coloured by type)
- Blue arrow cone visible on the first edge
- Arrow advances along the edge at ~2 m/s nominal velocity
- `simulator` terminal shows `gbp: s=X.XXXX v=X.XXXX` lines at 20 Hz

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/main.rs Cargo.toml
git commit -m "feat(m1): visualiser Bevy app — map scene + robot arrow, M1 complete"
```

---

## Summary

| Chunk | Deliverables | Verification |
|---|---|---|
| 1 | Simulator: physics (50 Hz), broadcast_task v_nom (20 Hz), axum WebSocket, serde on comms types | Unit tests pass, HTTP 101 smoke test |
| 2 | Visualiser: Bevy app, map scene, robot arrow, WebSocket client (with serde round-trip test) | Robot arrow advances in window |

**What M2 adds:** `SimComms`, `RobotAgent`, A* path planning, multi-edge trajectories, edge transitions, trapezoidal deceleration, NURBS edge rendering, dashed planned-path gizmo.
