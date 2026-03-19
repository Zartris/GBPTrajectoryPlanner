# M9: Hybrid Fleet — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run a mixed fleet of simulated and hardware robots simultaneously. The bridge routes by `robot_id`: robots listed in `--sim-ids` are proxied through the simulator WebSocket; robots in `--esp-ids` are relayed from ESP32 hardware via UDP. The visualiser sees a unified stream of `RobotStateMsg` regardless of robot origin.

**Architecture:** The bridge is extended with a `--sim-ws` argument pointing at a running simulator. For each tick, the bridge subscribes to the simulator WebSocket for sim-robot states and relays them alongside hardware robot states to all connected visualiser WebSocket clients. Trajectory commands from the visualiser are routed: if `robot_id ∈ sim-ids`, forward to simulator control WebSocket; if `robot_id ∈ esp-ids`, encode as postcard and send UDP. A time-sync monitor logs wall-clock latency per robot source.

**Tech Stack:** `tokio`, `tokio-tungstenite` (WS client for sim connection), `axum`, `serde_json`, `postcard`, `gbp-comms`. No new crates beyond M8.

**Prerequisite:** M8 plan complete and passing.

---

## File Structure

```
src/bins/bridge/
  src/
    main.rs          — updated: --sim-ws, --sim-ids, --esp-ids CLI args; routing logic
    sim_client.rs    — new tokio task: WS client to simulator, forwards states upstream
    router.rs        — pure: route_command(robot_id, sim_ids, esp_ids) -> CommandDest
    time_sync.rs     — latency tracker: records last-seen timestamp per robot_id
    udp_receiver.rs  — unchanged
    ws_server.rs     — unchanged
    relay.rs         — unchanged
```

---

## Chunk 1: Command routing + sim client

### Task 1: `router.rs` — pure command routing logic

**Files:**
- Create: `src/bins/bridge/src/router.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// src/bins/bridge/src/router.rs — unit tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_id_routes_to_simulator() {
        let sim_ids = &[0u32, 1, 2];
        let esp_ids = &[3u32, 4];
        assert_eq!(route_command(1, sim_ids, esp_ids), CommandDest::Simulator);
    }

    #[test]
    fn esp_id_routes_to_hardware() {
        let sim_ids = &[0u32, 1];
        let esp_ids = &[3u32, 4];
        assert_eq!(route_command(4, sim_ids, esp_ids), CommandDest::Hardware);
    }

    #[test]
    fn unknown_id_routes_to_drop() {
        let sim_ids = &[0u32];
        let esp_ids = &[1u32];
        assert_eq!(route_command(99, sim_ids, esp_ids), CommandDest::Drop);
    }

    #[test]
    fn empty_id_lists_all_drop() {
        assert_eq!(route_command(0, &[], &[]), CommandDest::Drop);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `router` module doesn't exist.

- [ ] **Step 2: Implement `router.rs`**

```rust
// src/bins/bridge/src/router.rs

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommandDest {
    Simulator,
    Hardware,
    Drop,
}

/// Determine where to route a trajectory command based on robot_id.
pub fn route_command(robot_id: u32, sim_ids: &[u32], esp_ids: &[u32]) -> CommandDest {
    if sim_ids.contains(&robot_id) { CommandDest::Simulator }
    else if esp_ids.contains(&robot_id) { CommandDest::Hardware }
    else { CommandDest::Drop }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```
Expected: all 4 router tests PASS.

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/router.rs
git commit -m "feat(m9): router — command routing by robot_id sim/esp/drop"
```

---

### Task 2: `time_sync.rs` — per-robot latency tracker

**Files:**
- Create: `src/bins/bridge/src/time_sync.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// src/bins/bridge/src/time_sync.rs — unit tests
#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn latency_tracker_records_first_seen() {
        let mut tracker = LatencyTracker::new();
        let now = Instant::now();
        tracker.record(7, now);
        assert!(tracker.last_seen(7).is_some());
    }

    #[test]
    fn latency_tracker_age_increases_over_time() {
        let mut tracker = LatencyTracker::new();
        let past = Instant::now() - Duration::from_millis(200);
        tracker.record(3, past);
        let age = tracker.age_ms(3).expect("should have age");
        assert!(age >= 200, "expected age >= 200 ms, got {}", age);
    }

    #[test]
    fn latency_tracker_unknown_robot_returns_none() {
        let tracker = LatencyTracker::new();
        assert!(tracker.last_seen(99).is_none());
        assert!(tracker.age_ms(99).is_none());
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `time_sync` module doesn't exist.

- [ ] **Step 2: Implement `time_sync.rs`**

```rust
// src/bins/bridge/src/time_sync.rs

use std::collections::HashMap;
use std::time::Instant;

pub struct LatencyTracker {
    last_seen: HashMap<u32, Instant>,
}

impl LatencyTracker {
    pub fn new() -> Self { Self { last_seen: HashMap::new() } }

    pub fn record(&mut self, robot_id: u32, at: Instant) {
        self.last_seen.insert(robot_id, at);
    }

    pub fn last_seen(&self, robot_id: u32) -> Option<Instant> {
        self.last_seen.get(&robot_id).copied()
    }

    /// Returns milliseconds since the robot was last seen, or None if never seen.
    pub fn age_ms(&self, robot_id: u32) -> Option<u64> {
        self.last_seen(robot_id)
            .map(|t| t.elapsed().as_millis() as u64)
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```
Expected: all 3 latency tracker tests PASS.

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/time_sync.rs
git commit -m "feat(m9): time_sync — per-robot latency tracker"
```

---

### Task 3: `sim_client.rs` — WebSocket client to simulator

**Files:**
- Create: `src/bins/bridge/src/sim_client.rs`

- [ ] **Step 1: Write the failing test**

```rust
// src/bins/bridge/src/sim_client.rs — unit tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_sim_state_msg_from_json() {
        // The sim WS sends JSON-encoded RobotStateMsg.
        // The sim_client must parse it back into a RobotStateMsg.
        let json = r#"{
            "robot_id": 1,
            "position_s": 2.5,
            "velocity": 0.6,
            "current_edge": {"0": 1},
            "pos_3d": [1.0, 0.5, 0.0],
            "belief_means": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "belief_vars":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "active_factor_count": 0,
            "sigma_dyn": 1.0,
            "sigma_r": 0.5
        }"#;
        let msg = parse_sim_state_json(json).expect("parse failed");
        assert_eq!(msg.robot_id, 1);
        assert!((msg.position_s - 2.5).abs() < 1e-5);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `sim_client` module doesn't exist.

- [ ] **Step 2: Implement `sim_client.rs`**

```rust
// src/bins/bridge/src/sim_client.rs

use gbp_comms::RobotStateMsg;
use tokio::sync::broadcast;
use tokio_tungstenite::connect_async;
use futures_util::StreamExt;

/// Parse a JSON string from the simulator WebSocket into a RobotStateMsg.
pub fn parse_sim_state_json(json: &str) -> Result<RobotStateMsg, serde_json::Error> {
    serde_json::from_str(json)
}

/// Tokio task: connect to simulator WebSocket, receive RobotStateMsg JSON, broadcast upstream.
/// `sim_ws_url` example: "ws://localhost:3000/ws"
/// Only messages with `robot_id ∈ sim_ids` are forwarded.
pub async fn sim_client_task(
    sim_ws_url: String,
    sim_ids:    Vec<u32>,
    tx:         broadcast::Sender<RobotStateMsg>,
) {
    let url = url::Url::parse(&sim_ws_url).expect("invalid sim WS URL");
    loop {
        match connect_async(&url).await {
            Ok((mut ws, _)) => {
                tracing::info!("sim_client: connected to {}", sim_ws_url);
                while let Some(msg) = ws.next().await {
                    match msg {
                        Ok(tokio_tungstenite::tungstenite::Message::Text(text)) => {
                            if let Ok(state) = parse_sim_state_json(&text) {
                                if sim_ids.contains(&state.robot_id) {
                                    tx.send(state).ok();
                                }
                            }
                        }
                        Err(e) => {
                            tracing::warn!("sim_client: WS error: {}", e);
                            break;
                        }
                        _ => {}
                    }
                }
                // Server closed the connection cleanly — wait before reconnecting
                // to avoid a tight spin loop when the simulator is restarting.
                tracing::info!("sim_client: disconnected from {}, retry in 2s", sim_ws_url);
                tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            }
            Err(e) => {
                tracing::warn!("sim_client: connect failed: {} — retry in 2s", e);
                tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            }
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```
Expected: `parse_sim_state_msg_from_json` PASS.

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/sim_client.rs
git commit -m "feat(m9): sim_client_task — WS client to simulator, filters by sim_ids"
```

---

## Chunk 2: CLI wiring + end-to-end verification

### Task 4: `main.rs` hybrid CLI wiring

**Files:**
- Modify: `src/bins/bridge/src/main.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// src/bins/bridge/src/main.rs — unit tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_id_list_splits_by_comma() {
        let ids = parse_id_list("0,1,2");
        assert_eq!(ids, vec![0u32, 1, 2]);
    }

    #[test]
    fn parse_id_list_empty_string_returns_empty() {
        let ids = parse_id_list("");
        assert!(ids.is_empty());
    }

    #[test]
    fn bridge_args_default_has_empty_id_lists() {
        let args = BridgeArgs::default();
        // Without --sim-ids / --esp-ids, both lists are empty (relay all or none)
        assert!(args.sim_ids.is_none());
        assert!(args.esp_ids.is_none());
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `parse_id_list` doesn't exist, `BridgeArgs` missing `sim_ids`/`esp_ids`.

- [ ] **Step 2: Implement hybrid wiring in `main.rs`**

```rust
// Additional CLI args added to BridgeArgs:
#[derive(Parser, Default)]
pub struct BridgeArgs {
    #[arg(long, default_value = "0.0.0.0:4242")]
    pub esp_udp: String,

    #[arg(long, default_value = "0.0.0.0:3001")]
    pub ws_bind: String,

    /// Simulator WebSocket URL (hybrid mode; optional)
    #[arg(long)]
    pub sim_ws: Option<String>,

    /// Comma-separated robot IDs served by simulator (e.g. "0,1,2")
    #[arg(long)]
    pub sim_ids: Option<String>,

    /// Comma-separated robot IDs served by ESP32 hardware (e.g. "3,4")
    #[arg(long)]
    pub esp_ids: Option<String>,

    #[arg(long)]
    pub map: Option<std::path::PathBuf>,
}

/// Parse "0,1,2" → vec![0, 1, 2]. Returns empty vec for empty string.
pub fn parse_id_list(s: &str) -> Vec<u32> {
    if s.is_empty() { return vec![]; }
    s.split(',').filter_map(|x| x.trim().parse().ok()).collect()
}

// In main():
if let (Some(sim_ws_url), Some(sim_ids_str)) = (args.sim_ws, args.sim_ids.as_deref()) {
    let sim_ids = parse_id_list(sim_ids_str);
    let tx_clone = state_tx.clone();
    tokio::spawn(crate::sim_client::sim_client_task(sim_ws_url, sim_ids, tx_clone));
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```
Expected: all 3 main.rs tests PASS (`parse_id_list_splits_by_comma`, `parse_id_list_empty_string_returns_empty`, `bridge_args_default_has_empty_id_lists`).

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/main.rs
git commit -m "feat(m9): hybrid bridge — --sim-ws, --sim-ids, --esp-ids CLI wiring"
```

---

### Task 5: End-to-end hybrid fleet verification

- [ ] **Step 1: Run all bridge tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed|error"
```
Expected: all PASS.

- [ ] **Step 2: Hybrid smoke test (simulator + bridge + visualiser)**

```bash
# Terminal 1: start simulator with robots 0, 1, 2
cargo run -p simulator -- --map maps/test_loop_map.yaml --num-robots 3 &

# Terminal 2: start bridge in hybrid mode
cargo run -p bridge -- \
    --map maps/test_loop_map.yaml \
    --sim-ws ws://localhost:3000/ws \
    --sim-ids 0,1,2 \
    --esp-ids 3,4 \
    --ws-bind 0.0.0.0:3001 \
    --esp-udp 0.0.0.0:4242 &

# Terminal 3: start visualiser pointed at bridge
cargo run -p visualiser -- --ws ws://localhost:3001/ws
```

Verify:
- Robots 0, 1, 2 appear in visualiser (sourced from simulator)
- Bridge logs show "sim_client: connected to ws://localhost:3000/ws"
- When an ESP32 running M7 firmware is on the network and assigned ID 3, it appears in the visualiser too
- Block-edge command from visualiser on robot 0 is forwarded to simulator control channel
- Block-edge command on robot 3 is forwarded as UDP to the hardware robot

- [ ] **Step 3: Time-sync monitoring**

Add a periodic log line from the bridge showing per-robot age:
```rust
// In main.rs — spawn a monitor task:
tokio::spawn(async move {
    let mut interval = tokio::time::interval(std::time::Duration::from_secs(5));
    loop {
        interval.tick().await;
        for id in sim_ids.iter().chain(esp_ids.iter()) {
            if let Some(age) = tracker.lock().unwrap().age_ms(*id) {
                if age > 1000 {
                    tracing::warn!("robot {} stale: {}ms since last state", id, age);
                }
            }
        }
    }
});
```

Verify: stale-robot warnings appear in bridge logs when a robot disconnects.

- [ ] **Step 4: Commit and tag M9**

```bash
git add -A
git commit -m "feat(m9): hybrid fleet — sim + hardware robots unified in visualiser"
git tag m9-complete
```

---
