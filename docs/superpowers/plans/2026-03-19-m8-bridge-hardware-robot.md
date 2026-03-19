# M8: Bridge — Hardware Robot Integration — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `bridge` binary that relays `RobotStateMsg` from ESP32 robots (UDP/UART) to the visualiser (WebSocket), and forwards trajectory commands from the visualiser back to the ESP32 robots.

**Architecture:** The bridge has two I/O faces: (1) a UDP socket bound on a configurable port listening for postcard-encoded `RobotStateMsg` frames from ESP32 robots; (2) a WebSocket server (axum) that speaks the same JSON protocol as the simulator. The bridge is stateless — it deserialises each UDP datagram, re-encodes as JSON, and fans out to all connected WebSocket clients. Trajectory commands from the visualiser are encoded as `RobotBroadcast` and sent back over UDP to the appropriate robot. CLI args: `--map`, `--esp-udp`, `--ws-bind`.

**Tech Stack:** `tokio`, `axum`, `serde_json`, `postcard`, `gbp-comms` (for `RobotStateMsg` / `RobotBroadcast`), `gbp-map` (with `parse` feature). No new crates.

**Prerequisite:** M7 plan complete and passing.

---

## File Structure

```
src/bins/bridge/
  src/
    main.rs          — CLI args, tokio runtime, UDP socket, axum WS server, channel wiring
    udp_receiver.rs  — tokio task: recv UDP datagrams → postcard decode → broadcast RobotStateMsg
    ws_server.rs     — axum WS handler: fans out RobotStateMsg JSON to clients; receives control
    relay.rs         — pure logic: postcard bytes → RobotStateMsg; RobotStateMsg → JSON bytes

  Cargo.toml         — add bridge bin (if not already present)
```

---

## Chunk 1: Core relay logic + UDP receiver

### Task 1: Pure relay decode/encode functions

**Files:**
- Create: `src/bins/bridge/src/relay.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// src/bins/bridge/src/relay.rs — unit tests
#[cfg(test)]
mod tests {
    use super::*;
    use gbp_comms::RobotStateMsg;

    fn sample_msg() -> RobotStateMsg {
        RobotStateMsg {
            robot_id:            2,
            position_s:          1.5,
            velocity:            0.4,
            current_edge:        gbp_map::EdgeId(1),
            pos_3d:              [0.1, 0.2, 0.3],
            belief_means:        [0.0; gbp_comms::MAX_HORIZON],
            belief_vars:         [0.0; gbp_comms::MAX_HORIZON],
            active_factor_count: 3,
            sigma_dyn:           1.0,
            sigma_r:             0.5,
        }
    }

    #[test]
    fn postcard_bytes_decode_to_robot_state_msg() {
        let msg = sample_msg();
        let mut buf = [0u8; 512];
        let bytes = postcard::to_slice(&msg, &mut buf).unwrap();
        let decoded = decode_udp_frame(bytes).expect("decode failed");
        assert_eq!(decoded.robot_id, 2);
        assert!((decoded.position_s - 1.5).abs() < 1e-5);
    }

    #[test]
    fn robot_state_msg_encodes_to_valid_json() {
        let msg = sample_msg();
        let json = encode_to_json(&msg).expect("json encode failed");
        assert!(json.contains("\"robot_id\":2"));
        assert!(json.contains("\"position_s\":"));
        assert!(json.contains("\"velocity\":"));
    }

    #[test]
    fn decode_invalid_bytes_returns_error() {
        let garbage = b"\xFF\xFE\xAB\xCD";
        assert!(decode_udp_frame(garbage).is_err());
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `relay` module doesn't exist.

- [ ] **Step 2: Implement `relay.rs`**

```rust
// src/bins/bridge/src/relay.rs

use gbp_comms::RobotStateMsg;

/// Decode a UDP datagram (postcard-encoded) into a `RobotStateMsg`.
pub fn decode_udp_frame(bytes: &[u8]) -> Result<RobotStateMsg, postcard::Error> {
    postcard::from_bytes(bytes)
}

/// Encode a `RobotStateMsg` as a JSON string for the WebSocket stream.
pub fn encode_to_json(msg: &RobotStateMsg) -> Result<String, serde_json::Error> {
    serde_json::to_string(msg)
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```
Expected: all 3 relay tests PASS.

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/relay.rs
git commit -m "feat(m8): relay decode/encode — postcard UDP → JSON WebSocket"
```

---

### Task 2: UDP receiver task

**Files:**
- Create: `src/bins/bridge/src/udp_receiver.rs`
- Modify: `src/bins/bridge/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
// src/bins/bridge/src/udp_receiver.rs — unit tests
#[cfg(test)]
mod tests {
    use super::*;
    use gbp_comms::RobotStateMsg;

    #[test]
    fn udp_receiver_frame_size_constant_matches_msg_size() {
        // MAX_UDP_FRAME must be large enough to hold a serialised RobotStateMsg.
        let msg = RobotStateMsg {
            robot_id:            0,
            position_s:          0.0,
            velocity:            0.0,
            current_edge:        gbp_map::EdgeId(0),
            pos_3d:              [0.0; 3],
            belief_means:        [0.0; gbp_comms::MAX_HORIZON],
            belief_vars:         [0.0; gbp_comms::MAX_HORIZON],
            active_factor_count: 0,
            sigma_dyn:           0.0,
            sigma_r:             0.0,
        };
        let mut buf = [0u8; MAX_UDP_FRAME];
        // Serialisation must succeed — if it panics, increase MAX_UDP_FRAME.
        let bytes = postcard::to_slice(&msg, &mut buf).expect("frame too small");
        assert!(bytes.len() < MAX_UDP_FRAME);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `udp_receiver` module doesn't exist.

- [ ] **Step 2: Implement `udp_receiver.rs`**

```rust
// src/bins/bridge/src/udp_receiver.rs

use tokio::net::UdpSocket;
use tokio::sync::broadcast;
use gbp_comms::RobotStateMsg;
use crate::relay::decode_udp_frame;

pub const MAX_UDP_FRAME: usize = 512;

/// Tokio task: receive UDP datagrams, decode, broadcast to WS clients.
pub async fn udp_receiver_task(
    socket: UdpSocket,
    tx:     broadcast::Sender<RobotStateMsg>,
) {
    let mut buf = [0u8; MAX_UDP_FRAME];
    loop {
        match socket.recv_from(&mut buf).await {
            Ok((n, _addr)) => {
                match decode_udp_frame(&buf[..n]) {
                    Ok(msg) => { tx.send(msg).ok(); }
                    Err(e)  => { tracing::warn!("UDP decode error: {}", e); }
                }
            }
            Err(e) => {
                tracing::error!("UDP recv error: {}", e);
                break;
            }
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/udp_receiver.rs
git commit -m "feat(m8): udp_receiver_task — recv UDP datagrams and broadcast RobotStateMsg"
```

---

## Chunk 2: WebSocket server + CLI wiring

### Task 3: WebSocket server (axum)

**Files:**
- Create: `src/bins/bridge/src/ws_server.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// src/bins/bridge/src/ws_server.rs — unit tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_command_block_edge_round_trips() {
        // ControlCommand from the visualiser must deserialise correctly.
        let json = r#"{"type":"BlockEdge","edge_id":5}"#;
        let cmd: ControlCommand = serde_json::from_str(json).expect("deser");
        match cmd {
            ControlCommand::BlockEdge { edge_id } => assert_eq!(edge_id, 5),
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn control_command_set_param_round_trips() {
        let json = r#"{"type":"SetParam","key":"d_safe","value":0.35}"#;
        let cmd: ControlCommand = serde_json::from_str(json).expect("deser");
        match cmd {
            ControlCommand::SetParam { key, value } => {
                assert_eq!(key, "d_safe");
                assert!((value - 0.35).abs() < 1e-5);
            }
            _ => panic!("wrong variant"),
        }
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `ws_server` / `ControlCommand` doesn't exist.

- [ ] **Step 2: Implement `ws_server.rs`**

```rust
// src/bins/bridge/src/ws_server.rs

use axum::extract::ws::{WebSocket, Message};
use tokio::sync::broadcast;
use gbp_comms::RobotStateMsg;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type")]
pub enum ControlCommand {
    BlockEdge   { edge_id: u32 },
    UnblockEdge { edge_id: u32 },
    SetParam    { key: String, value: f32 },
    ReloadMap,
}

/// WebSocket handler — fans out RobotStateMsg JSON to the connected client.
/// Client → server messages are parsed as ControlCommand and forwarded.
pub async fn ws_handler(
    mut socket:  WebSocket,
    mut rx:      broadcast::Receiver<RobotStateMsg>,
    control_tx:  broadcast::Sender<ControlCommand>,
) {
    loop {
        tokio::select! {
            msg = rx.recv() => {
                match msg {
                    Ok(state) => {
                        if let Ok(json) = serde_json::to_string(&state) {
                            if socket.send(Message::Text(json)).await.is_err() {
                                break; // client disconnected
                            }
                        }
                    }
                    Err(_) => break,
                }
            }
            client_msg = socket.recv() => {
                match client_msg {
                    Some(Ok(Message::Text(text))) => {
                        if let Ok(cmd) = serde_json::from_str::<ControlCommand>(&text) {
                            control_tx.send(cmd).ok();
                        }
                    }
                    _ => break, // disconnect or error
                }
            }
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/ws_server.rs
git commit -m "feat(m8): ws_server — axum WebSocket fan-out + ControlCommand receive"
```

---

### Task 4: `main.rs` CLI wiring

**Files:**
- Modify: `src/bins/bridge/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
// src/bins/bridge/src/main.rs — unit test
#[test]
fn default_ws_bind_port_is_3001() {
    // Bridge binds on 3001 by default to avoid conflict with simulator (3000).
    let args = BridgeArgs::default();
    assert_eq!(args.ws_bind, "0.0.0.0:3001");
}

#[test]
fn default_esp_udp_port_is_4242() {
    let args = BridgeArgs::default();
    assert_eq!(args.esp_udp, "0.0.0.0:4242");
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p bridge 2>&1 | head -20
```
Expected: compile error — `BridgeArgs` doesn't exist.

- [ ] **Step 2: Implement `main.rs` wiring**

```rust
// src/bins/bridge/src/main.rs

use clap::Parser;
use tokio::net::UdpSocket;
use tokio::sync::broadcast;
use axum::{Router, routing::get};
use axum::extract::ws::WebSocketUpgrade;
use gbp_comms::RobotStateMsg;

#[derive(Parser)]
pub struct BridgeArgs {
    /// UDP listen address for ESP32 robot frames (postcard)
    #[arg(long, default_value = "0.0.0.0:4242")]
    pub esp_udp: String,

    /// WebSocket server bind address for visualiser
    #[arg(long, default_value = "0.0.0.0:3001")]
    pub ws_bind: String,

    /// Map YAML path (for future use — not required for relay-only operation)
    #[arg(long)]
    pub map: Option<std::path::PathBuf>,
}

impl Default for BridgeArgs {
    fn default() -> Self {
        Self {
            esp_udp: "0.0.0.0:4242".into(),
            ws_bind: "0.0.0.0:3001".into(),
            map:     None,
        }
    }
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();
    let args = BridgeArgs::parse();

    let (state_tx, _) = broadcast::channel::<RobotStateMsg>(256);
    let (ctrl_tx, _)  = broadcast::channel::<crate::ws_server::ControlCommand>(64);

    // UDP receiver task
    let socket = UdpSocket::bind(&args.esp_udp).await.expect("UDP bind failed");
    let tx_clone = state_tx.clone();
    tokio::spawn(crate::udp_receiver::udp_receiver_task(socket, tx_clone));

    // WebSocket server
    let state_tx_ws = state_tx.clone();
    let ctrl_tx_ws  = ctrl_tx.clone();
    let app = Router::new().route("/ws", get(move |ws: WebSocketUpgrade| {
        let rx  = state_tx_ws.subscribe();
        let ctx = ctrl_tx_ws.clone();
        async move { ws.on_upgrade(|socket| crate::ws_server::ws_handler(socket, rx, ctx)) }
    }));

    let listener = tokio::net::TcpListener::bind(&args.ws_bind).await.expect("WS bind failed");
    tracing::info!("bridge: UDP={} WS={}", args.esp_udp, args.ws_bind);
    axum::serve(listener, app).await.unwrap();
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/bridge/src/main.rs
git commit -m "feat(m8): bridge main — CLI args, UDP receiver, axum WS server wiring"
```

---

### Task 5: End-to-end verification

- [ ] **Step 1: Run all bridge tests**

```bash
cargo test -p bridge 2>&1 | grep -E "test |FAILED|passed|error"
```
Expected: all PASS.

- [ ] **Step 2: Smoke test with a simulated ESP32 sender**

```bash
# Terminal 1: start bridge
cargo run -p bridge -- --esp-udp 0.0.0.0:4242 --ws-bind 0.0.0.0:3001 &

# Terminal 2: send a single test UDP frame (postcard-encoded RobotStateMsg)
# Use a small test binary or netcat with pre-encoded bytes.
# Verify bridge logs: "UDP decode" and WS fan-out message.
```

- [ ] **Step 3: Connect visualiser to bridge**

```bash
# In another terminal, point visualiser at bridge port:
cargo run -p visualiser -- --ws ws://localhost:3001/ws
```

Verify:
- Visualiser connects without error
- When a real ESP32 running M7 firmware is on the same network, its state appears in the visualiser
- Block/unblock commands sent from visualiser appear in bridge logs

- [ ] **Step 4: Commit and tag M8**

```bash
git add -A
git commit -m "feat(m8): bridge hardware robot integration complete"
git tag m8-complete
```

---
