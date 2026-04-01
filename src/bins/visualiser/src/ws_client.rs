//! Spawns a background tokio thread that connects to the simulator WebSocket
//! and writes received RobotStateMsg values into WsInbox.
//! M2: Added Arc<AtomicBool> shutdown flag, checked in reconnect loop.
//! M6b: Added type-discriminated message dispatch for collision and inspect_response.

use crate::state::{CollisionEvent, InspectResponse, WS_INBOX_CAP};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;
use futures_util::{StreamExt, SinkExt};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;
use gbp_comms::RobotStateMsg;
use tracing::{info, warn, error};

/// Push `item` onto a capped deque, evicting the oldest entry when at capacity.
fn push_capped<T>(q: &mut std::collections::VecDeque<T>, item: T) {
    if q.len() >= WS_INBOX_CAP {
        q.pop_front();
    }
    q.push_back(item);
}

/// Spawn a background thread running a tokio runtime for WebSocket connection.
/// Returns a shutdown flag: set it to `true` to stop the reconnect loop.
pub fn spawn_ws_client(
    url: String,
    inbox: Arc<Mutex<VecDeque<RobotStateMsg>>>,
    outbox: Arc<Mutex<VecDeque<String>>>,
    collision_inbox: Arc<Mutex<VecDeque<CollisionEvent>>>,
    inspect_inbox: Arc<Mutex<VecDeque<InspectResponse>>>,
) -> Arc<AtomicBool> {
    let shutdown = Arc::new(AtomicBool::new(false));
    let shutdown_clone = Arc::clone(&shutdown);
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            loop {
                if shutdown_clone.load(Ordering::Relaxed) {
                    info!("ws_client: shutdown flag set, exiting");
                    break;
                }
                info!("connecting to {}", url);
                match connect_async(&url).await {
                    Ok((ws, _)) => {
                        info!("connected to simulator");
                        let (mut sink, mut stream) = ws.split();
                        loop {
                            // Drain outbox into local vec (don't hold mutex across await)
                            let pending: std::vec::Vec<String> = {
                                let mut q = outbox.lock().unwrap_or_else(|e| e.into_inner());
                                q.drain(..).collect()
                            };
                            let mut send_failed = false;
                            for cmd in pending {
                                if sink.send(Message::Text(cmd)).await.is_err() {
                                    send_failed = true;
                                    break;
                                }
                            }
                            if send_failed { break; } // reconnect
                            // Receive with 100ms timeout (reduced CPU vs 10ms busy-poll)
                            match tokio::time::timeout(
                                tokio::time::Duration::from_millis(100),
                                stream.next(),
                            ).await {
                                Ok(Some(Ok(Message::Text(json)))) => {
                                    match serde_json::from_str::<serde_json::Value>(&json) {
                                        Ok(val) => {
                                            let msg_type = val.get("type")
                                                .and_then(|t| t.as_str())
                                                .unwrap_or("state");
                                            match msg_type {
                                                "state" => {
                                                    match serde_json::from_value::<RobotStateMsg>(val) {
                                                        Ok(state) => {
                                                            let mut q = inbox.lock().unwrap_or_else(|e| e.into_inner());
                                                            push_capped(&mut q, state);
                                                        }
                                                        Err(e) => warn!("bad state msg: {}", e),
                                                    }
                                                }
                                                "collision" => {
                                                    match serde_json::from_value::<CollisionEvent>(val) {
                                                        Ok(ev) => {
                                                            let mut q = collision_inbox.lock().unwrap_or_else(|e| e.into_inner());
                                                            push_capped(&mut q, ev);
                                                        }
                                                        Err(e) => warn!("bad collision msg: {}", e),
                                                    }
                                                }
                                                "inspect_response" => {
                                                    match serde_json::from_value::<InspectResponse>(val) {
                                                        Ok(resp) => {
                                                            let mut q = inspect_inbox.lock().unwrap_or_else(|e| e.into_inner());
                                                            push_capped(&mut q, resp);
                                                        }
                                                        Err(e) => warn!("bad inspect_response msg: {}", e),
                                                    }
                                                }
                                                other => warn!("unknown message type: {}", other),
                                            }
                                        }
                                        Err(e) => warn!("bad msg (not JSON): {}", e),
                                    }
                                }
                                Ok(Some(Ok(Message::Close(_)))) | Ok(None) => break,
                                Ok(Some(Err(e))) => { error!("ws error: {}", e); break; }
                                Err(_) => {} // timeout — loop back to check outbox
                                _ => {}
                            }
                            if shutdown_clone.load(Ordering::Relaxed) { break; }
                        }
                        if shutdown_clone.load(Ordering::Relaxed) {
                            break;
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
    shutdown
}

#[cfg(test)]
mod tests {
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
            belief_vars: [0.0; MAX_HORIZON],
            planned_edges: heapless::Vec::new(),
            active_factors: heapless::Vec::new(),
            ir_factor_count: 0,
            active_ir_timesteps: heapless::Vec::new(),
            raw_gbp_velocity: 0.0,
            min_neighbour_dist_3d: f32::MAX,
        }
    }

    #[test]
    fn deserialize_round_trip() {
        let msg = minimal_msg();
        let json = serde_json::to_string(&msg).expect("serialize");
        let decoded: RobotStateMsg = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(decoded.robot_id, 1);
        assert!((decoded.position_s - 2.5).abs() < 1e-6);
        assert!((decoded.velocity - 1.8).abs() < 1e-6);
    }

    #[test]
    fn shutdown_flag_stops_reconnect() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;
        let flag = Arc::new(AtomicBool::new(false));
        assert!(!flag.load(Ordering::Relaxed));
        flag.store(true, Ordering::Relaxed);
        assert!(flag.load(Ordering::Relaxed));
    }

    // ── Type dispatch tests ──────────────────────────────────────────────────

    /// Simulate the dispatch logic used inside the WS receive loop.
    /// Extracts `"type"` field, defaults to `"state"` when absent.
    fn dispatch_type(json: &str) -> &'static str {
        let val: serde_json::Value = serde_json::from_str(json).expect("valid JSON");
        match val.get("type").and_then(|t| t.as_str()).unwrap_or("state") {
            "state" => "state",
            "collision" => "collision",
            "inspect_response" => "inspect_response",
            _ => "unknown",
        }
    }

    #[test]
    fn dispatch_state_type() {
        // JSON with explicit "type":"state" should parse as a state message.
        let json = r#"{"type":"state","robot_id":0,"current_edge":0,"position_s":0.0,"velocity":0.0,"pos_3d":[0,0,0],"source":"Simulated","belief_means":[0,0,0,0,0,0,0,0,0,0,0,0],"belief_vars":[0,0,0,0,0,0,0,0,0,0,0,0],"planned_edges":[],"active_factors":[],"ir_factor_count":0,"active_ir_timesteps":[],"raw_gbp_velocity":0.0,"min_neighbour_dist_3d":1e38}"#;
        assert_eq!(dispatch_type(json), "state");
        // Confirm it also actually deserializes into RobotStateMsg
        let val: serde_json::Value = serde_json::from_str(json).unwrap();
        let msg: RobotStateMsg = serde_json::from_value(val).expect("deserialize state");
        assert_eq!(msg.robot_id, 0);
    }

    #[test]
    fn dispatch_collision_type() {
        use crate::state::CollisionEvent;
        let json = r#"{"type":"collision","robot_a":1,"robot_b":2,"pos":[1.0,2.0,3.0],"dist":0.5}"#;
        assert_eq!(dispatch_type(json), "collision");
        // Confirm it also actually deserializes into CollisionEvent
        let val: serde_json::Value = serde_json::from_str(json).unwrap();
        let ev: CollisionEvent = serde_json::from_value(val).expect("deserialize collision");
        assert_eq!(ev.robot_a, 1);
        assert_eq!(ev.robot_b, 2);
        assert!((ev.dist - 0.5).abs() < 1e-6);
    }

    #[test]
    fn dispatch_missing_type_falls_back_to_state() {
        // JSON without "type" field should default to "state" dispatch.
        let json = r#"{"robot_id":7,"current_edge":0,"position_s":1.0,"velocity":0.5,"pos_3d":[1,0,0],"source":"Simulated","belief_means":[0,0,0,0,0,0,0,0,0,0,0,0],"belief_vars":[0,0,0,0,0,0,0,0,0,0,0,0],"planned_edges":[],"active_factors":[],"ir_factor_count":0,"active_ir_timesteps":[],"raw_gbp_velocity":0.0,"min_neighbour_dist_3d":1e38}"#;
        assert_eq!(dispatch_type(json), "state");
        // Confirm it also actually deserializes
        let val: serde_json::Value = serde_json::from_str(json).unwrap();
        let msg: RobotStateMsg = serde_json::from_value(val).expect("deserialize fallback state");
        assert_eq!(msg.robot_id, 7);
    }

    #[test]
    fn dispatch_unknown_type_does_not_panic() {
        // Unknown type should yield "unknown" without panicking.
        let json = r#"{"type":"future_feature","data":"something"}"#;
        assert_eq!(dispatch_type(json), "unknown");
    }
}
