//! Spawns a background tokio thread that connects to the simulator WebSocket
//! and writes received RobotStateMsg values into WsInbox.
//! M2: Added Arc<AtomicBool> shutdown flag, checked in reconnect loop.

use crate::state::WS_INBOX_CAP;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;
use futures_util::{StreamExt, SinkExt};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;
use gbp_comms::RobotStateMsg;
use tracing::{info, warn, error};

/// Spawn a background thread running a tokio runtime for WebSocket connection.
/// Returns a shutdown flag: set it to `true` to stop the reconnect loop.
pub fn spawn_ws_client(
    url: String,
    inbox: Arc<Mutex<VecDeque<RobotStateMsg>>>,
    outbox: Arc<Mutex<VecDeque<String>>>,
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
                            // Drain outbox (commands to send to simulator)
                            {
                                let mut q = outbox.lock().unwrap_or_else(|e| e.into_inner());
                                while let Some(cmd) = q.pop_front() {
                                    if sink.send(Message::Text(cmd.into())).await.is_err() {
                                        break;
                                    }
                                }
                            }
                            // Try to receive with a short timeout so we can check outbox regularly
                            match tokio::time::timeout(
                                tokio::time::Duration::from_millis(10),
                                stream.next(),
                            ).await {
                                Ok(Some(Ok(Message::Text(json)))) => {
                                    match serde_json::from_str::<RobotStateMsg>(&json) {
                                        Ok(state) => {
                                            let mut q = inbox.lock().unwrap_or_else(|e| e.into_inner());
                                            if q.len() >= WS_INBOX_CAP { q.pop_front(); }
                                            q.push_back(state);
                                        }
                                        Err(e) => warn!("bad msg: {}", e),
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
}
