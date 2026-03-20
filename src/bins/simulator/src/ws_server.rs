// src/bins/simulator/src/ws_server.rs
use axum::{
    extract::{State, WebSocketUpgrade},
    response::Response,
    routing::get,
    Router,
};
use axum::extract::ws::{Message, WebSocket};
use tokio::sync::{broadcast, mpsc};
use tracing::{info, warn};

pub type WsTx = broadcast::Sender<String>;

#[derive(Clone)]
struct AppState {
    tx: WsTx,
    cmd_tx: mpsc::Sender<String>,
}

/// Build the axum router. Separated from bind so it can be tested.
pub fn build_router(tx: WsTx, cmd_tx: mpsc::Sender<String>) -> Router {
    let state = AppState { tx, cmd_tx };
    Router::new()
        .route("/ws", get(ws_handler))
        .with_state(state)
}

async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
) -> Response {
    ws.on_upgrade(move |socket| handle_socket(socket, state.tx, state.cmd_tx))
}

async fn handle_socket(mut socket: WebSocket, tx: WsTx, cmd_tx: mpsc::Sender<String>) {
    let mut rx = tx.subscribe();
    info!("visualiser connected");
    loop {
        tokio::select! {
            result = rx.recv() => {
                match result {
                    Ok(json) => {
                        if socket.send(Message::Text(json.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        warn!("ws lagged {} frames", n);
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
            msg = socket.recv() => {
                match msg {
                    Some(Ok(Message::Text(cmd))) => {
                        let _ = cmd_tx.send(cmd.to_string()).await;
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    Some(Err(_)) => break,
                    _ => {}
                }
            }
        }
    }
    info!("visualiser disconnected");
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::{broadcast, mpsc};

    #[tokio::test]
    async fn ws_router_builds_without_panic() {
        let (tx, _rx) = broadcast::channel::<String>(16);
        let (cmd_tx, _cmd_rx) = mpsc::channel::<String>(8);
        // Should not panic
        let _router = build_router(tx, cmd_tx);
    }
}
