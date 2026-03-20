// src/bins/simulator/src/ws_server.rs
use axum::{
    extract::{State, WebSocketUpgrade},
    response::Response,
    routing::get,
    Router,
};
use axum::extract::ws::{Message, WebSocket};
use tokio::sync::broadcast;
use futures_util::SinkExt;
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
    info!("visualiser disconnected");
}

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
