mod physics;
mod broadcast_task;
mod ws_server;

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tracing::info;
use tracing_subscriber::EnvFilter;
use gbp_comms::RobotStateMsg;
use physics::PhysicsState;
use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Path to map YAML file
    #[arg(long)]
    map: std::path::PathBuf,
    /// Address to bind the WebSocket server
    #[arg(long, default_value = "0.0.0.0:3000")]
    bind: String,
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("simulator=info")),
        )
        .init();

    let args = Args::parse();
    info!("loading map from {}", args.map.display());

    let yaml = std::fs::read_to_string(&args.map)
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

    // Domain channel: broadcast_task -> relay -> ws_server
    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    // JSON channel for WebSocket (ws_server is domain-agnostic)
    let (tx_json, _): (broadcast::Sender<String>, _) = broadcast::channel(16);

    // Relay: RobotStateMsg -> JSON string.
    let tx_json_relay = tx_json.clone();
    let mut rx_state = tx_state.subscribe();
    tokio::spawn(async move {
        loop {
            match rx_state.recv().await {
                Ok(msg) => {
                    match serde_json::to_string(&msg) {
                        Ok(json) => { let _ = tx_json_relay.send(json); }
                        Err(e) => tracing::error!("relay: JSON serialize failed: {}", e),
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    tracing::warn!("relay: skipped {} messages", n);
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    tracing::info!("relay: broadcast channel closed, shutting down");
                    break;
                }
            }
        }
    });

    tokio::spawn(physics::physics_task(Arc::clone(&physics)));
    tokio::spawn(broadcast_task::broadcast_task(
        Arc::clone(&physics), edge_id, v_nom, tx_state,
    ));

    let router   = ws_server::build_router(tx_json);
    let listener = tokio::net::TcpListener::bind(&args.bind).await
        .unwrap_or_else(|e| panic!("cannot bind {}: {}", args.bind, e));
    let client_host = if args.bind.starts_with("0.0.0.0") { args.bind.replacen("0.0.0.0", "localhost", 1) } else { args.bind.clone() };
    info!("WebSocket server listening on ws://{}/ws (bind: {})", client_host, args.bind);
    axum::serve(listener, router).await.unwrap();
}
