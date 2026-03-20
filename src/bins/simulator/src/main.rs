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

    // Domain channel: broadcast_task -> relay -> ws_server
    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    // JSON channel for WebSocket (ws_server is domain-agnostic)
    let (tx_json, _): (broadcast::Sender<String>, _) = broadcast::channel(16);

    // Relay: RobotStateMsg -> JSON string.
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
