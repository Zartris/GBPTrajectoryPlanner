mod physics;
mod broadcast_task;
mod ws_server;
mod sim_comms;
mod agent_runner;

use std::sync::{Arc, Mutex};
use tokio::sync::{broadcast, mpsc};
use tracing::info;
use tracing_subscriber::EnvFilter;
use gbp_comms::{RobotStateMsg, TrajectoryCommand};
use gbp_map::map::{EdgeId, Map, NodeId};
use heapless::Vec as HVec;
use physics::PhysicsState;
use sim_comms::SimComms;
use agent_runner::{AgentRunner, agent_task};
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

/// Convert A* edge path to (EdgeId, length) pairs for set_trajectory_from_edges.
pub fn build_trajectory_edges(
    map: &Map,
    path: &heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
) -> HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }> {
    let mut result: HVec<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }> = HVec::new();
    for &edge_id in path.iter() {
        if let Some(idx) = map.edge_index(edge_id) {
            let length = map.edges[idx].geometry.length();
            let _ = result.push((edge_id, length));
        }
    }
    result
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

    // M2: start on first edge, goal = last node in map
    let first_edge = map.edges.first().expect("map has no edges");
    let start_node = first_edge.start;
    let goal_node = map.nodes.last().expect("map has no nodes").id;
    let edge_id = first_edge.id;
    let edge_length = first_edge.geometry.length();
    info!(
        "edge {:?} length={:.2}m start={:?} goal={:?}",
        edge_id, edge_length, start_node, goal_node
    );

    let map_arc = Arc::new(map);
    let physics = Arc::new(Mutex::new(PhysicsState::new(edge_id, edge_length)));

    let runner = {
        let mut r = AgentRunner::new(SimComms, map_arc.clone(), 0);
        // Initial A* from start to goal
        if let Some(path) = gbp_map::astar::astar(&map_arc, start_node, goal_node) {
            let traj = build_trajectory_edges(&map_arc, &path);
            info!("A* found {} edges", traj.len());
            r.set_trajectory_from_edges(traj, 0.0);
        } else {
            info!("no A* path, using single edge");
            r.set_single_edge_trajectory(edge_id, edge_length);
        }
        Arc::new(Mutex::new(r))
    };

    // Domain channel: agent_task -> relay -> ws_server
    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    // JSON channel for WebSocket (ws_server is domain-agnostic)
    let (tx_json, _): (broadcast::Sender<String>, _) = broadcast::channel(16);
    // Command channel: ws_server -> command handler
    let (cmd_tx, mut cmd_rx): (mpsc::Sender<String>, _) = mpsc::channel(8);

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

    // Command handler: parse TrajectoryCommand, replan
    let map_cmd = Arc::clone(&map_arc);
    let runner_cmd = Arc::clone(&runner);
    let phys_cmd = Arc::clone(&physics);
    tokio::spawn(async move {
        while let Some(json) = cmd_rx.recv().await {
            if let Ok(cmd) = serde_json::from_str::<TrajectoryCommand>(&json) {
                let current_s = phys_cmd.lock().unwrap().position_s;
                let cur_edge = phys_cmd.lock().unwrap().current_edge;
                // Find current node (start of current edge)
                let from_node = map_cmd
                    .edges
                    .iter()
                    .find(|e| e.id == cur_edge)
                    .map(|e| e.start)
                    .unwrap_or(NodeId(0));
                if let Some(path) = gbp_map::astar::astar(&map_cmd, from_node, cmd.goal_node) {
                    let traj = build_trajectory_edges(&map_cmd, &path);
                    runner_cmd
                        .lock()
                        .unwrap()
                        .set_trajectory_from_edges(traj, current_s);
                    info!("replanned to {:?}", cmd.goal_node);
                }
            }
        }
    });

    tokio::spawn(physics::physics_task(Arc::clone(&physics)));
    tokio::spawn(agent_task(
        Arc::clone(&physics),
        runner,
        Arc::clone(&map_arc),
        tx_state,
    ));

    let router = ws_server::build_router(tx_json, cmd_tx);
    let listener = tokio::net::TcpListener::bind(&args.bind)
        .await
        .unwrap_or_else(|e| panic!("cannot bind {}: {}", args.bind, e));
    let client_host = if args.bind.starts_with("0.0.0.0") {
        args.bind.replacen("0.0.0.0", "localhost", 1)
    } else {
        args.bind.clone()
    };
    info!(
        "WebSocket server listening on ws://{}/ws (bind: {})",
        client_host, args.bind
    );
    axum::serve(listener, router).await.unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

    fn two_edge_map() -> Map {
        let mut m = Map::new("two");
        m.add_node(Node {
            id: NodeId(0),
            position: [0.0, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        })
        .unwrap();
        m.add_node(Node {
            id: NodeId(1),
            position: [3.0, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        })
        .unwrap();
        m.add_node(Node {
            id: NodeId(2),
            position: [6.0, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        })
        .unwrap();
        let sp = SpeedProfile {
            max: 2.5,
            nominal: 2.0,
            accel_limit: 1.0,
            decel_limit: 1.0,
        };
        let sf = SafetyProfile { clearance: 0.3 };
        m.add_edge(Edge {
            id: EdgeId(0),
            start: NodeId(0),
            end: NodeId(1),
            geometry: EdgeGeometry::Line {
                start: [0.0, 0.0, 0.0],
                end: [3.0, 0.0, 0.0],
                length: 3.0,
            },
            speed: sp.clone(),
            safety: sf.clone(),
        })
        .unwrap();
        m.add_edge(Edge {
            id: EdgeId(1),
            start: NodeId(1),
            end: NodeId(2),
            geometry: EdgeGeometry::Line {
                start: [3.0, 0.0, 0.0],
                end: [6.0, 0.0, 0.0],
                length: 3.0,
            },
            speed: sp,
            safety: sf,
        })
        .unwrap();
        m
    }

    #[test]
    fn build_trajectory_from_astar_path() {
        let map = two_edge_map();
        let path = gbp_map::astar::astar(&map, NodeId(0), NodeId(2)).unwrap();
        let traj = build_trajectory_edges(&map, &path);
        assert_eq!(traj.len(), 2);
        assert!((traj[0].1 - 3.0).abs() < 1e-5, "edge0 length={}", traj[0].1);
        assert!((traj[1].1 - 3.0).abs() < 1e-5, "edge1 length={}", traj[1].1);
    }
}
