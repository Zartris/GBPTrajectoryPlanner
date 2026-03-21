pub mod physics;
mod broadcast_task;
mod ws_server;
pub mod sim_comms;
pub mod agent_runner;

use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use tokio::sync::{broadcast, mpsc};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;
use gbp_comms::{RobotBroadcast, RobotStateMsg};
use gbp_map::map::{EdgeId, Map, NodeId, NodeType};
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
    /// Scenario: "fleet" (N random routes), "merge" (2 robots converging),
    /// "endcollision" (2 robots same route, staggered start positions)
    #[arg(long, default_value = "fleet")]
    scenario: String,
    /// Number of robots (used in fleet mode)
    #[arg(long, default_value = "4")]
    num_robots: usize,
}

/// Convert A* edge path to (EdgeId, length) pairs.
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

/// Pick `n` start nodes distributed evenly across map waypoint nodes.
fn pick_start_nodes(map: &Map, n: usize) -> std::vec::Vec<NodeId> {
    let waypoints: std::vec::Vec<NodeId> = map.nodes.iter()
        .filter(|node| node.node_type == NodeType::Waypoint)
        .map(|n| n.id)
        .collect();
    if waypoints.is_empty() { return vec![]; }
    let n = n.min(waypoints.len());
    let step = waypoints.len() / n;
    (0..n).map(|i| waypoints[i * step]).collect()
}

/// Find a node that has an edge leading to `target` (i.e. a predecessor).
fn find_predecessor(map: &Map, target: NodeId) -> Option<NodeId> {
    map.edges.iter()
        .find(|e| e.end == target)
        .map(|e| e.start)
}

/// Pick a goal node reachable from `start` (excluding start itself), selected by seed.
fn pick_random_goal(map: &Map, start: NodeId, seed: u32) -> Option<NodeId> {
    let mut reachable: std::vec::Vec<NodeId> = Vec::new();
    for node in &map.nodes {
        if node.id != start {
            if gbp_map::astar::astar(map, start, node.id).is_some() {
                reachable.push(node.id);
            }
        }
    }
    if reachable.is_empty() { return None; }
    Some(reachable[(seed as usize) % reachable.len()])
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

    let default_start = map.edges.first().expect("map has no edges").start;
    let default_goal = map.nodes.last().expect("map has no nodes").id;

    let map_arc = Arc::new(map);

    // Channels
    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    let (tx_json, _): (broadcast::Sender<String>, _) = broadcast::channel(16);
    let (cmd_tx, mut cmd_rx): (mpsc::Sender<String>, _) = mpsc::channel(8);

    let (bcast_tx, _) = broadcast::channel::<RobotBroadcast>(64);

    // Shared pause flag — checked by physics and agent tasks
    let sim_paused = Arc::new(AtomicBool::new(false));

    // Relay: RobotStateMsg -> JSON
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
                    tracing::info!("relay: channel closed");
                    break;
                }
            }
        }
    });

    // ── Build (start, goal) pairs based on scenario ──
    let assignments: std::vec::Vec<(NodeId, NodeId)> = match args.scenario.as_str() {
        "merge" => {
            // Two robots from different entry points converging on the same goal
            let s0 = NodeId(3);  // P004
            let s1 = NodeId(30); // P033
            let g = default_goal;
            if gbp_map::astar::astar(&map_arc, s0, g).is_some()
                && gbp_map::astar::astar(&map_arc, s1, g).is_some()
            {
                info!("merge scenario: robot 0 at {:?}, robot 1 at {:?}, goal {:?}", s0, s1, g);
                vec![(s0, g), (s1, g)]
            } else {
                warn!("merge scenario: P004/P033 can't reach goal, falling back to auto-discovery");
                let mut alt_start = default_start;
                let default_path = gbp_map::astar::astar(&map_arc, default_start, default_goal);
                for node in map_arc.nodes.iter() {
                    if node.id == default_start { continue; }
                    if let Some(alt_path) = gbp_map::astar::astar(&map_arc, node.id, default_goal) {
                        if let Some(ref dp) = default_path {
                            if !alt_path.is_empty() && !dp.is_empty()
                                && alt_path[0] != dp[0]
                                && alt_path.iter().any(|e| dp.contains(e))
                            {
                                alt_start = node.id;
                                break;
                            }
                        }
                    }
                }
                vec![(default_start, default_goal), (alt_start, default_goal)]
            }
        }
        "endcollision" => {
            // Two robots on the same route, staggered: robot 0 at start, robot 1 at predecessor
            let s0 = NodeId(22); // P025
            let g = NodeId(11);  // P012
            let s1 = find_predecessor(&map_arc, s0).unwrap_or(s0);
            if gbp_map::astar::astar(&map_arc, s0, g).is_some()
                && gbp_map::astar::astar(&map_arc, s1, g).is_some()
            {
                info!("endcollision scenario: robot 0 at {:?}, robot 1 at {:?}, goal {:?}", s0, s1, g);
                vec![(s0, g), (s1, g)]
            } else {
                warn!("endcollision scenario: no path, falling back to default");
                vec![(default_start, default_goal), (default_start, default_goal)]
            }
        }
        _ => {
            // Fleet mode: N robots with distributed starts and random goals
            let starts = pick_start_nodes(&map_arc, args.num_robots);
            starts.iter().enumerate().filter_map(|(i, &s)| {
                pick_random_goal(&map_arc, s, i as u32).map(|g| (s, g))
            }).collect()
        }
    };

    info!("scenario={}, spawning {} robots", args.scenario, assignments.len());

    // ── Spawn all robots in a single unified loop ──
    for (i, &(start, goal)) in assignments.iter().enumerate() {
        let rx = bcast_tx.subscribe();
        let comms = SimComms::new(bcast_tx.clone(), rx);
        let mut runner = AgentRunner::new(comms, map_arc.clone(), i as u32, &gbp_core::GbpConfig::default());

        let total_length = if let Some(path) = gbp_map::astar::astar(&map_arc, start, goal) {
            let traj = build_trajectory_edges(&map_arc, &path);
            info!("Robot {}: {} edges, {:.2}m ({:?}->{:?})",
                i, traj.len(), traj.iter().map(|(_, l)| l).sum::<f32>(), start, goal);
            runner.set_trajectory(traj)
        } else {
            warn!("Robot {}: no path from {:?} to {:?}", i, start, goal);
            0.0
        };

        let runner_arc = Arc::new(Mutex::new(runner));
        let physics = Arc::new(Mutex::new(PhysicsState::new(total_length)));

        tokio::spawn(physics::physics_task(Arc::clone(&physics), Arc::clone(&sim_paused)));
        tokio::spawn(agent_task(
            Arc::clone(&physics), runner_arc, tx_state.clone(), i as u32, Arc::clone(&sim_paused)
        ));
    }

    // Command handler (pause/resume)
    let pause_cmd = Arc::clone(&sim_paused);
    tokio::spawn(async move {
        while let Some(json) = cmd_rx.recv().await {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(&json) {
                if let Some(cmd) = v.get("command").and_then(|c| c.as_str()) {
                    match cmd {
                        "pause" => { pause_cmd.store(true, Ordering::Relaxed); info!("simulation PAUSED"); }
                        "resume" => { pause_cmd.store(false, Ordering::Relaxed); info!("simulation RESUMED"); }
                        _ => {}
                    }
                }
            }
        }
    });

    let router = ws_server::build_router(tx_json, cmd_tx);
    let listener = tokio::net::TcpListener::bind(&args.bind).await
        .unwrap_or_else(|e| panic!("cannot bind {}: {}", args.bind, e));
    let client_host = if args.bind.starts_with("0.0.0.0") {
        args.bind.replacen("0.0.0.0", "localhost", 1)
    } else {
        args.bind.clone()
    };
    info!("WebSocket server listening on ws://{}/ws (bind: {})", client_host, args.bind);
    axum::serve(listener, router).await.unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

    fn two_edge_map() -> Map {
        let mut m = Map::new("two");
        m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(1), position: [3.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(2), position: [6.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
        let sf = SafetyProfile { clearance: 0.3 };
        m.add_edge(Edge {
            id: EdgeId(0), start: NodeId(0), end: NodeId(1),
            geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [3.0,0.0,0.0], length: 3.0 },
            speed: sp.clone(), safety: sf.clone(),
        }).unwrap();
        m.add_edge(Edge {
            id: EdgeId(1), start: NodeId(1), end: NodeId(2),
            geometry: EdgeGeometry::Line { start: [3.0,0.0,0.0], end: [6.0,0.0,0.0], length: 3.0 },
            speed: sp, safety: sf,
        }).unwrap();
        m
    }

    #[test]
    fn build_trajectory_from_astar_path() {
        let map = two_edge_map();
        let path = gbp_map::astar::astar(&map, NodeId(0), NodeId(2)).unwrap();
        let traj = build_trajectory_edges(&map, &path);
        assert_eq!(traj.len(), 2);
        assert!((traj[0].1 - 3.0).abs() < 1e-5);
        assert!((traj[1].1 - 3.0).abs() < 1e-5);
    }
}
