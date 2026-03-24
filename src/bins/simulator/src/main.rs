pub mod physics;
mod broadcast_task;
mod ws_server;
pub mod sim_comms;
pub mod agent_runner;
pub mod toml_config;

use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicI32, AtomicU32, Ordering};
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
    /// Path to system config TOML (omit for defaults)
    #[arg(long)]
    config: Option<std::path::PathBuf>,
    /// Path to scenario TOML
    #[arg(long)]
    scenario: std::path::PathBuf,
    /// Address to bind the WebSocket server
    #[arg(long, default_value = "0.0.0.0:3000")]
    bind: String,
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
    if waypoints.is_empty() || n == 0 { return vec![]; }
    let n = n.min(waypoints.len());
    let step = waypoints.len() / n;
    (0..n).map(|i| waypoints[i * step]).collect()
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

    // Load config
    let config = if let Some(ref path) = args.config {
        let toml_str = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("cannot read config {}: {}", path.display(), e));
        toml_config::parse_config(&toml_str)
    } else {
        gbp_core::GbpConfig::default()
    };
    info!("config loaded: d_safe={}, iters={}/{}, v_min={}, v_max={}",
        config.d_safe, config.internal_iters, config.external_iters, config.v_min, config.v_max_default);

    // Load scenario
    let scenario_str = std::fs::read_to_string(&args.scenario)
        .unwrap_or_else(|e| panic!("cannot read scenario {}: {}", args.scenario.display(), e));
    let scenario = toml_config::parse_scenario(&scenario_str);
    info!("scenario '{}', map: {}", scenario.scenario.name, scenario.scenario.map);

    // Load map (path from scenario file)
    let yaml = std::fs::read_to_string(&scenario.scenario.map)
        .unwrap_or_else(|e| panic!("cannot read map {}: {}", scenario.scenario.map, e));
    let (map, node_names) = gbp_map::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("map parse error: {}", e));

    let map_arc = Arc::new(map);

    // Channels
    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    let (tx_json, _): (broadcast::Sender<String>, _) = broadcast::channel(16);
    let (cmd_tx, mut cmd_rx): (mpsc::Sender<String>, _) = mpsc::channel(8);

    let (bcast_tx, _) = broadcast::channel::<RobotBroadcast>(64);

    // Shared simulation state — checked by physics and agent tasks.
    // -1 = running, 0 = paused, N>0 = run N ticks then pause.
    let sim_state = Arc::new(AtomicI32::new(-1));
    // Tick interval in microseconds (default 20_000 = 50 Hz).
    let tick_interval_us = Arc::new(AtomicU32::new(20_000));

    // Relay: RobotStateMsg -> JSON
    // Envelope wraps any payload with a `"type"` discriminator in a single serialization pass,
    // avoiding the double-allocation of to_value() + to_string().
    #[derive(serde::Serialize)]
    struct Envelope<'a, T: serde::Serialize> {
        #[serde(rename = "type")]
        msg_type: &'a str,
        #[serde(flatten)]
        payload: &'a T,
    }

    let tx_json_relay = tx_json.clone();
    let mut rx_state = tx_state.subscribe();
    tokio::spawn(async move {
        loop {
            match rx_state.recv().await {
                Ok(msg) => {
                    match serde_json::to_string(&Envelope { msg_type: "state", payload: &msg }) {
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

    // ── Build (start, goal) pairs from scenario file ──
    let assignments: std::vec::Vec<(NodeId, NodeId)> = if !scenario.robots.is_empty() {
        // Explicit robot assignments — resolve node names
        scenario.robots.iter().map(|r| {
            let start = *node_names.get(&r.start)
                .unwrap_or_else(|| panic!("scenario node '{}' not found in map", r.start));
            let goal = *node_names.get(&r.goal)
                .unwrap_or_else(|| panic!("scenario node '{}' not found in map", r.goal));
            (start, goal)
        }).collect()
    } else if scenario.scenario.auto_assign {
        let starts = pick_start_nodes(&map_arc, scenario.scenario.num_robots);
        starts.iter().enumerate().filter_map(|(i, &s)| {
            pick_random_goal(&map_arc, s, i as u32).map(|g| (s, g))
        }).collect()
    } else {
        panic!("scenario must have [[robots]] entries or auto_assign = true");
    };

    info!("scenario '{}': spawning {} robots", scenario.scenario.name, assignments.len());

    // ── Spawn all robots in a single unified loop ──
    let mut all_runners: std::vec::Vec<Arc<Mutex<AgentRunner>>> = Vec::new();
    let mut all_physics: std::vec::Vec<Arc<Mutex<PhysicsState>>> = Vec::new();

    for (i, &(start, goal)) in assignments.iter().enumerate() {
        let rx = bcast_tx.subscribe();
        let comms = SimComms::new(bcast_tx.clone(), rx);
        let mut runner = AgentRunner::new(comms, map_arc.clone(), i as u32, &config);

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

        all_runners.push(Arc::clone(&runner_arc));
        all_physics.push(Arc::clone(&physics));

        tokio::spawn(physics::physics_task(
            Arc::clone(&physics),
            Arc::clone(&sim_state),
            Arc::clone(&tick_interval_us),
        ));
        tokio::spawn(agent_task(
            Arc::clone(&physics),
            runner_arc,
            tx_state.clone(),
            i as u32,
            Arc::clone(&sim_state),
            Arc::clone(&tick_interval_us),
        ));
    }

    // ── N-robot collision monitor (10 Hz, pairwise 3D distance) ──
    let map_mon = Arc::clone(&map_arc);
    let d_safe = config.d_safe;
    tokio::spawn(async move {
        const CHASSIS_LEN: f32 = 1.15;
        let n = all_physics.len();
        let mut collision_count: u32 = 0;
        let mut log_counter: u32 = 0;
        let mut ticker = tokio::time::interval(tokio::time::Duration::from_millis(100));
        loop {
            ticker.tick().await;
            log_counter += 1;
            // Collect positions for all robots
            let mut positions: std::vec::Vec<([f32; 3], f32, f32)> = Vec::with_capacity(n); // (pos_3d, s, v)
            for i in 0..n {
                let (s, v) = {
                    let p = all_physics[i].lock().unwrap_or_else(|e| e.into_inner());
                    (p.position_s, p.velocity)
                };
                let (edge, local) = all_runners[i].lock().unwrap_or_else(|e| e.into_inner()).edge_at_s(s);
                let pos = map_mon.eval_position(edge, local).unwrap_or([0.0; 3]);
                positions.push((pos, s, v));
            }
            // Pairwise collision check
            for i in 0..n {
                for j in (i+1)..n {
                    let (pi, si, vi) = positions[i];
                    let (pj, sj, vj) = positions[j];
                    let dx = pi[0]-pj[0]; let dy = pi[1]-pj[1]; let dz = pi[2]-pj[2];
                    let dist = (dx*dx + dy*dy + dz*dz).sqrt();
                    if dist < CHASSIS_LEN && si > 1.5 && sj > 1.5 {
                        collision_count += 1;
                        warn!(
                            "COLLISION R{}↔R{}: dist={:.3}m (< {:.2}m) \
                             R{}: s={:.2} v={:.2} pos=[{:.2},{:.2},{:.2}] \
                             R{}: s={:.2} v={:.2} pos=[{:.2},{:.2},{:.2}]",
                            i, j, dist, CHASSIS_LEN,
                            i, si, vi, pi[0], pi[1], pi[2],
                            j, sj, vj, pj[0], pj[1], pj[2],
                        );
                    }
                }
            }
            // Summary log every 5s
            if log_counter % 50 == 0 {
                let mut min_dist = f32::MAX;
                let mut min_pair = (0, 0);
                for i in 0..n {
                    for j in (i+1)..n {
                        let (pi, _, _) = positions[i];
                        let (pj, _, _) = positions[j];
                        let dx = pi[0]-pj[0]; let dy = pi[1]-pj[1]; let dz = pi[2]-pj[2];
                        let d = (dx*dx + dy*dy + dz*dz).sqrt();
                        if d < min_dist { min_dist = d; min_pair = (i, j); }
                    }
                }
                info!("MONITOR: {} robots, min_dist={:.2}m (R{}↔R{}), collisions={}, d_safe={}",
                    n, min_dist, min_pair.0, min_pair.1, collision_count, d_safe);
            }
        }
    });

    // Command handler (pause/resume/step/set_timescale)
    let cmd_sim_state = Arc::clone(&sim_state);
    let cmd_tick_interval_us = Arc::clone(&tick_interval_us);
    tokio::spawn(async move {
        while let Some(json) = cmd_rx.recv().await {
            if let Ok(v) = serde_json::from_str::<serde_json::Value>(&json) {
                if let Some(cmd) = v.get("command").and_then(|c| c.as_str()) {
                    match cmd {
                        "pause" => {
                            cmd_sim_state.store(0, Ordering::Relaxed);
                            info!("simulation PAUSED");
                        }
                        "resume" => {
                            cmd_sim_state.store(-1, Ordering::Relaxed);
                            info!("simulation RESUMED");
                        }
                        "step" => {
                            let ticks = (v.get("ticks")
                                .and_then(|t| t.as_i64())
                                .unwrap_or(1)
                                .max(1)
                                .min(i32::MAX as i64)) as i32;
                            cmd_sim_state.store(ticks, Ordering::Relaxed);
                            info!("simulation STEP {} ticks", ticks);
                        }
                        "set_timescale" => {
                            if let Some(scale) = v.get("scale").and_then(|s| s.as_f64()) {
                                if scale > 0.0 {
                                    let us = ((20_000.0 / scale).clamp(100.0, 60_000_000.0)) as u32;
                                    cmd_tick_interval_us.store(us, Ordering::Relaxed);
                                    info!("simulation timescale={:.3}x tick_interval={}us", scale, us);
                                } else {
                                    warn!("set_timescale: scale must be > 0, got {}", scale);
                                }
                            } else {
                                warn!("set_timescale: missing or invalid 'scale' field");
                            }
                        }
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
