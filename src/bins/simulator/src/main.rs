pub mod physics;
mod broadcast_task;
mod ws_server;
pub mod sim_comms;
pub mod agent_runner;

use std::sync::{Arc, Mutex};
use tokio::sync::{broadcast, mpsc};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;
use gbp_comms::{RobotBroadcast, RobotStateMsg, TrajectoryCommand};
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
    /// Scenario: "follow" (same route) or "merge" (different routes converging)
    #[arg(long, default_value = "follow")]
    scenario: String,
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

    // Determine start/goal per robot based on scenario
    let (start0, goal0, start1, goal1) = if args.scenario == "merge" {
        // P004 = NodeId(3), P033 = NodeId(30) — two different entry points into the loop
        let s0 = NodeId(3);  // P004
        let s1 = NodeId(30); // P033
        // Verify both can reach the goal
        let g = default_goal;
        if gbp_map::astar::astar(&map_arc, s0, g).is_some()
            && gbp_map::astar::astar(&map_arc, s1, g).is_some()
        {
            info!("merge scenario: robot 0 starts at P004 (NodeId(3)), robot 1 at P033 (NodeId(30))");
            (s0, g, s1, g)
        } else {
            info!("merge scenario: P004/P033 can't reach goal, falling back to auto-discovery");
            // Fallback: auto-discover an alternate start
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
            (default_start, default_goal, alt_start, default_goal)
        }
    } else if args.scenario == "endcollision" {
        // P025 → P012: single edge, both robots same route, robot 1 spawns 1s late
        let s = NodeId(22); // P025
        let g = NodeId(11); // P012
        if gbp_map::astar::astar(&map_arc, s, g).is_some() {
            info!("endcollision scenario: both start at P025 (NodeId(22)), goal P012 (NodeId(11))");
            (s, g, s, g)
        } else {
            info!("endcollision scenario: P025->P012 has no path, falling back to default");
            (default_start, default_goal, default_start, default_goal)
        }
    } else {
        (default_start, default_goal, default_start, default_goal)
    };

    info!("scenario={}, robot0: {:?}->{:?}, robot1: {:?}->{:?}",
        args.scenario, start0, goal0, start1, goal1);

    // Channels
    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    let (tx_json, _): (broadcast::Sender<String>, _) = broadcast::channel(16);
    let (cmd_tx, mut cmd_rx): (mpsc::Sender<String>, _) = mpsc::channel(8);

    let (bcast_tx, bcast_rx0) = broadcast::channel::<RobotBroadcast>(64);
    let bcast_rx1 = bcast_tx.subscribe();

    // Robot 0
    let (runner0, total_length0) = {
        let comms0 = SimComms::new(bcast_tx.clone(), bcast_rx0);
        let mut r = AgentRunner::new(comms0, map_arc.clone(), 0);
        let total = if let Some(path) = gbp_map::astar::astar(&map_arc, start0, goal0) {
            let traj = build_trajectory_edges(&map_arc, &path);
            info!("Robot 0: {} edges, {:.2}m", traj.len(), traj.iter().map(|(_, l)| l).sum::<f32>());
            r.set_trajectory(traj)
        } else {
            info!("Robot 0: no path");
            let e = map_arc.edges.first().unwrap();
            let mut t = HVec::new();
            let _ = t.push((e.id, e.geometry.length()));
            r.set_trajectory(t)
        };
        (Arc::new(Mutex::new(r)), total)
    };

    // Robot 1
    let (runner1, total_length1) = {
        let comms1 = SimComms::new(bcast_tx.clone(), bcast_rx1);
        let mut r = AgentRunner::new(comms1, map_arc.clone(), 1);
        let total = if let Some(path) = gbp_map::astar::astar(&map_arc, start1, goal1) {
            let traj = build_trajectory_edges(&map_arc, &path);
            info!("Robot 1: {} edges, {:.2}m", traj.len(), traj.iter().map(|(_, l)| l).sum::<f32>());
            r.set_trajectory(traj)
        } else {
            info!("Robot 1: no path");
            let e = map_arc.edges.first().unwrap();
            let mut t = HVec::new();
            let _ = t.push((e.id, e.geometry.length()));
            r.set_trajectory(t)
        };
        (Arc::new(Mutex::new(r)), total)
    };

    let physics0 = Arc::new(Mutex::new(PhysicsState::new(total_length0)));
    let delayed_start = args.scenario == "endcollision";
    if start0 == start1 && !delayed_start {
        physics0.lock().unwrap().position_s = 1.5; // head start >= D_SAFE to avoid initial chassis overlap
    }
    // When both robots share the same goal, Robot 1 must stop d_safe before
    // the goal. Without this, both robots' physics clamp at the same total_length,
    // causing dist=0 pile-up at the edge end. We shorten Robot 1's trajectory
    // so its v_nom taper decelerates to zero d_safe before the goal, and cap
    // the physics to match.
    const D_SAFE: f32 = 1.3;
    if goal0 == goal1 {
        let mut r1 = runner1.lock().unwrap();
        // Shorten the last edge in R1's trajectory by d_safe so the
        // trapezoidal v_nom taper brings R1 to a stop earlier.
        let shortened = (total_length1 - D_SAFE).max(0.1);
        // Re-build trajectory with shortened last edge
        let mut traj = HVec::<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>::new();
        let edges = r1.planned_edge_ids();
        let mut remaining = shortened;
        for &eid in edges.iter() {
            if let Some(idx) = map_arc.edge_index(eid) {
                let len = map_arc.edges[idx].geometry.length();
                let use_len = len.min(remaining);
                if use_len > 0.01 {
                    let _ = traj.push((eid, use_len));
                }
                remaining -= use_len;
                if remaining <= 0.0 { break; }
            }
        }
        r1.set_trajectory(traj);
        drop(r1);
    }
    let r1_total = if goal0 == goal1 {
        (total_length1 - D_SAFE).max(0.1)
    } else {
        total_length1
    };
    let physics1 = Arc::new(Mutex::new(PhysicsState::new(r1_total)));

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

    // Command handler (simplified for M3 — uses robot 0 only)
    let map_cmd = Arc::clone(&map_arc);
    let runner_cmd = Arc::clone(&runner0);
    let phys_cmd = Arc::clone(&physics0);
    tokio::spawn(async move {
        while let Some(json) = cmd_rx.recv().await {
            if let Ok(cmd) = serde_json::from_str::<TrajectoryCommand>(&json) {
                let global_s = phys_cmd.lock().unwrap_or_else(|e| e.into_inner()).position_s;
                let (cur_edge, _) = runner_cmd.lock().unwrap_or_else(|e| e.into_inner()).edge_at_s(global_s);
                let from_node = match map_cmd.edges.iter().find(|e| e.id == cur_edge).map(|e| e.start) {
                    Some(node) => node,
                    None => {
                        tracing::warn!("command handler: edge {:?} not in map, ignoring", cur_edge);
                        continue;
                    }
                };
                if let Some(path) = gbp_map::astar::astar(&map_cmd, from_node, cmd.goal_node) {
                    let traj = build_trajectory_edges(&map_cmd, &path);
                    let mut r = runner_cmd.lock().unwrap_or_else(|e| e.into_inner());
                    let new_total = r.set_trajectory(traj);
                    let mut p = phys_cmd.lock().unwrap_or_else(|e| e.into_inner());
                    p.position_s = 0.0;
                    p.total_length = new_total;
                    info!("replanned to {:?} ({:.2}m)", cmd.goal_node, new_total);
                }
            }
        }
    });

    // Spawn physics and agent tasks for both robots
    // Clone Arcs for the distance monitor before moving into agent tasks
    let r0_mon = Arc::clone(&runner0);
    let r1_mon = Arc::clone(&runner1);
    let p0_mon = Arc::clone(&physics0);
    let p1_mon = Arc::clone(&physics1);

    tokio::spawn(physics::physics_task(Arc::clone(&physics0)));
    tokio::spawn(agent_task(Arc::clone(&physics0), runner0, tx_state.clone(), 0));

    // Robot 1: delay spawn by 1s in endcollision scenario
    let physics1_clone = Arc::clone(&physics1);
    let runner1_arc = runner1;
    let tx1 = tx_state.clone();
    tokio::spawn(async move {
        if delayed_start {
            info!("endcollision: robot 1 spawns in 2 seconds...");
            tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
            info!("endcollision: robot 1 spawning now");
        }
        tokio::spawn(physics::physics_task(Arc::clone(&physics1_clone)));
        agent_task(physics1_clone, runner1_arc, tx1, 1).await;
    });
    let map_mon = Arc::clone(&map_arc);
    tokio::spawn(async move {
        // Collision detection state
        const CHASSIS_LEN: f32 = 1.15; // center-to-center overlap threshold
        let mut collision_active = false;
        let mut collision_start_s: (f32, f32) = (0.0, 0.0);
        let mut collision_min_dist: f32 = f32::MAX;
        let mut collision_count: u32 = 0;
        let mut last_collision_log = std::time::Instant::now();

        let mut ticker = tokio::time::interval(tokio::time::Duration::from_millis(100)); // 10 Hz for finer collision detection
        let mut dist_log_counter: u32 = 0;
        loop {
            ticker.tick().await;
            let s0 = p0_mon.lock().unwrap_or_else(|e| e.into_inner()).position_s;
            let s1 = p1_mon.lock().unwrap_or_else(|e| e.into_inner()).position_s;
            let v0 = p0_mon.lock().unwrap_or_else(|e| e.into_inner()).velocity;
            let v1 = p1_mon.lock().unwrap_or_else(|e| e.into_inner()).velocity;
            let (edge0, local0) = r0_mon.lock().unwrap_or_else(|e| e.into_inner()).edge_at_s(s0);
            let (edge1, local1) = r1_mon.lock().unwrap_or_else(|e| e.into_inner()).edge_at_s(s1);
            // 3D distance
            let p0 = map_mon.eval_position(edge0, local0).unwrap_or([0.0; 3]);
            let p1 = map_mon.eval_position(edge1, local1).unwrap_or([0.0; 3]);
            let dx = p0[0]-p1[0]; let dy = p0[1]-p1[1]; let dz = p0[2]-p1[2];
            let dist_3d = (dx*dx + dy*dy + dz*dz).sqrt();

            // Collision detection: chassis overlap when center-to-center < CHASSIS_LEN
            // Ignore overlap near start — robots spawn at s≈0 with initial overlap
            // in endcollision/follow scenarios. Only flag collisions after both have moved.
            let either_near_start = s0 < 1.5 || s1 < 1.5;
            let overlapping = dist_3d < CHASSIS_LEN && !either_near_start;
            if overlapping {
                if !collision_active {
                    // New collision event
                    collision_active = true;
                    collision_count += 1;
                    collision_start_s = (s0, s1);
                    collision_min_dist = dist_3d;
                    warn!(
                        "COLLISION #{}: robots overlapping! dist={:.3}m (< chassis {:.2}m) \
                         R0: s={:.2} v={:.2} edge={:?} pos=[{:.2},{:.2},{:.2}] \
                         R1: s={:.2} v={:.2} edge={:?} pos=[{:.2},{:.2},{:.2}]",
                        collision_count, dist_3d, CHASSIS_LEN,
                        s0, v0, edge0, p0[0], p0[1], p0[2],
                        s1, v1, edge1, p1[0], p1[1], p1[2],
                    );
                    last_collision_log = std::time::Instant::now();
                } else {
                    // Ongoing collision — throttle to once per second
                    collision_min_dist = collision_min_dist.min(dist_3d);
                    if last_collision_log.elapsed().as_secs_f32() >= 1.0 {
                        warn!(
                            "COLLISION #{} ongoing: dist={:.3}m min={:.3}m \
                             R0: s={:.2} v={:.2} R1: s={:.2} v={:.2}",
                            collision_count, dist_3d, collision_min_dist, s0, v0, s1, v1,
                        );
                        last_collision_log = std::time::Instant::now();
                    }
                }
            } else if collision_active {
                // Collision ended
                info!(
                    "COLLISION #{} resolved: min_dist={:.3}m R0 moved {:.2}m R1 moved {:.2}m",
                    collision_count, collision_min_dist,
                    (s0 - collision_start_s.0).abs(),
                    (s1 - collision_start_s.1).abs(),
                );
                collision_active = false;
                collision_min_dist = f32::MAX;
            }

            // Regular DIST log every 1s (every 10th tick at 10 Hz)
            dist_log_counter += 1;
            if dist_log_counter % 10 == 0 {
                info!("DIST: s0={:.2} s1={:.2} 3d={:.3}m edge0={:?} edge1={:?} (d_safe={}) collisions={}",
                    s0, s1, dist_3d, edge0, edge1, D_SAFE, collision_count);
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
