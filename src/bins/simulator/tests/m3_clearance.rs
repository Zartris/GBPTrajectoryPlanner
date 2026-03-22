//! Integration test: Robot B maintains d_safe clearance from Robot A.

use std::sync::Arc;
use tokio::sync::broadcast;
use gbp_comms::RobotBroadcast;
use gbp_core::GbpConfig;
use gbp_map::map::*;
use simulator::agent_runner::AgentRunner;
use simulator::physics::PhysicsState;
use simulator::sim_comms::SimComms;

fn single_edge_map() -> Arc<Map> {
    let mut m = Map::new("clearance_test");
    m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    m.add_node(Node { id: NodeId(1), position: [10.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
    let sf = SafetyProfile { clearance: 0.3 };
    m.add_edge(Edge {
        id: EdgeId(0), start: NodeId(0), end: NodeId(1),
        geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [10.0,0.0,0.0], length: 10.0 },
        speed: sp, safety: sf,
    }).unwrap();
    Arc::new(m)
}

fn pos_3d_for(map: &Map, edge: EdgeId, local_s: f32) -> [f32; 3] {
    map.eval_position(edge, local_s).unwrap_or([0.0, 0.0, 0.0])
}

#[test]
fn robot_b_maintains_d_safe_clearance_from_robot_a() {
    const D_SAFE: f32 = 0.3;
    const STEPS: usize = 200;
    const DT: f32 = 1.0 / 20.0;

    let map = single_edge_map();
    let (tx, rx0) = broadcast::channel::<RobotBroadcast>(64);
    let rx1 = tx.subscribe();

    let mut runner_a = AgentRunner::new(SimComms::new(tx.clone(), rx0), map.clone(), 0, &GbpConfig::default());
    runner_a.set_single_edge_trajectory(EdgeId(0), 10.0);
    let mut phys_a = PhysicsState::new(10.0);
    phys_a.position_s = 2.0;

    let mut runner_b = AgentRunner::new(SimComms::new(tx.clone(), rx1), map.clone(), 1, &GbpConfig::default());
    runner_b.set_single_edge_trajectory(EdgeId(0), 10.0);
    let mut phys_b = PhysicsState::new(10.0);
    phys_b.position_s = 0.0;

    let mut min_dist = f32::MAX;

    for _ in 0..STEPS {
        let pos_a = pos_3d_for(&map, EdgeId(0), phys_a.position_s);
        let pos_b = pos_3d_for(&map, EdgeId(0), phys_b.position_s);
        runner_a.broadcast_state(EdgeId(0), phys_a.position_s, pos_a);
        runner_b.broadcast_state(EdgeId(0), phys_b.position_s, pos_b);

        let out_a = runner_a.step(phys_a.position_s);
        let out_b = runner_b.step(phys_b.position_s);

        phys_a.velocity = out_a.velocity;
        phys_b.velocity = out_b.velocity;
        phys_a.position_s = (phys_a.position_s + phys_a.velocity * DT).clamp(0.0, 10.0);
        phys_b.position_s = (phys_b.position_s + phys_b.velocity * DT).clamp(0.0, 10.0);

        let da = out_a.pos_3d;
        let db = out_b.pos_3d;
        let dist = ((da[0]-db[0]).powi(2) + (da[1]-db[1]).powi(2) + (da[2]-db[2]).powi(2)).sqrt();
        if dist < min_dist { min_dist = dist; }

        if phys_a.position_s >= 9.5 && phys_b.position_s >= 9.5 { break; }
    }

    assert!(
        min_dist >= D_SAFE,
        "minimum observed distance {:.4} m < d_safe {:.4} m", min_dist, D_SAFE
    );
}
