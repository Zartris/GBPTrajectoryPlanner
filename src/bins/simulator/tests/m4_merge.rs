//! Integration test: Two robots on different incoming edges converging at a merge node.
//! Inter-robot factor is spawned BEFORE either robot reaches the shared post-merge edge,
//! and robots maintain 3D clearance throughout.

use std::sync::Arc;
use tokio::sync::broadcast;
use gbp_comms::RobotBroadcast;
use gbp_map::map::*;
use simulator::agent_runner::AgentRunner;
use simulator::physics::PhysicsState;
use simulator::sim_comms::SimComms;

/// Map: two edges converging at node 1, then one outgoing edge 1->3.
///   Edge 0: node 0 -> node 1 (robot A's approach, from left)
///   Edge 1: node 2 -> node 1 (robot B's approach, from above)
///   Edge 2: node 1 -> node 3 (shared post-merge edge)
fn merge_map() -> Arc<Map> {
    let mut m = Map::new("merge_test");
    m.add_node(Node { id: NodeId(0), position: [0.0, 0.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
    m.add_node(Node { id: NodeId(1), position: [5.0, 0.0, 0.0], node_type: NodeType::Merge }).unwrap();
    m.add_node(Node { id: NodeId(2), position: [5.0, 5.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
    m.add_node(Node { id: NodeId(3), position: [10.0, 0.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
    let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
    let sf = SafetyProfile { clearance: 0.3 };
    m.add_edge(Edge {
        id: EdgeId(0), start: NodeId(0), end: NodeId(1),
        geometry: EdgeGeometry::Line { start: [0.0, 0.0, 0.0], end: [5.0, 0.0, 0.0], length: 5.0 },
        speed: sp.clone(), safety: sf.clone(),
    }).unwrap();
    m.add_edge(Edge {
        id: EdgeId(1), start: NodeId(2), end: NodeId(1),
        geometry: EdgeGeometry::Line { start: [5.0, 5.0, 0.0], end: [5.0, 0.0, 0.0], length: 5.0 },
        speed: sp.clone(), safety: sf.clone(),
    }).unwrap();
    m.add_edge(Edge {
        id: EdgeId(2), start: NodeId(1), end: NodeId(3),
        geometry: EdgeGeometry::Line { start: [5.0, 0.0, 0.0], end: [10.0, 0.0, 0.0], length: 5.0 },
        speed: sp, safety: sf,
    }).unwrap();
    Arc::new(m)
}

#[test]
fn interrobot_factor_spawned_for_merge_scenario() {
    const D_SAFE: f32 = 0.3;
    const STEPS: usize = 500;
    const DT: f32 = 1.0 / 20.0;

    let map = merge_map();
    let (tx, rx0) = broadcast::channel::<RobotBroadcast>(64);
    let rx1 = tx.subscribe();

    // Robot A: edge 0 -> edge 2 (approaches merge from left)
    let mut runner_a = AgentRunner::new(SimComms::new(tx.clone(), rx0), map.clone(), 0);
    {
        let mut traj_a = heapless::Vec::<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>::new();
        traj_a.push((EdgeId(0), 5.0)).unwrap();
        traj_a.push((EdgeId(2), 5.0)).unwrap();
        runner_a.set_trajectory(traj_a);
    }
    let mut phys_a = PhysicsState::new(10.0); // total = 5+5 = 10
    phys_a.position_s = 0.0;

    // Robot B: edge 1 -> edge 2 (approaches merge from above)
    let mut runner_b = AgentRunner::new(SimComms::new(tx.clone(), rx1), map.clone(), 1);
    {
        let mut traj_b = heapless::Vec::<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>::new();
        traj_b.push((EdgeId(1), 5.0)).unwrap();
        traj_b.push((EdgeId(2), 5.0)).unwrap();
        runner_b.set_trajectory(traj_b);
    }
    let mut phys_b = PhysicsState::new(10.0); // total = 5+5 = 10
    phys_b.position_s = 0.0;

    let mut factor_ever_spawned = false;
    let mut min_dist_3d = f32::MAX;
    let mut first_on_shared_step: Option<usize> = None;
    let mut factor_spawn_step: Option<usize> = None;

    for step in 0..STEPS {
        let edge_a = runner_a.edge_at_s(phys_a.position_s).0;
        let edge_b = runner_b.edge_at_s(phys_b.position_s).0;

        runner_a.broadcast_state(edge_a, phys_a.position_s);
        runner_b.broadcast_state(edge_b, phys_b.position_s);

        let out_a = runner_a.step(phys_a.position_s);
        let out_b = runner_b.step(phys_b.position_s);

        // Track factor lifecycle
        let either_has_factor = out_a.active_factor_count > 0 || out_b.active_factor_count > 0;
        if either_has_factor {
            factor_ever_spawned = true;
            if factor_spawn_step.is_none() {
                factor_spawn_step = Some(step);
            }
        }

        // Track first step either robot arrives on shared edge (EdgeId(2))
        if first_on_shared_step.is_none()
            && (out_a.current_edge == EdgeId(2) || out_b.current_edge == EdgeId(2))
        {
            first_on_shared_step = Some(step);
        }

        phys_a.velocity = out_a.velocity;
        phys_b.velocity = out_b.velocity;
        phys_a.position_s = (phys_a.position_s + phys_a.velocity * DT).clamp(0.0, 10.0);
        phys_b.position_s = (phys_b.position_s + phys_b.velocity * DT).clamp(0.0, 10.0);

        // 3D distance
        let da = out_a.pos_3d;
        let db = out_b.pos_3d;
        let dist = ((da[0] - db[0]).powi(2) + (da[1] - db[1]).powi(2) + (da[2] - db[2]).powi(2)).sqrt();
        if dist < min_dist_3d && dist > 0.0 {
            min_dist_3d = dist;
        }

        // Exit once both are past the shared edge
        if phys_a.position_s >= 9.5 && phys_b.position_s >= 9.5 {
            break;
        }
    }

    // 1. Factor must have been spawned (robots share EdgeId(2))
    assert!(
        factor_ever_spawned,
        "inter-robot factor was never spawned — edge-set intersection gating failed"
    );

    // 2. Factor should spawn BEFORE either robot physically enters the shared edge
    if let (Some(spawn), Some(shared)) = (factor_spawn_step, first_on_shared_step) {
        assert!(
            spawn <= shared,
            "factor spawned at step {} after first robot on shared edge at step {}",
            spawn, shared
        );
    }

    // 3. 3D clearance maintained throughout
    assert!(
        min_dist_3d >= D_SAFE,
        "min 3D dist {:.4} < d_safe {:.4}",
        min_dist_3d, D_SAFE
    );
}

#[test]
fn planned_edges_snapshot_from_trims_on_merge_map() {
    let map = merge_map();
    let (tx, rx) = broadcast::channel::<RobotBroadcast>(32);
    let mut runner = AgentRunner::new(SimComms::new(tx, rx), map.clone(), 0);

    let mut traj = heapless::Vec::<(EdgeId, f32), { gbp_map::MAX_PATH_EDGES }>::new();
    traj.push((EdgeId(0), 5.0)).unwrap();
    traj.push((EdgeId(2), 5.0)).unwrap();
    runner.set_trajectory(traj);

    // From edge 0: both edges visible
    let snap = runner.planned_edges_snapshot_from(EdgeId(0));
    assert_eq!(snap.len(), 2);
    assert_eq!(snap[0], EdgeId(0));
    assert_eq!(snap[1], EdgeId(2));

    // From edge 2: only edge 2 visible
    let snap2 = runner.planned_edges_snapshot_from(EdgeId(2));
    assert_eq!(snap2.len(), 1);
    assert_eq!(snap2[0], EdgeId(2));
}
