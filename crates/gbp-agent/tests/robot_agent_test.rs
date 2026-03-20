use gbp_agent::RobotAgent;
use gbp_comms::{ObservationUpdate, RobotBroadcast};
use gbp_map::map::{EdgeId, Map, Node, NodeId, NodeType, Edge, EdgeGeometry, SpeedProfile, SafetyProfile};
use heapless::Vec;

struct NullComms;
impl gbp_comms::CommsInterface for NullComms {
    type Error = ();
    fn broadcast(&mut self, _: &RobotBroadcast) -> Result<(), ()> { Ok(()) }
    fn receive_broadcasts(&mut self) -> Vec<RobotBroadcast, 8> { Vec::new() }
}

fn one_edge_map() -> Map {
    let mut m = Map::new("one");
    m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    m.add_node(Node { id: NodeId(1), position: [10.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    m.add_edge(Edge {
        id: EdgeId(0), start: NodeId(0), end: NodeId(1),
        geometry: EdgeGeometry::Line { start:[0.0,0.0,0.0], end:[10.0,0.0,0.0], length:10.0 },
        speed: SpeedProfile { max:2.5, nominal:2.0, accel_limit:1.0, decel_limit:1.0 },
        safety: SafetyProfile { clearance: 0.3 },
    }).unwrap();
    m
}

#[test]
fn step_produces_nonzero_velocity_when_trajectory_set() {
    let map = one_edge_map();
    let mut agent = RobotAgent::new(NullComms, &map, 0);
    let mut edges: Vec<(EdgeId, f32), 32> = Vec::new();
    edges.push((EdgeId(0), 10.0)).unwrap();
    agent.set_trajectory(edges, 0.0);

    let obs = ObservationUpdate { position_s: 0.0, velocity: 0.0, current_edge: EdgeId(0) };
    let output = agent.step(obs);
    assert!(output.velocity > 0.0, "velocity should be positive after step");
}
