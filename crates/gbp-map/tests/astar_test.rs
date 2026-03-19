use gbp_map::astar::astar;
use gbp_map::map::*;

fn two_path_map() -> Map {
    // Two paths from node 0 to node 3:
    //   Short but slow: 0->1->3  (lengths 1.0, 1.0; nominal 1.0 m/s -> time = 2.0)
    //   Long but fast:  0->2->3  (lengths 2.0, 2.0; nominal 4.0 m/s -> time = 1.0)
    // A* should pick 0->2->3 (lower traversal time).
    let mut m = Map::new("test");
    for i in 0..4u16 {
        m.add_node(Node { id: NodeId(i), position: [i as f32, 0.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
    }
    let slow_speed = SpeedProfile { max: 1.0, nominal: 1.0, accel_limit: 1.0, decel_limit: 1.0 };
    let fast_speed = SpeedProfile { max: 4.0, nominal: 4.0, accel_limit: 1.0, decel_limit: 1.0 };
    let safety = SafetyProfile { clearance: 0.3 };
    m.add_edge(Edge { id: EdgeId(0), start: NodeId(0), end: NodeId(1), geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [1.0,0.0,0.0], length: 1.0 }, speed: slow_speed.clone(), safety: safety.clone() }).unwrap();
    m.add_edge(Edge { id: EdgeId(1), start: NodeId(1), end: NodeId(3), geometry: EdgeGeometry::Line { start: [1.0,0.0,0.0], end: [3.0,0.0,0.0], length: 1.0 }, speed: slow_speed.clone(), safety: safety.clone() }).unwrap();
    m.add_edge(Edge { id: EdgeId(2), start: NodeId(0), end: NodeId(2), geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [2.0,0.0,0.0], length: 2.0 }, speed: fast_speed.clone(), safety: safety.clone() }).unwrap();
    m.add_edge(Edge { id: EdgeId(3), start: NodeId(2), end: NodeId(3), geometry: EdgeGeometry::Line { start: [2.0,0.0,0.0], end: [3.0,0.0,0.0], length: 2.0 }, speed: fast_speed.clone(), safety: safety.clone() }).unwrap();
    m
}

#[test]
fn astar_picks_faster_route() {
    let m = two_path_map();
    let path = astar(&m, NodeId(0), NodeId(3)).unwrap();
    assert_eq!(path.len(), 2);
    assert_eq!(path[0], EdgeId(2));
    assert_eq!(path[1], EdgeId(3));
}

#[test]
fn astar_no_path_returns_none() {
    let mut m = Map::new("disconnected");
    m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    m.add_node(Node { id: NodeId(1), position: [1.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    assert!(astar(&m, NodeId(0), NodeId(1)).is_none());
}
