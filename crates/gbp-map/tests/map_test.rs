use gbp_map::map::*;
use gbp_map::{MAX_EDGES, MAX_NODES};

#[test]
#[cfg(feature = "serde")]
fn postcard_round_trip() {
    let mut map = Map::new("rt");
    let n0 = map.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    let n1 = map.add_node(Node { id: NodeId(1), position: [1.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    map.add_edge(Edge {
        id: EdgeId(0), start: n0, end: n1,
        geometry: EdgeGeometry::Line { start:[0.0,0.0,0.0], end:[1.0,0.0,0.0], length:1.0 },
        speed: SpeedProfile { max:2.5, nominal:2.0, accel_limit:1.0, decel_limit:1.0 },
        safety: SafetyProfile { clearance: 0.3 },
    }).unwrap();

    let mut buf = [0u8; 4096];
    let serialized = postcard::to_slice(&map, &mut buf).expect("serialization failed");
    let mut map2: Map = postcard::from_bytes(serialized).expect("deserialization failed");
    map2.rebuild_outgoing();

    assert_eq!(map2.nodes.len(), map.nodes.len());
    assert_eq!(map2.edges.len(), map.edges.len());
    assert_eq!(map2.nodes[0].id.0, map.nodes[0].id.0);
    assert_eq!(map2.edges[0].geometry.length(), map.edges[0].geometry.length());
}

#[test]
fn map_stores_nodes_and_edges() {
    let mut map = Map::new("test");
    let n0 = map.add_node(Node {
        id: NodeId(0),
        position: [0.0, 0.0, 0.0],
        node_type: NodeType::Waypoint,
    }).unwrap();
    let n1 = map.add_node(Node {
        id: NodeId(1),
        position: [1.0, 0.0, 0.0],
        node_type: NodeType::Waypoint,
    }).unwrap();
    let e = map.add_edge(Edge {
        id: EdgeId(0),
        start: n0,
        end: n1,
        geometry: EdgeGeometry::Line {
            start: [0.0, 0.0, 0.0],
            end: [1.0, 0.0, 0.0],
            length: 1.0,
        },
        speed: SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 },
        safety: SafetyProfile { clearance: 0.3 },
    }).unwrap();
    assert_eq!(map.nodes.len(), 2);
    assert_eq!(map.edges.len(), 1);
    assert_eq!(map.outgoing[n0.0 as usize].len(), 1);
    assert_eq!(map.eval_position(e, 0.0), Some([0.0, 0.0, 0.0]));
    assert_eq!(map.eval_position(e, 1.0), Some([1.0, 0.0, 0.0]));
}

#[test]
fn near_capacity_nodes() {
    let mut map = Map::new("stress_nodes");
    for i in 0..MAX_NODES as u16 {
        map.add_node(Node {
            id: NodeId(i),
            position: [i as f32, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        }).expect("should fit");
    }
    assert_eq!(map.nodes.len(), MAX_NODES);
    assert!(map.add_node(Node {
        id: NodeId(MAX_NODES as u16),
        position: [0.0, 0.0, 0.0],
        node_type: NodeType::Waypoint,
    }).is_err());
}

#[test]
fn near_capacity_edges() {
    // Build a map with MAX_EDGES edges using MAX_NODES nodes.
    // Edges wrap around: start=NodeId(i % MAX_NODES), end=NodeId((i+1) % MAX_NODES)
    let mut map = Map::new("stress_edges");
    for i in 0..MAX_NODES as u16 {
        map.add_node(Node {
            id: NodeId(i),
            position: [i as f32, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        }).expect("node should fit");
    }
    let safety = SafetyProfile { clearance: 0.3 };
    let speed = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
    for i in 0..MAX_EDGES as u16 {
        let start_n = i % MAX_NODES as u16;
        let end_n = (i + 1) % MAX_NODES as u16;
        map.add_edge(Edge {
            id: EdgeId(i),
            start: NodeId(start_n),
            end:   NodeId(end_n),
            geometry: EdgeGeometry::Line {
                start: [start_n as f32, 0.0, 0.0],
                end:   [end_n as f32, 0.0, 0.0],
                length: 1.0,
            },
            speed: speed.clone(),
            safety: safety.clone(),
        }).expect("edge should fit");
    }
    assert_eq!(map.edges.len(), MAX_EDGES);
    // One more edge should fail
    assert!(map.add_edge(Edge {
        id: EdgeId(MAX_EDGES as u16),
        start: NodeId(0),
        end:   NodeId(1),
        geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [1.0,0.0,0.0], length: 1.0 },
        speed: speed.clone(),
        safety: safety.clone(),
    }).is_err());
}
