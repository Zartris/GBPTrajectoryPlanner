use heapless::Vec;
use crate::{MAX_EDGES, MAX_NODES, ARC_TABLE_SAMPLES, MAX_CONTROL_POINTS, MAX_KNOTS};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NodeId(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EdgeId(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NodeType {
    Waypoint,
    Divert,
    Merge,
    Charger,
    Toploader,
    Discharge,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Node {
    pub id:        NodeId,
    pub position:  [f32; 3],
    pub node_type: NodeType,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SpeedProfile {
    pub max:         f32,
    pub nominal:     f32,
    pub accel_limit: f32,
    pub decel_limit: f32,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SafetyProfile {
    pub clearance: f32,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ArcTableEntry {
    pub t: f32,
    pub s: f32,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NurbsGeometry {
    pub control_points: Vec<[f32; 3], MAX_CONTROL_POINTS>,
    pub knots:          Vec<f32, MAX_KNOTS>,
    pub degree:         u8,
    pub length:         f32,
    /// Arc-length table: rebuilt from control_points/knots after deserialization.
    #[cfg_attr(feature = "serde", serde(skip, default = "default_arc_table"))]
    pub arc_t: [f32; ARC_TABLE_SAMPLES],
    #[cfg_attr(feature = "serde", serde(skip, default = "default_arc_table"))]
    pub arc_s: [f32; ARC_TABLE_SAMPLES],
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EdgeGeometry {
    Line { start: [f32; 3], end: [f32; 3], length: f32 },
    Nurbs(NurbsGeometry),
}

impl EdgeGeometry {
    pub fn length(&self) -> f32 {
        match self {
            Self::Line { length, .. } => *length,
            Self::Nurbs(n) => n.length,
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Edge {
    pub id:       EdgeId,
    pub start:    NodeId,
    pub end:      NodeId,
    pub geometry: EdgeGeometry,
    pub speed:    SpeedProfile,
    pub safety:   SafetyProfile,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Map {
    pub map_id:  heapless::String<32>,
    pub nodes:   Vec<Node, MAX_NODES>,
    pub edges:   Vec<Edge, MAX_EDGES>,
    /// Adjacency list: outgoing[NodeId.0] = list of edge indices.
    /// Skipped during serialization; rebuilt via `rebuild_outgoing()`.
    #[cfg_attr(feature = "serde", serde(skip, default = "default_outgoing"))]
    pub outgoing: [Vec<u16, 8>; MAX_NODES],
}

#[cfg(feature = "serde")]
fn default_arc_table() -> [f32; ARC_TABLE_SAMPLES] {
    [0.0f32; ARC_TABLE_SAMPLES]
}

#[cfg(feature = "serde")]
fn default_outgoing() -> [Vec<u16, 8>; MAX_NODES] {
    core::array::from_fn(|_| Vec::new())
}

impl Map {
    pub fn new(id: &str) -> Self {
        let mut map_id = heapless::String::new();
        for c in id.chars().take(32) { let _ = map_id.push(c); }
        Self {
            map_id,
            nodes: Vec::new(),
            edges: Vec::new(),
            outgoing: core::array::from_fn(|_| Vec::new()),
        }
    }

    /// Rebuild the outgoing adjacency list from edges. Call after deserialization.
    pub fn rebuild_outgoing(&mut self) {
        for slot in self.outgoing.iter_mut() {
            slot.clear();
        }
        for (idx, edge) in self.edges.iter().enumerate() {
            let start_slot = edge.start.0 as usize;
            if start_slot < MAX_NODES {
                self.outgoing[start_slot].push(idx as u16)
                    .expect("too many outgoing edges for node");
            }
        }
    }

    pub fn add_node(&mut self, node: Node) -> Result<NodeId, ()> {
        if (node.id.0 as usize) >= MAX_NODES { return Err(()); }
        if self.nodes.is_full() { return Err(()); }
        let id = node.id;
        self.nodes.push(node).map_err(|_| ())?;
        Ok(id)
    }

    pub fn add_edge(&mut self, edge: Edge) -> Result<EdgeId, ()> {
        if self.edges.is_full() { return Err(()); }
        let start_slot = edge.start.0 as usize;
        if start_slot >= MAX_NODES { return Err(()); }
        if self.node_index(edge.start).is_none() { return Err(()); }
        if self.node_index(edge.end).is_none() { return Err(()); }
        let edge_idx = self.edges.len() as u16;
        self.outgoing[start_slot].push(edge_idx).map_err(|_| ())?;
        let id = edge.id;
        self.edges.push(edge).map_err(|_| ())?;
        Ok(id)
    }

    pub fn node_index(&self, id: NodeId) -> Option<usize> {
        self.nodes.iter().position(|n| n.id == id)
    }

    pub fn edge_index(&self, id: EdgeId) -> Option<usize> {
        self.edges.iter().position(|e| e.id == id)
    }

    /// Rebuild outgoing adjacency AND NURBS arc tables after deserialization.
    pub fn rebuild_after_deserialize(&mut self) {
        self.rebuild_outgoing();
        for edge in self.edges.iter_mut() {
            if let EdgeGeometry::Nurbs(ref mut n) = edge.geometry {
                let (arc_t, arc_s) = crate::nurbs::build_arc_table(
                    &n.control_points, &n.knots, n.degree as usize
                );
                n.arc_t = arc_t;
                n.arc_s = arc_s;
            }
        }
    }

    /// Evaluate 3D position at arc-length s along edge.
    /// Returns `None` if `edge_id` is not found in the map.
    pub fn eval_position(&self, edge_id: EdgeId, s: f32) -> Option<[f32; 3]> {
        let idx = self.edge_index(edge_id)?;
        let edge = &self.edges[idx];
        Some(match &edge.geometry {
            EdgeGeometry::Line { start, end, length } => {
                let t = if *length > 1e-9 { (s / length).clamp(0.0, 1.0) } else { 0.0 };
                [
                    start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]),
                    start[2] + t * (end[2] - start[2]),
                ]
            }
            EdgeGeometry::Nurbs(n) => {
                let t = crate::nurbs::arc_s_to_t(s, n);
                crate::nurbs::eval_point(t, &n.control_points, &n.knots, n.degree as usize)
            }
        })
    }

    /// Evaluate unit tangent at arc-length s along edge.
    /// Returns `None` if `edge_id` is not found in the map.
    pub fn eval_tangent(&self, edge_id: EdgeId, s: f32) -> Option<[f32; 3]> {
        let idx = self.edge_index(edge_id)?;
        let edge = &self.edges[idx];
        Some(match &edge.geometry {
            EdgeGeometry::Line { start, end, length } => {
                let len = *length;
                if len < 1e-9 { return Some([1.0, 0.0, 0.0]); }
                [
                    (end[0] - start[0]) / len,
                    (end[1] - start[1]) / len,
                    (end[2] - start[2]) / len,
                ]
            }
            EdgeGeometry::Nurbs(n) => {
                let t = crate::nurbs::arc_s_to_t(s, n);
                crate::nurbs::eval_tangent(t, &n.control_points, &n.knots, n.degree as usize)
            }
        })
    }
}
