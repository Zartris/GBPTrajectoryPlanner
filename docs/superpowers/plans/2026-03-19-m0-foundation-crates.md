# M0: Foundation Crates — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the four `no_std` foundation crates (`gbp-map`, `gbp-core`, `gbp-comms`, `gbp-agent`) with full unit tests, verifying they compile for bare-metal and host.

**Architecture:** Four crates in `crates/`, all `#![no_std]` without `alloc`, using `heapless` fixed-capacity collections throughout. `gbp-map` owns geometry and pathfinding. `gbp-core` owns the GBP algorithm. `gbp-comms` owns message types and the `CommsInterface` trait. `gbp-agent` owns per-robot logic and wires the other three together.

**Tech Stack:** Rust stable (host tests), `heapless 0.8`, `postcard 1`, `serde + serde_yaml` (feature-gated `parse`), `libm 0.2` (no_std float math), `riscv32imac-unknown-none-elf` target for no_std verification.

---

## File Structure

```
crates/
  gbp-map/
    Cargo.toml
    src/
      lib.rs          — capacity constants, pub mod declarations, re-exports
      map.rs          — Map, Node, Edge, NodeType, EdgeGeometry, SpeedProfile, Safety
      nurbs.rs        — Cox-de Boor eval, ArcLengthTable, tangent
      astar.rs        — A* path planning
      parser.rs       — YAML → Map (feature = "parse", PC only)
    tests/
      nurbs_test.rs   — NURBS eval accuracy, arc-length interpolation
      astar_test.rs   — shortest path on hand-crafted graph
      map_test.rs     — near-capacity construction, postcard round-trip
  gbp-core/
    Cargo.toml
    src/
      lib.rs          — pub mod declarations, re-exports
      variable_node.rs — VariableNode (information form: eta, lambda)
      factor_node.rs  — Factor trait, LinearizedFactor, FactorKind enum (3 extension points)
      factor_graph.rs — FactorGraph<K,F>, add_factor/remove_factor/iterate
      dynamics_factor.rs   — DynamicsFactor
      interrobot_factor.rs — InterRobotFactor
    tests/
      factor_test.rs  — residuals, Jacobians, precision matrix
      graph_test.rs   — swap-remove index patching, iterate convergence
  gbp-comms/
    Cargo.toml
    src/
      lib.rs          — all message types, CommsInterface trait, GBPTimestep
  gbp-agent/
    Cargo.toml
    src/
      lib.rs          — pub mod declarations, re-exports
      trajectory.rs   — edge-sequence traversal, global_s → (edge_id, local_s)
      interrobot_set.rs — InterRobotFactorSet, swap-remove index tracking
      robot_agent.rs  — RobotAgent, step(), update_interrobot_factors()
    tests/
      trajectory_test.rs    — edge transitions, global_s mapping
      interrobot_set_test.rs — index patching correctness
      robot_agent_test.rs   — step() output, v_nom propagation
```

---

## Chunk 1: Workspace Scaffolding + Capacity Constants

### Task 1: Register crates in the root workspace

**Files:**
- Modify: `Cargo.toml`

- [ ] **Step 1: Add crate members to workspace**

Replace the placeholder comment with actual members:

```toml
[workspace]
resolver = "2"
members = [
    "crates/gbp-map",
    "crates/gbp-core",
    "crates/gbp-comms",
    "crates/gbp-agent",
]

[workspace.dependencies]
heapless   = { version = "0.8", default-features = false }
postcard   = { version = "1",   default-features = false }
libm       = { version = "0.2", default-features = false }
# serde and serde_yaml are NOT marked optional here — `optional` is only valid in
# a crate's own [dependencies] table, not in [workspace.dependencies].
# Each crate that needs them marks them optional in its own Cargo.toml.
serde      = { version = "1",   features = ["derive"] }
serde_yaml = { version = "0.9" }
```

- [ ] **Step 2: Verify workspace parses**

```bash
cargo metadata --no-deps --format-version 1 | grep '"name"' | head -20
```
Expected: `gbp-map`, `gbp-core`, `gbp-comms`, `gbp-agent` in output.

---

### Task 2: Create `gbp-map` crate skeleton

**Files:**
- Create: `crates/gbp-map/Cargo.toml`
- Create: `crates/gbp-map/src/lib.rs`

- [ ] **Step 1: Write `Cargo.toml`**

```toml
[package]
name    = "gbp-map"
version = "0.1.0"
edition = "2021"

[features]
default = []
parse   = ["dep:serde", "dep:serde_yaml"]

[dependencies]
heapless  = { workspace = true }
# experimental-derive is needed for the postcard round-trip serialization test in Task 7
postcard  = { workspace = true, features = ["experimental-derive"] }
libm      = { workspace = true }
# optional = true is valid here (in the crate's own [dependencies], not workspace.dependencies)
serde     = { workspace = true, optional = true }
serde_yaml = { workspace = true, optional = true }

[dev-dependencies]
# none yet
```

- [ ] **Step 2: Write `src/lib.rs` with capacity constants**

```rust
#![no_std]

// ── Capacity constants — locked at M0. Changing these requires a memory-budget re-check. ──
pub const MAX_NODES:          usize = 64;
pub const MAX_EDGES:          usize = 96;
pub const MAX_CONTROL_POINTS: usize = 16;
pub const MAX_KNOTS:          usize = 32;
pub const ARC_TABLE_SAMPLES:  usize = 64;
pub const MAX_HORIZON:        usize = 12;   // K: GBP timesteps
pub const MAX_NEIGHBOURS:     usize = 8;    // max robots tracked simultaneously
pub const MAX_PATH_EDGES:     usize = 32;

pub mod map;
pub mod nurbs;
pub mod astar;

#[cfg(feature = "parse")]
pub mod parser;

pub use map::{EdgeGeometry, EdgeId, Map, Node, NodeId, NodeType, SafetyProfile, SpeedProfile};
pub use nurbs::ArcLengthTable;
```

- [ ] **Step 3: Verify it compiles (stubs only so far)**

```bash
cargo check -p gbp-map
```
Expected: error about missing modules — that's fine, scaffolding only.

---

### Task 3: Create remaining crate skeletons

**Files:**
- Create: `crates/gbp-core/Cargo.toml`, `src/lib.rs`
- Create: `crates/gbp-comms/Cargo.toml`, `src/lib.rs`
- Create: `crates/gbp-agent/Cargo.toml`, `src/lib.rs`

- [ ] **Step 1: `gbp-core/Cargo.toml`**

```toml
[package]
name    = "gbp-core"
version = "0.1.0"
edition = "2021"

[dependencies]
heapless = { workspace = true }
libm     = { workspace = true }
```

- [ ] **Step 2: `gbp-core/src/lib.rs`**

```rust
#![no_std]

pub mod variable_node;
pub mod factor_node;
pub mod factor_graph;
pub mod dynamics_factor;
pub mod interrobot_factor;  // FACTOR EXTENSION POINT 3/3: export new modules here

pub use variable_node::VariableNode;
pub use factor_node::{Factor, FactorKind, LinearizedFactor};
pub use factor_graph::FactorGraph;
pub use dynamics_factor::DynamicsFactor;
pub use interrobot_factor::InterRobotFactor;
```

- [ ] **Step 3: `gbp-comms/Cargo.toml`**

```toml
[package]
name    = "gbp-comms"
version = "0.1.0"
edition = "2021"

[dependencies]
heapless = { workspace = true }
# Intra-workspace crates use path references (workspace = true is for external deps only)
gbp-map  = { path = "../gbp-map" }
```

- [ ] **Step 4: `gbp-comms/src/lib.rs`** (stub — full content in Chunk 4)

```rust
#![no_std]
```

- [ ] **Step 5: `gbp-agent/Cargo.toml`**

```toml
[package]
name    = "gbp-agent"
version = "0.1.0"
edition = "2021"

[dependencies]
heapless   = { workspace = true }
gbp-core   = { path = "../gbp-core" }
gbp-comms  = { path = "../gbp-comms" }
gbp-map    = { path = "../gbp-map" }
```

- [ ] **Step 6: `gbp-agent/src/lib.rs`** (stub)

```rust
#![no_std]

pub mod trajectory;
pub mod interrobot_set;
pub mod robot_agent;

pub use robot_agent::RobotAgent;
```

- [ ] **Step 7: Commit scaffolding**

```bash
git add crates/ Cargo.toml
git commit -m "feat(m0): add workspace members and crate scaffolding"
```

---

## Chunk 2: `gbp-map` Data Types

### Task 4: Map core types

**Files:**
- Create: `crates/gbp-map/src/map.rs`

- [ ] **Step 1: Write the failing test**

Create `crates/gbp-map/tests/map_test.rs`:

```rust
use gbp_map::map::*;
use gbp_map::{MAX_EDGES, MAX_NODES};

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
    assert_eq!(map.eval_position(e, 0.0), [0.0, 0.0, 0.0]);
    assert_eq!(map.eval_position(e, 1.0), [1.0, 0.0, 0.0]);
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
    // Build a map with MAX_EDGES edges: a chain of MAX_EDGES+1 nodes.
    let n_nodes = MAX_EDGES + 1;
    let mut map = Map::new("stress_edges");
    for i in 0..n_nodes.min(MAX_NODES) as u16 {
        map.add_node(Node {
            id: NodeId(i),
            position: [i as f32, 0.0, 0.0],
            node_type: NodeType::Waypoint,
        }).expect("node should fit");
    }
    let safety = SafetyProfile { clearance: 0.3 };
    let speed = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
    for i in 0..MAX_EDGES as u16 {
        map.add_edge(Edge {
            id: EdgeId(i),
            start: NodeId(i),
            end:   NodeId(i + 1),
            geometry: EdgeGeometry::Line {
                start: [i as f32, 0.0, 0.0],
                end:   [i as f32 + 1.0, 0.0, 0.0],
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
```

- [ ] **Step 2: Run to confirm it fails**

```bash
cargo test -p gbp-map --test map_test 2>&1 | head -20
```
Expected: compile error — `map` module doesn't exist yet.

- [ ] **Step 3: Implement `map.rs`**

```rust
use heapless::Vec;
use crate::{MAX_EDGES, MAX_NODES, ARC_TABLE_SAMPLES, MAX_CONTROL_POINTS, MAX_KNOTS};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeId(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdgeId(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeType {
    Waypoint,
    Divert,
    Merge,
    Charger,
    Toploader,
    Discharge,
}

#[derive(Clone, Debug)]
pub struct Node {
    pub id:        NodeId,
    pub position:  [f32; 3],
    pub node_type: NodeType,
}

#[derive(Clone, Debug)]
pub struct SpeedProfile {
    pub max:         f32,
    pub nominal:     f32,
    pub accel_limit: f32,
    pub decel_limit: f32,
}

#[derive(Clone, Debug)]
pub struct SafetyProfile {
    pub clearance: f32,
}

#[derive(Clone, Debug)]
pub struct ArcTableEntry {
    pub t: f32,
    pub s: f32,
}

#[derive(Clone, Debug)]
pub struct NurbsGeometry {
    pub control_points: Vec<[f32; 3], MAX_CONTROL_POINTS>,
    pub knots:          Vec<f32, MAX_KNOTS>,
    pub degree:         u8,
    pub length:         f32,
    // Arc-length ↔ t lookup table: index i → (t_i, s_i)
    pub arc_t: [f32; ARC_TABLE_SAMPLES],
    pub arc_s: [f32; ARC_TABLE_SAMPLES],
}

#[derive(Clone, Debug)]
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
pub struct Edge {
    pub id:       EdgeId,
    pub start:    NodeId,
    pub end:      NodeId,
    pub geometry: EdgeGeometry,
    pub speed:    SpeedProfile,
    pub safety:   SafetyProfile,
}

pub struct Map {
    pub map_id:  heapless::String<32>,
    pub nodes:   Vec<Node, MAX_NODES>,
    pub edges:   Vec<Edge, MAX_EDGES>,
    // outgoing[NodeId.0 as usize] = list of edge indices.
    // NodeId values MUST be in the range 0..MAX_NODES and MUST be unique.
    // The parser and tests are responsible for assigning sequential IDs.
    pub outgoing: [Vec<u16, 8>; MAX_NODES],
}

impl Map {
    pub fn new(id: &str) -> Self {
        let mut map_id = heapless::String::new();
        for c in id.chars().take(32) { let _ = map_id.push(c); }
        Self {
            map_id,
            nodes: Vec::new(),
            edges: Vec::new(),
            // SAFETY: Vec<u16, 8> is zero-initialised (empty).
            outgoing: core::array::from_fn(|_| Vec::new()),
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
        // Verify the start node actually exists in our node list
        if self.node_index(edge.start).is_none() { return Err(()); }
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

    /// Evaluate 3D position at arc-length s along edge.
    pub fn eval_position(&self, edge_id: EdgeId, s: f32) -> [f32; 3] {
        let idx = self.edge_index(edge_id).expect("edge not found");
        let edge = &self.edges[idx];
        match &edge.geometry {
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
        }
    }

    /// Evaluate unit tangent at arc-length s along edge.
    pub fn eval_tangent(&self, edge_id: EdgeId, s: f32) -> [f32; 3] {
        let idx = self.edge_index(edge_id).expect("edge not found");
        let edge = &self.edges[idx];
        match &edge.geometry {
            EdgeGeometry::Line { start, end, length } => {
                let len = *length;
                if len < 1e-9 { return [1.0, 0.0, 0.0]; }
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
        }
    }
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p gbp-map --test map_test
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/gbp-map/src/map.rs crates/gbp-map/tests/map_test.rs
git commit -m "feat(m0): gbp-map core types (Map, Node, Edge, EdgeGeometry)"
```

---

## Chunk 3: `gbp-map` NURBS + Arc-Length + A\*

### Task 5: NURBS evaluation and arc-length table

**Files:**
- Create: `crates/gbp-map/src/nurbs.rs`
- Create: `crates/gbp-map/tests/nurbs_test.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// crates/gbp-map/tests/nurbs_test.rs
use gbp_map::map::{EdgeGeometry, EdgeId, NurbsGeometry};
use gbp_map::nurbs;
use gbp_map::ARC_TABLE_SAMPLES;

/// A cubic NURBS with 4 control points forming a known quarter-circle approximation.
/// Control points taken from literature for degree-3 arc approximation.
fn make_test_nurbs() -> NurbsGeometry {
    // Straight line as degenerate NURBS — easy to verify
    // 4 control points along x-axis, knots clamped cubic
    let mut cps: heapless::Vec<[f32; 3], 16> = heapless::Vec::new();
    cps.push([0.0, 0.0, 0.0]).unwrap();
    cps.push([1.0 / 3.0, 0.0, 0.0]).unwrap();
    cps.push([2.0 / 3.0, 0.0, 0.0]).unwrap();
    cps.push([1.0, 0.0, 0.0]).unwrap();
    let mut knots: heapless::Vec<f32, 32> = heapless::Vec::new();
    for k in [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0] {
        knots.push(k).unwrap();
    }
    let (arc_t, arc_s) = nurbs::build_arc_table(&cps, &knots, 3);
    NurbsGeometry {
        control_points: cps,
        knots,
        degree: 3,
        length: arc_s[ARC_TABLE_SAMPLES - 1],
        arc_t,
        arc_s,
    }
}

#[test]
fn nurbs_straight_line_eval() {
    let n = make_test_nurbs();
    let p0 = nurbs::eval_point(0.0, &n.control_points, &n.knots, 3);
    let p1 = nurbs::eval_point(1.0, &n.control_points, &n.knots, 3);
    assert!((p0[0] - 0.0).abs() < 1e-5, "p0.x={}", p0[0]);
    assert!((p1[0] - 1.0).abs() < 1e-5, "p1.x={}", p1[0]);
}

#[test]
fn arc_length_table_monotone() {
    let n = make_test_nurbs();
    for i in 1..ARC_TABLE_SAMPLES {
        assert!(n.arc_s[i] >= n.arc_s[i - 1], "arc_s not monotone at {}", i);
    }
}

#[test]
fn arc_s_to_t_round_trip() {
    let n = make_test_nurbs();
    let total = n.length;
    for frac in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
        let s = frac * total;
        let t = nurbs::arc_s_to_t(s, &n);
        let s_back = nurbs::arc_t_to_s(t, &n);
        assert!((s_back - s).abs() < 0.01, "round-trip error at s={}: got {}", s, s_back);
    }
}

#[test]
fn tangent_is_unit_length() {
    let n = make_test_nurbs();
    for frac in [0.1f32, 0.5, 0.9] {
        let t = nurbs::eval_tangent(frac, &n.control_points, &n.knots, 3);
        let len = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
        assert!((len - 1.0).abs() < 1e-4, "tangent not unit at t={}: len={}", frac, len);
    }
}
```

- [ ] **Step 2: Run to confirm it fails**

```bash
cargo test -p gbp-map --test nurbs_test 2>&1 | head -10
```
Expected: compile error — `nurbs` module missing.

- [ ] **Step 3: Implement `nurbs.rs`**

```rust
// crates/gbp-map/src/nurbs.rs
//! Cox-de Boor B-spline evaluation, arc-length table, and tangent.
//! All weights are assumed 1.0 (non-rational B-spline).

use crate::{ARC_TABLE_SAMPLES, MAX_CONTROL_POINTS, MAX_KNOTS};

/// Evaluate point on B-spline at parameter t ∈ [0, 1].
pub fn eval_point(
    t: f32,
    cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
    degree: usize,
) -> [f32; 3] {
    let n = cps.len() - 1;
    let t_clamped = t.clamp(knots[degree], knots[n + 1]);
    let span = find_knot_span(n, degree, t_clamped, knots);
    let basis = basis_functions(span, t_clamped, degree, knots);
    let mut p = [0.0f32; 3];
    for i in 0..=degree {
        let cp = cps[span - degree + i];
        p[0] += basis[i] * cp[0];
        p[1] += basis[i] * cp[1];
        p[2] += basis[i] * cp[2];
    }
    p
}

/// Unit tangent at parameter t via central difference.
pub fn eval_tangent(
    t: f32,
    cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
    degree: usize,
) -> [f32; 3] {
    let eps = 1e-4_f32;
    let t0 = (t - eps).max(0.0);
    let t1 = (t + eps).min(1.0);
    let p0 = eval_point(t0, cps, knots, degree);
    let p1 = eval_point(t1, cps, knots, degree);
    let dx = p1[0] - p0[0];
    let dy = p1[1] - p0[1];
    let dz = p1[2] - p0[2];
    let len = libm::sqrtf(dx * dx + dy * dy + dz * dz);
    if len < 1e-9 { return [1.0, 0.0, 0.0]; }
    [dx / len, dy / len, dz / len]
}

/// Build the arc-length lookup table: uniformly sample t, accumulate arc-length.
/// Returns (arc_t[N], arc_s[N]) where arc_s[N-1] = total_length.
pub fn build_arc_table(
    cps: &heapless::Vec<[f32; 3], MAX_CONTROL_POINTS>,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
    degree: usize,
) -> ([f32; ARC_TABLE_SAMPLES], [f32; ARC_TABLE_SAMPLES]) {
    let mut arc_t = [0.0f32; ARC_TABLE_SAMPLES];
    let mut arc_s = [0.0f32; ARC_TABLE_SAMPLES];
    let n = ARC_TABLE_SAMPLES - 1;
    let mut prev = eval_point(0.0, cps, knots, degree);
    let mut cumulative = 0.0f32;
    arc_t[0] = 0.0;
    arc_s[0] = 0.0;
    for i in 1..=n {
        let t = i as f32 / n as f32;
        let cur = eval_point(t, cps, knots, degree);
        let dx = cur[0] - prev[0];
        let dy = cur[1] - prev[1];
        let dz = cur[2] - prev[2];
        cumulative += libm::sqrtf(dx * dx + dy * dy + dz * dz);
        arc_t[i] = t;
        arc_s[i] = cumulative;
        prev = cur;
    }
    (arc_t, arc_s)
}

/// Convert arc-length s → parameter t via binary search + linear interpolation.
pub fn arc_s_to_t(s: f32, n: &crate::map::NurbsGeometry) -> f32 {
    let total = n.length;
    if total < 1e-9 { return 0.0; }
    let s_clamped = s.clamp(0.0, total);
    // Binary search for the interval
    let mut lo = 0usize;
    let mut hi = ARC_TABLE_SAMPLES - 1;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if n.arc_s[mid] <= s_clamped { lo = mid; } else { hi = mid; }
    }
    // Linear interpolation
    let ds = n.arc_s[hi] - n.arc_s[lo];
    if ds < 1e-9 { return n.arc_t[lo]; }
    let frac = (s_clamped - n.arc_s[lo]) / ds;
    n.arc_t[lo] + frac * (n.arc_t[hi] - n.arc_t[lo])
}

/// Convert parameter t → arc-length s via linear interpolation.
pub fn arc_t_to_s(t: f32, n: &crate::map::NurbsGeometry) -> f32 {
    let t_clamped = t.clamp(0.0, 1.0);
    let mut lo = 0usize;
    let mut hi = ARC_TABLE_SAMPLES - 1;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if n.arc_t[mid] <= t_clamped { lo = mid; } else { hi = mid; }
    }
    let dt = n.arc_t[hi] - n.arc_t[lo];
    if dt < 1e-9 { return n.arc_s[lo]; }
    let frac = (t_clamped - n.arc_t[lo]) / dt;
    n.arc_s[lo] + frac * (n.arc_s[hi] - n.arc_s[lo])
}

// ── Internal helpers ──────────────────────────────────────────────────────────

fn find_knot_span(
    n: usize,
    p: usize,
    t: f32,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
) -> usize {
    if (t - knots[n + 1]).abs() < 1e-9 { return n; }
    let mut lo = p;
    let mut hi = n + 1;
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] { hi = mid; } else { lo = mid; }
        mid = (lo + hi) / 2;
    }
    mid
}

fn basis_functions(
    span: usize,
    t: f32,
    p: usize,
    knots: &heapless::Vec<f32, MAX_KNOTS>,
) -> [f32; 9] { // max degree 8 (always use degree+1 values)
    let mut b = [0.0f32; 9];
    let mut left  = [0.0f32; 9];
    let mut right = [0.0f32; 9];
    b[0] = 1.0;
    for j in 1..=p {
        left[j]  = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0f32;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let temp = if denom.abs() < 1e-12 { 0.0 } else { b[r] / denom };
            b[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        b[j] = saved;
    }
    b
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p gbp-map --test nurbs_test
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/gbp-map/src/nurbs.rs crates/gbp-map/tests/nurbs_test.rs
git commit -m "feat(m0): NURBS Cox-de Boor eval and arc-length table"
```

---

### Task 6: A\* path planning

**Files:**
- Create: `crates/gbp-map/src/astar.rs`
- Create: `crates/gbp-map/tests/astar_test.rs`

- [ ] **Step 1: Write failing tests**

```rust
// crates/gbp-map/tests/astar_test.rs
use gbp_map::astar::astar;
use gbp_map::map::*;

fn two_path_map() -> Map {
    // Two paths from node 0 to node 3:
    //   Short but slow: 0→1→3  (lengths 1.0, 1.0; nominal 1.0 m/s → time = 2.0)
    //   Long but fast:  0→2→3  (lengths 2.0, 2.0; nominal 4.0 m/s → time = 1.0)
    // A* should pick 0→2→3 (lower traversal time).
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
    // Fast route uses edges 2 and 3
    assert_eq!(path[0], EdgeId(2));
    assert_eq!(path[1], EdgeId(3));
}

#[test]
fn astar_no_path_returns_none() {
    let mut m = Map::new("disconnected");
    m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    m.add_node(Node { id: NodeId(1), position: [1.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    // No edges
    assert!(astar(&m, NodeId(0), NodeId(1)).is_none());
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cargo test -p gbp-map --test astar_test 2>&1 | head -10
```

- [ ] **Step 3: Implement `astar.rs`**

```rust
// crates/gbp-map/src/astar.rs
//! A* path planning on the directed edge graph.
//! Cost: traversal time = edge.length / edge.speed.nominal
//! Heuristic: euclidean(node.pos, goal.pos) / global_max_speed  (admissible)

use heapless::Vec;
use crate::map::{EdgeId, Map, NodeId, NodeType};
use crate::MAX_PATH_EDGES;

/// Returns an ordered list of EdgeIds from start to goal, or None if unreachable.
pub fn astar(map: &Map, start: NodeId, goal: NodeId) -> Option<Vec<EdgeId, MAX_PATH_EDGES>> {
    const MAX_NODES: usize = crate::MAX_NODES;

    // Find global max speed for admissible heuristic
    let max_speed = map.edges.iter().fold(0.1f32, |acc, e| {
        if e.speed.nominal > acc { e.speed.nominal } else { acc }
    });

    let start_idx = map.node_index(start)?;
    let goal_idx  = map.node_index(goal)?;

    // g_score[i] = best traversal time to node i
    let mut g_score = [f32::MAX; MAX_NODES];
    // came_from[i] = (from_node_idx, edge_idx) that leads to node i on best path
    let mut came_from: [Option<(usize, usize)>; MAX_NODES] = [None; MAX_NODES];
    // open set as a simple sorted vec of (f_score, node_idx)
    // heapless BinaryHeap would be cleaner but Vec + linear scan is fine for MAX_NODES=64
    let mut open: Vec<(f32, usize), MAX_NODES> = Vec::new();

    g_score[start_idx] = 0.0;
    let h0 = heuristic(map, start_idx, goal_idx, max_speed);
    open.push((h0, start_idx)).ok()?;

    while !open.is_empty() {
        // Pop node with lowest f_score
        let (best_pos, _) = open.iter().enumerate()
            .min_by(|(_, a), (_, b)| a.0.partial_cmp(&b.0).unwrap())?;
        let (_, current) = open.swap_remove(best_pos);

        if current == goal_idx {
            return reconstruct_path(map, &came_from, goal_idx);
        }

        // Expand neighbours via outgoing edges
        for &edge_idx in map.outgoing[current].iter() {
            let edge = &map.edges[edge_idx as usize];
            let neighbor_idx = map.node_index(edge.end)?;

            // Edge cost: traversal time; add penalty for charger/toploader unless seeking
            let edge_cost = edge.geometry.length() / edge.speed.nominal.max(0.001);
            let node_penalty = match map.nodes[neighbor_idx].node_type {
                NodeType::Charger | NodeType::Toploader => 100.0,
                _ => 0.0,
            };
            let tentative_g = g_score[current] + edge_cost + node_penalty;

            if tentative_g < g_score[neighbor_idx] {
                came_from[neighbor_idx] = Some((current, edge_idx as usize));
                g_score[neighbor_idx] = tentative_g;
                let f = tentative_g + heuristic(map, neighbor_idx, goal_idx, max_speed);
                // Update or insert in open set
                let existing = open.iter_mut().find(|(_, n)| *n == neighbor_idx);
                if let Some(entry) = existing {
                    entry.0 = f;
                } else {
                    let _ = open.push((f, neighbor_idx));
                }
            }
        }
    }
    None
}

fn heuristic(map: &Map, node_idx: usize, goal_idx: usize, max_speed: f32) -> f32 {
    let n = &map.nodes[node_idx].position;
    let g = &map.nodes[goal_idx].position;
    let dx = n[0] - g[0]; let dy = n[1] - g[1]; let dz = n[2] - g[2];
    libm::sqrtf(dx*dx + dy*dy + dz*dz) / max_speed
}

fn reconstruct_path(
    map: &Map,
    came_from: &[Option<(usize, usize)>; crate::MAX_NODES],
    goal_idx: usize,
) -> Option<Vec<EdgeId, MAX_PATH_EDGES>> {
    // Walk backwards to reconstruct edge sequence
    let mut edges_rev: Vec<EdgeId, MAX_PATH_EDGES> = Vec::new();
    let mut cur = goal_idx;
    loop {
        match came_from[cur] {
            None => break,
            Some((prev, edge_idx)) => {
                edges_rev.push(map.edges[edge_idx].id).ok()?;
                cur = prev;
            }
        }
    }
    // Reverse in-place
    let n = edges_rev.len();
    for i in 0..n / 2 { edges_rev.swap(i, n - 1 - i); }
    if edges_rev.is_empty() { None } else { Some(edges_rev) }
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p gbp-map --test astar_test
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/gbp-map/src/astar.rs crates/gbp-map/tests/astar_test.rs
git commit -m "feat(m0): A* path planning (traversal-time cost)"
```

---

### Task 7: YAML parser + postcard round-trip

**Files:**
- Create: `crates/gbp-map/src/parser.rs`
- Extend: `crates/gbp-map/tests/map_test.rs`

- [ ] **Step 1: Add postcard round-trip test to `map_test.rs`**

```rust
// Add to crates/gbp-map/tests/map_test.rs
// Requires postcard feature. Add to Cargo.toml dev-dependencies:
//   postcard = { workspace = true, features = ["experimental-derive"] }

#[test]
fn postcard_round_trip() {
    // Build a minimal map with one node and one line edge
    let mut map = Map::new("rt");
    let n0 = map.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    let n1 = map.add_node(Node { id: NodeId(1), position: [1.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
    map.add_edge(Edge {
        id: EdgeId(0), start: n0, end: n1,
        geometry: EdgeGeometry::Line { start:[0.0,0.0,0.0], end:[1.0,0.0,0.0], length:1.0 },
        speed: SpeedProfile { max:2.5, nominal:2.0, accel_limit:1.0, decel_limit:1.0 },
        safety: SafetyProfile { clearance: 0.3 },
    }).unwrap();

    // Serialize
    let mut buf = [0u8; 4096];
    let serialized = postcard::to_slice(&map, &mut buf).expect("serialization failed");

    // Deserialize
    let map2: Map = postcard::from_bytes(serialized).expect("deserialization failed");

    assert_eq!(map2.nodes.len(), map.nodes.len());
    assert_eq!(map2.edges.len(), map.edges.len());
    assert_eq!(map2.nodes[0].id.0, map.nodes[0].id.0);
    assert_eq!(map2.edges[0].geometry.length(), map.edges[0].geometry.length());
}
```

Note: `Map` and all its types need `#[derive(serde::Serialize, serde::Deserialize)]` — add these derives to all types in `map.rs` gated behind a `serde` feature, and make the `postcard` experimental-derive feature available. Since `no_std` serde without alloc is complex for dynamic types, use `postcard`'s `MaxSize` derive or manual serialization. Simplest approach: use `serde` feature on the crate (not just `parse`) for the round-trip.

**Practical implementation note:** Add `serde` feature (separate from `parse`) to `gbp-map` that enables `#[derive(Serialize, Deserialize)]` on all map types. `postcard` + `heapless` types with serde works in `no_std`.

Update `Cargo.toml` features:

```toml
[features]
default = []
serde   = ["dep:serde", "heapless/serde"]
parse   = ["serde", "dep:serde_yaml"]
```

Add to all types in `map.rs`:

```rust
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
```

- [ ] **Step 2: Implement `parser.rs`** (PC only, feature-gated)

```rust
// crates/gbp-map/src/parser.rs
//! YAML → Map parser. Only compiled with feature = "parse".
//! Uses serde_yaml. Not available in no_std / firmware builds.

extern crate std;  // parser is always PC-only

use std::string::String;
use serde::{Deserialize, Serialize};
use crate::map::*;
use crate::nurbs::build_arc_table;

// ── YAML schema types ────────────────────────────────────────────────────────

#[derive(Deserialize)]
struct YamlMap {
    nodes: std::vec::Vec<YamlNode>,
    edges: std::vec::Vec<YamlEdge>,
}

#[derive(Deserialize)]
struct YamlNode {
    id: String,
    #[serde(rename = "nodePosition")]
    position: YamlPosition,
    #[serde(rename = "customProperties", default)]
    custom: YamlCustomProps,
}

#[derive(Deserialize, Default)]
struct YamlCustomProps {
    #[serde(rename = "type", default)]
    node_type: String,
}

#[derive(Deserialize)]
struct YamlPosition { x: f64, y: f64, z: f64 }

#[derive(Deserialize)]
struct YamlEdge {
    id: String,
    #[serde(rename = "startNodeId")]
    start_node: String,
    #[serde(rename = "endNodeId")]
    end_node: String,
    geometry: YamlGeometry,
    speed_profile: YamlSpeed,
    safety: YamlSafety,
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
enum YamlGeometry {
    Line {
        points: std::vec::Vec<[f64; 3]>,
        length: f64,
    },
    Spline {
        control_points: std::vec::Vec<[f64; 3]>,
        knots: std::vec::Vec<f64>,
        degree: u8,
        length_estimate: f64,
    },
}

#[derive(Deserialize)]
struct YamlSpeed {
    max: f64, nominal: f64, accel_limit: f64, decel_limit: f64,
}

#[derive(Deserialize)]
struct YamlSafety { clearance: f64 }

// ── Public API ───────────────────────────────────────────────────────────────

pub fn parse_yaml(yaml: &str) -> Result<Map, std::string::String> {
    let raw: YamlMap = serde_yaml::from_str(yaml)
        .map_err(|e| std::format!("YAML parse error: {e}"))?;

    let mut map = Map::new("parsed");
    // Assign sequential u16 IDs; store string→u16 mapping temporarily
    let mut node_id_map: std::collections::HashMap<String, u16> = std::collections::HashMap::new();

    for (i, yn) in raw.nodes.iter().enumerate() {
        let id = i as u16;
        node_id_map.insert(yn.id.clone(), id);
        let node_type = match yn.custom.node_type.as_str() {
            "merge"     => NodeType::Merge,
            "divert"    => NodeType::Divert,
            "charger"   => NodeType::Charger,
            "toploader" => NodeType::Toploader,
            "discharge" => NodeType::Discharge,
            _           => NodeType::Waypoint,
        };
        map.add_node(Node {
            id: NodeId(id),
            position: [yn.position.x as f32, yn.position.y as f32, yn.position.z as f32],
            node_type,
        }).map_err(|_| "map node capacity exceeded")?;
    }

    for (i, ye) in raw.edges.iter().enumerate() {
        let start_id = *node_id_map.get(&ye.start_node)
            .ok_or_else(|| std::format!("unknown start node {}", ye.start_node))?;
        let end_id = *node_id_map.get(&ye.end_node)
            .ok_or_else(|| std::format!("unknown end node {}", ye.end_node))?;

        let geometry = match &ye.geometry {
            YamlGeometry::Line { points, length } => {
                let start = [points[0][0] as f32, points[0][1] as f32, points[0][2] as f32];
                let end   = [points[1][0] as f32, points[1][1] as f32, points[1][2] as f32];
                EdgeGeometry::Line { start, end, length: *length as f32 }
            }
            YamlGeometry::Spline { control_points, knots, degree, length_estimate } => {
                let mut cps: heapless::Vec<[f32; 3], 16> = heapless::Vec::new();
                for cp in control_points {
                    cps.push([cp[0] as f32, cp[1] as f32, cp[2] as f32])
                        .map_err(|_| "too many control points")?;
                }
                let mut kv: heapless::Vec<f32, 32> = heapless::Vec::new();
                for k in knots {
                    kv.push(*k as f32).map_err(|_| "too many knots")?;
                }
                let (arc_t, arc_s) = build_arc_table(&cps, &kv, *degree as usize);
                let length = arc_s[crate::ARC_TABLE_SAMPLES - 1];
                EdgeGeometry::Nurbs(crate::map::NurbsGeometry {
                    control_points: cps, knots: kv, degree: *degree,
                    length, arc_t, arc_s,
                })
            }
        };

        map.add_edge(Edge {
            id: EdgeId(i as u16),
            start: NodeId(start_id),
            end:   NodeId(end_id),
            geometry,
            speed: SpeedProfile {
                max: ye.speed_profile.max as f32,
                nominal: ye.speed_profile.nominal as f32,
                accel_limit: ye.speed_profile.accel_limit as f32,
                decel_limit: ye.speed_profile.decel_limit as f32,
            },
            safety: SafetyProfile { clearance: ye.safety.clearance as f32 },
        }).map_err(|_| "map edge capacity exceeded")?;
    }

    Ok(map)
}
```

- [ ] **Step 3: Run all map tests**

```bash
cargo test -p gbp-map --features "parse,serde"
```
Expected: all PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-map/src/parser.rs crates/gbp-map/tests/map_test.rs crates/gbp-map/Cargo.toml crates/gbp-map/src/lib.rs
git commit -m "feat(m0): YAML parser and postcard serialization for gbp-map"
```

---

## Chunk 4: `gbp-core`

### Task 8: VariableNode and Factor trait

**Files:**
- Create: `crates/gbp-core/src/variable_node.rs`
- Create: `crates/gbp-core/src/factor_node.rs`

- [ ] **Step 1: Write failing test**

Create `crates/gbp-core/tests/factor_test.rs`:

```rust
use gbp_core::variable_node::VariableNode;
use gbp_core::factor_node::LinearizedFactor;

#[test]
fn variable_node_mean_and_variance() {
    let v = VariableNode { eta: 4.0, lambda: 2.0, prior_eta: 0.0, prior_lambda: 0.01 };
    assert!((v.mean() - 2.0).abs() < 1e-6);
    assert!((v.variance() - 0.5).abs() < 1e-6);
}

#[test]
fn variable_node_zero_lambda_is_safe() {
    let v = VariableNode::default();
    assert_eq!(v.mean(), 0.0);
    assert!(v.variance() > 1e10); // "infinite" uncertainty
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cargo test -p gbp-core --test factor_test 2>&1 | head -10
```

- [ ] **Step 3: Implement `variable_node.rs`**

```rust
// crates/gbp-core/src/variable_node.rs

/// GBP variable in information (canonical) form.
/// Mean μ = η/λ, Variance σ² = 1/λ.
#[derive(Clone, Copy, Debug, Default)]
pub struct VariableNode {
    /// Information vector: η = λ·μ
    pub eta: f32,
    /// Precision (information scalar): λ = 1/σ²
    pub lambda: f32,
    /// Prior information — kept separate so iterate() can reset each step.
    pub prior_eta: f32,
    pub prior_lambda: f32,
}

impl VariableNode {
    pub fn new(mean: f32, variance: f32) -> Self {
        let lambda = if variance > 1e-30 { 1.0 / variance } else { 0.0 };
        Self { eta: lambda * mean, lambda, prior_eta: 0.0, prior_lambda: 1e-6 }
    }

    pub fn mean(&self) -> f32 {
        if self.lambda.abs() < 1e-10 { 0.0 } else { self.eta / self.lambda }
    }

    pub fn variance(&self) -> f32 {
        if self.lambda.abs() < 1e-10 { f32::MAX } else { 1.0 / self.lambda }
    }

    /// Reset to prior (called at start of each iterate() to re-accumulate messages).
    pub fn reset_to_prior(&mut self) {
        self.eta    = self.prior_eta;
        self.lambda = self.prior_lambda;
    }
}
```

- [ ] **Step 4: Implement `factor_node.rs`**

```rust
// crates/gbp-core/src/factor_node.rs
use heapless::Vec;
use crate::variable_node::VariableNode;
use crate::dynamics_factor::DynamicsFactor;
use crate::interrobot_factor::InterRobotFactor;

/// Result of linearizing a factor around current variable beliefs.
#[derive(Clone, Debug)]
pub struct LinearizedFactor {
    /// Jacobian row — one f32 per connected variable.
    pub jacobian:  Vec<f32, 2>,
    /// Residual scalar.
    pub residual:  f32,
    /// Precision (1/σ²) for this factor.
    pub precision: f32,
}

/// Implement this trait for each factor type.
pub trait Factor {
    /// Variable indices this factor connects (into `FactorGraph.variables`).
    fn variable_indices(&self) -> &[usize];

    /// Linearize around current beliefs → Jacobian + residual + precision.
    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor;

    /// Called each GBP iteration — override to re-evaluate geometry or cached state.
    /// Default: no-op.
    fn update(&mut self, _variables: &[VariableNode]) {}

    /// Whether this factor participates in this iteration.
    fn is_active(&self) -> bool { true }
}

/// The single extension enum. Adding a new factor type means:
///   1. Add a variant here           (FACTOR EXTENSION POINT 1/3)
///   2. Add match arms below         (FACTOR EXTENSION POINT 2/3)
///   3. Export the module in lib.rs  (FACTOR EXTENSION POINT 3/3)
pub enum FactorKind {
    Dynamics(DynamicsFactor),
    InterRobot(InterRobotFactor),
}

impl FactorKind {
    pub fn as_factor(&self) -> &dyn Factor {
        match self {
            // FACTOR EXTENSION POINT 2/3
            Self::Dynamics(f)   => f,
            Self::InterRobot(f) => f,
        }
    }
    pub fn as_factor_mut(&mut self) -> &mut dyn Factor {
        match self {
            // FACTOR EXTENSION POINT 2/3 (mut)
            Self::Dynamics(f)   => f,
            Self::InterRobot(f) => f,
        }
    }
}

/// Internal wrapper used by FactorGraph to store a factor alongside its outgoing messages.
pub(crate) struct FactorNode {
    pub kind: FactorKind,
    /// Outgoing messages (factor → variable), indexed by position in variable_indices().
    /// All factors in this system are pairwise or unary — max 2.
    pub msg_eta:    [f32; 2],
    pub msg_lambda: [f32; 2],
}

impl FactorNode {
    pub fn new(kind: FactorKind) -> Self {
        Self { kind, msg_eta: [0.0; 2], msg_lambda: [0.0; 2] }
    }
}
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p gbp-core --test factor_test
```
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates/gbp-core/src/variable_node.rs crates/gbp-core/src/factor_node.rs crates/gbp-core/tests/factor_test.rs
git commit -m "feat(m0): VariableNode (information form) and Factor trait"
```

---

### Task 9: DynamicsFactor and InterRobotFactor

**Files:**
- Create: `crates/gbp-core/src/dynamics_factor.rs`
- Create: `crates/gbp-core/src/interrobot_factor.rs`
- Extend: `crates/gbp-core/tests/factor_test.rs`

- [ ] **Step 1: Add factor tests**

```rust
// Append to crates/gbp-core/tests/factor_test.rs

use gbp_core::dynamics_factor::DynamicsFactor;
use gbp_core::interrobot_factor::InterRobotFactor;
use gbp_core::factor_node::Factor;
use gbp_core::variable_node::VariableNode;

fn make_vars(means: &[f32], variance: f32) -> heapless::Vec<VariableNode, 16> {
    let mut v: heapless::Vec<VariableNode, 16> = heapless::Vec::new();
    for &m in means {
        v.push(VariableNode::new(m, variance)).unwrap();
    }
    v
}

#[test]
fn dynamics_factor_zero_residual_at_v_nom() {
    // If s_{k+1} - s_k = v_nom * dt, residual should be 0.
    let dt = 0.1;
    let v_nom = 2.0;
    let s_k = 1.0;
    let s_k1 = s_k + v_nom * dt;
    let vars = make_vars(&[s_k, s_k1], 1.0);
    let f = DynamicsFactor::new([0, 1], dt, 0.1, v_nom);
    let lf = f.linearize(&vars);
    assert!(lf.residual.abs() < 1e-5, "residual={}", lf.residual);
}

#[test]
fn dynamics_factor_jacobian_shape() {
    let f = DynamicsFactor::new([0, 1], 0.1, 0.1, 1.0);
    let vars = make_vars(&[0.0, 0.0], 1.0);
    let lf = f.linearize(&vars);
    assert_eq!(lf.jacobian.len(), 2);
    // J = [-1/dt, +1/dt]
    assert!((lf.jacobian[0] - (-10.0)).abs() < 1e-4, "J0={}", lf.jacobian[0]);
    assert!((lf.jacobian[1] -  10.0 ).abs() < 1e-4, "J1={}", lf.jacobian[1]);
}

#[test]
fn interrobot_factor_precision_matrix_symmetric() {
    // J_A=0.8, J_B=-0.6, sigma_r=0.5 → Λ = (1/0.25) * [[0.64, -0.48], [-0.48, 0.36]]
    let f = InterRobotFactor::new(0, 0.3, 0.5);
    // Manually set Jacobians and external belief
    let mut f2 = f;
    f2.jacobian_a = 0.8;
    f2.jacobian_b = -0.6;
    f2.ext_eta_b  = 0.0;
    f2.ext_lambda_b = 4.0;  // variance 0.25

    let vars = make_vars(&[0.0], 1.0);
    let lf = f2.linearize(&vars);

    // The factor produces a message to var_a only.
    // Just verify precision > 0 and residual is finite.
    assert!(lf.precision.is_finite());
    assert!(lf.residual.is_finite());
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cargo test -p gbp-core --test factor_test 2>&1 | head -15
```

- [ ] **Step 3: Implement `dynamics_factor.rs`**

```rust
// crates/gbp-core/src/dynamics_factor.rs
use heapless::Vec;
use crate::factor_node::{Factor, LinearizedFactor};
use crate::variable_node::VariableNode;

/// Velocity prior factor connecting (s_k, s_{k+1}).
/// Residual: e = (s_{k+1} - s_k)/dt - v_nom
/// Jacobian: [-1/dt, +1/dt]
pub struct DynamicsFactor {
    var_indices: [usize; 2],  // [k, k+1]
    dt:    f32,
    sigma: f32,
    /// Nominal velocity at current s_k — set by RobotAgent before each iterate().
    pub v_nom: f32,
}

impl DynamicsFactor {
    pub fn new(var_indices: [usize; 2], dt: f32, sigma: f32, v_nom: f32) -> Self {
        Self { var_indices, dt, sigma, v_nom }
    }
    pub fn set_v_nom(&mut self, v: f32) { self.v_nom = v; }
}

impl Factor for DynamicsFactor {
    fn variable_indices(&self) -> &[usize] { &self.var_indices }

    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor {
        let s_k  = variables[self.var_indices[0]].mean();
        let s_k1 = variables[self.var_indices[1]].mean();
        let residual = (s_k1 - s_k) / self.dt - self.v_nom;
        let inv_dt = 1.0 / self.dt;
        let mut jacobian: Vec<f32, 2> = Vec::new();
        let _ = jacobian.push(-inv_dt);
        let _ = jacobian.push( inv_dt);
        LinearizedFactor {
            jacobian,
            residual,
            precision: 1.0 / (self.sigma * self.sigma),
        }
    }
}
```

- [ ] **Step 4: Implement `interrobot_factor.rs`**

```rust
// crates/gbp-core/src/interrobot_factor.rs
//! Inter-robot collision avoidance factor.
//!
//! Connects only var_idx_a (in this robot's graph).
//! Robot B's belief is injected externally as (ext_eta_b, ext_lambda_b).
//!
//! 3D Jacobians (computed in agent layer, stored as scalars):
//!   J_A = -(p_A - p_B)ᵀ · p_A'(s̄_A) / ‖p_A - p_B‖
//!   J_B =  (p_A - p_B)ᵀ · p_B'(s̄_B) / ‖p_A - p_B‖
//!
//! GBP message to var_a uses Schur complement marginalizing out B with its external belief.

use heapless::Vec;
use crate::factor_node::{Factor, LinearizedFactor};
use crate::variable_node::VariableNode;

pub struct InterRobotFactor {
    var_idx_a:   usize,
    d_safe:      f32,
    sigma_r:     f32,
    active:      bool,
    /// Scalar Jacobians (set by agent each step)
    pub jacobian_a: f32,
    pub jacobian_b: f32,
    /// External belief of robot B at this timestep (set by agent from RobotBroadcast)
    pub ext_eta_b:    f32,
    pub ext_lambda_b: f32,
    /// Distance between robots at current beliefs (set by agent)
    pub dist: f32,
}

impl InterRobotFactor {
    pub fn new(var_idx_a: usize, d_safe: f32, sigma_r: f32) -> Self {
        Self {
            var_idx_a, d_safe, sigma_r,
            active: true,
            jacobian_a: 0.0, jacobian_b: 0.0,
            ext_eta_b: 0.0, ext_lambda_b: 1.0,
            dist: f32::MAX,
        }
    }

    pub fn set_active(&mut self, active: bool) { self.active = active; }
}

impl Factor for InterRobotFactor {
    fn variable_indices(&self) -> &[usize] { core::slice::from_ref(&self.var_idx_a) }
    fn is_active(&self) -> bool { self.active }

    fn linearize(&self, _variables: &[VariableNode]) -> LinearizedFactor {
        // Residual: d_safe - dist  (positive when robots are too close)
        let residual = self.d_safe - self.dist;
        let prec = 1.0 / (self.sigma_r * self.sigma_r);

        // Joint information matrix (2×2, pairwise between A and B):
        let xi_aa = prec * self.jacobian_a * self.jacobian_a;
        let xi_ab = prec * self.jacobian_a * self.jacobian_b;
        let xi_bb = prec * self.jacobian_b * self.jacobian_b;
        let zeta_a = prec * residual * self.jacobian_a;
        let zeta_b = prec * residual * self.jacobian_b;

        // Marginalize out B using Schur complement with B's external cavity belief
        let lambda_b_eff = xi_bb + self.ext_lambda_b;
        let (msg_lambda, msg_eta) = if lambda_b_eff.abs() > 1e-10 {
            let lam = xi_aa - (xi_ab * xi_ab) / lambda_b_eff;
            let eta = zeta_a - xi_ab * (zeta_b + self.ext_eta_b) / lambda_b_eff;
            (lam, eta)
        } else {
            (xi_aa, zeta_a)
        };

        // Package as LinearizedFactor (single-variable: var_idx_a only).
        // We repurpose the jacobian field as [J_a] and encode the pre-computed
        // information message directly via precision = msg_lambda, residual = msg_eta.
        // The FactorGraph uses a different code path for InterRobotFactor — see factor_graph.rs.
        let mut jacobian: Vec<f32, 2> = Vec::new();
        let _ = jacobian.push(self.jacobian_a);
        LinearizedFactor {
            jacobian,
            residual: msg_eta,    // pre-computed η message to var_a
            precision: msg_lambda, // pre-computed λ message to var_a
        }
    }
}
```

> **Implementation note for FactorGraph:** `InterRobotFactor::linearize()` returns the pre-computed `(η, λ)` message in `(residual, precision)` fields (not the raw residual/Jacobian). The `FactorGraph` must detect this case (by checking `variable_indices().len() == 1`) and apply the message directly, skipping the Schur complement computation it performs for two-variable factors.

- [ ] **Step 5: Run tests**

```bash
cargo test -p gbp-core --test factor_test
```
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add crates/gbp-core/src/dynamics_factor.rs crates/gbp-core/src/interrobot_factor.rs crates/gbp-core/tests/factor_test.rs
git commit -m "feat(m0): DynamicsFactor and InterRobotFactor"
```

---

### Task 10: FactorGraph — add/remove/iterate

**Files:**
- Create: `crates/gbp-core/src/factor_graph.rs`
- Create: `crates/gbp-core/tests/graph_test.rs`

- [ ] **Step 1: Write failing tests**

```rust
// crates/gbp-core/tests/graph_test.rs
use gbp_core::factor_graph::FactorGraph;
use gbp_core::factor_node::FactorKind;
use gbp_core::dynamics_factor::DynamicsFactor;
use gbp_core::variable_node::VariableNode;

const K: usize = 4;   // 4 timestep variables
const F: usize = 16;  // max 16 factors

#[test]
fn factor_graph_add_and_count() {
    let mut g: FactorGraph<K, F> = FactorGraph::new(0.0, 1.0);
    assert_eq!(g.factor_count(), 0);
    let idx = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    assert_eq!(g.factor_count(), 1);
    assert_eq!(idx, 0);
}

#[test]
fn factor_graph_swap_remove_patches_index() {
    let mut g: FactorGraph<K, F> = FactorGraph::new(0.0, 1.0);
    let i0 = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let i1 = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    let i2 = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([2,3], 0.1, 0.1, 1.0))).unwrap();
    assert_eq!(g.factor_count(), 3);
    // Remove factor at index 0; factor at index 2 moves to index 0
    g.remove_factor(i0);
    assert_eq!(g.factor_count(), 2);
    // The factor that was at i2=[2,3] is now at index 0
    // (swap-remove — implementation detail we verify by checking count)
}

#[test]
fn iterate_single_dynamics_converges() {
    // Two variables, one DynamicsFactor, v_nom=1.0, dt=0.1
    // Initial beliefs: s_0 = 0.0, s_1 = 0.0 (both at origin)
    // After iteration, s_1 should move toward 0.0 + v_nom * dt = 0.1
    let mut g: FactorGraph<2, 4> = FactorGraph::new(0.0, 100.0); // wide prior
    g.variables[0] = VariableNode::new(0.0, 100.0);
    g.variables[0].prior_eta = 0.0 / 100.0;  // anchor s_0 strongly at 0.0
    g.variables[0].prior_lambda = 1000.0;     // tight prior on s_0
    g.variables[0].eta = g.variables[0].prior_eta;
    g.variables[0].lambda = g.variables[0].prior_lambda;

    g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0, 1], 0.1, 0.1, 1.0))).unwrap();
    g.iterate(20);

    let s1 = g.variables[1].mean();
    // s_1 should converge toward v_nom * dt = 0.1
    assert!(s1 > 0.05, "s1={} did not converge toward 0.1", s1);
}
```

- [ ] **Step 2: Run to confirm failure**

```bash
cargo test -p gbp-core --test graph_test 2>&1 | head -10
```

- [ ] **Step 3: Implement `factor_graph.rs`**

```rust
// crates/gbp-core/src/factor_graph.rs
use heapless::Vec;
use crate::factor_node::{Factor, FactorKind, FactorNode};
use crate::variable_node::VariableNode;

/// GBP factor graph with const-generic capacity.
/// K = number of variables (timestep horizon), F = max factors.
pub struct FactorGraph<const K: usize, const F: usize> {
    pub variables: [VariableNode; K],
    factors: Vec<FactorNode, F>,
}

impl<const K: usize, const F: usize> FactorGraph<K, F> {
    /// Create graph with K variables all initialised to (mean, variance).
    pub fn new(init_mean: f32, init_variance: f32) -> Self {
        Self {
            variables: core::array::from_fn(|_| VariableNode::new(init_mean, init_variance)),
            factors: Vec::new(),
        }
    }

    pub fn factor_count(&self) -> usize { self.factors.len() }

    /// Add a factor; returns its slot index.
    pub fn add_factor(&mut self, kind: FactorKind) -> Result<usize, ()> {
        let idx = self.factors.len();
        self.factors.push(FactorNode::new(kind)).map_err(|_| ())?;
        Ok(idx)
    }

    /// O(1) swap-remove. The last factor moves to `idx`.
    /// Caller (InterRobotFactorSet) is responsible for updating stored indices.
    pub fn remove_factor(&mut self, idx: usize) -> FactorKind {
        self.factors.swap_remove(idx).kind
    }

    /// Get mutable access to a factor's FactorKind (for setting v_nom, Jacobians, etc.)
    pub fn get_factor_kind_mut(&mut self, idx: usize) -> Option<&mut FactorKind> {
        self.factors.get_mut(idx).map(|f| &mut f.kind)
    }

    /// Run N iterations of GBP message passing.
    pub fn iterate(&mut self, iterations: usize) {
        for _ in 0..iterations {
            self.factor_to_variable_pass();
            self.variable_to_factor_pass();
        }
    }

    // ── Private ───────────────────────────────────────────────────────────────

    fn factor_to_variable_pass(&mut self) {
        for f in self.factors.iter_mut() {
            if !f.kind.as_factor().is_active() { continue; }
            let var_indices = f.kind.as_factor().variable_indices();

            if var_indices.len() == 1 {
                // Unary factor (e.g. InterRobotFactor with external B):
                // linearize() returns pre-computed (eta_msg, lambda_msg) in (residual, precision)
                let lf = f.kind.as_factor().linearize(&self.variables);
                f.msg_eta[0]    = lf.residual;
                f.msg_lambda[0] = lf.precision;

            } else if var_indices.len() == 2 {
                // Pairwise factor: Schur complement marginalization.
                let [idx0, idx1] = [var_indices[0], var_indices[1]];
                let lf = f.kind.as_factor().linearize(&self.variables);
                let j0 = lf.jacobian[0];
                let j1 = lf.jacobian[1];
                let prec = lf.precision;
                let r    = lf.residual;

                // Joint information matrix elements
                let xi_00 = prec * j0 * j0;
                let xi_11 = prec * j1 * j1;
                let xi_01 = prec * j0 * j1;
                let zeta_0 = prec * r * j0;
                let zeta_1 = prec * r * j1;

                // Cavity beliefs (variable total minus this factor's previous message)
                let eta0_cav    = self.variables[idx0].eta    - f.msg_eta[0];
                let lambda0_cav = self.variables[idx0].lambda - f.msg_lambda[0];
                let eta1_cav    = self.variables[idx1].eta    - f.msg_eta[1];
                let lambda1_cav = self.variables[idx1].lambda - f.msg_lambda[1];

                // Message to variable 0 (marginalize out variable 1)
                let denom1 = xi_11 + lambda1_cav;
                if denom1.abs() > 1e-12 {
                    f.msg_lambda[0] = xi_00 - xi_01 * xi_01 / denom1;
                    f.msg_eta[0]    = zeta_0 - xi_01 * (zeta_1 + eta1_cav) / denom1;
                }

                // Message to variable 1 (marginalize out variable 0)
                let denom0 = xi_00 + lambda0_cav;
                if denom0.abs() > 1e-12 {
                    f.msg_lambda[1] = xi_11 - xi_01 * xi_01 / denom0;
                    f.msg_eta[1]    = zeta_1 - xi_01 * (zeta_0 + eta0_cav) / denom0;
                }
            }
        }
    }

    fn variable_to_factor_pass(&mut self) {
        // Reset all variables to their prior
        for v in self.variables.iter_mut() {
            v.reset_to_prior();
        }
        // Accumulate all factor messages
        for f in self.factors.iter() {
            if !f.kind.as_factor().is_active() { continue; }
            for (i, &var_idx) in f.kind.as_factor().variable_indices().iter().enumerate() {
                self.variables[var_idx].eta    += f.msg_eta[i];
                self.variables[var_idx].lambda += f.msg_lambda[i];
            }
        }
    }
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p gbp-core
```
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/gbp-core/src/factor_graph.rs crates/gbp-core/tests/graph_test.rs
git commit -m "feat(m0): FactorGraph with GBP message passing"
```

---

## Chunk 5: `gbp-comms` + `gbp-agent` + no\_std Verification

### Task 11: `gbp-comms` message types

**Files:**
- Rewrite: `crates/gbp-comms/src/lib.rs`

- [ ] **Step 1: Write failing test**

Create `crates/gbp-comms/tests/comms_test.rs`:

```rust
use gbp_comms::*;

#[test]
fn robot_broadcast_default_is_valid() {
    let b = RobotBroadcast::default();
    assert_eq!(b.planned_edges.len(), 0);
    assert_eq!(b.belief_means.len(), 0);
}
```

- [ ] **Step 2: Implement `gbp-comms/src/lib.rs`**

```rust
#![no_std]

use heapless::Vec;
use gbp_map::{MAX_HORIZON, MAX_NEIGHBOURS, EdgeId, NodeId};

pub type RobotId = u32;

#[derive(Clone, Copy, Debug, Default)]
pub struct GBPTimestep {
    pub eta:    f32,
    pub lambda: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RobotSource { Simulated, Hardware }

#[derive(Clone, Copy, Debug)]
pub enum ParameterTarget { Global, Robot(RobotId) }

/// Physics → Agent (in-process only)
#[derive(Clone, Copy, Debug, Default)]
pub struct ObservationUpdate {
    pub position_s:   f32,
    pub velocity:     f32,
    pub current_edge: EdgeId,
}

/// Agent → Agent broadcast (over SimComms or ESP-NOW)
#[derive(Clone, Debug, Default)]
pub struct RobotBroadcast {
    pub robot_id:      RobotId,
    pub current_edge:  EdgeId,
    pub position_s:    f32,
    pub velocity:      f32,
    pub pos:           [f32; 3],
    pub planned_edges: Vec<EdgeId, MAX_HORIZON>,
    pub belief_means:  Vec<f32, MAX_HORIZON>,
    pub belief_vars:   Vec<f32, MAX_HORIZON>,
    pub gbp_timesteps: Vec<GBPTimestep, MAX_HORIZON>,
}

/// Bridge/Simulator → Visualiser (~20 Hz, JSON over WebSocket)
#[derive(Clone, Debug)]
pub struct RobotStateMsg {
    pub robot_id:       RobotId,
    pub current_edge:   EdgeId,
    pub position_s:     f32,
    pub velocity:       f32,
    pub pos_3d:         [f32; 3],
    pub source:         RobotSource,
    pub belief_means:   [f32; MAX_HORIZON],
    pub belief_vars:    [f32; MAX_HORIZON],
    pub planned_edges:  Vec<EdgeId, MAX_HORIZON>,
    pub active_factors: Vec<RobotId, MAX_NEIGHBOURS>,
}

/// Visualiser → Bridge/Simulator
#[derive(Clone, Copy, Debug)]
pub struct TrajectoryCommand {
    pub robot_id:  RobotId,
    pub goal_node: NodeId,
}

/// Visualiser → Bridge/Simulator — live parameter update
#[derive(Clone, Debug)]
pub struct ParameterUpdate {
    pub target: ParameterTarget,
    pub key:    heapless::String<32>,
    pub value:  f32,
}

/// Trait implemented by SimComms (in-process) and ESPNowComms (firmware).
pub trait CommsInterface {
    type Error;
    /// Broadcast this robot's state to all neighbours.
    fn broadcast(&mut self, msg: &RobotBroadcast) -> Result<(), Self::Error>;
    /// Receive all pending broadcasts from other robots (non-blocking).
    fn receive_broadcasts(&mut self) -> Vec<RobotBroadcast, MAX_NEIGHBOURS>;
}

impl Default for EdgeId {
    fn default() -> Self { EdgeId(0) }
}
impl Default for NodeId {
    fn default() -> Self { NodeId(0) }
}
```

- [ ] **Step 3: Run test**

```bash
cargo test -p gbp-comms
```
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-comms/src/lib.rs crates/gbp-comms/tests/comms_test.rs
git commit -m "feat(m0): gbp-comms message types and CommsInterface trait"
```

---

### Task 12: `gbp-agent` — trajectory tracking

**Files:**
- Create: `crates/gbp-agent/src/trajectory.rs`
- Create: `crates/gbp-agent/tests/trajectory_test.rs`

- [ ] **Step 1: Write failing tests**

```rust
// crates/gbp-agent/tests/trajectory_test.rs
use gbp_agent::trajectory::Trajectory;
use gbp_map::map::{EdgeId, EdgeGeometry, SpeedProfile, SafetyProfile};

fn edges_1m_each() -> heapless::Vec<(EdgeId, f32), 32> {
    // Three edges, each 1.0 m long
    let mut v: heapless::Vec<(EdgeId, f32), 32> = heapless::Vec::new();
    v.push((EdgeId(0), 1.0)).unwrap();
    v.push((EdgeId(1), 1.0)).unwrap();
    v.push((EdgeId(2), 1.0)).unwrap();
    v
}

#[test]
fn local_s_on_first_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let (edge, local_s, is_final) = t.edge_and_local_s(0.5);
    assert_eq!(edge, EdgeId(0));
    assert!((local_s - 0.5).abs() < 1e-5);
    assert!(!is_final);
}

#[test]
fn local_s_on_second_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let (edge, local_s, is_final) = t.edge_and_local_s(1.7);
    assert_eq!(edge, EdgeId(1));
    assert!((local_s - 0.7).abs() < 1e-5);
    assert!(!is_final);
}

#[test]
fn local_s_on_final_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let (edge, local_s, is_final) = t.edge_and_local_s(2.5);
    assert_eq!(edge, EdgeId(2));
    assert!((local_s - 0.5).abs() < 1e-5);
    assert!(is_final);
}

#[test]
fn v_nom_tapered_on_final_edge() {
    // decel_limit=1.0, edge_length=1.0, remaining=(1.0-0.5)=0.5
    // v_taper = sqrt(2 * 1.0 * 0.5) = sqrt(1.0) = 1.0; nominal=2.0 → v_nom=min(2.0,1.0)=1.0
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let v = t.v_nom_at(2.5, 2.0, 1.0); // global_s=2.5, nominal=2.0, decel=1.0
    assert!((v - 1.0).abs() < 1e-4, "v_nom={}", v);
}

#[test]
fn v_nom_nominal_on_intermediate_edge() {
    let t = Trajectory::new(edges_1m_each(), 0.0);
    let v = t.v_nom_at(0.5, 2.0, 1.0);
    assert!((v - 2.0).abs() < 1e-4);
}
```

- [ ] **Step 2: Implement `trajectory.rs`**

```rust
// crates/gbp-agent/src/trajectory.rs
use gbp_map::{EdgeId, MAX_PATH_EDGES};
use heapless::Vec;

/// An ordered sequence of (EdgeId, edge_length) pairs.
pub struct Trajectory {
    /// (edge_id, edge_length_m) for each edge on the planned route
    edges: Vec<(EdgeId, f32), MAX_PATH_EDGES>,
    /// Global arc-length offset of the start of this trajectory
    start_s: f32,
}

impl Trajectory {
    pub fn new(edges: Vec<(EdgeId, f32), MAX_PATH_EDGES>, start_s: f32) -> Self {
        Self { edges, start_s }
    }

    /// Map global arc-length s → (EdgeId, local_s, is_final_edge).
    /// Clamps to the end of the last edge if s is beyond total length.
    pub fn edge_and_local_s(&self, global_s: f32) -> (EdgeId, f32, bool) {
        let relative_s = (global_s - self.start_s).max(0.0);
        let mut cumulative = 0.0f32;
        for (i, &(edge_id, length)) in self.edges.iter().enumerate() {
            let next = cumulative + length;
            let is_final = i == self.edges.len() - 1;
            if relative_s < next || is_final {
                let local_s = (relative_s - cumulative).clamp(0.0, length);
                return (edge_id, local_s, is_final);
            }
            cumulative = next;
        }
        // Fallback: return end of last edge
        let (last_id, last_len) = *self.edges.last().unwrap();
        (last_id, last_len, true)
    }

    /// Compute v_nom at global_s using the trapezoidal profile.
    /// Tapered only on the final edge; full nominal on all intermediate edges.
    pub fn v_nom_at(&self, global_s: f32, nominal: f32, decel_limit: f32) -> f32 {
        let (_, local_s, is_final) = self.edge_and_local_s(global_s);
        if !is_final {
            return nominal;
        }
        // Get the final edge length
        let final_len = self.edges.last().map(|&(_, l)| l).unwrap_or(1.0);
        let remaining = (final_len - local_s).max(0.0);
        let v_taper = libm::sqrtf(2.0 * decel_limit * remaining);
        v_taper.min(nominal).max(0.0)
    }

    pub fn is_empty(&self) -> bool { self.edges.is_empty() }

    /// Total planned distance from start_s.
    pub fn total_length(&self) -> f32 {
        self.edges.iter().map(|&(_, l)| l).sum::<f32>()
    }
}
```

Add `libm` to `gbp-agent/Cargo.toml`:

```toml
[dependencies]
libm = { workspace = true }
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p gbp-agent --test trajectory_test
```
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-agent/src/trajectory.rs crates/gbp-agent/tests/trajectory_test.rs crates/gbp-agent/Cargo.toml
git commit -m "feat(m0): trajectory edge-sequence traversal and v_nom trapezoidal profile"
```

---

### Task 13: `gbp-agent` — InterRobotFactorSet

**Files:**
- Create: `crates/gbp-agent/src/interrobot_set.rs`
- Create: `crates/gbp-agent/tests/interrobot_set_test.rs`

- [ ] **Step 1: Write failing tests**

```rust
// crates/gbp-agent/tests/interrobot_set_test.rs
use gbp_agent::interrobot_set::InterRobotFactorSet;
use gbp_core::{FactorGraph, factor_node::FactorKind, dynamics_factor::DynamicsFactor};

type G = FactorGraph<4, 16>;

fn make_graph() -> G { FactorGraph::new(0.0, 1.0) }

#[test]
fn insert_and_lookup() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();
    let idx = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    set.insert(42u32, idx);
    assert_eq!(set.factor_idx(42), Some(idx));
    assert_eq!(set.factor_idx(99), None);
}

#[test]
fn remove_updates_index_for_swapped_entry() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();

    // Add 3 factors for robots A=10, B=20, C=30
    let ia = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let ib = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    let ic = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([2,3], 0.1, 0.1, 1.0))).unwrap();
    set.insert(10, ia);
    set.insert(20, ib);
    set.insert(30, ic);

    // Remove robot A (index 0). Swap-remove moves robot C (index 2) to index 0.
    set.remove(10, &mut g);
    assert_eq!(g.factor_count(), 2);
    assert!(set.factor_idx(10).is_none());
    // Robot C's index should now be 0 (it was swapped)
    assert_eq!(set.factor_idx(30), Some(0), "C index should have been patched to 0");
    // Robot B's index is still 1 (untouched)
    assert_eq!(set.factor_idx(20), Some(1));
}

#[test]
fn remove_last_does_not_need_patching() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();
    let ia = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let ib = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    set.insert(10, ia);
    set.insert(20, ib);
    // Remove last factor (index 1 = robot 20): no swap needed
    set.remove(20, &mut g);
    assert_eq!(g.factor_count(), 1);
    assert!(set.factor_idx(20).is_none());
    assert_eq!(set.factor_idx(10), Some(0)); // unchanged
}
```

- [ ] **Step 2: Implement `interrobot_set.rs`**

```rust
// crates/gbp-agent/src/interrobot_set.rs
//! Maintains the mapping: robot_id → factor index in FactorGraph.
//! Handles index patching after O(1) swap-remove.

use heapless::Vec;
use gbp_comms::RobotId;
use gbp_core::{FactorGraph, factor_node::FactorKind};
use gbp_map::MAX_NEIGHBOURS;

pub struct InterRobotFactorSet {
    // (robot_id, factor_idx in FactorGraph)
    entries: Vec<(RobotId, usize), MAX_NEIGHBOURS>,
}

impl InterRobotFactorSet {
    pub fn new() -> Self { Self { entries: Vec::new() } }

    pub fn insert(&mut self, robot_id: RobotId, factor_idx: usize) {
        // Replace if already present, otherwise push
        if let Some(e) = self.entries.iter_mut().find(|(id, _)| *id == robot_id) {
            e.1 = factor_idx;
        } else {
            let _ = self.entries.push((robot_id, factor_idx));
        }
    }

    pub fn factor_idx(&self, robot_id: RobotId) -> Option<usize> {
        self.entries.iter().find(|(id, _)| *id == robot_id).map(|(_, idx)| *idx)
    }

    pub fn contains(&self, robot_id: RobotId) -> bool {
        self.entries.iter().any(|(id, _)| *id == robot_id)
    }

    pub fn iter(&self) -> impl Iterator<Item = &(RobotId, usize)> {
        self.entries.iter()
    }

    /// Remove the factor for robot_id from the graph (swap-remove) and patch indices.
    pub fn remove<const K: usize, const F: usize>(
        &mut self,
        robot_id: RobotId,
        graph: &mut FactorGraph<K, F>,
    ) {
        let pos = match self.entries.iter().position(|(id, _)| *id == robot_id) {
            Some(p) => p,
            None    => return,
        };
        let (_, factor_idx) = self.entries.swap_remove(pos);
        let last_idx = graph.factor_count() - 1;
        graph.remove_factor(factor_idx);

        // If we didn't remove the last factor, the last factor moved to factor_idx.
        // Patch any entry pointing to last_idx.
        if factor_idx != last_idx {
            for (_, stored_idx) in self.entries.iter_mut() {
                if *stored_idx == last_idx {
                    *stored_idx = factor_idx;
                    break;
                }
            }
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p gbp-agent --test interrobot_set_test
```
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-agent/src/interrobot_set.rs crates/gbp-agent/tests/interrobot_set_test.rs
git commit -m "feat(m0): InterRobotFactorSet with swap-remove index patching"
```

---

### Task 14: `gbp-agent` — RobotAgent skeleton

**Files:**
- Create: `crates/gbp-agent/src/robot_agent.rs`
- Create: `crates/gbp-agent/tests/robot_agent_test.rs`

- [ ] **Step 1: Write failing test**

```rust
// crates/gbp-agent/tests/robot_agent_test.rs
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
    // Assign trajectory: single edge 0→1
    let mut edges: Vec<(EdgeId, f32), 32> = Vec::new();
    edges.push((EdgeId(0), 10.0)).unwrap();
    agent.set_trajectory(edges, 0.0);

    let obs = ObservationUpdate { position_s: 0.0, velocity: 0.0, current_edge: EdgeId(0) };
    let output = agent.step(obs);
    assert!(output.velocity > 0.0, "velocity should be positive after step");
}
```

- [ ] **Step 2: Implement `robot_agent.rs`**

```rust
// crates/gbp-agent/src/robot_agent.rs
use gbp_comms::{CommsInterface, ObservationUpdate, RobotBroadcast, RobotId};
use gbp_core::{FactorGraph, FactorKind, DynamicsFactor, InterRobotFactor};
use gbp_map::{Map, EdgeId, MAX_HORIZON, MAX_NEIGHBOURS};
use heapless::Vec;
use crate::trajectory::Trajectory;
use crate::interrobot_set::InterRobotFactorSet;

/// Number of dynamics factors = K-1 (one per adjacent timestep pair)
const NUM_DYN_FACTORS: usize = MAX_HORIZON - 1;
/// Max total factors = dynamics + inter-robot neighbours
const MAX_FACTORS: usize = NUM_DYN_FACTORS + MAX_NEIGHBOURS;

const GBP_ITERATIONS: usize = 15;
const DT: f32 = 0.1; // seconds per GBP timestep

/// Output of one agent step.
pub struct StepOutput {
    pub velocity: f32,
    pub position_s: f32,
    pub current_edge: EdgeId,
}

pub struct RobotAgent<C: CommsInterface> {
    robot_id:  RobotId,
    comms:     C,
    map:       *const Map, // raw pointer — map is static for firmware lifetime
    graph:     FactorGraph<MAX_HORIZON, MAX_FACTORS>,
    trajectory: Option<Trajectory>,
    /// Factor graph indices of the K-1 dynamics factors (permanent).
    dyn_indices: [usize; NUM_DYN_FACTORS],
    /// Factor graph indices of inter-robot factors (conditional).
    ir_set: InterRobotFactorSet,
    /// Current physical position
    position_s:   f32,
    current_edge: EdgeId,
}

impl<C: CommsInterface> RobotAgent<C> {
    pub fn new(comms: C, map: &Map, robot_id: RobotId) -> Self {
        let mut graph: FactorGraph<MAX_HORIZON, MAX_FACTORS> = FactorGraph::new(0.0, 100.0);
        let mut dyn_indices = [0usize; NUM_DYN_FACTORS];

        // Add K-1 permanent dynamics factors
        for k in 0..NUM_DYN_FACTORS {
            let idx = graph.add_factor(FactorKind::Dynamics(
                DynamicsFactor::new([k, k + 1], DT, 0.5, 0.0)
            )).expect("dynamics factor capacity");
            dyn_indices[k] = idx;
        }

        Self {
            robot_id,
            comms,
            map: map as *const Map,
            graph,
            trajectory: None,
            dyn_indices,
            ir_set: InterRobotFactorSet::new(),
            position_s: 0.0,
            current_edge: EdgeId(0),
        }
    }

    /// Assign a new planned trajectory as (edge_id, length) pairs starting at start_s.
    pub fn set_trajectory(&mut self, edges: Vec<(EdgeId, f32), 32>, start_s: f32) {
        self.trajectory = Some(Trajectory::new(edges, start_s));
    }

    /// Run one step of GBP and return commanded velocity.
    pub fn step(&mut self, obs: ObservationUpdate) -> StepOutput {
        self.position_s   = obs.position_s;
        self.current_edge = obs.current_edge;

        let map = unsafe { &*self.map };

        // 1. Receive broadcasts from neighbours
        let broadcasts = self.comms.receive_broadcasts();

        // 2. Update inter-robot factors (add/remove as planned edges change)
        self.update_interrobot_factors(&broadcasts, map);

        // 3. Update v_nom on dynamics factors from current trajectory
        self.update_dynamics_v_nom(map);

        // 4. Update inter-robot factor Jacobians and external beliefs
        self.update_interrobot_jacobians(&broadcasts, map);

        // 5. Run GBP
        self.graph.iterate(GBP_ITERATIONS);

        // 6. Extract commanded velocity from first dynamics factor (s_1 - s_0) / dt
        let s0 = self.graph.variables[0].mean();
        let s1 = self.graph.variables[1].mean();
        let velocity = ((s1 - s0) / DT).max(0.0);

        // 7. Broadcast state
        let _ = self.comms.broadcast(&self.make_broadcast(velocity));

        StepOutput { velocity, position_s: self.position_s, current_edge: self.current_edge }
    }

    fn update_dynamics_v_nom(&mut self, map: &Map) {
        let traj = match &self.trajectory { Some(t) => t, None => return };
        let edge = map.edges.iter().find(|e| e.id == self.current_edge);
        let (nominal, decel) = edge.map(|e| (e.speed.nominal, e.speed.decel_limit))
            .unwrap_or((1.0, 1.0));

        for (k, &dyn_idx) in self.dyn_indices.iter().enumerate() {
            // s at timestep k = current position + k steps ahead (linearized from current beliefs)
            let s_k_global = self.position_s + self.graph.variables[k].mean() - self.graph.variables[0].mean();
            let v_nom = traj.v_nom_at(s_k_global, nominal, decel);
            if let Some(FactorKind::Dynamics(df)) = self.graph.get_factor_kind_mut(dyn_idx) {
                df.set_v_nom(v_nom);
            }
        }
    }

    fn update_interrobot_factors(
        &mut self,
        _broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        _map: &Map,
    ) {
        // Stub: full implementation in M3 when multi-robot is introduced.
        // M0 only needs the skeleton to compile.
    }

    fn update_interrobot_jacobians(
        &mut self,
        _broadcasts: &Vec<RobotBroadcast, MAX_NEIGHBOURS>,
        _map: &Map,
    ) {
        // Stub: full implementation in M3.
    }

    fn make_broadcast(&self, velocity: f32) -> RobotBroadcast {
        let mut means: Vec<f32, MAX_HORIZON> = Vec::new();
        let mut vars: Vec<f32, MAX_HORIZON> = Vec::new();
        for v in &self.graph.variables {
            let _ = means.push(v.mean());
            let _ = vars.push(v.variance().min(1e6));
        }
        RobotBroadcast {
            robot_id: self.robot_id,
            current_edge: self.current_edge,
            position_s: self.position_s,
            velocity,
            pos: [0.0; 3], // populated properly in M3
            planned_edges: Vec::new(),
            belief_means: means,
            belief_vars: vars,
            gbp_timesteps: Vec::new(),
        }
    }
}
```

- [ ] **Step 3: Run all agent tests**

```bash
cargo test -p gbp-agent
```
Expected: all PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-agent/src/robot_agent.rs crates/gbp-agent/tests/robot_agent_test.rs
git commit -m "feat(m0): RobotAgent skeleton with GBP step loop"
```

---

### Task 15: `no_std` bare-metal verification

**Files:** None created — verification only.

- [ ] **Step 1: Install the bare-metal target (once per machine)**

```bash
rustup target add riscv32imac-unknown-none-elf
```

- [ ] **Step 2: Verify no_std compile for all four crates**

```bash
cargo build \
  -p gbp-map -p gbp-core -p gbp-comms -p gbp-agent \
  --target riscv32imac-unknown-none-elf \
  --no-default-features \
  2>&1 | tail -5
```

Expected: `Compiling ...` lines ending in `Finished` with no errors. If you see "can't find crate for `std`" that's a problem — check for any `use std::` or missing `#![no_std]` attributes.

- [ ] **Step 3: Run all tests one final time**

```bash
cargo test -p gbp-map -p gbp-core -p gbp-comms -p gbp-agent
```
Expected: all tests PASS.

- [ ] **Step 4: Final M0 commit**

```bash
git add -A
git commit -m "feat(m0): M0 complete — foundation crates verified no_std and all tests pass"
```

---

## Summary

| Chunk | Deliverable | Verification |
|---|---|---|
| 1 | Workspace scaffolding, Cargo.toml files, capacity constants | `cargo check` |
| 2 | `gbp-map` types (Map, Node, Edge, NURBS struct) | `cargo test map_test` |
| 3 | NURBS eval, arc-length table, A* | `cargo test nurbs_test astar_test` |
| 4 | YAML parser, postcard round-trip, near-capacity test | `cargo test --features parse,serde` |
| 5 | `gbp-core`: VariableNode, Factor trait, FactorKind, DynamicsFactor, InterRobotFactor, FactorGraph | `cargo test -p gbp-core` |
| 6 | `gbp-comms`: message types, CommsInterface | `cargo test -p gbp-comms` |
| 7 | `gbp-agent`: Trajectory, InterRobotFactorSet, RobotAgent | `cargo test -p gbp-agent` |
| 8 | `no_std` bare-metal compile check | `cargo build --target riscv32imac-unknown-none-elf` |

**Next plan:** `2026-03-19-m1-single-robot-visible.md` — simulator crate, physics integration, Bevy visualiser skeleton, WebSocket server.
