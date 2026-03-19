# GBPTrajectoryPlanner

**Distributed 1D velocity coordination for line-following robots using Gaussian Belief Propagation.**

Each robot follows a directed graph of 3D trajectories and can only control its velocity along its current edge. When edges converge at merge nodes, robots negotiate velocity adjustments through a fully distributed factor graph — no central planner, no pre-assigned yielding rules.

---

## Repository

`https://github.com/<your-org>/GBPTrajectoryPlanner`

**Language**: Rust  
**Embedded target**: ESP32-C5 (via `esp32c6` chip feature in esp-hal — confirmed working on physical boards)  
**Simulator target**: native `x86_64`  
**Visualiser**: Bevy — runs native or compiles to WebAssembly via `trunk`  
**Concurrency model**: Embassy async (ESP32), Tokio (simulator/bridge)  
**Reference**: [gbpplanner — Patwardhan et al., IEEE RA-L 2023](https://arxiv.org/abs/2203.11618)

---

## Problem Definition

### Physical setup

- Each robot follows edges in a **directed graph**. It cannot change its path, only its speed.
- The single degree of freedom is **scalar arc-length position** `s_k ∈ ℝ` along the current edge. Velocity is derived: `v_k = (s_{k+1} - s_k) / Δt`.
- The world is **3D**. Node positions and edge geometry are 3D. The GBP variable remains scalar — 3D geometry enters only through the Jacobian computation in the inter-robot factor.
- A trajectory is an **ordered list of edges** produced by A\* path planning on the directed graph.
- Edges are either **line segments** (two 3D points) or **NURBS curves** (cubic, centripetal parameterization).

### Collision scenarios

Two distinct types, both handled:

1. **Same-edge rear-end**: Robot B is behind Robot A on the same edge. If A brakes, B must respond.
2. **Merge collision**: Robots on different edges converging to the same downstream edge cannot enter simultaneously.

### Factor gating — planned edge set intersection

Two robots establish an inter-robot factor if and only if their **planned edge sequences share at least one edge** within the planning horizon. Geometrically exact — robots on parallel non-intersecting edges never share an edge. Robots converging at a merge node will share the post-merge edge, so the factor fires exactly when needed.

```
Robot A planned edges: [E12, E13, E14, E15]
Robot B planned edges: [E09, E13, E15, E16]
Shared: {E13, E15} → spawn inter-robot factors for those timesteps
```

Each robot broadcasts its planned edge sequence alongside GBP messages. The check is a set intersection on `u16` edge indices.

---

## Velocity-Based Dynamics — Design Rationale

### Why gbpplanner's goal factor is wrong for this system

gbpplanner slides the goal state forward every tick regardless of what the robot is doing. This creates a runaway reference — during a forced stop the gap between actual and goal position grows unboundedly, the goal factor accumulates information that overwhelms everything else, and demands physically impossible accelerations. It is an integrating error with no reset: windup.

### The replacement: velocity prior + trapezoidal profile

There is **no goal factor** in this system. Instead:

- The **dynamics factor** encodes "try to move at `v_nom`" — a soft velocity prior on each timestep transition.
- `v_nom(s)` is a position-dependent function derived from the edge's speed profile and the robot's remaining distance to the goal node.
- When an inter-robot factor forces a stop, the dynamics factor simply loses that negotiation for those timesteps — no accumulated error, no windup. When the constraint lifts, the robot naturally accelerates back toward `v_nom`.
- Deceleration at the goal node is handled by **tapering `v_nom` to zero** using the edge's `decel_limit` working backwards from the end node.

### Trapezoidal velocity profile

```
v_nom(s) = min(
    edge.speed.nominal,
    sqrt(2.0 * edge.speed.decel_limit * (edge.length - s))
)
```

This gives full nominal speed in the middle of an edge and a smooth deceleration to zero at the end. For intermediate nodes (robot continues onto the next edge), `v_nom` is not tapered — the robot maintains speed through the node. Only the final edge of the planned trajectory gets the taper.

When a robot is waiting (forced to zero velocity by an inter-robot factor):
- `v_nom(s)` remains positive (still wants to move)
- The inter-robot factor precision dominates
- The dynamics factor asks for `v_nom` but the graph finds the velocity that satisfies all factors jointly
- No windup because the dynamics factor has no memory — it re-evaluates at current `s` every iteration

### Dynamics factor formula

```
f_dyn(s_k, s_{k+1}) ∝ exp(-½ · ((s_{k+1} - s_k)/Δt - v_nom(s_k))² / σ_dyn²)
```

Residual:
```
e = (s_{k+1} - s_k)/Δt - v_nom(s_k)
```

Jacobian:
```
∂e/∂s_k     = -1/Δt   (plus correction if v_nom varies with s_k, usually small)
∂e/∂s_{k+1} = +1/Δt
```

`σ_dyn` controls how strongly the robot tracks `v_nom`. Smaller = more rigid velocity tracking. Per-edge tuning is possible (e.g. ramps get tighter `σ_dyn`).

---

## Factor System Design

### Core principle

Adding a new factor type requires changes in **exactly three places**, all marked `// FACTOR EXTENSION POINT` in the source. Nothing else in `gbp-core` changes. See `docs/adding_a_factor.md` for the step-by-step guide.

### The `Factor` trait

```rust
// crates/gbp-core/src/factor_node.rs

pub trait Factor {
    /// Variable indices this factor connects (into FactorGraph.variables)
    fn variable_indices(&self) -> &[usize];

    /// Linearize around current beliefs → Jacobian row + residual + precision
    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor;

    /// Called each GBP iteration — re-evaluate geometry, update cached values
    /// Default: no-op. Override for nonlinear factors that re-linearize.
    fn update(&mut self, variables: &[VariableNode]) {}

    /// Whether this factor participates in message passing this iteration.
    /// Inactive factors are skipped but remain in the graph (avoids index churn).
    fn is_active(&self) -> bool { true }
}

pub struct LinearizedFactor {
    pub jacobian:  heapless::Vec<f32, MAX_FACTOR_VARIABLES>,
    pub residual:  f32,
    pub precision: f32,    // 1/σ²
}
```

### `FactorKind` — the single extension enum

```rust
// crates/gbp-core/src/factor_node.rs
// FACTOR EXTENSION POINT 1/3 — add new factor variants here

pub enum FactorKind {
    Dynamics(DynamicsFactor),
    InterRobot(InterRobotFactor),
    // Future: Obstacle(ObstacleFactor), SpeedLimit(SpeedLimitFactor), ...
}

impl FactorKind {
    pub fn as_factor(&self) -> &dyn Factor {
        match self {
            // FACTOR EXTENSION POINT 2/3 — add match arm here
            Self::Dynamics(f)   => f,
            Self::InterRobot(f) => f,
        }
    }
    pub fn as_factor_mut(&mut self) -> &mut dyn Factor {
        match self {
            // FACTOR EXTENSION POINT 2/3 (mut) — add match arm here
            Self::Dynamics(f)   => f,
            Self::InterRobot(f) => f,
        }
    }
}
```

### `FactorGraph` — clean add/remove API

```rust
// crates/gbp-core/src/factor_graph.rs

pub struct FactorGraph<const K: usize, const F: usize> {
    pub variables: [VariableNode; K],
    factors: heapless::Vec<FactorKind, F>,
}

impl<const K: usize, const F: usize> FactorGraph<K, F> {
    /// Add any factor. Returns slot index for later reference.
    pub fn add_factor(&mut self, factor: FactorKind) -> Result<usize, CapacityError> {
        let idx = self.factors.len();
        self.factors.push(factor)?;
        Ok(idx)
    }

    /// Remove by index (swap-remove, O(1)). Invalidates the last factor's index.
    /// The agent layer is responsible for tracking index validity.
    pub fn remove_factor(&mut self, idx: usize) -> FactorKind {
        self.factors.swap_remove(idx)
    }

    /// Iterate — the hot path. Skips inactive factors.
    pub fn iterate(&mut self, iterations: usize) {
        for _ in 0..iterations {
            // 1. Each factor re-evaluates its geometry / cached state
            for factor in self.factors.iter_mut() {
                if factor.as_factor().is_active() {
                    factor.as_factor_mut().update(&self.variables);
                }
            }
            // 2. Factors → variables: compute and send information messages
            self.factor_to_variable_pass();
            // 3. Variables → factors: update variable beliefs, send back
            self.variable_to_factor_pass();
        }
    }
}
```

### Factor lifecycle

| Factor type | Lifecycle | Who manages it |
|---|---|---|
| `DynamicsFactor` | Permanent — added at construction, never removed | `FactorGraph::new()` |
| `InterRobotFactor` | Conditional — added/removed by edge intersection check | `RobotAgent::update_interrobot_factors()` |
| Future obstacle factors | Conditional — based on proximity to static obstacles | Agent layer |
| Future speed limit factors | Permanent per-edge — added at trajectory assignment | Agent layer |

### Index stability and swap-remove

`remove_factor` uses swap-remove for O(1) removal. This means removing factor at index `i` moves the last factor to slot `i`. The agent layer tracks factor indices using a small map:

```rust
// crates/gbp-agent/src/robot_agent.rs

struct InterRobotFactorSet {
    // robot_id → factor index in graph
    entries: heapless::Vec<(u32, usize), MAX_NEIGHBOURS>,
}

impl InterRobotFactorSet {
    fn remove(&mut self, graph: &mut FactorGraph<K, F>, robot_id: u32) {
        if let Some(pos) = self.entries.iter().position(|(id, _)| *id == robot_id) {
            let (_, idx) = self.entries.swap_remove(pos);
            let last_idx = graph.factor_count() - 1;
            graph.remove_factor(idx);
            // If we didn't remove the last factor, the last factor moved to idx
            // Update any entry that was pointing to last_idx
            if idx != last_idx {
                for (_, stored_idx) in self.entries.iter_mut() {
                    if *stored_idx == last_idx {
                        *stored_idx = idx;
                        break;
                    }
                }
            }
        }
    }
}
```

This is the only place swap-remove index tracking exists — isolated in the agent layer, invisible to `gbp-core`.

---

## Existing Factor Types

### `DynamicsFactor` (permanent, pairwise)

Connects `(s_k, s_{k+1})`. Encodes the velocity prior.

```rust
pub struct DynamicsFactor {
    var_indices: [usize; 2],   // [k, k+1]
    dt:          f32,
    sigma:       f32,
    // Updated each iteration from the trajectory + agent position:
    v_nom:       f32,          // trapezoidal profile value at current s_k belief
}

// v_nom(s) = min(nominal, sqrt(2 * decel * (edge_length - s)))
// Only tapered on the final edge of the planned trajectory.
// On all other edges: v_nom = edge.speed.nominal
```

No windup. No time-indexed reference. When an inter-robot factor forces `v = 0`,
the dynamics factor simply loses the negotiation for that timestep — `v_nom` is
still positive, but the joint optimum across all factors produces near-zero
velocity. When the constraint lifts, the robot accelerates back toward `v_nom`.

### `InterRobotFactor` (conditional, pairwise across robots)

Connects `s_Ak` and `s_Bk` for robots A and B at the same timestep k. Linearized Euclidean collision avoidance projected onto 1D arc-length space.

Spawned when planned edge sequences share an edge. Removed when they no longer share any edge.

```rust
pub struct InterRobotFactor {
    var_idx_a:  usize,          // index into robot A's variable array
    var_idx_b:  usize,          // conceptual — B's belief is injected as an external message
    d_safe:     f32,            // from edge.safety.clearance
    sigma_r:    f32,            // weakens for larger timestep index k
    // Re-evaluated each iteration:
    jacobian_a: f32,
    jacobian_b: f32,
    is_active_: bool,
}
```

3D Jacobians (computed in agent layer, passed to factor as scalars):

```
J_A = -(p_A(s̄_A) - p_B(s̄_B))ᵀ · p_A'(s̄_A) / ‖p_A - p_B‖
J_B =  (p_A(s̄_A) - p_B(s̄_B))ᵀ · p_B'(s̄_B) / ‖p_A - p_B‖
```

Information matrix:

```
Λ_r = (1/σ_r²) · [J_A²,    J_A·J_B]
                  [J_A·J_B, J_B²   ]
```

---

## Map Format

### Overview

YAML map file. Authoritative world representation. Parsed on PC, binary-serialized (`postcard`) for ESP32 delivery.

```yaml
format_version: "1.0"
map_id: "test_loop"
metadata:
  description: "..."
  coordinate_frame: "map"
  units: { length: "m", angle: "rad" }
tags:
  environment: "simulation"
  revision: 1
nodes:   [...]
edges:   [...]
fiducials: [...]
zones:   []
layers:  []
```

### Nodes

```yaml
- id: "P001"
  nodePosition:
    x: 0.850       # f64, meters
    y: 8.965
    z: 0.000
    theta: 0.000   # radians, 0 = facing +X
  actions:         # optional
    - actionType: "none"          # "none"|"charge"|"pickup"|"dropoff"|...
      actionDescription: "..."
      blockingType: "NONE"        # "NONE"|"HARD"|"SOFT"
  customProperties:               # optional
    type: "waypoint"              # see node type table below
    name: "Charger-01"            # optional display name
    zone: "loop"                  # optional zone label
```

| `customProperties.type` | Semantic meaning | A\* cost hint |
|---|---|---|
| `waypoint` | Generic intermediate point | Normal |
| `divert` | Outgoing edge splits | Routing decision |
| `merge` | Incoming edges converge | Factor gating hotspot |
| `charger` | Charging station | High unless seeking charge |
| `toploader` | Load pickup | High unless loading |
| `discharge` | Unload station | High unless unloading |
| *(absent)* | Generic node | Normal |

### Edges

```yaml
- id: "P001 --- P002"     # convention: "StartId --- EndId"
  startNodeId: "P001"
  endNodeId:   "P002"
  direction: "left"        # "left"|"right"|"" — lateral hint for robot
  directionality: "forward_only"   # "forward_only"|"bidirectional"
  geometry:
    # Line geometry
    type: "line"
    points:
      - [0.850, 8.965, 0.000]    # [x, y, z]
      - [3.545, 9.886, 0.000]
    length: 3.029                # meters, precomputed
    length_computed: true

    # OR: NURBS spline geometry
    type: "spline"
    representation: "nurbs"      # always "nurbs" in current maps
    frame: "map"
    control_points:              # array of [x, y, z], 4–12 points
      - [0.850,  8.965, 0.000]
      - [0.860,  9.311, 0.002]
      # ...
    degree: 3                    # always cubic in current maps
    knots: [0.0, 0.0, 0.0, 0.0, 0.111, ..., 1.0, 1.0, 1.0, 1.0]
    # length = n_control_points + degree + 1
    # always clamped (first/last value repeated degree+1 times)
    weights: [1.0, 1.0, ...]     # all 1.0 in current maps (non-rational)
    length_estimate: 3.728       # meters
    length_computed: true
    parameterization: "centripetal"   # always centripetal in current maps

  speed_profile:
    max:         2.5   # m/s — hard limit
    nominal:     2.0   # m/s — feeds v_nom in DynamicsFactor
    accel_limit: 1.0   # m/s²
    decel_limit: 1.0   # m/s² — used in trapezoidal profile taper
  safety:
    clearance: 0.3     # meters — d_safe for InterRobotFactor
  tags:
    surface:   ""      # ""|"flat"|"ramp"|"platform"
    test_edge: ""      # ""|"true"
    role:      ""      # ""|"toploader_line"|"charging"|"ramp_up"|"ramp_down"|...
```

**Observed in `test_loop_map.yaml`**:
- 45 nodes, ~60 edges
- NURBS edges for all curved transitions and ramps (z varies 0.0 → 1.545 over ~5m on ramp edges)
- Line edges for all straight sections
- Control point count: 4–12 per NURBS edge
- Edge lengths: 0.284m – 12.134m

### Fiducials

```yaml
- id: "pm1"
  type: "magnetic_marker"
  pose:
    position: [6.4945, 9.500, 0.0]
    yaw: 0.0
  association:
    edge_id: "P033 --- P024"
    s: 2.5           # arc-length from startNode (meters)
  tags:
    role: "localization"    # "localization"|"charger_marker"
    phys_id: "12"
```

---

## NURBS Evaluation

### B-spline basis (Cox-de Boor)

```rust
fn nurbs_point(t: f32, control_points: &[[f32;3]], knots: &[f32], degree: usize) -> [f32;3] {
    let span  = find_knot_span(control_points.len()-1, degree, t, knots);
    let basis = basis_functions(span, t, degree, knots);
    let mut p = [0.0f32; 3];
    for i in 0..=degree {
        let cp = control_points[span - degree + i];
        p[0] += basis[i] * cp[0];
        p[1] += basis[i] * cp[1];
        p[2] += basis[i] * cp[2];
    }
    p
}
```

All weights are 1.0 in current maps — NURBS reduces to non-rational B-spline evaluation.

### Arc-length parameterization

The raw parameter `t` does not give uniform speed. A precomputed lookup table maps between `t` and true arc-length `s`.

```rust
pub struct ArcLengthTable {
    t_values:     [f32; ARC_TABLE_SAMPLES],   // uniform in t
    s_values:     [f32; ARC_TABLE_SAMPLES],   // cumulative arc-length
    total_length: f32,
}
// s → t: binary search + linear interpolation
// t → s: direct index + linear interpolation
// ARC_TABLE_SAMPLES = 64 (tune based on sharpest curve error tolerance)
```

### Tangent for Jacobian

```rust
fn nurbs_tangent(t: f32, ...) -> [f32; 3] {
    // Central difference: (point(t+ε) - point(t-ε)) / 2ε, then normalize
}
```

---

## `gbp-map` Crate

```toml
[dependencies]
heapless = { version = "0.8", default-features = false }
postcard = { version = "1",   default-features = false }

[features]
default = []
parse   = ["dep:serde", "dep:serde_yaml"]   # PC only

[dependencies]
serde      = { version = "1", features = ["derive"], optional = true }
serde_yaml = { version = "0.9", optional = true }
```

### Key types

```rust
#![no_std]

pub const MAX_NODES:          usize = 64;
pub const MAX_EDGES:          usize = 96;
pub const MAX_CONTROL_POINTS: usize = 16;
pub const MAX_KNOTS:          usize = 32;
pub const ARC_TABLE_SAMPLES:  usize = 64;

pub struct Map {
    pub map_id:   heapless::String<32>,
    pub nodes:    heapless::Vec<Node, MAX_NODES>,
    pub edges:    heapless::Vec<Edge, MAX_EDGES>,
    pub outgoing: heapless::Vec<heapless::Vec<EdgeId, 8>, MAX_NODES>,
}

pub enum EdgeGeometry {
    Line { start: [f32;3], end: [f32;3], length: f32 },
    Nurbs {
        control_points: heapless::Vec<[f32;3], MAX_CONTROL_POINTS>,
        knots:          heapless::Vec<f32, MAX_KNOTS>,
        degree:         u8,
        length:         f32,
        arc_table:      ArcLengthTable,
    },
}

pub struct SpeedProfile {
    pub max:         f32,
    pub nominal:     f32,    // → v_nom in DynamicsFactor
    pub accel_limit: f32,
    pub decel_limit: f32,    // → taper formula coefficient
}

impl Map {
    pub fn eval_position(&self, edge: EdgeId, s: f32) -> [f32; 3] { ... }
    pub fn eval_tangent(&self,  edge: EdgeId, s: f32) -> [f32; 3] { ... }
}
```

### Map memory footprint (ESP32-C5)

| Component | Size |
|---|---|
| Nodes (45 × ~20B) | ~900 B |
| Line edges (~30 × ~60B) | ~1.8 KB |
| NURBS edges (~30 × ~572B, 12 CP + 64-sample arc table) | ~17 KB |
| Adjacency list | ~720 B |
| **Total** | **~20 KB** |

---

## A\* Path Planning

Runs on PC (simulator/bridge). Result sent to robot as `TrajectoryCommand`.

```rust
fn astar(map: &Map, start: NodeId, goal: NodeId) -> Option<heapless::Vec<EdgeId, MAX_PATH_EDGES>>
// Cost:      g(n) = cumulative traversal time = Σ edge.length / edge.speed.nominal
// Heuristic: h(n) = euclidean(n.pos, goal.pos) / global_max_speed   (admissible)
// Node entry cost: high for charger/toploader unless robot is seeking them
```

---

## Crate Dependencies

```
gbp-core    →  heapless
gbp-comms   →  heapless
gbp-map     →  heapless, postcard  [+serde, serde_yaml with "parse" feature]
gbp-agent   →  gbp-core, gbp-comms, gbp-map
simulator   →  gbp-agent, gbp-comms, gbp-map(+parse), tokio, axum, serde_json
esp32       →  gbp-agent, gbp-comms, gbp-map, esp-hal, esp-radio, embassy-*
bridge      →  gbp-comms, gbp-map(+parse), tokio, axum, serde_json
visualiser  →  gbp-comms, gbp-map(+parse), bevy, bevy_egui
```

**Hard rule**: `gbp-core`, `gbp-agent`, `gbp-comms`, `gbp-map` core must compile `#![no_std]` without `alloc`.

---

## Library Choices

| Concern | Library | Rationale |
|---|---|---|
| Fixed-size collections (`no_std`) | `heapless 0.8` | `Vec`, `String`, `spsc::Queue` with const-generic capacity. Only option for `no_std` without alloc. |
| Binary serialization (ESP32 map delivery) | `postcard 1` | Compact, `no_std`-first, `serde`-based. Designed for embedded. |
| YAML parsing (PC only) | `serde_yaml 0.9` | Feature-gated, never compiled for ESP32. |
| Zero-copy ESP-NOW packet casting | `bytemuck 1` | `Pod`/`Zeroable` derive for `#[repr(C, packed)]` structs. |
| Async runtime (simulator/bridge) | `tokio 1` | Full async, pairs with axum. |
| HTTP + WebSocket server | `axum 0.8` | Tokio-native, serves both static WASM files and WebSocket in one binary. |
| JSON WebSocket protocol | `serde_json 1` | Simple, well-understood, no schema drift risk in this project. |
| 3D rendering + game loop | `bevy 0.15` | Compiles to WASM via `trunk`. Full 3D scene graph, ECS, input handling. |
| Immediate-mode UI panels | `bevy_egui 0.30` | Embeds egui directly into Bevy viewport. Plugin-based panel system. |
| WASM build toolchain | `trunk` | Handles `wasm-bindgen`, asset bundling, dev server. One command. |
| Embedded logging | `defmt 0.3` | Structured logging with minimal code size overhead on ESP32. |
| defmt log transport (ESP32) | `defmt-rtt 0.4` | Logs over RTT (USB JTAG on ESP32-C5). |
| defmt decoding (bridge) | `defmt-decoder 0.3` | Decodes raw defmt frames using the ELF symbol table. |
| ESP32 HAL | `esp-hal ~1.0` | Officially supported by Espressif Rust team. `esp32c6` feature covers C5. |
| ESP-NOW + WiFi | `esp-radio ~0.16` | Same repo as esp-hal. Unstable API flag — pin with `~`. |
| Embassy async executor | `embassy-executor 0.7` | `no_std` async, cooperative scheduling, no FreeRTOS needed. |
| Embassy time | `embassy-time 0.4` | `Timer::after_millis()` for task periods. |
| Embassy sync primitives | `embassy-sync 0.6` | `Channel<T, N>` for inter-task message passing without mutexes. |
| ESP32 allocator | `esp-alloc 0.6` | Heap allocator for the WiFi/embassy stack. GBP core does not use it. |
| Panic handler (ESP32) | `esp-backtrace 0.15` | Prints backtrace on panic, resets chip. |
| NURBS / spline math | Hand-rolled in `gbp-map` | ~150 lines. Cox-de Boor + arc-length table. No `no_std`-compatible NURBS crate exists with the right feature set. |

### Libraries considered and rejected

| Library | Reason not used |
|---|---|
| `nalgebra` | Heavy for `no_std`; 2×2 matrix ops are trivial to inline for this problem |
| `rapier` (physics) | Robot physics is 1D (`s += v·dt`); full rigid body engine is overkill |
| `tokio-tungstenite` (raw WebSocket) | Replaced by `axum` which also serves static WASM files |
| Rerun | Considered for visualisation; rejected because bevy_egui covers the interactive UI requirement |
| `splines` crate | Not `no_std` with the required NURBS support; hand-rolled is simpler |

---

## Repository Structure

```
GBPTrajectoryPlanner/
├── Cargo.toml
│
├── crates/
│   ├── gbp-core/
│   │   └── src/
│   │       ├── lib.rs                  # FACTOR EXTENSION POINT 3/3: export module
│   │       ├── factor_node.rs          # Factor trait, LinearizedFactor, FactorKind enum
│   │       │                           # FACTOR EXTENSION POINT 1/3: FactorKind variant
│   │       │                           # FACTOR EXTENSION POINT 2/3: match arms
│   │       ├── variable_node.rs
│   │       ├── factor_graph.rs         # add_factor / remove_factor / iterate
│   │       ├── dynamics_factor.rs      # velocity prior + trapezoidal profile
│   │       └── interrobot_factor.rs    # 3D Jacobian, edge-intersection gated
│   │
│   ├── gbp-agent/
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── robot_agent.rs          # step(), InterRobotFactorSet, swap-remove tracking
│   │       └── trajectory.rs           # edge sequence traversal, global_s → edge + local_s
│   │
│   ├── gbp-comms/
│   │   └── src/lib.rs                  # CommsInterface, all message types
│   │
│   └── gbp-map/
│       └── src/
│           ├── lib.rs                  # Map, Node, Edge, NodeType, EdgeGeometry
│           ├── nurbs.rs                # Cox-de Boor, ArcLengthTable, tangent
│           ├── astar.rs
│           └── parser.rs              # YAML → Map (feature = "parse")
│
├── targets/
│   ├── simulator/
│   │   └── src/
│   │       ├── main.rs
│   │       ├── simulator.rs
│   │       ├── sim_comms.rs
│   │       └── ws_server.rs
│   │
│   ├── esp32/
│   │   └── src/
│   │       ├── main.rs                 # Embassy tasks
│   │       ├── espnow_comms.rs
│   │       └── physics.rs
│   │
│   ├── bridge/
│   │   └── src/
│   │       ├── main.rs                 # --sim-ws / --esp-udp / --sim-ids / --esp-ids
│   │       ├── espnow_receiver.rs
│   │       └── ws_server.rs
│   │
│   └── visualiser/
│       └── src/
│           ├── main.rs
│           ├── scene.rs
│           ├── belief_renderer.rs
│           ├── ws_client.rs
│           ├── trajectory_input.rs
│           └── ui/
│               ├── mod.rs             # Plugin registry — add new panels here only
│               ├── global_panel.rs    # Sim controls, global GBP params
│               ├── robot_panel.rs     # Per-robot status, params, trajectory input
│               ├── log_panel.rs       # defmt log viewer, filterable by robot + level
│               └── map_panel.rs       # Node/edge inspector
│
├── maps/
│   └── test_loop_map.yaml
│
└── docs/
    ├── adding_a_factor.md             # Step-by-step guide with worked example
    └── architecture.md                # (future) high-level system overview
```

---

## Message Types (`gbp-comms`)

```rust
/// Simulator/physics → Agent (in-process only, never crosses network)
pub struct ObservationUpdate {
    pub position_s:   f32,
    pub velocity:     f32,
    pub current_edge: EdgeId,
}

/// Agent → Agent broadcast (ESP-NOW or SimComms)
pub struct RobotBroadcast {
    pub robot_id:      u32,
    pub current_edge:  EdgeId,
    pub position_s:    f32,
    pub velocity:      f32,
    pub pos:           [f32; 3],
    pub planned_edges: heapless::Vec<EdgeId, MAX_HORIZON>,
    pub belief_means:  [f32; MAX_HORIZON],
    pub belief_vars:   [f32; MAX_HORIZON],
    pub gbp_timesteps: [GBPTimestep; MAX_HORIZON],
}

pub struct GBPTimestep { pub eta: f32, pub lambda: f32 }

/// Visualiser → Bridge/Simulator
pub struct TrajectoryCommand {
    pub robot_id:  u32,
    pub goal_node: NodeId,    // A* runs on receiver
}

/// Bridge/Simulator → Visualiser (~20 Hz, JSON over WebSocket)
pub struct RobotStateMsg {
    pub robot_id:       u32,
    pub current_edge:   EdgeId,
    pub position_s:     f32,
    pub velocity:       f32,
    pub pos_3d:         [f32; 3],
    pub source:         RobotSource,
    pub belief_means:   [f32; MAX_HORIZON],
    pub belief_vars:    [f32; MAX_HORIZON],
    pub planned_edges:  heapless::Vec<EdgeId, MAX_HORIZON>,
    pub active_factors: heapless::Vec<u32, MAX_NEIGHBOURS>,
}

pub enum RobotSource { Simulated, Hardware }

/// Runtime parameter update — visualiser → bridge/simulator
pub struct ParameterUpdate {
    pub target: ParameterTarget,
    pub key:    heapless::String<32>,
    pub value:  f32,
}
pub enum ParameterTarget { Global, Robot(u32) }
```

---

## Deployment Modes

### Mode 1 — Simulator only
```
visualiser  ←WebSocket→  simulator  (all agents in-process, SimComms)
```

### Mode 2 — Hardware only
```
visualiser  ←WebSocket→  bridge  ←UDP→  ESP32 fleet  (ESPNowComms)
```

### Mode 3 — Hybrid
```
visualiser  ←WebSocket→  bridge  ┬←→  simulator (sim-ids)
                                  └←→  ESP32 fleet (esp-ids)
```

`RobotSource` field lets the visualiser render simulated and hardware robots distinctly. `TrajectoryCommand` routed by `robot_id` to correct backend.

### Progressive hardware bring-up
Start all agents in simulator. Migrate one at a time to hardware by moving `robot_id` from `--sim-ids` to `--esp-ids` in the bridge. Fleet continues uninterrupted.

---

## ESP32-C5 Target

```toml
[dependencies]
gbp-core     = { path = "../../crates/gbp-core" }
gbp-agent    = { path = "../../crates/gbp-agent" }
gbp-comms    = { path = "../../crates/gbp-comms" }
gbp-map      = { path = "../../crates/gbp-map" }         # no "parse" feature

esp-hal      = { version = "~1.0",  features = ["esp32c6", "unstable"] }
esp-radio    = { version = "~0.16", features = ["esp32c6", "esp-now", "esp-alloc"] }
embassy-executor = { version = "0.7", features = ["task-arena-size-32768"] }
embassy-time = "0.4"
embassy-sync = "0.6"
esp-alloc    = "0.6"
esp-backtrace = { version = "0.15", features = ["esp32c6", "panic-handler", "println"] }
heapless     = "0.8"
postcard     = { version = "1", default-features = false }
bytemuck     = { version = "1", features = ["derive"] }
defmt        = "0.3"
defmt-rtt    = "0.4"
```

### Embassy tasks

```rust
#[embassy_executor::task]
async fn gbp_task(esp_now: EspNow<'static>, map: &'static Map) {
    let comms = ESPNowComms::new(esp_now);
    let mut agent = RobotAgent::new(comms, map, ROBOT_ID);
    loop {
        let obs = PHYSICS_CHANNEL.receive().await;
        agent.step(obs);
        embassy_time::Timer::after_millis(GBP_PERIOD_MS).await;
    }
}

#[embassy_executor::task] async fn physics_task() { /* s += v·dt */ }
#[embassy_executor::task] async fn vis_task()     { /* serialize + send to bridge */ }
```

### ESP-NOW wire format

```rust
#[repr(C, packed)]
#[derive(bytemuck::Pod, bytemuck::Zeroable, Clone, Copy)]
struct ESPNowPacket {
    robot_id:      u32,
    current_edge:  u16,
    position_s:    f32,
    velocity:      f32,
    pos:           [f32; 3],
    n_planned:     u8,
    planned_edges: [u16; MAX_HORIZON],
    n_timesteps:   u8,
    timesteps:     [GBPTimestepWire; MAX_HORIZON],
}
// At K=12: 4+2+4+4+12+1+24+1+96 = 148 bytes — within 250-byte ESP-NOW limit
```

### Memory budget

| Component | RAM |
|---|---|
| ESP-NOW / WiFi stack | ~100 KB |
| Embassy executor + tasks | ~20 KB |
| Map (nodes + edges + arc tables) | ~20 KB |
| GBP core (K=12, N=8 neighbours) | ~18 KB |
| Agent + trajectory state | ~12 KB |
| esp-alloc heap | 72 KB |
| Buffers, misc | ~8 KB |
| **Total** | **~250 KB / 384 KB SRAM** |

---

## Visualiser

### 3D Scene

- **Edges**: NURBS sampled at N points → 3D curves. Line edges as segments. Node spheres coloured by type.
- **Robots**: arrow mesh oriented along edge tangent at `position_s`. Blue = simulated, orange = hardware.
- **Belief tubes**: evaluate `map.eval_position(edge, mean_k)` for k = 0..K. Tube radius = `sqrt(variance_k)`. Recomputed each frame.
- **Active factors**: line gizmos between robots in `active_factors`.
- **Planned path**: dashed line following `planned_edges` ahead of robot.

### UI panels (bevy_egui, plugin-based)

New panels added to `ui/mod.rs` with one line — no existing code changes needed.

- **Global panel**: play/pause/restart/step, global GBP params (`K`, iterations, `d_safe`, `r_comm`), random trajectory mode.
- **Robot panel**: per-robot status, per-robot params (`σ_dyn`, `σ_r`), trajectory assignment.
- **Log panel**: live defmt stream per robot, filterable by level.
- **Map panel**: click node/edge in 3D view → inspect properties.

### defmt log pipeline

```
ESP32 (defmt-rtt) → USB/UART → bridge (defmt-decoder + ELF) → decoded strings → WebSocket → visualiser log panel
```

### Trajectory input

- **Click node**: click any node in 3D view → assign as goal → `TrajectoryCommand` → A\* on bridge/simulator.
- **Random mode**: simulator assigns random reachable nodes automatically. Toggled from global panel.

---

## Build

```bash
# Serialize map (run after map changes)
cargo run -p simulator -- --serialize-map maps/test_loop_map.yaml maps/test_loop.bin

# Simulator
cargo run -p simulator -- --map maps/test_loop_map.yaml

# Visualiser (native)
cargo run -p visualiser

# Visualiser (WASM)
trunk serve targets/visualiser

# Bridge — hybrid
cargo run -p bridge -- \
    --map maps/test_loop_map.yaml \
    --sim-ws ws://localhost:3000 \
    --esp-udp 0.0.0.0:4242 \
    --sim-ids 0,1,2 --esp-ids 3,4

# ESP32
cargo +esp build -p esp32 --release --target riscv32imac-unknown-none-elf
espflash flash --monitor targets/esp32/target/riscv32imac-unknown-none-elf/release/esp32
```

---

## Key Design Properties

| Property | How achieved |
|---|---|
| **No goal factor windup** | Velocity prior replaces position goal. No time-indexed reference. No accumulated error. |
| **Natural waiting** | Robot held by inter-robot factor simply has `v_nom` as a soft pull. Releases cleanly when constraint lifts. |
| **Per-edge dynamics** | `v_nom`, `σ_dyn`, decel taper all from edge `speed_profile`. Ramps and straights differ automatically. |
| **Extensible factors** | Three marked extension points. Zero changes to graph iteration code. Full guide in `docs/adding_a_factor.md`. |
| **Exact factor gating** | Planned edge set intersection. No Euclidean heuristics. |
| **Distributed** | Each robot owns its GBP graph. No central solver. |
| **3D world, 1D problem** | 3D geometry absorbed into scalar Jacobians in agent layer. |
| **Hybrid mode** | Simulated and hardware robots coexist by `robot_id`. |
| **Progressive bring-up** | Migrate one robot at a time without disrupting fleet. |
| **Extensible UI** | bevy_egui plugin per panel. New panel = one `add_plugins()` call. |
| **Live defmt logs** | Full decode pipeline from ESP32 UART to visualiser log panel. |
| **Static allocation** | `no_std` + `heapless` throughout core crates. |

---

## Open Questions / Future Work

- **Arc-length table resolution**: `ARC_TABLE_SAMPLES = 64` — validate interpolation error on sharpest NURBS curves.
- **A\* on ESP32**: currently PC-side only. For on-device replanning, the map fits in RAM and the graph is small enough.
- **Blocked edge handling**: no mechanism yet for marking an edge unavailable and triggering replanning.
- **Hybrid time sync**: simulated and hardware robots run on different clocks. May need alignment for tight scenarios.
- **Robot ID assignment**: currently hardcoded per firmware build. NVS flash or bridge-assigned IDs at boot are future options.
- **`σ_dyn` per edge type**: ramps may warrant tighter dynamics than flat straights. Currently global.
- **Map live reload**: reload map without simulator restart. Bevy asset hot-reload may help.

---

## References

- Patwardhan et al. *Distributing Collaborative Multi-Robot Planning with GBP*. IEEE RA-L 2023. [arxiv:2203.11618](https://arxiv.org/abs/2203.11618)
- Ortiz et al. *A Visual Introduction to Gaussian Belief Propagation*. [gaussianbp.github.io](https://gaussianbp.github.io)
- esp-hal + esp-radio: [github.com/esp-rs/esp-hal](https://github.com/esp-rs/esp-hal)
- Embassy: [embassy.dev](https://embassy.dev)
- Bevy: [bevyengine.org](https://bevyengine.org)
- bevy_egui: [github.com/mvlabat/bevy_egui](https://github.com/mvlabat/bevy_egui)
- Trunk: [trunkrs.dev](https://trunkrs.dev)
- defmt: [defmt.ferrous-systems.com](https://defmt.ferrous-systems.com)
- postcard: [github.com/jamesmunns/postcard](https://github.com/jamesmunns/postcard)
- heapless: [github.com/rust-embedded/heapless](https://github.com/rust-embedded/heapless)
- The Rust on ESP Book: [docs.espressif.com/projects/rust](https://docs.espressif.com/projects/rust)
