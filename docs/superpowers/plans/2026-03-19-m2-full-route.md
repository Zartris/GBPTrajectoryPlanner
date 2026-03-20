# M2: One Robot, Full Route — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One robot navigates a full multi-edge route on `test_loop_map.yaml` using A* path planning, with trapezoidal deceleration to the goal, and the planned route shown as a dashed gizmo in the visualiser.

**Architecture:** `RobotAgent` (from `gbp-agent`) is wired into the simulator for the first time. The robot receives a `TrajectoryCommand` via WebSocket, runs A* to produce a `Trajectory`, then calls `agent.step()` each 20 Hz cycle — `DynamicsFactor` produces the correct `v_nom(s)` including the taper on the final edge. `SimComms` is a no-op stub (single robot; no inter-robot factors yet). Edge transitions are driven by the agent's returned `current_edge` — when it differs from the physics `current_edge`, physics calls `transition_to()`. This ensures the trajectory (not the map adjacency list) drives which edge comes next. `AgentRunner` holds `Arc<Map>` (not a raw reference) to make the lifetime self-enforcing across Tokio tasks. The `agent_task` populates `planned_edges` and `pos_3d` on each `RobotStateMsg` so the visualiser renders them correctly. The visualiser adds: NURBS edges rendered as sampled curves (32 points), dashed planned-path gizmo, and a minimal HUD showing current speed.

**Tech Stack:** Same as M1, plus `gbp-agent`, `gbp-map`(+parse). No new external dependencies.

**Prerequisite:** M1 plan complete and passing.

---

## File Structure

```
src/bins/simulator/
  src/
    main.rs            — updated: wire agent, handle TrajectoryCommand, edge transitions
    sim_comms.rs       — new: no-op SimComms (needed by RobotAgent)
    agent_runner.rs    — new: wraps RobotAgent step(), replaces broadcast_task
    physics.rs         — unchanged from M1

src/bins/visualiser/
  src/
    main.rs            — unchanged from M1
    map_scene.rs       — updated: NURBS curves rendered as 32-sample polylines
    robot_render.rs    — updated: dashed planned-path gizmo, multi-edge position
    ui.rs              — new: bevy_egui HUD with current speed readout
    ws_client.rs       — unchanged
    state.rs           — unchanged
```

---

## Chunk 1: Simulator — RobotAgent wired in

### Task 1: SimComms no-op stub

**Files:**
- Create: `src/bins/simulator/src/sim_comms.rs`

M2 now needs `RobotAgent`, which requires a `CommsInterface` implementation. In M2 (single robot) this is a no-op.

- [ ] **Step 1: Write SimComms**

```rust
// src/bins/simulator/src/sim_comms.rs
//! No-op CommsInterface for single-robot simulation.
//! In M3 this is replaced with a real in-process broadcast.

use gbp_comms::{CommsInterface, RobotBroadcast};
use heapless::Vec;

pub struct SimComms;

impl CommsInterface for SimComms {
    type Error = ();
    fn broadcast(&mut self, _: &RobotBroadcast) -> Result<(), ()> { Ok(()) }
    fn receive_broadcasts(&mut self) -> Vec<RobotBroadcast, 8>    { Vec::new() }
}
```

- [ ] **Step 1b: Write the failing test**

```rust
// Inline test in sim_comms.rs
#[cfg(test)]
mod tests {
    use super::*;
    use gbp_comms::CommsInterface;

    #[test]
    fn broadcast_returns_ok() {
        let mut c = SimComms;
        let dummy = gbp_comms::RobotBroadcast::default();
        assert!(c.broadcast(&dummy).is_ok());
    }

    #[test]
    fn receive_returns_empty() {
        let mut c = SimComms;
        assert_eq!(c.receive_broadcasts().len(), 0);
    }
}
```

- [ ] **Step 1c: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -10
```
Expected: compile error — `sim_comms` module not found.

- [ ] **Step 2: Implement sim_comms.rs** (code unchanged from above, add the `#[cfg(test)]` block at the end)

- [ ] **Step 3: Run tests** and confirm pass, then proceed to Cargo.toml change.

- [ ] **Step 4: Add `gbp-agent` to simulator Cargo.toml**

```toml
# In src/bins/simulator/Cargo.toml
[dependencies]
gbp-agent  = { path = "../../crates/gbp-agent" }
```

- [ ] **Step 5: Commit**

```bash
git add src/bins/simulator/src/sim_comms.rs src/bins/simulator/Cargo.toml
git commit -m "feat(m2): SimComms no-op stub, add gbp-agent to simulator"
```

---

### Task 2: AgentRunner — replaces broadcast_task, calls agent.step()

**Files:**
- Create: `src/bins/simulator/src/agent_runner.rs`

- [ ] **Step 1: Write the failing test**

```rust
// Inline test in agent_runner.rs
#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim_comms::SimComms;
    use gbp_map::map::*;

    fn straight_map() -> Map {
        let mut m = Map::new("s");
        m.add_node(Node { id:NodeId(0), position:[0.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
        m.add_node(Node { id:NodeId(1), position:[5.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
        m.add_edge(Edge {
            id:EdgeId(0), start:NodeId(0), end:NodeId(1),
            geometry: EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[5.0,0.0,0.0],length:5.0},
            speed: SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0},
            safety: SafetyProfile{clearance:0.3},
        }).unwrap();
        m
    }

    #[test]
    fn step_returns_positive_velocity_on_trajectory() {
        let map = std::sync::Arc::new(straight_map());
        let runner = AgentRunner::new(SimComms, map.clone(), 0);
        let mut runner = runner;
        runner.set_single_edge_trajectory(EdgeId(0), 5.0);
        let out = runner.step(0.0, EdgeId(0));
        assert!(out.velocity > 0.0, "v={}", out.velocity);
    }

    #[test]
    fn step_velocity_tapers_near_end_of_final_edge() {
        let map = std::sync::Arc::new(straight_map());
        let mut runner = AgentRunner::new(SimComms, map.clone(), 0);
        runner.set_single_edge_trajectory(EdgeId(0), 5.0);
        let v_start = runner.step(0.0, EdgeId(0)).velocity;
        let v_mid   = runner.step(2.5, EdgeId(0)).velocity;
        let v_near_end = runner.step(4.9, EdgeId(0)).velocity;
        // v at s=0 should be close to nominal (2.0 m/s)
        assert!(v_start > 1.5, "v_start={} expected ~2.0", v_start);
        // v near end should taper toward 0
        assert!(v_near_end < 0.8, "v_near_end={} expected ~0 (decel_limit=1.0, remaining=0.1m → sqrt(0.2)≈0.45)", v_near_end);
        assert!(v_near_end < v_mid, "expected monotone taper");
    }

    #[test]
    fn step_velocity_nominal_on_intermediate_edge_tapered_on_final() {
        // 2-edge trajectory; edge 0 is intermediate, edge 1 is final.
        let map = {
            let mut m = Map::new("two");
            m.add_node(Node { id:NodeId(0), position:[0.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_node(Node { id:NodeId(1), position:[5.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_node(Node { id:NodeId(2), position:[10.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
            let sf = SafetyProfile{clearance:0.3};
            m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
                geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[5.0,0.0,0.0],length:5.0},speed:sp.clone(),safety:sf.clone()}).unwrap();
            m.add_edge(Edge{id:EdgeId(1),start:NodeId(1),end:NodeId(2),
                geometry:EdgeGeometry::Line{start:[5.0,0.0,0.0],end:[10.0,0.0,0.0],length:5.0},speed:sp,safety:sf}).unwrap();
            std::sync::Arc::new(m)
        };
        let mut runner = AgentRunner::new(SimComms, map.clone(), 0);
        let mut traj: heapless::Vec<(EdgeId, f32), 32> = heapless::Vec::new();
        traj.push((EdgeId(0), 5.0)).unwrap();
        traj.push((EdgeId(1), 5.0)).unwrap();
        runner.set_trajectory_from_edges(traj, 0.0);

        // On intermediate edge 0 at mid-point: should be nominal (not tapered)
        let v_intermediate = runner.step(2.5, EdgeId(0)).velocity;
        assert!(v_intermediate > 1.8, "expected v_nom on intermediate edge, got {}", v_intermediate);

        // On final edge 1 near end: should be tapered
        let v_final_near_end = runner.step(4.9, EdgeId(1)).velocity;
        assert!(v_final_near_end < 0.8, "expected taper on final edge, got {}", v_final_near_end);
    }

    #[test]
    fn planned_edges_snapshot_returns_set_trajectory() {
        let map = std::sync::Arc::new(straight_map());
        let mut runner = AgentRunner::new(SimComms, map.clone(), 0);
        runner.set_single_edge_trajectory(EdgeId(0), 5.0);
        let snap = runner.planned_edges_snapshot();
        assert_eq!(snap.len(), 1);
        assert_eq!(snap[0], EdgeId(0));
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -10
```
Expected: compile error — `agent_runner` module not found.

- [ ] **Step 2: Implement `agent_runner.rs`**

```rust
// src/bins/simulator/src/agent_runner.rs
//! Wraps RobotAgent and drives its step() loop.
//! Replaces broadcast_task from M1.
//! Holds Arc<Map> to make the lifetime self-enforcing across Tokio tasks.

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use tokio::time::{interval, Duration};
use tracing::debug;
use gbp_agent::RobotAgent;
use gbp_comms::{ObservationUpdate, RobotStateMsg, RobotSource};
use gbp_map::{EdgeId, MAX_HORIZON, map::Map};
use heapless::Vec as HVec;
use crate::physics::PhysicsState;
use crate::sim_comms::SimComms;

pub struct AgentRunner {
    /// Holds the Arc so that RobotAgent's raw *const Map pointer remains valid
    /// for the lifetime of this struct.
    _map:  Arc<Map>,
    agent: RobotAgent<SimComms>,
    /// Mirrors the full trajectory edge list set via set_trajectory_from_edges /
    /// set_single_edge_trajectory so we can broadcast planned_edges without
    /// requiring RobotAgent to expose its internal trajectory (that exposure
    /// belongs to M3 when RobotBroadcast is wired).
    trajectory_edges: HVec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
}

impl AgentRunner {
    /// `map` must be kept alive for as long as the AgentRunner — holding the Arc satisfies this.
    pub fn new(comms: SimComms, map: Arc<Map>, robot_id: u32) -> Self {
        let agent = RobotAgent::new(comms, &*map, robot_id);
        Self { _map: map, agent, trajectory_edges: HVec::new() }
    }

    pub fn set_single_edge_trajectory(&mut self, edge_id: EdgeId, edge_length: f32) {
        let mut edges: HVec<(EdgeId, f32), 32> = HVec::new();
        let _ = edges.push((edge_id, edge_length));
        self.trajectory_edges.clear();
        let _ = self.trajectory_edges.push(edge_id);
        self.agent.set_trajectory(edges, 0.0);
    }

    pub fn set_trajectory_from_edges(&mut self, edges: HVec<(EdgeId, f32), 32>, start_s: f32) {
        self.trajectory_edges.clear();
        for (eid, _) in &edges {
            let _ = self.trajectory_edges.push(*eid);
        }
        self.agent.set_trajectory(edges, start_s);
    }

    /// Single step. Returns velocity, agent's authoritative current_edge, and 3D world position.
    /// The caller should call `physics.transition_to()` if `out.current_edge != physics.current_edge`.
    pub fn step(&mut self, position_s: f32, current_edge: EdgeId) -> StepOut {
        let obs = ObservationUpdate { position_s, velocity: 0.0, current_edge };
        let out = self.agent.step(obs);
        // Evaluate 3D world position (Bevy Y-up: map(x,y,z) → bevy(x,z,-y))
        let p = self._map.eval_position(out.current_edge, out.position_s);
        let pos_3d = [p[0], p[2], -p[1]];
        StepOut {
            velocity:     out.velocity,
            position_s:   out.position_s,
            current_edge: out.current_edge,
            pos_3d,
        }
    }

    /// Returns the agent's planned edge sequence for broadcast in RobotStateMsg.
    /// Returns all edges set via set_trajectory_from_edges, mirrored locally.
    /// In M3 this is superseded by the full RobotBroadcast from agent.make_broadcast().
    pub fn planned_edges_snapshot(&self) -> HVec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
        self.trajectory_edges.clone()
    }
}

pub struct StepOut {
    pub velocity:     f32,
    pub position_s:   f32,
    pub current_edge: EdgeId,
    pub pos_3d:       [f32; 3],
}

/// Runs at 20 Hz: steps the agent, drives edge transitions, broadcasts RobotStateMsg.
/// Edge transitions are driven by the agent's returned current_edge (not map adjacency).
pub async fn agent_task(
    physics: Arc<Mutex<PhysicsState>>,
    runner:  Arc<Mutex<AgentRunner>>,
    map:     Arc<Map>, // needed only to get edge length on transition
    tx:      broadcast::Sender<RobotStateMsg>,
) {
    let mut ticker = interval(Duration::from_millis(50)); // 20 Hz
    loop {
        ticker.tick().await;
        let (pos_s, phys_edge) = {
            let p = physics.lock().unwrap();
            (p.position_s, p.current_edge)
        };

        let out = { runner.lock().unwrap().step(pos_s, phys_edge) };

        // Drive edge transitions via agent's authoritative current_edge
        if out.current_edge != phys_edge {
            let new_len = map.edges.iter()
                .find(|e| e.id == out.current_edge)
                .map(|e| e.geometry.length())
                .unwrap_or(1.0);
            let mut p = physics.lock().unwrap();
            p.transition_to(out.current_edge, new_len);
        }
        { physics.lock().unwrap().velocity = out.velocity; }

        let msg = RobotStateMsg {
            robot_id: 0, current_edge: out.current_edge,
            position_s: out.position_s, velocity: out.velocity,
            pos_3d: out.pos_3d,
            source: RobotSource::Simulated,
            belief_means: [0.0; MAX_HORIZON], belief_vars: [0.0; MAX_HORIZON],
            planned_edges: { runner.lock().unwrap().planned_edges_snapshot() },
            active_factors: HVec::new(),
        };
        debug!("agent: s={:.3} v={:.3} edge={:?}", out.position_s, out.velocity, out.current_edge);
        let _ = tx.send(msg);
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```
Expected: all tests pass including the taper test.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/agent_runner.rs
git commit -m "feat(m2): AgentRunner wraps RobotAgent, drives step() at 20 Hz"
```

---

### Task 3: Edge transitions and PhysicsState update

**Files:**
- Modify: `src/bins/simulator/src/physics.rs`

M2 needs `PhysicsState` to track `current_edge` and trigger a transition when `position_s >= edge_length`.

- [ ] **Step 1: Write the failing tests**

```rust
// Add to physics.rs tests module
#[test]
fn physics_triggers_edge_transition_when_at_end() {
    use gbp_map::map::EdgeId;
    let mut state = PhysicsState {
        position_s: 9.95, velocity: 2.0,
        edge_length: 10.0, current_edge: EdgeId(0),
        edge_done: false,
    };
    state.step(0.02);
    // 9.95 + 2.0 * 0.02 = 9.99 — not yet at 10.0
    assert!(!state.edge_done);
    state.step(0.02);
    // 9.99 + 2.0 * 0.02 = 10.03 → clamped to 10.0 → edge_done = true
    assert!(state.edge_done);
}

#[test]
fn physics_resets_edge_done_on_new_edge() {
    use gbp_map::map::EdgeId;
    let mut state = PhysicsState {
        position_s: 10.0, velocity: 0.0,
        edge_length: 10.0, current_edge: EdgeId(0),
        edge_done: true,
    };
    state.transition_to(EdgeId(1), 8.0);
    assert_eq!(state.current_edge, EdgeId(1));
    assert_eq!(state.edge_length, 8.0);
    assert!((state.position_s).abs() < 1e-6);
    assert!(!state.edge_done);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -10
```
Expected: compile error — `edge_done` and `current_edge` fields don't exist yet.

- [ ] **Step 2: Update `physics.rs`**

```rust
// src/bins/simulator/src/physics.rs
use gbp_map::map::EdgeId;
use std::sync::{Arc, Mutex};
use tokio::time::{interval, Duration};
use tracing::debug;

#[derive(Clone, Debug)]
pub struct PhysicsState {
    pub position_s:   f32,
    pub velocity:     f32,
    pub edge_length:  f32,
    pub current_edge: EdgeId,
    /// Set to true when position_s reaches edge_length; cleared on transition_to().
    pub edge_done:    bool,
}

impl PhysicsState {
    pub fn new(edge_id: EdgeId, edge_length: f32) -> Self {
        Self { position_s: 0.0, velocity: 0.0, edge_length, current_edge: edge_id, edge_done: false }
    }

    pub fn step(&mut self, dt: f32) {
        let new_s = self.position_s + self.velocity * dt;
        if new_s >= self.edge_length {
            self.position_s = self.edge_length;
            self.edge_done = true;
        } else {
            self.position_s = new_s.max(0.0);
        }
    }

    /// Advance to the next edge, resetting position to 0.
    pub fn transition_to(&mut self, next_edge: EdgeId, next_length: f32) {
        self.current_edge = next_edge;
        self.edge_length  = next_length;
        self.position_s   = 0.0;
        self.edge_done    = false;
    }
}

pub async fn physics_task(state: Arc<Mutex<PhysicsState>>) {
    const DT: f32 = 1.0 / 50.0;
    let mut ticker = interval(Duration::from_millis(20));
    loop {
        ticker.tick().await;
        let mut s = state.lock().unwrap();
        s.step(DT);
        debug!("physics: s={:.4} edge={:?}", s.position_s, s.current_edge);
    }
}
```

> **Note:** The M1 unit tests for `PhysicsState` used `PhysicsState { position_s, velocity, edge_length }`. They must be updated to include `current_edge: EdgeId(0), edge_done: false` now that those fields exist.

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```
Expected: all tests pass (M1 tests updated to new struct shape).

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/physics.rs
git commit -m "feat(m2): PhysicsState edge transitions — current_edge, edge_done"
```

---

### Task 4: A* + TrajectoryCommand handling in main

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
// Inline test in main.rs (or a dedicated route_planner.rs if preferred)
// Test the route-building helper used in main.

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

    fn two_edge_map() -> Map {
        let mut m = Map::new("two");
        m.add_node(Node { id:NodeId(0), position:[0.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
        m.add_node(Node { id:NodeId(1), position:[3.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
        m.add_node(Node { id:NodeId(2), position:[6.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
        let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
        let sf = SafetyProfile{clearance:0.3};
        m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
            geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[3.0,0.0,0.0],length:3.0},
            speed:sp.clone(),safety:sf.clone()}).unwrap();
        m.add_edge(Edge{id:EdgeId(1),start:NodeId(1),end:NodeId(2),
            geometry:EdgeGeometry::Line{start:[3.0,0.0,0.0],end:[6.0,0.0,0.0],length:3.0},
            speed:sp,safety:sf}).unwrap();
        m
    }

    #[test]
    fn build_trajectory_from_astar_path() {
        let map = two_edge_map();
        let path = gbp_map::astar::astar(&map, NodeId(0), NodeId(2)).unwrap();
        let traj = build_trajectory_edges(&map, &path);
        assert_eq!(traj.len(), 2);
        assert!((traj[0].1 - 3.0).abs() < 1e-5, "edge0 length={}", traj[0].1);
        assert!((traj[1].1 - 3.0).abs() < 1e-5, "edge1 length={}", traj[1].1);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -10
```

- [ ] **Step 2: Implement `build_trajectory_edges` helper and update `main.rs`**

Add to `main.rs`:
```rust
/// Convert A* edge path to (EdgeId, length) pairs for set_trajectory_from_edges.
pub fn build_trajectory_edges(
    map: &gbp_map::map::Map,
    path: &heapless::Vec<gbp_map::map::EdgeId, { gbp_map::MAX_PATH_EDGES }>,
) -> heapless::Vec<(gbp_map::map::EdgeId, f32), 32> {
    let mut result: heapless::Vec<(gbp_map::map::EdgeId, f32), 32> = heapless::Vec::new();
    for &edge_id in path.iter() {
        if let Some(idx) = map.edge_index(edge_id) {
            let length = map.edges[idx].geometry.length();
            let _ = result.push((edge_id, length));
        }
    }
    result
}
```

Update `main.rs` to:
- Accept `TrajectoryCommand` from WebSocket clients (parse JSON, extract `goal_node`)
- Run A* from `current_node` to `goal_node`
- Call `runner.set_trajectory_from_edges(edges, current_s)`
- Handle edge transition: in the 50 Hz loop check `physics.edge_done`; if true, advance `current_edge` in a trajectory iterator

Add a `TrajectoryCommand` receiver to the WebSocket handler:

```rust
// In ws_server.rs — extend handle_socket to also handle inbound messages
async fn handle_socket(mut socket: WebSocket, tx: WsTx, cmd_tx: mpsc::Sender<String>) {
    let mut rx = tx.subscribe();
    loop {
        tokio::select! {
            Ok(json) = rx.recv() => {
                if socket.send(Message::Text(json)).await.is_err() { break; }
            }
            Some(Ok(Message::Text(cmd))) = socket.recv() => {
                let _ = cmd_tx.send(cmd).await;
            }
            else => break,
        }
    }
}
```

And add to `main.rs` a command handler task that reads from `cmd_rx`, parses `TrajectoryCommand`, runs A*, and updates the runner's trajectory.

**Exact main.rs rewrite** (replace full file):

```rust
// src/bins/simulator/src/main.rs
mod physics;
mod sim_comms;
mod agent_runner;
mod ws_server;

use std::sync::{Arc, Mutex};
use tokio::sync::{broadcast, mpsc};
use tracing::info;
use tracing_subscriber::EnvFilter;
use gbp_comms::{RobotStateMsg, TrajectoryCommand};
use gbp_map::{astar::astar, map::{EdgeId, Map, NodeId}};
use heapless::Vec as HVec;
use physics::PhysicsState;
use sim_comms::SimComms;
use agent_runner::{AgentRunner, agent_task};

struct Args { map_path: String, bind_addr: String }
fn parse_args() -> Args {
    let mut a = std::env::args().skip(1);
    Args {
        map_path: a.next().unwrap_or_else(|| "maps/test_loop_map.yaml".into()),
        bind_addr: a.next().unwrap_or_else(|| "0.0.0.0:3000".into()),
    }
}

pub fn build_trajectory_edges(
    map: &Map,
    path: &heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
) -> HVec<(EdgeId, f32), 32> {
    let mut r: HVec<(EdgeId, f32), 32> = HVec::new();
    for &eid in path { if let Some(i) = map.edge_index(eid) { let _ = r.push((eid, map.edges[i].geometry.length())); } }
    r
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("simulator=debug".parse().unwrap()))
        .init();
    let args = parse_args();
    let yaml = std::fs::read_to_string(&args.map_path).unwrap_or_else(|e| panic!("{}", e));
    let map  = gbp_map::parser::parse_yaml(&yaml).unwrap_or_else(|e| panic!("{}", e));

    // M2: start on first edge, goal = last node in map
    let first_edge  = map.edges.first().expect("no edges");
    let start_node  = first_edge.start;
    let goal_node   = map.nodes.last().expect("no nodes").id;
    let edge_id     = first_edge.id;
    let edge_length = first_edge.geometry.length();

    let map_arc = Arc::new(map);
    let physics = Arc::new(Mutex::new(PhysicsState::new(edge_id, edge_length)));

    let runner = {
        let mut r = AgentRunner::new(SimComms, map_arc.clone(), 0);
        // Initial A* from start to goal
        if let Some(path) = astar(&map_arc, start_node, goal_node) {
            let traj = build_trajectory_edges(&map_arc, &path);
            r.set_trajectory_from_edges(traj, 0.0);
        } else {
            r.set_single_edge_trajectory(edge_id, edge_length);
        }
        Arc::new(Mutex::new(r))
    };

    let (tx_state, _): (broadcast::Sender<RobotStateMsg>, _) = broadcast::channel(16);
    let (tx_json,  _): (broadcast::Sender<String>, _)        = broadcast::channel(16);
    let (cmd_tx, mut cmd_rx): (mpsc::Sender<String>, _)      = mpsc::channel(8);

    // JSON relay
    let tx_json_relay = tx_json.clone();
    let mut rx_relay  = tx_state.subscribe();
    tokio::spawn(async move {
        loop {
            if let Ok(msg) = rx_relay.recv().await {
                if let Ok(json) = serde_json::to_string(&msg) { let _ = tx_json_relay.send(json); }
            }
        }
    });

    // Command handler: parse TrajectoryCommand, replan
    let map_cmd   = Arc::clone(&map_arc);
    let runner_cmd = Arc::clone(&runner);
    let phys_cmd   = Arc::clone(&physics);
    tokio::spawn(async move {
        while let Some(json) = cmd_rx.recv().await {
            if let Ok(cmd) = serde_json::from_str::<TrajectoryCommand>(&json) {
                let current_s = phys_cmd.lock().unwrap().position_s;
                let cur_edge  = phys_cmd.lock().unwrap().current_edge;
                // Find current node (start of current edge)
                let from_node = map_cmd.edges.iter()
                    .find(|e| e.id == cur_edge).map(|e| e.start)
                    .unwrap_or(NodeId(0));
                if let Some(path) = astar(&map_cmd, from_node, cmd.goal_node) {
                    let traj = build_trajectory_edges(&map_cmd, &path);
                    runner_cmd.lock().unwrap().set_trajectory_from_edges(traj, current_s);
                    info!("replanned to {:?}", cmd.goal_node);
                }
            }
        }
    });

    // Edge transitions are driven by agent_task — no separate watcher needed.
    // When agent.step() returns a different current_edge, agent_task calls transition_to().
    tokio::spawn(physics::physics_task(Arc::clone(&physics)));
    tokio::spawn(agent_task(Arc::clone(&physics), runner, Arc::clone(&map_arc), tx_state));

    let router   = ws_server::build_router(tx_json, cmd_tx);
    let listener = tokio::net::TcpListener::bind(&args.bind_addr).await.unwrap();
    info!("listening on ws://{}/ws", args.bind_addr);
    axum::serve(listener, router).await.unwrap();
}
```

Also update `ws_server.rs` to accept a `cmd_tx: mpsc::Sender<String>` parameter and forward inbound WebSocket text messages to it. Update `build_router` signature accordingly.

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/main.rs src/bins/simulator/src/ws_server.rs
git commit -m "feat(m2): A* route planning, TrajectoryCommand handling, edge transitions"
```

---

## Chunk 2: Visualiser — NURBS curves + planned-path gizmo + speed HUD

### Task 5: NURBS edge rendering (already partial in M1 — now ensure 32-sample polylines)

**Files:**
- Modify: `src/bins/visualiser/src/map_scene.rs`

M1 already renders NURBS via gizmos with 32 samples. Verify the constant is named and matches the spec:

- [ ] **Step 1: Write the failing test**

```rust
// In map_scene.rs tests
#[test]
fn nurbs_sample_count_is_32() {
    assert_eq!(NURBS_EDGE_SAMPLES, 32);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib 2>&1 | head -5
```
Expected: `NURBS_EDGE_SAMPLES` not defined.

- [ ] **Step 2: Add the constant to `map_scene.rs`**

```rust
/// Number of line segments used to render each NURBS edge.
pub const NURBS_EDGE_SAMPLES: usize = 32;
```

Replace the inline `32` literal in `draw_edge_gizmos` with `NURBS_EDGE_SAMPLES`.

- [ ] **Step 3: Run test**

```bash
cargo test -p visualiser --lib 2>&1 | grep "nurbs_sample"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/map_scene.rs
git commit -m "feat(m2): name NURBS_EDGE_SAMPLES=32 constant in map_scene"
```

---

### Task 6: Planned-path dashed gizmo

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In robot_render.rs tests
#[test]
fn planned_edge_positions_for_two_edges() {
    use gbp_map::map::*;
    let mut m = Map::new("t");
    m.add_node(Node{id:NodeId(0),position:[0.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    m.add_node(Node{id:NodeId(1),position:[3.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    m.add_node(Node{id:NodeId(2),position:[6.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
    let sf = SafetyProfile{clearance:0.3};
    m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
        geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[3.0,0.0,0.0],length:3.0},speed:sp.clone(),safety:sf.clone()}).unwrap();
    m.add_edge(Edge{id:EdgeId(1),start:NodeId(1),end:NodeId(2),
        geometry:EdgeGeometry::Line{start:[3.0,0.0,0.0],end:[6.0,0.0,0.0],length:3.0},speed:sp,safety:sf}).unwrap();

    // edge 0 start at s=1.5
    let pts = edge_sample_points(&m, EdgeId(0), 4);
    assert_eq!(pts.len(), 4);
    assert!((pts[0][0] - 0.0).abs() < 1e-4);
    assert!((pts[3][0] - 3.0).abs() < 1e-4);
}

#[test]
fn coordinate_transform_swaps_map_y_and_z() {
    // Edge along map-Y axis with map-Z elevation: Map(0,2,1) → Bevy(0,1,-2)
    use gbp_map::map::*;
    let mut m = Map::new("t");
    m.add_node(Node{id:NodeId(0),position:[0.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    m.add_node(Node{id:NodeId(1),position:[0.0,2.0,1.0],node_type:NodeType::Waypoint}).unwrap();
    let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
    let sf = SafetyProfile{clearance:0.3};
    let len = (4.0_f32 + 1.0_f32).sqrt();
    m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
        geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[0.0,2.0,1.0],length:len},
        speed:sp,safety:sf}).unwrap();

    let pts = edge_sample_points(&m, EdgeId(0), 2);
    // pts[1] = end point: map (x=0,y=2,z=1) → bevy (x=0, y=map_z=1, z=-map_y=-2)
    assert!((pts[1][1] - 1.0).abs() < 1e-3, "bevy_y(=map_z) expected 1.0, got {}", pts[1][1]);
    assert!((pts[1][2] - (-2.0)).abs() < 1e-3, "bevy_z(=-map_y) expected -2.0, got {}", pts[1][2]);
}

#[test]
fn planned_path_gizmo_samples_is_8() {
    assert_eq!(PLANNED_PATH_GIZMO_SAMPLES, 8);
}

#[test]
fn dashed_segment_pairs_skips_even_indices() {
    // dashed_segment_pairs draws segments at odd loop indices (i=1,3,5,...),
    // skipping even loop indices (i=2,4,...). For 4 points: draws (pts[0],pts[1]) and (pts[2],pts[3]).
    let pts: &[[f32; 3]] = &[
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
    ];
    let pairs = dashed_segment_pairs(pts);
    // segments at i=1 and i=3 (indices 1,3 are odd)
    assert_eq!(pairs.len(), 2);
    assert_eq!(pairs[0], ([0.0,0.0,0.0],[1.0,0.0,0.0]));
    assert_eq!(pairs[1], ([2.0,0.0,0.0],[3.0,0.0,0.0]));
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib 2>&1 | head -10
```

- [ ] **Step 2: Implement `edge_sample_points` and dashed gizmo system**

Add to `robot_render.rs`:

```rust
/// Sample N evenly-spaced world-space points along an edge (for dashed gizmo).
pub fn edge_sample_points(map: &Map, edge_id: EdgeId, n: usize) -> heapless::Vec<[f32; 3], 64> {
    let mut pts: heapless::Vec<[f32; 3], 64> = heapless::Vec::new();
    if let Some(idx) = map.edge_index(edge_id) {
        let len = map.edges[idx].geometry.length();
        for i in 0..n {
            let s = if n > 1 { (i as f32 / (n - 1) as f32) * len } else { 0.0 };
            let p = map.eval_position(edge_id, s);
            let _ = pts.push([p[0], p[2], -p[1]]); // map→Bevy coord
        }
    }
    pts
}

/// Return (start, end) pairs for every other segment (dashed effect).
/// Draws segments at indices i=1,3,5,... (odd i means the segment from pts[i-1] to pts[i]).
pub fn dashed_segment_pairs(pts: &[[f32; 3]]) -> heapless::Vec<([f32;3],[f32;3]), 32> {
    let mut pairs = heapless::Vec::new();
    for i in 1..pts.len() {
        if i % 2 == 1 {
            let _ = pairs.push((pts[i-1], pts[i]));
        }
    }
    pairs
}

// PLANNED_PATH_GIZMO_SAMPLES uses 8 (not NURBS_EDGE_SAMPLES=32) to produce
// visible gaps between dashes rather than a near-continuous line.
const PLANNED_PATH_GIZMO_SAMPLES: usize = 8;

fn draw_planned_path(
    map:    Res<MapRes>,
    states: Res<RobotStates>,
    mut gizmos: Gizmos,
) {
    for state in states.0.values() {
        let color = Color::srgba(0.3, 0.8, 1.0, 0.6);
        for &edge_id in state.planned_edges.iter() {
            let pts = edge_sample_points(&map.0, edge_id, PLANNED_PATH_GIZMO_SAMPLES);
            for (a, b) in dashed_segment_pairs(&pts) {
                gizmos.line(Vec3::from(a), Vec3::from(b), color);
            }
        }
    }
}
```

Register in `RobotRenderPlugin::build`:
```rust
app.add_systems(Update, (drain_ws_inbox, update_robot_transforms, draw_planned_path).chain());
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/robot_render.rs
git commit -m "feat(m2): dashed planned-path gizmo in visualiser"
```

---

### Task 7: Minimal HUD — play/pause + current speed readout

**Files:**
- Create: `src/bins/visualiser/src/ui.rs`
- Modify: `src/bins/visualiser/src/main.rs` (register plugin + resource)

- [ ] **Step 1: Write the failing tests**

```rust
// In ui.rs
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_velocity_string_two_decimal() {
        assert_eq!(fmt_velocity(2.0), "2.00 m/s");
        assert_eq!(fmt_velocity(0.0), "0.00 m/s");
    }

    #[test]
    fn sim_paused_default_is_false() {
        let p = SimPaused::default();
        assert!(!p.0);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib 2>&1 | head -10
```

- [ ] **Step 2: Implement `ui.rs`**

```rust
// src/bins/visualiser/src/ui.rs
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use crate::state::RobotStates;

/// Shared pause flag. The visualiser sets this; simulator reads it via the
/// WebSocket control channel (see ws_server.rs — "pause"/"resume" messages).
#[derive(Resource, Default)]
pub struct SimPaused(pub bool);

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimPaused>()
           .add_systems(Update, draw_hud);
    }
}

pub fn fmt_velocity(v: f32) -> String {
    format!("{:.2} m/s", v)
}

fn draw_hud(
    mut ctxs: EguiContexts,
    states:   Res<RobotStates>,
    mut paused: ResMut<SimPaused>,
) {
    let ctx = ctxs.ctx_mut();
    egui::Window::new("Control").show(ctx, |ui| {
        let label = if paused.0 { "▶ Resume" } else { "⏸ Pause" };
        if ui.button(label).clicked() {
            paused.0 = !paused.0;
            // TODO(M6): send "pause"/"resume" command to simulator via WebSocket
        }
        ui.separator();
        for (id, state) in &states.0 {
            ui.label(format!("Robot {}: {}", id, fmt_velocity(state.velocity)));
        }
        if states.0.is_empty() {
            ui.label("No robots connected");
        }
    });
}
```

Register in `main.rs`:
```rust
.add_plugins(UiPlugin)
```
And add `mod ui; use ui::UiPlugin;` to `main.rs`.

Note: The pause button toggles the local `SimPaused` resource in M2. The WebSocket control channel to the simulator is deferred to M6 (robustness milestone), which introduces `ParameterUpdate` handling. The button is visible and toggles state in M2; actual simulation pause is an M6 feature.

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/ui.rs src/bins/visualiser/src/main.rs
git commit -m "feat(m2): minimal HUD with play/pause toggle and speed readout"
```

---

### Task 8: End-to-end verification

- [ ] **Step 1: Run all unit tests**

```bash
cargo test -p simulator -p visualiser 2>&1 | grep -E "test |FAILED|passed|error"
```
Expected: all tests pass.

- [ ] **Step 2: Manual visual verification**

```bash
# Terminal 1
RUST_LOG=simulator=info cargo run -p simulator -- maps/test_loop_map.yaml

# Terminal 2
MAP_PATH=maps/test_loop_map.yaml cargo run -p visualiser
```

Expected:
- Robot arrow moves along the first edge, then transitions to subsequent edges
- Arrow decelerates visibly as it approaches the goal node (trapezoidal taper)
- Dashed cyan line traces planned edges ahead of the robot
- Speed readout in the HUD decreases to 0 m/s at the goal
- `test_loop_map.yaml` includes NURBS edges — these are rendered as sampled polylines

- [ ] **Step 3: Commit and tag M2**

```bash
git add -A
git commit -m "feat(m2): full-route robot — A*, edge transitions, taper, NURBS, dashed path"
```

---

## Summary

| Chunk | Deliverables | Verification |
|---|---|---|
| 1 | SimComms, AgentRunner (RobotAgent::step()), edge transitions, A*, TrajectoryCommand | Unit tests: taper, A* path, edge transitions |
| 2 | NURBS_EDGE_SAMPLES constant, dashed planned-path gizmo, speed HUD | Visual: robot decelerates + dashed path shown |

## Deferred from M1 PR review (must address in M2)

- [x] **WS client thread lifecycle**: `spawn_ws_client()` returns `Arc<AtomicBool>` shutdown flag checked in the WS reconnect loop; set after `App::run()` returns so the process doesn't hang on shutdown.
- [ ] **`RobotAgent` raw pointer → `Arc<Map>`**: M2 introduces `AgentRunner` which holds `Arc<Map>` (per plan line 7). Verify the raw `*const Map` in `robot_agent.rs` is no longer used in the simulator path. (Full fix deferred to M7 for firmware `&'static Map`.)

**What M3 adds:** `SimComms` becomes a real in-process broadcast, `update_interrobot_factors()` is implemented, GBP negotiation between two robots, belief tubes and factor lines in the visualiser.
