# M3: Two Robots, GBP Rear-End Avoidance — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Two robots on the same edge negotiate via GBP. Robot B (behind) slows to maintain `d_safe` clearance from Robot A (ahead). GBP `iterate()` runs every step. Belief tubes and inter-robot factor lines visible in the visualiser.

**Architecture:** `SimComms` becomes a real in-process broadcast channel — every agent's `RobotBroadcast` is forwarded to all other agents via a `tokio::sync::broadcast` channel. `RobotAgent::update_interrobot_factors()` spawns/removes `InterRobotFactor` entries based on whether robots share a planned edge. GBP `iterate()` runs N=20 iterations per 20 Hz step. Two `AgentRunner` instances run in the same Tokio runtime. The visualiser adds: belief tubes (K arc-length means ± variance tube), factor-connection gizmo lines, and a per-robot egui side panel.

**Tech Stack:** No new external dependencies beyond M2. Uses `tokio::sync::broadcast` (already in scope).

**Prerequisite:** M2 plan complete and passing.

---

## File Structure

```
src/bins/simulator/
  src/
    main.rs            — updated: two agents, SimComms wired, shared broadcast channel
    sim_comms.rs       — updated: real broadcast channel (was no-op stub in M2)
    agent_runner.rs    — updated: update_interrobot_factors(), iterate() call

src/bins/visualiser/
  src/
    main.rs            — unchanged
    state.rs           — updated: add belief_means, belief_vars, active_factors fields
    robot_render.rs    — updated: belief tube gizmos, factor-link gizmo lines
    ui.rs              — updated: per-robot side panel (velocity, σ_dyn, σ_r, factor count)
    ws_client.rs       — unchanged
    map_scene.rs       — unchanged
```

---

## Chunk 1: Real SimComms + two-agent simulator

### Task 1: Upgrade SimComms to real broadcast channel

**Files:**
- Modify: `src/bins/simulator/src/sim_comms.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In sim_comms.rs tests
#[cfg(test)]
mod tests {
    use super::*;
    use gbp_comms::RobotBroadcast;

    #[test]
    fn broadcast_is_received_by_second_receiver() {
        // SimComms is constructed with a sender and a receiver for incoming messages.
        // A broadcast sent on one sender should arrive on a second receiver subscribed
        // to the same channel.
        let (tx, _rx) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
        let rx2 = tx.subscribe();
        let comms = SimComms::new(tx.clone(), rx2);
        let msg = RobotBroadcast {
            robot_id: 1,
            current_edge: gbp_map::EdgeId(0),
            position_s: 1.5,
            planned_edges: heapless::Vec::new(),
        };
        // send via the channel — comms.broadcast() should put msg on tx
        comms.broadcast(&msg).unwrap();
        // a third subscriber can receive it
        let mut rx3 = tx.subscribe();
        // send again so rx3 sees it
        comms.broadcast(&msg).unwrap();
        let received = rx3.try_recv().unwrap();
        assert_eq!(received.robot_id, 1);
    }

    #[test]
    fn receive_returns_broadcasts_from_others() {
        let (tx, rx) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
        let mut comms = SimComms::new(tx.clone(), rx);
        let msg = RobotBroadcast {
            robot_id: 2,
            current_edge: gbp_map::EdgeId(0),
            position_s: 3.0,
            planned_edges: heapless::Vec::new(),
        };
        tx.send(msg.clone()).unwrap();
        let received = comms.receive_broadcasts();
        assert!(received.iter().any(|b| b.robot_id == 2));
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib sim_comms 2>&1 | head -20
```
Expected: compile error or test failure — `SimComms::new` with channel args doesn't exist yet.

- [ ] **Step 2: Implement real `SimComms`**

Replace the M2 no-op stub in `sim_comms.rs`:

```rust
// src/bins/simulator/src/sim_comms.rs
use gbp_comms::{CommsInterface, RobotBroadcast};
use heapless::Vec as HVec;
use tokio::sync::broadcast;
use gbp_map::MAX_NEIGHBOURS;

/// In-process broadcast. One instance per agent.
/// All agents share the same broadcast::Sender<RobotBroadcast>.
/// Each agent holds a private broadcast::Receiver to drain incoming.
pub struct SimComms {
    tx: broadcast::Sender<RobotBroadcast>,
    rx: broadcast::Receiver<RobotBroadcast>,
}

impl SimComms {
    pub fn new(tx: broadcast::Sender<RobotBroadcast>, rx: broadcast::Receiver<RobotBroadcast>) -> Self {
        Self { tx, rx }
    }
}

impl CommsInterface for SimComms {
    type Error = ();

    fn broadcast(&mut self, msg: &RobotBroadcast) -> Result<(), Self::Error> {
        self.tx.send(msg.clone()).map(|_| ()).map_err(|_| ())
    }

    fn receive_broadcasts(&mut self) -> HVec<RobotBroadcast, MAX_NEIGHBOURS> {
        let mut out = HVec::new();
        loop {
            match self.rx.try_recv() {
                Ok(msg) => { let _ = out.push(msg); }
                Err(_) => break,
            }
        }
        out
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator --lib sim_comms 2>&1 | grep -E "test |FAILED|passed"
```
Expected: `broadcast_is_received_by_second_receiver` and `receive_returns_broadcasts_from_others` pass.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/sim_comms.rs
git commit -m "feat(m3): SimComms real broadcast channel"
```

---

### Task 2: Two-agent simulator main.rs

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Write the failing test**

Add to the `tests` block already in `sim_comms.rs`:

```rust
// In sim_comms.rs tests — add alongside existing tests
#[test]
fn two_simcomms_share_one_channel() {
    // Two SimComms can be constructed from the same Sender without panic.
    // Cross-agent messaging correctness is covered by Task 1 tests and the
    // Task 4 integration test.
    let (tx, rx0) = tokio::sync::broadcast::channel::<RobotBroadcast>(64);
    let rx1 = tx.subscribe();
    let _comms0 = SimComms::new(tx.clone(), rx0);
    let _comms1 = SimComms::new(tx.clone(), rx1);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib sim_comms 2>&1 | head -20
```
Expected: compile error if `SimComms::new` is not yet updated to handle two instances, or passes if Task 1 is already done (in which case Step 2 below is still needed to wire two agents into `main.rs`).

- [ ] **Step 2: Update `main.rs` to spawn two agents**

Update the simulator `main()` to create two `AgentRunner` instances (robots 0 and 1) on a shared broadcast channel and spawn two `agent_task` instances:

```rust
// In main.rs — spawn two agents sharing one broadcast channel
use tokio::sync::broadcast;
use gbp_comms::RobotBroadcast;

// Create shared broadcast channel for inter-robot messages
let (bcast_tx, bcast_rx0) = broadcast::channel::<RobotBroadcast>(64);
let bcast_rx1 = bcast_tx.subscribe();

// Robot 0
let comms0 = SimComms::new(bcast_tx.clone(), bcast_rx0);
let runner0 = Arc::new(Mutex::new(AgentRunner::new(comms0, map_arc.clone(), 0)));
// Place robot 0 at the start of edge 0
let physics0 = Arc::new(Mutex::new(PhysicsState::new(start_edge_id, start_edge_len)));
if let Some(path) = astar(&map_arc, start_node, goal_node_0) {
    let traj = build_trajectory_edges(&map_arc, &path);
    runner0.lock().unwrap().set_trajectory_from_edges(traj, 0.0);
}

// Robot 1 — starts 1.0 m behind robot 0 on the same edge
let comms1 = SimComms::new(bcast_tx.clone(), bcast_rx1);
let runner1 = Arc::new(Mutex::new(AgentRunner::new(comms1, map_arc.clone(), 1)));
let physics1 = Arc::new(Mutex::new(PhysicsState::new(start_edge_id, start_edge_len)));
physics1.lock().unwrap().position_s = 1.0; // behind robot 0 by 1 m at start
if let Some(path) = astar(&map_arc, start_node, goal_node_1) {
    let traj = build_trajectory_edges(&map_arc, &path);
    runner1.lock().unwrap().set_trajectory_from_edges(traj, 1.0);
}

// Spawn both agent_task loops
tokio::spawn(agent_task(physics0.clone(), runner0.clone(), map_arc.clone(), ws_tx.clone(), 0));
tokio::spawn(agent_task(physics1.clone(), runner1.clone(), map_arc.clone(), ws_tx.clone(), 1));
tokio::spawn(physics_task(physics0, 0));
tokio::spawn(physics_task(physics1, 1));
```

Note: `ws_tx` is the `broadcast::Sender<RobotStateMsg>` for the WebSocket — the visualiser already handles multiple robots (state is keyed by `robot_id`).

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator --lib 2>&1 | grep -E "test |FAILED|passed|error"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/main.rs
git commit -m "feat(m3): two-agent simulator setup with shared broadcast channel"
```

---

### Task 3: GBP inter-robot factors in AgentRunner

**Files:**
- Modify: `src/bins/simulator/src/agent_runner.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// In agent_runner.rs tests
#[test]
fn update_interrobot_factors_spawns_factor_when_same_edge() {
    // When robot B is on the same edge as robot A's planned trajectory,
    // update_interrobot_factors should result in at least one active factor.
    let map = std::sync::Arc::new(straight_map()); // single edge, length 5.0
    let (tx, rx0) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
    let rx1 = tx.subscribe();
    let comms0 = SimComms::new(tx.clone(), rx0);
    let comms1 = SimComms::new(tx.clone(), rx1);
    let mut runner0 = AgentRunner::new(comms0, map.clone(), 0);
    runner0.set_single_edge_trajectory(EdgeId(0), 5.0);
    let mut runner1 = AgentRunner::new(comms1, map.clone(), 1);
    runner1.set_single_edge_trajectory(EdgeId(0), 5.0);

    // Robot 1 broadcasts its state so robot 0 can see it
    runner1.broadcast_state(EdgeId(0), 4.0); // robot 1 is ahead at s=4.0

    // Robot 0 steps — should spawn InterRobotFactor because they share EdgeId(0)
    let out = runner0.step(0.5, EdgeId(0));
    assert!(out.active_factor_count > 0, "expected inter-robot factor to be spawned");
}

#[test]
fn iterate_runs_without_panic_with_two_robots() {
    // Just checks that running GBP iterate() doesn't panic under normal conditions.
    let map = std::sync::Arc::new(straight_map());
    let (tx, rx0) = tokio::sync::broadcast::channel::<RobotBroadcast>(32);
    let rx1 = tx.subscribe();
    let comms0 = SimComms::new(tx.clone(), rx0);
    let comms1 = SimComms::new(tx.clone(), rx1);
    let mut runner0 = AgentRunner::new(comms0, map.clone(), 0);
    runner0.set_single_edge_trajectory(EdgeId(0), 5.0);
    let mut runner1 = AgentRunner::new(comms1, map.clone(), 1);
    runner1.set_single_edge_trajectory(EdgeId(0), 5.0);
    runner1.broadcast_state(EdgeId(0), 3.5);
    // Steps with GBP iterate — should not panic
    for s in [0.5_f32, 1.0, 1.5, 2.0] {
        let _ = runner0.step(s, EdgeId(0));
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib agent_runner 2>&1 | head -20
```
Expected: compile error — `broadcast_state`, `active_factor_count` don't exist yet.

- [ ] **Step 2: Update `agent_runner.rs`**

Add `update_interrobot_factors()`, `iterate()`, and `broadcast_state()` calls to `AgentRunner`:

```rust
// Changes to agent_runner.rs

// Add to StepOut:
pub struct StepOut {
    pub velocity:           f32,
    pub position_s:         f32,
    pub current_edge:       EdgeId,
    pub pos_3d:             [f32; 3],
    pub active_factor_count: usize,
    pub belief_means:       [f32; MAX_HORIZON],
    pub belief_vars:        [f32; MAX_HORIZON],
}

// In AgentRunner::step():
pub fn step(&mut self, position_s: f32, current_edge: EdgeId) -> StepOut {
    let obs = ObservationUpdate { position_s, velocity: 0.0, current_edge };

    // GBP step ordering (per timestep):
    // 1. Receive inter-robot broadcasts and update factor graph (add/remove factors).
    // 2. Run iterate() — propagates messages on the factor graph built in step 1.
    //    The DynamicsFactor was already linearized at the previous timestep's position,
    //    which is close enough for warm-started GBP convergence.
    // 3. agent.step(obs) — updates the DynamicsFactor linearization at the new observed
    //    position and extracts the velocity decision from the converged beliefs.
    //    This re-linearization feeds into the NEXT timestep's iterate() call.
    //
    // This warm-start ordering (iterate → re-linearize) is consistent with the GBP
    // reference implementation and produces correct velocity profiles in practice.

    // 1. Receive broadcasts from other robots; update inter-robot factors
    let broadcasts = self.agent.comms_mut().receive_broadcasts();
    self.agent.update_interrobot_factors(&broadcasts);

    // 2. GBP iterate — 20 iterations per step
    self.agent.iterate(20);

    // 3. Re-linearize DynamicsFactor at current observation, extract velocity from beliefs
    let out = self.agent.step(obs);
    let p = self._map.eval_position(out.current_edge, out.position_s);
    let pos_3d = [p[0], p[2], -p[1]];

    StepOut {
        velocity:            out.velocity,
        position_s:          out.position_s,
        current_edge:        out.current_edge,
        pos_3d,
        active_factor_count: self.agent.interrobot_factor_count(),
        belief_means:        self.agent.belief_means(),
        belief_vars:         self.agent.belief_vars(),
    }
}

/// Broadcast this robot's current state so other robots' SimComms can receive it.
/// Called once per agent_task cycle after step().
pub fn broadcast_state(&mut self, current_edge: EdgeId, position_s: f32) {
    let msg = RobotBroadcast {
        robot_id:      self.robot_id,
        current_edge,
        position_s,
        planned_edges: self.planned_edges_snapshot(),
    };
    let _ = self.agent.comms_mut().broadcast(&msg);
}
```

Also add `robot_id: u32` field to `AgentRunner` and populate in `new()`.

Note: `RobotAgent` must expose `comms()`, `comms_mut()`, `interrobot_factor_count()`, `belief_means()`, `belief_vars()` — these are additions to `gbp-agent/src/lib.rs`. Add them in this task (they are simple accessors on existing fields, not new GBP logic).

Note: `RobotStateMsg` (defined in `gbp-comms`) must have `belief_means: [f32; MAX_HORIZON]` and `belief_vars: [f32; MAX_HORIZON]` fields for the WebSocket pipeline to carry belief data to the visualiser. If these fields were not added by M2 (M2 populated them with zeros), explicitly add them to `RobotStateMsg` in `crates/gbp-comms/src/lib.rs` now:

```rust
// In RobotStateMsg (gbp-comms/src/lib.rs) — add if not present:
pub belief_means: [f32; MAX_HORIZON],
pub belief_vars:  [f32; MAX_HORIZON],
```

Then in `agent_task`, replace the M2 zero-filled stubs with real values from `StepOut`:
```rust
belief_means: out.belief_means,
belief_vars:  out.belief_vars,
```

The visualiser's `ws_client.rs` → `drain_ws_inbox` converts `[f32; MAX_HORIZON]` arrays to `heapless::Vec<f32, MAX_HORIZON>` when populating `RobotState`:
```rust
state.belief_means = out.belief_means.iter().copied().collect();
state.belief_vars  = out.belief_vars.iter().copied().collect();
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator --lib agent_runner 2>&1 | grep -E "test |FAILED|passed"
```
Expected: both tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/agent_runner.rs crates/gbp-agent/src/lib.rs
git commit -m "feat(m3): GBP iterate + update_interrobot_factors in AgentRunner"
```

---

### Task 4: Integration test — minimum clearance maintained

**Files:**
- Create: `src/bins/simulator/tests/m3_clearance.rs`

- [ ] **Step 1: Write the failing integration test**

```rust
// src/bins/simulator/tests/m3_clearance.rs
//! Integration test: Robot B maintains d_safe clearance from Robot A.
//! Both robots are on the same edge. We run the simulator step loop in-process
//! for 200 steps and assert the minimum observed 3D distance >= d_safe.

use std::sync::{Arc, Mutex};
use tokio::sync::broadcast;
use gbp_comms::RobotBroadcast;
use gbp_map::{Map, EdgeId, map::{Node, NodeId, NodeType, Edge, EdgeGeometry, SpeedProfile, SafetyProfile}};
use simulator::{agent_runner::AgentRunner, physics::PhysicsState, sim_comms::SimComms};

fn single_edge_map() -> Arc<Map> {
    let mut m = Map::new("clearance_test");
    m.add_node(Node{id:NodeId(0),position:[0.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    m.add_node(Node{id:NodeId(1),position:[10.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
    let sf = SafetyProfile{clearance:0.3};
    m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
        geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[10.0,0.0,0.0],length:10.0},
        speed:sp,safety:sf}).unwrap();
    Arc::new(m)
}

#[test]
fn robot_b_maintains_d_safe_clearance_from_robot_a() {
    const D_SAFE: f32 = 0.3;
    const STEPS: usize = 200;
    const DT: f32 = 1.0 / 20.0; // 20 Hz agent step

    let map = single_edge_map();
    let (tx, rx0) = broadcast::channel::<RobotBroadcast>(64);
    let rx1 = tx.subscribe();

    let mut runner_a = AgentRunner::new(SimComms::new(tx.clone(), rx0), map.clone(), 0);
    runner_a.set_single_edge_trajectory(EdgeId(0), 10.0);
    let mut phys_a = PhysicsState::new(EdgeId(0), 10.0);
    phys_a.position_s = 2.0; // robot A starts at s=2.0

    let mut runner_b = AgentRunner::new(SimComms::new(tx.clone(), rx1), map.clone(), 1);
    runner_b.set_single_edge_trajectory(EdgeId(0), 10.0);
    let mut phys_b = PhysicsState::new(EdgeId(0), 10.0);
    phys_b.position_s = 0.0; // robot B starts at s=0.0 (behind A)

    let mut min_dist = f32::MAX;

    for _ in 0..STEPS {
        // Broadcast current states so each robot sees the other
        runner_a.broadcast_state(phys_a.current_edge, phys_a.position_s);
        runner_b.broadcast_state(phys_b.current_edge, phys_b.position_s);

        let out_a = runner_a.step(phys_a.position_s, phys_a.current_edge);
        let out_b = runner_b.step(phys_b.position_s, phys_b.current_edge);

        phys_a.velocity = out_a.velocity;
        phys_b.velocity = out_b.velocity;
        phys_a.position_s += phys_a.velocity * DT;
        phys_b.position_s += phys_b.velocity * DT;

        // Use 3D Euclidean distance as required by spec
        let da = out_a.pos_3d;
        let db = out_b.pos_3d;
        let dist = ((da[0]-db[0]).powi(2) + (da[1]-db[1]).powi(2) + (da[2]-db[2]).powi(2)).sqrt();
        if dist < min_dist { min_dist = dist; }

        // Stop once both robots have reached the end
        if phys_a.position_s >= 9.5 && phys_b.position_s >= 9.5 { break; }
    }

    assert!(
        min_dist >= D_SAFE,
        "minimum observed distance {:.4} m < d_safe {:.4} m", min_dist, D_SAFE
    );
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --test m3_clearance 2>&1 | head -20
```
Expected: compile error or test failure.

- [ ] **Step 2: Wire `PhysicsState` and `AgentRunner` for in-process test**

`PhysicsState` must be accessible from integration tests — ensure it's `pub` and its fields (`position_s`, `velocity`, `current_edge`) are `pub`. No code changes needed if already public from M1/M2; just verify.

- [ ] **Step 3: Run integration test**

```bash
cargo test -p simulator --test m3_clearance 2>&1 | grep -E "test |FAILED|passed|error"
```
Expected: `robot_b_maintains_d_safe_clearance_from_robot_a` passes.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/tests/m3_clearance.rs
git commit -m "test(m3): integration test — robot B maintains d_safe clearance from robot A"
```

---

## Chunk 2: Visualiser — belief tubes + factor lines + per-robot panel

### Task 5: Update `state.rs` for GBP fields

**Files:**
- Modify: `src/bins/visualiser/src/state.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In state.rs tests
#[test]
fn robot_state_default_has_zero_beliefs() {
    use gbp_map::MAX_HORIZON;
    let s = RobotState::default();
    assert_eq!(s.belief_means.len(), MAX_HORIZON);
    assert!(s.belief_means.iter().all(|&v| v == 0.0));
    assert!(s.belief_vars.iter().all(|&v| v == 0.0));
    assert_eq!(s.active_factor_count, 0);
    assert_eq!(s.sigma_dyn, 0.0);
    assert_eq!(s.sigma_r, 0.0);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib state 2>&1 | head -20
```

- [ ] **Step 2: Update `RobotState`**

```rust
// In state.rs — add fields to RobotState
#[derive(Default, Clone)]
pub struct RobotState {
    pub current_edge:       EdgeId,
    pub position_s:         f32,
    pub velocity:           f32,
    pub pos_3d:             [f32; 3],
    pub planned_edges:      heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
    // New in M3:
    pub belief_means:       heapless::Vec<f32, { gbp_map::MAX_HORIZON }>,
    pub belief_vars:        heapless::Vec<f32, { gbp_map::MAX_HORIZON }>,
    pub active_factor_count: usize,
    pub sigma_dyn:          f32,   // dynamics noise — for per-robot panel display
    pub sigma_r:            f32,   // inter-robot noise — for per-robot panel display
}
```

Also update `ws_client.rs` → `drain_ws_inbox` to populate these fields from `RobotStateMsg`:
```rust
state.belief_means        = msg.belief_means.iter().copied().collect();
state.belief_vars         = msg.belief_vars.iter().copied().collect();
state.active_factor_count = msg.active_factor_count as usize;
state.sigma_dyn           = msg.sigma_dyn;
state.sigma_r             = msg.sigma_r;
```

Note: `RobotStateMsg` in `gbp-comms` must also have `active_factor_count: u8`, `sigma_dyn: f32`, and `sigma_r: f32` fields. If not already present, add them now. At M3 the simulator populates `sigma_dyn`/`sigma_r` from the agent's current parameter values (read-only; sliders come in M6).

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib state 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/state.rs src/bins/visualiser/src/ws_client.rs
git commit -m "feat(m3): add belief_means/belief_vars/active_factor_count to visualiser state"
```

---

### Task 6: Belief tubes in `robot_render.rs`

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// In robot_render.rs tests
#[test]
fn belief_tube_radius_is_sqrt_variance() {
    // The tube radius at each timestep is sqrt(variance).
    // This tests the pure math, not Bevy rendering.
    let var: f32 = 4.0;
    let radius = belief_tube_radius(var);
    assert!((radius - 2.0).abs() < 1e-4, "expected sqrt(4)=2, got {}", radius);
}

#[test]
fn belief_tube_radius_clamps_near_zero_variance() {
    // variance=0 should produce radius=0 (not NaN from sqrt(0))
    let radius = belief_tube_radius(0.0);
    assert!(radius >= 0.0 && radius.is_finite());
}

#[test]
fn belief_positions_len_matches_k() {
    use gbp_map::map::*;
    let mut m = Map::new("t");
    m.add_node(Node{id:NodeId(0),position:[0.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    m.add_node(Node{id:NodeId(1),position:[5.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
    let sf = SafetyProfile{clearance:0.3};
    m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
        geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[5.0,0.0,0.0],length:5.0},
        speed:sp,safety:sf}).unwrap();

    let means: heapless::Vec<f32, 12> = (0..8).map(|i| i as f32 * 0.5).collect();
    let positions = belief_tube_positions(&m, EdgeId(0), &means);
    assert_eq!(positions.len(), means.len());
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib robot_render 2>&1 | head -20
```

- [ ] **Step 2: Implement belief tube helpers and Bevy system**

Add to `robot_render.rs`:

```rust
/// Tube radius = sqrt(variance). Clamp to 0 to avoid NaN.
pub fn belief_tube_radius(variance: f32) -> f32 {
    variance.max(0.0).sqrt()
}

/// Evaluate 3D world positions for a set of arc-length belief means on an edge.
/// Applies map→Bevy coordinate transform.
pub fn belief_tube_positions(
    map: &Map,
    edge_id: EdgeId,
    means: &heapless::Vec<f32, { gbp_map::MAX_HORIZON }>,
) -> heapless::Vec<[f32; 3], { gbp_map::MAX_HORIZON }> {
    let mut pts = heapless::Vec::new();
    for &s in means.iter() {
        let p = map.eval_position(edge_id, s);
        let _ = pts.push([p[0], p[2], -p[1]]);
    }
    pts
}

fn draw_belief_tubes(
    map:    Res<MapRes>,
    states: Res<RobotStates>,
    mut gizmos: Gizmos,
) {
    for state in states.0.values() {
        let pts = belief_tube_positions(&map.0, state.current_edge, &state.belief_means);
        for (i, &center) in pts.iter().enumerate() {
            let r = belief_tube_radius(*state.belief_vars.get(i).unwrap_or(&0.0));
            if r > 0.001 {
                // Draw a small circle gizmo at each belief mean position, radius = sqrt(variance)
                gizmos.circle(
                    Vec3::from(center),
                    Dir3::Z,          // normal to the tube axis (approximate; fine for debug)
                    r,
                    Color::srgba(1.0, 0.8, 0.2, 0.5),
                );
            }
        }
    }
}

fn draw_factor_links(
    states: Res<RobotStates>,
    mut gizmos: Gizmos,
) {
    // Draw a line between each pair of robots that have active inter-robot factors.
    // Heuristic: if robot_a and robot_b both report active_factor_count > 0 AND share
    // a planned edge, draw a gizmo line between their current 3D positions.
    let robot_list: heapless::Vec<(u32, [f32;3], &heapless::Vec<EdgeId, _>), 8> = states.0
        .iter()
        .filter(|(_, s)| s.active_factor_count > 0)
        .map(|(&id, s)| (id, s.pos_3d, &s.planned_edges))
        .collect();

    for i in 0..robot_list.len() {
        for j in (i+1)..robot_list.len() {
            let (_, pos_i, edges_i) = &robot_list[i];
            let (_, pos_j, edges_j) = &robot_list[j];
            let shares_edge = edges_i.iter().any(|e| edges_j.contains(e));
            if shares_edge {
                gizmos.line(
                    Vec3::from(*pos_i),
                    Vec3::from(*pos_j),
                    Color::srgba(1.0, 0.3, 0.3, 0.8),
                );
            }
        }
    }
}
```

Register in `RobotRenderPlugin::build`:
```rust
app.add_systems(Update,
    (drain_ws_inbox, update_robot_transforms, draw_planned_path, draw_belief_tubes, draw_factor_links).chain()
);
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib robot_render 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/robot_render.rs
git commit -m "feat(m3): belief tubes and factor-link gizmos in visualiser"
```

---

### Task 7: Per-robot egui side panel in `ui.rs`

**Files:**
- Modify: `src/bins/visualiser/src/ui.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In ui.rs tests
#[test]
fn format_factor_count_singular_and_plural() {
    assert_eq!(fmt_factor_count(0), "0 factors");
    assert_eq!(fmt_factor_count(1), "1 factor");
    assert_eq!(fmt_factor_count(3), "3 factors");
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib ui 2>&1 | head -20
```

- [ ] **Step 2: Update `ui.rs` with per-robot panel**

```rust
// Add to ui.rs

pub fn fmt_factor_count(n: usize) -> String {
    if n == 1 { "1 factor".into() } else { format!("{} factors", n) }
}

fn draw_hud(
    mut ctxs:  EguiContexts,
    states:    Res<RobotStates>,
    mut paused: ResMut<SimPaused>,
) {
    let ctx = ctxs.ctx_mut();

    // Global control panel
    egui::Window::new("Control").show(ctx, |ui| {
        let label = if paused.0 { "▶ Resume" } else { "⏸ Pause" };
        if ui.button(label).clicked() {
            paused.0 = !paused.0;
        }
    });

    // Per-robot side panels
    for (id, state) in &states.0 {
        egui::Window::new(format!("Robot {}", id)).show(ctx, |ui| {
            ui.label(format!("Speed:   {}", fmt_velocity(state.velocity)));
            ui.label(format!("Edge:    {:?} s={:.2}", state.current_edge, state.position_s));
            ui.label(format!("σ_dyn:   {:.4}", state.sigma_dyn));
            ui.label(format!("σ_r:     {:.4}", state.sigma_r));
            ui.label(fmt_factor_count(state.active_factor_count));
        });
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib ui 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/ui.rs
git commit -m "feat(m3): per-robot egui side panel with velocity and factor count"
```

---

### Task 8: End-to-end verification

- [ ] **Step 1: Run all unit and integration tests**

```bash
cargo test -p simulator -p visualiser -p gbp-agent 2>&1 | grep -E "test |FAILED|passed|error"
```
Expected: all tests pass, including `robot_b_maintains_d_safe_clearance_from_robot_a`.

- [ ] **Step 2: Manual visual verification**

```bash
cargo run -p simulator -- --map maps/test_loop_map.yaml &
cargo run -p visualiser
```

Verify:
- Two robots visible as arrow meshes
- Robot B placed behind Robot A on the same edge
- Robot B slows and maintains gap — minimum 3D distance >= 0.3 m observed
- Belief tubes visible at each robot, expanding/shrinking across timesteps
- Red factor-link gizmo line between the two robots while they share a planned edge
- Per-robot egui panels show correct speed and factor count
- Factor count drops to 0 once robots no longer share a planned edge

- [ ] **Step 3: Commit and tag M3**

```bash
git add -A
git commit -m "feat(m3): two robots, GBP rear-end avoidance complete"
git tag m3-complete
```

---
