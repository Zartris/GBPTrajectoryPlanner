# M4: Merge Collision Avoidance — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Two robots on different incoming edges negotiate before a merge node. Inter-robot factors spawn based on planned-edge-set intersection (not just current edge). One robot yields, both pass the merge without collision. Dashed planned routes per robot, shared edges highlighted, factor lines visible.

**Architecture:** `update_interrobot_factors()` is extended to compare the full planned edge sequence of each robot pair — a factor is spawned for every timestep index K where planned edges overlap. `planned_edges_snapshot_from(current_edge)` in `AgentRunner` returns only the remaining edges (at-or-after the current edge) so other robots can check intersection without stale completed edges. The visualiser adds per-robot planned-route overlays and highlights shared edges in yellow when a factor is active on that edge.

**Tech Stack:** No new external dependencies beyond M3.

**Prerequisite:** M3 plan complete and passing.

---

## File Structure

```
src/bins/simulator/
  src/
    main.rs            — updated: merge-scenario robot placement
    agent_runner.rs    — updated: planned_edges_snapshot returns full remaining trajectory
    sim_comms.rs       — unchanged

src/bins/visualiser/
  src/
    robot_render.rs    — updated: per-robot planned-route colour, shared-edge highlight
    ui.rs              — unchanged
    state.rs           — unchanged
    ws_client.rs       — unchanged
    map_scene.rs       — unchanged
```

---

## Chunk 1: Simulator — planned-edge-set intersection factor gating

### Task 1: `planned_edges_snapshot` returns full remaining trajectory

**Files:**
- Modify: `src/bins/simulator/src/agent_runner.rs`

The M2/M3 implementation stores `trajectory_edges` as the full set of all planned edges. By M3, `planned_edges_snapshot()` already returns these edges (the full trajectory, not just remaining). In M4 we need it to return only the _remaining_ edges (edges at or after the current edge) so that robots that have already passed a shared edge don't keep factors active unnecessarily. This task upgrades `planned_edges_snapshot` to trim already-completed edges.

- [ ] **Step 1: Write the failing test**

```rust
// In agent_runner.rs tests
#[test]
fn planned_edges_snapshot_trims_completed_edges() {
    // After advancing past edge 0, snapshot should contain only edge 1 (and beyond).
    let map = {
        let mut m = Map::new("two");
        m.add_node(Node{id:NodeId(0),position:[0.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
        m.add_node(Node{id:NodeId(1),position:[5.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
        m.add_node(Node{id:NodeId(2),position:[10.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
        let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
        let sf = SafetyProfile{clearance:0.3};
        m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
            geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[5.0,0.0,0.0],length:5.0},
            speed:sp.clone(),safety:sf.clone()}).unwrap();
        m.add_edge(Edge{id:EdgeId(1),start:NodeId(1),end:NodeId(2),
            geometry:EdgeGeometry::Line{start:[5.0,0.0,0.0],end:[10.0,0.0,0.0],length:5.0},
            speed:sp,safety:sf}).unwrap();
        std::sync::Arc::new(m)
    };
    let (tx, rx) = tokio::sync::broadcast::channel(32);
    let comms = SimComms::new(tx.clone(), rx);
    let mut runner = AgentRunner::new(comms, map.clone(), 0);
    let mut edges: heapless::Vec<(EdgeId, f32), 32> = heapless::Vec::new();
    edges.push((EdgeId(0), 5.0)).unwrap();
    edges.push((EdgeId(1), 5.0)).unwrap();
    runner.set_trajectory_from_edges(edges, 0.0);

    // Before any steps: snapshot contains both edges
    let snap_before = runner.planned_edges_snapshot_from(EdgeId(0));
    assert_eq!(snap_before.len(), 2);

    // After transitioning to edge 1: snapshot contains only edge 1
    let snap_after = runner.planned_edges_snapshot_from(EdgeId(1));
    assert_eq!(snap_after.len(), 1);
    assert_eq!(snap_after[0], EdgeId(1));
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib agent_runner 2>&1 | head -20
```
Expected: compile error — `planned_edges_snapshot_from` doesn't exist yet.

- [ ] **Step 2: Implement `planned_edges_snapshot_from`**

Add to `AgentRunner`:

```rust
/// Returns only the edges in the trajectory at or after `current_edge`.
/// Used for inter-robot factor gating: only future/current edges matter.
pub fn planned_edges_snapshot_from(
    &self,
    current_edge: EdgeId,
) -> heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
    let mut out = heapless::Vec::new();
    let mut found = false;
    for &eid in &self.trajectory_edges {
        if eid == current_edge { found = true; }
        if found { let _ = out.push(eid); }
    }
    out
}
```

Update `broadcast_state` to use this method:
```rust
pub fn broadcast_state(&mut self, current_edge: EdgeId, position_s: f32) {
    let msg = RobotBroadcast {
        robot_id:      self.robot_id,
        current_edge,
        position_s,
        planned_edges: self.planned_edges_snapshot_from(current_edge),
    };
    let _ = self.agent.comms_mut().broadcast(&msg);
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator --lib agent_runner 2>&1 | grep -E "test |FAILED|passed"
```
Expected: `planned_edges_snapshot_trims_completed_edges` passes, all previous tests still pass.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/agent_runner.rs
git commit -m "feat(m4): planned_edges_snapshot_from trims completed edges"
```

---

### Task 2: Factor lifecycle integration test — spawn before merge, remove after

**Files:**
- Create: `src/bins/simulator/tests/m4_merge.rs`

- [ ] **Step 1: Write the failing integration test**

```rust
// src/bins/simulator/tests/m4_merge.rs
//! Integration test: Two robots on different incoming edges.
//! Inter-robot factor is spawned BEFORE either robot reaches the shared merge edge,
//! and REMOVED AFTER the leader has fully cleared the merge node.

use std::sync::Arc;
use tokio::sync::broadcast;
use gbp_comms::RobotBroadcast;
use gbp_map::{Map, EdgeId, map::{Node, NodeId, NodeType, Edge, EdgeGeometry, SpeedProfile, SafetyProfile}};
use simulator::{agent_runner::AgentRunner, physics::PhysicsState, sim_comms::SimComms};

/// Map: two edges converging at node 1, then one outgoing edge 2→3.
///   Edge 0: node 0 → node 1 (robot A's approach)
///   Edge 1: node 2 → node 1 (robot B's approach)
///   Edge 2: node 1 → node 3 (shared post-merge edge)
fn merge_map() -> Arc<Map> {
    let mut m = Map::new("merge_test");
    m.add_node(Node{id:NodeId(0),position:[0.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    m.add_node(Node{id:NodeId(1),position:[5.0,0.0,0.0],node_type:NodeType::MergeNode}).unwrap();
    m.add_node(Node{id:NodeId(2),position:[5.0,5.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    m.add_node(Node{id:NodeId(3),position:[10.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    let sp = SpeedProfile{max:2.5,nominal:2.0,accel_limit:1.0,decel_limit:1.0};
    let sf = SafetyProfile{clearance:0.3};
    m.add_edge(Edge{id:EdgeId(0),start:NodeId(0),end:NodeId(1),
        geometry:EdgeGeometry::Line{start:[0.0,0.0,0.0],end:[5.0,0.0,0.0],length:5.0},
        speed:sp.clone(),safety:sf.clone()}).unwrap();
    m.add_edge(Edge{id:EdgeId(1),start:NodeId(2),end:NodeId(1),
        geometry:EdgeGeometry::Line{start:[5.0,5.0,0.0],end:[5.0,0.0,0.0],length:5.0},
        speed:sp.clone(),safety:sf.clone()}).unwrap();
    m.add_edge(Edge{id:EdgeId(2),start:NodeId(1),end:NodeId(3),
        geometry:EdgeGeometry::Line{start:[5.0,0.0,0.0],end:[10.0,0.0,0.0],length:5.0},
        speed:sp,safety:sf}).unwrap();
    Arc::new(m)
}

#[test]
fn interrobot_factor_spawned_before_merge_cleared_after() {
    const D_SAFE: f32 = 0.3;
    const STEPS: usize = 300;
    const DT: f32 = 1.0 / 20.0;

    let map = merge_map();
    let (tx, rx0) = broadcast::channel::<RobotBroadcast>(64);
    let rx1 = tx.subscribe();

    // Robot A: edge 0 → edge 2 (approaches from left)
    let mut runner_a = AgentRunner::new(SimComms::new(tx.clone(), rx0), map.clone(), 0);
    let mut traj_a: heapless::Vec<(EdgeId, f32), 32> = heapless::Vec::new();
    traj_a.push((EdgeId(0), 5.0)).unwrap();
    traj_a.push((EdgeId(2), 5.0)).unwrap();
    runner_a.set_trajectory_from_edges(traj_a, 0.0);
    let mut phys_a = PhysicsState::new(EdgeId(0), 5.0);

    // Robot B: edge 1 → edge 2 (approaches from above)
    let mut runner_b = AgentRunner::new(SimComms::new(tx.clone(), rx1), map.clone(), 1);
    let mut traj_b: heapless::Vec<(EdgeId, f32), 32> = heapless::Vec::new();
    traj_b.push((EdgeId(1), 5.0)).unwrap();
    traj_b.push((EdgeId(2), 5.0)).unwrap();
    runner_b.set_trajectory_from_edges(traj_b, 0.0);
    let mut phys_b = PhysicsState::new(EdgeId(1), 5.0);

    let mut factor_spawn_step:       Option<usize> = None;
    let mut factor_removed_step:     Option<usize> = None;
    let mut leader_cleared_step:     Option<usize> = None;
    let mut first_on_shared_edge_step: Option<usize> = None;
    let mut min_dist = f32::MAX;

    for step in 0..STEPS {
        runner_a.broadcast_state(phys_a.current_edge, phys_a.position_s);
        runner_b.broadcast_state(phys_b.current_edge, phys_b.position_s);

        let out_a = runner_a.step(phys_a.position_s, phys_a.current_edge);
        let out_b = runner_b.step(phys_b.position_s, phys_b.current_edge);

        // Track factor lifecycle
        let either_has_factor = out_a.active_factor_count > 0 || out_b.active_factor_count > 0;
        if either_has_factor && factor_spawn_step.is_none() {
            factor_spawn_step = Some(step);
        }
        if factor_spawn_step.is_some() && !either_has_factor && factor_removed_step.is_none() {
            factor_removed_step = Some(step);
        }

        // Track first step either robot arrives on the shared post-merge edge (EdgeId(2))
        if first_on_shared_edge_step.is_none()
            && (phys_a.current_edge == EdgeId(2) || phys_b.current_edge == EdgeId(2)) {
            first_on_shared_edge_step = Some(step);
        }

        // Track when the leader (first robot on edge 2) has fully cleared it (s >= edge.length)
        const EDGE2_LEN: f32 = 5.0; // must match merge_map() edge 2 length
        if phys_a.current_edge == EdgeId(2) && phys_a.position_s >= EDGE2_LEN && leader_cleared_step.is_none() {
            leader_cleared_step = Some(step);
        }
        if phys_b.current_edge == EdgeId(2) && phys_b.position_s >= EDGE2_LEN && leader_cleared_step.is_none() {
            leader_cleared_step = Some(step);
        }

        phys_a.velocity = out_a.velocity;
        phys_b.velocity = out_b.velocity;
        phys_a.position_s += phys_a.velocity * DT;
        phys_b.position_s += phys_b.velocity * DT;

        // Edge transitions via PhysicsState::transition_to
        if out_a.current_edge != phys_a.current_edge {
            if let Some(idx) = map.edge_index(out_a.current_edge) {
                phys_a.transition_to(out_a.current_edge, map.edges[idx].geometry.length());
            }
        }
        if out_b.current_edge != phys_b.current_edge {
            if let Some(idx) = map.edge_index(out_b.current_edge) {
                phys_b.transition_to(out_b.current_edge, map.edges[idx].geometry.length());
            }
        }

        // 3D distance
        let da = out_a.pos_3d;
        let db = out_b.pos_3d;
        let dist = ((da[0]-db[0]).powi(2) + (da[1]-db[1]).powi(2) + (da[2]-db[2]).powi(2)).sqrt();
        if dist < min_dist { min_dist = dist; }

        // Exit only after BOTH robots have fully cleared edge 2 (position_s >= edge length).
        // Using EDGE2_LEN (not 0.9 * EDGE2_LEN) ensures leader_cleared_step is always set.
        if phys_a.current_edge == EdgeId(2) && phys_b.current_edge == EdgeId(2)
            && phys_a.position_s >= EDGE2_LEN && phys_b.position_s >= EDGE2_LEN { break; }
    }

    // 1. Factor must have been spawned BEFORE either robot reached the shared edge
    assert!(factor_spawn_step.is_some(), "inter-robot factor was never spawned");
    assert!(first_on_shared_edge_step.is_some(), "neither robot reached the shared edge — test scenario invalid");
    let spawn       = factor_spawn_step.unwrap();
    let first_shared = first_on_shared_edge_step.unwrap();
    assert!(
        spawn < first_shared,
        "factor spawned at step {} must be before first robot on shared edge at step {}", spawn, first_shared
    );
    // 2. Factor must be removed AFTER the leader has fully cleared the merge node
    assert!(leader_cleared_step.is_some(), "leader never fully cleared merge node — increase STEPS or check dynamics");
    assert!(factor_removed_step.is_some(), "inter-robot factor was never removed — factor lifecycle broken");
    let removed = factor_removed_step.unwrap();
    let cleared = leader_cleared_step.unwrap();
    assert!(
        removed >= cleared,
        "factor removed at step {} before leader fully cleared merge at step {}", removed, cleared
    );
    // 3. 3D clearance maintained throughout
    assert!(min_dist >= D_SAFE, "min 3D dist {:.4} < d_safe {:.4}", min_dist, D_SAFE);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --test m4_merge 2>&1 | head -20
```
Expected: compile error — `first_on_shared_edge_step` variable or `PhysicsState::transition_to` not yet available; factor spawning not yet wired with planned-edge intersection.

- [ ] **Step 3: Run integration test**

```bash
cargo test -p simulator --test m4_merge 2>&1 | grep -E "test |FAILED|passed|error"
```
Expected: `interrobot_factor_spawned_before_merge_cleared_after` passes.

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/tests/m4_merge.rs
git commit -m "test(m4): factor lifecycle integration test — spawn before merge, cleared after"
```

---

## Chunk 2: Visualiser — per-robot route colours + shared-edge highlight

### Task 3: Per-robot planned route and shared-edge highlight

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// In robot_render.rs tests
#[test]
fn robot_colour_by_id_produces_distinct_colours() {
    // All 8 palette entries must be distinct
    let colours: [_; 8] = std::array::from_fn(|i| robot_colour(i as u32));
    for i in 0..8 {
        for j in (i+1)..8 {
            assert_ne!(colours[i], colours[j],
                "robot_colour({}) == robot_colour({}) — colours must be distinct", i, j);
        }
    }
    // Wrapping: robot 8 must equal robot 0 (palette length = 8)
    assert_eq!(robot_colour(8), robot_colour(0));
}

#[test]
fn shared_edges_between_two_robots_returns_intersection() {
    let mut a: heapless::Vec<EdgeId, 8> = heapless::Vec::new();
    let mut b: heapless::Vec<EdgeId, 8> = heapless::Vec::new();
    a.push(EdgeId(0)).unwrap(); a.push(EdgeId(2)).unwrap();
    b.push(EdgeId(1)).unwrap(); b.push(EdgeId(2)).unwrap();
    let shared = shared_edges(&a, &b);
    assert_eq!(shared.len(), 1);
    assert_eq!(shared[0], EdgeId(2));
}

#[test]
fn shared_edges_empty_when_no_overlap() {
    let mut a: heapless::Vec<EdgeId, 8> = heapless::Vec::new();
    let mut b: heapless::Vec<EdgeId, 8> = heapless::Vec::new();
    a.push(EdgeId(0)).unwrap();
    b.push(EdgeId(1)).unwrap();
    assert_eq!(shared_edges(&a, &b).len(), 0);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib robot_render 2>&1 | head -20
```

- [ ] **Step 2: Implement helpers and systems**

Add to `robot_render.rs`:

```rust
/// Per-robot colour palette — each robot id gets a distinct colour for its planned route.
/// 8 entries cover the M5 default fleet (4 robots) with room to spare.
/// For fleets larger than 8 colours wrap (robot_id % 8).
pub fn robot_colour(robot_id: u32) -> Color {
    const PALETTE: &[[f32; 4]] = &[
        [0.3, 0.8, 1.0, 0.8],  // robot 0: cyan
        [1.0, 0.6, 0.2, 0.8],  // robot 1: orange
        [0.4, 1.0, 0.4, 0.8],  // robot 2: green
        [0.9, 0.3, 0.9, 0.8],  // robot 3: magenta
        [1.0, 1.0, 0.4, 0.8],  // robot 4: yellow
        [0.4, 0.8, 0.4, 0.8],  // robot 5: lime
        [0.8, 0.4, 0.2, 0.8],  // robot 6: brown-orange
        [0.6, 0.6, 1.0, 0.8],  // robot 7: lavender
    ];
    let [r, g, b, a] = PALETTE[(robot_id as usize) % PALETTE.len()];
    Color::srgba(r, g, b, a)
}

/// Returns edges that appear in both planned-edge lists (intersection).
pub fn shared_edges(
    a: &heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
    b: &heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }>,
) -> heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }> {
    let mut out = heapless::Vec::new();
    for &eid in a.iter() {
        if b.contains(&eid) && !out.contains(&eid) {
            let _ = out.push(eid);
        }
    }
    out
}

fn draw_planned_paths_per_robot(
    map:    Res<MapRes>,
    states: Res<RobotStates>,
    mut gizmos: Gizmos,
) {
    // Collect all planned edges across all robots for shared-edge detection.
    // heapless::Vec does not implement FromIterator, so build with an explicit loop.
    let mut robot_list: heapless::Vec<(u32, &heapless::Vec<EdgeId, { gbp_map::MAX_PATH_EDGES }>), 8> = heapless::Vec::new();
    for (&id, s) in states.0.iter() {
        let _ = robot_list.push((id, &s.planned_edges));
    }

    // Draw each robot's planned route in its own colour
    for &(robot_id, edges) in &robot_list {
        let base_color = robot_colour(robot_id);
        for &edge_id in edges.iter() {
            // Check if this edge is shared with any other robot that has an active factor
            let is_shared = robot_list.iter().any(|&(other_id, other_edges)| {
                other_id != robot_id
                    && other_edges.contains(&edge_id)
                    && states.0.get(&other_id).map_or(false, |s| s.active_factor_count > 0)
            });
            let color = if is_shared {
                Color::srgba(1.0, 1.0, 0.0, 0.9) // yellow = shared with active factor
            } else {
                base_color
            };
            let pts = edge_sample_points(&map.0, edge_id, PLANNED_PATH_GIZMO_SAMPLES);
            for (a, b) in dashed_segment_pairs(&pts) {
                gizmos.line(Vec3::from(a), Vec3::from(b), color);
            }
        }
    }
}
```

Replace the old `draw_planned_path` with `draw_planned_paths_per_robot` in the plugin registration:
```rust
app.add_systems(Update,
    (drain_ws_inbox, update_robot_transforms, draw_planned_paths_per_robot,
     draw_belief_tubes, draw_factor_links).chain()
);
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib robot_render 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/robot_render.rs
git commit -m "feat(m4): per-robot planned-route colours and shared-edge highlight"
```

---

### Task 4: End-to-end verification

- [ ] **Step 1: Run all tests**

```bash
cargo test -p simulator -p visualiser -p gbp-agent 2>&1 | grep -E "test |FAILED|passed|error"
```
Expected: all pass including `interrobot_factor_spawned_before_merge_cleared_after`.

- [ ] **Step 2: Manual visual verification**

```bash
cargo run -p simulator -- --map maps/test_loop_map.yaml --scenario merge &
cargo run -p visualiser
```

Verify:
- Two robots assigned routes that converge at a merge node simultaneously
- Each robot's planned route shown in its own colour (cyan vs orange)
- Shared post-merge edge highlighted in yellow when the factor is active
- Factor gizmo line connects the robots while they share a planned edge
- One robot holds back (near-zero velocity in its belief tube), other passes through
- After the leader clears the merge, the follower accelerates and the factor gizmo disappears

- [ ] **Step 3: Commit and tag M4**

```bash
git add -A
git commit -m "feat(m4): merge collision avoidance complete"
git tag m4-complete
```

---

## Required change from M3: 3D-based safety cap

M3's `nearest_ahead_distance` / `max_position` compare global arc-length (`position_s`) which only works when both robots share the same trajectory. For merge scenarios (different trajectories converging on a shared edge), this comparison is meaningless.

**Must change in M4:**
- [ ] Safety cap uses **3D world distance** (`|pos_3d_a - pos_3d_b|`) instead of arc-length difference
- [ ] "Ahead" is determined by whether the other robot is on a shared edge AND further along it in the travel direction
- [ ] At merge points, both robots approaching the same node should slow down based on 3D proximity, not trajectory-relative position
