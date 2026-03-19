# M5: Fleet + Full Visualiser — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** N robots (default 4) run in random trajectory mode on the full map. Robot mesh upgraded to `chassis.stl`. Physical track geometry and magnetic markers overlaid. Full UI panels: global control, per-robot status, map node/edge inspector, click-to-assign goal.

**Architecture:** Simulator spawns N `AgentRunner` instances at startup at distributed start nodes. Random mode: when a robot reaches its goal, A* picks a new random reachable node. The visualiser loads STL assets for environment rendering and replaces the arrow mesh with the chassis model. Bevy egui panels are added for the global control panel and map inspector. Click-to-assign uses Bevy raycasting against node sphere colliders.

**Tech Stack:** `bevy_asset_loader` for STL loading; `bevy_mod_picking` or Bevy 0.15 built-in `Picking` for click-to-assign. No firmware changes.

**Prerequisite:** M4 plan complete and passing.

---

## File Structure

```
src/bins/simulator/
  src/
    main.rs            — updated: N-robot spawn, random trajectory mode
    agent_runner.rs    — unchanged
    sim_comms.rs       — unchanged

src/bins/visualiser/
  src/
    main.rs            — updated: add STL asset loading, click-to-assign, map inspector plugin
    robot_render.rs    — updated: chassis STL mesh replaces arrow
    map_scene.rs       — updated: physical_track + magnetic_mainlines + markers overlaid
    ui.rs              — updated: full global panel + map inspector + trajectory input
    state.rs           — unchanged
    ws_client.rs       — unchanged
```

---

## Chunk 1: Simulator — N-robot fleet + random mode

### Task 1: N-robot spawn at distributed start nodes

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In main.rs tests
#[test]
fn distributed_start_nodes_picks_n_distinct_nodes() {
    // Given a map with M nodes, pick N start nodes distributed across them.
    // All selected nodes must be distinct.
    // heapless::Vec has no FromIterator, so build with an explicit loop.
    let mut nodes: heapless::Vec<NodeId, 16> = heapless::Vec::new();
    for i in 0..8u32 { let _ = nodes.push(NodeId(i)); }
    let starts = pick_distributed_starts(&nodes, 4);
    assert_eq!(starts.len(), 4);
    let mut seen: heapless::Vec<NodeId, 8> = heapless::Vec::new();
    for &n in &starts {
        assert!(!seen.contains(&n), "duplicate start node {:?}", n);
        let _ = seen.push(n);
    }
}

#[test]
fn distributed_start_nodes_clamps_to_node_count() {
    // When n > node count, clamp so no duplicates are returned.
    let mut nodes: heapless::Vec<NodeId, 16> = heapless::Vec::new();
    for i in 0..3u32 { let _ = nodes.push(NodeId(i)); }
    let starts = pick_distributed_starts(&nodes, 5); // asking for 5 from 3
    assert_eq!(starts.len(), 3, "should clamp to available node count");
    // All returned nodes must still be distinct
    let mut seen: heapless::Vec<NodeId, 8> = heapless::Vec::new();
    for &n in &starts {
        assert!(!seen.contains(&n));
        let _ = seen.push(n);
    }
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -20
```
Expected: compile error — `pick_distributed_starts` doesn't exist.

- [ ] **Step 2: Implement `pick_distributed_starts` and N-robot spawn**

```rust
/// Pick `n` start nodes distributed evenly across the node list.
/// Spacing = max(1, total / n). Wraps if n > total.
pub fn pick_distributed_starts(
    nodes: &[NodeId],
    n: usize,
) -> heapless::Vec<NodeId, { gbp_map::MAX_NODES }> {
    let mut out = heapless::Vec::new();
    if nodes.is_empty() || n == 0 { return out; }
    // Clamp n to node count to guarantee distinctness.
    // Requesting more robots than map nodes is a configuration error — silently clamp.
    let n = n.min(nodes.len());
    let step = (nodes.len() / n).max(1);
    for i in 0..n {
        let _ = out.push(nodes[i * step]); // i*step < nodes.len() because step >= nodes.len()/n
    }
    out
}
```

In `main()`, replace the two-robot hardcoded setup with:
```rust
// N robots at distributed start nodes
let num_robots: usize = args.num_robots.unwrap_or(4);
let all_nodes: Vec<NodeId> = map_arc.nodes.iter().map(|n| n.id).collect();
let start_nodes = pick_distributed_starts(&all_nodes, num_robots);
let (bcast_tx, _) = broadcast::channel::<RobotBroadcast>(64);

for (i, &start_node) in start_nodes.iter().enumerate() {
    let rx = bcast_tx.subscribe();
    let comms = SimComms::new(bcast_tx.clone(), rx);
    let runner = Arc::new(Mutex::new(AgentRunner::new(comms, map_arc.clone(), i as u32)));
    let physics = Arc::new(Mutex::new(PhysicsState::new(start_edge_for_node(&map_arc, start_node), 0.0)));
    // Assign random initial goal
    if let Some(goal) = random_reachable_node(&map_arc, start_node) {
        if let Some(path) = astar(&map_arc, start_node, goal) {
            let traj = build_trajectory_edges(&map_arc, &path);
            runner.lock().unwrap().set_trajectory_from_edges(traj, 0.0);
        }
    }
    tokio::spawn(agent_task(physics.clone(), runner.clone(), map_arc.clone(), ws_tx.clone(), i as u32));
    tokio::spawn(physics_task(physics, i as u32));
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator --lib 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/main.rs
git commit -m "feat(m5): N-robot fleet spawn at distributed start nodes"
```

---

### Task 2: Random trajectory reassignment on goal arrival

**Files:**
- Modify: `src/bins/simulator/src/main.rs` (or `agent_runner.rs`)

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn random_reachable_node_returns_different_node() {
    // random_reachable_node should never return the start node itself.
    let map = single_edge_map(); // two nodes: 0 and 1
    // From node 0, only node 1 is reachable (excluding self)
    let result = random_reachable_node(&map, NodeId(0));
    assert!(result.is_some());
    assert_ne!(result.unwrap(), NodeId(0));
}

#[test]
fn random_reachable_node_returns_none_for_isolated_node() {
    let mut m = Map::new("isolated");
    m.add_node(Node{id:NodeId(0),position:[0.0,0.0,0.0],node_type:NodeType::Waypoint}).unwrap();
    // No edges — node 0 is isolated
    let result = random_reachable_node(&m, NodeId(0));
    assert!(result.is_none());
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p simulator --lib 2>&1 | head -20
```

- [ ] **Step 2: Implement `random_reachable_node` and goal-arrival reassignment**

```rust
/// Return a random node reachable from `start` (excluding `start` itself).
/// Uses a BFS/DFS over map edges to find all reachable nodes, then picks one at random.
/// Returns None if no other node is reachable.
pub fn random_reachable_node(map: &Map, start: NodeId) -> Option<NodeId> {
    // Simple: collect all nodes that A* can reach (i.e., there exists a path)
    let mut reachable: heapless::Vec<NodeId, { gbp_map::MAX_NODES }> = heapless::Vec::new();
    for node in &map.nodes {
        if node.id != start {
            if astar(map, start, node.id).is_some() {
                let _ = reachable.push(node.id);
            }
        }
    }
    if reachable.is_empty() { return None; }
    // Deterministic pseudo-random pick: use a simple hash of start id + reachable count
    let idx = (start.0 as usize + reachable.len()) % reachable.len();
    Some(reachable[idx])
}
```

In `agent_task`, after detecting goal arrival (velocity ≈ 0 and `position_s >= edge_length - 0.05`), reassign:
```rust
// Goal-arrival detection in agent_task
if out.velocity < 0.01 && phys_lock.position_s >= phys_lock.edge_length - 0.05 {
    if let Some(new_goal) = random_reachable_node(&map, current_node_id) {
        if let Some(path) = astar(&map, current_node_id, new_goal) {
            let traj = build_trajectory_edges(&map, &path);
            runner.lock().unwrap().set_trajectory_from_edges(traj, 0.0);
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p simulator --lib 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/main.rs
git commit -m "feat(m5): random trajectory reassignment on goal arrival"
```

---

## Chunk 2: Visualiser — STL models + full UI panels

### Task 3: STL environment models in `map_scene.rs`

**Files:**
- Modify: `src/bins/visualiser/src/map_scene.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In map_scene.rs tests
#[test]
fn stl_asset_paths_are_correct() {
    // These paths are relative to the visualiser's asset directory.
    // If any change, this test will catch it before runtime.
    assert_eq!(CHASSIS_STL_PATH, "models/chassis.stl");
    assert_eq!(PHYSICAL_TRACK_STL_PATH, "models/physical_track.stl");
    assert_eq!(MAGNETIC_MAINLINES_STL_PATH, "models/magnetic_mainlines.stl");
    assert_eq!(MAGNETIC_MARKERS_STL_PATH, "models/magnetic_markers.stl");
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib map_scene 2>&1 | head -20
```

- [ ] **Step 2: Implement STL loading in `map_scene.rs`**

```rust
// In map_scene.rs
pub const CHASSIS_STL_PATH:            &str = "models/chassis.stl";
pub const PHYSICAL_TRACK_STL_PATH:     &str = "models/physical_track.stl";
pub const MAGNETIC_MAINLINES_STL_PATH: &str = "models/magnetic_mainlines.stl";
pub const MAGNETIC_MARKERS_STL_PATH:   &str = "models/magnetic_markers.stl";

#[derive(Resource)]
pub struct EnvironmentAssets {
    pub chassis:           Handle<Mesh>,
    pub physical_track:    Handle<Mesh>,
    pub magnetic_mainlines:Handle<Mesh>,
    pub magnetic_markers:  Handle<Mesh>,
}

fn load_environment_assets(
    mut commands:   Commands,
    asset_server:   Res<AssetServer>,
) {
    commands.insert_resource(EnvironmentAssets {
        chassis:            asset_server.load(CHASSIS_STL_PATH),
        physical_track:     asset_server.load(PHYSICAL_TRACK_STL_PATH),
        magnetic_mainlines: asset_server.load(MAGNETIC_MAINLINES_STL_PATH),
        magnetic_markers:   asset_server.load(MAGNETIC_MARKERS_STL_PATH),
    });
}

fn spawn_environment(
    mut commands:    Commands,
    assets:          Res<EnvironmentAssets>,
    mut materials:   ResMut<Assets<StandardMaterial>>,
    map:             Res<MapRes>,
) {
    // Physical track — single mesh instance at world origin
    let track_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.4, 0.4, 0.4),
        ..default()
    });
    commands.spawn(PbrBundle {
        mesh: assets.physical_track.clone(),
        material: track_mat.clone(),
        ..default()
    });
    commands.spawn(PbrBundle {
        mesh: assets.magnetic_mainlines.clone(),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.1, 0.8),
            ..default()
        }),
        ..default()
    });

    // Magnetic markers — one instance per fiducial node in the map
    let marker_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.8, 0.0),
        ..default()
    });
    for node in &map.0.nodes {
        if node.node_type == NodeType::Fiducial {
            let p = node.position;
            commands.spawn(PbrBundle {
                mesh: assets.magnetic_markers.clone(),
                material: marker_mat.clone(),
                transform: Transform::from_xyz(p[0], p[2], -p[1]), // map→Bevy coord
                ..default()
            });
        }
    }
}
```

Register in `MapScenePlugin::build`:
```rust
app.add_systems(Startup, (load_environment_assets, setup_map_scene).chain())
   .add_systems(Update, spawn_environment.run_if(resource_added::<EnvironmentAssets>));
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib map_scene 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/map_scene.rs
git commit -m "feat(m5): STL environment models loaded in visualiser"
```

---

### Task 4: Chassis STL replaces arrow mesh in `robot_render.rs`

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In robot_render.rs tests
#[test]
fn chassis_transform_applies_bevy_coord() {
    // Robot at map position (1, 2, 0.5), heading along X.
    // Bevy transform: x=1, y=0.5 (map_z), z=-2 (-map_y).
    let map_pos = [1.0_f32, 2.0, 0.5];
    let t = robot_transform_from_map_pos(map_pos, [1.0, 0.0, 0.0]);
    assert!((t.translation.x - 1.0).abs() < 1e-4);
    assert!((t.translation.y - 0.5).abs() < 1e-4);
    assert!((t.translation.z - (-2.0)).abs() < 1e-4);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib robot_render 2>&1 | head -20
```

- [ ] **Step 2: Implement chassis mesh and `robot_transform_from_map_pos`**

```rust
// In robot_render.rs

/// Build a Bevy Transform from a map-space position and tangent direction.
/// Applies map(x,y,z) → Bevy(x,z,-y) coordinate transform.
pub fn robot_transform_from_map_pos(map_pos: [f32; 3], map_tangent: [f32; 3]) -> Transform {
    let pos = Vec3::new(map_pos[0], map_pos[2], -map_pos[1]);
    let fwd = Vec3::new(map_tangent[0], map_tangent[2], -map_tangent[1]).normalize_or_zero();
    let mut t = Transform::from_translation(pos);
    if fwd.length_squared() > 0.01 {
        t.look_to(fwd, Vec3::Y);
    }
    t
}

fn spawn_robot_meshes(
    mut commands:    Commands,
    assets:          Res<EnvironmentAssets>,
    mut materials:   ResMut<Assets<StandardMaterial>>,
    states:          Res<RobotStates>,
    query:           Query<&RobotId>,
) {
    // Spawn a chassis mesh entity per new robot_id seen in RobotStates
    for (&id, _state) in states.0.iter() {
        if query.iter().any(|r| r.0 == id) { continue; } // already spawned
        let color = robot_colour(id);
        commands.spawn((
            PbrBundle {
                mesh: assets.chassis.clone(),
                material: materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                }),
                ..default()
            },
            RobotId(id),
        ));
    }
}

fn update_robot_transforms(
    states: Res<RobotStates>,
    map:    Res<MapRes>,
    mut query: Query<(&RobotId, &mut Transform)>,
) {
    for (robot_id, mut transform) in query.iter_mut() {
        if let Some(state) = states.0.get(&robot_id.0) {
            // Get edge tangent at current s for orientation
            let tangent = map.0.eval_tangent(state.current_edge, state.position_s);
            *transform = robot_transform_from_map_pos(state.pos_3d, tangent);
        }
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib robot_render 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/robot_render.rs
git commit -m "feat(m5): chassis STL mesh replaces arrow, robot_transform_from_map_pos"
```

---

### Task 5: Full UI panels in `ui.rs`

**Files:**
- Modify: `src/bins/visualiser/src/ui.rs`

- [ ] **Step 1: Write the failing tests**

```rust
// In ui.rs tests
#[test]
fn format_gbp_iterations_label() {
    assert_eq!(fmt_iterations(20), "GBP iters: 20");
}

#[test]
fn format_d_safe_label() {
    assert_eq!(fmt_d_safe(0.30), "d_safe: 0.300 m");
}

#[test]
fn global_params_defaults() {
    let p = GlobalParams::default();
    assert_eq!(p.k, 12);
    assert_eq!(p.gbp_iterations, 20);
    assert!((p.d_safe - 0.3).abs() < 1e-6);
    assert!((p.r_comm - 2.0).abs() < 1e-6);
    assert!(!p.random_mode);
    assert!(!p.restart_requested);
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib ui 2>&1 | head -20
```

- [ ] **Step 2: Implement full global panel**

```rust
// Add to ui.rs

pub fn fmt_iterations(n: usize) -> String { format!("GBP iters: {}", n) }
pub fn fmt_d_safe(d: f32) -> String { format!("d_safe: {:.3} m", d) }

#[derive(Resource)]
pub struct GlobalParams {
    pub k:              usize,    // GBP variable timesteps horizon; default 12
    pub gbp_iterations: usize,    // iterations per step; default 20
    pub d_safe:         f32,      // clearance; default 0.3
    pub r_comm:         f32,      // comms radius; default 2.0
    pub random_mode:    bool,
    pub restart_requested: bool,  // set by Restart button; consumed by simulator task
    pub step_requested:    bool,  // set by Step button; consumed by simulator task (single step while paused)
}

impl Default for GlobalParams {
    fn default() -> Self {
        Self { k: 12, gbp_iterations: 20, d_safe: 0.3, r_comm: 2.0,
               random_mode: false, restart_requested: false, step_requested: false }
    }
}

fn draw_global_panel(
    mut ctxs:   EguiContexts,
    mut paused: ResMut<SimPaused>,
    mut params: ResMut<GlobalParams>,
) {
    let ctx = ctxs.ctx_mut();
    egui::Window::new("Global Control").show(ctx, |ui| {
        ui.horizontal(|ui| {
            let label = if paused.0 { "▶ Resume" } else { "⏸ Pause" };
            if ui.button(label).clicked() { paused.0 = !paused.0; }
            if ui.button("⟳ Restart").clicked() { params.restart_requested = true; }
            // Step advances one tick when paused
            if paused.0 && ui.button("→ Step").clicked() { params.step_requested = true; }
        });
        ui.checkbox(&mut params.random_mode, "Random mode");
        ui.add(egui::Slider::new(&mut params.k, 1..=20).text("K (timesteps)"));
        ui.add(egui::Slider::new(&mut params.gbp_iterations, 1..=50).text("GBP iterations"));
        ui.add(egui::Slider::new(&mut params.d_safe, 0.1..=1.0).text("d_safe (m)"));
        ui.add(egui::Slider::new(&mut params.r_comm, 0.5..=5.0).text("r_comm (m)"));
    });
}
```

Update `UiPlugin::build` to add `draw_global_panel` and `GlobalParams`:
```rust
app.init_resource::<GlobalParams>()
   .add_systems(Update, (draw_hud, draw_global_panel));
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib ui 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/ui.rs
git commit -m "feat(m5): full global panel with play/pause, random mode, GBP param sliders"
```

---

### Task 6: Map inspector + click-to-assign in `ui.rs`

**Files:**
- Modify: `src/bins/visualiser/src/ui.rs`
- Modify: `src/bins/visualiser/src/main.rs`

- [ ] **Step 1: Write the failing test**

```rust
// In ui.rs tests
#[test]
fn selected_node_none_by_default() {
    let s = SelectedNode::default();
    assert!(s.0.is_none());
}

#[test]
fn trajectory_command_robot_id_matches() {
    let cmd = PendingTrajectoryCommand { robot_id: 3, goal_node: NodeId(7) };
    assert_eq!(cmd.robot_id, 3);
    assert_eq!(cmd.goal_node, NodeId(7));
}
```

- [ ] **Step 1b: Run to confirm failure**

```bash
cargo test -p visualiser --lib ui 2>&1 | head -20
```

- [ ] **Step 2: Implement `SelectedNode`, `PendingTrajectoryCommand`, and map inspector panel**

```rust
// Add to ui.rs

#[derive(Resource, Default)]
pub struct SelectedNode(pub Option<NodeId>);

#[derive(Resource, Default)]
pub struct SelectedRobotId(pub Option<u32>);

/// In Bevy 0.15, types used with `add_event` must derive `Event`.
#[derive(Event)]
pub struct PendingTrajectoryCommand {
    pub robot_id:  u32,
    pub goal_node: NodeId,
}

fn draw_map_inspector(
    mut ctxs:    EguiContexts,
    selected:    Res<SelectedNode>,
    map:         Res<MapRes>,
    robot_sel:   Res<SelectedRobotId>,
    mut pending: ResMut<Events<PendingTrajectoryCommand>>,
) {
    let ctx = ctxs.ctx_mut();
    if let Some(node_id) = selected.0 {
        egui::Window::new("Node Inspector").show(ctx, |ui| {
            if let Some(idx) = map.0.node_index(node_id) {
                let node = &map.0.nodes[idx];
                ui.label(format!("Node {:?}  type={:?}", node.id, node.node_type));
                ui.label(format!("Pos: ({:.2},{:.2},{:.2})", node.position[0], node.position[1], node.position[2]));
            }
            if let Some(rid) = robot_sel.0 {
                if ui.button(format!("Assign to Robot {}", rid)).clicked() {
                    pending.send(PendingTrajectoryCommand { robot_id: rid, goal_node: node_id });
                }
            }
        });
    }
}
```

Register in `UiPlugin::build`:
```rust
app.init_resource::<SelectedNode>()
   .init_resource::<SelectedRobotId>()
   .add_event::<PendingTrajectoryCommand>()
   .add_systems(Update, (draw_hud, draw_global_panel, draw_map_inspector));
```

Node selection via click is wired as follows:

- [ ] **Step 2b: Implement `NodePickable` marker and click system**

Add to `map_scene.rs` — attach `NodePickable` to each node sphere when spawning map scene:
```rust
// In map_scene.rs — marker component for click detection
#[derive(Component)]
pub struct NodePickable(pub NodeId);
```

When spawning node sphere entities in `setup_map_scene`, add the `NodePickable` component:
```rust
commands.spawn((
    PbrBundle { mesh: sphere_mesh.clone(), material: mat.clone(),
                transform: Transform::from_xyz(bx, by, bz), ..default() },
    NodePickable(node.id),
    bevy::picking::PickableBundle::default(), // enables Bevy 0.15 built-in picking
));
```

Add to `main.rs` — system that reads click events and updates `SelectedNode`:
```rust
// In main.rs or ui.rs
fn handle_node_clicks(
    mut click_events: EventReader<Pointer<Click>>,
    pickable_query:   Query<&NodePickable>,
    mut selected:     ResMut<SelectedNode>,
) {
    for event in click_events.read() {
        if let Ok(node_pickable) = pickable_query.get(event.target) {
            selected.0 = Some(node_pickable.0);
        }
    }
}
```

Register in plugin:
```rust
app.add_systems(Update, handle_node_clicks);
```

Note: This system is a Bevy integration system and cannot be unit-tested without a Bevy `App`. The acceptance criterion is the Task 7 manual verification step: clicking a node opens the Node Inspector panel. The `SelectedNode`/`PendingTrajectoryCommand` data flow is covered by unit tests in Steps 1-1b.

- [ ] **Step 3: Run tests**

```bash
cargo test -p visualiser --lib ui 2>&1 | grep -E "test |FAILED|passed"
```

- [ ] **Step 4: Commit**

```bash
git add src/bins/visualiser/src/ui.rs src/bins/visualiser/src/main.rs
git commit -m "feat(m5): map inspector + click-to-assign goal via SelectedNode resource"
```

---

### Task 7: End-to-end verification

- [ ] **Step 1: Run all tests**

```bash
cargo test -p simulator -p visualiser 2>&1 | grep -E "test |FAILED|passed|error"
```

- [ ] **Step 2: Manual visual verification**

```bash
cargo run -p simulator -- --map maps/test_loop_map.yaml --num-robots 4 &
cargo run -p visualiser
```

Verify:
- 4 robots visible as chassis STL models in their respective colours
- Physical track, magnetic mainlines, and magnetic marker STL meshes visible in scene
- Robots run in random mode continuously without collision for a sustained run
- Global panel shows play/pause, random mode toggle, GBP iteration slider, d_safe slider
- Per-robot panel shows velocity, σ_dyn, σ_r, and active factor count
- Clicking a map node opens the Node Inspector panel
- Selecting a robot in the per-robot panel and clicking "Assign to Robot N" sends a TrajectoryCommand

- [ ] **Step 3: Commit and tag M5**

```bash
git add -A
git commit -m "feat(m5): fleet + full visualiser complete"
git tag m5-complete
```

---
