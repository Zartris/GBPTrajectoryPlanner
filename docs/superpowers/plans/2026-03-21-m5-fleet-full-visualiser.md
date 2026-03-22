# M5: Fleet + Full Visualiser — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **CRITICAL for subagents:** Use the Edit tool for all changes to existing files — NEVER use Write to overwrite. Do NOT modify functions/sections not mentioned in your task. Leave all other code unchanged.

**Goal:** N robots (default 4, configurable via `--num-robots`) run on the full map with random trajectory reassignment. STL models for chassis and environment. Full UI panels with global controls and per-robot diagnostics.

**Architecture:** Simulator refactored from 2 hardcoded robots to a dynamic fleet loop. Each robot gets its own `AgentRunner` + `PhysicsState` + tokio tasks. Random mode: on goal arrival, A* picks a new random reachable goal. Visualiser loads STL meshes via `bevy_stl` and spawns chassis entities dynamically as new robot IDs appear. Bevy's built-in `bevy_picking` (already enabled) handles click-to-assign.

**Tech Stack:** Rust, Bevy 0.18, bevy_egui 0.39, `bevy_stl` 0.18 (new dep), tokio, `#![no_std]` core crates unchanged.

**Prerequisite:** M4 merged to main.

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Modify | `Cargo.toml` | Add `bevy_stl` dependency |
| Modify | `src/bins/simulator/src/main.rs` | N-robot fleet spawn, `--num-robots` arg, random mode, remove 2-robot hardcoding |
| Modify | `src/bins/simulator/src/agent_runner.rs` | Add `set_pos_3d` passthrough for 3D position |
| Modify | `src/bins/visualiser/src/main.rs` | Add `bevy_stl::StlPlugin` |
| Modify | `src/bins/visualiser/src/robot_render.rs` | Load chassis STL, dynamic robot entity spawn for N robots |
| Modify | `src/bins/visualiser/src/map_scene.rs` | Load environment STL models (track, mainlines, markers) |
| Modify | `src/bins/visualiser/src/ui.rs` | Global control panel, expanded per-robot panels |
| Modify | `src/bins/visualiser/src/state.rs` | No changes expected |

---

## Task 1: Add `bevy_stl` dependency and verify STL loading works

**Files:**
- Modify: `Cargo.toml` (root workspace)
- Modify: `src/bins/visualiser/src/main.rs`

- [ ] **Step 1: Add dependency**

In root `Cargo.toml`, add to `[workspace.dependencies]`:
```toml
bevy_stl = "0.18"
```

In `src/bins/visualiser/Cargo.toml`, add:
```toml
bevy_stl = { workspace = true }
```

- [ ] **Step 2: Register plugin**

In `src/bins/visualiser/src/main.rs`, add after the EguiPlugin:
```rust
.add_plugins(bevy_stl::StlPlugin)
```

- [ ] **Step 3: Build to verify**

Run: `cargo build -p visualiser`
Expected: Compiles. STL plugin registered.

- [ ] **Step 4: Commit**

```bash
git add Cargo.toml Cargo.lock src/bins/visualiser/Cargo.toml src/bins/visualiser/src/main.rs
git commit -m "feat(m5): add bevy_stl dependency for STL mesh loading"
```

---

## Task 2: Load environment STL models in map_scene

**Files:**
- Modify: `src/bins/visualiser/src/map_scene.rs`

- [ ] **Step 1: Add STL environment spawning in map_scene**

Add a new system `spawn_environment_stl` that loads and spawns the three environment meshes at world origin. Use `AssetServer::load()` with the STL paths.

```rust
// Add to map_scene.rs

/// Asset paths for environment STL models (relative to `assets/` directory).
const PHYSICAL_TRACK_STL: &str = "models/physical_track.stl";
const MAGNETIC_MAINLINES_STL: &str = "models/magnetic_mainlines.stl";
const MAGNETIC_MARKERS_STL: &str = "models/magnetic_markers.stl";

fn spawn_environment_stl(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Physical track — grey
    commands.spawn((
        Mesh3d(asset_server.load(PHYSICAL_TRACK_STL)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.45, 0.45, 0.45, 0.8),
            ..default()
        })),
        Transform::IDENTITY,
    ));

    // Magnetic mainlines — dark blue
    commands.spawn((
        Mesh3d(asset_server.load(MAGNETIC_MAINLINES_STL)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.1, 0.1, 0.7, 0.6),
            ..default()
        })),
        Transform::IDENTITY,
    ));

    // Magnetic markers — yellow
    commands.spawn((
        Mesh3d(asset_server.load(MAGNETIC_MARKERS_STL)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.8, 0.8, 0.0, 0.6),
            ..default()
        })),
        Transform::IDENTITY,
    ));
}
```

Register in `MapScenePlugin::build`:
```rust
app.add_systems(Startup, (spawn_map_scene, spawn_environment_stl));
```

**Note:** The STL models are in map coordinates. If they need the map→Bevy transform (x,z,-y), add it to the Transform. Test visually.

- [ ] **Step 2: Build and verify**

Run: `cargo build -p visualiser`

- [ ] **Step 3: Commit**

```bash
git add src/bins/visualiser/src/map_scene.rs
git commit -m "feat(m5): load environment STL models (track, mainlines, markers)"
```

---

## Task 3: Dynamic robot chassis STL spawning for N robots

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs`

Currently `spawn_robot_arrows` hardcodes 2 robots with procedural Cuboid meshes at startup. Replace with dynamic spawning: each frame, check `RobotStates` for new robot IDs and spawn a chassis STL entity for each.

- [ ] **Step 1: Replace static spawn with dynamic spawn**

Remove `spawn_robot_arrows` from Startup. Add a new Update system `spawn_new_robot_meshes` that:
1. Reads `RobotStates` for all known robot IDs
2. Queries existing `RobotArrow` entities
3. For any robot ID not yet spawned, loads `models/chassis.stl` via `AssetServer` and spawns it

```rust
const CHASSIS_STL: &str = "models/chassis.stl";

fn spawn_new_robot_meshes(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    states: Res<RobotStates>,
    query: Query<&RobotArrow>,
) {
    let existing: std::vec::Vec<u32> = query.iter().map(|a| a.robot_id).collect();
    for &id in states.0.keys() {
        if existing.contains(&id) { continue; }
        let (r, g, b) = ROBOT_COLORS.get(id as usize % ROBOT_COLORS.len())
            .copied().unwrap_or((0.5, 0.5, 0.5));
        commands.spawn((
            Mesh3d(asset_server.load(CHASSIS_STL)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(r, g, b),
                emissive: bevy::color::LinearRgba::new(r * 0.3, g * 0.3, b * 0.3, 1.0),
                ..default()
            })),
            Transform::IDENTITY,
            RobotArrow { robot_id: id },
        ));
    }
}
```

Update plugin registration:
```rust
// Remove: spawn_robot_arrows from Startup
// Add: spawn_new_robot_meshes to Update chain (before update_robot_transforms)
app.add_systems(Update, (
    drain_ws_inbox,
    spawn_new_robot_meshes,
    update_robot_transforms,
    draw_planned_path,
    draw_belief_tubes,
    draw_factor_links,
).chain());
```

- [ ] **Step 2: Build and verify**

Run: `cargo build -p visualiser`

- [ ] **Step 3: Commit**

```bash
git add src/bins/visualiser/src/robot_render.rs
git commit -m "feat(m5): dynamic chassis STL spawning for N robots"
```

---

## Task 4: N-robot fleet spawn in simulator

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

This is the biggest task. The current `main.rs` hardcodes 2 robots with scenario-specific logic. Refactor to:
1. Add `--num-robots N` CLI arg (default 4)
2. Pick N distributed start nodes from the map
3. Assign each robot a random A* goal
4. Spawn N `AgentRunner` + `PhysicsState` + tokio tasks in a loop
5. Keep `--scenario` for backward compat (merge/endcollision use 2 robots)

- [ ] **Step 1: Add CLI arg and `pick_start_nodes`**

Add to `Args`:
```rust
/// Number of robots (default 4, ignored for merge/endcollision scenarios)
#[arg(long, default_value = "4")]
num_robots: usize,
```

Add helper:
```rust
/// Pick `n` start nodes distributed evenly across map waypoint nodes.
fn pick_start_nodes(map: &Map, n: usize) -> std::vec::Vec<NodeId> {
    let waypoints: std::vec::Vec<NodeId> = map.nodes.iter()
        .filter(|node| node.node_type == NodeType::Waypoint)
        .map(|n| n.id)
        .collect();
    if waypoints.is_empty() { return vec![]; }
    let n = n.min(waypoints.len());
    let step = waypoints.len() / n;
    (0..n).map(|i| waypoints[i * step]).collect()
}

/// Pick a random goal node reachable from `start` (excluding start itself).
fn pick_random_goal(map: &Map, start: NodeId, seed: u32) -> Option<NodeId> {
    let mut reachable: std::vec::Vec<NodeId> = Vec::new();
    for node in &map.nodes {
        if node.id != start {
            if gbp_map::astar::astar(map, start, node.id).is_some() {
                reachable.push(node.id);
            }
        }
    }
    if reachable.is_empty() { return None; }
    Some(reachable[(seed as usize) % reachable.len()])
}
```

- [ ] **Step 2: Refactor main to spawn N robots in a loop**

Replace the hardcoded 2-robot setup with:
```rust
// For merge/endcollision, keep existing 2-robot logic
// For default/follow, use N-robot fleet
if args.scenario == "merge" || args.scenario == "endcollision" {
    // ... keep existing 2-robot scenario code ...
} else {
    // N-robot fleet
    let starts = pick_start_nodes(&map_arc, args.num_robots);
    info!("fleet mode: spawning {} robots", starts.len());

    for (i, &start) in starts.iter().enumerate() {
        let rx = bcast_tx.subscribe();
        let comms = SimComms::new(bcast_tx.clone(), rx);
        let mut runner = AgentRunner::new(comms, map_arc.clone(), i as u32);

        let goal = pick_random_goal(&map_arc, start, i as u32);
        let total_length = if let Some(g) = goal {
            if let Some(path) = gbp_map::astar::astar(&map_arc, start, g) {
                let traj = build_trajectory_edges(&map_arc, &path);
                info!("Robot {}: {} edges, {:.2}m ({:?}->{:?})",
                    i, traj.len(), traj.iter().map(|(_, l)| l).sum::<f32>(), start, g);
                runner.set_trajectory(traj)
            } else { 0.0 }
        } else {
            info!("Robot {}: no reachable goal from {:?}", i, start);
            0.0
        };

        let runner_arc = Arc::new(Mutex::new(runner));
        let physics = Arc::new(Mutex::new(PhysicsState::new(total_length)));

        tokio::spawn(physics::physics_task(Arc::clone(&physics)));
        tokio::spawn(agent_task(
            Arc::clone(&physics), runner_arc, tx_state.clone(), i as u32
        ));
    }
}
```

- [ ] **Step 3: Build and test**

Run: `cargo build -p simulator`
Run: `cargo test -p simulator`

- [ ] **Step 4: Commit**

```bash
git add src/bins/simulator/src/main.rs
git commit -m "feat(m5): N-robot fleet spawn with distributed start nodes and random goals"
```

---

## Task 5: Global control panel in UI

**Files:**
- Modify: `src/bins/visualiser/src/ui.rs`

Add a proper global control panel with play/pause, random mode toggle, and parameter display.

- [ ] **Step 1: Add GlobalParams resource and expanded panel**

```rust
/// Global simulation parameters (read-only display for now; live tuning deferred to M6).
#[derive(Resource)]
pub struct GlobalParams {
    pub num_robots: usize,
    pub random_mode: bool,
}

impl Default for GlobalParams {
    fn default() -> Self {
        Self { num_robots: 4, random_mode: false }
    }
}
```

Expand `draw_hud` to include:
- Performance overlay (already exists)
- Global control panel with pause/resume
- Per-robot panels showing: cmd_v, gbp_v, dist3d, edge, IR factor count (already added in M4)

Register `GlobalParams` in `UiPlugin::build`:
```rust
app.init_resource::<GlobalParams>()
```

- [ ] **Step 2: Build and verify**

Run: `cargo build -p visualiser`

- [ ] **Step 3: Commit**

```bash
git add src/bins/visualiser/src/ui.rs
git commit -m "feat(m5): global control panel with params display"
```

---

## Task 6: NURBS edge polyline caching (deferred from M1)

**Files:**
- Modify: `src/bins/visualiser/src/map_scene.rs`

`draw_edge_gizmos()` recomputes `eval_point()` 32× per edge per frame. With N robots and a full map this is a CPU bottleneck. Cache sampled polylines at startup.

- [ ] **Step 1: Add `EdgePolylines` resource**

```rust
/// Cached polylines for each edge — computed once at startup, drawn every frame.
#[derive(Resource)]
pub struct EdgePolylines {
    /// (edge_id, Vec of Bevy-space points)
    pub lines: std::vec::Vec<(EdgeId, std::vec::Vec<Vec3>)>,
}

fn build_edge_polylines(
    mut commands: Commands,
    map: Res<MapRes>,
) {
    let mut lines = std::vec::Vec::new();
    for edge in &map.0.edges {
        let n = 32;
        let len = edge.geometry.length();
        let pts: std::vec::Vec<Vec3> = (0..n).map(|i| {
            let s = (i as f32 / (n - 1) as f32) * len;
            match map.0.eval_position(edge.id, s) {
                Some(p) => map_to_bevy(p),
                None => Vec3::ZERO,
            }
        }).collect();
        lines.push((edge.id, pts));
    }
    commands.insert_resource(EdgePolylines { lines });
}
```

Update `draw_edge_gizmos` to read from cache instead of calling `eval_point`.

Register: `app.add_systems(Startup, (spawn_map_scene, spawn_environment_stl, build_edge_polylines));`

- [ ] **Step 2: Build and verify**

Run: `cargo build -p visualiser`

- [ ] **Step 3: Commit**

```bash
git add src/bins/visualiser/src/map_scene.rs
git commit -m "perf(m5): cache NURBS edge polylines at startup"
```

---

## Task 7: End-to-end verification

- [ ] **Step 1: Run all tests**

```bash
cargo test --workspace
```

- [ ] **Step 2: Visual verification — fleet mode**

```bash
cargo run -p simulator -- --map maps/test_loop_map.yaml --num-robots 4 &
sleep 1 && DISPLAY=:0 cargo run -p visualiser
```

Verify:
- 4 robots visible as STL chassis meshes in distinct colours
- Physical track, magnetic mainlines, markers visible as STL meshes
- Robots navigate along their assigned trajectories
- Per-robot HUD panels show cmd_v, gbp_v, dist3d
- No crashes, no panics

- [ ] **Step 3: Visual verification — backward compat**

```bash
cargo run -p simulator -- --map maps/test_loop_map.yaml --scenario merge &
sleep 1 && DISPLAY=:0 cargo run -p visualiser
```

Verify: merge scenario still works with 2 robots, 0 collisions.

- [ ] **Step 4: Commit and push**

```bash
git add -A
git commit -m "feat(m5): fleet + full visualiser — end-to-end verified"
git push origin feature/m5-fleet-full-visualiser
```
