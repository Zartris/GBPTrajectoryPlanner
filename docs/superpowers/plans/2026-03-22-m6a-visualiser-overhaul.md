# M6a: Visualiser Overhaul — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **CRITICAL for subagents:** Use the Edit tool for all changes to existing files — NEVER use Write to overwrite. Do NOT modify functions/sections not mentioned in your task. Leave all other code unchanged.

**Goal:** Replace the fixed camera with orbit/pan/zoom/follow controls, move draw toggles to runtime egui, add metrics window, entity inspector, path traces, and new visualisation layers.

**Architecture:** New camera.rs module handles all camera logic. DrawConfig becomes runtime-mutable via egui checkboxes. Spawn-time entities (STLs, node spheres) get marker components for Visibility toggling. New gizmo layers use early-return guards. bevy-inspector-egui behind F1, metrics behind F2.

**Tech Stack:** Bevy 0.18, bevy_egui 0.39, built-in bevy_picking, bevy-inspector-egui (TBD version), bevy_infinite_grid (TBD version or manual fallback)

**Spec:** `docs/superpowers/specs/2026-03-22-m6a-visualiser-overhaul-design.md`

**MAGICS reference:** Clone with `git clone https://github.com/AU-Master-Thesis/magics.git /tmp/magics`. Key files:
- Camera: `/tmp/magics/crates/magics/src/environment/camera.rs` (CameraPlugin, orbit/pan), `/tmp/magics/crates/magics/src/input/camera.rs` (input handling)
- Follow cameras: `/tmp/magics/crates/magics/src/environment/follow_cameras.rs` (P-controller tracking, Tab cycling)
- Draw toggles: `/tmp/magics/crates/gbp_config/src/lib.rs` (DrawSection struct, DrawSetting enum)
- Settings UI: `/tmp/magics/crates/magics/src/ui/settings.rs` (lines 756-910 — draw toggle UI with bulk buttons)
- Path traces: `/tmp/magics/crates/magics/src/planner/visualiser/tracer.rs` (ring buffer, gizmo lines)
- Uncertainty: `/tmp/magics/crates/magics/src/planner/visualiser/uncertainty.rs` (ellipses at variables)
- IR factor vis: `/tmp/magics/crates/magics/src/planner/visualiser/interrobot.rs` (colored lines by proximity)
- Metrics: `/tmp/magics/crates/magics/src/ui/metrics.rs` (floating FPS/entity count window)
- Collider vis: `/tmp/magics/crates/magics/src/planner/visualiser/collider.rs` (wireframe bounding shapes)

---

## Task 1: Camera system (orbit/pan/zoom)

**Files:**
- Create: `src/bins/visualiser/src/camera.rs`
- Modify: `src/bins/visualiser/src/map_scene.rs` — remove camera spawn, extract MapBounds resource
- Modify: `src/bins/visualiser/src/main.rs` — register CameraPlugin

- [ ] **Step 1:** Create `camera.rs` with CameraMode enum (Orbit/Pan/Follow), CameraSettings resource, CameraPlugin
- [ ] **Step 2:** Implement orbit mode: mouse drag rotates around focus point, scroll zooms (distance-scaled)
- [ ] **Step 3:** Implement pan mode: WASD/arrows lateral movement, speed scales with distance
- [ ] **Step 4:** Add `C` key toggle between orbit/pan, `R` key reset to default overhead view
- [ ] **Step 5:** Add egui input guard: skip camera input when `EguiContexts::ctx().is_pointer_over_area()`
- [ ] **Step 6:** Extract `MapBounds` resource from map_scene.rs bounding box computation, use in camera reset
- [ ] **Step 7:** Remove fixed camera spawn from `spawn_map_scene`, register CameraPlugin in main.rs
- [ ] **Step 8:** Build and test: `cargo build -p visualiser`
- [ ] **Step 9:** Commit

---

## Task 2: Follow camera (per-robot, Tab to cycle)

**Files:**
- Modify: `src/bins/visualiser/src/camera.rs`
- Modify: `src/bins/visualiser/src/robot_render.rs` — RobotArrow already exists as marker

- [ ] **Step 1:** Add Follow variant to CameraMode with `followed_robot: Option<u32>`
- [ ] **Step 2:** Implement Tab key cycling through robot IDs (query RobotArrow entities)
- [ ] **Step 3:** Implement P-controller tracking: camera smoothly follows robot position with offset behind and above
- [ ] **Step 4:** Escape key returns to free camera (Orbit mode)
- [ ] **Step 5:** Build and test
- [ ] **Step 6:** Commit

---

## Task 3: Runtime draw toggles with marker components

**Files:**
- Modify: `src/bins/visualiser/src/state.rs` — expand DrawConfig, add marker components
- Modify: `src/bins/visualiser/src/map_scene.rs` — add markers to spawned entities, always spawn
- Modify: `src/bins/visualiser/src/robot_render.rs` — add markers to robot entities

- [ ] **Step 1:** Add new fields to DrawConfig: `uncertainty_bars`, `path_traces`, `comm_radius_circles`, `ir_safety_distance`, `robot_colliders`, `collision_markers`, `infinite_grid`
- [ ] **Step 2:** Add marker components: `EnvironmentMesh`, `NodeSphere`
- [ ] **Step 3:** Modify spawn_map_scene to always spawn node spheres (remove early return), add NodeSphere marker
- [ ] **Step 4:** Modify spawn_environment_stl to always spawn all STLs, add EnvironmentMesh marker with a sub-type field
- [ ] **Step 5:** Add `sync_draw_visibility` system that toggles Visibility on marker-tagged entities when DrawConfig changes
- [ ] **Step 6:** Build and test
- [ ] **Step 7:** Commit

---

## Task 4: Draw toggle UI (egui checkboxes + bulk buttons)

**Files:**
- Modify: `src/bins/visualiser/src/ui.rs`

- [ ] **Step 1:** Add a "Draw" section in the Control panel with a checkbox for each DrawConfig field
- [ ] **Step 2:** Add None/All/Flip/Reset bulk buttons
- [ ] **Step 3:** Add gizmo master toggle (disables all gizmo rendering via GizmoConfigStore)
- [ ] **Step 4:** Build and test
- [ ] **Step 5:** Commit

---

## Task 5: IR factor line color gradient

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs` — update draw_factor_links

- [ ] **Step 1:** Replace single-color IR factor lines with 3-stop lerp: red (dist < d_safe) → yellow → green (dist > 2*d_safe)
- [ ] **Step 2:** Implement inline color lerp (no colorgrad dependency): `Color::srgb(r, g, b)` interpolation based on `dist / d_safe` ratio
- [ ] **Step 3:** Add IR safety distance markers (short perpendicular lines or dots at d_safe distance along factor lines) gated by `draw.ir_safety_distance`
- [ ] **Step 4:** Build and test
- [ ] **Step 5:** Commit

---

## Task 6: Path trace history

**Files:**
- Modify: `src/bins/visualiser/src/state.rs` — add TraceHistory resource
- Modify: `src/bins/visualiser/src/robot_render.rs` — sampling + drawing systems

- [ ] **Step 1:** Add `TraceHistory` resource: `HashMap<u32, VecDeque<Vec3>>` with per-robot cap of 10000
- [ ] **Step 2:** Add `sample_trace` system: each frame, push robot's current 3D position into its trace deque
- [ ] **Step 3:** Add `draw_traces` system: draw gizmo lines through each robot's trace in its assigned color, gated by `draw.path_traces`
- [ ] **Step 4:** Add cleanup: prune entries for robot IDs not in RobotStates for 5+ seconds
- [ ] **Step 5:** Build and test
- [ ] **Step 6:** Commit

---

## Task 7: Uncertainty bars

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs`

- [ ] **Step 1:** Add `draw_uncertainty_bars` system gated by `draw.uncertainty_bars`
- [ ] **Step 2:** For each robot, at each variable k, draw a vertical bar at the variable's 3D position with height proportional to sqrt(variance). Color matches robot color with alpha.
- [ ] **Step 3:** Build and test
- [ ] **Step 4:** Commit

---

## Task 8: Robot collider wireframes

**Files:**
- Modify: `src/bins/visualiser/src/robot_render.rs`

- [ ] **Step 1:** Add `draw_robot_colliders` system gated by `draw.robot_colliders`
- [ ] **Step 2:** Draw a gizmo cuboid wireframe (CHASSIS_LENGTH × CHASSIS_WIDTH × CHASSIS_HEIGHT) at each robot's transform
- [ ] **Step 3:** Build and test
- [ ] **Step 4:** Commit

---

## Task 9: Metrics window (F2)

**Files:**
- Modify: `src/bins/visualiser/src/ui.rs`
- Modify: `src/bins/visualiser/src/state.rs` — add MetricsVisible resource
- Modify: `Cargo.toml` — add bevy_diagnostic feature if needed

- [ ] **Step 1:** Add `MetricsVisible(bool)` resource toggled by F2 key
- [ ] **Step 2:** Add floating egui window showing: FPS, frame time, robot count, total IR factor count, collision count, backend Hz
- [ ] **Step 3:** Build and test
- [ ] **Step 4:** Commit

---

## Task 10: Entity inspector (F1)

**Files:**
- Modify: `src/bins/visualiser/src/main.rs`
- Modify: `src/bins/visualiser/Cargo.toml`
- Modify: `Cargo.toml`

- [ ] **Step 1:** Add `bevy-inspector-egui` to workspace and visualiser deps (verify Bevy 0.18 compat version). If no compatible version exists, implement a minimal manual inspector showing all Bevy resources via egui.
- [ ] **Step 2:** Add InspectorVisible(bool) resource toggled by F1
- [ ] **Step 3:** Register WorldInspectorPlugin (or manual equivalent) gated by InspectorVisible
- [ ] **Step 4:** Build and test
- [ ] **Step 5:** Commit

---

## Task 11: Infinite grid

**Files:**
- Modify: `src/bins/visualiser/src/map_scene.rs` or new `grid.rs`

- [ ] **Step 1:** Try `bevy_infinite_grid` (verify Bevy 0.18 compat). If incompatible, implement a simple gizmo grid: draw N×N lines on the XZ plane centered on MapBounds.
- [ ] **Step 2:** Gate behind `draw.infinite_grid` (default off)
- [ ] **Step 3:** Build and test
- [ ] **Step 4:** Commit

---

## Task 12: End-to-end verification

- [ ] **Step 1:** `cargo test --workspace`
- [ ] **Step 2:** Run visualiser, verify: orbit/pan/zoom, Tab follow, C toggle, R reset
- [ ] **Step 3:** Verify all draw toggles work (including STL visibility toggling)
- [ ] **Step 4:** Verify metrics window (F2) and inspector (F1)
- [ ] **Step 5:** Commit
