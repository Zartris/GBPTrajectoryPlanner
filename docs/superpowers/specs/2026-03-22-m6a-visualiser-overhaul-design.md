# M6a: Visualiser Overhaul — Design Spec

## Goal

Replace the fixed camera and config-file-only draw toggles with a fully interactive visualiser: orbit/pan/zoom camera, follow cameras, runtime draw toggles in egui, metrics window, entity inspector, path traces, and new visualisation layers.

## Camera System

New module `src/bins/visualiser/src/camera.rs` replaces the fixed camera spawn in `map_scene.rs`.

**Two modes, toggled with `C` key:**
- **Orbit (default):** Mouse drag rotates around a focus point. Scroll wheel zooms (distance-scaled so movement feels natural at any zoom level). Focus point adjustable by right-click drag.
- **Pan:** WASD/arrow keys move the camera laterally. Speed scales proportionally with distance from ground.

**Additional controls:**
- `R` — Reset camera to default overhead view
- `Tab` — Cycle through follow cameras (one per robot)
- `Escape` — Return to free camera from follow mode

**Follow camera:** When Tab-cycling, camera tracks a specific robot with a P-controller (smooth tracking, not instant snap). Camera follows the robot's 3D position with a fixed offset behind and above, rotating to match the robot's heading.

**Implementation:** Raw Bevy input (no `leafwing-input-manager`). CameraMode enum (Orbit/Pan/Follow), CameraSettings resource with sensitivity/speed values.

**Input conflict with egui:** Camera input systems must check `EguiContexts::ctx().is_pointer_over_area()` and skip camera movement when the pointer is over an egui panel. Without this, orbit/pan fires while clicking UI elements.

**Map bounding box:** The current camera placement computes a map bounding box in `spawn_map_scene`. This computation should be extracted into a `MapBounds` resource so that camera.rs can reuse it for the `R` (reset) keybind.

## Draw Toggles (Runtime)

`DrawConfig` resource becomes mutable at runtime via the settings panel (M6b). Existing config.toml values set initial state; egui checkboxes override at runtime.

**Spawn-time vs runtime toggles:** STL meshes (physical_track, magnetic_mainlines, magnetic_markers) and node_spheres are spawned at startup. To toggle them at runtime, these entities get marker components (e.g. `EnvironmentMesh`, `NodeSphere`) and a system toggles their `Visibility` component when `DrawConfig` changes. They are always spawned; visibility is controlled, not existence. New gizmo-based layers are trivially toggled by early-return in draw systems.

**New toggleable layers (in addition to existing 9):**
- `uncertainty_bars` — 1D uncertainty bars along trajectory at each variable
- `path_traces` — Historical path trails per robot (ring buffer, 10K samples)
- `comm_radius_circles` — Communication range circles per robot (M6c)
- `ir_safety_distance` — d_safe distance markers at IR factor positions
- `robot_colliders` — Bounding wireframe around each robot
- `collision_markers` — Red spheres at collision event locations (fade after 3s)
- `infinite_grid` — Ground reference grid

**Layer notes:**
- `collision_markers` — data source is a new `CollisionEvent` WebSocket message from the simulator (implemented in M6b). M6a only adds the toggle infrastructure; M6b provides the data.
- `comm_radius_circles` — rendering implemented in M6c. M6a adds the toggle; M6c adds the draw system.

**Bulk buttons:** None / All / Flip / Reset (restore to config.toml defaults)
**Gizmo master toggle:** Disables all gizmo drawing at once

**IR factor lines upgrade:** Color gradient using `colorgrad` — red when dist < d_safe, transitioning through yellow to green when dist > 2*d_safe.

## Path Trace History

Per-robot `VecDeque<Vec3>` (cap 10000) stored in a new `TraceHistory` resource (HashMap<u32, VecDeque<Vec3>>). Sampled each frame from the robot's current 3D position. Drawn as gizmo lines in the robot's assigned color.

**Cleanup:** Entries for robot IDs not seen in `RobotStates` for 5+ seconds are pruned. On scenario reload (M6d), the entire TraceHistory is cleared.

## Metrics Window

Floating, togglable egui window (F2 key):
- FPS + frame time (from `FrameTimeDiagnosticsPlugin`)
- Entity count (from `EntityCountDiagnosticsPlugin`)
- Robot count (from `RobotStates.len()`)
- Active IR factor count (sum of all robots' `ir_factor_count`)
- Collision count (from collision monitor)
- Backend message rate (existing `BackendStats`)

## Entity Inspector

`bevy-inspector-egui` plugin, toggled with F1 key. Opens a dedicated egui window with entity/resource/asset browsers. Not embedded in the settings panel — separate overlay window.

## Infinite Grid

`bevy_infinite_grid` plugin. Togglable via draw config. Default: off (as requested). Provides spatial orientation reference.

## New Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| `bevy-inspector-egui` | TBD (must match Bevy 0.18) | Entity/resource browser. If no compatible release exists at implementation time, defer this feature or gate behind a feature flag. |
| `bevy_infinite_grid` | TBD (must match Bevy 0.18) | Ground reference grid. Fallback: implement a simple gizmo grid manually (~20 lines). |

**NOT needed:**
- `bevy_mod_picking` — Bevy 0.18 has built-in `bevy_picking` (already enabled in our workspace Cargo.toml feature list). Use that instead.
- `colorgrad` — The IR factor color gradient (red→yellow→green) is a simple 3-stop linear interpolation. Implement inline with `Color::srgb()` lerping (~5 lines) rather than adding an external crate.

**Bevy feature addition needed:** `bevy_diagnostic` must be added to the Bevy feature list in workspace `Cargo.toml` for `EntityCountDiagnosticsPlugin`.

All dependency versions must be verified against Bevy 0.18 compatibility at implementation time.

## File Map

| Action | File | Purpose |
|--------|------|---------|
| Create | `src/bins/visualiser/src/camera.rs` | Camera plugin (orbit/pan/zoom/follow) |
| Modify | `src/bins/visualiser/src/map_scene.rs` | Remove fixed camera spawn (moved to camera.rs) |
| Modify | `src/bins/visualiser/src/state.rs` | Add TraceHistory, CollisionMarkers, expand DrawConfig |
| Modify | `src/bins/visualiser/src/robot_render.rs` | Path trace sampling, IR line color gradient, uncertainty bars |
| Modify | `src/bins/visualiser/src/ui.rs` | Metrics window (F2), draw toggle section with bulk buttons |
| Modify | `src/bins/visualiser/src/main.rs` | Register new plugins, inspector (F1) |
| Modify | `src/bins/visualiser/Cargo.toml` | Add new dependencies |
| Modify | `Cargo.toml` | Add workspace dependencies |
