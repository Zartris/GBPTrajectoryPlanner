# GBPTrajectoryPlanner — Milestone Design

**Date:** 2026-03-19
**Status:** Approved

## Summary

Concrete implementation milestones for GBPTrajectoryPlanner. Priority: full simulation first (simulation + visualiser), ESP32/bridge second. Each simulation milestone ends with a visually observable, working state in the Bevy visualiser. Verification bar: compiles + unit/integration tests + manual visual confirmation.

## Milestone Structure

**Approach:** Hybrid — one foundation milestone (horizontal layer, crates only), then vertical slices (each ends with a demo-able visualiser state), then ESP32 milestones.

**Simulation phase:** M0–M6
**ESP32/bridge phase:** M7–M9

## Directory Layout

Canonical layout per `CLAUDE.md`:
- `crates/gbp-core`, `crates/gbp-agent`, `crates/gbp-comms`, `crates/gbp-map` — no_std library crates
- `src/bins/simulator`, `src/bins/bridge`, `src/bins/visualiser` — desktop binaries, root workspace
- `firmware/src/bins/esp32c5` — ESP32 firmware, separate workspace

All `cargo` commands for the desktop crates/bins run from `/repo`. For firmware, `cd /repo/firmware` first.

---

## M0 — Foundation Crates

**Goal:** All four `no_std` crates built, tested, and solid before any simulator or visualiser code is written.

### What gets built

**Capacity constants (locked at M0)**

```rust
pub const MAX_NODES:          usize = 64;
pub const MAX_EDGES:          usize = 96;
pub const MAX_CONTROL_POINTS: usize = 16;
pub const MAX_KNOTS:          usize = 32;
pub const ARC_TABLE_SAMPLES:  usize = 64;
pub const MAX_HORIZON:        usize = 12;   // K timesteps
pub const MAX_NEIGHBOURS:     usize = 8;    // max robots tracked
pub const MAX_PATH_EDGES:     usize = 32;
```

These drive all `heapless` collection sizes. Changing them after M0 requires a memory budget re-check.

**`gbp-map`**
- `Map`, `Node`, `Edge`, `EdgeGeometry` (line + NURBS), `SpeedProfile`, `Safety`
- NURBS Cox-de Boor evaluation, `ArcLengthTable` (64-sample, s↔t mapping)
- Tangent via central difference
- A* path planning (traversal-time cost, Euclidean admissible heuristic, node-type cost hints)
- YAML parser (feature-gated `parse`, PC only)
- `postcard` round-trip serialization for binary map delivery

**`gbp-core`**
- `Factor` trait, `LinearizedFactor`, `VariableNode`
- `FactorGraph<K, F>` with `add_factor` / `remove_factor` (O(1) swap-remove) / `iterate`
- `DynamicsFactor`: trapezoidal velocity profile, velocity prior, residual + Jacobian
- `InterRobotFactor`: 3D→1D Jacobian projection, precision matrix

**`gbp-comms`**
- All message types: `ObservationUpdate`, `RobotBroadcast`, `RobotStateMsg`, `TrajectoryCommand`, `ParameterUpdate`
- `CommsInterface` trait (no-std)
- `GBPTimestep`, `RobotSource`, `ParameterTarget`

**`gbp-agent`**
- `RobotAgent`: `step()`, `update_interrobot_factors()`
- `InterRobotFactorSet`: swap-remove index tracking, patches moved-index on every removal
- Trajectory edge-sequence traversal: `global_s → (edge_id, local_s)`
- Edge transition logic (advance to next edge when `s >= edge.length`)

### Verification
- `cargo test -p gbp-map -p gbp-core -p gbp-comms -p gbp-agent`
- `cargo build -p gbp-map -p gbp-core -p gbp-comms -p gbp-agent --target riscv32imac-unknown-none-elf --no-default-features` — confirms `no_std` without `alloc` (run from `/repo`, requires the target installed)
- Key tests:
  - NURBS eval accuracy vs. known control point positions
  - Arc-length table interpolation error on a curved edge
  - A* finds correct shortest path on a small hand-crafted graph
  - `DynamicsFactor` residual and Jacobian values at known inputs
  - `InterRobotFactor` precision matrix is symmetric and correct
  - `InterRobotFactorSet::remove` patches indices correctly after swap-remove
  - Trapezoidal profile: `v_nom = 0` at `s = edge_length`, full nominal well before end
  - Near-capacity collections: construct a `Map` with exactly `MAX_EDGES` edges; confirm no panic
  - `postcard` round-trip: serialize a `Map` to bytes, deserialize, compare all fields

---

## M1 — 1 Robot, 1 Edge, Visible

**Goal:** Minimal end-to-end stack. One robot travels a straight edge at nominal velocity, visible in the Bevy visualiser.

### What gets built

**`simulator`** crate
- Tokio async loop, single `RobotAgent` in-process
- `SimComms` stub (no-op broadcast — no GBP negotiation yet)
- Physics integration: `s += v·dt` at 50 Hz
- `axum` WebSocket server sending `RobotStateMsg` at 20 Hz

**`visualiser`** crate
- Bevy window with `ws_client` connecting to simulator WebSocket
- Map scene: edges as 3D line meshes, node spheres coloured by `NodeType`
- Robot rendered as arrow mesh at `position_s`, oriented along edge tangent
- No GBP running yet — robot moves at `v_nom` directly

### Verification
- Robot travels from start to end of a straight line edge at nominal speed
- Arrow visible and position advances smoothly in visualiser
- Unit test: physics integration math (`s` increments correctly over N steps)

---

## M2 — 1 Robot, Full Route

**Goal:** A* path planning, multi-edge trajectory, trapezoidal deceleration to goal.

### What gets built

**Simulator**
- A* runs on `TrajectoryCommand` receipt, produces multi-edge route
- Edge transitions: `s` wraps at `edge.length`, robot advances to next edge
- `DynamicsFactor` active with proper `v_nom(s)`: tapered only on the final edge of planned trajectory, nominal on all intermediate edges

**Visualiser**
- Dashed planned-path line ahead of robot following `planned_edges`
- Minimal global panel: play/pause, current speed readout
- NURBS edges rendered as sampled curves (N=32 points per edge)

### Verification
- Robot assigned a goal across the full `test_loop_map.yaml`, including NURBS and ramp edges (z varies 0→1.545 m)
- Robot decelerates smoothly to zero at goal node
- Unit test: trapezoidal profile `v_nom(s)` values at `s=0`, `s=edge_length/2`, `s=edge_length`

---

## M3 — 2 Robots, Rear-End Avoidance

**Goal:** GBP fully active for the first time. Two robots negotiate on the same edge.

### What gets built

**Simulator**
- `SimComms`: in-process broadcast of `RobotBroadcast` to all other agents each step
- `RobotAgent::update_interrobot_factors()`: spawns `InterRobotFactor` when both robots are on the same edge, removes when they no longer share any planned edge
- GBP `iterate()` runs N times per step

**Visualiser**
- Belief tubes: evaluate `map.eval_position(edge, mean_k)` for k=0..K, tube radius = `sqrt(variance_k)`, recomputed each frame
- Line gizmos between robots that share an active `InterRobotFactor`
- Per-robot panel: current velocity, `σ_dyn`, `σ_r`, active factor count

### Verification
- Robot B placed behind Robot A on same edge, both assigned goals past the edge end
- B slows to maintain `d_safe` clearance, A continues at nominal
- Integration test: log the 3D distance between robots at every step; assert minimum observed distance >= `d_safe` (0.3 m from `test_loop_map.yaml`) over the full run
- Belief tubes visible and shrinking/expanding correctly
- Unit test: `InterRobotFactorSet::remove` with 3 entries — verify index patching after each removal

---

## M4 — Merge Collision Avoidance

**Goal:** Factor gating via planned edge set intersection. Robots on different incoming edges negotiate before a merge node.

### What gets built

**Simulator / Agent**
- Planned edge set intersection in `update_interrobot_factors()`: spawn inter-robot factors for all timesteps where planned edges overlap, not just the current edge
- Each robot broadcasts `planned_edges` in `RobotBroadcast`
- One robot yields, both pass the merge node without collision

**Visualiser**
- Full `planned_edges` ahead shown as dashed route per robot
- Shared edges highlighted in a distinct colour when a factor is active for that edge
- Factor lines visible between robots whose planned paths intersect

### Verification
- Two robots assigned routes converging at a merge node simultaneously
- One robot holds back (belief tube shows near-zero velocity), other passes through
- After leader clears the merge, follower accelerates through
- Factor lifecycle assertion: log the simulator step at which the inter-robot factor is first spawned (must be before either robot reaches the shared edge) and the step at which it is removed (must be after the leader has fully cleared the merge node, i.e. its `position_s` has advanced past `edge.length` on the post-merge edge)
- Integration test: minimum 3D distance between the two robots over the full scenario must be >= `d_safe`

---

## M5 — Fleet + Full Visualiser

**Goal:** N robots on the full map, full UI, realistic 3D scene with STL models.

### What gets built

**Simulator**
- Spawn N agents (default 4, configurable) at startup at distributed start nodes
- Random trajectory mode: reassign random reachable goal on arrival at current goal
- `TrajectoryCommand` routed by `robot_id`

**Visualiser scene upgrade**
- Replace arrow mesh with `assets/models/chassis.stl` for robot rendering
- Overlay `assets/models/physical_track.stl` + `assets/models/magnetic_mainlines.stl` as environment geometry
- Place `assets/models/magnetic_markers.stl` instances at fiducial positions from map

**Full UI panels**
- Global panel: play/pause/restart/step, random mode toggle, global GBP params (`K`, iterations, `d_safe`, `r_comm`)
- Robot panel: per-robot status (edge, `position_s`, velocity), `σ_dyn`/`σ_r` sliders, trajectory assignment input
- Map panel: click node or edge in 3D view → inspect properties in sidebar
- Trajectory input: click any node in 3D view → assign as goal → `TrajectoryCommand` → A*

### Verification
- 4 robots run in random mode for an extended continuous run without collision
- STL models visible and correctly positioned on the physical track geometry
- Clicking a node assigns it as a goal to the selected robot

---

## M6 — Robustness + Polish

**Goal:** Blocked edge handling, live parameter tuning, map live-reload, log panel.

> Note: map live-reload is listed as future work in `gbp_merge_design.md`. It is promoted to M6 here because it is required for the robustness goal (recover from map changes without losing in-flight robots) and because the simulator has direct file access — no Bevy asset hot-reload complexity needed on the server side.

### What gets built

**Simulator**
- Blocked edge API: mark an edge unavailable, trigger A* replanning on all affected agents (agents currently on that edge gracefully abort to nearest valid node)
- `ParameterUpdate` handling: changes to `σ_dyn`, `σ_r`, `d_safe` apply immediately to running agents
- Map live-reload: reload `test_loop_map.yaml` from disk without simulator restart

**Visualiser**
- Block/unblock edge via map panel or a dedicated control
- Log panel: timestamped messages from simulator (foundation for defmt pipeline in M8)
- Parameter sliders in robot panel send `ParameterUpdate` messages live

### Verification
- Block an edge mid-run; affected robots replan around it
- Adjust `d_safe` slider; clearance change visible in belief tubes within one update cycle
- Reload map while robots are running; robots recover to valid positions

---

## M7 — GBP Agent on Firmware

**Goal:** ESP32 firmware running a real `RobotAgent`. First time GBP runs on hardware.

### What gets built

- Wire `gbp-core`, `gbp-agent`, `gbp-comms`, `gbp-map` into firmware workspace (`no "parse"` feature)
- `physics_task`: arc-length integration replacing the current stub, reads velocity from Embassy channel
- `gbp_task`: `RobotAgent::step()` loop receiving `ObservationUpdate` from `physics_task`
- `vis_task`: serialize `ESPNowPacket` (148 bytes at K=12) and send to bridge via ESP-NOW
- Map delivered as `postcard`-serialized binary baked into firmware at build time

### Verification
- Single ESP32 running GBP; `defmt` logs show belief means updating each step
- Velocity output non-zero and within expected range
- Memory budget check: `cargo size` output reviewed against the budget table in `gbp_merge_design.md`; all components fit within ~250 KB SRAM
- `cargo clippy` passes in firmware workspace with no warnings; `clippy::mem_forget` and `clippy::large_stack_frames` deny lints remain active

---

## M8 — Bridge + Single Hardware Robot

**Goal:** One ESP32 visible in the visualiser alongside simulated robots.

### What gets built

- `bridge` crate: UDP listener for `ESPNowPacket`, translates to `RobotStateMsg`, forwards to visualiser WebSocket
- Routes `TrajectoryCommand` from visualiser back to ESP32 (A* runs on bridge side)
- Hardware robot rendered orange (`RobotSource::Hardware`), simulated robots blue
- defmt log pipeline: bridge decodes defmt frames using ELF symbol table → decoded strings → WebSocket → visualiser log panel

### Verification
- One ESP32 + two simulated robots all visible in visualiser simultaneously
- Hardware robot's belief tubes and factor lines appear correctly
- Hardware robot negotiates with simulated neighbours (inter-robot factors spawn between hardware and simulated robots)
- defmt logs from ESP32 appear in visualiser log panel
- A* location check: `nm` or `cargo size --format=sysv` on the ESP32 binary confirms no A* symbols are present (A* must run only on the bridge, not the firmware)

---

## M9 — Hybrid Fleet + Progressive Bring-Up

**Goal:** Full hybrid mode. Migrate robots from simulation to hardware one at a time without disrupting the fleet.

### What gets built

- `--sim-ids` / `--esp-ids` routing in bridge; moving a `robot_id` between lists migrates that robot live
- Time sync monitoring: log a warning if simulated and hardware robot clocks diverge by more than a threshold
- Robot ID assignment documented: current MAC-based approach, NVS flash option noted as future work

### Verification
- Start with 4 simulated robots; migrate 2 to hardware one at a time
- Fleet continues negotiating without interruption during each migration
- Mixed fleet (2 sim + 2 hardware) runs in random trajectory mode without collision
- Clock drift check: bridge logs a warning when the timestamp difference between a simulated robot and a hardware robot exceeds 50 ms; no such warning observed during a 60-second run under normal conditions

---

## Dependency Order

```
M0 → M1 → M2 → M3 → M4 → M5 → M6 → M7 → M8 → M9
```

Each milestone depends on all prior milestones. No parallel tracks — the system builds up one layer at a time.

---

## Assets

| File | Used in |
|---|---|
| `assets/models/chassis.stl` | M5 — robot mesh |
| `assets/models/physical_track.stl` | M5 — environment |
| `assets/models/magnetic_mainlines.stl` | M5 — environment |
| `assets/models/magnetic_markers.stl` | M5 — fiducial positions |
| `maps/test_loop_map.yaml` | M0 (A* tests), M2 onwards |

---

## Open Questions (post-M9)

- Arc-length table resolution: validate `ARC_TABLE_SAMPLES=64` interpolation error on sharpest NURBS curves (addressed partially by M0 near-capacity test, full tolerance validation deferred)
- A* on ESP32: map fits in RAM; on-device replanning is feasible future work
- `σ_dyn` per edge type: ramps may warrant tighter dynamics than flat straights
- Hybrid time sync: 50 ms threshold in M9 is a starting point; full alignment strategy (NTP, PTP, or bridge-side correction) deferred if drift causes issues in practice
- Robot ID assignment: MAC-based approach works; NVS flash or bridge-assigned IDs at boot are future options
