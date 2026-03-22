# M6d: Simulator Robustness — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **CRITICAL for subagents:** Use the Edit tool for all changes to existing files — NEVER use Write to overwrite. Do NOT modify functions/sections not mentioned in your task. Leave all other code unchanged.
>
> **MAGICS reference:** Clone `/tmp/magics` or `git clone https://github.com/AU-Master-Thesis/magics.git /tmp/magics` for cross-reference. Key files noted per task.

**Goal:** Blocked edge handling with A* replanning, scenario/map reload, despawn-at-goal, loop mode (goal replanning), and log panel.

**Architecture:** `astar_filtered` in gbp-map for blocked edge routing. CancellationToken for task lifecycle. Loop mode swaps fresh AgentRunner into Arc<Mutex>. SimConfig (simulator-only) for despawn/loop settings. Log panel via tagged WebSocket messages.

**Tech Stack:** tokio, tokio_util (CancellationToken), heapless, Bevy 0.18, bevy_egui

**Spec:** `docs/superpowers/specs/2026-03-22-m6d-simulator-robustness-design.md`

**Prerequisite:** M6b complete (WebSocket protocol with tagged messages, settings panel)

---

## Task 1: astar_filtered in gbp-map

**Files:**
- Modify: `crates/gbp-map/src/astar.rs`

**MAGICS reference:** `/tmp/magics/crates/magics/src/factorgraph/factorgraph.rs` — see `delete_interrobot_factors_connected_to()` for how MAGICS handles dynamic graph changes.

- [ ] **Step 1:** Write test `astar_filtered_skips_blocked_edge`: two-edge map, filter out edge 0, assert no path
- [ ] **Step 2:** Implement `astar_filtered<F>(map, start, goal, allow_edge: F)` — copy A* body, add `if !allow_edge(edge.id) { continue; }` when expanding neighbours
- [ ] **Step 3:** Refactor existing `astar()` as wrapper: `astar_filtered(map, start, goal, |_| true)`
- [ ] **Step 4:** `cargo test -p gbp-map`
- [ ] **Step 5:** Commit

---

## Task 2: SimConfig and TOML parsing

**Files:**
- Create: `src/bins/simulator/src/sim_config.rs`
- Modify: `src/bins/simulator/src/toml_config.rs`
- Modify: `config/config.toml`

- [ ] **Step 1:** Create SimConfig struct: `despawn_at_goal: bool` (default false), `loop_mode: bool` (default false)
- [ ] **Step 2:** Add `[simulation]` section parsing to toml_config.rs
- [ ] **Step 3:** Add `[simulation]` section to config.toml with comments
- [ ] **Step 4:** Write tests for SimConfig parsing (defaults, overrides)
- [ ] **Step 5:** `cargo test -p simulator`
- [ ] **Step 6:** Commit

---

## Task 3: Goal node tracking + loop mode

**Files:**
- Modify: `src/bins/simulator/src/agent_runner.rs` — goal_node field, loop mode logic
- Modify: `crates/gbp-agent/src/robot_agent.rs` — goal_node field + accessor
- Modify: `src/bins/simulator/src/main.rs` — pass SimConfig, loop mode in agent_task
- Modify: `src/bins/simulator/Cargo.toml` — add tokio_util

- [ ] **Step 1:** Add `goal_node: NodeId` field to AgentRunner, set during trajectory assignment
- [ ] **Step 2:** Add `current_goal_node() -> NodeId` accessor
- [ ] **Step 3:** In agent_task, after each tick: check `physics.at_goal() && sim_config.loop_mode`
- [ ] **Step 4:** On loop trigger: pick_random_goal from current goal node, A* path, construct fresh AgentRunner, swap into Arc<Mutex>, reset physics
- [ ] **Step 5:** Write test: verify goal_node is set correctly after set_trajectory
- [ ] **Step 6:** `cargo test --workspace`
- [ ] **Step 7:** Commit

---

## Task 4: Despawn at goal

**Files:**
- Modify: `src/bins/simulator/src/agent_runner.rs`

- [ ] **Step 1:** In agent_task, check `physics.at_goal() && sim_config.despawn_at_goal && !sim_config.loop_mode`
- [ ] **Step 2:** On despawn: break the agent_task loop, set velocity=0, send final RobotStateMsg
- [ ] **Step 3:** Build and test
- [ ] **Step 4:** Commit

---

## Task 5: CancellationToken for task lifecycle

**Files:**
- Modify: `src/bins/simulator/src/main.rs`
- Modify: `src/bins/simulator/src/physics.rs`
- Modify: `src/bins/simulator/src/agent_runner.rs`
- Modify: `src/bins/simulator/Cargo.toml`

- [ ] **Step 1:** Add `tokio_util` to workspace and simulator deps
- [ ] **Step 2:** Create `CancellationToken` in main, pass to all physics_task and agent_task
- [ ] **Step 3:** In physics_task loop: `tokio::select! { _ = token.cancelled() => break, _ = ticker.tick() => { ... } }`
- [ ] **Step 4:** Same pattern in agent_task
- [ ] **Step 5:** Store all JoinHandles in a Vec for clean shutdown
- [ ] **Step 6:** Build and test
- [ ] **Step 7:** Commit

---

## Task 6: Blocked edges

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1:** Add `BlockedEdges: Arc<Mutex<heapless::Vec<EdgeId, 16>>>` shared state
- [ ] **Step 2:** Add command handlers for `block_edge` and `unblock_edge`
- [ ] **Step 3:** On block: add EdgeId, trigger A* replan for affected robots using `astar_filtered`
- [ ] **Step 4:** Fallback: if replan returns None (goal unreachable), log warning, robot stops at nearest node
- [ ] **Step 5:** On unblock: remove EdgeId, re-trigger planning for stopped robots
- [ ] **Step 6:** Build and test
- [ ] **Step 7:** Commit

---

## Task 7: Scenario and map reload

**Files:**
- Modify: `src/bins/simulator/src/main.rs`

- [ ] **Step 1:** Add `reload_scenario` command handler
- [ ] **Step 2:** Implementation: cancel all tasks (CancellationToken), await JoinHandles, re-read scenario+map, respawn everything with new CancellationToken
- [ ] **Step 3:** `reload_map` command handler: same as reload_scenario (due to raw pointer safety)
- [ ] **Step 4:** Send `{"type":"map_changed"}` to visualiser after reload
- [ ] **Step 5:** Build and test
- [ ] **Step 6:** Commit

---

## Task 8: Log panel

**Files:**
- Modify: `src/bins/visualiser/src/state.rs` — LogBuffer, LogEntry
- Modify: `src/bins/visualiser/src/ui.rs` — log panel window
- Modify: `src/bins/visualiser/src/ws_client.rs` — receive LogMsg
- Modify: `src/bins/simulator/src/main.rs` — send LogMsg on events

**MAGICS reference:** `/tmp/magics/crates/magics/src/ui/metrics.rs` — see how MAGICS displays diagnostic data in floating windows.

- [ ] **Step 1:** Add LogEntry struct (timestamp, level, text) and LogBuffer resource (VecDeque<LogEntry>, cap 200)
- [ ] **Step 2:** Add log panel egui window: scrollable, stick-to-bottom, color-coded by level, filter buttons, clear button
- [ ] **Step 3:** In ws_client.rs: handle `"type":"log"` messages, push to LogBuffer
- [ ] **Step 4:** In simulator main.rs: send LogMsg on block/unblock, replan, reload, param change, despawn, loop replan
- [ ] **Step 5:** Build and test
- [ ] **Step 6:** Commit

---

## Task 9: Blocked edge visualisation

**Files:**
- Modify: `src/bins/visualiser/src/state.rs` — BlockedEdgesState resource
- Modify: `src/bins/visualiser/src/map_scene.rs` — blocked edge coloring

- [ ] **Step 1:** Add BlockedEdgesState resource (HashSet<u32> of blocked edge IDs)
- [ ] **Step 2:** Simulator sends `{"type":"blocked_edges", "edges":[5,12]}` periodically or on change
- [ ] **Step 3:** In draw_edge_gizmos: render blocked edges in red instead of yellow
- [ ] **Step 4:** Build and test
- [ ] **Step 5:** Commit

---

## Task 10: Settings panel integration

**Files:**
- Modify: `src/bins/visualiser/src/settings_panel.rs` (from M6b)

- [ ] **Step 1:** Add Simulation section: despawn_at_goal checkbox, loop_mode checkbox
- [ ] **Step 2:** Add Edge Management section: list edges, block/unblock buttons
- [ ] **Step 3:** Add Reload buttons: "Reload Scenario", "Reload Map"
- [ ] **Step 4:** Build and test
- [ ] **Step 5:** Commit

---

## Task 11: End-to-end verification

- [ ] **Step 1:** `cargo test --workspace`
- [ ] **Step 2:** Run fleet_4, enable loop_mode → verify robots replan on goal arrival
- [ ] **Step 3:** Block an edge → verify affected robots replan around it
- [ ] **Step 4:** Unblock → verify routing resumes
- [ ] **Step 5:** Reload scenario → verify full reset
- [ ] **Step 6:** Verify log panel shows all events
- [ ] **Step 7:** Commit
