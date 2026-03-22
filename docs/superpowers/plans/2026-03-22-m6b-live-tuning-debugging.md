# M6b: Live Tuning & Debugging — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **CRITICAL for subagents:** Use the Edit tool for all changes to existing files — NEVER use Write to overwrite. Do NOT modify functions/sections not mentioned in your task. Leave all other code unchanged.

**Goal:** Add settings panel with live-tunable GBP parameters, single-step simulation, time scale control, click-to-inspect variables, and collision event markers — all propagating to the running simulator via WebSocket.

**Architecture:** Settings panel mutates LiveParams resource → change detection → WebSocket → simulator watch channel → agent update_config(). Step/timescale use AtomicI32/AtomicU32. Click-to-inspect via on-demand WebSocket query. WS protocol extended with tagged message types.

**Tech Stack:** Bevy 0.18, bevy_egui 0.39, built-in bevy_picking, tokio::sync::watch

**Spec:** `docs/superpowers/specs/2026-03-22-m6b-live-tuning-debugging-design.md`

**MAGICS reference:** Clone with `git clone https://github.com/AU-Master-Thesis/magics.git /tmp/magics`. Key files:
- Settings panel: `/tmp/magics/crates/magics/src/ui/settings.rs` (1261 lines — full settings panel with live-tuning, the primary reference for this entire task)
- Factor graph viz + click-to-inspect: `/tmp/magics/crates/magics/src/planner/visualiser/factorgraphs.rs` (bevy_mod_picking on variable meshes, prints factor details)
- Collision detection: `/tmp/magics/crates/magics/src/planner/collisions.rs` (collision events, clickable markers)
- Config resource: `/tmp/magics/crates/gbp_config/src/lib.rs` (Config struct used as Bevy Resource, mutated live from UI)

**Prerequisite:** M6a complete (draw toggles, camera)

---

## Task 1: WebSocket protocol — tagged message types

**Files:**
- Modify: `src/bins/simulator/src/main.rs` — add "type" field to state messages
- Modify: `src/bins/visualiser/src/ws_client.rs` — message type discrimination
- Modify: `src/bins/visualiser/src/state.rs` — new message types

- [ ] **Step 1:** Wrap RobotStateMsg serialization in main.rs relay to include `"type":"state"` field
- [ ] **Step 2:** Update ws_client.rs to parse `serde_json::Value` first, check `"type"` field, dispatch to correct deserializer
- [ ] **Step 3:** Add CollisionEvent and InspectResponse structs to state.rs
- [ ] **Step 4:** Build and test (existing functionality must not break)
- [ ] **Step 5:** Commit

---

## Task 2: Simulation state — AtomicI32 + timescale

**Files:**
- Modify: `src/bins/simulator/src/main.rs` — replace AtomicBool pause with AtomicI32
- Modify: `src/bins/simulator/src/physics.rs` — read AtomicI32 and timescale
- Modify: `src/bins/simulator/src/agent_runner.rs` — read AtomicI32 and timescale

- [ ] **Step 1:** Replace `sim_paused: Arc<AtomicBool>` with `sim_state: Arc<AtomicI32>` (-1=run, 0=pause, N>0=ticks remaining)
- [ ] **Step 2:** Add `tick_interval_us: Arc<AtomicU32>` (default 20000 = 50Hz)
- [ ] **Step 3:** Update physics_task to check sim_state and read tick_interval_us for interval duration
- [ ] **Step 4:** Update agent_task similarly
- [ ] **Step 5:** Add command handlers for `step` (sets sim_state to N) and `set_timescale` (computes tick_interval_us)
- [ ] **Step 6:** Update visualiser pause/resume to use new protocol (`{"command":"pause"}` sets sim_state=0, resume sets -1)
- [ ] **Step 7:** Build and test
- [ ] **Step 8:** Commit

---

## Task 3: Watch channel for config propagation

**Files:**
- Modify: `src/bins/simulator/src/main.rs` — create watch channel, pass to agent tasks
- Modify: `src/bins/simulator/src/agent_runner.rs` — accept watch receiver, check on each tick

- [ ] **Step 1:** Create `tokio::sync::watch::channel(config)` in main
- [ ] **Step 2:** Pass `watch::Receiver<GbpConfig>` to each agent_task
- [ ] **Step 3:** In agent_task loop, check `watch_rx.has_changed()` and call `runner.apply_config()`
- [ ] **Step 4:** Add `set_params` command handler that updates watch channel sender
- [ ] **Step 5:** Build and test
- [ ] **Step 6:** Commit

---

## Task 4: RobotAgent::update_config()

**Files:**
- Modify: `crates/gbp-agent/src/robot_agent.rs`

- [ ] **Step 1:** Add `update_config(&mut self, config: &GbpConfig)` method
- [ ] **Step 2:** Update self.config
- [ ] **Step 3:** Walk dyn_indices — update sigma, timestep on each DynamicsFactor
- [ ] **Step 4:** Walk vel_bound_indices — update v_min, v_max, kappa, margin, max_precision
- [ ] **Step 5:** Walk ir_set — update d_safe, sigma_r, decay_alpha on each InterRobotFactor
- [ ] **Step 6:** Update graph.msg_damping
- [ ] **Step 7:** Update constraints (max_accel, max_jerk, max_speed, v_min)
- [ ] **Step 8:** Add setters on DynamicsFactor, VelocityBoundFactor, InterRobotFactor as needed
- [ ] **Step 9:** Build and test: `cargo test -p gbp-agent`
- [ ] **Step 10:** Commit

---

## Task 5: Settings panel

**Files:**
- Create: `src/bins/visualiser/src/settings_panel.rs`
- Modify: `src/bins/visualiser/src/state.rs` — add LiveParams resource
- Modify: `src/bins/visualiser/src/ui.rs` — remove old Control panel
- Modify: `src/bins/visualiser/src/main.rs` — register settings panel system

- [ ] **Step 1:** Create LiveParams resource mirroring GbpConfig fields
- [ ] **Step 2:** Create settings_panel.rs with collapsible egui sections: Simulation, GBP Solver, Factor Sigmas, Safety, Robot, Draw Toggles
- [ ] **Step 3:** Wire each slider/field to mutate LiveParams
- [ ] **Step 4:** Add `detect_param_changes` system that compares LiveParams to last-sent, sends full config over WS outbox when changed
- [ ] **Step 5:** Remove old Control panel from ui.rs (keep per-robot panels and metrics)
- [ ] **Step 6:** Build and test
- [ ] **Step 7:** Commit

---

## Task 6: Click-to-inspect (on-demand query)

**Files:**
- Create: `src/bins/visualiser/src/inspector.rs`
- Modify: `crates/gbp-agent/src/robot_agent.rs` — add inspect_variable() method
- Modify: `src/bins/simulator/src/main.rs` — handle inspect command
- Modify: `src/bins/visualiser/src/ws_client.rs` — send inspect, receive response
- Modify: `src/bins/visualiser/src/main.rs` — register inspector plugin

- [ ] **Step 1:** Add `inspect_variable(&self, k: usize) -> InspectData` to RobotAgent — returns variable beliefs, connected factors, messages
- [ ] **Step 2:** Add inspect command handler in simulator: receives robot_id + variable_k, queries agent, serializes response
- [ ] **Step 3:** Create inspector.rs: use bevy_picking to make belief dots clickable. On click, send inspect command via WS outbox.
- [ ] **Step 4:** On InspectResponse received, print detailed dump to terminal
- [ ] **Step 5:** Show floating egui popup with summary near clicked entity
- [ ] **Step 6:** Build and test
- [ ] **Step 7:** Commit

---

## Task 7: Collision event markers

**Files:**
- Modify: `src/bins/simulator/src/main.rs` — collision monitor sends CollisionEvent via tx_json
- Modify: `src/bins/visualiser/src/robot_render.rs` — spawn/fade collision markers

- [ ] **Step 1:** Give collision monitor access to tx_json. On collision, send `{"type":"collision", "robot_a":..., "pos":[...], "dist":...}`
- [ ] **Step 2:** In visualiser, on receiving CollisionEvent, spawn translucent red sphere at collision position
- [ ] **Step 3:** Add fade system: decrease alpha each frame, despawn when alpha < 0.01
- [ ] **Step 4:** Gate behind `draw.collision_markers`
- [ ] **Step 5:** Build and test
- [ ] **Step 6:** Commit

---

## Task 8: End-to-end verification

- [ ] **Step 1:** `cargo test --workspace`
- [ ] **Step 2:** Run simulator + visualiser, verify: settings panel appears, sliders change params live
- [ ] **Step 3:** Verify pause/step/resume work (step advances exactly N ticks)
- [ ] **Step 4:** Verify timescale slider changes simulation speed
- [ ] **Step 5:** Click a belief dot → verify terminal dump + egui popup
- [ ] **Step 6:** Verify collision markers appear at collision points and fade
- [ ] **Step 7:** Commit
