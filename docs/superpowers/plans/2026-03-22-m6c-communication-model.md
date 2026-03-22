# M6c: Communication Model — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **CRITICAL for subagents:** Use the Edit tool for all changes to existing files — NEVER use Write to overwrite. Do NOT modify functions/sections not mentioned in your task. Leave all other code unchanged.

**Goal:** Add distance-based communication alongside edge-sharing filter, make edge filter togglable, add communication failure rate, and visualise comm radius circles.

**Architecture:** GbpConfig gains comm_radius, edge_filter_enabled, failure_rate. Dual gate in update_interrobot_factors: distance AND (optionally) edge-sharing. Failure rate uses xorshift hash with internal tick counter. Visualiser draws comm radius circles gated by draw toggle.

**Tech Stack:** Rust no_std (gbp-core, gbp-agent), Bevy 0.18 gizmos

**Spec:** `docs/superpowers/specs/2026-03-22-m6c-communication-model-design.md`

**Prerequisite:** M6b complete (settings panel for runtime toggles)

---

## Task 1: Add communication params to GbpConfig

**Files:**
- Modify: `crates/gbp-core/src/config.rs`
- Modify: `src/bins/simulator/src/toml_config.rs`
- Modify: `config/config.toml`

- [ ] **Step 1:** Add to GbpConfig: `comm_radius: f32` (default 10.0), `edge_filter_enabled: bool` (default true), `failure_rate: f32` (default 0.0)
- [ ] **Step 2:** Update Default impl and `default_matches_hardcoded_values` test
- [ ] **Step 3:** Add CommunicationSection to TomlConfig, update From<TomlConfig> conversion
- [ ] **Step 4:** Add `[gbp.communication]` section to config.toml with comments
- [ ] **Step 5:** Add validation: `comm_radius > 0`, `failure_rate` in 0.0..=1.0
- [ ] **Step 6:** `cargo test --workspace`
- [ ] **Step 7:** Commit

---

## Task 2: Dual-gated communication in robot_agent

**Files:**
- Modify: `crates/gbp-agent/src/robot_agent.rs`

- [ ] **Step 1:** Add `tick_count: u32` field to RobotAgent, increment at start of step()
- [ ] **Step 2:** In update_interrobot_factors: compute 3D distance to each broadcast robot BEFORE the edge check
- [ ] **Step 3:** Replace edge-only gate with dual gate: `within_range && (!edge_filter_enabled || shares_edge)`
- [ ] **Step 4:** Update min_3d_distance_to_neighbours to use same dual gate
- [ ] **Step 5:** `cargo test -p gbp-agent`
- [ ] **Step 6:** Commit

---

## Task 3: Communication failure rate

**Files:**
- Modify: `crates/gbp-agent/src/robot_agent.rs`

- [ ] **Step 1:** In step(), before iterating broadcasts, add failure rate check using xorshift hash:
```rust
let mut h = (self.robot_id as u32) ^ (self.tick_count.wrapping_mul(2654435761));
h ^= (bcast.robot_id as u32).wrapping_mul(2246822519);
h ^= h >> 16;
h = h.wrapping_mul(0x45d9f3b);
let roll = (h % 1000) as f32 / 1000.0;
if roll < self.config.failure_rate { continue; }
```
- [ ] **Step 2:** When broadcast dropped: do NOT remove existing IR factors (they persist with stale data)
- [ ] **Step 3:** Write a unit test verifying robot_id=0 doesn't always drop (the old hash was biased)
- [ ] **Step 4:** `cargo test -p gbp-agent`
- [ ] **Step 5:** Commit

---

## Task 4: Comm radius circles (visualiser)

**Files:**
- Modify: `src/bins/visualiser/src/state.rs` — DrawConfig already has comm_radius_circles field (from M6a)
- Modify: `src/bins/visualiser/src/robot_render.rs` — new draw system
- Modify: `src/bins/visualiser/src/main.rs` — parse comm_radius from config for visualiser

- [ ] **Step 1:** Parse `comm_radius` from config.toml in the visualiser (add to VisConfig/DrawToml parsing)
- [ ] **Step 2:** Store comm_radius in a new `CommConfig` resource or extend existing state
- [ ] **Step 3:** Add `draw_comm_radius_circles` system: for each robot, draw a horizontal gizmo circle at position with radius = comm_radius. Blue if robot has active IR factors, grey otherwise.
- [ ] **Step 4:** Gate behind `draw.comm_radius_circles`
- [ ] **Step 5:** Build and test
- [ ] **Step 6:** Commit

---

## Task 5: Settings panel integration

**Files:**
- Modify: `src/bins/visualiser/src/settings_panel.rs` (created in M6b)

- [ ] **Step 1:** Add Communication section to settings panel: comm_radius slider, edge_filter_enabled checkbox, failure_rate slider
- [ ] **Step 2:** Wire to LiveParams so changes propagate to simulator
- [ ] **Step 3:** Build and test
- [ ] **Step 4:** Commit

---

## Task 6: End-to-end verification

- [ ] **Step 1:** `cargo test --workspace`
- [ ] **Step 2:** Run merge scenario with `edge_filter_enabled = false` — verify robots on different edges still negotiate
- [ ] **Step 3:** Toggle edge filter ON/OFF at runtime — verify IR factors appear/disappear correctly
- [ ] **Step 4:** Set failure_rate = 0.3 — verify some broadcasts are dropped (robots react more slowly)
- [ ] **Step 5:** Verify comm radius circles visible and correctly sized
- [ ] **Step 6:** Commit
