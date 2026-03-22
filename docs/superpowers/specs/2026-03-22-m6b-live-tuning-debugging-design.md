# M6b: Live Tuning & Debugging — Design Spec

## Goal

Add a comprehensive settings panel with live-tunable GBP parameters, single-step simulation control, click-to-inspect variable nodes, and collision event markers. Changes propagate immediately to the running simulator via WebSocket.

## Settings Panel

Right-side egui panel replacing the current minimal Control window. Collapsible sections:

### Simulation Section
- **Pause / Play / Step** buttons. Step sends `{"command":"step", "ticks": N}`.
- **Time scale** slider (0.1x–5x). Sends `{"command":"set_timescale", "value": N}`.
- **Timesteps per step** — how many ticks a single "Step" advances (default 1).

**Step mechanism:** Replace the current `AtomicBool` pause flag with an `AtomicI32` simulation state: `-1` = running, `0` = paused, `N > 0` = N ticks remaining. The `physics_task` (single task, drives the sim clock) checks and decrements atomically each tick. When it hits 0, simulation pauses. Agent tasks check the same atomic: run if value != 0.

**Timescale mechanism:** A separate `Arc<AtomicU32>` stores the tick interval in microseconds (default 20000 = 50Hz). `set_timescale` computes `20000 / value` and stores it. Both `physics_task` and `agent_task` read this atomic to set their `tokio::time::interval` duration each tick.

### GBP Solver Section
- **M_I** (internal iterations) — number field, editable when paused
- **M_E** (external iterations) — number field, editable when paused
- **msg_damping** — slider 0.0–1.0

### Factor Sigmas Section
Per factor type:
- **Dynamics:** `sigma_dynamics` slider + enabled checkbox
- **InterRobot:** `sigma_interrobot` slider + enabled checkbox
- **VelocityBound:** `vb_kappa`, `vb_margin`, `vb_max_precision` sliders + enabled checkbox (VelocityBound has no single sigma — expose actual BIPM parameters)

**Factor enable/disable:** Disabling a factor type sets all factors of that type to `is_active = false` via `update_config()`. This uses the existing `Factor::is_active()` mechanism — no core trait changes needed. Re-enabling sets them back to `true`.

### Safety Section
- **d_safe** — slider
- **activation_range** — slider
- **front_damping** — slider 0.0–1.0
- **decay_alpha** — slider

### Robot Section
- **max_speed** — slider
- **max_accel** — slider
- **v_min** — slider

### Draw Toggles Section
(From M6a — all runtime toggles, bulk buttons, gizmo master toggle)

## Data Flow for Live Tuning

```
Settings Panel (egui)
    |
    | mutates LiveParams resource
    |
LiveParams change detection system (each frame)
    |
    | if changed: serialize to JSON
    |
WebSocket outbox → simulator
    |
    | simulator receives SetParams command
    |
tokio::sync::watch channel
    |
    | all AgentRunner instances pick up new GbpConfig on next tick
    |
GbpConfig applied to RobotAgent, factors, constraints
```

**LiveParams resource:** A Bevy resource mirroring all tunable fields from GbpConfig. The settings panel mutates it directly. A `detect_param_changes` system compares LiveParams against the last-sent values and sends a delta over WebSocket only when something changed.

**Simulator handling:** The existing `cmd_rx` channel receives a `{"command":"set_params", "params": {...}}` JSON message. The command handler updates the shared GbpConfig in a `tokio::sync::watch` channel. Each `agent_task` checks `watch_rx.has_changed()` at the top of its tick loop and calls `agent.update_config(&new_config)`.

**RobotAgent::update_config():** New method on RobotAgent that:
1. Updates the stored `self.config` field
2. Walks all dynamics factors (via `dyn_indices`) — updates sigma, timestep
3. Walks all velocity bound factors (via `vel_bound_indices`) — updates v_min, v_max, kappa, margin, max_precision
4. Walks all IR factors (via `ir_set`) — updates d_safe, sigma_r, decay_alpha
5. Updates `graph.msg_damping`
6. Updates `constraints` (max_accel, max_jerk, max_speed, v_min)

This is the most complex part of the implementation — every factor instance must be individually updated since the config values are baked into factor fields at construction time.

**Param change messages:** Send the full GbpConfig on any change (not a delta). The payload is ~20 floats — trivially small. Simplifies both serialization and deserialization.

## Click-to-Inspect

**Dependency:** Built-in `bevy_picking` (already enabled in our Bevy feature list).

**Clickable entities:**
- Variable node meshes (the belief dots along trajectories)
- Robot cuboids

**On click — terminal dump:**
```
=== Variable k=3, Robot 0 ===
  mean: 4.52    variance: 0.23
  eta: 19.65    lambda: 4.35
  prior_eta: 0.00    prior_lambda: 0.01
  Cavity (minus IR): mean=4.50, var=0.25
  Connected factors:
    [0] Dynamics k=[2,3]: v_nom=2.0, sigma=0.5, msg_eta=8.2, msg_lambda=4.0
    [1] Dynamics k=[3,4]: v_nom=2.0, sigma=0.5, msg_eta=9.1, msg_lambda=4.0
    [2] VelocityBound k=[2,3]: v_max=2.5, msg_eta=0.0, msg_lambda=0.0
    [3] InterRobot k=3 (Robot 1): dist=1.82, d_safe=1.3, prec=12.3, jac_a=-0.7
```

**On click — egui popup:**
Floating window near the clicked entity showing:
- Mean, variance, cavity mean
- Factor count + list of factor types
- For IR factors: distance, d_safe, precision
- Dismiss by clicking elsewhere

**Data path for inspect:** On-demand query. Visualiser sends `{"command":"inspect", "robot": 0, "variable": 3}`. Simulator serializes the full variable + factor state as a JSON response and sends it back over the same WebSocket. The visualiser parses and displays it. This avoids streaming per-factor data every tick (too expensive).

## WebSocket Protocol

The current protocol is simple: simulator sends `RobotStateMsg` JSON, visualiser sends command JSON. M6b adds new message types in both directions.

**Message discrimination:** All simulator→visualiser messages get a `"type"` field:
```json
{"type": "state", ...}       // RobotStateMsg (existing, add type field)
{"type": "collision", ...}   // CollisionEvent (new)
{"type": "inspect", ...}     // InspectResponse (new, on-demand)
{"type": "log", ...}         // LogMsg (M6d)
```

The visualiser WS client first checks the `"type"` field via `serde_json::Value`, then deserializes to the correct struct. Unrecognized types are logged and skipped.

The simulator's collision monitor needs access to `tx_json` to send CollisionEvent messages to connected visualisers.

## Collision Markers

When the N-robot collision monitor detects an overlap, it sends a `CollisionEvent` message over WebSocket:
```json
{"type": "collision", "robot_a": 0, "robot_b": 1, "pos": [3.5, 9.8, 0.0], "dist": 0.95}
```

The visualiser spawns a translucent red sphere at the collision midpoint. The sphere fades out over 3 seconds (alpha decreases each frame) then despawns.

Collision markers are a toggleable draw layer.

## File Map

| Action | File | Purpose |
|--------|------|---------|
| Create | `src/bins/visualiser/src/settings_panel.rs` | Full settings panel (replaces minimal Control window in ui.rs) |
| Create | `src/bins/visualiser/src/inspector.rs` | Click-to-inspect system (picking callbacks, terminal dump, egui popup) |
| Modify | `src/bins/visualiser/src/ui.rs` | Remove old Control panel, keep per-robot panels and metrics |
| Modify | `src/bins/visualiser/src/state.rs` | Add LiveParams resource, CollisionEvent, InspectResponse handling |
| Modify | `src/bins/visualiser/src/robot_render.rs` | Collision marker spawn/fade system |
| Modify | `src/bins/visualiser/src/ws_client.rs` | Message type discrimination, send SetParams/Inspect commands, receive CollisionEvent/InspectResponse |
| Modify | `src/bins/simulator/src/main.rs` | Handle set_params, step, set_timescale, inspect commands; AtomicI32 sim state; timescale atomic; send CollisionEvent via tx_json |
| Modify | `src/bins/simulator/src/agent_runner.rs` | Watch channel for config updates, apply to running agent |
| Modify | `src/bins/simulator/src/physics.rs` | Read timescale atomic for tick interval, read AtomicI32 sim state |
| Modify | `src/bins/simulator/src/ws_server.rs` | Route inspect commands to correct agent |
| Modify | `crates/gbp-agent/src/robot_agent.rs` | Add update_config() method, add inspect_variable() method for on-demand dumps |
| Modify | `config/config.toml` | Document new live-tunable params |
