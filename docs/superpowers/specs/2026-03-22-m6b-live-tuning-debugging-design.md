# M6b: Live Tuning & Debugging — Design Spec

## Goal

Add a comprehensive settings panel with live-tunable GBP parameters, single-step simulation control, click-to-inspect variable nodes, and collision event markers. Changes propagate immediately to the running simulator via WebSocket.

## Settings Panel

Right-side egui panel replacing the current minimal Control window. Collapsible sections:

### Simulation Section
- **Pause / Play / Step** buttons. Step sends `{"command":"step"}` — simulator runs one tick then re-pauses.
- **Time scale** slider (0.1x–5x). Sends `{"command":"set_timescale", "value": N}`. Simulator adjusts tick interval (e.g., 2.0x makes 50Hz tick run at 25ms instead of 20ms effective speed).
- **Timesteps per step** — how many ticks a single "Step" advances (default 1).

### GBP Solver Section
- **M_I** (internal iterations) — number field, editable when paused
- **M_E** (external iterations) — number field, editable when paused
- **msg_damping** — slider 0.0–1.0

### Factor Sigmas Section
Per factor type (Dynamics, InterRobot, VelocityBound):
- **sigma** — slider with current value display
- **enabled** — checkbox to disable all factors of that type

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

**RobotAgent::update_config():** New method on RobotAgent that updates the stored config and propagates changes to existing factors (e.g., updates d_safe on all IR factors, sigma on dynamics factors).

## Click-to-Inspect

**Dependency:** `bevy_mod_picking` (added in M6a).

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
| Modify | `src/bins/visualiser/src/state.rs` | Add LiveParams resource, CollisionEvent handling |
| Modify | `src/bins/visualiser/src/robot_render.rs` | Collision marker spawn/fade system |
| Modify | `src/bins/visualiser/src/ws_client.rs` | Send SetParams commands, receive CollisionEvent messages |
| Modify | `src/bins/simulator/src/main.rs` | Handle set_params, step, set_timescale commands; send CollisionEvent |
| Modify | `src/bins/simulator/src/agent_runner.rs` | Watch channel for config updates, apply to running agent |
| Modify | `crates/gbp-agent/src/robot_agent.rs` | Add update_config() method |
| Modify | `config/config.toml` | Document new live-tunable params |
