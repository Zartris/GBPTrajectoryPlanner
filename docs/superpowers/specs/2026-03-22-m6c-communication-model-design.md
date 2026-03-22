# M6c: Communication Model — Design Spec

## Goal

Add distance-based communication alongside the existing edge-sharing filter. Make the edge filter togglable so we can diagnose whether it causes merge collision gaps. Add communication failure rate simulation and visual comm radius circles.

## Current Model (Problem)

In `robot_agent.rs::update_interrobot_factors()`:
```rust
let shares_edge = bcast.planned_edges.iter().any(|e| my_edges.contains(e))
    || my_edges.contains(&bcast.current_edge);
if !shares_edge { continue; }
```

Two robots 2m apart on different edges with no planned edge overlap are completely invisible to each other. This can cause:
- Merge collision gaps when edge transitions briefly break the shared-edge condition
- Robots on parallel/converging tracks that should negotiate but don't

## New Dual-Gated Model

A robot is a communication partner if:
1. **Distance gate:** 3D distance < `comm_radius` — always checked
2. **Edge gate (optional):** shares at least one planned edge — togglable via `edge_filter_enabled`

When `edge_filter_enabled = true`: both conditions must be true (current + distance).
When `edge_filter_enabled = false`: only distance matters (any nearby robot interacts).

```rust
// New logic in update_interrobot_factors():
let within_range = dist_3d < self.config.comm_radius;
let shares_edge = bcast.planned_edges.iter().any(|e| my_edges.contains(e))
    || my_edges.contains(&bcast.current_edge);

let is_partner = within_range && (!self.config.edge_filter_enabled || shares_edge);
if !is_partner { continue; }
```

## Communication Failure Rate

On each tick, before processing a received broadcast, roll against `failure_rate`:
```rust
// In step(), when iterating broadcasts:
if self.config.failure_rate > 0.0 {
    // Simple deterministic hash-based "random" — no_std compatible, no RNG needed
    let hash = (self.robot_id as u32).wrapping_mul(tick_count).wrapping_mul(bcast.robot_id as u32 + 1);
    let roll = (hash % 1000) as f32 / 1000.0;
    if roll < self.config.failure_rate { continue; } // drop this broadcast
}
```

When a broadcast is dropped:
- IR factors for that robot are NOT updated this tick (stale Jacobians/distances persist)
- The robot is NOT removed from the IR set (factors persist from last successful tick)
- This simulates real ESP-NOW packet loss behavior

## Comm Radius Circles (Visualiser)

New draw toggle `comm_radius_circles`. For each robot:
- Draw a gizmo circle at the robot's 3D position with radius `comm_radius`
- Color: blue when the robot has active IR factors with at least one neighbour, grey when no neighbours in range
- Circle is horizontal (in the XZ plane in Bevy Y-up coords)

Requires the `comm_radius` value to be available in the visualiser. Options:
- Read from config.toml (already loaded for draw toggles) — add `[gbp.communication]` section parsing
- Or receive from simulator via a periodic status message

Recommend: read from config.toml since we already parse it in the visualiser.

## New Config Params

```toml
[gbp.communication]
comm_radius = 10.0            # meters — 3D distance for communication range
edge_filter_enabled = true    # require shared planned edges (togglable at runtime)
failure_rate = 0.0            # probability of dropping a broadcast (0.0–1.0)
```

All three are added to `GbpConfig` and live-tunable from the settings panel (M6b).

## File Map

| Action | File | Purpose |
|--------|------|---------|
| Modify | `crates/gbp-core/src/config.rs` | Add comm_radius, edge_filter_enabled, failure_rate to GbpConfig |
| Modify | `crates/gbp-agent/src/robot_agent.rs` | Dual-gated communication check, failure rate logic |
| Modify | `config/config.toml` | Add [gbp.communication] section |
| Modify | `src/bins/simulator/src/toml_config.rs` | Parse [gbp.communication] section |
| Modify | `src/bins/visualiser/src/state.rs` | Add comm_radius_circles to DrawConfig |
| Modify | `src/bins/visualiser/src/robot_render.rs` | Draw comm radius circles system |
| Modify | `src/bins/visualiser/src/main.rs` | Parse comm_radius from config for visualiser |
