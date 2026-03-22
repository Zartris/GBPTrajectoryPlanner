# M6d: Simulator Robustness — Design Spec

## Goal

Blocked edge handling with A* replanning, map and scenario reload, despawn-at-goal option, loop mode (replan on goal arrival), and a log panel for simulator messages.

## Blocked Edges

### A* Filtered
New function in `gbp-map`:
```rust
pub fn astar_filtered<F>(map: &Map, start: NodeId, goal: NodeId, allow_edge: F)
    -> Option<heapless::Vec<EdgeId, MAX_PATH_EDGES>>
where F: Fn(EdgeId) -> bool
```

Existing `astar()` becomes a thin wrapper: `astar_filtered(map, start, goal, |_| true)`.

### BlockedEdges State
Simulator maintains `Arc<Mutex<heapless::Vec<EdgeId, 16>>>`. When an edge is blocked:
1. Add EdgeId to the blocked set
2. For each robot currently routing through that edge: trigger A* replan using `astar_filtered` that skips blocked edges
3. Robots physically ON the blocked edge continue to the edge's end node, then replan from there

### Block/Unblock Commands
WebSocket commands:
```json
{"command": "block_edge", "edge_id": 5}
{"command": "unblock_edge", "edge_id": 5}
```

### Visualiser Integration
- Blocked edges rendered in red gizmo colour instead of yellow
- Settings panel shows edge list with block/unblock buttons (or click-to-block on the 3D edge)
- The `BlockedEdges` set is sent to the visualiser periodically so it can color edges correctly

## Scenario Reload

"Reload Scenario" button in settings panel sends `{"command":"reload_scenario"}`.

Simulator behavior:
1. Re-read the scenario TOML file from disk
2. Stop all agent and physics tasks (set a shutdown flag)
3. Re-parse map (in case map file changed)
4. Re-assign start/goals from scenario
5. Respawn all AgentRunners and PhysicsStates
6. Resume

This is a full reset — all robot state is discarded. Collision count resets to 0.

## Map Reload

"Reload Map" button sends `{"command":"reload_map"}`.

Simulator behavior:
1. Re-read the map YAML from disk
2. Replace `Arc<Map>` via `Arc::swap` or RwLock
3. Trigger A* replanning for all robots from their current node
4. Edge polyline cache in visualiser rebuilt (send a "map_changed" event)

Robots mid-edge continue on the old geometry until they reach a node.

## Despawn at Goal

Config option:
```toml
[simulation]
despawn_at_goal = false
```

When `true`, a robot that reaches `at_goal()`:
- Stops its agent_task and physics_task (tasks exit their loops)
- Sends a final `RobotStateMsg` with `velocity = 0`
- Is removed from the collision monitor
- Visualiser keeps the robot mesh at its final position (greyed out) until scenario reload

When `false` (default), current behavior: robot stays at goal with v_nom=0, jacobian_a=0, keeps running GBP for IR avoidance.

## Loop Mode (Goal Replanning)

Config option:
```toml
[simulation]
loop_mode = false
```

When `true`, a robot that reaches `at_goal()`:
1. Picks a new random reachable goal via `pick_random_goal()`
2. Runs A* from the current goal node to the new goal
3. Sets the new trajectory on the AgentRunner
4. Resets PhysicsState (position_s = 0, velocity = 0)
5. Continues running

**Priority:** `loop_mode` overrides `despawn_at_goal`. If both true, robot replans (loops) instead of despawning.

**Implementation:** In `agent_task`, after each tick, check `physics.at_goal()`. If true and loop_mode enabled:
```rust
if at_goal && config.loop_mode {
    let current_goal = runner.current_goal_node();
    let new_goal = pick_random_goal(&map, current_goal, tick_count);
    if let Some(path) = astar(&map, current_goal, new_goal) {
        let traj = build_trajectory_edges(&map, &path);
        runner.set_trajectory(traj);
        physics.position_s = 0.0;
        physics.total_length = traj_total_length;
    }
}
```

Both `loop_mode` and `despawn_at_goal` are togglable at runtime from the settings panel.

## Log Panel

### Simulator Side
New `LogMsg` type sent over the existing WebSocket alongside `RobotStateMsg`:
```json
{"type": "log", "level": "info", "text": "Robot 2: replanned around blocked edge 5", "t": 12.34}
```

Log messages are generated for:
- Edge blocked/unblocked
- Robot replanned (new goal, new trajectory)
- Map reloaded
- Scenario reloaded
- Parameter changed
- Robot despawned / entered loop mode
- Collision events

### Visualiser Side
`LogBuffer` resource: `VecDeque<LogEntry>` with cap 200. Each entry has timestamp, level, text.

Floating egui window (togglable):
- Scrollable, stick-to-bottom
- Color-coded by level: red=error, yellow=warn, white=info
- Filter buttons: show/hide by level
- Clear button

## New Config Params

```toml
[simulation]
despawn_at_goal = false
loop_mode = false
```

Added to GbpConfig or a separate SimConfig (since these are simulator-only, not needed in no_std core crates). Recommend a separate `SimConfig` struct in the simulator crate to keep GbpConfig focused on GBP parameters.

## File Map

| Action | File | Purpose |
|--------|------|---------|
| Modify | `crates/gbp-map/src/astar.rs` | Add astar_filtered, refactor astar as wrapper |
| Modify | `src/bins/simulator/src/main.rs` | BlockedEdges state, reload commands, loop mode, despawn, log messages |
| Modify | `src/bins/simulator/src/agent_runner.rs` | Goal detection, loop mode replanning, despawn flag |
| Create | `src/bins/simulator/src/sim_config.rs` | SimConfig struct (despawn_at_goal, loop_mode) |
| Modify | `src/bins/simulator/src/toml_config.rs` | Parse [simulation] section |
| Modify | `src/bins/simulator/src/ws_server.rs` | Handle block/unblock/reload commands, send LogMsg |
| Modify | `src/bins/visualiser/src/state.rs` | LogBuffer, LogEntry, BlockedEdgesState |
| Modify | `src/bins/visualiser/src/ui.rs` | Log panel window |
| Modify | `src/bins/visualiser/src/map_scene.rs` | Red gizmo color for blocked edges |
| Modify | `src/bins/visualiser/src/ws_client.rs` | Receive LogMsg and BlockedEdges messages |
| Modify | `config/config.toml` | Add [simulation] section |
