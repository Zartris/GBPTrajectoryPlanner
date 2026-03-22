# GBP Trajectory Planner

Distributed 1D velocity coordination for line-following robots using **Gaussian Belief Propagation**. Each robot follows a directed graph of 3D trajectories and can only control its velocity along its current edge. Robots negotiate at merge nodes through a fully distributed factor graph — no central planner, no pre-assigned yielding rules.

Based on [Patwardhan et al., IEEE RA-L 2023](https://arxiv.org/abs/2203.11618).

## Quick Start

### Prerequisites

- Docker with [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) (for GPU-accelerated visualiser)
- WSL2 with NVIDIA GPU driver (Windows)

### Setup

```bash
# 1. Install NVIDIA Docker support (WSL2 host, one-time)
bash scripts/setup-nvidia-docker.sh

# 2. Start the dev container
cd docker
docker compose -f docker-compose.dev.yml up -d
docker exec -it gbptrajectoryplanner-sandbox bash
```

### Running

```bash
# Terminal 1: Start the simulator with a scenario
cargo run -p simulator -- --config config/config.toml --scenario config/scenarios/fleet_4.toml

# Terminal 2: Start the visualiser (Bevy 3D window)
DISPLAY=:0 cargo run -p visualiser

# Optional: If you have trouble with wayland vs X11, try:
WAYLAND_DISPLAY= cargo run -p visualiser
# or 
DISPLAY=:0 cargo run -p visualiser --no-default-features --features "visualiser/x11"
```

Available scenarios:
- `config/scenarios/fleet_4.toml` — 4 robots, auto-assigned distributed starts and random goals
- `config/scenarios/merge.toml` — 2 robots from different entry points converging on the same goal
- `config/scenarios/endcollision.toml` — 2 robots on the same route, staggered start positions

The `--config` flag is optional — omitting it uses built-in defaults.

You should see a 3D window with:
- Yellow lines showing the track edges
- Colored spheres at map nodes (grey=waypoint, green=merge, blue=divert, yellow=charger)
- Environment STL meshes (physical track, magnetic lines)
- Coloured cuboid robots moving along their planned trajectories
- Per-robot HUD panels showing velocity, edge position, and inter-robot distance
- A control panel with pause/resume (pauses the actual simulation)

### Running Tests

```bash
# All workspace tests
cargo test --workspace

# Individual crate tests
cargo test -p gbp-map
cargo test -p gbp-core
cargo test -p gbp-comms
cargo test -p gbp-agent
cargo test -p simulator
cargo test -p visualiser

# Bare-metal build verification (ESP32-C5 target)
cargo build -p gbp-map -p gbp-core -p gbp-comms -p gbp-agent \
    --target riscv32imac-unknown-none-elf
```

## Architecture

```
                    +-------------+
                    |  Visualiser |  Bevy 0.18 + bevy_egui
                    |  (3D view)  |
                    +------+------+
                           | WebSocket (JSON)
                    +------+------+
                    |  Simulator  |  Tokio + axum
                    |  (N robots) |
                    +------+------+
                           |
              +------------+------------+
              |            |            |
         +----+----+  +---+---+  +-----+-----+
         | gbp-map |  |gbp-core|  | gbp-comms |
         | (paths) |  | (GBP)  |  | (messages)|
         +---------+  +--------+  +-----------+
              |            |            |
         +----+------------+------------+----+
         |           gbp-agent               |
         |    (per-robot coordination)       |
         +-----------------------------------+
```

### Crates

| Crate | Purpose | `no_std` |
|-------|---------|----------|
| `gbp-map` | Map topology, NURBS curves, A* pathfinding | Yes |
| `gbp-core` | GBP algorithm, factor graph, message passing | Yes |
| `gbp-comms` | Message types, `CommsInterface` trait | Yes |
| `gbp-agent` | Robot agent, trajectory tracking, factor lifecycle | Yes |
| `simulator` | Desktop simulator, WebSocket server | No |
| `visualiser` | Bevy 3D visualiser, WebSocket client | No |

All four core crates compile for `riscv32imac-unknown-none-elf` (ESP32-C5) without `alloc`.

## Project Structure

```
crates/
  gbp-core/     -- Factor trait, FactorGraph, GbpConfig, DynamicsFactor, InterRobotFactor
  gbp-agent/    -- RobotAgent, trajectory tracking, inter-robot factor lifecycle
  gbp-comms/    -- All message types, CommsInterface trait
  gbp-map/      -- Map, Node, Edge, NURBS eval, arc-length table, A*
src/bins/
  simulator/    -- Tokio + axum, all agents in-process
  visualiser/   -- Bevy 0.18 + bevy_egui 0.39
config/
  config.toml          -- GBP solver, factor weights, velocity limits, draw toggles
  scenarios/           -- Per-run scenario files (map + robot assignments)
firmware/              -- ESP32-C5 firmware (separate Cargo workspace)
maps/                  -- YAML map files
assets/models/         -- STL meshes (track, mainlines, markers, chassis)
docs/                  -- Design docs and implementation plans
scripts/               -- Setup and flashing scripts
docker/                -- Dev container configuration
```

## Configuration

All GBP tuning parameters, robot physical constraints, and visualisation toggles are in `config/config.toml`. The file is heavily commented — each section explains what every parameter does and how it affects behaviour.

Key sections:
- `[gbp]` — solver parameters (damping, iteration counts, timestep)
- `[gbp.dynamics]` — dynamics factor noise (velocity smoothing strength)
- `[gbp.interrobot]` — collision avoidance (safe distance, activation range, decay)
- `[gbp.velocity_bound]` — BIPM-inspired soft velocity limits
- `[robot]` — post-GBP physical constraints (accel, jerk, speed limits)
- `[visualisation.draw]` — toggle individual rendering elements on/off

Scenarios are separate TOML files in `config/scenarios/`. Each defines the map and robot start/goal assignments (by node name) or auto-assignment for fleet mode.

## Firmware (ESP32-C5)

```bash
# Flash all connected probes
bash scripts/flash-all.sh

# Flash a single probe
bash scripts/flash-device.sh <index>

# Monitor serial output
bash scripts/monitor.sh
```

See [PROBE_GUIDE.md](PROBE_GUIDE.md) for WSL2 USB passthrough setup.

## License

See [LICENSE](LICENSE) for details.
