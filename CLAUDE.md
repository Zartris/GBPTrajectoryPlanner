# GBPTrajectoryPlanner — Claude Code Project Context

## Project Overview
Distributed 1D velocity coordination for line-following robots using **Gaussian Belief Propagation**. Each robot follows a directed graph of 3D trajectories and can only control its velocity along its current edge. Robots negotiate at merge nodes through a fully distributed factor graph — no central planner, no pre-assigned yielding rules.

Reference: [Patwardhan et al., IEEE RA-L 2023](https://arxiv.org/abs/2203.11618)

## Repo Structure
```
crates/
  gbp-core/     — Factor trait, FactorGraph, GbpConfig, DynamicsFactor, InterRobotFactor, VelocityBoundFactor
  gbp-agent/    — RobotAgent, trajectory tracking, inter-robot factor lifecycle
  gbp-comms/    — All message types, CommsInterface trait
  gbp-map/      — Map, Node, Edge, NURBS eval, arc-length table, A*
src/bins/
  simulator/    — Tokio + axum, all agents in-process, TOML config/scenario loading
  bridge/       — Connects simulator WebSocket ↔ ESP32 UDP
  visualiser/   — Bevy 0.18 + bevy_egui 0.39 + bevy_stl
config/
  config.toml          — GBP solver, factor weights, velocity limits, draw toggles
  scenarios/           — Per-run scenario TOML files (map + robot assignments)
firmware/              — ESP32-C5 firmware, separate Cargo workspace
maps/                  — YAML map files (test_loop_map.yaml)
assets/models/         — STL meshes (track, mainlines, markers, chassis)
docs/                  — Design specs, implementation plans, handoff docs
scripts/               — flash-all.sh, flash-device.sh, monitor.sh
```

## Hard Rules — Core Crates
`gbp-core`, `gbp-agent`, `gbp-comms`, `gbp-map` MUST compile `#![no_std]` **without `alloc`**. Use `heapless` collections only. Never introduce `std`/`alloc` dependencies in these crates.

## Two Cargo Workspaces
- Root (`/repo/Cargo.toml`) — desktop crates + bins
- Firmware (`/repo/firmware/Cargo.toml`) — ESP32-C5 only (nightly, `riscv32imac-unknown-none-elf`)

Always `cd /repo/firmware` before running cargo commands for firmware.

## Factor System — Extension Pattern
Adding a new factor requires changes in **exactly 3 places**, all marked `// FACTOR EXTENSION POINT` in the source:
1. `crates/gbp-core/src/factor_node.rs` — add variant to `FactorKind` enum
2. `crates/gbp-core/src/factor_node.rs` — add match arms in `as_factor()` / `as_factor_mut()`
3. `crates/gbp-core/src/lib.rs` — export the new module

Nothing else in `gbp-core` changes. See `docs/adding_a_factor.md` for the step-by-step guide.

## Key Design Decisions
- **No goal factor** — gbpplanner's sliding goal creates windup (unbounded position error during forced stops). Replaced with a **velocity prior** (`DynamicsFactor`) that encodes "try to move at `v_nom`". No accumulated error, no windup. NOTE: This design choice is under review for M6.5 — the dynamics-as-motivation approach penalises stopping, making collision avoidance harder.
- **Trapezoidal profile**: `v_nom(s) = min(nominal, sqrt(2 * decel_limit * (edge_length - s)))`. Taper only on the final edge of planned trajectory.
- **VelocityBoundFactor**: BIPM-inspired adaptive precision barrier for v_min/v_max. Prevents GBP from predicting impossible velocities. See `velocity_bound_factor.rs` for theory.
- **Factor gating**: inter-robot factors spawn when planned edge sequences share an edge AND 3D distance < activation_range. Per-variable spawning/despawning. NOTE: M6c will add distance-based communication alongside edge-sharing, with togglable edge filter.
- **Cavity beliefs**: Factor-to-variable messages use proper cavity (marginal minus factor's own message) for both pairwise and unary factors. Broadcasts use cavity beliefs to prevent circular evidence.
- **iterate_split**: Internal factors (dynamics, velocity_bound) get N_I rounds before external (IR) factors run. External round runs ALL factors. Variables always accumulate all messages. M_I=10, M_E=10 per MAGICS thesis §6.2.1.
- **At-goal behavior**: v_nom=0 on all dynamics factors, jacobian_a=0 on IR factors (goal robot has nowhere to go, only approaching robot yields).
- **`swap_remove` index tracking**: `remove_factor` is O(1) swap-remove. `InterRobotFactorSet` in `gbp-agent` maintains the `robot_id → factor_idx` map and patches it on every removal.
- **1D problem in 3D world**: 3D geometry absorbed into scalar Jacobians in the agent layer. GBP variable is always scalar arc-length `s`.
- **GbpConfig**: All tunable parameters in a `#![no_std]` Copy struct (`gbp-core/src/config.rs`). Simulator parses TOML into this struct. Core crates never parse files.

## Firmware Key Facts
- Runtime: Embassy async (`embassy-executor`, `embassy-time`)
- HAL: `esp-hal ~1.0` (uses `esp32c6` feature — confirmed working on C5 hardware)
- Wireless: `esp-radio ~0.16`, ESP-NOW protocol
- Serialization: `postcard` + `bytemuck` (`#[repr(C, packed)]` for ESP-NOW packets)
- Logging: `defmt` only — never use `println!`/`eprintln!`
- Heap: 65536 bytes (tight) — GBP core does not use heap
- ESP-NOW packet budget: 250 bytes max. At K=12: 148 bytes used.
- Memory budget: ~250 KB / 384 KB SRAM (WiFi stack ~100 KB, map ~20 KB, GBP ~18 KB)
- Strict denies: `clippy::mem_forget`, `clippy::large_stack_frames`
- New tasks: use `mk_static!` macro for `'static` data, spawn via `spawner.spawn(...).unwrap()`
- **Always use context7** to look up library docs before writing or reviewing code — APIs change between versions and training data may be stale. This applies to all dependencies: `esp-hal`, `embassy-*`, `esp-radio`, `postcard`, `bevy`, `bevy_egui`, `tokio`, `axum`, `heapless`, `serde`, etc.

## Crate Dependency Graph
```
gbp-core  →  heapless, libm
gbp-comms →  heapless
gbp-map   →  heapless, postcard  [+serde, serde_yaml with "parse" feature — PC only]
gbp-agent →  gbp-core, gbp-comms, gbp-map
simulator →  gbp-agent, gbp-core, gbp-comms, gbp-map(+parse), tokio, axum, serde_json, toml, clap
esp32     →  gbp-agent, gbp-comms, gbp-map, esp-hal, esp-radio, embassy-*
bridge    →  gbp-comms, gbp-map(+parse), tokio, axum, serde_json
visualiser → gbp-comms, gbp-map(+parse), bevy, bevy_egui, bevy_stl, toml, serde
```

## Deployment Modes
- **Simulator only**: `visualiser ←WebSocket→ simulator`
- **Hardware only**: `visualiser ←WebSocket→ bridge ←UDP→ ESP32 fleet`
- **Hybrid**: bridge connects to both. `robot_id` in `--sim-ids` vs `--esp-ids` routes commands.

Progressive bring-up: start all agents in simulator, migrate one robot at a time to hardware.

## Build Commands
```bash
# Simulator (with config + scenario)
cargo run -p simulator -- --config config/config.toml --scenario config/scenarios/fleet_4.toml

# Simulator (defaults, no config file)
cargo run -p simulator -- --scenario config/scenarios/merge.toml

# Visualiser (reads config/config.toml for draw toggles via CONFIG_PATH env)
DISPLAY=:0 cargo run -p visualiser

# Tests
cargo test --workspace

# Bridge — hybrid mode (not yet updated for TOML config)
cargo run -p bridge -- --map maps/test_loop_map.yaml \
    --sim-ws ws://localhost:3000 --esp-udp 0.0.0.0:4242 \
    --sim-ids 0,1,2 --esp-ids 3,4
```

## Flashing / Debugging
- Hardware: probe-rs over USB, connected via usbipd-win in WSL2
- Flash all probes in parallel: `bash scripts/flash-all.sh`
- Flash one probe: `bash scripts/flash-device.sh <index>`
- Monitor serial: `bash scripts/monitor.sh`
- See `PROBE_GUIDE.md` for WSL2 USB passthrough setup

## Dev Environment
- Docker devcontainer ("Valhalla") via `docker/docker-compose.dev.yml`
- VS Code extensions: rust-analyzer, probe-rs-debugger, even-better-toml, bevy
- Clippy runs on save in VS Code; Claude hook also runs clippy after firmware edits

## Git Hooks
Shared hooks live in `.githooks/`. New contributors must run once after cloning:
```bash
git config core.hooksPath .githooks
```
The `commit-msg` hook strips AI agent co-author credits. Do NOT add `Co-Authored-By` lines for AI agents, but this should also be the case for any git related activites. 
