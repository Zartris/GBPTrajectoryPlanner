# Session Handoff — 2026-03-22

## Who Is the User

Robotics expert, author of the MAGIC/MAGICS paper on GBP multi-agent path planning. Deep knowledge of factor graphs, Gaussian Belief Propagation, and the mathematical foundations. Prefers to understand the "why" behind decisions. Will push back on hacks and wants principled solutions.

## How They Like to Work

- **Subagent-driven development** for implementation — fresh subagent per task, review between tasks
- **No AI attribution** — never add Co-Authored-By, "Generated with", or any AI credit to commits, PRs, or output
- **Edit only, never Write** on existing files — subagents have repeatedly destroyed code by overwriting entire files with Write. This is a critical rule.
- **Cross-reference MAGICS** — clone `https://github.com/AU-Master-Thesis/magics.git` to `/tmp/magics` for reference. The MAGICS thesis is at `/repo/papers/thesis_magics.pdf`.
- **Config-driven** — all tunable parameters live in `config/config.toml`, not hardcoded
- **Use context7** to look up library docs before writing code — APIs change between versions
- **Follow the brainstorming → spec → review → plan → implement flow** (superpowers skills)

## Current State

### Just Merged: PR #7 — M5 (Fleet + Full Visualiser + Config + GBP Fixes)

Branch `feature/m5-fleet-full-visualiser` merged to `main`. Key deliverables:

- **N-robot fleet mode** with TOML scenario files (`config/scenarios/*.toml`)
- **TOML config system** (`config/config.toml`) with all GBP/robot/visualisation params
- **Visualiser**: environment STLs, dynamic robot spawn, pause/resume (wired to simulator), draw toggles, N-robot collision monitor
- **6 GBP factor bug fixes** (zeta linearization point, cavity beliefs, residual sign, iterate_split, broadcast cavities, belief re-accumulation)
- **Iteration count tuning**: M_I=10, M_E=10 per MAGICS thesis §6.2.1

### Known Issues (NOT Fixed in M5)

These are the reason M6.5 exists:

1. **Merge scenario: ~30 marginal collisions** (dist 0.6–1.15m vs d_safe 1.3m). The IR factor slows one robot but not enough — the other arrives too fast. GBP is fundamentally working (was 227 collisions before fixes, spread 10m → 2m) but doesn't fully prevent overlap.

2. **Goal collision**: Robot with very short trajectory (1 edge, 3.73m) reaches goal instantly. Approaching robot can't brake in time. At-goal robot has v_nom=0 and jacobian_a=0, but the approaching robot still arrives at full speed.

3. **Dynamics factor as driving motivation**: The current design uses a velocity prior (DynamicsFactor with v_nom) as the primary motivation for movement. This means the robot is heavily penalised for stopping — the dynamics factor constantly pushes toward nominal speed, fighting the IR factor. This is a fundamental design question.

## Immediate Next Task: M6.5 Milestone

**Create a milestone for M6.5** — this is the factor rethinking milestone. It should address everything we couldn't get working in M5. Key questions to discuss with the user:

### Factor Design Questions (Need Discussion)

1. **Should target speed be the driving motivation?** Currently DynamicsFactor encodes "try to move at v_nom". This penalises stopping, making collision avoidance harder. Alternatives:
   - Separate "goal attraction" factor (pulls toward goal position, not speed)
   - Tracking factor (follows a reference trajectory, like MAGICS)
   - Weaker dynamics (lower precision) so IR can overpower it more easily

2. **Should we add more specific factors?** Currently we have 3 factor types (Dynamics, InterRobot, VelocityBound). MAGICS has 5 (Dynamic, InterRobot, Obstacle, Tracking, Pose). Should we add:
   - **Goal factor**: Strong pull toward the goal node position
   - **Tracking factor**: Follow the A* path (different from dynamics — tracks position, not speed)
   - **Speed profile factor**: Separate from dynamics — enforces edge-specific speed limits

3. **IR factor strength vs dynamics**: At sigma_ir=0.12, IR precision ≈ 69 vs dynamics precision ≈ 4. Despite this 17:1 ratio, the dynamics factor wins in merge scenarios because it's active on all K-1 variable pairs while IR factors are only on some. The "mass" of dynamics overwhelms IR.

4. **At-goal behavior**: Current approach (v_nom=0, jacobian_a=0) is a workaround. A proper goal factor that pins the robot to the goal position would be cleaner.

5. **Communication model**: Edge-sharing filter may be causing collision gaps at merge points. M6c adds distance-based communication — implement this first before factor redesign to rule it out.

## M6 Plans (Ready to Execute)

Four sub-projects, specs reviewed and plans written:

| Sub-project | Spec | Plan | Tasks |
|-------------|------|------|-------|
| M6a: Visualiser Overhaul | `specs/2026-03-22-m6a-*` | `plans/2026-03-22-m6a-*` | 12 |
| M6b: Live Tuning & Debugging | `specs/2026-03-22-m6b-*` | `plans/2026-03-22-m6b-*` | 8 |
| M6c: Communication Model | `specs/2026-03-22-m6c-*` | `plans/2026-03-22-m6c-*` | 6 |
| M6d: Simulator Robustness | `specs/2026-03-22-m6d-*` | `plans/2026-03-22-m6d-*` | 11 |

**Execution order**: M6a → M6b → M6c → M6d. Each builds on the previous. Start on a new `feature/m6-*` branch from main.

**Important**: M6 specs/plans are committed to main (via the M5 merge). The implementation should happen on fresh feature branches.

## Key Files

| File | What |
|------|------|
| `config/config.toml` | All GBP params, heavily commented |
| `config/scenarios/*.toml` | Scenario definitions (merge, endcollision, fleet_4) |
| `crates/gbp-core/src/config.rs` | GbpConfig struct (no_std, Copy) |
| `crates/gbp-core/src/factor_graph.rs` | GBP solver (iterate_split, cavity, damping) |
| `crates/gbp-core/src/interrobot_factor.rs` | IR factor (Schur complement, decay precision) |
| `crates/gbp-agent/src/robot_agent.rs` | Agent orchestration (most complex file ~600 lines) |
| `src/bins/simulator/src/main.rs` | Simulator entry (fleet spawn, collision monitor) |
| `src/bins/visualiser/src/` | Bevy visualiser (camera, rendering, UI) |
| `papers/thesis_magics.pdf` | MAGICS thesis — §5.4 and §6.2 for iteration tuning |
| `CLAUDE.md` | Project rules and conventions |

## Build & Run

```bash
# Simulator
cargo run -p simulator -- --config config/config.toml --scenario config/scenarios/merge.toml

# Visualiser
DISPLAY=:0 cargo run -p visualiser

# Tests
cargo test --workspace

# Clone MAGICS for reference
git clone https://github.com/AU-Master-Thesis/magics.git /tmp/magics
```
