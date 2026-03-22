# GBPTrajectoryPlanner — Docs Roadmap

This folder tracks the big picture: milestones define **what** gets built and in what order; specs define the **design** before implementation; plans break designs into **bite-sized tasks**.

## Philosophy

Build the GBP pipeline end-to-end first (M0-M4), then scale to N robots with full visualisation (M5), then add debugging/tuning tools (M6) to understand and fix the remaining factor issues (M6.5), then bring it to hardware (M7-M9). The MAGICS thesis (`papers/thesis_magics.pdf`) is the primary reference for GBP algorithm design.

## Milestone Sequence

| # | Name | Focus | Status |
|---|------|-------|--------|
| **M0** | Foundation Crates | `gbp-map`, `gbp-core`, `gbp-comms`, `gbp-agent` — all `#![no_std]` | Complete |
| **M1** | 1 Robot, 1 Edge, Visible | Simulator + visualiser, single robot on straight edge | Complete |
| **M2** | 1 Robot, Full Route | A* path planning, multi-edge trajectory, trapezoidal deceleration | Complete |
| **M3** | 2 Robots, Rear-End Avoidance | GBP active, `InterRobotFactor`, belief tubes, factor links | Complete |
| **M4** | Merge Collision Avoidance | Factor gating, 3D Jacobians, `VelocityBoundFactor` (BIPM), iterate_split | Complete |
| **M5** | Fleet + Full Visualiser | N-robot fleet, TOML config system, environment STLs, 6 GBP factor fixes | Complete |
| **M6** | Robustness + Debugging Tools | Camera controls, live tuning, click-to-inspect, communication model, blocked edges | Planned |
| **M6.5** | Factor Rethinking | Rethink dynamics-as-motivation, add goal/tracking factors, fix merge collisions | Planned |
| **M7** | GBP Agent on Firmware | ESP32 running real `RobotAgent`, postcard map delivery | Not Started |
| **M8** | Bridge + Single Hardware Robot | One ESP32 visible alongside simulated robots in visualiser | Not Started |
| **M9** | Hybrid Fleet + Progressive Bring-Up | Migrate robots from sim to hardware one at a time | Not Started |

## M6 Sub-Projects

M6 is decomposed into 4 independent sub-projects executed in order:

| Sub | Name | Focus | Spec | Plan |
|-----|------|-------|------|------|
| **M6a** | Visualiser Overhaul | Orbit/pan/zoom camera, draw toggles in egui, metrics, inspector, path traces | [spec](superpowers/specs/2026-03-22-m6a-visualiser-overhaul-design.md) | [plan](superpowers/plans/2026-03-22-m6a-visualiser-overhaul.md) |
| **M6b** | Live Tuning & Debugging | Settings panel, param sliders, single-step, click-to-inspect, collision markers | [spec](superpowers/specs/2026-03-22-m6b-live-tuning-debugging-design.md) | [plan](superpowers/plans/2026-03-22-m6b-live-tuning-debugging.md) |
| **M6c** | Communication Model | Distance-based comms, togglable edge filter, failure rate, comm radius circles | [spec](superpowers/specs/2026-03-22-m6c-communication-model-design.md) | [plan](superpowers/plans/2026-03-22-m6c-communication-model.md) |
| **M6d** | Simulator Robustness | Blocked edges, map/scenario reload, despawn at goal, loop mode, log panel | [spec](superpowers/specs/2026-03-22-m6d-simulator-robustness-design.md) | [plan](superpowers/plans/2026-03-22-m6d-simulator-robustness.md) |

## Dependency Graph

```
M0 (foundation crates)
 └→ M1 (1 robot visible)
      └→ M2 (full route)
           └→ M3 (2 robots, rear-end)
                └→ M4 (merge avoidance)
                     └→ M5 (fleet + config + GBP fixes)
                          ├→ M6a (visualiser overhaul)
                          │    └→ M6b (live tuning)
                          │         └→ M6c (communication model)
                          │              └→ M6d (simulator robustness)
                          │                   └→ M6.5 (factor rethinking)
                          │                        └→ M7 (firmware)
                          │                             └→ M8 (bridge)
                          │                                  └→ M9 (hybrid fleet)
                          └→ M6.5 can start after M6b (needs live tuning to experiment)
```

M6a-d are sequential (each builds on the previous). M6.5 needs at least M6b (live tuning) to experiment with factor designs interactively.

## Folder Structure

```
docs/
├── README.md                          ← This file (roadmap overview)
├── adding_a_factor.md                 ← Step-by-step guide for new factor types
├── gbp_merge_design.md                ← Original M4 merge design doc
├── handoff/                           ← Session handoff documents
│   ├── 2026-03-21-m4-handoff.md       ← (superseded)
│   └── 2026-03-22-session-handoff.md  ← Current state, next steps, user preferences
└── superpowers/                       ← AI-assisted development artifacts
    ├── specs/                         ← Design specifications (reviewed before implementation)
    │   ├── 2026-03-19-milestones-design.md
    │   ├── 2026-03-21-config-system-design.md
    │   ├── 2026-03-22-m6a-visualiser-overhaul-design.md
    │   ├── 2026-03-22-m6b-live-tuning-debugging-design.md
    │   ├── 2026-03-22-m6c-communication-model-design.md
    │   └── 2026-03-22-m6d-simulator-robustness-design.md
    └── plans/                         ← Implementation plans (task-by-task breakdown)
        ├── 2026-03-19-m0-foundation-crates.md
        ├── 2026-03-19-m1-one-robot-visible.md
        ├── 2026-03-19-m2-full-route.md
        ├── 2026-03-19-m6-robustness-polish.md  ← (outdated, replaced by m6a-d)
        ├── 2026-03-21-m5-fleet-full-visualiser.md
        ├── 2026-03-21-config-system.md
        ├── 2026-03-22-m6a-visualiser-overhaul.md
        ├── 2026-03-22-m6b-live-tuning-debugging.md
        ├── 2026-03-22-m6c-communication-model.md
        └── 2026-03-22-m6d-simulator-robustness.md
```

## Workflow

1. **Brainstorm** — Explore the idea, ask clarifying questions, propose approaches
2. **Spec** — Write design doc in `superpowers/specs/`, get it reviewed
3. **Plan** — Break spec into task-by-task implementation plan in `superpowers/plans/`
4. **Implement** — Execute plan via subagent-driven development on a feature branch
5. **Review** — PR with Copilot review, fix comments, merge to main
6. **Handoff** — Write handoff doc if switching sessions/PCs

## Key References

| Document | What |
|----------|------|
| `/repo/CLAUDE.md` | Project rules, repo structure, hard constraints, build commands |
| `/repo/config/config.toml` | All GBP parameters with detailed comments |
| `/repo/papers/thesis_magics.pdf` | MAGICS thesis — §5.4 iteration tuning, §6.2 algorithm analysis |
| `/repo/papers/Magic.pdf` | Original MAGIC paper (GBP for multi-agent) |
| [MAGICS repo](https://github.com/AU-Master-Thesis/magics) | Reference implementation (clone to `/tmp/magics` for cross-reference) |
