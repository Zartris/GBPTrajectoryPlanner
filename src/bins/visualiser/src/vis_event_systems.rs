// src/bins/visualiser/src/vis_event_systems.rs
//
//! Event emission systems for the visualiser event bus.
//!
//! These systems run **only when `RobotStates` has changed** (i.e., when
//! `drain_ws_inbox` processed new WebSocket data). This eliminates per-frame
//! overhead on idle frames.
//!
//! # Architecture
//!
//! ```text
//!  drain_ws_inbox (robot_render.rs)
//!        |  mutates RobotStates
//!        v
//!  ┌───────────────────────────┐    resource_changed::<RobotStates>
//!  │ emit_events_on_data       │    (run condition)
//!  │  ├─ proximity check       │──► MessageWriter<ProximityAlert>
//!  │  ├─ state change detect   │──► MessageWriter<RobotStateChanged>
//!  │  └─ data summary          │──► MessageWriter<DataReceived>
//!  └───────────────────────────┘
//! ```
//!
//! The single `emit_events_on_data` system replaces three per-frame systems
//! (`proximity_emitter`, `sim_tick_emitter`, `state_change_emitter`).
//! It is registered by [`VisEventPlugin`] and runs in [`Update`] with a
//! `resource_changed::<RobotStates>` run condition.

use bevy::prelude::*;
use bevy::ecs::message::MessageWriter;
use std::collections::HashMap;

use crate::state::{DrawConfig, RobotStates};
use crate::vis_events::{
    DataReceived, ProximityAlert, RobotChangeType, RobotStateChanged,
};
use gbp_map::map::EdgeId;

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Registers all event message types and the change-triggered emission system.
///
/// Add this plugin to the Bevy [`App`] to enable the event bus. It is
/// automatically added by [`AddonPlugins`](crate::addons::AddonPlugins).
pub struct VisEventPlugin;

impl Plugin for VisEventPlugin {
    fn build(&self, app: &mut App) {
        // Register message types so MessageReader/MessageWriter work.
        app.add_message::<ProximityAlert>()
            .add_message::<DataReceived>()
            .add_message::<RobotStateChanged>()
            // Single emission system — runs only when RobotStates was mutated.
            .add_systems(
                Update,
                emit_events_on_data
                    .run_if(resource_changed::<RobotStates>),
            );
    }
}

// ---------------------------------------------------------------------------
// Combined Event Emission System
// ---------------------------------------------------------------------------

/// Rate-limit state: maps `(min_id, max_id)` to the last time (elapsed_secs)
/// a ProximityAlert was emitted for that pair.
#[derive(Default)]
struct ProximityRateLimiter {
    last_alert: HashMap<(u32, u32), f32>,
}

/// Minimum interval (seconds) between ProximityAlert emissions for the same
/// robot pair. Prevents log/screenshot spam when two robots are stuck close.
const PROXIMITY_COOLDOWN_SECS: f32 = 1.0;

/// Snapshot of per-robot state from the previous check, used to detect changes.
#[derive(Clone, Debug)]
struct RobotSnapshot {
    current_edge: EdgeId,
    active_factor_count: usize,
    was_near_collision: bool,
}

/// Previous robot state for change detection.
#[derive(Default)]
struct PreviousRobotStates {
    snapshots: HashMap<u32, RobotSnapshot>,
}

/// Unified event emission system. Runs only when `RobotStates` changed.
///
/// Combines proximity checking, state change detection, and data summary
/// into a single system to avoid iterating over robot states multiple times.
fn emit_events_on_data(
    robot_states: Res<RobotStates>,
    draw: Res<DrawConfig>,
    time: Res<Time>,
    mut proximity_writer: MessageWriter<ProximityAlert>,
    mut data_writer: MessageWriter<DataReceived>,
    mut state_writer: MessageWriter<RobotStateChanged>,
    mut limiter: Local<ProximityRateLimiter>,
    mut prev: Local<PreviousRobotStates>,
    mut tick_counter: Local<u64>,
) {
    let now = time.elapsed_secs();
    let d_safe = draw.ir_d_safe;

    // Collect IDs into a sorted vec for deterministic pair ordering.
    let mut ids: Vec<u32> = robot_states.0.keys().copied().collect();
    ids.sort_unstable();

    // ── 1. Proximity alerts ──────────────────────────────────────────────
    for i in 0..ids.len() {
        let id_a = ids[i];
        let Some(state_a) = robot_states.0.get(&id_a) else {
            continue;
        };
        let pa = state_a.pos_3d;

        for &id_b in &ids[(i + 1)..] {
            let Some(state_b) = robot_states.0.get(&id_b) else {
                continue;
            };
            let pb = state_b.pos_3d;

            let dist = dist_3d(pa, pb);
            if dist < d_safe {
                let pair_key = (id_a, id_b);

                // Rate-limit: skip if we emitted for this pair recently.
                if let Some(&last) = limiter.last_alert.get(&pair_key) {
                    if now - last < PROXIMITY_COOLDOWN_SECS {
                        continue;
                    }
                }

                limiter.last_alert.insert(pair_key, now);

                proximity_writer.write(ProximityAlert {
                    robot_a: id_a,
                    robot_b: id_b,
                    distance: dist,
                    d_safe,
                    position_a: pa,
                    position_b: pb,
                });
            }
        }
    }

    // Prune stale entries from the rate-limiter to prevent unbounded memory growth.
    if limiter.last_alert.len() > 64 {
        limiter
            .last_alert
            .retain(|_, last| now - *last < PROXIMITY_COOLDOWN_SECS * 10.0);
    }

    // ── 2. State change detection ────────────────────────────────────────
    // Detect new robots (Connected) and state changes.
    for (&id, state) in &robot_states.0 {
        let is_near = state.min_neighbour_dist_3d < d_safe;

        if let Some(old) = prev.snapshots.get(&id) {
            // Edge changed?
            if state.current_edge != old.current_edge {
                state_writer.write(RobotStateChanged {
                    robot_id: id,
                    event_type: RobotChangeType::EdgeChanged {
                        from: old.current_edge,
                        to: state.current_edge,
                    },
                });
            }

            // Factor count changed?
            if state.active_factor_count != old.active_factor_count {
                state_writer.write(RobotStateChanged {
                    robot_id: id,
                    event_type: RobotChangeType::FactorCountChanged {
                        from: old.active_factor_count,
                        to: state.active_factor_count,
                    },
                });
            }

            // Crossed into near-collision zone (rising edge only)?
            if is_near && !old.was_near_collision {
                state_writer.write(RobotStateChanged {
                    robot_id: id,
                    event_type: RobotChangeType::NearCollision {
                        distance: state.min_neighbour_dist_3d,
                    },
                });
            }
        } else {
            // New robot — emit Connected.
            state_writer.write(RobotStateChanged {
                robot_id: id,
                event_type: RobotChangeType::Connected,
            });
        }
    }

    // Detect disconnected robots (present in prev but not in current).
    for &id in prev.snapshots.keys() {
        if !robot_states.0.contains_key(&id) {
            state_writer.write(RobotStateChanged {
                robot_id: id,
                event_type: RobotChangeType::Disconnected,
            });
        }
    }

    // Update snapshot for next check.
    prev.snapshots.clear();
    for (&id, state) in &robot_states.0 {
        prev.snapshots.insert(
            id,
            RobotSnapshot {
                current_edge: state.current_edge,
                active_factor_count: state.active_factor_count,
                was_near_collision: state.min_neighbour_dist_3d < d_safe,
            },
        );
    }

    // ── 3. DataReceived summary ──────────────────────────────────────────
    let robot_count = robot_states.0.len();
    let total_ir_factors: usize = robot_states
        .0
        .values()
        .map(|s| s.active_factor_count)
        .sum();

    let min_pair_distance = robot_states
        .0
        .values()
        .map(|s| s.min_neighbour_dist_3d)
        .fold(f32::MAX, f32::min);

    data_writer.write(DataReceived {
        tick: *tick_counter,
        robot_count,
        total_ir_factors,
        min_pair_distance,
    });

    *tick_counter += 1;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Euclidean 3D distance between two `[f32; 3]` positions.
pub(crate) fn dist_3d(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Compute the minimum pairwise 3D distance across all tracked robots.
/// Returns `f32::MAX` if fewer than 2 robots are tracked.
#[cfg(test)]
fn compute_min_pair_distance(robot_states: &RobotStates) -> f32 {
    let positions: Vec<(u32, [f32; 3])> = robot_states
        .0
        .iter()
        .map(|(&id, s)| (id, s.pos_3d))
        .collect();

    let mut min_dist = f32::MAX;
    for i in 0..positions.len() {
        for j in (i + 1)..positions.len() {
            let d = dist_3d(positions[i].1, positions[j].1);
            if d < min_dist {
                min_dist = d;
            }
        }
    }
    min_dist
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dist_3d_zero() {
        let a = [0.0, 0.0, 0.0];
        assert!((dist_3d(a, a) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn dist_3d_unit_x() {
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        assert!((dist_3d(a, b) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn dist_3d_diagonal() {
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 1.0, 1.0];
        let expected = (3.0_f32).sqrt();
        assert!((dist_3d(a, b) - expected).abs() < 1e-6);
    }

    #[test]
    fn dist_3d_symmetric() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        assert!((dist_3d(a, b) - dist_3d(b, a)).abs() < 1e-9);
    }

    #[test]
    fn dist_3d_negative_coords() {
        let a = [-1.0, -2.0, -3.0];
        let b = [1.0, 2.0, 3.0];
        let expected = ((2.0_f32).powi(2) + (4.0_f32).powi(2) + (6.0_f32).powi(2)).sqrt();
        assert!((dist_3d(a, b) - expected).abs() < 1e-6);
    }

    #[test]
    fn min_pair_distance_empty() {
        let rs = RobotStates::default();
        assert_eq!(compute_min_pair_distance(&rs), f32::MAX);
    }

    #[test]
    fn min_pair_distance_single_robot() {
        let mut rs = RobotStates::default();
        rs.0.insert(0, crate::state::RobotState::default());
        assert_eq!(compute_min_pair_distance(&rs), f32::MAX);
    }

    #[test]
    fn min_pair_distance_two_robots() {
        let mut rs = RobotStates::default();
        let mut s0 = crate::state::RobotState::default();
        s0.pos_3d = [0.0, 0.0, 0.0];
        let mut s1 = crate::state::RobotState::default();
        s1.pos_3d = [3.0, 4.0, 0.0];
        rs.0.insert(0, s0);
        rs.0.insert(1, s1);
        assert!((compute_min_pair_distance(&rs) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn min_pair_distance_three_robots_picks_closest() {
        let mut rs = RobotStates::default();
        let mut s0 = crate::state::RobotState::default();
        s0.pos_3d = [0.0, 0.0, 0.0];
        let mut s1 = crate::state::RobotState::default();
        s1.pos_3d = [10.0, 0.0, 0.0];
        let mut s2 = crate::state::RobotState::default();
        s2.pos_3d = [1.0, 0.0, 0.0]; // closest to s0
        rs.0.insert(0, s0);
        rs.0.insert(1, s1);
        rs.0.insert(2, s2);
        assert!((compute_min_pair_distance(&rs) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn proximity_rate_limiter_default_is_empty() {
        let limiter = ProximityRateLimiter::default();
        assert!(limiter.last_alert.is_empty());
    }

    #[test]
    fn robot_snapshot_clone() {
        let snap = RobotSnapshot {
            current_edge: EdgeId(5),
            active_factor_count: 3,
            was_near_collision: true,
        };
        let snap2 = snap.clone();
        assert_eq!(snap2.current_edge, EdgeId(5));
        assert_eq!(snap2.active_factor_count, 3);
        assert!(snap2.was_near_collision);
    }
}
