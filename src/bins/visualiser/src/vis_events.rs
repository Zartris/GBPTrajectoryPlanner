// src/bins/visualiser/src/vis_events.rs
//
//! Visualiser event system — buffered messages that addons can subscribe to.
//!
//! These are Bevy [`Message`] types (buffered, pull-based), not [`Event`] types
//! (observer-based, immediate). Addons read them with [`MessageReader<T>`] and
//! the emission systems write them with [`MessageWriter<T>`].
//!
//! # Usage
//!
//! ```rust,ignore
//! use bevy::prelude::*;
//! use bevy::ecs::message::MessageReader;
//! use crate::vis_events::{ProximityAlert, RobotStateChanged};
//! use crate::vis_api::VisApi;
//!
//! fn my_addon_system(
//!     mut api: VisApi,
//!     mut proximity: MessageReader<ProximityAlert>,
//!     mut state_changes: MessageReader<RobotStateChanged>,
//! ) {
//!     for alert in proximity.read() {
//!         api.log(&format!(
//!             "PROXIMITY: r{} <-> r{} dist={:.2}m",
//!             alert.robot_a, alert.robot_b, alert.distance
//!         ));
//!     }
//!     for change in state_changes.read() {
//!         api.log(&format!(
//!             "STATE CHANGE: robot {} — {:?}",
//!             change.robot_id, change.event_type
//!         ));
//!     }
//! }
//! ```
//!
//! # Available Messages
//!
//! | Message | Emitted when | Typical use |
//! |---------|-------------|-------------|
//! | [`ProximityAlert`] | Two robots closer than `d_safe` | Screenshot, pause, log |
//! | [`SimTickEvent`] | Every frame (or every N frames) | Periodic monitoring |
//! | [`RobotStateChanged`] | Robot connects, disconnects, changes edge, etc. | State tracking |

use bevy::prelude::*;

/// Emitted when two robots are closer than the configured safety distance.
///
/// The emission system compares all robot pairs each frame using their 3D
/// positions from [`RobotStates`](crate::state::RobotStates). Rate-limited
/// to at most once per second per robot pair to avoid log spam.
///
/// # Example
///
/// ```rust,ignore
/// fn on_proximity(
///     mut api: VisApi,
///     mut alerts: MessageReader<ProximityAlert>,
/// ) {
///     for alert in alerts.read() {
///         api.log(&format!(
///             "robots {} and {} are {:.2}m apart (safety: {:.2}m)",
///             alert.robot_a, alert.robot_b, alert.distance, alert.d_safe,
///         ));
///         api.screenshot("/tmp/proximity.png");
///     }
/// }
/// ```
#[derive(Message, Debug, Clone)]
pub struct ProximityAlert {
    /// ID of the first robot in the pair.
    pub robot_a: u32,
    /// ID of the second robot in the pair (always `robot_b > robot_a`).
    pub robot_b: u32,
    /// Euclidean 3D distance between the two robots (metres).
    pub distance: f32,
    /// The safety distance threshold that was violated (metres).
    pub d_safe: f32,
    /// World-space position `[x, y, z]` of robot A.
    pub position_a: [f32; 3],
    /// World-space position `[x, y, z]` of robot B.
    pub position_b: [f32; 3],
}

/// Emitted once per frame with aggregate simulation summary data.
///
/// Useful for addons that want periodic updates without polling individual
/// robot states. The `tick` counter increments each frame the emission system
/// runs.
///
/// # Example
///
/// ```rust,ignore
/// fn on_tick(mut ticks: MessageReader<SimTickEvent>) {
///     for tick in ticks.read() {
///         if tick.min_pair_distance < 0.5 {
///             tracing::warn!("global min distance very low: {:.2}m", tick.min_pair_distance);
///         }
///     }
/// }
/// ```
// Fields are public API for addon consumers; not all are read internally.
#[allow(dead_code)]
#[derive(Message, Debug, Clone)]
pub struct SimTickEvent {
    /// Monotonically increasing frame counter (starts at 0).
    pub tick: u64,
    /// Number of robots currently tracked by the visualiser.
    pub robot_count: usize,
    /// Sum of `active_factor_count` across all robots.
    pub total_ir_factors: usize,
    /// Minimum 3D distance between any pair of robots. `f32::MAX` if < 2 robots.
    pub min_pair_distance: f32,
}

/// Emitted when a robot's state changes in a meaningful way.
///
/// State changes are detected by comparing the current frame's
/// [`RobotStates`](crate::state::RobotStates) against a snapshot from the
/// previous frame. This captures connect/disconnect, edge transitions,
/// factor count changes, and near-collision crossings.
///
/// # Example
///
/// ```rust,ignore
/// fn on_state_change(
///     mut changes: MessageReader<RobotStateChanged>,
/// ) {
///     for change in changes.read() {
///         match &change.event_type {
///             RobotChangeType::Connected => {
///                 tracing::info!("Robot {} connected", change.robot_id);
///             }
///             RobotChangeType::EdgeChanged { from, to } => {
///                 tracing::info!("Robot {} edge {:?} -> {:?}", change.robot_id, from, to);
///             }
///             _ => {}
///         }
///     }
/// }
/// ```
#[derive(Message, Debug, Clone)]
pub struct RobotStateChanged {
    /// The robot whose state changed.
    pub robot_id: u32,
    /// What kind of state change occurred.
    pub event_type: RobotChangeType,
}

/// Describes the specific kind of state change for a robot.
#[derive(Debug, Clone, PartialEq)]
pub enum RobotChangeType {
    /// A new robot appeared in [`RobotStates`](crate::state::RobotStates).
    Connected,
    /// A robot disappeared from [`RobotStates`](crate::state::RobotStates).
    Disconnected,
    /// The robot transitioned from one edge to another.
    EdgeChanged {
        /// The previous edge.
        from: gbp_map::map::EdgeId,
        /// The new edge.
        to: gbp_map::map::EdgeId,
    },
    /// The number of active inter-robot factors changed.
    FactorCountChanged {
        /// Previous factor count.
        from: usize,
        /// New factor count.
        to: usize,
    },
    /// The robot crossed below the near-collision threshold (d_safe).
    NearCollision {
        /// The 3D distance to the nearest neighbour (metres).
        distance: f32,
    },
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proximity_alert_is_constructible() {
        let alert = ProximityAlert {
            robot_a: 0,
            robot_b: 1,
            distance: 0.5,
            d_safe: 1.3,
            position_a: [0.0, 0.0, 0.0],
            position_b: [0.5, 0.0, 0.0],
        };
        assert_eq!(alert.robot_a, 0);
        assert_eq!(alert.robot_b, 1);
        assert!(alert.distance < alert.d_safe);
    }

    #[test]
    fn sim_tick_event_is_constructible() {
        let tick = SimTickEvent {
            tick: 42,
            robot_count: 4,
            total_ir_factors: 8,
            min_pair_distance: 1.5,
        };
        assert_eq!(tick.tick, 42);
        assert_eq!(tick.robot_count, 4);
    }

    #[test]
    fn robot_state_changed_is_constructible() {
        let change = RobotStateChanged {
            robot_id: 3,
            event_type: RobotChangeType::Connected,
        };
        assert_eq!(change.robot_id, 3);
        assert!(matches!(change.event_type, RobotChangeType::Connected));
    }

    #[test]
    fn robot_change_type_variants() {
        // Ensure all variants are constructible and Debug-printable.
        let variants: Vec<RobotChangeType> = vec![
            RobotChangeType::Connected,
            RobotChangeType::Disconnected,
            RobotChangeType::EdgeChanged {
                from: gbp_map::map::EdgeId(0),
                to: gbp_map::map::EdgeId(1),
            },
            RobotChangeType::FactorCountChanged { from: 0, to: 3 },
            RobotChangeType::NearCollision { distance: 0.8 },
        ];
        for v in &variants {
            let _ = format!("{:?}", v);
        }
        assert_eq!(variants.len(), 5);
    }
}
