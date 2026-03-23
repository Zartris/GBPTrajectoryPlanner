// src/bins/visualiser/src/addons/state_change_logger.rs
//
//! StateChangeLoggerAddon — logs all robot state changes from the event bus.
//!
//! # Activation
//!
//! Disabled by default. Enable by setting the environment variable before
//! launching the visualiser:
//!
//! ```bash
//! VIS_LOG_STATE_CHANGES=1 cargo run -p visualiser
//! ```
//!
//! # How it works
//!
//! Each frame, the system reads all [`RobotStateChanged`] messages and logs
//! them at `info` level via [`VisApi::log`]. This is useful for:
//!
//! - Debugging robot lifecycle (connect/disconnect)
//! - Tracking edge transitions
//! - Monitoring inter-robot factor activity
//! - Detecting near-collision crossings
//!
//! This addon demonstrates the **event-driven state-tracking pattern**: instead
//! of comparing states manually, it subscribes to pre-computed change events.

use bevy::prelude::*;
use bevy::ecs::message::MessageReader;
use crate::vis_api::VisApi;
use crate::vis_events::{RobotChangeType, RobotStateChanged};

/// Bevy plugin that logs robot state changes.
///
/// Gated behind `VIS_LOG_STATE_CHANGES=1` environment variable.
pub struct StateChangeLoggerAddon;

impl Plugin for StateChangeLoggerAddon {
    fn build(&self, app: &mut App) {
        let enabled = std::env::var("VIS_LOG_STATE_CHANGES")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);

        if enabled {
            tracing::info!("[state_change_logger] addon enabled");
            app.add_systems(Update, state_change_logger_system);
        }
    }
}

/// Reads [`RobotStateChanged`] messages and logs each one.
fn state_change_logger_system(
    api: VisApi,
    mut changes: MessageReader<RobotStateChanged>,
) {
    for change in changes.read() {
        let msg = match &change.event_type {
            RobotChangeType::Connected => {
                format!("robot {} CONNECTED", change.robot_id)
            }
            RobotChangeType::Disconnected => {
                format!("robot {} DISCONNECTED", change.robot_id)
            }
            RobotChangeType::EdgeChanged { from, to } => {
                format!(
                    "robot {} edge {:?} -> {:?}",
                    change.robot_id, from, to
                )
            }
            RobotChangeType::FactorCountChanged { from, to } => {
                format!(
                    "robot {} factors {} -> {}",
                    change.robot_id, from, to
                )
            }
            RobotChangeType::NearCollision { distance } => {
                format!(
                    "robot {} NEAR COLLISION dist={:.3}m",
                    change.robot_id, distance
                )
            }
        };
        api.log(&format!("[state_change] {}", msg));
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn addon_module_compiles() {
        // If this test runs, the module compiled successfully.
    }
}
