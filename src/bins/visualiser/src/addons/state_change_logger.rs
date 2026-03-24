// src/bins/visualiser/src/addons/state_change_logger.rs
//
//! StateChangeLoggerAddon — logs all robot state changes from the event bus.
//!
//! # Activation
//!
//! Disabled by default. Enable via `[addons.state_change_logger]` in
//! config.toml or toggle at runtime in the Addons UI panel.
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
use crate::addon_config::AddonConfig;
use crate::vis_api::VisApi;
use crate::vis_events::{RobotChangeType, RobotStateChanged};

/// Bevy plugin that logs robot state changes.
///
/// Gated by `[addons.state_change_logger].enabled` in config.toml.
pub struct StateChangeLoggerAddon;

impl Plugin for StateChangeLoggerAddon {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, state_change_logger_system);
    }
}

/// Reads [`RobotStateChanged`] messages and logs each one.
fn state_change_logger_system(
    api: VisApi,
    config: Res<AddonConfig>,
    mut changes: MessageReader<RobotStateChanged>,
) {
    if !config.state_change_logger.enabled {
        // Still drain changes to avoid message backup.
        for _ in changes.read() {}
        return;
    }

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
