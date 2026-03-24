// src/bins/visualiser/src/addons/proximity_screenshot.rs
//
//! ProximityScreenshotAddon — captures a screenshot and logs details whenever a
//! [`ProximityAlert`] is emitted.
//!
//! # Activation
//!
//! Disabled by default. Enable via `[addons.proximity_screenshot]` in
//! config.toml or toggle at runtime in the Addons UI panel.
//!
//! Screenshots are saved to `/tmp/proximity_{robot_a}_{robot_b}_{tick}.png`.
//!
//! # How it works
//!
//! Each frame, the system reads all [`ProximityAlert`] messages from the event
//! bus. For every alert it:
//!
//! 1. Logs the pair, distance, and positions via [`VisApi::log`].
//! 2. Takes a screenshot via [`VisApi::screenshot`].
//!
//! This addon demonstrates the **event-driven pattern**: instead of polling
//! robot positions directly, it reacts to pre-computed events from the
//! emission systems.

use bevy::prelude::*;
use bevy::ecs::message::MessageReader;
use crate::addon_config::AddonConfig;
use crate::vis_api::VisApi;
use crate::vis_events::ProximityAlert;

/// Bevy plugin that screenshots on proximity alerts.
///
/// Gated by `[addons.proximity_screenshot].enabled` in config.toml.
pub struct ProximityScreenshotAddon;

impl Plugin for ProximityScreenshotAddon {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, proximity_screenshot_system);
    }
}

/// Reads [`ProximityAlert`] messages and takes screenshots.
fn proximity_screenshot_system(
    mut api: VisApi,
    config: Res<AddonConfig>,
    mut alerts: MessageReader<ProximityAlert>,
    mut shot_count: Local<u64>,
    mut last_shot_time: Local<f32>,
) {
    if !config.proximity_screenshot.enabled {
        for _ in alerts.read() {}
        return;
    }

    let now = api.elapsed_secs();
    let cooldown = config.proximity_screenshot.cooldown_secs;

    for alert in alerts.read() {
        // Respect cooldown between screenshots
        if now - *last_shot_time < cooldown {
            continue;
        }

        *last_shot_time = now;
        let idx = *shot_count;
        *shot_count += 1;

        api.log(&format!(
            "PROXIMITY SCREENSHOT: r{} <-> r{} dist={:.3}m (d_safe={:.3}m)",
            alert.robot_a, alert.robot_b, alert.distance, alert.d_safe,
        ));

        let dir = &config.proximity_screenshot.output_dir;
        let path = format!(
            "{}/proximity_{}_{}_{}.png",
            dir, alert.robot_a, alert.robot_b, idx
        );
        api.screenshot(&path);
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn addon_module_compiles() {
        // If this test runs, the module compiled successfully.
    }
}
