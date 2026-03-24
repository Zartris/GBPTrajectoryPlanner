// src/bins/visualiser/src/addons/debug_monitor.rs
//
// DebugMonitorAddon — periodically logs robot positions, velocities, and
// proximity warnings to the tracing output.
//
// Configuration via `[addons.debug_monitor]` section in config.toml:
//
//   enabled             — true/false (default: false)
//   interval_secs       — seconds between log dumps (default: 2.0)
//   proximity_threshold — distance (m) for proximity warnings (default: 1.5)
//
// Settings can also be changed at runtime via the Addons UI panel.

use bevy::prelude::*;
use crate::addon_config::AddonConfig;
use crate::vis_api::VisApi;

/// Bevy plugin that logs robot state at a configurable interval and emits
/// proximity warnings when any two robots come within a threshold distance.
pub struct DebugMonitorAddon;

impl Plugin for DebugMonitorAddon {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, debug_monitor_system);
    }
}

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------

fn debug_monitor_system(
    api: VisApi,
    config: Res<AddonConfig>,
    mut last_log: Local<f32>,
    mut proximity_alerts: MessageReader<crate::vis_events::ProximityAlert>,
) {
    if !config.debug_monitor.enabled {
        // Still drain alerts to avoid message backup.
        for _ in proximity_alerts.read() {}
        return;
    }

    let now = api.elapsed_secs();

    // Periodic state dump.
    if now - *last_log >= config.debug_monitor.interval_secs {
        *last_log = now;
        let ids = api.robot_ids();
        if ids.is_empty() {
            tracing::info!("[debug_monitor] t={:.1}s — no robots tracked", now);
        } else {
            tracing::info!(
                "[debug_monitor] t={:.1}s — {} robot(s):",
                now,
                ids.len()
            );
            for id in &ids {
                let pos = api.robot_position(*id).unwrap_or([0.0; 3]);
                let vel = api.robot_velocity(*id).unwrap_or(0.0);
                let factors = api.robot_factor_count(*id).unwrap_or(0);
                let min_dist = api.robot_min_distance(*id).unwrap_or(f32::MAX);
                tracing::info!(
                    "[debug_monitor]   robot {id}: pos=({:.2},{:.2},{:.2}) vel={:.3} m/s  \
                     factors={factors}  min_dist={:.3}",
                    pos[0], pos[1], pos[2], vel, min_dist
                );
            }
        }
    }

    // Proximity warnings — use ProximityAlert events instead of per-frame O(n^2).
    // The VisEventPlugin already rate-limits alerts to 1/sec per pair.
    for alert in proximity_alerts.read() {
        tracing::warn!(
            "[debug_monitor] PROXIMITY t={:.1}s  robots {} & {}  dist={:.3}m < {:.3}m",
            now, alert.robot_a, alert.robot_b, alert.distance, alert.d_safe
        );
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use crate::addon_config::DebugMonitorConfig;

    #[test]
    fn default_config_values() {
        let cfg = DebugMonitorConfig::default();
        assert!((cfg.interval_secs - 2.0).abs() < 1e-6);
        assert!((cfg.proximity_threshold - 1.5).abs() < 1e-6);
        assert!(!cfg.enabled);
    }
}
