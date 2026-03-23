// src/bins/visualiser/src/addons/debug_monitor.rs
//
// DebugMonitorAddon — periodically logs robot positions, velocities, and
// proximity warnings to the tracing output.
//
// Configuration via environment variables (set before launching the visualiser):
//
//   VIS_DEBUG_INTERVAL   — seconds between log dumps (default: 2.0)
//   VIS_DEBUG_PROXIMITY  — distance threshold (m) that triggers a proximity
//                          warning in the log (default: 1.5)
//
// Example usage:
//   VIS_DEBUG_INTERVAL=1.0 VIS_DEBUG_PROXIMITY=2.0 cargo run -p visualiser

use bevy::prelude::*;
use crate::vis_api::VisApi;

/// Bevy plugin that logs robot state at a configurable interval and emits
/// proximity warnings when any two robots come within a threshold distance.
pub struct DebugMonitorAddon;

impl Plugin for DebugMonitorAddon {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugMonitorConfig>()
            .add_systems(Update, debug_monitor_system);
    }
}

// ---------------------------------------------------------------------------
// Config resource
// ---------------------------------------------------------------------------

/// Runtime configuration for the debug monitor.
///
/// Values are read once from environment variables on first use and then
/// cached here.  Changing the env vars after the app starts has no effect.
#[derive(Resource)]
pub struct DebugMonitorConfig {
    /// Seconds between periodic log dumps.
    pub interval_secs: f32,
    /// 3-D distance (m) below which a proximity warning is emitted.
    pub proximity_threshold: f32,
}

impl Default for DebugMonitorConfig {
    fn default() -> Self {
        let interval_secs = std::env::var("VIS_DEBUG_INTERVAL")
            .ok()
            .and_then(|s| s.parse::<f32>().ok())
            .unwrap_or(2.0);
        let proximity_threshold = std::env::var("VIS_DEBUG_PROXIMITY")
            .ok()
            .and_then(|s| s.parse::<f32>().ok())
            .unwrap_or(1.5);
        Self { interval_secs, proximity_threshold }
    }
}

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------

fn debug_monitor_system(
    api: VisApi,
    cfg: Res<DebugMonitorConfig>,
    mut last_log: Local<f32>,
    mut proximity_alerts: MessageReader<crate::vis_events::ProximityAlert>,
) {
    let now = api.elapsed_secs();

    // Periodic state dump.
    if now - *last_log >= cfg.interval_secs {
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
    use super::*;

    #[test]
    fn default_config_uses_env_fallbacks() {
        // Env vars not set in this test — should use built-in defaults.
        // We unset just in case a test runner happens to have them set.
        std::env::remove_var("VIS_DEBUG_INTERVAL");
        std::env::remove_var("VIS_DEBUG_PROXIMITY");
        let cfg = DebugMonitorConfig::default();
        assert!((cfg.interval_secs - 2.0).abs() < 1e-6);
        assert!((cfg.proximity_threshold - 1.5).abs() < 1e-6);
    }
}
