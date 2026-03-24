// src/bins/visualiser/src/addon_config.rs
//
//! Configuration for visualiser addons.
//!
//! All addon settings live in the `[addons]` section of `config/config.toml`.
//! Each addon has an `enabled` toggle plus addon-specific parameters. The UI
//! exposes these as runtime-changeable controls in the Control panel.
//!
//! # Config file example
//!
//! ```toml
//! [addons.screenshot]
//! enabled = false
//! delay_secs = 3.0
//! output_path = "/tmp/vis-screenshot.png"
//! quit_after = false
//!
//! [addons.debug_monitor]
//! enabled = false
//! interval_secs = 2.0
//! proximity_threshold = 1.5
//!
//! [addons.proximity_screenshot]
//! enabled = false
//! output_dir = "/tmp"
//!
//! [addons.state_change_logger]
//! enabled = false
//! ```

use bevy::prelude::*;
use serde::Deserialize;

/// Top-level addon configuration resource, parsed from `[addons]` in config.toml.
///
/// All addons check their `enabled` flag at runtime. When `enabled = false`,
/// the addon system returns immediately with zero overhead.
#[derive(Resource, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct AddonConfig {
    /// Startup screenshot — captures a PNG after a delay.
    pub screenshot: ScreenshotConfig,
    /// Debug monitor — logs robot state periodically.
    pub debug_monitor: DebugMonitorConfig,
    /// Proximity screenshot — captures PNG on proximity alert.
    pub proximity_screenshot: ProximityScreenshotConfig,
    /// State change logger — logs robot state transitions.
    pub state_change_logger: StateChangeLoggerConfig,
}

impl Default for AddonConfig {
    fn default() -> Self {
        Self {
            screenshot: ScreenshotConfig::default(),
            debug_monitor: DebugMonitorConfig::default(),
            proximity_screenshot: ProximityScreenshotConfig::default(),
            state_change_logger: StateChangeLoggerConfig::default(),
        }
    }
}

/// Configuration for the startup screenshot addon.
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct ScreenshotConfig {
    /// Whether the addon is active.
    pub enabled: bool,
    /// Seconds to wait after app start before capturing.
    pub delay_secs: f32,
    /// Output file path for the screenshot PNG.
    pub output_path: String,
    /// If true, quit the app after the screenshot is saved.
    pub quit_after: bool,
}

impl Default for ScreenshotConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            delay_secs: 3.0,
            output_path: "/tmp/vis-screenshot.png".to_string(),
            quit_after: false,
        }
    }
}

/// Configuration for the debug monitor addon.
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct DebugMonitorConfig {
    /// Whether the addon is active.
    pub enabled: bool,
    /// Seconds between periodic log dumps.
    pub interval_secs: f32,
    /// 3D distance threshold (m) for proximity warnings in the log.
    pub proximity_threshold: f32,
}

impl Default for DebugMonitorConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            interval_secs: 2.0,
            proximity_threshold: 1.5,
        }
    }
}

/// Configuration for the proximity screenshot addon.
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct ProximityScreenshotConfig {
    /// Whether the addon is active.
    pub enabled: bool,
    /// Output directory for proximity screenshot PNGs.
    /// Files are named `proximity_{robotA}_{robotB}_{timestamp}.png`.
    pub output_dir: String,
}

impl Default for ProximityScreenshotConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            output_dir: "/tmp".to_string(),
        }
    }
}

/// Configuration for the state change logger addon.
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct StateChangeLoggerConfig {
    /// Whether the addon is active.
    pub enabled: bool,
}

impl Default for StateChangeLoggerConfig {
    fn default() -> Self {
        Self { enabled: false }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_all_disabled() {
        let cfg = AddonConfig::default();
        assert!(!cfg.screenshot.enabled);
        assert!(!cfg.debug_monitor.enabled);
        assert!(!cfg.proximity_screenshot.enabled);
        assert!(!cfg.state_change_logger.enabled);
    }

    #[test]
    fn default_screenshot_config_values() {
        let cfg = ScreenshotConfig::default();
        assert!((cfg.delay_secs - 3.0).abs() < 1e-6);
        assert_eq!(cfg.output_path, "/tmp/vis-screenshot.png");
        assert!(!cfg.quit_after);
    }

    #[test]
    fn default_debug_monitor_config_values() {
        let cfg = DebugMonitorConfig::default();
        assert!((cfg.interval_secs - 2.0).abs() < 1e-6);
        assert!((cfg.proximity_threshold - 1.5).abs() < 1e-6);
    }

    #[test]
    fn deserialize_from_toml() {
        let toml_str = r#"
            [screenshot]
            enabled = true
            delay_secs = 5.0
            output_path = "/tmp/test.png"
            quit_after = true

            [debug_monitor]
            enabled = true
            interval_secs = 1.0
            proximity_threshold = 2.0

            [proximity_screenshot]
            enabled = true

            [state_change_logger]
            enabled = true
        "#;
        let cfg: AddonConfig = toml::from_str(toml_str).unwrap();
        assert!(cfg.screenshot.enabled);
        assert!((cfg.screenshot.delay_secs - 5.0).abs() < 1e-6);
        assert_eq!(cfg.screenshot.output_path, "/tmp/test.png");
        assert!(cfg.screenshot.quit_after);
        assert!(cfg.debug_monitor.enabled);
        assert!((cfg.debug_monitor.interval_secs - 1.0).abs() < 1e-6);
        assert!((cfg.debug_monitor.proximity_threshold - 2.0).abs() < 1e-6);
        assert!(cfg.proximity_screenshot.enabled);
        assert!(cfg.state_change_logger.enabled);
    }

    #[test]
    fn deserialize_partial_toml_uses_defaults() {
        let toml_str = r#"
            [screenshot]
            enabled = true
        "#;
        let cfg: AddonConfig = toml::from_str(toml_str).unwrap();
        assert!(cfg.screenshot.enabled);
        assert!((cfg.screenshot.delay_secs - 3.0).abs() < 1e-6); // default
        assert!(!cfg.debug_monitor.enabled); // default
    }

    #[test]
    fn deserialize_empty_toml_uses_all_defaults() {
        let toml_str = "";
        let cfg: AddonConfig = toml::from_str(toml_str).unwrap();
        assert!(!cfg.screenshot.enabled);
        assert!(!cfg.debug_monitor.enabled);
        assert!(!cfg.proximity_screenshot.enabled);
        assert!(!cfg.state_change_logger.enabled);
    }
}
