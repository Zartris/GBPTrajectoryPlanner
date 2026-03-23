// src/bins/visualiser/src/addons/mod.rs
//
//! Addon registry — each addon is a Bevy Plugin.
//!
//! # How to create a new addon
//!
//! 1. **Create the file**: `src/bins/visualiser/src/addons/my_addon.rs`
//!
//! 2. **Declare the module** by adding `mod my_addon;` below.
//!
//! 3. **Write a plugin struct** that implements `bevy::prelude::Plugin`:
//!
//!    ```rust,ignore
//!    use bevy::prelude::*;
//!    use crate::vis_api::VisApi;
//!
//!    pub struct MyAddon;
//!
//!    impl Plugin for MyAddon {
//!        fn build(&self, app: &mut App) {
//!            app.add_systems(Update, my_system);
//!        }
//!    }
//!
//!    fn my_system(mut api: VisApi) {
//!        api.log("hello from my addon!");
//!    }
//!    ```
//!
//! 4. **Register it** in `AddonPlugins::build` below:
//!    ```rust,ignore
//!    app.add_plugins(my_addon::MyAddon);
//!    ```
//!
//! 5. **(Optional) Gate behind an env var** for opt-in activation:
//!    ```rust,ignore
//!    impl Plugin for MyAddon {
//!        fn build(&self, app: &mut App) {
//!            let enabled = std::env::var("VIS_MY_ADDON")
//!                .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
//!                .unwrap_or(false);
//!            if enabled {
//!                app.add_systems(Update, my_system);
//!            }
//!        }
//!    }
//!    ```
//!
//! # Event-driven addons
//!
//! Addons can react to events from the event bus instead of polling every
//! frame. Import [`MessageReader`](bevy::ecs::message::MessageReader) and
//! the event types from [`vis_events`](crate::vis_events):
//!
//! ```rust,ignore
//! use bevy::prelude::*;
//! use bevy::ecs::message::MessageReader;
//! use crate::vis_api::VisApi;
//! use crate::vis_events::{ProximityAlert, RobotStateChanged, SimTickEvent};
//!
//! fn my_event_addon(
//!     mut api: VisApi,
//!     mut proximity: MessageReader<ProximityAlert>,
//!     mut state_changes: MessageReader<RobotStateChanged>,
//!     mut ticks: MessageReader<SimTickEvent>,
//! ) {
//!     for alert in proximity.read() {
//!         api.log(&format!("robots {} & {} too close!", alert.robot_a, alert.robot_b));
//!     }
//!     for change in state_changes.read() {
//!         api.log(&format!("robot {} changed: {:?}", change.robot_id, change.event_type));
//!     }
//!     for tick in ticks.read() {
//!         if tick.tick % 60 == 0 {
//!             api.log(&format!("tick {} — {} robots", tick.tick, tick.robot_count));
//!         }
//!     }
//! }
//! ```
//!
//! # Built-in addons
//!
//! | Addon | Env var | Description |
//! |-------|---------|-------------|
//! | [`StartupScreenshotAddon`] | `VIS_SCREENSHOT_DELAY` | Screenshot after N seconds |
//! | [`DebugMonitorAddon`] | `VIS_DEBUG_INTERVAL` | Periodic robot state logging |
//! | [`ProximityScreenshotAddon`] | `VIS_PROXIMITY_SCREENSHOT=1` | Screenshot on proximity alert |
//! | [`StateChangeLoggerAddon`] | `VIS_LOG_STATE_CHANGES=1` | Log all robot state changes |

mod startup_screenshot;
mod debug_monitor;
mod proximity_screenshot;
mod state_change_logger;

use bevy::prelude::*;
use crate::vis_event_systems::VisEventPlugin;

/// Aggregate plugin that registers the event system and every addon with the
/// Bevy app.
///
/// The event system ([`VisEventPlugin`]) must be registered before any addon
/// that reads event messages. This plugin handles the ordering automatically.
pub struct AddonPlugins;

impl Plugin for AddonPlugins {
    fn build(&self, app: &mut App) {
        // Event bus — must come first so message types are registered before
        // addon systems try to use MessageReader<T>.
        app.add_plugins(VisEventPlugin);

        // Built-in addons — screenshot addon is always available (no-ops without env vars).
        app.add_plugins(startup_screenshot::StartupScreenshotAddon);

        // Debug/diagnostic addons — only active when their env var is set.
        // These addons check their env var internally and no-op if not configured,
        // but we gate registration too to avoid unnecessary system overhead.
        if std::env::var("VIS_DEBUG_INTERVAL").is_ok() || std::env::var("VIS_DEBUG_PROXIMITY").is_ok() {
            app.add_plugins(debug_monitor::DebugMonitorAddon);
        }
        if std::env::var("VIS_PROXIMITY_SCREENSHOT").is_ok() {
            app.add_plugins(proximity_screenshot::ProximityScreenshotAddon);
        }
        if std::env::var("VIS_LOG_STATE_CHANGES").is_ok() {
            app.add_plugins(state_change_logger::StateChangeLoggerAddon);
        }
    }
}
