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
//! 5. **Gate behind config** — addons check `Res<AddonConfig>` at runtime:
//!    ```rust,ignore
//!    fn my_system(config: Res<AddonConfig>, mut api: VisApi) {
//!        if !config.my_addon.enabled { return; }
//!        api.log("hello from my addon!");
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
//! use crate::vis_events::{ProximityAlert, RobotStateChanged, DataReceived};
//!
//! fn my_event_addon(
//!     mut api: VisApi,
//!     mut proximity: MessageReader<ProximityAlert>,
//!     mut state_changes: MessageReader<RobotStateChanged>,
//!     mut data: MessageReader<DataReceived>,
//! ) {
//!     for alert in proximity.read() {
//!         api.log(&format!("robots {} & {} too close!", alert.robot_a, alert.robot_b));
//!     }
//!     for change in state_changes.read() {
//!         api.log(&format!("robot {} changed: {:?}", change.robot_id, change.event_type));
//!     }
//!     for batch in data.read() {
//!         if batch.tick % 60 == 0 {
//!             api.log(&format!("tick {} — {} robots", batch.tick, batch.robot_count));
//!         }
//!     }
//! }
//! ```
//!
//! # Configuration
//!
//! All addon settings live in `[addons]` section of config.toml and are
//! exposed as runtime-changeable controls in the Control panel's Addons
//! section. See [`AddonConfig`](crate::addon_config::AddonConfig).
//!
//! # Built-in addons
//!
//! | Addon | Config section | Description |
//! |-------|---------------|-------------|
//! | [`StartupScreenshotAddon`] | `[addons.screenshot]` | Screenshot after N seconds |
//! | [`DebugMonitorAddon`] | `[addons.debug_monitor]` | Periodic robot state logging |
//! | [`ProximityScreenshotAddon`] | `[addons.proximity_screenshot]` | Screenshot on proximity alert |
//! | [`StateChangeLoggerAddon`] | `[addons.state_change_logger]` | Log all robot state changes |

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
///
/// All addons are registered unconditionally. Each addon checks
/// `Res<AddonConfig>` at runtime to decide whether to run, which allows
/// addons to be toggled on/off via the UI without restarting the app.
pub struct AddonPlugins;

impl Plugin for AddonPlugins {
    fn build(&self, app: &mut App) {
        // Event bus — must come first so message types are registered before
        // addon systems try to use MessageReader<T>.
        app.add_plugins(VisEventPlugin);

        // All built-in addons — registered unconditionally.
        // Each addon checks config.enabled internally and returns immediately
        // when disabled. This allows runtime toggling via the Addons UI panel.
        app.add_plugins(startup_screenshot::StartupScreenshotAddon);
        app.add_plugins(debug_monitor::DebugMonitorAddon);
        app.add_plugins(proximity_screenshot::ProximityScreenshotAddon);
        app.add_plugins(state_change_logger::StateChangeLoggerAddon);
    }
}
