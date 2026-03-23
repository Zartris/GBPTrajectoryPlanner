// src/bins/visualiser/src/addons/mod.rs
//
// Addon registry — each addon is a Bevy Plugin.
//
// To add a new addon:
//   1. Create `src/bins/visualiser/src/addons/my_addon.rs`
//   2. Add `mod my_addon;` below.
//   3. Register `my_addon::MyAddon` inside `AddonPlugins::build`.
//
// Addons can use `VisApi` (see `vis_api.rs`) for a stable API surface, or
// reach directly into Bevy resources for full power.

mod startup_screenshot;

use bevy::prelude::*;

/// Aggregate plugin that registers every addon with the Bevy app.
pub struct AddonPlugins;

impl Plugin for AddonPlugins {
    fn build(&self, app: &mut App) {
        app.add_plugins(startup_screenshot::StartupScreenshotAddon);
        // Add further addons here, e.g.:
        // app.add_plugins(my_addon::MyAddon);
    }
}
