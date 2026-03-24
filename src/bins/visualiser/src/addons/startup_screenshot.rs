// src/bins/visualiser/src/addons/startup_screenshot.rs
//
// StartupScreenshotAddon — takes a screenshot N seconds after the app starts.
//
// Configuration via `[addons.screenshot]` section in config.toml:
//
//   enabled     — true/false (default: false)
//   delay_secs  — seconds to wait before capturing (default: 3.0)
//   output_path — output file path (default: /tmp/vis-screenshot.png)
//   quit_after  — quit the app after the screenshot is saved (default: false)
//
// Settings can also be changed at runtime via the Addons UI panel.

use bevy::prelude::*;
use crate::addon_config::AddonConfig;
use crate::vis_api::VisApi;

/// Bevy plugin that optionally captures a screenshot after a configurable delay.
pub struct StartupScreenshotAddon;

impl Plugin for StartupScreenshotAddon {
    fn build(&self, app: &mut App) {
        app.init_resource::<ScreenshotState>()
           // screenshot_trigger must run before deferred_quit each frame.
           .add_systems(Update, screenshot_trigger)
           .add_systems(Update, deferred_quit.after(screenshot_trigger));
    }
}

// ---------------------------------------------------------------------------
// Internal resource — tracks one-frame-armed quit state machine
// ---------------------------------------------------------------------------

/// Tracks the two-phase screenshot -> quit state machine.
///
/// After the screenshot is taken, quitting uses a two-frame delay:
///   Frame N:   `quit_pending` set by `screenshot_trigger`.
///   Frame N+1: `deferred_quit` sees `quit_pending`, sets `quit_armed`.
///   Frame N+2: `deferred_quit` sees `quit_armed`, calls `api.quit()`.
#[derive(Resource, Default)]
struct ScreenshotState {
    /// Screenshot has been requested; quit is pending for next frame.
    quit_pending: bool,
    /// Quit was armed last frame; fire AppExit this frame.
    quit_armed: bool,
}

// ---------------------------------------------------------------------------
// Systems
// ---------------------------------------------------------------------------

/// Phase 1: waits for the configured delay, then fires the screenshot.
fn screenshot_trigger(
    mut api: VisApi,
    config: Res<AddonConfig>,
    mut state: ResMut<ScreenshotState>,
    mut done: Local<bool>,
) {
    if *done || !config.screenshot.enabled {
        return;
    }

    if api.elapsed_secs() < config.screenshot.delay_secs {
        return;
    }

    // Delay elapsed — take the screenshot exactly once.
    *done = true;

    let path = &config.screenshot.output_path;
    // Ensure parent directory exists.
    if let Some(parent) = std::path::Path::new(path).parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            tracing::warn!("[screenshot] cannot create dir '{}': {e}", parent.display());
        }
    }
    api.screenshot(path);
    api.log(&format!("startup screenshot -> {path}"));

    if config.screenshot.quit_after {
        api.log("quit_after set — will quit after render flushes screenshot");
        // Arm a two-frame delay (pending -> armed -> quit) so the render pipeline
        // has two full frames to flush the screenshot entity before AppExit.
        state.quit_pending = true;
    }
}

/// Phase 2: fires AppExit two frames after `quit_pending` is set.
///
/// Frame N:   `quit_pending` set by `screenshot_trigger`.
/// Frame N+1: `deferred_quit` sees `quit_pending`, sets `quit_armed`, clears pending.
/// Frame N+2: `deferred_quit` sees `quit_armed`, calls `api.quit()`.
fn deferred_quit(mut api: VisApi, mut state: ResMut<ScreenshotState>) {
    if state.quit_armed {
        api.quit();
        state.quit_armed = false;
        return;
    }
    if state.quit_pending {
        state.quit_pending = false;
        state.quit_armed = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn screenshot_state_default_is_idle() {
        let s = ScreenshotState::default();
        assert!(!s.quit_pending);
        assert!(!s.quit_armed);
    }
}
