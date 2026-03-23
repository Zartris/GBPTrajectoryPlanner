// src/bins/visualiser/src/addons/startup_screenshot.rs
//
// StartupScreenshotAddon — takes a screenshot N seconds after the app starts.
//
// Configuration via environment variables (set before launching the visualiser):
//
//   VIS_SCREENSHOT_DELAY   — seconds to wait before capturing (default: unset = disabled)
//   VIS_SCREENSHOT_PATH    — output file path           (default: /tmp/vis-screenshot.png)
//   VIS_SCREENSHOT_QUIT    — "1" or "true" → quit after the screenshot is written
//
// Example usage:
//   VIS_SCREENSHOT_DELAY=4 VIS_SCREENSHOT_PATH=/tmp/out.png cargo run -p visualiser

use bevy::prelude::*;
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
// Internal resource — tracks two-frame state machine
// ---------------------------------------------------------------------------

/// Tracks the two-phase screenshot → quit state machine.
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
    mut state: ResMut<ScreenshotState>,
    mut done: Local<bool>,
) {
    if *done {
        return;
    }

    // Read delay from env; if the variable is absent, the addon does nothing.
    let delay: f32 = match std::env::var("VIS_SCREENSHOT_DELAY") {
        Ok(s) => match s.parse::<f32>() {
            Ok(v) => v,
            Err(_) => {
                tracing::warn!(
                    "[startup_screenshot] VIS_SCREENSHOT_DELAY='{}' is not a valid number — addon disabled",
                    s
                );
                *done = true;
                return;
            }
        },
        // Env var not set — addon is inactive; return silently every frame.
        Err(_) => return,
    };

    if api.elapsed_secs() < delay {
        return;
    }

    // Delay elapsed — take the screenshot exactly once.
    *done = true;

    let path = std::env::var("VIS_SCREENSHOT_PATH")
        .unwrap_or_else(|_| "/tmp/vis-screenshot.png".into());

    api.screenshot(&path);
    api.log(&format!("startup screenshot -> {path}"));

    // Check whether the caller wants the app to quit after the screenshot.
    let should_quit = std::env::var("VIS_SCREENSHOT_QUIT")
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
        .unwrap_or(false);

    if should_quit {
        api.log("VIS_SCREENSHOT_QUIT set — will quit after render flushes screenshot");
        // Arm a two-frame delay so the render pipeline can flush the screenshot
        // entity before we send AppExit.
        state.quit_pending = true;
    }
}

/// Phase 2: fires AppExit one frame after `quit_pending` is set.
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
