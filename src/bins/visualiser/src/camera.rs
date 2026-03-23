// src/bins/visualiser/src/camera.rs
//
// Orbit/Pan/Zoom camera system for the GBP visualiser.
// Operates in Bevy Y-up space (map coords are transformed via map_to_bevy).

use bevy::input::mouse::{AccumulatedMouseMotion, MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use tracing::info;
use bevy_egui::input::EguiWantsInput;

use crate::map_scene::MapBounds;

/// Camera interaction mode.
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub enum CameraMode {
    /// Orbit around a focus point; left-drag rotates, right-drag pans focus, scroll zooms.
    #[default]
    Orbit,
    /// Free pan with WASD / arrow keys; camera looks straight down.
    Pan,
    /// Follow a specific robot with a P-controller smooth camera.
    Follow(u32),
}

/// Resource holding all camera state (written each frame to the Camera3d transform).
#[derive(Resource, Debug)]
pub struct CameraState {
    pub mode: CameraMode,
    /// The point the camera orbits around / looks at.
    pub focus: Vec3,
    /// Distance from the focus point (orbit mode).
    pub distance: f32,
    /// Azimuth angle in radians (horizontal rotation around Y).
    pub yaw: f32,
    /// Elevation angle in radians; clamped to (PITCH_MIN, PITCH_MAX).
    pub pitch: f32,

    // Saved initial values for the R-reset.
    initial_focus: Vec3,
    initial_distance: f32,
    initial_yaw: f32,
    initial_pitch: f32,
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            mode: CameraMode::default(),
            focus: Vec3::ZERO,
            distance: 20.0,
            yaw: 0.0,
            pitch: std::f32::consts::FRAC_PI_4,
            initial_focus: Vec3::ZERO,
            initial_distance: 20.0,
            initial_yaw: 0.0,
            initial_pitch: std::f32::consts::FRAC_PI_4,
        }
    }
}

impl CameraState {
    /// Persist the current values as the reset target.
    fn record_initial(&mut self) {
        self.initial_focus = self.focus;
        self.initial_distance = self.distance;
        self.initial_yaw = self.yaw;
        self.initial_pitch = self.pitch;
    }

    /// Reset to initial values.
    pub fn reset(&mut self) {
        self.focus = self.initial_focus;
        self.distance = self.initial_distance;
        self.yaw = self.initial_yaw;
        self.pitch = self.initial_pitch;
        self.mode = CameraMode::Orbit;
    }

    /// Toggle between Orbit and Pan modes.
    pub fn toggle_mode(&mut self) {
        self.mode = match self.mode {
            CameraMode::Orbit => CameraMode::Pan,
            CameraMode::Pan | CameraMode::Follow(_) => CameraMode::Orbit,
        };
    }

    /// Request follow mode for the next robot (ID 0 as default).
    pub fn request_follow_next(&mut self) {
        self.mode = CameraMode::Follow(0);
    }

    /// Exit follow mode back to orbit.
    pub fn exit_follow(&mut self) {
        if matches!(self.mode, CameraMode::Follow(_)) {
            self.mode = CameraMode::Orbit;
        }
    }

    /// Compute the world-space camera position from orbit parameters.
    pub fn world_position(&self) -> Vec3 {
        let (sin_yaw, cos_yaw) = self.yaw.sin_cos();
        let (sin_pitch, cos_pitch) = self.pitch.sin_cos();
        self.focus
            + Vec3::new(sin_yaw * cos_pitch, sin_pitch, cos_yaw * cos_pitch) * self.distance
    }
}

/// Marker component for the camera entity managed by CameraPlugin.
#[derive(Component)]
pub struct MainCamera;

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraState>()
            // Camera MUST be spawned AFTER spawn_map_scene — not before.
            // On llvmpipe, spawning the camera before other scene entities breaks
            // bevy_egui's render overlay.
            .add_systems(Startup, spawn_camera.after(crate::map_scene::spawn_map_scene));
            // DISABLED for testing:
            // .add_systems(Update, camera_input_system)
            // .add_systems(Update, apply_camera_transform.after(camera_input_system));
    }
}

/// Startup system: spawn the Camera3d entity and initialise CameraState from MapBounds.
fn spawn_camera(mut commands: Commands, bounds: Res<MapBounds>, mut state: ResMut<CameraState>) {
    state.focus = bounds.center;
    state.distance = bounds.span * 1.5;
    state.pitch = std::f32::consts::FRAC_PI_4;
    state.yaw = 0.0;
    state.record_initial();

    let pos = state.world_position();
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(pos).looking_at(state.focus, Vec3::Y),
        // MainCamera removed for testing
    ));
}

const ORBIT_SENSITIVITY: f32 = 0.005;
const ZOOM_FACTOR: f32 = 0.1;
const ZOOM_MIN: f32 = 0.5;
const ZOOM_MAX: f32 = 200.0;
const PITCH_MIN: f32 = 0.02;
const PITCH_MAX: f32 = std::f32::consts::FRAC_PI_2 - 0.02;
const PAN_SPEED_SCALE: f32 = 0.6;

/// Process mouse/keyboard input and update CameraState.
fn camera_input_system(
    mut state: ResMut<CameraState>,
    egui_wants: Res<EguiWantsInput>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mut scroll_evr: MessageReader<MouseWheel>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    // Toggle mode: C key.
    if keys.just_pressed(KeyCode::KeyC) {
        state.mode = match state.mode {
            CameraMode::Orbit | CameraMode::Follow(_) => CameraMode::Pan,
            CameraMode::Pan => CameraMode::Orbit,
        };
    }

    // Reset: R key.
    if keys.just_pressed(KeyCode::KeyR) {
        state.reset();
        return;
    }

    // Guard scroll wheel when the pointer is physically over an egui area.
    let egui_over = egui_wants.is_pointer_over_area();
    // Guard drag and keyboard when egui is actively consuming pointer input.
    let egui_consuming = egui_wants.wants_pointer_input();

    // --- Scroll wheel zoom ---
    for ev in scroll_evr.read() {
        if egui_over {
            continue;
        }
        let delta = match ev.unit {
            MouseScrollUnit::Line => ev.y,
            MouseScrollUnit::Pixel => ev.y * 0.05,
        };
        state.distance = (state.distance * (1.0 - delta * ZOOM_FACTOR))
            .clamp(ZOOM_MIN, ZOOM_MAX);
    }

    if egui_consuming {
        return;
    }

    match state.mode {
        CameraMode::Orbit => {
            // Left mouse drag: rotate yaw/pitch.
            if mouse_buttons.pressed(MouseButton::Left) {
                let delta = mouse_motion.delta;
                state.yaw -= delta.x * ORBIT_SENSITIVITY;
                state.pitch = (state.pitch - delta.y * ORBIT_SENSITIVITY)
                    .clamp(PITCH_MIN, PITCH_MAX);
            // Right mouse drag: pan the focus point laterally.
            } else if mouse_buttons.pressed(MouseButton::Right) {
                let delta = mouse_motion.delta;
                // Build right/forward vectors from current yaw.
                let right = Vec3::new(state.yaw.cos(), 0.0, -state.yaw.sin());
                let fwd = Vec3::new(-state.yaw.sin(), 0.0, -state.yaw.cos());
                // Scale pan speed proportionally to distance.
                let pan_scale = state.distance * 0.002;
                state.focus += right * (-delta.x * pan_scale) + fwd * (delta.y * pan_scale);
            }
        }

        CameraMode::Pan => {
            // WASD / arrow keys: move focus laterally (speed scales with distance).
            let speed = state.distance * PAN_SPEED_SCALE;
            let dt = time.delta_secs();

            let right = Vec3::new(state.yaw.cos(), 0.0, -state.yaw.sin());
            let fwd = Vec3::new(-state.yaw.sin(), 0.0, -state.yaw.cos());

            if keys.pressed(KeyCode::KeyW) || keys.pressed(KeyCode::ArrowUp) {
                state.focus += fwd * speed * dt;
            }
            if keys.pressed(KeyCode::KeyS) || keys.pressed(KeyCode::ArrowDown) {
                state.focus -= fwd * speed * dt;
            }
            if keys.pressed(KeyCode::KeyA) || keys.pressed(KeyCode::ArrowLeft) {
                state.focus -= right * speed * dt;
            }
            if keys.pressed(KeyCode::KeyD) || keys.pressed(KeyCode::ArrowRight) {
                state.focus += right * speed * dt;
            }
        }
        CameraMode::Follow(_) => {}
    }
}

/// Apply the computed camera position/orientation to the Camera3d entity transform.
/// Only writes when CameraState has actually changed (avoids triggering Bevy change
/// detection every frame, which breaks bevy_egui rendering on llvmpipe).
fn apply_camera_transform(
    state: Res<CameraState>,
    mut camera: Single<&mut Transform, With<MainCamera>>,
) {
    if !state.is_changed() { return; }
    let pos = state.world_position();
    **camera = Transform::from_translation(pos).looking_at(state.focus, Vec3::Y);
}
