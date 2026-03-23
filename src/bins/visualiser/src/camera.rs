// src/bins/visualiser/src/camera.rs
//
// Orbit/Pan/Zoom camera for the GBP visualiser.
// Uses CursorMoved events (not MouseMotion — broken on X11/WSL2) and
// computes deltas manually from cursor position differences.

use bevy::input::mouse::{MouseScrollUnit, MouseWheel};
use bevy::prelude::*;
use tracing::info;
use bevy::window::PrimaryWindow;
use bevy_egui::input::EguiWantsInput;

use crate::map_scene::MapBounds;
use crate::state::RobotStates;

// ── Mode ────────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
pub enum CameraMode {
    #[default]
    Orbit,
    Pan,
    Follow(u32),
}

// ── State resource ──────────────────────────────────────────────────────────

#[derive(Resource, Debug)]
pub struct CameraState {
    pub mode: CameraMode,
    pub focus: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
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
    fn record_initial(&mut self) {
        self.initial_focus = self.focus;
        self.initial_distance = self.distance;
        self.initial_yaw = self.yaw;
        self.initial_pitch = self.pitch;
    }

    pub fn reset(&mut self) {
        self.focus = self.initial_focus;
        self.distance = self.initial_distance;
        self.yaw = self.initial_yaw;
        self.pitch = self.initial_pitch;
        self.mode = CameraMode::Orbit;
    }

    fn map_to_bevy(pos: [f32; 3]) -> Vec3 {
        Vec3::new(pos[0], pos[2], -pos[1])
    }

    pub fn world_position(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        self.focus + Vec3::new(sy * cp, sp, cy * cp) * self.distance
    }
}

// ── Drag tracker ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DragTracker {
    left_frames: u32,
    right_frames: u32,
    /// Last known cursor position (in window pixels). None = no drag active.
    last_cursor_pos: Option<Vec2>,
}

// ── Marker + Plugin ─────────────────────────────────────────────────────────

#[derive(Component)]
pub struct MainCamera;

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraState>()
            .add_systems(
                Startup,
                spawn_camera.after(crate::map_scene::spawn_map_scene),
            )
            .add_systems(Update, camera_input_system)
            .add_systems(Update, apply_camera_transform.after(camera_input_system));
    }
}

fn spawn_camera(mut commands: Commands, bounds: Res<MapBounds>, mut state: ResMut<CameraState>) {
    state.focus = bounds.center;
    state.distance = bounds.span * 1.5;
    state.pitch = std::f32::consts::FRAC_PI_4;
    state.yaw = 0.0;
    state.record_initial();
    commands.spawn((Camera3d::default(), MainCamera));
}

// ── Constants ───────────────────────────────────────────────────────────────

const ORBIT_SENSITIVITY: f32 = 0.005;
const PAN_MOUSE_SCALE: f32 = 0.003;
const ZOOM_FACTOR: f32 = 0.1;
const ZOOM_MIN: f32 = 0.5;
const ZOOM_MAX: f32 = 200.0;
const PITCH_MIN: f32 = 0.05;
const PITCH_MAX: f32 = std::f32::consts::FRAC_PI_2 - 0.05;
const PAN_KEY_SPEED: f32 = 0.6;

// ── Input system ────────────────────────────────────────────────────────────

fn camera_input_system(
    mut state: ResMut<CameraState>,
    egui_wants: Res<EguiWantsInput>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut cursor_evr: MessageReader<CursorMoved>,
    mut scroll_evr: MessageReader<MouseWheel>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    robot_states: Res<RobotStates>,
    window_q: Query<&Window, With<PrimaryWindow>>,
    mut drag: Local<DragTracker>,
    mut log_timer: Local<f32>,
) {
    let dt = time.delta_secs();
    *log_timer += dt;
    let should_log = *log_timer >= 2.0;
    if should_log {
        *log_timer = 0.0;
    }

    // ── 0. Window focus ─────────────────────────────────────────────────
    let focused = window_q.single().is_ok_and(|w| w.focused);
    if should_log {
        info!("[cam] focused={focused} mode={:?} yaw={:.2} pitch={:.2} dist={:.1}",
            state.mode, state.yaw, state.pitch, state.distance);
    }
    if !focused {
        for _ in cursor_evr.read() {}
        for _ in scroll_evr.read() {}
        drag.left_frames = 0;
        drag.right_frames = 0;
        drag.last_cursor_pos = None;
        return;
    }

    // ── 1. Button state ─────────────────────────────────────────────────
    let left_pressed = mouse_buttons.pressed(MouseButton::Left);
    let right_pressed = mouse_buttons.pressed(MouseButton::Right);
    let any_button = left_pressed || right_pressed;

    if left_pressed {
        drag.left_frames = drag.left_frames.saturating_add(1);
    } else {
        drag.left_frames = 0;
    }
    if right_pressed {
        drag.right_frames = drag.right_frames.saturating_add(1);
    } else {
        drag.right_frames = 0;
    }

    // Reset cursor tracking when no button is held (start fresh on next drag).
    if !any_button {
        drag.last_cursor_pos = None;
    }

    // ── 2. Compute cursor delta from CursorMoved events ─────────────────
    // CursorMoved gives absolute window-space positions.
    // We compute the delta manually, which works correctly on all platforms.
    let mut cursor_delta = Vec2::ZERO;
    for ev in cursor_evr.read() {
        if any_button {
            if let Some(last) = drag.last_cursor_pos {
                cursor_delta += ev.position - last;
            }
            drag.last_cursor_pos = Some(ev.position);
        } else {
            // Track position even when not dragging so we have a start point.
            drag.last_cursor_pos = Some(ev.position);
        }
    }

    let egui_over = egui_wants.is_pointer_over_area();
    let egui_kb = egui_wants.wants_any_keyboard_input();

    if (left_pressed || right_pressed) && cursor_delta != Vec2::ZERO {
        info!(
            "[cam] drag: L={}(f{}) R={}(f{}) delta=({:.1},{:.1}) egui_over={egui_over}",
            left_pressed, drag.left_frames, right_pressed, drag.right_frames,
            cursor_delta.x, cursor_delta.y
        );
    }
    if should_log {
        info!("[cam] egui: over={egui_over} kb={egui_kb}");
    }

    // ── 3. Keyboard ─────────────────────────────────────────────────────
    if !egui_kb {
        if keys.just_pressed(KeyCode::KeyC) {
            let new_mode = match state.mode {
                CameraMode::Orbit | CameraMode::Follow(_) => CameraMode::Pan,
                CameraMode::Pan => CameraMode::Orbit,
            };
            info!("[cam] C pressed: {:?} -> {:?}", state.mode, new_mode);
            state.mode = new_mode;
        }
        if keys.just_pressed(KeyCode::KeyR) {
            info!("[cam] R pressed: reset");
            state.reset();
            return;
        }
        if keys.just_pressed(KeyCode::Tab) {
            let mut ids: Vec<u32> = robot_states.0.keys().copied().collect();
            ids.sort_unstable();
            if !ids.is_empty() {
                let next = match state.mode {
                    CameraMode::Follow(cur) => {
                        let pos = ids.iter().position(|&id| id == cur);
                        ids[pos.map_or(0, |i| (i + 1) % ids.len())]
                    }
                    _ => ids[0],
                };
                info!("[cam] Tab: follow robot {next}");
                state.mode = CameraMode::Follow(next);
            }
        }
        if keys.just_pressed(KeyCode::Escape) && matches!(state.mode, CameraMode::Follow(_)) {
            info!("[cam] Escape: back to Orbit");
            state.mode = CameraMode::Orbit;
        }
        if keys.just_pressed(KeyCode::F1) {
            info!("[cam] F1 pressed (handled in ui.rs toggle_overlays)");
        }
        if keys.just_pressed(KeyCode::F2) {
            info!("[cam] F2 pressed (handled in ui.rs toggle_overlays)");
        }
    }

    // ── 4. Scroll zoom ──────────────────────────────────────────────────
    for ev in scroll_evr.read() {
        if egui_over || matches!(state.mode, CameraMode::Follow(_)) {
            continue;
        }
        let d = match ev.unit {
            MouseScrollUnit::Line => ev.y,
            MouseScrollUnit::Pixel => ev.y * 0.05,
        };
        let old = state.distance;
        state.distance = (state.distance * (1.0 - d * ZOOM_FACTOR)).clamp(ZOOM_MIN, ZOOM_MAX);
        info!("[cam] scroll: d={d:.2} dist {old:.1} -> {:.1}", state.distance);
    }

    // ── 5. Mouse drag ───────────────────────────────────────────────────
    if egui_over || cursor_delta == Vec2::ZERO {
        return;
    }

    match state.mode {
        CameraMode::Orbit => {
            if left_pressed {
                state.yaw -= cursor_delta.x * ORBIT_SENSITIVITY;
                state.pitch = (state.pitch + cursor_delta.y * ORBIT_SENSITIVITY)
                    .clamp(PITCH_MIN, PITCH_MAX);
            } else if right_pressed {
                let right = Vec3::new(state.yaw.cos(), 0.0, -state.yaw.sin());
                let fwd = Vec3::new(-state.yaw.sin(), 0.0, -state.yaw.cos());
                let scale = state.distance * PAN_MOUSE_SCALE;
                state.focus += right * (-cursor_delta.x * scale) + fwd * (cursor_delta.y * scale);
            }
        }
        CameraMode::Pan => {
            let speed = state.distance * PAN_KEY_SPEED;
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

// ── Transform application ───────────────────────────────────────────────────

const FOLLOW_P_GAIN: f32 = 5.0;
const FOLLOW_OFFSET: Vec3 = Vec3::new(0.0, 3.0, 4.0);

fn apply_camera_transform(
    state: Res<CameraState>,
    mut camera: Single<&mut Transform, With<MainCamera>>,
    robot_states: Res<RobotStates>,
    time: Res<Time>,
) {
    match state.mode {
        CameraMode::Follow(rid) => {
            if let Some(rs) = robot_states.0.get(&rid) {
                let rpos = CameraState::map_to_bevy(rs.pos_3d);
                let desired = rpos + FOLLOW_OFFSET;
                let alpha = (time.delta_secs() * FOLLOW_P_GAIN).min(1.0);
                let pos = camera.translation.lerp(desired, alpha);
                **camera = Transform::from_translation(pos).looking_at(rpos, Vec3::Y);
            } else {
                let pos = state.world_position();
                **camera = Transform::from_translation(pos).looking_at(state.focus, Vec3::Y);
            }
        }
        _ => {
            let pos = state.world_position();
            **camera = Transform::from_translation(pos).looking_at(state.focus, Vec3::Y);
        }
    }
}
