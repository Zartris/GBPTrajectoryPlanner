// src/bins/visualiser/src/remote_cmd.rs
//
// File-based remote command system for automated testing.
// Reads commands from /tmp/vis-commands.txt and executes them.
// Works without X11 tools — commands go directly into Bevy systems.

use bevy::prelude::*;
use std::io::BufRead;
use tracing::info;

use crate::camera::CameraState;
use crate::state::DrawConfig;

const CMD_FILE: &str = "/tmp/vis-commands.txt";

pub struct RemoteCmdPlugin;

impl Plugin for RemoteCmdPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, poll_commands);
    }
}

fn poll_commands(
    mut commands: Commands,
    mut camera: ResMut<CameraState>,
    mut draw: ResMut<DrawConfig>,
    mut exit: EventWriter<AppExit>,
) {
    // Read and clear the command file atomically
    let cmds = match std::fs::read_to_string(CMD_FILE) {
        Ok(s) if !s.is_empty() => {
            // Truncate the file after reading
            let _ = std::fs::write(CMD_FILE, "");
            s
        }
        _ => return,
    };

    for line in cmds.lines() {
        let line = line.trim();
        if line.is_empty() { continue; }

        let parts: Vec<&str> = line.splitn(3, ' ').collect();
        match parts[0] {
            "screenshot" => {
                let path = parts.get(1).unwrap_or(&"/tmp/vis-screenshot.png");
                info!("[cmd] screenshot -> {path}");
                commands
                    .spawn(bevy::render::view::screenshot::Screenshot::primary_window())
                    .observe(bevy::render::view::screenshot::save_to_disk(path.to_string()));
            }

            "key" => {
                if let Some(&key) = parts.get(1) {
                    info!("[cmd] key {key}");
                    match key {
                        "R" => camera.reset(),
                        "C" => camera.toggle_mode(),
                        "Tab" => camera.request_follow_next(),
                        "Escape" => camera.exit_follow(),
                        "F1" => { /* handled by toggle_overlays via injected flag */ }
                        "F2" => { /* handled by toggle_overlays via injected flag */ }
                        _ => info!("[cmd] unknown key: {key}"),
                    }
                }
            }

            "orbit" => {
                if let (Some(yaw), Some(pitch)) = (
                    parts.get(1).and_then(|s| s.parse::<f32>().ok()),
                    parts.get(2).and_then(|s| s.parse::<f32>().ok()),
                ) {
                    info!("[cmd] orbit yaw={yaw} pitch={pitch}");
                    camera.yaw = yaw;
                    camera.pitch = pitch;
                }
            }

            "zoom" => {
                if let Some(dist) = parts.get(1).and_then(|s| s.parse::<f32>().ok()) {
                    info!("[cmd] zoom {dist}");
                    camera.distance = dist;
                }
            }

            "draw" => {
                if let (Some(&field), Some(&val)) = (parts.get(1), parts.get(2)) {
                    let on = val == "on" || val == "true" || val == "1";
                    info!("[cmd] draw {field} = {on}");
                    set_draw_field(&mut draw, field, on);
                }
            }

            "draw_all" => {
                let on = parts.get(1).map_or(true, |&v| v == "on" || v == "true");
                info!("[cmd] draw_all {on}");
                set_all_draw(&mut draw, on);
            }

            "quit" => {
                info!("[cmd] quit");
                exit.write(AppExit::Success);
            }

            _ => {
                info!("[cmd] unknown command: {line}");
            }
        }
    }
}

fn set_draw_field(draw: &mut DrawConfig, field: &str, on: bool) {
    match field {
        "physical_track" => draw.physical_track = on,
        "magnetic_mainlines" => draw.magnetic_mainlines = on,
        "magnetic_markers" => draw.magnetic_markers = on,
        "node_spheres" => draw.node_spheres = on,
        "edge_lines" => draw.edge_lines = on,
        "robots" => draw.robots = on,
        "planned_paths" => draw.planned_paths = on,
        "belief_tubes" => draw.belief_tubes = on,
        "factor_links" => draw.factor_links = on,
        "uncertainty_bars" => draw.uncertainty_bars = on,
        "path_traces" => draw.path_traces = on,
        "comm_radius_circles" => draw.comm_radius_circles = on,
        "ir_safety_distance" => draw.ir_safety_distance = on,
        "robot_colliders" => draw.robot_colliders = on,
        "collision_markers" => draw.collision_markers = on,
        "infinite_grid" => draw.infinite_grid = on,
        _ => info!("[cmd] unknown draw field: {field}"),
    }
}

fn set_all_draw(draw: &mut DrawConfig, on: bool) {
    draw.physical_track = on;
    draw.magnetic_mainlines = on;
    draw.magnetic_markers = on;
    draw.node_spheres = on;
    draw.edge_lines = on;
    draw.robots = on;
    draw.planned_paths = on;
    draw.belief_tubes = on;
    draw.factor_links = on;
    draw.uncertainty_bars = on;
    draw.path_traces = on;
    draw.comm_radius_circles = on;
    draw.ir_safety_distance = on;
    draw.robot_colliders = on;
    draw.collision_markers = on;
    draw.infinite_grid = on;
}
