mod state;
mod ws_client;
mod map_scene;
mod robot_render;
mod ui;

use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use state::{MapRes, RobotStates, WsInbox};
use map_scene::MapScenePlugin;
use robot_render::RobotRenderPlugin;
use ui::UiPlugin;
use tracing_subscriber::EnvFilter;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env()
            .add_directive("visualiser=info".parse().unwrap()))
        .init();

    // Load map (default path or from env)
    let map_path = std::env::var("MAP_PATH")
        .unwrap_or_else(|_| "maps/test_loop_map.yaml".to_string());
    let yaml = std::fs::read_to_string(&map_path)
        .unwrap_or_else(|e| panic!("cannot read map {}: {}", map_path, e));
    let map = gbp_map::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("map parse error: {}", e));

    // Connect WebSocket client with shutdown flag
    let ws_url = std::env::var("WS_URL")
        .unwrap_or_else(|_| "ws://localhost:3000/ws".to_string());
    let inbox: Arc<Mutex<VecDeque<_>>> = Arc::new(Mutex::new(VecDeque::new()));
    let ws_shutdown = ws_client::spawn_ws_client(ws_url, Arc::clone(&inbox));

    App::new()
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1))) // dark blue background
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "GBP Trajectory Planner".into(),
                resolution: (1280u32, 720u32).into(),
                present_mode: bevy::window::PresentMode::AutoNoVsync,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(MapRes(map))
        .insert_resource(RobotStates::default())
        .insert_resource(WsInbox(inbox))
        .add_plugins(MapScenePlugin)
        .add_plugins(RobotRenderPlugin)
        .add_plugins(UiPlugin)
        .run();

    // App::run() blocks until exit. Set the WS shutdown flag so the background
    // thread stops its reconnect loop and the process can exit cleanly.
    ws_shutdown.store(true, Ordering::Relaxed);
    tracing::info!("app exited, WS client shutdown flag set");
}
