mod state;
mod ws_client;
mod map_scene;
mod robot_render;
mod ui;

use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use bevy::prelude::*;
use bevy::render::renderer::RenderAdapterInfo;
use bevy_egui::EguiPlugin;
use serde::Deserialize;
use state::{DrawConfig, MapRes, RobotStates, WsInbox, WsOutbox};
use map_scene::MapScenePlugin;
use robot_render::RobotRenderPlugin;
use ui::UiPlugin;
use tracing_subscriber::EnvFilter;

/// TOML structure for the [visualisation.draw] section.
#[derive(Deserialize, Default)]
struct VisConfig {
    #[serde(default)]
    visualisation: VisSection,
}

#[derive(Deserialize, Default)]
struct VisSection {
    #[serde(default)]
    draw: DrawToml,
}

#[derive(Deserialize, Default)]
struct DrawToml {
    physical_track: Option<bool>,
    magnetic_mainlines: Option<bool>,
    magnetic_markers: Option<bool>,
    node_spheres: Option<bool>,
    edge_lines: Option<bool>,
    robots: Option<bool>,
    planned_paths: Option<bool>,
    belief_tubes: Option<bool>,
    factor_links: Option<bool>,
}

impl From<DrawToml> for DrawConfig {
    fn from(t: DrawToml) -> Self {
        let d = DrawConfig::default();
        DrawConfig {
            physical_track: t.physical_track.unwrap_or(d.physical_track),
            magnetic_mainlines: t.magnetic_mainlines.unwrap_or(d.magnetic_mainlines),
            magnetic_markers: t.magnetic_markers.unwrap_or(d.magnetic_markers),
            node_spheres: t.node_spheres.unwrap_or(d.node_spheres),
            edge_lines: t.edge_lines.unwrap_or(d.edge_lines),
            robots: t.robots.unwrap_or(d.robots),
            planned_paths: t.planned_paths.unwrap_or(d.planned_paths),
            belief_tubes: t.belief_tubes.unwrap_or(d.belief_tubes),
            factor_links: t.factor_links.unwrap_or(d.factor_links),
        }
    }
}

fn log_gpu_info(adapter: Res<RenderAdapterInfo>) {
    tracing::info!("┌─── GPU Renderer Report ───────────────────────┐");
    tracing::info!("│ Name:    {}", adapter.name);
    tracing::info!("│ Backend: {:?}", adapter.backend);
    tracing::info!("│ Type:    {:?}", adapter.device_type);
    tracing::info!("│ Driver:  {} {}", adapter.driver, adapter.driver_info);
    tracing::info!("└───────────────────────────────────────────────-┘");
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env()
            .add_directive("visualiser=info".parse().unwrap())
            .add_directive("bevy_render::renderer=info".parse().unwrap()))
        .init();

    // Load draw config from the same config.toml the simulator uses
    let config_path = std::env::var("CONFIG_PATH")
        .unwrap_or_else(|_| "config/config.toml".to_string());
    let draw_config = if let Ok(toml_str) = std::fs::read_to_string(&config_path) {
        let vc: VisConfig = toml::from_str(&toml_str).unwrap_or_default();
        DrawConfig::from(vc.visualisation.draw)
    } else {
        DrawConfig::default()
    };

    // Load map (default path or from env)
    let map_path = std::env::var("MAP_PATH")
        .unwrap_or_else(|_| "maps/test_loop_map.yaml".to_string());
    let yaml = std::fs::read_to_string(&map_path)
        .unwrap_or_else(|e| panic!("cannot read map {}: {}", map_path, e));
    let (map, _node_names) = gbp_map::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("map parse error: {}", e));

    // Connect WebSocket client with shutdown flag
    let ws_url = std::env::var("WS_URL")
        .unwrap_or_else(|_| "ws://localhost:3000/ws".to_string());
    let inbox: Arc<Mutex<VecDeque<_>>> = Arc::new(Mutex::new(VecDeque::new()));
    let outbox: Arc<Mutex<VecDeque<String>>> = Arc::new(Mutex::new(VecDeque::new()));
    let ws_shutdown = ws_client::spawn_ws_client(ws_url, Arc::clone(&inbox), Arc::clone(&outbox));

    App::new()
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1))) // dark blue background
        .add_plugins(DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "GBP Trajectory Planner".into(),
                    resolution: (1280u32, 720u32).into(),
                    present_mode: bevy::window::PresentMode::AutoNoVsync,
                    ..default()
                }),
                ..default()
            })
            .set(AssetPlugin {
                file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/../../../assets").to_string(),
                ..default()
            })
        )
        .add_plugins(EguiPlugin::default())
        .add_plugins(bevy_stl::StlPlugin)
        // Most entities are static (environment STLs, node spheres); force-enable
        // transform propagation skipping for unchanged entities.
        .insert_resource(bevy::transform::StaticTransformOptimizations::enabled())
        .insert_resource(MapRes(map))
        .insert_resource(draw_config)
        .insert_resource(RobotStates::default())
        .insert_resource(WsInbox(inbox))
        .insert_resource(WsOutbox(outbox))
        .add_systems(Startup, log_gpu_info)
        .add_plugins(MapScenePlugin)
        .add_plugins(RobotRenderPlugin)
        .add_plugins(UiPlugin)
        .run();

    // App::run() blocks until exit. Set the WS shutdown flag so the background
    // thread stops its reconnect loop and the process can exit cleanly.
    ws_shutdown.store(true, Ordering::Relaxed);
    tracing::info!("app exited, WS client shutdown flag set");
}
