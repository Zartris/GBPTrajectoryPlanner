mod state;
mod ws_client;
mod map_scene;
mod camera;
mod robot_render;
mod ui;
mod vis_api;
mod addons;

use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use bevy::prelude::*;
use bevy::render::renderer::RenderAdapterInfo;
use bevy_egui::EguiPlugin;
// bevy_inspector_egui — used via DefaultInspectorConfigPlugin + manual ui_for_world
use serde::Deserialize;
use state::{DrawConfig, InspectorVisible, MapRes, RobotStates, WsInbox, WsOutbox};
use map_scene::MapScenePlugin;
use camera::CameraPlugin;
use robot_render::RobotRenderPlugin;
use ui::UiPlugin;
use tracing_subscriber::EnvFilter;

/// TOML structure for the [visualisation.draw] section plus [gbp.interrobot].
#[derive(Deserialize, Default)]
struct VisConfig {
    #[serde(default)]
    visualisation: VisSection,
    #[serde(default)]
    gbp: GbpSection,
}

#[derive(Deserialize, Default)]
struct GbpSection {
    #[serde(default)]
    interrobot: InterRobotToml,
}

#[derive(Deserialize, Default)]
struct InterRobotToml {
    d_safe: Option<f32>,
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
    uncertainty_bars: Option<bool>,
    path_traces: Option<bool>,
    comm_radius_circles: Option<bool>,
    ir_safety_distance: Option<bool>,
    robot_colliders: Option<bool>,
    collision_markers: Option<bool>,
    infinite_grid: Option<bool>,
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
            uncertainty_bars: t.uncertainty_bars.unwrap_or(d.uncertainty_bars),
            path_traces: t.path_traces.unwrap_or(d.path_traces),
            comm_radius_circles: t.comm_radius_circles.unwrap_or(d.comm_radius_circles),
            ir_safety_distance: t.ir_safety_distance.unwrap_or(d.ir_safety_distance),
            robot_colliders: t.robot_colliders.unwrap_or(d.robot_colliders),
            collision_markers: t.collision_markers.unwrap_or(d.collision_markers),
            infinite_grid: t.infinite_grid.unwrap_or(d.infinite_grid),
            ir_d_safe: d.ir_d_safe, // populated separately from [gbp.interrobot]
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
        let mut dc = DrawConfig::from(vc.visualisation.draw);
        // Propagate [gbp.interrobot] d_safe so the IR color gradient uses the real value
        if let Some(d) = vc.gbp.interrobot.d_safe {
            dc.ir_d_safe = d.max(1e-3);
        }
        dc
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
        // Cap frame rate: ~60 FPS when focused, ~10 FPS when unfocused.
        // Critical for software renderers (llvmpipe) to avoid 100% CPU.
        .insert_resource(bevy::winit::WinitSettings {
            focused_mode: bevy::winit::UpdateMode::reactive(
                std::time::Duration::from_millis(16),
            ),
            unfocused_mode: bevy::winit::UpdateMode::reactive(
                std::time::Duration::from_millis(100),
            ),
        })
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1))) // dark blue background
        .add_plugins(DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "GBP Trajectory Planner".into(),
                    resolution: (1280u32, 720u32).into(),
                    present_mode: bevy::window::PresentMode::AutoVsync,
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
        .init_resource::<InspectorVisible>()
        // disabled: breaks egui on llvmpipe
        
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
        .add_plugins(CameraPlugin)
        .add_plugins(RobotRenderPlugin)
        .add_plugins(UiPlugin)
        .add_plugins(addons::AddonPlugins)
        .run();

    // App::run() blocks until exit. Set the WS shutdown flag so the background
    // thread stops its reconnect loop and the process can exit cleanly.
    ws_shutdown.store(true, Ordering::Relaxed);
    tracing::info!("app exited, WS client shutdown flag set");
}

// auto_screenshot removed — use vis_api addon system instead
