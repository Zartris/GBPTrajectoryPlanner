// src/bins/visualiser/src/map_scene.rs
use bevy::prelude::*;
use bevy::gizmos::config::{GizmoConfigStore, DefaultGizmoConfigGroup, GizmoLineConfig};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin, InfiniteGridSettings};
use gbp_map::map::{EdgeGeometry, EdgeId, NodeType};
use crate::state::{DrawConfig, MapRes, NodeSphere, PhysicalTrackMesh, MainlinesMesh, MarkersMesh};

/// Marker component for the infinite grid entity.
#[derive(Component)]
pub struct InfiniteGridMarker;

/// Axis-aligned bounding box and derived camera-placement helpers for the loaded map.
// min/max reserved for future grid bounds clamping
#[allow(dead_code)]
#[derive(Resource, Debug, Clone)]
pub struct MapBounds {
    /// World-space centre of all map nodes (Bevy Y-up coordinates).
    pub center: Vec3,
    /// Max of (x-span, z-span), at least 5.0. Useful for setting initial camera distance.
    pub span: f32,
    /// Min corner of the bounding box.
    pub min: Vec3,
    /// Max corner of the bounding box.
    pub max: Vec3,
}

/// Cached polylines for each edge — computed once at startup, drawn every frame.
#[derive(Resource)]
pub struct EdgePolylines {
    pub lines: std::vec::Vec<(EdgeId, std::vec::Vec<Vec3>)>,
}

const PHYSICAL_TRACK_STL: &str = "models/physical_track.stl";
const MAGNETIC_MAINLINES_STL: &str = "models/magnetic_mainlines.stl";
const MAGNETIC_MARKERS_STL: &str = "models/magnetic_markers.stl";

/// STL models are authored in millimeters; map coordinates are in meters.
const STL_SCALE: f32 = 0.001;

pub struct MapScenePlugin;

impl Plugin for MapScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InfiniteGridPlugin)
           .add_systems(
                Startup,
                (
                    configure_gizmos,
                    compute_map_bounds,
                    spawn_map_scene,
                    spawn_environment_stl,
                    build_edge_polylines,
                    spawn_infinite_grid,
                ).chain(),
           )
           .add_systems(Update, (draw_edge_gizmos, sync_draw_visibility, sync_infinite_grid_visibility));
    }
}

/// Set gizmo line width to be visible (default is ~1px, invisible over X11).
fn configure_gizmos(mut config_store: ResMut<GizmoConfigStore>) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    config.line = GizmoLineConfig {
        width: 2.0,
        ..default()
    };
}

/// Convert map coordinates [x, y, z] to Bevy world coords (Y-up).
fn map_to_bevy(p: [f32; 3]) -> Vec3 {
    Vec3::new(p[0], p[2], -p[1])
}

/// Colour by node type.
pub fn node_color(nt: NodeType) -> Color {
    match nt {
        NodeType::Waypoint  => Color::srgb(0.55, 0.55, 0.55),
        NodeType::Merge     => Color::srgb(0.2,  0.8,  0.2),
        NodeType::Divert    => Color::srgb(0.2,  0.5,  0.9),
        NodeType::Charger   => Color::srgb(0.9,  0.8,  0.1),
        NodeType::Toploader => Color::srgb(0.9,  0.4,  0.1),
        NodeType::Discharge => Color::srgb(0.7,  0.2,  0.7),
    }
}

#[allow(dead_code)]
pub fn midpoint(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [(a[0]+b[0])/2.0, (a[1]+b[1])/2.0, (a[2]+b[2])/2.0]
}

/// Compute and insert MapBounds from the loaded map. Runs before spawn_map_scene.
pub fn compute_map_bounds(mut commands: Commands, map: Res<MapRes>) {
    let (mut min_x, mut max_x) = (f32::MAX, f32::MIN);
    let (mut min_y, mut max_y) = (f32::MAX, f32::MIN);
    let (mut cx, mut cy, mut cz) = (0.0f32, 0.0f32, 0.0f32);
    let n = map.0.nodes.len().max(1) as f32;
    for node in map.0.nodes.iter() {
        let [x, y, z] = node.position;
        cx += x; cy += y; cz += z;
        min_x = min_x.min(x); max_x = max_x.max(x);
        min_y = min_y.min(y); max_y = max_y.max(y);
    }
    // Map coords (x, y, z) → Bevy (x, z, -y).
    let center = Vec3::new(cx / n, cz / n, -cy / n);
    let span = ((max_x - min_x).max(max_y - min_y)).max(5.0);
    let min_bevy = Vec3::new(min_x, 0.0, -max_y);
    let max_bevy = Vec3::new(max_x, 0.0, -min_y);
    commands.insert_resource(MapBounds { center, span, min: min_bevy, max: max_bevy });
}

/// Spawn lights and node spheres. Camera is handled by CameraPlugin.
pub fn spawn_map_scene(
    mut commands: Commands,
    map: Res<MapRes>,
    draw: Res<DrawConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Strong ambient light
    commands.spawn(AmbientLight { color: Color::WHITE, brightness: 3000.0, ..default() });

    // Directional light (sun)
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(10.0, 20.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Node spheres with emissive glow — always spawned; visibility controlled at runtime
    let sphere_mesh = meshes.add(Sphere::new(0.2).mesh().ico(2).unwrap());
    let ns_vis = if draw.node_spheres { Visibility::Visible } else { Visibility::Hidden };

    for node in map.0.nodes.iter() {
        let color = node_color(node.node_type);
        let srgba = color.to_srgba();
        let mat = materials.add(StandardMaterial {
            base_color: color,
            emissive: bevy::color::LinearRgba::new(srgba.red, srgba.green, srgba.blue, 1.0),
            ..default()
        });
        let [x, y, z] = node.position;
        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(mat),
            Transform::from_xyz(x, z, -y),
            NodeSphere,
            ns_vis,
        ));
    }
}

/// Number of line segments used to render each edge.
pub const NURBS_EDGE_SAMPLES: usize = 32;

/// Cache sampled polylines at startup so draw_edge_gizmos doesn't recompute per frame.
fn build_edge_polylines(
    mut commands: Commands,
    map: Res<MapRes>,
) {
    let mut lines = std::vec::Vec::new();
    for edge in map.0.edges.iter() {
        let n = NURBS_EDGE_SAMPLES;
        let len = edge.geometry.length();
        let mut pts: std::vec::Vec<Vec3> = std::vec::Vec::with_capacity(n + 1);
        let mut last_valid = Vec3::ZERO;
        for i in 0..=n {
            let s = (i as f32 / n as f32) * len;
            let p = match map.0.eval_position(edge.id, s) {
                Some(p) => { let v = map_to_bevy(p); last_valid = v; v }
                None => last_valid, // reuse last valid point instead of jumping to origin
            };
            pts.push(p);
        }
        lines.push((edge.id, pts));
    }
    commands.insert_resource(EdgePolylines { lines });
}

/// Draw edge gizmo lines each frame from cached polylines (2px wide, bright yellow).
fn draw_edge_gizmos(
    polylines: Option<Res<EdgePolylines>>,
    map: Res<MapRes>,
    draw: Res<DrawConfig>,
    mut gizmos: Gizmos,
) {
    if !draw.edge_lines { return; }
    let color = Color::srgb(1.0, 1.0, 0.3);

    if let Some(cache) = polylines {
        // Fast path: draw from cache
        for (_id, pts) in &cache.lines {
            for pair in pts.windows(2) {
                gizmos.line(pair[0], pair[1], color);
            }
        }
    } else {
        // Fallback before cache is built (first frame)
        for edge in map.0.edges.iter() {
            match &edge.geometry {
                EdgeGeometry::Line { start, end, .. } => {
                    gizmos.line(map_to_bevy(*start), map_to_bevy(*end), color);
                }
                EdgeGeometry::Nurbs(n) => {
                    let mut prev = map_to_bevy(
                        gbp_map::nurbs::eval_point(0.0, &n.control_points, &n.knots, n.degree as usize)
                    );
                    for i in 1..=NURBS_EDGE_SAMPLES {
                        let t = i as f32 / NURBS_EDGE_SAMPLES as f32;
                        let p = gbp_map::nurbs::eval_point(t, &n.control_points, &n.knots, n.degree as usize);
                        let cur = map_to_bevy(p);
                        gizmos.line(prev, cur, color);
                        prev = cur;
                    }
                }
            }
        }
    }
}

/// Load and spawn the three environment STL meshes (track, mainlines, markers).
fn spawn_environment_stl(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    draw: Res<DrawConfig>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // STL models are Z-up (CAD convention), Bevy is Y-up.
    // Rotate -90° around X to match map_to_bevy: (x, y, z) → (x, z, -y).
    let stl_transform = Transform {
        scale: Vec3::splat(STL_SCALE),
        rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
        ..default()
    };

    // Physical track — grey (opaque for performance: 56K tris)
    // Always spawned; visibility controlled at runtime via PhysicalTrackMesh marker.
    commands.spawn((
        Mesh3d(asset_server.load(PHYSICAL_TRACK_STL)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.45, 0.45, 0.45),
            ..default()
        })),
        stl_transform,
        PhysicalTrackMesh,
        if draw.physical_track { Visibility::Visible } else { Visibility::Hidden },
    ));

    // Magnetic mainlines — dark blue (opaque for performance: 86K tris)
    commands.spawn((
        Mesh3d(asset_server.load(MAGNETIC_MAINLINES_STL)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.1, 0.7),
            ..default()
        })),
        stl_transform,
        MainlinesMesh,
        if draw.magnetic_mainlines { Visibility::Visible } else { Visibility::Hidden },
    ));

    // Magnetic markers — yellow
    commands.spawn((
        Mesh3d(asset_server.load(MAGNETIC_MARKERS_STL)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.8, 0.0),
            ..default()
        })),
        stl_transform,
        MarkersMesh,
        if draw.magnetic_markers { Visibility::Visible } else { Visibility::Hidden },
    ));
}

/// Sync Visibility on marker entities when DrawConfig changes.
/// Runs every Update frame but is a no-op unless DrawConfig was mutated.
fn sync_draw_visibility(
    draw: Res<DrawConfig>,
    mut node_spheres: Query<&mut Visibility, With<NodeSphere>>,
    mut track_meshes: Query<&mut Visibility, (With<PhysicalTrackMesh>, Without<NodeSphere>)>,
    mut mainlines_meshes: Query<&mut Visibility, (With<MainlinesMesh>, Without<NodeSphere>, Without<PhysicalTrackMesh>)>,
    mut markers_meshes: Query<&mut Visibility, (With<MarkersMesh>, Without<NodeSphere>, Without<PhysicalTrackMesh>, Without<MainlinesMesh>)>,
) {
    if !draw.is_changed() { return; }

    let ns_vis = if draw.node_spheres { Visibility::Visible } else { Visibility::Hidden };
    for mut vis in &mut node_spheres {
        *vis = ns_vis;
    }

    let track_vis = if draw.physical_track { Visibility::Visible } else { Visibility::Hidden };
    for mut vis in &mut track_meshes {
        *vis = track_vis;
    }

    let mainlines_vis = if draw.magnetic_mainlines { Visibility::Visible } else { Visibility::Hidden };
    for mut vis in &mut mainlines_meshes {
        *vis = mainlines_vis;
    }

    let markers_vis = if draw.magnetic_markers { Visibility::Visible } else { Visibility::Hidden };
    for mut vis in &mut markers_meshes {
        *vis = markers_vis;
    }
}

/// Spawn the infinite grid entity. Visibility is driven by DrawConfig at startup.
fn spawn_infinite_grid(mut commands: Commands, draw: Res<DrawConfig>) {
    let vis = if draw.infinite_grid { Visibility::Visible } else { Visibility::Hidden };
    commands.spawn((
        InfiniteGridBundle {
            settings: InfiniteGridSettings {
                minor_line_color: bevy::color::Color::srgba(0.3, 0.3, 0.3, 0.3),
                major_line_color: bevy::color::Color::srgba(0.5, 0.5, 0.5, 0.5),
                fadeout_distance: 200.0,
                ..Default::default()
            },
            visibility: vis,
            ..Default::default()
        },
        InfiniteGridMarker,
    ));
}

/// Sync infinite grid Visibility when DrawConfig changes.
fn sync_infinite_grid_visibility(
    draw: Res<DrawConfig>,
    mut grids: Query<&mut Visibility, With<InfiniteGridMarker>>,
) {
    if !draw.is_changed() { return; }
    let vis = if draw.infinite_grid { Visibility::Visible } else { Visibility::Hidden };
    for mut v in &mut grids {
        *v = vis;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edge_midpoints_computed_correctly() {
        let start = [0.0f32, 0.0, 0.0];
        let end   = [2.0f32, 0.0, 0.0];
        let mid = midpoint(start, end);
        assert!((mid[0] - 1.0).abs() < 1e-5);
        assert!((mid[1] - 0.0).abs() < 1e-5);
    }

    #[test]
    fn node_type_color_waypoint_is_grey() {
        let c = node_color(NodeType::Waypoint);
        let rgba = c.to_srgba();
        assert!(rgba.red > 0.3 && rgba.red < 0.8);
        assert!((rgba.red - rgba.green).abs() < 0.01);
        assert!((rgba.red - rgba.blue).abs() < 0.01);
    }

    #[test]
    fn nurbs_sample_count_is_32() {
        assert_eq!(NURBS_EDGE_SAMPLES, 32);
    }

    #[test]
    fn node_type_color_merge_is_distinct_from_divert() {
        let merge  = node_color(NodeType::Merge);
        let divert = node_color(NodeType::Divert);
        let m = merge.to_srgba();
        let d = divert.to_srgba();
        let diff = (m.red - d.red).abs()
                 + (m.green - d.green).abs()
                 + (m.blue - d.blue).abs();
        assert!(diff > 0.1, "merge and divert colours too similar");
    }
}
