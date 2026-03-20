// src/bins/visualiser/src/map_scene.rs
use bevy::prelude::*;
use bevy::gizmos::config::{GizmoConfigStore, DefaultGizmoConfigGroup, GizmoLineConfig};
use gbp_map::map::{EdgeGeometry, NodeType};
use crate::state::MapRes;

pub struct MapScenePlugin;

impl Plugin for MapScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, (spawn_map_scene, configure_gizmos))
           .add_systems(Update, draw_edge_gizmos);
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

pub fn midpoint(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [(a[0]+b[0])/2.0, (a[1]+b[1])/2.0, (a[2]+b[2])/2.0]
}

/// Spawn camera, lights, and node spheres.
fn spawn_map_scene(
    mut commands: Commands,
    map: Res<MapRes>,
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

    // Compute map bounding box for camera placement
    let (mut min_x, mut max_x) = (f32::MAX, f32::MIN);
    let (mut min_y, mut max_y) = (f32::MAX, f32::MIN);
    let (mut cx, mut cy, mut cz) = (0.0f32, 0.0f32, 0.0f32);
    let n = map.0.nodes.len().max(1) as f32;
    for node in map.0.nodes.iter() {
        cx += node.position[0]; cy += node.position[1]; cz += node.position[2];
        min_x = min_x.min(node.position[0]); max_x = max_x.max(node.position[0]);
        min_y = min_y.min(node.position[1]); max_y = max_y.max(node.position[1]);
    }
    let center = Vec3::new(cx / n, cz / n, -cy / n);
    let span = ((max_x - min_x).max(max_y - min_y)).max(5.0);

    // Camera: elevated view looking down at map center
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(center.x, span * 1.2, center.z + span * 0.8)
            .looking_at(center, Vec3::Y),
    ));

    // Node spheres with emissive glow
    let sphere_mesh = meshes.add(Sphere::new(0.2).mesh().ico(2).unwrap());

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
        ));
    }
}

/// Draw edge gizmo lines each frame (5px wide, bright yellow).
fn draw_edge_gizmos(
    map: Res<MapRes>,
    mut gizmos: Gizmos,
) {
    const NURBS_SAMPLES: usize = 32;
    let color = Color::srgb(1.0, 1.0, 0.3);

    for edge in map.0.edges.iter() {
        match &edge.geometry {
            EdgeGeometry::Line { start, end, .. } => {
                gizmos.line(map_to_bevy(*start), map_to_bevy(*end), color);
            }
            EdgeGeometry::Nurbs(n) => {
                let mut prev = map_to_bevy(
                    gbp_map::nurbs::eval_point(0.0, &n.control_points, &n.knots, n.degree as usize)
                );
                for i in 1..=NURBS_SAMPLES {
                    let t = i as f32 / NURBS_SAMPLES as f32;
                    let p = gbp_map::nurbs::eval_point(t, &n.control_points, &n.knots, n.degree as usize);
                    let cur = map_to_bevy(p);
                    gizmos.line(prev, cur, color);
                    prev = cur;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

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
