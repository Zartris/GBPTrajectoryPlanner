// src/bins/visualiser/src/map_scene.rs
use bevy::prelude::*;
use gbp_map::map::{EdgeGeometry, NodeType};
use crate::state::MapRes;

pub struct MapScenePlugin;

impl Plugin for MapScenePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_map_scene)
           .add_systems(Update, draw_edge_gizmos);
    }
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

/// Spawn a sphere for each node.
fn spawn_map_scene(
    mut commands: Commands,
    map: Res<MapRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ambient light
    commands.spawn(AmbientLight { color: Color::WHITE, brightness: 400.0, ..default() });

    // Camera -- positioned to see the full map
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5.0, 12.0, 12.0).looking_at(Vec3::new(5.0, 0.0, 5.0), Vec3::Y),
    ));

    let sphere_mesh = meshes.add(Sphere::new(0.08).mesh().ico(1).unwrap());

    for node in map.0.nodes.iter() {
        let color = node_color(node.node_type);
        let mat = materials.add(StandardMaterial::from_color(color));
        let [x, y, z] = node.position;
        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(mat),
            Transform::from_xyz(x, z, -y), // Bevy: Y-up, map uses Z for height
        ));
    }
}

/// Draw edge gizmos each frame (thin lines -- no mesh allocation).
fn draw_edge_gizmos(
    map: Res<MapRes>,
    mut gizmos: Gizmos,
) {
    const NURBS_SAMPLES: usize = 32;
    for edge in map.0.edges.iter() {
        match &edge.geometry {
            EdgeGeometry::Line { start, end, .. } => {
                let s = Vec3::new(start[0], start[2], -start[1]);
                let e = Vec3::new(end[0],   end[2],   -end[1]);
                gizmos.line(s, e, Color::srgb(0.8, 0.8, 0.8));
            }
            EdgeGeometry::Nurbs(n) => {
                let mut prev = {
                    let p = gbp_map::nurbs::eval_point(0.0, &n.control_points, &n.knots, n.degree as usize);
                    Vec3::new(p[0], p[2], -p[1])
                };
                for i in 1..=NURBS_SAMPLES {
                    let t = i as f32 / NURBS_SAMPLES as f32;
                    let p = gbp_map::nurbs::eval_point(t, &n.control_points, &n.knots, n.degree as usize);
                    let cur = Vec3::new(p[0], p[2], -p[1]);
                    gizmos.line(prev, cur, Color::srgb(0.8, 0.8, 0.8));
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
        // Waypoint: grey (all channels equal, not black)
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
