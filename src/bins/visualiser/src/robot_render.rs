// src/bins/visualiser/src/robot_render.rs
use bevy::prelude::*;
use gbp_map::map::{Map, EdgeId};
use crate::state::{MapRes, RobotStates, WsInbox};
use tracing::warn;

pub struct RobotRenderPlugin;

impl Plugin for RobotRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_robot_arrow)
           .add_systems(Update, (drain_ws_inbox, update_robot_transforms, draw_planned_path).chain());
    }
}

/// Marker component for the robot arrow entity.
#[derive(Component)]
pub struct RobotArrow { pub robot_id: u32 }

/// Compute 3D world position for a robot at arc-length s on an edge.
/// Applies the map->Bevy coordinate transform (Bevy Y-up; map Z = height).
pub fn robot_world_pos(map: &Map, edge_id: EdgeId, s: f32) -> [f32; 3] {
    match map.eval_position(edge_id, s) {
        Some(p) => [p[0], p[2], -p[1]],
        None => {
            warn!("robot_world_pos: eval_position returned None for edge {:?}", edge_id);
            [0.0, 0.0, 0.0]
        }
    }
}

/// Compute world-space unit tangent for the robot's orientation.
pub fn robot_tangent(map: &Map, edge_id: EdgeId, s: f32) -> [f32; 3] {
    match map.eval_tangent(edge_id, s) {
        Some(t) => [t[0], t[2], -t[1]],
        None => {
            warn!("robot_tangent: eval_tangent returned None for edge {:?}", edge_id);
            [1.0, 0.0, 0.0]
        }
    }
}

/// Sample N evenly-spaced world-space points along an edge (for dashed gizmo).
/// Returns points in Bevy coordinates (map x,y,z -> bevy x,z,-y).
pub fn edge_sample_points(map: &Map, edge_id: EdgeId, n: usize) -> heapless::Vec<[f32; 3], 64> {
    let mut pts: heapless::Vec<[f32; 3], 64> = heapless::Vec::new();
    if let Some(idx) = map.edge_index(edge_id) {
        let len = map.edges[idx].geometry.length();
        for i in 0..n {
            let s = if n > 1 {
                (i as f32 / (n - 1) as f32) * len
            } else {
                0.0
            };
            if let Some(p) = map.eval_position(edge_id, s) {
                let _ = pts.push([p[0], p[2], -p[1]]); // map -> Bevy coord
            }
        }
    }
    pts
}

/// Return (start, end) pairs for every other segment (dashed effect).
/// Draws segments at odd indices i=1,3,5,...
pub fn dashed_segment_pairs(pts: &[[f32; 3]]) -> heapless::Vec<([f32; 3], [f32; 3]), 32> {
    let mut pairs = heapless::Vec::new();
    for i in 1..pts.len() {
        if i % 2 == 1 {
            let _ = pairs.push((pts[i - 1], pts[i]));
        }
    }
    pairs
}

/// Target dash length in meters. Gaps are the same length.
pub const DASH_LENGTH: f32 = 0.3;

fn spawn_robot_arrow(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Cone pointing along +Y in mesh space; we rotate to align with tangent.
    let cone = meshes.add(Cone { radius: 0.2, height: 0.5 });
    let mat  = materials.add(StandardMaterial {
        base_color: Color::srgb(0.1, 0.7, 1.0),
        emissive: bevy::color::LinearRgba::new(0.1, 0.5, 1.0, 1.0),
        ..default()
    });
    commands.spawn((
        Mesh3d(cone),
        MeshMaterial3d(mat),
        Transform::IDENTITY,
        RobotArrow { robot_id: 0 },
    ));
}

/// Drain WsInbox and update RobotStates each frame.
fn drain_ws_inbox(
    inbox: Res<WsInbox>,
    mut states: ResMut<RobotStates>,
) {
    let mut q = inbox.0.lock().unwrap_or_else(|e| e.into_inner());
    while let Some(msg) = q.pop_front() {
        states.0.insert(msg.robot_id, msg);
    }
}

/// Move each robot arrow to its current position.
fn update_robot_transforms(
    map: Res<MapRes>,
    states: Res<RobotStates>,
    mut query: Query<(&RobotArrow, &mut Transform)>,
) {
    for (arrow, mut transform) in query.iter_mut() {
        if let Some(state) = states.0.get(&arrow.robot_id) {
            let pos = robot_world_pos(&map.0, state.current_edge, state.position_s);
            let tan = robot_tangent(&map.0, state.current_edge, state.position_s);
            transform.translation = Vec3::from(pos);
            // Align arrow to tangent (rotate default +Y up to tangent direction)
            let dir = Vec3::from(tan);
            if dir.length_squared() > 0.01 {
                transform.rotation = Quat::from_rotation_arc(Vec3::Y, dir);
            }
        }
    }
}

/// Draw dashed lines along the remaining planned path (from current edge onward).
/// Offset slightly upward (+Y in Bevy) so they're visible above the yellow edge lines.
fn draw_planned_path(
    map: Res<MapRes>,
    states: Res<RobotStates>,
    mut gizmos: Gizmos,
) {
    let color = Color::srgb(0.1, 1.0, 0.3);
    let up = Vec3::new(0.0, 0.08, 0.0); // slight vertical offset above edge lines

    for state in states.0.values() {
        // Skip edges before the current one — only show remaining path
        let current = state.current_edge;
        let mut found_current = false;
        for &edge_id in state.planned_edges.iter() {
            if edge_id == current {
                found_current = true;
            }
            if !found_current {
                continue;
            }
            // Compute sample count from edge length so dashes are a consistent size
            let edge_len = map.0.edge_index(edge_id)
                .map(|i| map.0.edges[i].geometry.length())
                .unwrap_or(1.0);
            let n_samples = ((edge_len / DASH_LENGTH) as usize).clamp(4, 64);
            let pts = edge_sample_points(&map.0, edge_id, n_samples);
            for (a, b) in dashed_segment_pairs(&pts) {
                gizmos.line(Vec3::from(a) + up, Vec3::from(b) + up, color);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gbp_map::map::*;

    #[test]
    fn robot_position_on_line_edge_at_half() {
        let map = {
            let mut m = Map::new("t");
            m.add_node(Node { id:NodeId(0), position:[0.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_node(Node { id:NodeId(1), position:[4.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_edge(Edge {
                id:EdgeId(0), start:NodeId(0), end:NodeId(1),
                geometry: EdgeGeometry::Line { start:[0.0,0.0,0.0], end:[4.0,0.0,0.0], length:4.0 },
                speed: SpeedProfile { max:2.5, nominal:2.0, accel_limit:1.0, decel_limit:1.0 },
                safety: SafetyProfile { clearance:0.3 },
            }).unwrap();
            m
        };
        let pos = robot_world_pos(&map, EdgeId(0), 2.0);
        assert!((pos[0] - 2.0).abs() < 1e-4, "x={}", pos[0]);
        assert!((pos[1] - 0.0).abs() < 1e-4, "y={}", pos[1]);
    }

    #[test]
    fn robot_tangent_on_line_edge_is_unit_x() {
        let map = {
            let mut m = Map::new("t");
            m.add_node(Node { id:NodeId(0), position:[0.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_node(Node { id:NodeId(1), position:[4.0,0.0,0.0], node_type:NodeType::Waypoint }).unwrap();
            m.add_edge(Edge {
                id:EdgeId(0), start:NodeId(0), end:NodeId(1),
                geometry: EdgeGeometry::Line { start:[0.0,0.0,0.0], end:[4.0,0.0,0.0], length:4.0 },
                speed: SpeedProfile { max:2.5, nominal:2.0, accel_limit:1.0, decel_limit:1.0 },
                safety: SafetyProfile { clearance:0.3 },
            }).unwrap();
            m
        };
        let tan = robot_tangent(&map, EdgeId(0), 2.0);
        assert!((tan[0] - 1.0).abs() < 1e-4, "tan.x={}", tan[0]);
        assert!((tan[1]).abs() < 1e-4);
    }

    #[test]
    fn planned_edge_positions_for_two_edges() {
        let mut m = Map::new("t");
        m.add_node(Node { id: NodeId(0), position: [0.0, 0.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(1), position: [3.0, 0.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(2), position: [6.0, 0.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
        let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
        let sf = SafetyProfile { clearance: 0.3 };
        m.add_edge(Edge {
            id: EdgeId(0), start: NodeId(0), end: NodeId(1),
            geometry: EdgeGeometry::Line { start: [0.0, 0.0, 0.0], end: [3.0, 0.0, 0.0], length: 3.0 },
            speed: sp.clone(), safety: sf.clone(),
        }).unwrap();
        m.add_edge(Edge {
            id: EdgeId(1), start: NodeId(1), end: NodeId(2),
            geometry: EdgeGeometry::Line { start: [3.0, 0.0, 0.0], end: [6.0, 0.0, 0.0], length: 3.0 },
            speed: sp, safety: sf,
        }).unwrap();

        let pts = edge_sample_points(&m, EdgeId(0), 4);
        assert_eq!(pts.len(), 4);
        assert!((pts[0][0] - 0.0).abs() < 1e-4);
        assert!((pts[3][0] - 3.0).abs() < 1e-4);
    }

    #[test]
    fn coordinate_transform_swaps_map_y_and_z() {
        let mut m = Map::new("t");
        m.add_node(Node { id: NodeId(0), position: [0.0, 0.0, 0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(1), position: [0.0, 2.0, 1.0], node_type: NodeType::Waypoint }).unwrap();
        let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
        let sf = SafetyProfile { clearance: 0.3 };
        let len = (4.0_f32 + 1.0_f32).sqrt();
        m.add_edge(Edge {
            id: EdgeId(0), start: NodeId(0), end: NodeId(1),
            geometry: EdgeGeometry::Line { start: [0.0, 0.0, 0.0], end: [0.0, 2.0, 1.0], length: len },
            speed: sp, safety: sf,
        }).unwrap();

        let pts = edge_sample_points(&m, EdgeId(0), 2);
        // pts[1] = end point: map (x=0,y=2,z=1) -> bevy (x=0, y=map_z=1, z=-map_y=-2)
        assert!((pts[1][1] - 1.0).abs() < 1e-3, "bevy_y(=map_z) expected 1.0, got {}", pts[1][1]);
        assert!((pts[1][2] - (-2.0)).abs() < 1e-3, "bevy_z(=-map_y) expected -2.0, got {}", pts[1][2]);
    }

    #[test]
    fn planned_path_gizmo_samples_is_8() {
        assert_eq!(PLANNED_PATH_GIZMO_SAMPLES, 8);
    }

    #[test]
    fn dashed_segment_pairs_skips_even_indices() {
        let pts: &[[f32; 3]] = &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ];
        let pairs = dashed_segment_pairs(pts);
        assert_eq!(pairs.len(), 2);
        assert_eq!(pairs[0], ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]));
        assert_eq!(pairs[1], ([2.0, 0.0, 0.0], [3.0, 0.0, 0.0]));
    }
}
