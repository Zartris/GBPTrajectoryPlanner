// src/bins/visualiser/src/robot_render.rs
use bevy::prelude::*;
use gbp_map::map::{Map, EdgeId};
use gbp_map::MAX_HORIZON;
use crate::state::{MapRes, RobotState, RobotStates, WsInbox};
use tracing::warn;

pub struct RobotRenderPlugin;

impl Plugin for RobotRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, (drain_ws_inbox, spawn_new_robot_meshes, update_robot_transforms, draw_planned_path, draw_belief_tubes, draw_factor_links).chain());
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

/// Tube radius = sqrt(variance). Clamp to 0 to avoid NaN.
pub fn belief_tube_radius(variance: f32) -> f32 {
    variance.max(0.0).sqrt()
}

/// Map a global arc-length s to a 3D world position by walking the planned edges.
/// Returns Bevy coordinates (map x,y,z -> bevy x,z,-y).
fn global_s_to_world(map: &Map, planned_edges: &[EdgeId], global_s: f32) -> [f32; 3] {
    let mut cumulative = 0.0;
    for &eid in planned_edges {
        if let Some(idx) = map.edge_index(eid) {
            let len = map.edges[idx].geometry.length();
            if global_s < cumulative + len {
                let local_s = global_s - cumulative;
                return match map.eval_position(eid, local_s) {
                    Some(p) => [p[0], p[2], -p[1]],
                    None => [0.0, 0.0, 0.0],
                };
            }
            cumulative += len;
        }
    }
    // Past end — evaluate at last edge's end
    if let Some(&last_eid) = planned_edges.last() {
        if let Some(idx) = map.edge_index(last_eid) {
            let len = map.edges[idx].geometry.length();
            return match map.eval_position(last_eid, len) {
                Some(p) => [p[0], p[2], -p[1]],
                None => [0.0, 0.0, 0.0],
            };
        }
    }
    [0.0, 0.0, 0.0]
}

/// Evaluate 3D world positions for belief means (global arc-lengths) along a trajectory.
pub fn belief_tube_positions_trajectory(
    map: &Map,
    planned_edges: &[EdgeId],
    means: &[f32],
) -> heapless::Vec<[f32; 3], MAX_HORIZON> {
    let mut pts = heapless::Vec::new();
    for &s in means.iter() {
        let _ = pts.push(global_s_to_world(map, planned_edges, s));
    }
    pts
}

/// Robot arrow colors for multi-robot display.
const ROBOT_COLORS: &[(f32, f32, f32)] = &[
    (0.1, 0.7, 1.0),   // blue
    (1.0, 0.4, 0.1),   // orange
    (0.2, 1.0, 0.4),   // green
    (1.0, 0.2, 0.8),   // pink
];

// Robot chassis dimensions (from carrier URDF)
const CHASSIS_LENGTH: f32 = 1.15; // front to rear (m)
const CHASSIS_WIDTH: f32 = 0.90;  // left to right (m)
const CHASSIS_HEIGHT: f32 = 0.126; // top to bottom (m)

const CHASSIS_STL: &str = "models/chassis.stl";

/// STL models are authored in millimeters; map coordinates are in meters.
const STL_SCALE: f32 = 0.001;

fn spawn_new_robot_meshes(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    states: Res<RobotStates>,
    query: Query<&RobotArrow>,
) {
    // Check which robot IDs already have entities
    let existing: std::vec::Vec<u32> = query.iter().map(|a| a.robot_id).collect();

    for &id in states.0.keys() {
        if existing.contains(&id) { continue; }
        let (r, g, b) = ROBOT_COLORS.get(id as usize % ROBOT_COLORS.len())
            .copied().unwrap_or((0.5, 0.5, 0.5));
        commands.spawn((
            Mesh3d(asset_server.load(CHASSIS_STL)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(r, g, b),
                emissive: bevy::color::LinearRgba::new(r * 0.3, g * 0.3, b * 0.3, 1.0),
                ..default()
            })),
            // Z-up STL → Y-up Bevy: scale mm→m + rotate -90° around X
            Transform {
                scale: Vec3::splat(STL_SCALE),
                rotation: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
                ..default()
            },
            RobotArrow { robot_id: id },
        ));
    }
}

/// Drain WsInbox and update RobotStates each frame.
fn drain_ws_inbox(
    inbox: Res<WsInbox>,
    mut states: ResMut<RobotStates>,
    mut backend: ResMut<crate::ui::BackendStats>,
) {
    let mut q = inbox.0.lock().unwrap_or_else(|e| e.into_inner());
    while let Some(msg) = q.pop_front() {
        backend.record_message();
        let state = states.0.entry(msg.robot_id).or_insert_with(RobotState::default);
        state.update_from_msg(&msg);
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
            // Lift chassis so bottom sits on track (half height above track surface)
            transform.translation = Vec3::from(pos) + Vec3::new(0.0, CHASSIS_HEIGHT / 2.0, 0.0);
            // Align chassis length (Z axis) along travel direction
            let dir = Vec3::from(tan);
            if dir.length_squared() > 0.01 {
                transform.rotation = Quat::from_rotation_arc(Vec3::NEG_Z, dir);
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

/// Draw belief variable positions as dots + uncertainty circles along the planned trajectory.
fn draw_belief_tubes(
    map: Res<MapRes>,
    states: Res<RobotStates>,
    mut gizmos: Gizmos,
) {
    for (robot_id, state) in &states.0 {
        let (r, g, b) = ROBOT_COLORS.get(*robot_id as usize).copied().unwrap_or((0.5, 0.5, 0.5));
        let edge_ids: Vec<EdgeId> = state.planned_edges.iter().copied().collect();
        let pts = belief_tube_positions_trajectory(&map.0, &edge_ids, &state.belief_means);
        let up = Vec3::new(0.0, 0.15, 0.0); // offset above the track

        for (i, &center) in pts.iter().enumerate() {
            let pos = Vec3::from(center) + up;

            // Draw a small dot at each variable's mean position
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                0.05,
                Color::srgb(r, g, b),
            );

            // Draw uncertainty circle (radius = sqrt(variance))
            let radius = belief_tube_radius(state.belief_vars[i]);
            if radius > 0.01 && radius < 5.0 {
                gizmos.circle(
                    Isometry3d::new(pos, Quat::IDENTITY),
                    radius.min(1.0),
                    Color::srgba(r, g, b, 0.4),
                );
            }
        }
    }
}

/// Draw red lines between paired variable positions at the same timestep k.
/// For each pair of robots with active IR factors, connects their belief dots.
fn draw_factor_links(
    map: Res<MapRes>,
    states: Res<RobotStates>,
    mut gizmos: Gizmos,
) {
    // Collect robots with active factors: their belief positions and active timestep set
    let up = Vec3::new(0.0, 0.15, 0.0);
    let mut robot_data: std::vec::Vec<(u32, std::vec::Vec<Vec3>, std::vec::Vec<u8>)> = std::vec::Vec::new();

    for (&id, state) in &states.0 {
        if state.active_factor_count == 0 { continue; }
        let edge_ids: std::vec::Vec<EdgeId> = state.planned_edges.iter().copied().collect();
        let pts = belief_tube_positions_trajectory(&map.0, &edge_ids, &state.belief_means);
        let world_pts: std::vec::Vec<Vec3> = pts.iter().map(|p| Vec3::from(*p) + up).collect();
        let active_ks: std::vec::Vec<u8> = state.active_ir_timesteps.iter().copied().collect();
        robot_data.push((id, world_pts, active_ks));
    }

    // Draw red lines only at timesteps where EITHER robot has an active IR factor
    for i in 0..robot_data.len() {
        for j in (i + 1)..robot_data.len() {
            let k_max = robot_data[i].1.len().min(robot_data[j].1.len());
            for k in 0..k_max {
                let k_u8 = k as u8;
                let active_i = robot_data[i].2.contains(&k_u8);
                let active_j = robot_data[j].2.contains(&k_u8);
                if active_i || active_j {
                    let a = robot_data[i].1[k];
                    let b = robot_data[j].1[k];
                    gizmos.line(a, b, Color::srgba(1.0, 0.2, 0.2, 0.6));
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
        assert!((pts[1][1] - 1.0).abs() < 1e-3, "bevy_y(=map_z) expected 1.0, got {}", pts[1][1]);
        assert!((pts[1][2] - (-2.0)).abs() < 1e-3, "bevy_z(=-map_y) expected -2.0, got {}", pts[1][2]);
    }

    #[test]
    fn dash_length_is_positive() {
        assert!(DASH_LENGTH > 0.0);
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

    #[test]
    fn belief_tube_radius_is_sqrt_variance() {
        let var: f32 = 4.0;
        let radius = belief_tube_radius(var);
        assert!((radius - 2.0).abs() < 1e-4, "expected sqrt(4)=2, got {}", radius);
    }

    #[test]
    fn belief_tube_radius_clamps_near_zero_variance() {
        let radius = belief_tube_radius(0.0);
        assert!(radius >= 0.0 && radius.is_finite());
    }

    #[test]
    fn belief_positions_len_matches_k() {
        let mut m = Map::new("t");
        m.add_node(Node { id: NodeId(0), position: [0.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        m.add_node(Node { id: NodeId(1), position: [5.0,0.0,0.0], node_type: NodeType::Waypoint }).unwrap();
        let sp = SpeedProfile { max: 2.5, nominal: 2.0, accel_limit: 1.0, decel_limit: 1.0 };
        let sf = SafetyProfile { clearance: 0.3 };
        m.add_edge(Edge {
            id: EdgeId(0), start: NodeId(0), end: NodeId(1),
            geometry: EdgeGeometry::Line { start: [0.0,0.0,0.0], end: [5.0,0.0,0.0], length: 5.0 },
            speed: sp, safety: sf,
        }).unwrap();

        let means: Vec<f32> = (0..8).map(|i| i as f32 * 0.5).collect();
        let edges = &[EdgeId(0)];
        let positions = belief_tube_positions_trajectory(&m, edges, &means);
        assert_eq!(positions.len(), means.len());
    }
}
