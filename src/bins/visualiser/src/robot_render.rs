// src/bins/visualiser/src/robot_render.rs
use bevy::prelude::*;
use gbp_map::map::{Map, EdgeId};
use gbp_map::MAX_HORIZON;
use crate::state::{CollisionInbox, DrawConfig, MapRes, RobotStates, TraceHistory, WsInbox};
use crate::ui::SimPaused;
use tracing::warn;

pub struct RobotRenderPlugin;

impl Plugin for RobotRenderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TraceHistory>()
           .add_systems(Update, (drain_ws_inbox, spawn_new_robot_meshes, update_robot_transforms, draw_planned_path, draw_belief_tubes, draw_uncertainty_bars, draw_factor_links).chain())
           .add_systems(Update, (sample_trace, draw_traces).chain())
           .add_systems(Update, sync_robot_visibility)
           .add_systems(Update, draw_robot_colliders)
           .add_systems(Update, spawn_collision_markers)
           .add_systems(Update, fade_collision_markers);
    }
}

/// Marker component for collision flash entities.
/// `alpha` starts at 1.0 and decreases each frame until the entity is despawned.
#[derive(Component)]
pub struct CollisionMarker {
    pub alpha: f32,
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


fn spawn_new_robot_meshes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    states: Res<RobotStates>,
    query: Query<&RobotArrow>,
    draw: Res<DrawConfig>,
) {
    let existing: std::collections::HashSet<u32> = query.iter().map(|a| a.robot_id).collect();
    let robot_vis = if draw.robots { Visibility::Visible } else { Visibility::Hidden };

    for &id in states.0.keys() {
        if existing.contains(&id) { continue; }
        let (r, g, b) = ROBOT_COLORS.get(id as usize % ROBOT_COLORS.len())
            .copied().unwrap_or((0.5, 0.5, 0.5));
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(CHASSIS_WIDTH, CHASSIS_HEIGHT, CHASSIS_LENGTH))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(r, g, b),
                emissive: bevy::color::LinearRgba::new(r * 0.3, g * 0.3, b * 0.3, 1.0),
                ..default()
            })),
            Transform::IDENTITY,
            RobotArrow { robot_id: id },
            robot_vis,
        ));
    }
}

/// Drain WsInbox and update RobotStates each frame. Skipped when paused.
fn drain_ws_inbox(
    inbox: Res<WsInbox>,
    mut states: ResMut<RobotStates>,
    mut backend: ResMut<crate::ui::BackendStats>,
    paused: Res<SimPaused>,
) {
    if paused.0 { return; }
    let mut q = inbox.0.lock().unwrap_or_else(|e| e.into_inner());
    while let Some(msg) = q.pop_front() {
        backend.record_message();
        let state = states.0.entry(msg.robot_id).or_default();
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
    draw: Res<DrawConfig>,
    mut gizmos: Gizmos,
) {
    if !draw.planned_paths { return; }
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
    draw: Res<DrawConfig>,
    mut gizmos: Gizmos,
) {
    if !draw.belief_tubes { return; }
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

/// Draw 1D vertical bars at each belief variable position, with height proportional to uncertainty.
fn draw_uncertainty_bars(
    map: Res<MapRes>,
    states: Res<RobotStates>,
    draw: Res<DrawConfig>,
    mut gizmos: Gizmos,
) {
    if !draw.uncertainty_bars { return; }

    for (robot_id, state) in &states.0 {
        let (r, g, b) = ROBOT_COLORS[(*robot_id as usize) % ROBOT_COLORS.len()];
        let edge_ids: Vec<EdgeId> = state.planned_edges.iter().copied().collect();
        let positions = belief_tube_positions_trajectory(&map.0, &edge_ids, &state.belief_means);

        for (i, pos) in positions.iter().enumerate() {
            let variance = state.belief_vars[i].max(0.0);
            let height = variance.sqrt().clamp(0.01, 3.0);

            // pos is already in Bevy space (belief_tube_positions_trajectory returns Bevy coords)
            let base = Vec3::from(*pos) + Vec3::Y * 0.15;
            let top = base + Vec3::Y * height;

            // Draw vertical bar with semi-transparent color
            gizmos.line(base, top, Color::srgba(r, g, b, 0.6));
        }
    }
}

/// Compute a 3-stop color gradient for IR factor lines based on 3D distance.
///
/// Gradient: red (dist <= d_safe) → yellow (dist ≈ 1.5 * d_safe) → green (dist >= 2 * d_safe).
/// `t` is normalized to [0, 1] over the range [d_safe, 2 * d_safe].
fn ir_factor_color(dist: f32, d_safe: f32) -> Color {
    let d = if d_safe > 0.0 { d_safe } else { f32::EPSILON };
    let t = ((dist - d) / d).clamp(0.0, 1.0);
    if t < 0.5 {
        // red → yellow: lerp R=1, G: 0→1, B=0
        let g = t * 2.0;
        Color::srgba(1.0, g, 0.0, 0.7)
    } else {
        // yellow → green: lerp R: 1→0, G=1, B=0
        let r = 1.0 - (t - 0.5) * 2.0;
        Color::srgba(r, 1.0, 0.0, 0.7)
    }
}

/// Draw colored lines between paired variable positions at the same timestep k.
/// For each pair of robots with active IR factors, connects their belief dots.
/// Color encodes proximity: red (inside d_safe) → yellow → green (safe distance).
fn draw_factor_links(
    map: Res<MapRes>,
    states: Res<RobotStates>,
    draw: Res<DrawConfig>,
    mut gizmos: Gizmos,
) {
    if !draw.factor_links { return; }
    // Collect robots with active factors: their belief positions, active timestep set, and 3D pos
    let up = Vec3::new(0.0, 0.15, 0.0);
    let mut robot_data: std::vec::Vec<(u32, std::vec::Vec<Vec3>, std::vec::Vec<u8>, [f32; 3])> = std::vec::Vec::new();

    for (&id, state) in &states.0 {
        if state.active_factor_count == 0 { continue; }
        let edge_ids: std::vec::Vec<EdgeId> = state.planned_edges.iter().copied().collect();
        let pts = belief_tube_positions_trajectory(&map.0, &edge_ids, &state.belief_means);
        let world_pts: std::vec::Vec<Vec3> = pts.iter().map(|p| Vec3::from(*p) + up).collect();
        let active_ks: std::vec::Vec<u8> = state.active_ir_timesteps.iter().copied().collect();
        robot_data.push((id, world_pts, active_ks, state.pos_3d));
    }

    let d_safe = draw.ir_d_safe;

    // Draw lines at timesteps where EITHER robot has an active IR factor.
    // Color is based on the current 3D distance between the robot pair (min_neighbour_dist_3d
    // is per-robot, so we use the straight-line distance between their current 3D positions).
    for i in 0..robot_data.len() {
        for j in (i + 1)..robot_data.len() {
            // Compute current 3D distance between the two robots (map-space, metric)
            let pa = robot_data[i].3;
            let pb = robot_data[j].3;
            let dx = pa[0] - pb[0];
            let dy = pa[1] - pb[1];
            let dz = pa[2] - pb[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            let color = ir_factor_color(dist, d_safe);

            let k_max = robot_data[i].1.len().min(robot_data[j].1.len());
            for k in 0..k_max {
                let k_u8 = k as u8;
                let active_i = robot_data[i].2.contains(&k_u8);
                let active_j = robot_data[j].2.contains(&k_u8);
                if active_i || active_j {
                    let pos_a = robot_data[i].1[k];
                    let pos_b = robot_data[j].1[k];
                    gizmos.line(pos_a, pos_b, color);

                    // Safety distance marker: small sphere at the midpoint, colored by danger level
                    if draw.ir_safety_distance {
                        let mid = (pos_a + pos_b) / 2.0;
                        let danger_color = if dist < d_safe {
                            Color::srgba(1.0, 0.0, 0.0, 0.6)
                        } else {
                            Color::srgba(1.0, 0.65, 0.0, 0.5)
                        };
                        gizmos.sphere(Isometry3d::from_translation(mid), 0.1, danger_color);
                    }
                }
            }
        }
    }
}

/// Sample each robot's current Bevy-space position into the TraceHistory ring buffer.
/// Called every frame; pruning removes robots silent for more than 5 seconds.
fn sample_trace(
    states: Res<RobotStates>,
    mut traces: ResMut<TraceHistory>,
    time: Res<Time>,
    draw: Res<DrawConfig>,
) {
    if !draw.path_traces { return; }
    let now = time.elapsed_secs();
    for (robot_id, state) in &states.0 {
        // pos_3d is already in Bevy coords (same as robot_world_pos output)
        let bevy_pos = Vec3::from(state.pos_3d);
        traces.push(*robot_id, bevy_pos, now);
    }
    traces.prune_stale(now, 5.0);
}

/// Draw path trace lines for all robots when DrawConfig::path_traces is enabled.
fn draw_traces(
    mut gizmos: Gizmos,
    traces: Res<TraceHistory>,
    draw: Res<DrawConfig>,
) {
    if !draw.path_traces { return; }

    for (robot_id, trace) in &traces.traces {
        let (r, g, b) = ROBOT_COLORS
            .get(*robot_id as usize % ROBOT_COLORS.len())
            .copied()
            .unwrap_or((0.5, 0.5, 0.5));
        let color = Color::srgb(r, g, b);
        // linestrip takes an iterator of Vec3 — no heap allocation needed
        gizmos.linestrip(trace.iter().copied(), color);
    }
}

/// Draw wireframe cuboid colliders at each robot's Transform, sized to the chassis dimensions.
fn draw_robot_colliders(
    draw: Res<DrawConfig>,
    query: Query<(&Transform, &RobotArrow)>,
    mut gizmos: Gizmos,
) {
    if !draw.robot_colliders { return; }

    for (transform, arrow) in query.iter() {
        let (r, g, b) = ROBOT_COLORS[arrow.robot_id as usize % ROBOT_COLORS.len()];
        gizmos.cube(
            Transform {
                translation: transform.translation,
                rotation: transform.rotation,
                scale: Vec3::new(CHASSIS_WIDTH, CHASSIS_HEIGHT, CHASSIS_LENGTH),
            },
            Color::srgb(r, g, b),
        );
    }
}

/// Sync Visibility on RobotArrow entities when DrawConfig::robots changes.
fn sync_robot_visibility(
    draw: Res<DrawConfig>,
    mut robots: Query<&mut Visibility, With<RobotArrow>>,
) {
    if !draw.is_changed() { return; }
    let vis = if draw.robots { Visibility::Visible } else { Visibility::Hidden };
    for mut v in &mut robots {
        *v = vis;
    }
}

/// Drain CollisionInbox and spawn a translucent red sphere at each collision midpoint.
/// Gated behind `draw.collision_markers` — skipped when the toggle is off.
fn spawn_collision_markers(
    inbox: Res<CollisionInbox>,
    draw: Res<DrawConfig>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Always drain the inbox so events don't accumulate when the toggle is off.
    let events: std::vec::Vec<_> = {
        let mut q = inbox.0.lock().unwrap_or_else(|e| e.into_inner());
        q.drain(..).collect()
    };
    if !draw.collision_markers { return; }
    for ev in events {
        // pos is in map coords [x, y, z]; convert to Bevy coords: [map_x, map_z, -map_y]
        let bevy_pos = Vec3::new(ev.pos[0], ev.pos[2], -ev.pos[1]);
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(0.3).mesh().build())),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(1.0, 0.1, 0.1, 1.0),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(bevy_pos + Vec3::Y * 0.3),
            CollisionMarker { alpha: 1.0 },
        ));
    }
}

/// Decrease each `CollisionMarker`'s alpha each frame and despawn when fully transparent.
/// Always runs regardless of `draw.collision_markers` so existing markers finish fading.
fn fade_collision_markers(
    mut commands: Commands,
    mut query: Query<(Entity, &mut CollisionMarker, &MeshMaterial3d<StandardMaterial>)>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    time: Res<Time>,
) {
    // Fade to zero over ~3 seconds.
    let fade_rate = time.delta_secs() / 3.0;
    for (entity, mut marker, mat_handle) in query.iter_mut() {
        marker.alpha = (marker.alpha - fade_rate).max(0.0);
        if marker.alpha < 0.01 {
            commands.entity(entity).despawn();
        } else if let Some(mat) = materials.get_mut(mat_handle) {
            mat.base_color = Color::srgba(1.0, 0.1, 0.1, marker.alpha);
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
