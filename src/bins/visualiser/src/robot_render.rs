// src/bins/visualiser/src/robot_render.rs
use bevy::prelude::*;
use gbp_map::map::{Map, EdgeId};
use crate::state::{MapRes, RobotStates, WsInbox};
use tracing::warn;

pub struct RobotRenderPlugin;

impl Plugin for RobotRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_robot_arrow)
           .add_systems(Update, (drain_ws_inbox, update_robot_transforms).chain());
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
}
