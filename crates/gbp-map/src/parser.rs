//! YAML -> Map parser. Only compiled with feature = "parse".
//! Uses serde_yaml. Not available in no_std / firmware builds.

extern crate std;

use std::string::String;
use serde::Deserialize;
use crate::map::*;
use crate::nurbs::build_arc_table;

// -- YAML schema types --

#[derive(Deserialize)]
struct YamlMap {
    nodes: std::vec::Vec<YamlNode>,
    edges: std::vec::Vec<YamlEdge>,
}

#[derive(Deserialize)]
struct YamlNode {
    id: String,
    #[serde(rename = "nodePosition")]
    position: YamlPosition,
    #[serde(rename = "customProperties", default)]
    custom: YamlCustomProps,
}

#[derive(Deserialize, Default)]
struct YamlCustomProps {
    #[serde(rename = "type", default)]
    node_type: String,
}

#[derive(Deserialize)]
struct YamlPosition { x: f64, y: f64, z: f64 }

#[derive(Deserialize)]
struct YamlEdge {
    id: String,
    #[serde(rename = "startNodeId")]
    start_node: String,
    #[serde(rename = "endNodeId")]
    end_node: String,
    geometry: YamlGeometry,
    speed_profile: YamlSpeed,
    safety: YamlSafety,
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
enum YamlGeometry {
    Line {
        points: std::vec::Vec<[f64; 3]>,
        length: f64,
    },
    Spline {
        control_points: std::vec::Vec<[f64; 3]>,
        knots: std::vec::Vec<f64>,
        degree: u8,
        length_estimate: f64,
    },
}

#[derive(Deserialize)]
struct YamlSpeed {
    max: f64, nominal: f64, accel_limit: f64, decel_limit: f64,
}

#[derive(Deserialize)]
struct YamlSafety { clearance: f64 }

// -- Public API --

pub fn parse_yaml(yaml: &str) -> Result<Map, String> {
    let raw: YamlMap = serde_yaml::from_str(yaml)
        .map_err(|e| std::format!("YAML parse error: {e}"))?;

    let mut map = Map::new("parsed");
    let mut node_id_map: std::collections::HashMap<String, u16> = std::collections::HashMap::new();

    for (i, yn) in raw.nodes.iter().enumerate() {
        let id = i as u16;
        node_id_map.insert(yn.id.clone(), id);
        let node_type = match yn.custom.node_type.as_str() {
            "merge"     => NodeType::Merge,
            "divert"    => NodeType::Divert,
            "charger"   => NodeType::Charger,
            "toploader" => NodeType::Toploader,
            "discharge" => NodeType::Discharge,
            _           => NodeType::Waypoint,
        };
        map.add_node(Node {
            id: NodeId(id),
            position: [yn.position.x as f32, yn.position.y as f32, yn.position.z as f32],
            node_type,
        }).map_err(|_| std::string::String::from("map node capacity exceeded"))?;
    }

    for (i, ye) in raw.edges.iter().enumerate() {
        let start_id = *node_id_map.get(&ye.start_node)
            .ok_or_else(|| std::format!("unknown start node {}", ye.start_node))?;
        let end_id = *node_id_map.get(&ye.end_node)
            .ok_or_else(|| std::format!("unknown end node {}", ye.end_node))?;

        let geometry = match &ye.geometry {
            YamlGeometry::Line { points, length } => {
                if points.len() < 2 {
                    return Err(std::string::String::from("Line geometry requires at least 2 points"));
                }
                let start = [points[0][0] as f32, points[0][1] as f32, points[0][2] as f32];
                let end   = [points[1][0] as f32, points[1][1] as f32, points[1][2] as f32];
                EdgeGeometry::Line { start, end, length: *length as f32 }
            }
            YamlGeometry::Spline { control_points, knots, degree, .. } => {
                if *degree > 15 {
                    return Err(std::string::String::from("NURBS degree must be <= 15"));
                }
                if control_points.len() < (*degree as usize) + 1 {
                    return Err(std::string::String::from("NURBS requires control_points.len() >= degree + 1"));
                }
                if knots.len() != control_points.len() + (*degree as usize) + 1 {
                    return Err(std::string::String::from("NURBS requires knots.len() == control_points.len() + degree + 1"));
                }
                let mut cps: heapless::Vec<[f32; 3], 16> = heapless::Vec::new();
                for cp in control_points {
                    cps.push([cp[0] as f32, cp[1] as f32, cp[2] as f32])
                        .map_err(|_| std::string::String::from("too many control points"))?;
                }
                let mut kv: heapless::Vec<f32, 32> = heapless::Vec::new();
                for k in knots {
                    kv.push(*k as f32).map_err(|_| std::string::String::from("too many knots"))?;
                }
                let (arc_t, arc_s) = build_arc_table(&cps, &kv, *degree as usize);
                let length = arc_s[crate::ARC_TABLE_SAMPLES - 1];
                EdgeGeometry::Nurbs(NurbsGeometry {
                    control_points: cps, knots: kv, degree: *degree,
                    length, arc_t, arc_s,
                })
            }
        };

        map.add_edge(Edge {
            id: EdgeId(i as u16),
            start: NodeId(start_id),
            end:   NodeId(end_id),
            geometry,
            speed: SpeedProfile {
                max: ye.speed_profile.max as f32,
                nominal: ye.speed_profile.nominal as f32,
                accel_limit: ye.speed_profile.accel_limit as f32,
                decel_limit: ye.speed_profile.decel_limit as f32,
            },
            safety: SafetyProfile { clearance: ye.safety.clearance as f32 },
        }).map_err(|_| std::string::String::from("map edge capacity exceeded"))?;
    }

    Ok(map)
}
