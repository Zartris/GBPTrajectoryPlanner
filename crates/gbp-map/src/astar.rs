//! A* path planning on the directed edge graph.
//! Cost: traversal time = edge.length / edge.speed.nominal
//! Heuristic: euclidean(node.pos, goal.pos) / global_max_speed  (admissible)

use heapless::Vec;
use crate::map::{EdgeId, Map, NodeId, NodeType};
use crate::MAX_PATH_EDGES;

/// Returns an ordered list of EdgeIds from start to goal, or None if unreachable.
pub fn astar(map: &Map, start: NodeId, goal: NodeId) -> Option<Vec<EdgeId, MAX_PATH_EDGES>> {
    const MAX_NODES: usize = crate::MAX_NODES;

    // Find global max speed for admissible heuristic
    let max_speed = map.edges.iter().fold(0.1f32, |acc, e| {
        if e.speed.nominal > acc { e.speed.nominal } else { acc }
    });

    let start_idx = map.node_index(start)?;
    let goal_idx  = map.node_index(goal)?;

    // g_score[i] = best traversal time to node i
    let mut g_score = [f32::MAX; MAX_NODES];
    // came_from[i] = (from_node_idx, edge_idx) that leads to node i on best path
    let mut came_from: [Option<(usize, usize)>; MAX_NODES] = [None; MAX_NODES];
    // open set as a simple vec of (f_score, node_idx)
    let mut open: Vec<(f32, usize), MAX_NODES> = Vec::new();

    g_score[start_idx] = 0.0;
    let h0 = heuristic(map, start_idx, goal_idx, max_speed);
    open.push((h0, start_idx)).ok()?;

    while !open.is_empty() {
        // Pop node with lowest f_score
        let (best_pos, _) = open.iter().enumerate()
            .min_by(|(_, a), (_, b)| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal))?;
        let (_, current) = open.swap_remove(best_pos);

        if current == goal_idx {
            return reconstruct_path(map, &came_from, goal_idx);
        }

        // Expand neighbours via outgoing edges (outgoing is indexed by NodeId.0, not Vec position)
        let current_node_id = map.nodes[current].id.0 as usize;
        for &edge_idx in map.outgoing[current_node_id].iter() {
            let edge = &map.edges[edge_idx as usize];
            let neighbor_idx = map.node_index(edge.end)?;

            let edge_cost = edge.geometry.length() / edge.speed.nominal.max(0.001);
            let node_penalty = match map.nodes[neighbor_idx].node_type {
                NodeType::Charger | NodeType::Toploader => 100.0,
                _ => 0.0,
            };
            let tentative_g = g_score[current] + edge_cost + node_penalty;

            if tentative_g < g_score[neighbor_idx] {
                came_from[neighbor_idx] = Some((current, edge_idx as usize));
                g_score[neighbor_idx] = tentative_g;
                let f = tentative_g + heuristic(map, neighbor_idx, goal_idx, max_speed);
                let existing = open.iter_mut().find(|(_, n)| *n == neighbor_idx);
                if let Some(entry) = existing {
                    entry.0 = f;
                } else {
                    let _ = open.push((f, neighbor_idx));
                }
            }
        }
    }
    None
}

fn heuristic(map: &Map, node_idx: usize, goal_idx: usize, max_speed: f32) -> f32 {
    let n = &map.nodes[node_idx].position;
    let g = &map.nodes[goal_idx].position;
    let dx = n[0] - g[0]; let dy = n[1] - g[1]; let dz = n[2] - g[2];
    libm::sqrtf(dx*dx + dy*dy + dz*dz) / max_speed
}

fn reconstruct_path(
    map: &Map,
    came_from: &[Option<(usize, usize)>; crate::MAX_NODES],
    goal_idx: usize,
) -> Option<Vec<EdgeId, MAX_PATH_EDGES>> {
    let mut edges_rev: Vec<EdgeId, MAX_PATH_EDGES> = Vec::new();
    let mut cur = goal_idx;
    loop {
        match came_from[cur] {
            None => break,
            Some((prev, edge_idx)) => {
                edges_rev.push(map.edges[edge_idx].id).ok()?;
                cur = prev;
            }
        }
    }
    // Reverse in-place
    let n = edges_rev.len();
    for i in 0..n / 2 { edges_rev.swap(i, n - 1 - i); }
    if edges_rev.is_empty() { None } else { Some(edges_rev) }
}
