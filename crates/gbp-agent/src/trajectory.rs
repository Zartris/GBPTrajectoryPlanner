use gbp_map::map::EdgeId;
use gbp_map::MAX_PATH_EDGES;
use heapless::Vec;

/// An ordered sequence of (EdgeId, edge_length) pairs.
pub struct Trajectory {
    /// (edge_id, edge_length_m) for each edge on the planned route
    edges: Vec<(EdgeId, f32), MAX_PATH_EDGES>,
    /// Global arc-length offset of the start of this trajectory
    start_s: f32,
}

impl Trajectory {
    pub fn new(edges: Vec<(EdgeId, f32), MAX_PATH_EDGES>, start_s: f32) -> Self {
        Self { edges, start_s }
    }

    /// Map global arc-length s -> (EdgeId, local_s, is_final_edge).
    /// Clamps to the end of the last edge if s is beyond total length.
    pub fn edge_and_local_s(&self, global_s: f32) -> (EdgeId, f32, bool) {
        let relative_s = (global_s - self.start_s).max(0.0);
        let mut cumulative = 0.0f32;
        for (i, &(edge_id, length)) in self.edges.iter().enumerate() {
            let next = cumulative + length;
            let is_final = i == self.edges.len() - 1;
            if relative_s < next || is_final {
                let local_s = (relative_s - cumulative).clamp(0.0, length);
                return (edge_id, local_s, is_final);
            }
            cumulative = next;
        }
        // Fallback: return end of last edge
        let (last_id, last_len) = *self.edges.last().unwrap();
        (last_id, last_len, true)
    }

    /// Compute v_nom at global_s using the trapezoidal profile.
    /// Tapered only on the final edge; full nominal on all intermediate edges.
    pub fn v_nom_at(&self, global_s: f32, nominal: f32, decel_limit: f32) -> f32 {
        let (_, local_s, is_final) = self.edge_and_local_s(global_s);
        if !is_final {
            return nominal;
        }
        let final_len = self.edges.last().map(|&(_, l)| l).unwrap_or(1.0);
        let remaining = (final_len - local_s).max(0.0);
        let v_taper = libm::sqrtf(2.0 * decel_limit * remaining);
        v_taper.min(nominal).max(0.0)
    }

    pub fn is_empty(&self) -> bool { self.edges.is_empty() }

    /// Total planned distance from start_s.
    pub fn total_length(&self) -> f32 {
        let mut total = 0.0f32;
        for &(_, l) in self.edges.iter() {
            total += l;
        }
        total
    }
}
