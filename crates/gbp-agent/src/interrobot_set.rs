//! Maintains the mapping: robot_id -> factor index in FactorGraph.
//! Handles index patching after O(1) swap-remove.

use heapless::Vec;
use gbp_comms::RobotId;
use gbp_core::FactorGraph;
use gbp_map::MAX_NEIGHBOURS;

pub struct InterRobotFactorSet {
    // (robot_id, factor_idx in FactorGraph)
    entries: Vec<(RobotId, usize), MAX_NEIGHBOURS>,
}

impl InterRobotFactorSet {
    pub fn new() -> Self { Self { entries: Vec::new() } }

    pub fn insert(&mut self, robot_id: RobotId, factor_idx: usize) {
        // Replace if already present, otherwise push
        if let Some(e) = self.entries.iter_mut().find(|(id, _)| *id == robot_id) {
            e.1 = factor_idx;
        } else {
            let _ = self.entries.push((robot_id, factor_idx));
        }
    }

    pub fn factor_idx(&self, robot_id: RobotId) -> Option<usize> {
        self.entries.iter().find(|(id, _)| *id == robot_id).map(|(_, idx)| *idx)
    }

    pub fn contains(&self, robot_id: RobotId) -> bool {
        self.entries.iter().any(|(id, _)| *id == robot_id)
    }

    pub fn iter(&self) -> impl Iterator<Item = &(RobotId, usize)> {
        self.entries.iter()
    }

    /// Remove the factor for robot_id from the graph (swap-remove) and patch indices.
    pub fn remove<const K: usize, const F: usize>(
        &mut self,
        robot_id: RobotId,
        graph: &mut FactorGraph<K, F>,
    ) {
        let pos = match self.entries.iter().position(|(id, _)| *id == robot_id) {
            Some(p) => p,
            None    => return,
        };
        let (_, factor_idx) = self.entries.swap_remove(pos);
        let last_idx = graph.factor_count() - 1;
        graph.remove_factor(factor_idx);

        // If we didn't remove the last factor, the last factor moved to factor_idx.
        // Patch any entry pointing to last_idx.
        if factor_idx != last_idx {
            for (_, stored_idx) in self.entries.iter_mut() {
                if *stored_idx == last_idx {
                    *stored_idx = factor_idx;
                    break;
                }
            }
        }
    }
}
