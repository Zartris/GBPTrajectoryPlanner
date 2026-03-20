//! Manages inter-robot factors in the FactorGraph.
//! Each tick, all IR factors are removed and re-added based on current predictions.
//! This avoids complex index tracking for variable-timestep factor connections.

use heapless::Vec;
use gbp_comms::RobotId;
use gbp_core::FactorGraph;
use gbp_map::MAX_NEIGHBOURS;

/// Maximum IR factors active at once (across all neighbours and timesteps).
/// With K=12 and ~2 neighbours, 8 factors is sufficient.
pub const MAX_IR_FACTORS: usize = MAX_NEIGHBOURS;

pub struct InterRobotFactorSet {
    /// (robot_id, factor_idx in FactorGraph)
    entries: Vec<(RobotId, usize), MAX_IR_FACTORS>,
}

impl InterRobotFactorSet {
    pub fn new() -> Self { Self { entries: Vec::new() } }

    pub fn count(&self) -> usize { self.entries.len() }

    pub fn insert(&mut self, robot_id: RobotId, factor_idx: usize) {
        let _ = self.entries.push((robot_id, factor_idx));
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

    /// Remove ALL inter-robot factors from the graph.
    /// Called at the start of each tick before re-adding based on current predictions.
    pub fn remove_all<const K: usize, const F: usize>(
        &mut self,
        graph: &mut FactorGraph<K, F>,
    ) {
        // Remove in reverse order so swap-remove doesn't invalidate earlier indices
        while let Some((_, factor_idx)) = self.entries.pop() {
            let last_idx = graph.factor_count() - 1;
            graph.remove_factor(factor_idx);

            // Patch any remaining entry that pointed to last_idx
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

    /// Remove factors for a specific robot. Handles swap-remove index patching.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_and_lookup() {
        let mut set = InterRobotFactorSet::new();
        set.insert(5, 10);
        assert!(set.contains(5));
        assert_eq!(set.factor_idx(5), Some(10));
    }

    #[test]
    fn count_tracks_entries() {
        let mut set = InterRobotFactorSet::new();
        assert_eq!(set.count(), 0);
        set.insert(1, 0);
        set.insert(2, 1);
        assert_eq!(set.count(), 2);
    }
}
