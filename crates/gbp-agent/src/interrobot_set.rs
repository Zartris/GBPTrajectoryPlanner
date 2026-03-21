//! Manages inter-robot factors in the FactorGraph.
//! Stores (robot_id, variable_k, factor_idx) triples.
//! Supports multiple factors per robot (one per variable timestep).

use heapless::Vec;
use gbp_comms::RobotId;
use gbp_core::FactorGraph;
use gbp_map::{MAX_HORIZON, MAX_NEIGHBOURS};

/// Maximum IR factors: one per variable per neighbour.
const MAX_IR_ENTRIES: usize = MAX_HORIZON * MAX_NEIGHBOURS;

pub struct InterRobotFactorSet {
    /// (robot_id, variable_k, factor_idx in FactorGraph)
    entries: Vec<(RobotId, usize, usize), MAX_IR_ENTRIES>,
}

impl InterRobotFactorSet {
    pub fn new() -> Self { Self { entries: Vec::new() } }

    pub fn count(&self) -> usize { self.entries.len() }

    /// Count factors for a specific robot.
    pub fn count_for(&self, robot_id: RobotId) -> usize {
        self.entries.iter().filter(|(rid, _, _)| *rid == robot_id).count()
    }

    /// Check if a factor exists for (robot_id, variable_k).
    pub fn contains(&self, robot_id: RobotId, k: usize) -> bool {
        self.entries.iter().any(|(rid, kk, _)| *rid == robot_id && *kk == k)
    }

    /// Check if any factor exists for a robot.
    pub fn contains_robot(&self, robot_id: RobotId) -> bool {
        self.entries.iter().any(|(rid, _, _)| *rid == robot_id)
    }

    /// Insert an IR factor entry. Returns false if the collection is full.
    pub fn insert(&mut self, robot_id: RobotId, k: usize, factor_idx: usize) -> bool {
        self.entries.push((robot_id, k, factor_idx)).is_ok()
    }

    pub fn factor_idx(&self, robot_id: RobotId, k: usize) -> Option<usize> {
        self.entries.iter()
            .find(|(rid, kk, _)| *rid == robot_id && *kk == k)
            .map(|(_, _, idx)| *idx)
    }

    /// Get all factor indices for a robot (for Jacobian updates).
    pub fn factors_for(&self, robot_id: RobotId) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.entries.iter()
            .filter(move |(rid, _, _)| *rid == robot_id)
            .map(|(_, k, idx)| (*k, *idx))
    }

    pub fn iter(&self) -> impl Iterator<Item = &(RobotId, usize, usize)> {
        self.entries.iter()
    }

    /// Remove a single factor for (robot_id, k). Handles swap-remove index patching.
    pub fn remove_single<const K: usize, const F: usize>(
        &mut self,
        robot_id: RobotId,
        k: usize,
        graph: &mut FactorGraph<K, F>,
    ) {
        let pos = match self.entries.iter().position(|(rid, kk, _)| *rid == robot_id && *kk == k) {
            Some(p) => p,
            None => return,
        };
        if graph.factor_count() == 0 { self.entries.clear(); return; }
        let (_, _, factor_idx) = self.entries.swap_remove(pos);
        let last_idx = graph.factor_count() - 1;
        graph.remove_factor(factor_idx);

        if factor_idx != last_idx {
            for (_, _, stored_idx) in self.entries.iter_mut() {
                if *stored_idx == last_idx {
                    *stored_idx = factor_idx;
                    break;
                }
            }
        }
    }

    /// Remove ALL factors for a specific robot. Handles swap-remove index patching.
    pub fn remove_robot<const K: usize, const F: usize>(
        &mut self,
        robot_id: RobotId,
        graph: &mut FactorGraph<K, F>,
    ) {
        loop {
            let pos = match self.entries.iter().position(|(rid, _, _)| *rid == robot_id) {
                Some(p) => p,
                None => break,
            };
            if graph.factor_count() == 0 { self.entries.clear(); return; }
            let (_, _, factor_idx) = self.entries.swap_remove(pos);
            let last_idx = graph.factor_count() - 1;
            graph.remove_factor(factor_idx);

            // Patch any entry pointing to the swapped-in factor
            if factor_idx != last_idx {
                for (_, _, stored_idx) in self.entries.iter_mut() {
                    if *stored_idx == last_idx {
                        *stored_idx = factor_idx;
                        break;
                    }
                }
            }
        }
    }

    /// Remove ALL inter-robot factors from the graph.
    pub fn remove_all<const K: usize, const F: usize>(
        &mut self,
        graph: &mut FactorGraph<K, F>,
    ) {
        while let Some((_, _, factor_idx)) = self.entries.pop() {
            if graph.factor_count() == 0 { self.entries.clear(); return; }
            let last_idx = graph.factor_count() - 1;
            graph.remove_factor(factor_idx);

            if factor_idx != last_idx {
                for (_, _, stored_idx) in self.entries.iter_mut() {
                    if *stored_idx == last_idx {
                        *stored_idx = factor_idx;
                        break;
                    }
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
        set.insert(5, 3, 10);
        assert!(set.contains(5, 3));
        assert!(!set.contains(5, 4));
        assert_eq!(set.factor_idx(5, 3), Some(10));
    }

    #[test]
    fn count_for_robot() {
        let mut set = InterRobotFactorSet::new();
        set.insert(1, 0, 10);
        set.insert(1, 1, 11);
        set.insert(1, 2, 12);
        set.insert(2, 0, 13);
        assert_eq!(set.count_for(1), 3);
        assert_eq!(set.count_for(2), 1);
        assert_eq!(set.count(), 4);
    }

    #[test]
    fn contains_robot() {
        let mut set = InterRobotFactorSet::new();
        assert!(!set.contains_robot(1));
        set.insert(1, 0, 10);
        assert!(set.contains_robot(1));
    }
}
