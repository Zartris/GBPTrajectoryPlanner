use gbp_agent::interrobot_set::InterRobotFactorSet;
use gbp_core::{FactorGraph, factor_node::FactorKind, dynamics_factor::DynamicsFactor};

type G = FactorGraph<4, 16>;

fn make_graph() -> G { FactorGraph::new(0.0, 1.0) }

#[test]
fn insert_and_lookup() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();
    let idx = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    set.insert(42u32, idx);
    assert_eq!(set.factor_idx(42), Some(idx));
    assert_eq!(set.factor_idx(99), None);
}

#[test]
fn remove_updates_index_for_swapped_entry() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();

    let ia = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let ib = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    let _ic = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([2,3], 0.1, 0.1, 1.0))).unwrap();
    set.insert(10, ia);
    set.insert(20, ib);
    set.insert(30, _ic);

    // Remove robot A (index 0). Swap-remove moves robot C (index 2) to index 0.
    set.remove(10, &mut g);
    assert_eq!(g.factor_count(), 2);
    assert!(set.factor_idx(10).is_none());
    assert_eq!(set.factor_idx(30), Some(0), "C index should have been patched to 0");
    assert_eq!(set.factor_idx(20), Some(1));
}

#[test]
fn remove_last_does_not_need_patching() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();
    let ia = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let ib = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    set.insert(10, ia);
    set.insert(20, ib);
    set.remove(20, &mut g);
    assert_eq!(g.factor_count(), 1);
    assert!(set.factor_idx(20).is_none());
    assert_eq!(set.factor_idx(10), Some(0));
}
