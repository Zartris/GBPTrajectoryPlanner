use gbp_agent::interrobot_set::InterRobotFactorSet;
use gbp_core::{FactorGraph, factor_node::FactorKind, dynamics_factor::DynamicsFactor};

type G = FactorGraph<4, 16>;

fn make_graph() -> G { FactorGraph::new(0.0, 1.0, 0.5) }

#[test]
fn insert_and_lookup() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();
    let idx = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    assert!(set.insert(42u32, 1, idx));
    assert_eq!(set.factor_idx(42, 1), Some(idx));
    assert_eq!(set.factor_idx(42, 2), None);
    assert_eq!(set.factor_idx(99, 1), None);
}

#[test]
fn remove_robot_updates_index_for_swapped_entry() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();

    let ia = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let ib = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    let ic = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([2,3], 0.1, 0.1, 1.0))).unwrap();
    assert!(set.insert(10, 1, ia));
    assert!(set.insert(20, 1, ib));
    assert!(set.insert(30, 1, ic));

    set.remove_robot(10, &mut g);
    assert_eq!(g.factor_count(), 2);
    assert!(!set.contains_robot(10));
    assert!(set.factor_idx(30, 1).is_some());
    assert!(set.factor_idx(20, 1).is_some());
}

#[test]
fn remove_last_does_not_need_patching() {
    let mut set = InterRobotFactorSet::new();
    let mut g = make_graph();
    let ia = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let ib = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    assert!(set.insert(10, 1, ia));
    assert!(set.insert(20, 1, ib));
    set.remove_robot(20, &mut g);
    assert_eq!(g.factor_count(), 1);
    assert!(!set.contains_robot(20));
    assert_eq!(set.factor_idx(10, 1), Some(0));
}

#[test]
fn contains_and_count() {
    let mut set = InterRobotFactorSet::new();
    assert!(!set.contains(5, 1));
    assert!(set.insert(5, 1, 10));
    assert!(set.insert(5, 2, 11));
    assert!(set.insert(7, 1, 12));
    assert!(set.contains(5, 1));
    assert!(set.contains(5, 2));
    assert!(!set.contains(5, 3));
    assert_eq!(set.count_for(5), 2);
    assert_eq!(set.count_for(7), 1);
    assert_eq!(set.count(), 3);
}
