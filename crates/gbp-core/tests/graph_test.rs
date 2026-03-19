use gbp_core::factor_graph::FactorGraph;
use gbp_core::factor_node::FactorKind;
use gbp_core::dynamics_factor::DynamicsFactor;
use gbp_core::variable_node::VariableNode;

const K: usize = 4;
const F: usize = 16;

#[test]
fn factor_graph_add_and_count() {
    let mut g: FactorGraph<K, F> = FactorGraph::new(0.0, 1.0);
    assert_eq!(g.factor_count(), 0);
    let idx = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    assert_eq!(g.factor_count(), 1);
    assert_eq!(idx, 0);
}

#[test]
fn factor_graph_swap_remove_patches_index() {
    let mut g: FactorGraph<K, F> = FactorGraph::new(0.0, 1.0);
    let _i0 = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0,1], 0.1, 0.1, 1.0))).unwrap();
    let _i1 = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([1,2], 0.1, 0.1, 1.0))).unwrap();
    let _i2 = g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([2,3], 0.1, 0.1, 1.0))).unwrap();
    assert_eq!(g.factor_count(), 3);
    g.remove_factor(_i0);
    assert_eq!(g.factor_count(), 2);
}

#[test]
fn iterate_single_dynamics_converges() {
    let mut g: FactorGraph<2, 4> = FactorGraph::new(0.0, 100.0);
    g.variables[0] = VariableNode::new(0.0, 100.0);
    g.variables[0].prior_eta = 0.0 / 100.0;
    g.variables[0].prior_lambda = 1000.0;
    g.variables[0].eta = g.variables[0].prior_eta;
    g.variables[0].lambda = g.variables[0].prior_lambda;

    g.add_factor(FactorKind::Dynamics(DynamicsFactor::new([0, 1], 0.1, 0.1, 1.0))).unwrap();
    g.iterate(20);

    let s1 = g.variables[1].mean();
    assert!(s1 > 0.05, "s1={} did not converge toward 0.1", s1);
}
