use gbp_core::variable_node::VariableNode;

#[test]
fn variable_node_mean_and_variance() {
    let v = VariableNode { eta: 4.0, lambda: 2.0, prior_eta: 0.0, prior_lambda: 0.01 };
    assert!((v.mean() - 2.0).abs() < 1e-6);
    assert!((v.variance() - 0.5).abs() < 1e-6);
}

#[test]
fn variable_node_zero_lambda_is_safe() {
    let v = VariableNode::default();
    assert_eq!(v.mean(), 0.0);
    assert!(v.variance() > 1e10); // "infinite" uncertainty
}
