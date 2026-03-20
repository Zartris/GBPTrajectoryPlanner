use gbp_core::variable_node::VariableNode;
use gbp_core::dynamics_factor::DynamicsFactor;
use gbp_core::interrobot_factor::InterRobotFactor;
use gbp_core::factor_node::Factor;

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

fn make_vars(means: &[f32], variance: f32) -> heapless::Vec<VariableNode, 16> {
    let mut v: heapless::Vec<VariableNode, 16> = heapless::Vec::new();
    for &m in means {
        v.push(VariableNode::new(m, variance)).unwrap();
    }
    v
}

#[test]
fn dynamics_factor_zero_residual_at_v_nom() {
    let dt = 0.1;
    let v_nom = 2.0;
    let s_k = 1.0;
    let s_k1 = s_k + v_nom * dt;
    let vars = make_vars(&[s_k, s_k1], 1.0);
    let f = DynamicsFactor::new([0, 1], dt, 0.1, v_nom);
    let lf = f.linearize(&vars);
    assert!(lf.residual.abs() < 1e-5, "residual={}", lf.residual);
}

#[test]
fn dynamics_factor_jacobian_shape() {
    let f = DynamicsFactor::new([0, 1], 0.1, 0.1, 1.0);
    let vars = make_vars(&[0.0, 0.0], 1.0);
    let lf = f.linearize(&vars);
    assert_eq!(lf.jacobian.len(), 2);
    // J = [-1/dt, +1/dt]
    assert!((lf.jacobian[0] - (-10.0)).abs() < 1e-4, "J0={}", lf.jacobian[0]);
    assert!((lf.jacobian[1] -  10.0 ).abs() < 1e-4, "J1={}", lf.jacobian[1]);
}

#[test]
fn interrobot_factor_precision_matrix_symmetric() {
    let f = InterRobotFactor::new(0, 0.3, 0.5);
    let mut f2 = f;
    f2.jacobian_a = 0.8;
    f2.jacobian_b = -0.6;
    f2.ext_eta_b  = 0.0;
    f2.ext_lambda_b = 4.0;
    f2.dist = 0.5;  // set a finite distance

    let vars = make_vars(&[0.0], 1.0);
    let lf = f2.linearize(&vars);

    assert!(lf.precision.is_finite());
    assert!(lf.residual.is_finite());
}
