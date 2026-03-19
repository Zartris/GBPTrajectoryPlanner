//! Inter-robot collision avoidance factor.
//!
//! Connects only var_idx_a (in this robot's graph).
//! Robot B's belief is injected externally as (ext_eta_b, ext_lambda_b).

use heapless::Vec;
use crate::factor_node::{Factor, LinearizedFactor};
use crate::variable_node::VariableNode;

pub struct InterRobotFactor {
    var_idx_a:   usize,
    d_safe:      f32,
    sigma_r:     f32,
    active:      bool,
    /// Scalar Jacobians (set by agent each step)
    pub jacobian_a: f32,
    pub jacobian_b: f32,
    /// External belief of robot B at this timestep (set by agent from RobotBroadcast)
    pub ext_eta_b:    f32,
    pub ext_lambda_b: f32,
    /// Distance between robots at current beliefs (set by agent)
    pub dist: f32,
}

impl InterRobotFactor {
    pub fn new(var_idx_a: usize, d_safe: f32, sigma_r: f32) -> Self {
        Self {
            var_idx_a, d_safe, sigma_r,
            active: true,
            jacobian_a: 0.0, jacobian_b: 0.0,
            ext_eta_b: 0.0, ext_lambda_b: 1.0,
            dist: f32::MAX,
        }
    }

    pub fn set_active(&mut self, active: bool) { self.active = active; }
}

impl Factor for InterRobotFactor {
    fn variable_indices(&self) -> &[usize] { core::slice::from_ref(&self.var_idx_a) }
    fn is_active(&self) -> bool { self.active }

    fn linearize(&self, _variables: &[VariableNode]) -> LinearizedFactor {
        let residual = self.d_safe - self.dist;
        let prec = 1.0 / (self.sigma_r * self.sigma_r);

        // Joint information matrix (2x2, pairwise between A and B)
        let xi_aa = prec * self.jacobian_a * self.jacobian_a;
        let xi_ab = prec * self.jacobian_a * self.jacobian_b;
        let xi_bb = prec * self.jacobian_b * self.jacobian_b;
        let zeta_a = prec * residual * self.jacobian_a;
        let zeta_b = prec * residual * self.jacobian_b;

        // Marginalize out B using Schur complement with B's external cavity belief
        let lambda_b_eff = xi_bb + self.ext_lambda_b;
        let (msg_lambda, msg_eta) = if lambda_b_eff.abs() > 1e-10 {
            let lam = xi_aa - (xi_ab * xi_ab) / lambda_b_eff;
            let eta = zeta_a - xi_ab * (zeta_b + self.ext_eta_b) / lambda_b_eff;
            (lam, eta)
        } else {
            (xi_aa, zeta_a)
        };

        let mut jacobian: Vec<f32, 2> = Vec::new();
        let _ = jacobian.push(self.jacobian_a);
        LinearizedFactor {
            jacobian,
            residual: msg_eta,
            precision: msg_lambda,
        }
    }
}
