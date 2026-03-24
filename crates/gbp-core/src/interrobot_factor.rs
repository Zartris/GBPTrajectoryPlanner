//! Inter-robot collision avoidance factor (exponential decay penalty).
//!
//! Connects only var_idx_a (in this robot's graph).
//! Robot B's belief is injected externally as (ext_eta_b, ext_lambda_b).
//!
//! Uses exponential decay precision — smooth everywhere, no discontinuities:
//!   - `dist <= d_safe`: full precision (collision zone, maximum repulsion)
//!   - `dist > d_safe`: precision decays as `exp(-alpha * (dist - d_safe))`
//! The smooth decay eliminates GBP oscillation caused by hard cutoffs while
//! still providing early warning. At 2× d_safe, precision is ~2% of full.
//!
//! NOTE on `linearize()` return values: Because this is a unary factor (after Schur
//! complement marginalization over robot B), `linearize()` returns the pre-computed
//! information-form message (msg_eta, msg_lambda) in the `residual` and `precision`
//! fields of `LinearizedFactor`. These are NOT a raw residual and precision — they
//! are the final factor-to-variable message. The `jacobian` field is unused for
//! unary factors in `factor_graph.rs`.

use heapless::Vec;
use crate::factor_node::{Factor, LinearizedFactor};
use crate::variable_node::VariableNode;

pub struct InterRobotFactor {
    var_idx_a:   usize,
    d_safe:      f32,
    sigma_r:     f32,
    pub decay_alpha: f32,
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
    pub fn new(var_idx_a: usize, d_safe: f32, sigma_r: f32, decay_alpha: f32) -> Self {
        Self {
            var_idx_a, d_safe, sigma_r, decay_alpha,
            active: true,
            jacobian_a: 0.0, jacobian_b: 0.0,
            ext_eta_b: 0.0, ext_lambda_b: 1.0,
            dist: f32::MAX,
        }
    }

    pub fn set_active(&mut self, active: bool) { self.active = active; }
    pub fn set_d_safe(&mut self, d_safe: f32) { self.d_safe = d_safe; }
    pub fn set_sigma(&mut self, sigma: f32) { self.sigma_r = f32::max(sigma, 1e-6); }
    pub fn set_decay_alpha(&mut self, alpha: f32) { self.decay_alpha = alpha; }

    /// Update which variable this factor constrains (e.g. the timestep
    /// where this robot is predicted to be closest to the neighbour).
    pub fn set_variable_index(&mut self, idx: usize) { self.var_idx_a = idx; }
}

impl Factor for InterRobotFactor {
    fn variable_indices(&self) -> &[usize] { core::slice::from_ref(&self.var_idx_a) }
    fn is_active(&self) -> bool { self.active }

    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor {
        // Exponential decay precision — smooth everywhere, no discontinuities.
        //
        //   dist <= d_safe  → full precision (collision zone)
        //   dist > d_safe   → precision decays as exp(-alpha * (dist - d_safe))
        //
        // alpha controls the decay rate. Higher alpha = faster decay = less influence at distance.
        // With alpha=3.0 and d_safe=1.3: at 2*d_safe (2.6m) precision is ~2% of full.
        let decay_alpha = self.decay_alpha;

        // Residual: dist - d_safe (positive when safe, negative when too close).
        // Sign convention matches pairwise factors where r = h(x) - z, so the
        // factor_graph formula jx - r = jx - (dist - d_safe) = jx - dist + d_safe
        // pushes variables apart when dist < d_safe.
        let residual = self.dist - self.d_safe;
        let sigma_r = f32::max(self.sigma_r, 1e-6);
        let full_prec = 1.0 / (sigma_r * sigma_r);

        let prec = if self.dist <= self.d_safe {
            full_prec
        } else {
            let decay = libm::expf(-decay_alpha * (self.dist - self.d_safe));
            full_prec * decay
        };

        // Linearization point: variable A from graph, variable B from external belief
        let x_a = variables[self.var_idx_a].mean();
        let x_b = if self.ext_lambda_b > 1e-10 {
            self.ext_eta_b / self.ext_lambda_b
        } else {
            0.0
        };

        // Joint information matrix (2x2, pairwise between A and B)
        let xi_aa = prec * self.jacobian_a * self.jacobian_a;
        let xi_ab = prec * self.jacobian_a * self.jacobian_b;
        let xi_bb = prec * self.jacobian_b * self.jacobian_b;

        // Information vector: η = J^T * Λ * (J*x - r)
        // This includes the linearization point J*x, matching the pairwise formula
        // in factor_graph.rs. Without this, the factor's push has wrong magnitude.
        let jx = self.jacobian_a * x_a + self.jacobian_b * x_b;
        let zeta_a = prec * self.jacobian_a * (jx - residual);
        let zeta_b = prec * self.jacobian_b * (jx - residual);

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
