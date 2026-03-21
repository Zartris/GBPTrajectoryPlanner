//! GBP factor graph with damped message passing.
//!
//! NOTE: The variable-to-factor pass resets variables to priors and re-accumulates
//! all factor messages. This is NOT standard GBP cavity computation — it's a
//! "full marginal" approach equivalent to mean-field updates. For tree-structured
//! graphs this converges identically; for loopy graphs (multiple IR factors active),
//! damping (alpha < 1.0) is applied to factor-to-variable messages to stabilize
//! convergence.

use heapless::Vec;
use crate::factor_node::{Factor, FactorKind, FactorNode};
use crate::variable_node::VariableNode;

/// Damping factor for message updates. 1.0 = no damping, 0.5 = average old/new.
/// Lower values improve convergence stability in loopy graphs at the cost of speed.
const MSG_DAMPING: f32 = 0.5;

/// GBP factor graph with const-generic capacity.
/// K = number of variables (timestep horizon), F = max factors.
pub struct FactorGraph<const K: usize, const F: usize> {
    pub variables: [VariableNode; K],
    factors: Vec<FactorNode, F>,
}

impl<const K: usize, const F: usize> FactorGraph<K, F> {
    /// Create graph with K variables all initialised to (mean, variance).
    pub fn new(init_mean: f32, init_variance: f32) -> Self {
        Self {
            variables: core::array::from_fn(|_| VariableNode::new(init_mean, init_variance)),
            factors: Vec::new(),
        }
    }

    pub fn factor_count(&self) -> usize { self.factors.len() }

    /// Add a factor; returns its slot index.
    pub fn add_factor(&mut self, kind: FactorKind) -> Result<usize, ()> {
        let idx = self.factors.len();
        self.factors.push(FactorNode::new(kind)).map_err(|_| ())?;
        Ok(idx)
    }

    /// O(1) swap-remove. The last factor moves to `idx`.
    /// Caller (InterRobotFactorSet) is responsible for updating stored indices.
    pub fn remove_factor(&mut self, idx: usize) -> FactorKind {
        self.factors.swap_remove(idx).kind
    }

    /// Get immutable access to a factor's FactorKind.
    pub fn get_factor_kind(&self, idx: usize) -> Option<&FactorKind> {
        self.factors.get(idx).map(|f| &f.kind)
    }

    /// Get immutable access to the raw FactorNode (for reading stored messages).
    pub fn factor_node(&self, idx: usize) -> Option<&FactorNode> {
        self.factors.get(idx)
    }

    /// Get mutable access to a factor's FactorKind (for setting v_nom, Jacobians, etc.)
    pub fn get_factor_kind_mut(&mut self, idx: usize) -> Option<&mut FactorKind> {
        self.factors.get_mut(idx).map(|f| &mut f.kind)
    }

    /// Run N iterations of damped GBP message passing.
    pub fn iterate(&mut self, iterations: usize) {
        for _ in 0..iterations {
            self.factor_to_variable_pass();
            self.variable_to_factor_pass();
        }
    }

    /// Run split internal/external GBP message passing (MAGICS architecture).
    ///
    /// Separates factor-to-variable messages into two classes:
    /// - **Internal factors** (Dynamics, VelocityBound): connect only variables owned
    ///   by this robot. These converge quickly and benefit from multiple iterations.
    /// - **External factors** (InterRobot): depend on beliefs broadcast by other robots.
    ///   These carry stale evidence — running them every internal iteration amplifies
    ///   that staleness, destabilising convergence in loopy graphs.
    ///
    /// For each of `n_external` external iterations:
    ///   1. Run `n_internal` rounds of internal-only factor→variable + variable→factor.
    ///   2. Run one round of external-only factor→variable + variable→factor.
    ///
    /// This lets the local chain settle before injecting inter-robot influence,
    /// reducing oscillation. The original gbpplanner (Patwardhan et al.) uses
    /// TI=10 internal iterations per TE=10 external iterations as the default.
    pub fn iterate_split(&mut self, n_internal: usize, n_external: usize) {
        for _ in 0..n_external {
            // Internal convergence rounds
            for _ in 0..n_internal {
                self.factor_to_variable_pass_filtered(true);
                self.variable_to_factor_pass();
            }
            // One external round
            self.factor_to_variable_pass_filtered(false);
            self.variable_to_factor_pass();
        }
    }

    // -- Private --

    fn factor_to_variable_pass_filtered(&mut self, internal_only: bool) {
        for f in self.factors.iter_mut() {
            if !f.kind.as_factor().is_active() { continue; }
            if f.kind.is_internal() != internal_only { continue; }
            f.kind.as_factor_mut().update(&self.variables);
            let var_indices = f.kind.as_factor().variable_indices();

            if var_indices.len() == 1 {
                let lf = f.kind.as_factor().linearize(&self.variables);
                let new_eta = lf.residual;
                let new_lambda = lf.precision;
                f.msg_eta[0]    = MSG_DAMPING * new_eta    + (1.0 - MSG_DAMPING) * f.msg_eta[0];
                f.msg_lambda[0] = MSG_DAMPING * new_lambda + (1.0 - MSG_DAMPING) * f.msg_lambda[0];

            } else if var_indices.len() == 2 {
                let [idx0, idx1] = [var_indices[0], var_indices[1]];
                let lf = f.kind.as_factor().linearize(&self.variables);
                let j0 = lf.jacobian[0];
                let j1 = lf.jacobian[1];
                let prec = lf.precision;
                let r    = lf.residual;

                let x0 = self.variables[idx0].mean();
                let x1 = self.variables[idx1].mean();

                let xi_00 = prec * j0 * j0;
                let xi_11 = prec * j1 * j1;
                let xi_01 = prec * j0 * j1;

                let jx = j0 * x0 + j1 * x1;
                let zeta_0 = prec * j0 * (jx - r);
                let zeta_1 = prec * j1 * (jx - r);

                let eta0_cav    = self.variables[idx0].eta    - f.msg_eta[0];
                let lambda0_cav = self.variables[idx0].lambda - f.msg_lambda[0];
                let eta1_cav    = self.variables[idx1].eta    - f.msg_eta[1];
                let lambda1_cav = self.variables[idx1].lambda - f.msg_lambda[1];

                let denom1 = xi_11 + lambda1_cav;
                if denom1.abs() > 1e-12 {
                    let new_lambda0 = xi_00 - xi_01 * xi_01 / denom1;
                    let new_eta0    = zeta_0 - xi_01 * (zeta_1 + eta1_cav) / denom1;
                    f.msg_lambda[0] = MSG_DAMPING * new_lambda0 + (1.0 - MSG_DAMPING) * f.msg_lambda[0];
                    f.msg_eta[0]    = MSG_DAMPING * new_eta0    + (1.0 - MSG_DAMPING) * f.msg_eta[0];
                }

                let denom0 = xi_00 + lambda0_cav;
                if denom0.abs() > 1e-12 {
                    let new_lambda1 = xi_11 - xi_01 * xi_01 / denom0;
                    let new_eta1    = zeta_1 - xi_01 * (zeta_0 + eta0_cav) / denom0;
                    f.msg_lambda[1] = MSG_DAMPING * new_lambda1 + (1.0 - MSG_DAMPING) * f.msg_lambda[1];
                    f.msg_eta[1]    = MSG_DAMPING * new_eta1    + (1.0 - MSG_DAMPING) * f.msg_eta[1];
                }
            }
        }
    }

    fn factor_to_variable_pass(&mut self) {
        for f in self.factors.iter_mut() {
            if !f.kind.as_factor().is_active() { continue; }
            f.kind.as_factor_mut().update(&self.variables);
            let var_indices = f.kind.as_factor().variable_indices();

            if var_indices.len() == 1 {
                // Unary factor (e.g. InterRobotFactor):
                // linearize() returns pre-computed information-form message
                // (msg_eta, msg_lambda) in the (residual, precision) fields.
                let lf = f.kind.as_factor().linearize(&self.variables);
                let new_eta = lf.residual;
                let new_lambda = lf.precision;
                // Apply damping
                f.msg_eta[0]    = MSG_DAMPING * new_eta    + (1.0 - MSG_DAMPING) * f.msg_eta[0];
                f.msg_lambda[0] = MSG_DAMPING * new_lambda + (1.0 - MSG_DAMPING) * f.msg_lambda[0];

            } else if var_indices.len() == 2 {
                // Pairwise factor: Schur complement marginalization.
                let [idx0, idx1] = [var_indices[0], var_indices[1]];
                let lf = f.kind.as_factor().linearize(&self.variables);
                let j0 = lf.jacobian[0];
                let j1 = lf.jacobian[1];
                let prec = lf.precision;
                let r    = lf.residual;

                // Current linearization point (variable means)
                let x0 = self.variables[idx0].mean();
                let x1 = self.variables[idx1].mean();

                // Factor information matrix: Lambda_f = J^T * prec * J
                let xi_00 = prec * j0 * j0;
                let xi_11 = prec * j1 * j1;
                let xi_01 = prec * j0 * j1;

                // Factor information vector: eta_f = J^T * prec * (J*x - r)
                let jx = j0 * x0 + j1 * x1;
                let zeta_0 = prec * j0 * (jx - r);
                let zeta_1 = prec * j1 * (jx - r);

                // Cavity beliefs (variable total minus this factor's previous message)
                let eta0_cav    = self.variables[idx0].eta    - f.msg_eta[0];
                let lambda0_cav = self.variables[idx0].lambda - f.msg_lambda[0];
                let eta1_cav    = self.variables[idx1].eta    - f.msg_eta[1];
                let lambda1_cav = self.variables[idx1].lambda - f.msg_lambda[1];

                // Message to variable 0 (marginalize out variable 1)
                let denom1 = xi_11 + lambda1_cav;
                if denom1.abs() > 1e-12 {
                    let new_lambda0 = xi_00 - xi_01 * xi_01 / denom1;
                    let new_eta0    = zeta_0 - xi_01 * (zeta_1 + eta1_cav) / denom1;
                    f.msg_lambda[0] = MSG_DAMPING * new_lambda0 + (1.0 - MSG_DAMPING) * f.msg_lambda[0];
                    f.msg_eta[0]    = MSG_DAMPING * new_eta0    + (1.0 - MSG_DAMPING) * f.msg_eta[0];
                }

                // Message to variable 1 (marginalize out variable 0)
                let denom0 = xi_00 + lambda0_cav;
                if denom0.abs() > 1e-12 {
                    let new_lambda1 = xi_11 - xi_01 * xi_01 / denom0;
                    let new_eta1    = zeta_1 - xi_01 * (zeta_0 + eta0_cav) / denom0;
                    f.msg_lambda[1] = MSG_DAMPING * new_lambda1 + (1.0 - MSG_DAMPING) * f.msg_lambda[1];
                    f.msg_eta[1]    = MSG_DAMPING * new_eta1    + (1.0 - MSG_DAMPING) * f.msg_eta[1];
                }
            }
        }
    }

    fn variable_to_factor_pass(&mut self) {
        // Reset all variables to their prior, then accumulate all factor messages.
        // NOTE: This is a "full marginal" approach — each variable's belief is
        // prior + sum(all_factor_messages), not a per-factor cavity. For standard
        // GBP, the variable-to-factor message would be marginal - factor_message
        // (cavity). The pairwise F-to-V pass above correctly uses cavities, so
        // convergence is maintained for tree-structured subgraphs. Message damping
        // stabilises the loopy case.
        for v in self.variables.iter_mut() {
            v.reset_to_prior();
        }
        for f in self.factors.iter() {
            if !f.kind.as_factor().is_active() { continue; }
            for (i, &var_idx) in f.kind.as_factor().variable_indices().iter().enumerate() {
                self.variables[var_idx].eta    += f.msg_eta[i];
                self.variables[var_idx].lambda += f.msg_lambda[i];
            }
        }
    }
}
