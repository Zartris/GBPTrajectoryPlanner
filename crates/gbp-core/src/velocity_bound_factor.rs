//! Velocity bound factor using adaptive precision inspired by the Barrier
//! Interior Point Method (BIPM).
//!
//! # Background
//!
//! In constrained optimization, barrier methods (also called interior point
//! methods) enforce inequality constraints by adding a penalty term that grows
//! toward infinity as the solution approaches the constraint boundary. The
//! classic log-barrier replaces a hard constraint `g(x) <= 0` with a soft
//! penalty `-mu * ln(-g(x))`, where `mu` controls how sharply the barrier
//! activates.
//!
//! Patwardhan et al. (2025) adapt this idea for Gaussian Belief Propagation in
//! "Gaussian Belief Propagation for Constrained Optimization with Barrier
//! Interior Point Method" (arXiv: <https://arxiv.org/abs/2506.14341>). Their
//! key insight: rather than adding a nonlinear log-barrier term (which breaks
//! the Gaussian message-passing structure), one can implement the barrier
//! effect by making the factor's *precision* a function of the constraint
//! violation. Far from the boundary the factor is essentially inactive
//! (precision ~ 0); near the boundary the precision ramps up, pushing the
//! belief away from the infeasible region.
//!
//! # Our simplified approach
//!
//! We apply this concept to velocity bounding. The constraint is:
//!
//! ```text
//!   g = (s_{k+1} - s_k) / dt  -  v_max  <=  0
//! ```
//!
//! where `g` is the constraint violation (positive means over the limit).
//!
//! The adaptive precision schedule is:
//!
//! - **Well under limit** (`g < -margin`): precision = 0 (factor inactive,
//!   robot is safely below `v_max`).
//! - **Barrier zone** (`-margin < g < -epsilon`): precision =
//!   `1 / (kappa * g^2)`, capped at 1000. As `g -> 0` (velocity approaches
//!   `v_max`), precision rises steeply, creating the barrier effect.
//! - **At or over limit** (`g >= -epsilon`): precision = 1000 (hard clamp,
//!   maximum corrective force).
//!
//! The Jacobian is identical to the dynamics factor: `[-1/dt, +1/dt]`, and the
//! residual is `g` itself, so the factor's information contribution pushes
//! the predicted velocity back below `v_max`.
//!
//! # Parameters
//!
//! - `kappa` (default 10.0): controls barrier steepness. Higher values make the
//!   barrier activate later but more sharply.
//! - `margin` (default 1.0 m/s): the distance below `v_max` at which the
//!   barrier begins to activate.

use heapless::Vec;
use crate::factor_node::{Factor, LinearizedFactor};
use crate::variable_node::VariableNode;

/// Pairwise velocity bound factor connecting `(s_k, s_{k+1})`.
///
/// Enforces `v_min <= velocity <= v_max` using adaptive precision.
/// The upper bound prevents GBP from predicting impossibly fast speeds.
/// The lower bound prevents the belief chain from flipping backwards
/// (which happens when IR factors overwhelm dynamics), while still
/// allowing a small creep-back speed for recovering from merge overshoots.
pub struct VelocityBoundFactor {
    var_indices: [usize; 2],
    dt: f32,
    v_max: f32,
    /// Minimum allowed velocity. Slightly negative to allow creep-back.
    v_min: f32,
    /// Barrier steepness parameter. Higher = sharper activation.
    kappa: f32,
    /// Distance from limit (in m/s) at which the barrier begins to activate.
    margin: f32,
}

impl VelocityBoundFactor {
    /// Create a new velocity bound factor.
    ///
    /// - `var_indices`: indices of `[s_k, s_{k+1}]` in the factor graph.
    /// - `dt`: timestep between the two variables.
    /// - `v_max`: maximum allowed forward velocity.
    pub fn new(var_indices: [usize; 2], dt: f32, v_max: f32) -> Self {
        Self {
            var_indices,
            dt: f32::max(dt, 1e-6),
            v_max,
            v_min: -0.3, // allow slow creep-back (0.3 m/s reverse)
            kappa: 10.0,
            margin: 1.0,
        }
    }

    /// Update the velocity limit (e.g. when approaching a curve).
    pub fn set_v_max(&mut self, v: f32) {
        self.v_max = v;
    }

    /// BIPM-inspired adaptive precision for a single constraint g <= 0.
    /// Returns 0 when well under limit, rising to MAX_PREC at the boundary.
    fn barrier_precision(g: f32, margin: f32, kappa: f32) -> f32 {
        const MAX_PREC: f32 = 100.0;
        if g < -margin {
            0.0
        } else if g < -1e-4 {
            let raw = 1.0 / (kappa * g * g);
            if raw < MAX_PREC { raw } else { MAX_PREC }
        } else {
            MAX_PREC
        }
    }
}

impl Factor for VelocityBoundFactor {
    fn variable_indices(&self) -> &[usize] {
        &self.var_indices
    }

    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor {
        let s_k  = variables[self.var_indices[0]].mean();
        let s_k1 = variables[self.var_indices[1]].mean();

        let velocity = (s_k1 - s_k) / self.dt;
        let inv_dt = 1.0 / self.dt;

        // Check both bounds — whichever is violated (or closer to violation) wins.
        // Upper bound: g_upper = velocity - v_max  (positive = over max)
        // Lower bound: g_lower = v_min - velocity  (positive = under min)
        let g_upper = velocity - self.v_max;
        let g_lower = self.v_min - velocity;

        // Compute barrier precision for each bound independently
        let prec_upper = Self::barrier_precision(g_upper, self.margin, self.kappa);
        let prec_lower = Self::barrier_precision(g_lower, self.margin, self.kappa);

        // The active constraint is whichever has higher precision (closer to violation).
        // Use its residual and precision. If neither is active, factor is dormant.
        let (residual, precision) = if prec_upper >= prec_lower {
            // Upper bound dominates: residual = g_upper, Jacobian = d(velocity)/d(s)
            (g_upper, prec_upper)
        } else {
            // Lower bound dominates: residual = g_lower, Jacobian = d(-velocity)/d(s) = -d(velocity)/d(s)
            // We negate the Jacobian sign by negating the residual direction:
            // g_lower = v_min - velocity, so d(g_lower)/d(s_k) = +1/dt, d(g_lower)/d(s_k1) = -1/dt
            // But our Jacobian is fixed as [-1/dt, +1/dt] (for velocity).
            // To make the factor push velocity UP (away from v_min), we use -g_lower as residual
            // with the standard Jacobian. This works because:
            //   eta contribution = precision * (-g_lower) * jacobian
            //   = precision * (velocity - v_min) * [-1/dt, +1/dt]
            //   When velocity < v_min: (velocity - v_min) < 0, pushes s_k1 down less / s_k up = increase velocity
            (-g_lower, prec_lower)
        };

        let mut jacobian: Vec<f32, 2> = Vec::new();
        let _ = jacobian.push(-inv_dt);
        let _ = jacobian.push( inv_dt);

        LinearizedFactor {
            jacobian,
            residual,
            precision,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::variable_node::VariableNode;
    use crate::factor_graph::FactorGraph;
    use crate::factor_node::FactorKind;
    use crate::dynamics_factor::DynamicsFactor;

    /// Helper: create two variables representing s_k and s_{k+1} with the
    /// given means (low variance = tight beliefs).
    fn make_vars(s_k: f32, s_k1: f32) -> [VariableNode; 2] {
        [VariableNode::new(s_k, 0.01), VariableNode::new(s_k1, 0.01)]
    }

    #[test]
    fn zero_precision_when_well_under_limit() {
        // velocity = (1.1 - 1.0) / 0.1 = 1.0 m/s, v_max = 2.5
        // g = 1.0 - 2.5 = -1.5, which is < -margin(1.0) => precision = 0
        let vars = make_vars(1.0, 1.1);
        let factor = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        let lf = factor.linearize(&vars);
        assert_eq!(lf.precision, 0.0);
    }

    #[test]
    fn increasing_precision_near_limit() {
        let dt = 0.1;
        let v_max = 2.5;

        // velocity = 2.2 m/s => g = -0.3 (in barrier zone)
        let vars_far = make_vars(0.0, 0.22);
        let f_far = VelocityBoundFactor::new([0, 1], dt, v_max);
        let lf_far = f_far.linearize(&vars_far);

        // velocity = 2.4 m/s => g = -0.1 (closer to limit)
        let vars_close = make_vars(0.0, 0.24);
        let f_close = VelocityBoundFactor::new([0, 1], dt, v_max);
        let lf_close = f_close.linearize(&vars_close);

        assert!(lf_far.precision > 0.0, "should be in barrier zone");
        assert!(lf_close.precision > 0.0, "should be in barrier zone");
        assert!(
            lf_close.precision > lf_far.precision,
            "closer to limit should have higher precision: {} vs {}",
            lf_close.precision, lf_far.precision
        );
    }

    #[test]
    fn max_precision_when_over_limit() {
        // velocity = (0.3 - 0.0) / 0.1 = 3.0 m/s, v_max = 2.5
        // g = 0.5 >= -1e-4 => precision = 1000
        let vars = make_vars(0.0, 0.3);
        let factor = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        let lf = factor.linearize(&vars);
        assert_eq!(lf.precision, 100.0);
    }

    #[test]
    fn jacobian_signs_correct() {
        let vars = make_vars(0.0, 0.2);
        let factor = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        let lf = factor.linearize(&vars);
        assert!(lf.jacobian[0] < 0.0, "d(velocity)/d(s_k) should be negative");
        assert!(lf.jacobian[1] > 0.0, "d(velocity)/d(s_{{k+1}}) should be positive");
    }

    #[test]
    fn active_when_going_backwards() {
        // velocity = (0.95 - 1.0) / 0.1 = -0.5 m/s, below v_min=-0.3
        let vars = make_vars(1.0, 0.95);
        let factor = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        let lf = factor.linearize(&vars);
        assert!(lf.precision > 0.0, "should activate when going backwards past v_min");
    }

    #[test]
    fn inactive_when_creeping_back_slowly() {
        // velocity = (0.98 - 1.0) / 0.1 = -0.2 m/s, above v_min=-0.3 (within allowed creep)
        // g_lower = -0.3 - (-0.2) = -0.1, g_upper = -0.2 - 2.5 = -2.7
        // g_lower = -0.1 is within margin(1.0) so barrier activates slightly
        // but it should be low precision since we're still within bounds
        let vars = make_vars(1.0, 0.98);
        let factor = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        let lf = factor.linearize(&vars);
        // g_lower = v_min - velocity = -0.3 - (-0.2) = -0.1
        // This IS in the barrier zone but not violated — precision should be moderate
        assert!(lf.precision < 100.0, "should not be at max when within v_min");
    }

    #[test]
    fn gbp_reduces_velocity_with_bound_factor() {
        // 4 variables, K=4, F=8: dynamics factors want v_nom=5.0,
        // velocity bound factors cap at v_max=2.5.
        let dt = 0.1;
        let mut graph: FactorGraph<4, 8> = FactorGraph::new(0.0, 10.0);

        // Add dynamics factors wanting high velocity
        for i in 0..3 {
            let dyn_f = DynamicsFactor::new([i, i + 1], dt, 0.5, 5.0);
            let _ = graph.add_factor(FactorKind::Dynamics(dyn_f));
        }

        // Add velocity bound factors
        for i in 0..3 {
            let vb_f = VelocityBoundFactor::new([i, i + 1], dt, 2.5);
            let _ = graph.add_factor(FactorKind::VelocityBound(vb_f));
        }

        graph.iterate(30);

        // Check that actual velocities are bounded (allowing some slack for
        // GBP convergence — the hard constraint is 2.5, we check < 3.5).
        for i in 0..3 {
            let s_k  = graph.variables[i].mean();
            let s_k1 = graph.variables[i + 1].mean();
            let vel = (s_k1 - s_k) / dt;
            assert!(
                vel < 3.5,
                "velocity between var {} and {} should be bounded: got {}",
                i, i + 1, vel
            );
        }
    }
}
