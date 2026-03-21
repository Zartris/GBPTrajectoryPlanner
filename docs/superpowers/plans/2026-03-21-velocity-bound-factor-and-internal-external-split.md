# Velocity Bound Factor + Internal/External Iteration Split

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent GBP variables from predicting physically impossible velocities, and separate internal/external message passing so inter-robot factors don't overwhelm dynamics.

**Architecture:** Two changes: (1) A new `VelocityBoundFactor` — a pairwise factor on `(s_k, s_{k+1})` that uses an adaptive-precision penalty inspired by the Barrier Interior Point Method (BIPM) to create a "soft wall" at `v_max`. (2) Split `FactorGraph::iterate()` into internal-only and external-only passes so dynamics factors get multiple iterations per one IR factor update, matching the MAGICS architecture of `TI` internal / `TE` external iterations.

**Tech Stack:** Rust, `#![no_std]`, `heapless`, `libm` — all in `gbp-core` and `gbp-agent` crates.

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `crates/gbp-core/src/velocity_bound_factor.rs` | The new barrier-inspired velocity bounding factor |
| Modify | `crates/gbp-core/src/factor_node.rs` | Add `VelocityBound` variant to `FactorKind` enum (FACTOR EXTENSION POINT 1+2) |
| Modify | `crates/gbp-core/src/lib.rs` | Export new module (FACTOR EXTENSION POINT 3) |
| Modify | `crates/gbp-core/src/factor_graph.rs` | Split iterate into internal/external passes |
| Modify | `crates/gbp-agent/src/robot_agent.rs` | Spawn velocity bound factors, use new iterate API, add internal/external constants |
| Test | `crates/gbp-core/src/velocity_bound_factor.rs` (inline) | Unit tests for the factor |

---

## Task 1: Create VelocityBoundFactor

**Files:**
- Create: `crates/gbp-core/src/velocity_bound_factor.rs`

- [ ] **Step 1: Write the failing tests**

Add `velocity_bound_factor.rs` with tests only (the struct and impl will be empty/todo):

```rust
// crates/gbp-core/src/velocity_bound_factor.rs

//! Velocity bound factor — prevents GBP from predicting physically impossible speeds.
//!
//! # Background: Barrier Interior Point Method (BIPM)
//!
//! Standard GBP can only encode Gaussian (quadratic) penalties. It has no way to express
//! hard inequality constraints like "velocity <= v_max". The dynamics factor says "try to
//! go at v_nom" with precision 4, but the IR factor says "stay apart" with precision 44.
//! When these compete, GBP produces velocities of 10+ m/s — physically impossible but
//! mathematically valid in the unconstrained Gaussian world.
//!
//! The full BIPM (see: "Barrier Method for Inequality Constrained Factor Graph
//! Optimization", arxiv.org/abs/2506.14341) solves this by replacing hard inequality
//! constraints g(X) <= 0 with a logarithmic barrier:
//!
//!     cost = -(1/kappa) * ln(-g(X))     when g(X) < 0  (feasible)
//!     cost = infinity                     when g(X) >= 0 (infeasible)
//!
//! This integrates into a least-squares solver via an "inequality factor node" with
//! adaptive precision: Omega = kappa^-1 * g(X)^-2. As the variable approaches the
//! constraint boundary, precision grows without bound — creating a "wall".
//!
//! The full BIPM requires nested outer/inner optimization loops with kappa-scheduling
//! (kappa_0=0.5 -> kappa_final=1500, factor nu=8) and backtracking line search to
//! maintain feasibility. This is incompatible with our fixed-iteration GBP solver.
//!
//! # Our simplified approach
//!
//! We use a **fixed-kappa barrier factor** that captures the essential "wall" behavior
//! without the solver restructure:
//!
//! - Constraint: g = (s_{k+1} - s_k)/dt - v_max
//! - When g < -margin (well under limit): factor is inactive (zero contribution)
//! - When -margin < g < 0 (approaching limit): precision = kappa^-1 * g^-2
//! - When g >= 0 (violation): precision = large fixed value (hard clamp)
//!
//! The adaptive precision means this factor barely affects the solution when velocity
//! is well under the limit, but becomes overwhelmingly strong as velocity approaches
//! v_max. Unlike a simple Gaussian penalty which can always be "outbid" by a stronger
//! factor, the barrier precision grows quadratically toward infinity.
//!
//! This is a pragmatic compromise: one iteration of BIPM without the outer loop.
//! It won't find the exact constrained optimum, but it prevents the gross constraint
//! violations (10+ m/s predictions) that cause belief dots to scatter across the map.

use heapless::Vec;
use crate::factor_node::{Factor, LinearizedFactor};
use crate::variable_node::VariableNode;

/// Pairwise factor on (s_k, s_{k+1}) that enforces velocity <= v_max.
///
/// Uses adaptive precision inspired by BIPM barrier functions:
/// precision grows as velocity approaches v_max, creating a soft wall.
pub struct VelocityBoundFactor {
    var_indices: [usize; 2],
    dt: f32,
    v_max: f32,
    /// Fixed barrier parameter. Higher = sharper wall but less smooth.
    /// The full BIPM schedules this from 0.5 to 1500; we use a fixed value.
    kappa: f32,
    /// Activation margin: factor only activates when velocity > v_max - margin.
    /// Prevents unnecessary computation when well under the limit.
    margin: f32,
}

impl VelocityBoundFactor {
    pub fn new(var_indices: [usize; 2], dt: f32, v_max: f32) -> Self {
        Self {
            var_indices,
            dt: f32::max(dt, 1e-6),
            v_max,
            kappa: 10.0,
            margin: 1.0, // activate when within 1 m/s of v_max
        }
    }

    pub fn set_v_max(&mut self, v: f32) { self.v_max = v; }
}

impl Factor for VelocityBoundFactor {
    fn variable_indices(&self) -> &[usize] { &self.var_indices }

    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor {
        let s_k  = variables[self.var_indices[0]].mean();
        let s_k1 = variables[self.var_indices[1]].mean();
        let velocity = (s_k1 - s_k) / self.dt;

        // g = velocity - v_max
        // g < 0 means feasible (under limit), g >= 0 means violation
        let g = velocity - self.v_max;

        let inv_dt = 1.0 / self.dt;
        let mut jacobian: Vec<f32, 2> = Vec::new();
        let _ = jacobian.push(-inv_dt); // d(velocity)/d(s_k)   = -1/dt
        let _ = jacobian.push( inv_dt); // d(velocity)/d(s_k+1) =  1/dt

        // Adaptive precision from barrier function:
        //   - Well under limit (g < -margin): precision = 0 (inactive)
        //   - Approaching limit (-margin < g < 0): precision = 1/(kappa * g^2)
        //   - At or over limit (g >= 0): precision = large fixed cap
        let precision = if g < -self.margin {
            // Well under the limit — don't interfere with dynamics
            0.0
        } else if g < -1e-4 {
            // Approaching the limit — barrier precision grows as 1/g^2
            // This is the key BIPM insight: precision -> infinity as g -> 0
            let barrier_prec = 1.0 / (self.kappa * g * g);
            // Cap to prevent numerical issues (equivalent to kappa_final in full BIPM)
            barrier_prec.min(1000.0)
        } else {
            // At or over the limit — maximum penalty
            1000.0
        };

        // Residual = g (the constraint violation amount)
        // When g > 0: positive residual + high precision = strong push back
        // When g ~ 0: small residual + high precision = gentle wall
        LinearizedFactor {
            jacobian,
            residual: g,
            precision,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VariableNode;

    #[test]
    fn zero_precision_when_well_under_limit() {
        let f = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        // velocity = (1.0 - 0.9) / 0.1 = 1.0 m/s, well under 2.5
        let vars = [VariableNode::new(0.9, 1.0), VariableNode::new(1.0, 1.0)];
        let lf = f.linearize(&vars);
        assert_eq!(lf.precision, 0.0, "should be inactive when well under v_max");
    }

    #[test]
    fn increasing_precision_near_limit() {
        let f = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        // velocity = (1.22 - 1.0) / 0.1 = 2.2 m/s, 0.3 under limit (within margin)
        let vars_far = [VariableNode::new(1.0, 1.0), VariableNode::new(1.22, 1.0)];
        let lf_far = f.linearize(&vars_far);

        // velocity = (1.24 - 1.0) / 0.1 = 2.4 m/s, 0.1 under limit
        let vars_close = [VariableNode::new(1.0, 1.0), VariableNode::new(1.24, 1.0)];
        let lf_close = f.linearize(&vars_close);

        assert!(lf_far.precision > 0.0, "should be active within margin");
        assert!(lf_close.precision > lf_far.precision,
            "precision should increase closer to limit: close={} far={}",
            lf_close.precision, lf_far.precision);
    }

    #[test]
    fn max_precision_when_over_limit() {
        let f = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        // velocity = (1.3 - 1.0) / 0.1 = 3.0 m/s, OVER 2.5
        let vars = [VariableNode::new(1.0, 1.0), VariableNode::new(1.3, 1.0)];
        let lf = f.linearize(&vars);
        assert_eq!(lf.precision, 1000.0, "should be max penalty when over limit");
        assert!(lf.residual > 0.0, "residual should be positive (violation)");
    }

    #[test]
    fn jacobian_signs_correct() {
        let f = VelocityBoundFactor::new([0, 1], 0.1, 2.5);
        let vars = [VariableNode::new(1.0, 1.0), VariableNode::new(1.3, 1.0)];
        let lf = f.linearize(&vars);
        // d(velocity)/d(s_k) = -1/dt < 0
        assert!(lf.jacobian[0] < 0.0);
        // d(velocity)/d(s_{k+1}) = 1/dt > 0
        assert!(lf.jacobian[1] > 0.0);
    }

    #[test]
    fn gbp_reduces_velocity_with_bound_factor() {
        use crate::{FactorGraph, FactorKind, DynamicsFactor};

        // Small graph: 3 variables, dynamics want v_nom=5.0 but v_max=2.5
        let mut graph: FactorGraph<3, 4> = FactorGraph::new(0.0, 100.0);

        // Anchor variable 0
        graph.variables[0].prior_eta = 0.0;
        graph.variables[0].prior_lambda = 1000.0;
        graph.variables[0].eta = 0.0;
        graph.variables[0].lambda = 1000.0;

        // Dynamics factor wants v_nom=5.0 (way over v_max)
        graph.add_factor(FactorKind::Dynamics(
            DynamicsFactor::new([0, 1], 0.1, 0.5, 5.0)
        )).unwrap();
        graph.add_factor(FactorKind::Dynamics(
            DynamicsFactor::new([1, 2], 0.1, 0.5, 5.0)
        )).unwrap();

        // Velocity bound factor at v_max=2.5
        graph.add_factor(FactorKind::VelocityBound(
            VelocityBoundFactor::new([0, 1], 0.1, 2.5)
        )).unwrap();
        graph.add_factor(FactorKind::VelocityBound(
            VelocityBoundFactor::new([1, 2], 0.1, 2.5)
        )).unwrap();

        graph.iterate(15);

        let v01 = (graph.variables[1].mean() - graph.variables[0].mean()) / 0.1;
        let v12 = (graph.variables[2].mean() - graph.variables[1].mean()) / 0.1;

        // Without bound factor, velocity would converge near 5.0
        // With bound factor, velocity should be pulled down toward 2.5
        assert!(v01 < 3.5, "v01={:.2} should be well under 5.0 with barrier", v01);
        assert!(v12 < 3.5, "v12={:.2} should be well under 5.0 with barrier", v12);
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p gbp-core velocity_bound`
Expected: Compilation error — `VelocityBoundFactor` not in `FactorKind` yet.

- [ ] **Step 3: Register the factor (3 extension points)**

In `crates/gbp-core/src/factor_node.rs`, add to imports:
```rust
use crate::velocity_bound_factor::VelocityBoundFactor;
```

Add variant to `FactorKind`:
```rust
pub enum FactorKind {
    Dynamics(DynamicsFactor),
    InterRobot(InterRobotFactor),
    VelocityBound(VelocityBoundFactor),  // FACTOR EXTENSION POINT 1/3
}
```

Add match arms:
```rust
// in as_factor():
Self::VelocityBound(f) => f,  // FACTOR EXTENSION POINT 2/3

// in as_factor_mut():
Self::VelocityBound(f) => f,  // FACTOR EXTENSION POINT 2/3 (mut)
```

In `crates/gbp-core/src/lib.rs`:
```rust
pub mod velocity_bound_factor;  // FACTOR EXTENSION POINT 3/3
pub use velocity_bound_factor::VelocityBoundFactor;
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p gbp-core velocity_bound -- --nocapture`
Expected: All 5 tests PASS

- [ ] **Step 5: Commit**

```bash
git add crates/gbp-core/src/velocity_bound_factor.rs crates/gbp-core/src/factor_node.rs crates/gbp-core/src/lib.rs
git commit -m "feat: add VelocityBoundFactor — BIPM-inspired velocity bounding"
```

---

## Task 2: Split iterate() into internal/external passes

**Files:**
- Modify: `crates/gbp-core/src/factor_graph.rs`
- Modify: `crates/gbp-core/src/factor_node.rs`

- [ ] **Step 1: Add `is_internal()` to FactorKind**

In `crates/gbp-core/src/factor_node.rs`, add a method to `FactorKind`:

```rust
impl FactorKind {
    /// Whether this factor uses only local (internal) variables.
    /// Internal factors: Dynamics, VelocityBound (both connect variables we own).
    /// External factors: InterRobot (depends on external belief from another robot).
    pub fn is_internal(&self) -> bool {
        match self {
            Self::Dynamics(_) => true,
            Self::VelocityBound(_) => true,
            Self::InterRobot(_) => false,
        }
    }

    // ... existing as_factor(), as_factor_mut() ...
}
```

- [ ] **Step 2: Add `iterate_split()` to FactorGraph**

In `crates/gbp-core/src/factor_graph.rs`, add alongside existing `iterate()`:

```rust
/// Run GBP with separate internal/external iteration counts.
///
/// # Internal vs External iteration (from MAGICS architecture)
///
/// Internal factors (dynamics, velocity bound) connect variables we fully own.
/// Their messages update with fresh variable beliefs every iteration.
///
/// External factors (inter-robot) depend on beliefs from another robot's
/// broadcast, which arrive at a slower rate (e.g. 50 Hz agent tick vs
/// 15 internal iterations). Within a single step(), the external belief
/// is frozen. Running external factors every internal iteration amplifies
/// their influence: the same stale evidence gets re-accumulated N times.
///
/// The fix: run `n_internal` internal-only passes per 1 external pass.
/// This gives dynamics factors time to converge before the next round
/// of inter-robot influence. The original gbpplanner uses TI=10, TE=10
/// but with external updates gated by communication frequency.
pub fn iterate_split(&mut self, n_internal: usize, n_external: usize) {
    for _ in 0..n_external {
        // Internal sub-loop: dynamics + velocity bounds converge first
        for _ in 0..n_internal {
            self.factor_to_variable_pass_filtered(true);  // internal only
            self.variable_to_factor_pass();
        }
        // External pass: inter-robot factors inject one round of influence
        self.factor_to_variable_pass_filtered(false);  // external only
        self.variable_to_factor_pass();
    }
}

fn factor_to_variable_pass_filtered(&mut self, internal_only: bool) {
    for f in self.factors.iter_mut() {
        if !f.kind.as_factor().is_active() { continue; }
        // Filter: if internal_only=true, skip external factors; if false, skip internal
        if f.kind.is_internal() != internal_only { continue; }

        // Rest is identical to factor_to_variable_pass...
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
```

- [ ] **Step 3: Run existing tests to verify nothing broke**

Run: `cargo test -p gbp-core`
Expected: All existing tests PASS (iterate() unchanged, new method is additive)

- [ ] **Step 4: Commit**

```bash
git add crates/gbp-core/src/factor_graph.rs crates/gbp-core/src/factor_node.rs
git commit -m "feat: add iterate_split() for internal/external message passing separation"
```

---

## Task 3: Spawn velocity bound factors and use iterate_split in RobotAgent

**Files:**
- Modify: `crates/gbp-agent/src/robot_agent.rs`

- [ ] **Step 1: Update capacity constants**

Velocity bound factors: K-1 (one per consecutive pair), same count as dynamics.

```rust
const NUM_DYN_FACTORS: usize = MAX_HORIZON - 1;
const NUM_VEL_BOUND_FACTORS: usize = MAX_HORIZON - 1;
const MAX_IR_FACTORS: usize = MAX_HORIZON * MAX_NEIGHBOURS;
const MAX_FACTORS: usize = NUM_DYN_FACTORS + NUM_VEL_BOUND_FACTORS + MAX_IR_FACTORS;
```

Add iteration constants:
```rust
/// Internal iterations (dynamics + velocity bound) per external iteration (inter-robot).
/// Higher ratio = dynamics converge more before IR factors inject external evidence.
const GBP_INTERNAL_ITERS: usize = 5;
/// External iterations (inter-robot factors). Total GBP iterations = INTERNAL * EXTERNAL.
const GBP_EXTERNAL_ITERS: usize = 3;
```

- [ ] **Step 2: Add velocity bound factor indices and spawn them in `new()`**

Add field `vel_bound_indices: [usize; NUM_VEL_BOUND_FACTORS]` to `RobotAgent`.

In `new()`, after spawning dynamics factors:
```rust
// Add K-1 permanent velocity bound factors (barrier-inspired, see velocity_bound_factor.rs)
let mut vel_bound_indices = [0usize; NUM_VEL_BOUND_FACTORS];
for k in 0..NUM_VEL_BOUND_FACTORS {
    let idx = graph.add_factor(FactorKind::VelocityBound(
        VelocityBoundFactor::new([k, k + 1], DT, 2.5) // v_max updated per step
    )).expect("BUG: MAX_FACTORS too small for velocity bound factors");
    vel_bound_indices[k] = idx;
}
```

- [ ] **Step 3: Update v_max each step (alongside v_nom)**

In `update_dynamics_v_nom()`, also update the velocity bound factor's v_max from the current edge's speed profile:

```rust
fn update_dynamics_v_nom(&mut self, map: &Map) {
    let traj = match &self.trajectory { Some(t) => t, None => return };
    let edge = map.edges.iter().find(|e| e.id == self.current_edge);
    let (nominal, decel) = edge.map(|e| (e.speed.nominal, e.speed.decel_limit))
        .unwrap_or((1.0, 1.0));
    let max_speed = edge.map(|e| e.speed.max).unwrap_or(2.5);

    for (k, &dyn_idx) in self.dyn_indices.iter().enumerate() {
        let s_k_global = self.position_s + self.graph.variables[k].mean() - self.graph.variables[0].mean();
        let v_nom = traj.v_nom_at(s_k_global, nominal, decel);
        if let Some(FactorKind::Dynamics(df)) = self.graph.get_factor_kind_mut(dyn_idx) {
            df.set_v_nom(v_nom);
        }
    }

    // Update velocity bound factors with current edge's max speed
    for &vb_idx in self.vel_bound_indices.iter() {
        if let Some(FactorKind::VelocityBound(vbf)) = self.graph.get_factor_kind_mut(vb_idx) {
            vbf.set_v_max(max_speed);
        }
    }
}
```

- [ ] **Step 4: Switch from iterate() to iterate_split()**

In `step()`, replace:
```rust
self.graph.iterate(GBP_ITERATIONS);
```
with:
```rust
self.graph.iterate_split(GBP_INTERNAL_ITERS, GBP_EXTERNAL_ITERS);
```

Remove the now-unused `GBP_ITERATIONS` constant.

- [ ] **Step 5: Update imports**

```rust
use gbp_core::{FactorGraph, FactorKind, DynamicsFactor, InterRobotFactor, VelocityBoundFactor};
```

- [ ] **Step 6: Build and run all tests**

Run: `cargo build --workspace && cargo test --workspace --exclude visualiser`
Expected: All tests PASS, no compilation errors.

- [ ] **Step 7: Commit**

```bash
git add crates/gbp-agent/src/robot_agent.rs
git commit -m "feat: spawn velocity bound factors, use iterate_split for internal/external separation"
```

---

## Task 4: Run simulator scenarios and verify improvement

- [ ] **Step 1: Run endcollision scenario**

```bash
pkill -f "target/debug/simulator" 2>/dev/null; sleep 1
timeout 20 cargo run -p simulator -- --map maps/test_loop_map.yaml --scenario endcollision 2>&1 | tee /tmp/endcollision_post.log
```

Check the logs for:
- `gbp_v` should be much closer to `cmd_v` (no more 11+ m/s raw GBP velocity)
- `dist3d` should still converge to ~1.3m (d_safe still works)
- IR factors still spawn (`ir=11` lines present)

- [ ] **Step 2: Run merge scenario**

```bash
pkill -f "target/debug/simulator" 2>/dev/null; sleep 1
timeout 20 cargo run -p simulator -- --map maps/test_loop_map.yaml --scenario merge 2>&1 | tee /tmp/merge_post.log
```

Check:
- No pass-through (dist3d never goes to 0)
- `gbp_v` values reasonable (under ~4 m/s, not 10+)
- Robots still separate after merge

- [ ] **Step 3: Compare before/after**

Compare `gbp_v` values from `/tmp/endcollision_post.log` with the pre-fix logs.
The improvement should be visible: raw GBP velocity staying in physically feasible range.

- [ ] **Step 4: Commit any tuning adjustments**

If `kappa`, `margin`, or iteration counts need adjustment based on results:
```bash
git add -u
git commit -m "tune: adjust velocity bound factor parameters after scenario testing"
```
