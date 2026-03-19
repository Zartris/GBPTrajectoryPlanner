# Adding a New Factor to GBPTrajectoryPlanner

This guide explains how to add a new factor type to the GBP inference engine.
The system is designed so that adding a factor requires changes in exactly
**three places**, all clearly marked with `// FACTOR EXTENSION POINT` comments
in the source.

---

## What is a factor?

A factor is a probabilistic constraint between one or more variable nodes in
the factor graph. During GBP message passing, each factor linearizes its
constraint around the current variable beliefs, computes a residual and
Jacobian, and sends information messages back to its connected variables.

All factors in `gbp-core` are **scalar-residual Gaussian factors** — the
constraint is expressed as a scalar function `e = h(x) - z` with Gaussian
noise model `N(0, σ²)`. The information this contributes to each connected
variable is derived from the Jacobian `∂e/∂x` evaluated at the current belief.

---

## Before you start — classify your factor

| Question | Answer | Implication |
|---|---|---|
| How many variables does it connect? | 1 = unary, 2 = pairwise, N = higher-order | Jacobian has N columns |
| Is it always active? | Yes = permanent, No = conditional | Lifecycle handling needed |
| Does it need external state? | Robot position, neighbour data, etc. | Passed at construction or via `update()` |
| Does it re-linearize every iteration? | Yes for nonlinear constraints | Override `update()` |

---

## Step 1 — Implement the `Factor` trait

Create a new file in `crates/gbp-core/src/`. The struct holds all state the
factor needs to compute its linearization.

```rust
// crates/gbp-core/src/my_factor.rs

use crate::{Factor, LinearizedFactor, VariableNode};
use heapless::Vec;

/// [One-line description of what constraint this factor encodes]
///
/// Connected variables: [list which variable indices and what they represent]
/// Residual: e = [your formula]
/// Jacobian: [∂e/∂x for each connected variable]
pub struct MyFactor {
    /// Index into FactorGraph.variables for each connected variable
    var_indices: Vec<usize, 2>,   // change capacity to match your variable count

    /// Noise model — lower σ = stronger constraint
    precision: f32,               // 1 / σ²

    // Add any other state your factor needs:
    // target_value: f32,
    // cached_jacobian: [f32; 2],
}

impl MyFactor {
    pub fn new(var_a: usize, var_b: usize, sigma: f32) -> Self {
        let mut var_indices = Vec::new();
        var_indices.push(var_a).unwrap();
        var_indices.push(var_b).unwrap();
        Self {
            var_indices,
            precision: 1.0 / (sigma * sigma),
        }
    }
}

impl Factor for MyFactor {
    fn variable_indices(&self) -> &[usize] {
        &self.var_indices
    }

    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor {
        let x_a = variables[self.var_indices[0]].mean;
        let x_b = variables[self.var_indices[1]].mean;

        // Compute residual: how much the constraint is violated at current beliefs
        let residual = x_a - x_b; // replace with your e = h(x) - z

        // Compute Jacobian: ∂e/∂x for each connected variable
        // For linear factors this is constant. For nonlinear, evaluate at current mean.
        let mut jacobian = Vec::new();
        jacobian.push(1.0).unwrap();   // ∂e/∂x_a
        jacobian.push(-1.0).unwrap();  // ∂e/∂x_b

        LinearizedFactor {
            jacobian,
            residual,
            precision: self.precision,
        }
    }

    // Optional: override update() if you need to re-evaluate state each iteration
    // (e.g. recompute a Jacobian that depends on external geometry)
    fn update(&mut self, variables: &[VariableNode]) {
        // Default implementation does nothing — override only if needed
    }

    // Optional: override is_active() if the factor can be conditionally disabled
    // without removing it from the graph (avoids index invalidation)
    fn is_active(&self) -> bool {
        true  // always active — change to conditional logic if needed
    }
}
```

---

## Step 2 — Add a variant to `FactorKind`

Open `crates/gbp-core/src/factor_node.rs` and find the section marked
`// FACTOR EXTENSION POINT`. Add your variant:

```rust
// crates/gbp-core/src/factor_node.rs

// FACTOR EXTENSION POINT — add new factor variants here
pub enum FactorKind {
    Dynamics(DynamicsFactor),
    InterRobot(InterRobotFactor),
    MyFactor(MyFactor),           // ← add this line
}

impl FactorKind {
    pub fn as_factor(&self) -> &dyn Factor {
        match self {
            Self::Dynamics(f)   => f,
            Self::InterRobot(f) => f,
            Self::MyFactor(f)   => f,   // ← add this arm
        }
    }

    pub fn as_factor_mut(&mut self) -> &mut dyn Factor {
        match self {
            Self::Dynamics(f)   => f,
            Self::InterRobot(f) => f,
            Self::MyFactor(f)   => f,   // ← add this arm
        }
    }
}
```

---

## Step 3 — Export from `lib.rs`

Open `crates/gbp-core/src/lib.rs` and add your module:

```rust
// crates/gbp-core/src/lib.rs

// FACTOR EXTENSION POINT — export new factor modules here
pub mod dynamics_factor;
pub mod interrobot_factor;
pub mod my_factor;              // ← add this line

pub use my_factor::MyFactor;   // ← and this line
```

---

## Step 4 — Spawn the factor from the agent layer

The `gbp-core` crate only does inference. The decision of *when* to add or
remove a factor belongs in `gbp-agent`. Open
`crates/gbp-agent/src/robot_agent.rs` and add the spawn logic:

```rust
// crates/gbp-agent/src/robot_agent.rs

use gbp_core::{FactorGraph, FactorKind, MyFactor};

impl<C: CommsInterface, const K: usize, const F: usize> RobotAgent<C, K, F> {

    fn maybe_add_my_factor(&mut self) {
        // Decide whether to add the factor based on agent-level state
        if self.some_condition() && !self.my_factor_active {
            let factor = MyFactor::new(
                0,     // variable index A (e.g. current timestep)
                1,     // variable index B (e.g. next timestep)
                0.5,   // σ — tune this
            );
            match self.graph.add_factor(FactorKind::MyFactor(factor)) {
                Ok(idx) => self.my_factor_idx = Some(idx),
                Err(_)  => defmt::warn!("Factor capacity exceeded"),
            }
            self.my_factor_active = true;
        }
    }

    fn maybe_remove_my_factor(&mut self) {
        if !self.some_condition() {
            if let Some(idx) = self.my_factor_idx.take() {
                self.graph.remove_factor(idx);
                self.my_factor_active = false;
            }
        }
    }
}
```

Call `maybe_add_my_factor()` and `maybe_remove_my_factor()` from the
`step()` method at the appropriate point in the agent loop.

---

## Permanent vs. conditional factors

**Permanent factors** (like `DynamicsFactor`) are added once at graph
construction and never removed. Simply add them in `FactorGraph::new()` or
`RobotAgent::new()`.

**Conditional factors** (like `InterRobotFactor`) are added and removed based
on runtime conditions. Use the `add_factor` / `remove_factor` API. Note that
`remove_factor` uses swap-remove — if you store an index to a factor, that
index may become invalid after any removal. The agent layer is responsible for
tracking indices correctly (see `InterRobotFactorSet` in `robot_agent.rs` for
the reference implementation).

**Deactivatable factors** can be toggled on/off without index churn. Override
`is_active()` and set an internal flag. The graph skips inactive factors during
message passing but keeps them in the slot array. Use this when a factor
toggles frequently and swap-remove index invalidation would be costly.

---

## Testing your factor

Add a unit test in `crates/gbp-core/src/my_factor.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::VariableNode;

    #[test]
    fn test_linearization_at_zero_residual() {
        // When constraint is satisfied, residual should be zero
        let factor = MyFactor::new(0, 1, 0.5);
        let vars = [
            VariableNode::with_mean(1.0),
            VariableNode::with_mean(1.0),  // x_a == x_b → residual = 0
        ];
        let lin = factor.linearize(&vars);
        assert!((lin.residual).abs() < 1e-6);
    }

    #[test]
    fn test_jacobian_sign() {
        // Verify Jacobian sign convention
        let factor = MyFactor::new(0, 1, 0.5);
        let vars = [
            VariableNode::with_mean(2.0),
            VariableNode::with_mean(1.0),  // x_a > x_b → positive residual
        ];
        let lin = factor.linearize(&vars);
        assert!(lin.residual > 0.0);
        assert!(lin.jacobian[0] > 0.0);  // ∂e/∂x_a = +1
        assert!(lin.jacobian[1] < 0.0);  // ∂e/∂x_b = -1
    }
}
```

Run with `cargo test -p gbp-core`.

---

## Checklist

- [ ] New file `crates/gbp-core/src/my_factor.rs` with `Factor` impl
- [ ] Variant added to `FactorKind` enum in `factor_node.rs`
- [ ] Match arms added to `as_factor()` and `as_factor_mut()` in `FactorKind`
- [ ] Module exported from `lib.rs`
- [ ] Spawn/despawn logic added to `robot_agent.rs`
- [ ] Unit tests pass: `cargo test -p gbp-core`
- [ ] No `std` or `alloc` imports added to `gbp-core`

---

## Worked example — the `DynamicsFactor`

The dynamics factor is the simplest real factor in the system and a good
reference for pairwise factors. See
`crates/gbp-core/src/dynamics_factor.rs`.

It connects two consecutive variable nodes `(s_k, s_{k+1})` and encodes the
constraint that the velocity `(s_{k+1} - s_k) / Δt` should equal `v_nom(s_k)`,
the target velocity at the current arc-length position.

The residual:

```
e = (s_{k+1} - s_k) / Δt - v_nom(s_k)
```

The Jacobian:

```
∂e/∂s_k     = -1/Δt   (and a correction term if v_nom varies with s_k)
∂e/∂s_{k+1} = +1/Δt
```

The `v_nom(s_k)` function returns the trapezoidal velocity profile value at
arc-length `s_k` — full nominal speed in the middle of an edge, tapering to
zero near the goal node using the edge's `decel_limit`.
