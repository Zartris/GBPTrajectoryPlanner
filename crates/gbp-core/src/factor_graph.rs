// Stub - to be implemented in Task 10
use heapless::Vec;
use crate::factor_node::{FactorKind, FactorNode};
use crate::variable_node::VariableNode;

/// GBP factor graph with const-generic capacity.
/// K = number of variables (timestep horizon), F = max factors.
pub struct FactorGraph<const K: usize, const F: usize> {
    pub variables: [VariableNode; K],
    factors: Vec<FactorNode, F>,
}
