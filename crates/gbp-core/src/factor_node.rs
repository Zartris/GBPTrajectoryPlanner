use heapless::Vec;
use crate::variable_node::VariableNode;
use crate::dynamics_factor::DynamicsFactor;
use crate::interrobot_factor::InterRobotFactor;
use crate::velocity_bound_factor::VelocityBoundFactor;

/// Result of linearizing a factor around current variable beliefs.
#[derive(Clone, Debug)]
pub struct LinearizedFactor {
    /// Jacobian row -- one f32 per connected variable.
    pub jacobian:  Vec<f32, 2>,
    /// Residual scalar.
    pub residual:  f32,
    /// Precision (1/sigma^2) for this factor.
    pub precision: f32,
}

/// Implement this trait for each factor type.
pub trait Factor {
    /// Variable indices this factor connects (into `FactorGraph.variables`).
    fn variable_indices(&self) -> &[usize];

    /// Linearize around current beliefs -> Jacobian + residual + precision.
    fn linearize(&self, variables: &[VariableNode]) -> LinearizedFactor;

    /// Called each GBP iteration -- override to re-evaluate geometry or cached state.
    /// Default: no-op.
    fn update(&mut self, _variables: &[VariableNode]) {}

    /// Whether this factor participates in this iteration.
    fn is_active(&self) -> bool { true }
}

/// The single extension enum. Adding a new factor type means:
///   1. Add a variant here           (FACTOR EXTENSION POINT 1/3)
///   2. Add match arms below         (FACTOR EXTENSION POINT 2/3)
///   3. Export the module in lib.rs  (FACTOR EXTENSION POINT 3/3)
pub enum FactorKind {
    Dynamics(DynamicsFactor),
    InterRobot(InterRobotFactor),
    VelocityBound(VelocityBoundFactor),
}

impl FactorKind {
    pub fn as_factor(&self) -> &dyn Factor {
        match self {
            // FACTOR EXTENSION POINT 2/3
            Self::Dynamics(f)       => f,
            Self::InterRobot(f)     => f,
            Self::VelocityBound(f)  => f,
        }
    }
    pub fn as_factor_mut(&mut self) -> &mut dyn Factor {
        match self {
            // FACTOR EXTENSION POINT 2/3 (mut)
            Self::Dynamics(f)       => f,
            Self::InterRobot(f)     => f,
            Self::VelocityBound(f)  => f,
        }
    }

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
}

/// Internal wrapper used by FactorGraph to store a factor alongside its outgoing messages.
/// Stores a factor alongside its outgoing messages.
/// Public so downstream crates can read stored messages (e.g. for cavity belief computation).
pub struct FactorNode {
    pub kind: FactorKind,
    /// Outgoing messages (factor -> variable), indexed by position in variable_indices().
    /// All factors in this system are pairwise or unary -- max 2.
    pub msg_eta:    [f32; 2],
    pub msg_lambda: [f32; 2],
}

impl FactorNode {
    pub fn new(kind: FactorKind) -> Self {
        Self { kind, msg_eta: [0.0; 2], msg_lambda: [0.0; 2] }
    }
}
