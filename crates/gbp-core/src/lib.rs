#![no_std]

pub mod variable_node;
pub mod factor_node;
pub mod factor_graph;
pub mod dynamics_factor;
pub mod interrobot_factor;  // FACTOR EXTENSION POINT 3/3: export new modules here
pub mod velocity_bound_factor;

pub use variable_node::VariableNode;
pub use factor_node::{Factor, FactorKind, FactorNode, LinearizedFactor};
pub use factor_graph::FactorGraph;
pub use dynamics_factor::DynamicsFactor;
pub use interrobot_factor::InterRobotFactor;
pub use velocity_bound_factor::VelocityBoundFactor;
